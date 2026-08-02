#!/usr/bin/env python3
"""Plot localization diagnostics from completed ROS 2 system-simulation runs.

The figure combines the evaluator's cumulative ATE summaries with the recorded
``map -> odom`` transform.  It is descriptive evidence: the largest adjacent
transform correction is reported directly, without introducing a post-hoc
pass/fail threshold.

Runtime dependencies are ROS 2 Jazzy's ``rosbag2_py`` / CDR message support,
the MCAP storage plugin, NumPy, Matplotlib, and PyYAML.  Source ROS and the
workspace setup files before invoking this script.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

try:
    from scripts.render_system_sim_bag_media import load_run_context
except ModuleNotFoundError:  # Direct ``python scripts/...py`` invocation.
    from render_system_sim_bag_media import load_run_context


OUTPUT_SCHEMA = "sstg_system_sim_localization_diagnostic/v1"
MANIFEST_SCHEMA = "sstg_system_sim_localization_diagnostic_manifest/v1"
FIGURE_NAME = "localization_diagnostic.png"
REPORT_NAME = "localization_diagnostic.json"
MANIFEST_NAME = "localization_diagnostic_manifest.json"


class LocalizationDiagnosticError(ValueError):
    """Raised when localization evidence is missing, unsafe, or ambiguous."""


@dataclass(frozen=True)
class AteSample:
    ros_time_s: float
    map_revision: int
    sample_count: int
    mean_m: float
    rmse_m: float
    maximum_m: float


@dataclass(frozen=True)
class TransformSample:
    header_time_s: float
    bag_time_s: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class TransformCorrection:
    before_index: int
    after_index: int
    header_time_s: float
    bag_time_s: float
    translation_m: float
    yaw_rad: float
    before_x_m: float
    before_y_m: float
    after_x_m: float
    after_y_m: float


@dataclass(frozen=True)
class RunDiagnostic:
    label: str
    run_dir: str
    study_id: str
    schedule_id: str
    method: str
    execution_status: str
    artifact_valid: bool
    ate_samples: tuple[AteSample, ...]
    transform_samples: tuple[TransformSample, ...]
    largest_transform_correction: TransformCorrection


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _finite_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise LocalizationDiagnosticError(f"{label} must be numeric") from error
    if not math.isfinite(result):
        raise LocalizationDiagnosticError(f"{label} must be finite")
    return result


def _mapping(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise LocalizationDiagnosticError(f"{label} is missing or a symlink: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise LocalizationDiagnosticError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise LocalizationDiagnosticError(f"{label} must be a mapping: {path}")
    return value


def _artifact_valid(manifest: Mapping[str, Any]) -> bool:
    execution = manifest.get("execution")
    if not isinstance(execution, Mapping):
        return False
    audit = execution.get("artifact_audit")
    return isinstance(audit, Mapping) and audit.get("valid") is True


def _inside_root(root: Path, value: Path, label: str) -> Path:
    candidate = (
        value.expanduser().resolve()
        if value.is_absolute()
        else (root / value).expanduser().resolve()
    )
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise LocalizationDiagnosticError(
            f"{label} escapes repository root: {value}"
        ) from error
    return candidate


def _repo_path(root: Path, value: Path | str) -> str:
    return Path(value).resolve().relative_to(root).as_posix()


def read_ate_samples(path: Path) -> tuple[AteSample, ...]:
    """Read cumulative ATE summaries without deriving a new acceptance gate."""
    if path.is_symlink() or not path.is_file():
        raise LocalizationDiagnosticError(
            f"evaluation metrics are missing or a symlink: {path}"
        )
    samples: list[AteSample] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise LocalizationDiagnosticError(f"cannot read metrics: {path}: {error}") from error
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise LocalizationDiagnosticError(
                f"metrics line {line_number} is invalid JSON: {error}"
            ) from error
        if not isinstance(record, Mapping) or record.get("event") != "metrics_snapshot":
            continue
        payload = record.get("payload")
        motion = payload.get("ground_truth_motion") if isinstance(payload, Mapping) else None
        if not isinstance(motion, Mapping) or motion.get("ate_status") != "available":
            continue
        required = ("ate_mean_m", "ate_rmse_m", "ate_max_m")
        if any(motion.get(field) is None for field in required):
            continue
        ros_time_ns = record.get("ros_time_ns")
        map_revision = record.get("map_revision")
        sample_count = motion.get("ate_sample_count")
        if type(ros_time_ns) is not int or ros_time_ns < 0:
            raise LocalizationDiagnosticError(
                f"metrics line {line_number} has invalid ros_time_ns"
            )
        if type(map_revision) is not int or map_revision < 0:
            raise LocalizationDiagnosticError(
                f"metrics line {line_number} has invalid map_revision"
            )
        if type(sample_count) is not int or sample_count <= 0:
            raise LocalizationDiagnosticError(
                f"metrics line {line_number} has invalid ATE sample count"
            )
        sample = AteSample(
            ros_time_s=ros_time_ns / 1e9,
            map_revision=map_revision,
            sample_count=sample_count,
            mean_m=_finite_float(motion["ate_mean_m"], "ATE mean"),
            rmse_m=_finite_float(motion["ate_rmse_m"], "ATE RMSE"),
            maximum_m=_finite_float(motion["ate_max_m"], "ATE maximum"),
        )
        if samples and sample.ros_time_s < samples[-1].ros_time_s:
            raise LocalizationDiagnosticError("ATE snapshots are not time ordered")
        samples.append(sample)
    if not samples:
        raise LocalizationDiagnosticError(f"no available ATE snapshots: {path}")
    return tuple(samples)


def collapse_transform_samples(
    samples: Sequence[TransformSample], *, tolerance: float = 1e-12
) -> tuple[TransformSample, ...]:
    """Keep transform changes and endpoints while preserving update order."""
    if not samples:
        raise LocalizationDiagnosticError("map -> odom transform has no samples")
    collapsed = [samples[0]]
    for sample in samples[1:]:
        previous = collapsed[-1]
        changed = any(
            abs(current - old) > tolerance
            for current, old in (
                (sample.x_m, previous.x_m),
                (sample.y_m, previous.y_m),
                (sample.yaw_rad, previous.yaw_rad),
            )
        )
        if changed:
            collapsed.append(sample)
    if samples[-1] != collapsed[-1]:
        collapsed.append(samples[-1])
    return tuple(collapsed)


def read_map_to_odom(path: Path) -> tuple[TransformSample, ...]:
    """Deserialize only ``/tf`` and retain recorded ``map -> odom`` updates."""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import (
            ConverterOptions,
            SequentialReader,
            StorageFilter,
            StorageOptions,
        )
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise LocalizationDiagnosticError(
            "ROS bag runtime unavailable; source /opt/ros/jazzy/setup.bash and "
            "the workspace setup before running this diagnostic"
        ) from error

    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=str(path), storage_id="mcap"),
            ConverterOptions("cdr", "cdr"),
        )
    except Exception as error:
        raise LocalizationDiagnosticError(f"cannot open core MCAP bag: {error}") from error
    topic_types = {
        metadata.name: metadata.type for metadata in reader.get_all_topics_and_types()
    }
    if topic_types.get("/tf") != "tf2_msgs/msg/TFMessage":
        raise LocalizationDiagnosticError("core bag lacks typed /tf evidence")
    message_type = get_message(topic_types["/tf"])
    reader.set_filter(StorageFilter(topics=["/tf"]))

    raw: list[TransformSample] = []
    while reader.has_next():
        try:
            _, encoded, bag_timestamp_ns = reader.read_next()
            message = deserialize_message(encoded, message_type)
        except Exception as error:
            raise LocalizationDiagnosticError(
                f"cannot decode /tf near bag timestamp {locals().get('bag_timestamp_ns')}: {error}"
            ) from error
        for transform in message.transforms:
            parent = str(transform.header.frame_id).strip().lstrip("/")
            child = str(transform.child_frame_id).strip().lstrip("/")
            if (parent, child) != ("map", "odom"):
                continue
            translation = transform.transform.translation
            quaternion = transform.transform.rotation
            norm = math.sqrt(
                float(quaternion.x) ** 2
                + float(quaternion.y) ** 2
                + float(quaternion.z) ** 2
                + float(quaternion.w) ** 2
            )
            if not math.isfinite(norm) or norm <= 1e-12:
                raise LocalizationDiagnosticError("map -> odom quaternion is invalid")
            x = float(quaternion.x) / norm
            y = float(quaternion.y) / norm
            z = float(quaternion.z) / norm
            w = float(quaternion.w) / norm
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            stamp = transform.header.stamp
            raw.append(
                TransformSample(
                    header_time_s=int(stamp.sec) + int(stamp.nanosec) / 1e9,
                    bag_time_s=int(bag_timestamp_ns) / 1e9,
                    x_m=_finite_float(translation.x, "map -> odom x"),
                    y_m=_finite_float(translation.y, "map -> odom y"),
                    yaw_rad=_finite_float(yaw, "map -> odom yaw"),
                )
            )
    return collapse_transform_samples(raw)


def largest_transform_correction(
    samples: Sequence[TransformSample],
) -> TransformCorrection:
    if len(samples) < 2:
        raise LocalizationDiagnosticError(
            "map -> odom transform needs at least two distinct samples"
        )
    best: TransformCorrection | None = None
    for index, (before, after) in enumerate(zip(samples, samples[1:])):
        translation = math.hypot(after.x_m - before.x_m, after.y_m - before.y_m)
        yaw = math.atan2(
            math.sin(after.yaw_rad - before.yaw_rad),
            math.cos(after.yaw_rad - before.yaw_rad),
        )
        correction = TransformCorrection(
            before_index=index,
            after_index=index + 1,
            header_time_s=after.header_time_s,
            bag_time_s=after.bag_time_s,
            translation_m=translation,
            yaw_rad=yaw,
            before_x_m=before.x_m,
            before_y_m=before.y_m,
            after_x_m=after.x_m,
            after_y_m=after.y_m,
        )
        if best is None or correction.translation_m > best.translation_m:
            best = correction
    assert best is not None
    return best


def load_run(label: str, run_dir: Path) -> RunDiagnostic:
    if not label or "=" in label:
        raise LocalizationDiagnosticError(f"invalid run label: {label!r}")
    context = load_run_context(run_dir)
    manifest = _mapping(context.run_dir / "run_launch_manifest.yaml", "run manifest")
    if context.execution_status != "terminal_completed":
        raise LocalizationDiagnosticError(
            f"run {label} is not terminal_completed: {context.execution_status}"
        )
    if not _artifact_valid(manifest):
        raise LocalizationDiagnosticError(f"run {label} has no valid artifact audit")
    ate = read_ate_samples(context.run_dir / "evaluation_metrics.jsonl")
    transforms = read_map_to_odom(context.bag_dir)
    return RunDiagnostic(
        label=label,
        run_dir=str(context.run_dir),
        study_id=context.study_id,
        schedule_id=context.schedule_id,
        method=str(context.identity.get("method", "unknown")),
        execution_status=context.execution_status,
        artifact_valid=True,
        ate_samples=ate,
        transform_samples=transforms,
        largest_transform_correction=largest_transform_correction(transforms),
    )


def _safe_output(root: Path, output_dir: Path) -> Path:
    root = root.resolve()
    candidate = _inside_root(root, output_dir, "output directory")
    current = root
    for part in candidate.relative_to(root).parts:
        current /= part
        if current.is_symlink():
            raise LocalizationDiagnosticError(f"output path traverses symlink: {current}")
    candidate.mkdir(parents=True, exist_ok=True)
    for name in (FIGURE_NAME, REPORT_NAME, MANIFEST_NAME):
        if os.path.lexists(candidate / name):
            raise LocalizationDiagnosticError(f"refusing existing output: {candidate / name}")
    return candidate


def _render_figure(path: Path, runs: Sequence[RunDiagnostic]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = ("#2563eb", "#dc2626", "#059669", "#7c3aed")
    figure, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    for index, run in enumerate(runs):
        color = colors[index % len(colors)]
        ate_time = np.asarray([sample.ros_time_s for sample in run.ate_samples])
        axes[0, 0].plot(
            ate_time,
            [sample.rmse_m for sample in run.ate_samples],
            label=run.label,
            color=color,
            linewidth=2.0,
        )
        axes[0, 1].plot(
            ate_time,
            [sample.maximum_m for sample in run.ate_samples],
            label=run.label,
            color=color,
            linewidth=2.0,
        )
        transform_time = np.asarray(
            [sample.header_time_s for sample in run.transform_samples]
        )
        axes[1, 0].plot(
            transform_time,
            [sample.x_m for sample in run.transform_samples],
            label=run.label,
            color=color,
            linewidth=1.8,
            marker=".",
            markersize=3,
        )
        axes[1, 1].plot(
            transform_time,
            [sample.y_m for sample in run.transform_samples],
            label=run.label,
            color=color,
            linewidth=1.8,
            marker=".",
            markersize=3,
        )
        correction = run.largest_transform_correction
        axes[0, 0].axvline(
            correction.header_time_s, color=color, alpha=0.28, linestyle=":"
        )
        axes[1, 1].annotate(
            f"{run.label}: max adjacent correction\n"
            f"{correction.translation_m:.3f} m at {correction.header_time_s:.1f} s",
            xy=(correction.header_time_s, correction.after_y_m),
            xytext=(8, 12 if index % 2 == 0 else -40),
            textcoords="offset points",
            color=color,
            fontsize=9,
            arrowprops={"arrowstyle": "->", "color": color, "alpha": 0.7},
        )

    panels = (
        (axes[0, 0], "Cumulative ATE RMSE", "ATE RMSE [m]"),
        (axes[0, 1], "Cumulative maximum ATE", "Maximum ATE [m]"),
        (axes[1, 0], "Recorded map → odom correction: x", "Translation x [m]"),
        (axes[1, 1], "Recorded map → odom correction: y", "Translation y [m]"),
    )
    for axis, title, ylabel in panels:
        axis.set_title(title, fontweight="bold")
        axis.set_xlabel("ROS time [s]")
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    figure.suptitle(
        "Paired Gazebo localization diagnostic — descriptive development evidence",
        fontsize=16,
        fontweight="bold",
    )
    figure.savefig(
        path,
        dpi=180,
        facecolor="white",
        metadata={
            "Title": "Paired Gazebo localization diagnostic",
            "Description": (
                "Descriptive development evidence; no post-hoc localization threshold"
            ),
        },
    )
    plt.close(figure)


def generate_diagnostic(
    *, root: Path, run_specs: Sequence[tuple[str, Path]], output_dir: Path
) -> dict[str, Any]:
    if len(run_specs) < 1:
        raise LocalizationDiagnosticError("at least one --run LABEL=PATH is required")
    labels = [label for label, _ in run_specs]
    if len(labels) != len(set(labels)):
        raise LocalizationDiagnosticError("run labels must be unique")
    root = root.resolve()
    runs = tuple(
        load_run(label, _inside_root(root, path, f"run {label}"))
        for label, path in run_specs
    )
    studies = {run.study_id for run in runs}
    if len(studies) != 1:
        raise LocalizationDiagnosticError("diagnostic runs must belong to one study")
    output = _safe_output(root, output_dir)
    figure_path = output / FIGURE_NAME
    report_path = output / REPORT_NAME
    manifest_path = output / MANIFEST_NAME
    _render_figure(figure_path, runs)
    report = {
        "schema": OUTPUT_SCHEMA,
        "evidence_tier": "development_simulation",
        "formal_result_eligible": False,
        "interpretation": (
            "descriptive localization evidence with no post-hoc pass/fail threshold"
        ),
        "study_id": runs[0].study_id,
        "runs": [
            {
                "label": run.label,
                "run_dir": _repo_path(root, run.run_dir),
                "schedule_id": run.schedule_id,
                "method": run.method,
                "execution_status": run.execution_status,
                "artifact_valid": run.artifact_valid,
                "ate_final": asdict(run.ate_samples[-1]),
                "map_to_odom_retained_sample_count": len(run.transform_samples),
                "largest_adjacent_map_to_odom_correction": asdict(
                    run.largest_transform_correction
                ),
            }
            for run in runs
        ],
    }
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "study_id": runs[0].study_id,
        "inputs": {
            run.label: {
                "run_dir": _repo_path(root, run.run_dir),
                "evaluation_metrics_sha256": sha256_file(
                    Path(run.run_dir) / "evaluation_metrics.jsonl"
                ),
                "core_mcap": [
                    {
                        "path": _repo_path(root, path),
                        "sha256": sha256_file(path),
                        "size_bytes": path.stat().st_size,
                    }
                    for path in sorted((Path(run.run_dir) / "bags/core").glob("*.mcap"))
                ],
            }
            for run in runs
        },
        "outputs": {
            path.name: {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
            for path in (figure_path, report_path)
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return {
        "output_dir": str(output),
        "report": str(report_path),
        "figure": str(figure_path),
        "manifest": str(manifest_path),
    }


def _run_spec(value: str) -> tuple[str, Path]:
    label, separator, path = value.partition("=")
    if not separator or not label.strip() or not path.strip():
        raise argparse.ArgumentTypeError("run must be LABEL=PATH")
    return label.strip(), Path(path.strip())


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        type=_run_spec,
        metavar="LABEL=PATH",
        help="completed run to include; may be supplied more than once",
    )
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1]
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = generate_diagnostic(
            root=args.root,
            run_specs=args.run,
            output_dir=args.output_dir,
        )
    except (OSError, LocalizationDiagnosticError, ValueError) as error:
        print(f"localization diagnostic failed: {error}")
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
