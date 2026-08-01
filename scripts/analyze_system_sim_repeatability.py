#!/usr/bin/env python3
"""Compare two same-seed system-simulation runs without post-hoc thresholds."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

import yaml


SCHEMA = "sstg_system_sim_repeatability_report/v1"
MANIFEST_SCHEMA = "sstg_system_sim_repeatability_manifest/v1"
RUN_SCHEMA = "sstg_system_sim_run_launch/v1"
FREEZE_SCHEMA = "sstg_system_sim_schedule_freeze/v2"
ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
METRIC_FIELDS = (
    "information_coverage",
    "topological_coverage",
    "joint_coverage",
    "target_recall_proxy",
    "ground_truth_travel_m",
    "ate_rmse_m",
)


class RepeatabilityError(ValueError):
    """Raised when a repeatability comparison is invalid or ambiguous."""


@dataclass(frozen=True)
class RunEvidence:
    label: str
    run_dir: str
    study_id: str
    schedule_id: str
    world_id: str
    start_id: str
    method: str
    condition: str
    replicate_seed: int
    repository_commit: str
    source_tree_sha256: str
    input_fingerprint: str
    trace_sha256: str
    first_decision_ros_time_s: float
    first_map_revision: int
    first_known_free_cells: int
    first_execution_key: list[Any]
    first_target_x_m: float
    first_target_y_m: float
    decision_count: int
    navigation_goal_count: int
    navigation_success_count: int
    collision_count: int
    information_coverage: float
    topological_coverage: float
    joint_coverage: float
    target_recall_proxy: float
    ground_truth_travel_m: float
    ate_rmse_m: float


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _mapping(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise RepeatabilityError(f"{label} is missing or is a symlink: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RepeatabilityError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise RepeatabilityError(f"{label} must be a mapping: {path}")
    return value


def _jsonl(path: Path, label: str) -> list[dict[str, Any]]:
    if path.is_symlink() or not path.is_file():
        raise RepeatabilityError(f"{label} is missing or is a symlink: {path}")
    records: list[dict[str, Any]] = []
    try:
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise RepeatabilityError(f"{label} line {number} is not an object")
            records.append(value)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RepeatabilityError(f"cannot parse {label}: {path}: {error}") from error
    if not records:
        raise RepeatabilityError(f"{label} contains no records: {path}")
    return records


def _inside(root: Path, value: str, label: str) -> Path:
    supplied = Path(value)
    candidate = supplied.resolve() if supplied.is_absolute() else (root / supplied).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError as error:
        raise RepeatabilityError(f"{label} escapes repository root: {value}") from error
    return candidate


def _number(value: Any, label: str, *, integer: bool = False) -> float | int:
    if isinstance(value, bool):
        raise RepeatabilityError(f"{label} must be numeric")
    if integer:
        if type(value) is int:
            return value
        if isinstance(value, str) and re.fullmatch(r"-?(0|[1-9][0-9]*)", value):
            return int(value)
        else:
            raise RepeatabilityError(f"{label} must be an integer")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RepeatabilityError(f"{label} must be numeric") from error
    if not math.isfinite(result):
        raise RepeatabilityError(f"{label} must be finite")
    return result


def _nested(value: Mapping[str, Any], *path: str) -> Any:
    current: Any = value
    for key in path:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return current


def _verify_artifacts(run_dir: Path, manifest: Mapping[str, Any]) -> None:
    execution = manifest.get("execution")
    if not isinstance(execution, Mapping):
        raise RepeatabilityError("run manifest lacks execution evidence")
    if execution.get("status") != "terminal_completed":
        raise RepeatabilityError("repeatability input must be terminal_completed")
    audit = execution.get("artifact_audit")
    if not isinstance(audit, Mapping) or audit.get("valid") is not True:
        raise RepeatabilityError("repeatability input has no valid artifact audit")
    files = audit.get("files")
    if not isinstance(files, Mapping):
        raise RepeatabilityError("repeatability input has no artifact hashes")
    required = {"policy_trace.jsonl", "evaluation_metrics.jsonl", "launch.log"}
    if not required.issubset(files):
        raise RepeatabilityError("repeatability input lacks required artifact hashes")
    for relative, record in files.items():
        path = Path(str(relative))
        if path.is_absolute() or ".." in path.parts:
            raise RepeatabilityError(f"unsafe artifact path: {relative}")
        artifact = run_dir / path
        if artifact.is_symlink() or not artifact.is_file():
            raise RepeatabilityError(f"declared artifact is missing: {relative}")
        if not isinstance(record, Mapping):
            raise RepeatabilityError(f"artifact hash record is invalid: {relative}")
        expected = str(record.get("sha256", ""))
        if len(expected) != 64 or sha256_file(artifact) != expected:
            raise RepeatabilityError(f"artifact hash mismatch: {relative}")


def _input_fingerprint(freeze: Mapping[str, Any], identity: Mapping[str, Any]) -> str:
    inputs = freeze.get("inputs")
    source = freeze.get("source")
    if not isinstance(inputs, Mapping) or not isinstance(source, Mapping):
        raise RepeatabilityError("freeze manifest lacks source or input provenance")
    methods = inputs.get("methods")
    worlds = inputs.get("worlds")
    if not isinstance(methods, list) or not isinstance(worlds, list):
        raise RepeatabilityError("freeze manifest lacks method or world provenance")
    method = next(
        (
            item
            for item in methods
            if isinstance(item, Mapping)
            and item.get("method") == identity.get("method")
        ),
        None,
    )
    world = next(
        (
            item
            for item in worlds
            if isinstance(item, Mapping)
            and item.get("world_id") == identity.get("world_id")
        ),
        None,
    )
    if not isinstance(method, Mapping) or not isinstance(world, Mapping):
        raise RepeatabilityError("freeze inputs disagree with run identity")
    shared = inputs.get("shared_stack")
    condition = inputs.get("condition")
    registry = inputs.get("world_registry")
    if not all(isinstance(item, Mapping) for item in (shared, condition, registry)):
        raise RepeatabilityError("freeze manifest input records are incomplete")
    payload = {
        "repository_commit": source.get("repository_commit"),
        "repository_dirty": source.get("repository_dirty"),
        "source_tree_sha256": source.get("source_tree_sha256"),
        "shared_stack_sha256": shared.get("sha256"),
        "condition_sha256": condition.get("sha256"),
        "method_sha256": method.get("sha256"),
        "world_bundle_sha256": _nested(world, "sha256", "bundle"),
        "world_registry_sha256": registry.get("sha256"),
        "experiment_budget": freeze.get("experiment_budget"),
    }
    if payload["repository_dirty"] is not False:
        raise RepeatabilityError("repeatability input source must be clean")
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def load_run_evidence(root: Path, run_dir: Path | str, *, label: str) -> RunEvidence:
    root = root.resolve()
    run = _inside(root, str(run_dir), f"run {label}")
    if run.is_symlink() or not run.is_dir():
        raise RepeatabilityError(f"run {label} is missing or is a symlink")
    manifest = _mapping(run / "run_launch_manifest.yaml", f"run {label} manifest")
    if manifest.get("schema") != RUN_SCHEMA:
        raise RepeatabilityError(f"run {label} has unsupported schema")
    _verify_artifacts(run, manifest)

    identity = manifest.get("identity")
    launch = manifest.get("launch")
    arguments = launch.get("arguments") if isinstance(launch, Mapping) else None
    if not isinstance(identity, Mapping) or not isinstance(arguments, Mapping):
        raise RepeatabilityError(f"run {label} lacks identity or launch arguments")
    seed = _number(identity.get("replicate_seed"), "replicate seed", integer=True)
    if str(arguments.get("policy_seed")) != str(seed):
        raise RepeatabilityError(f"run {label} policy seed is not attested")
    if str(arguments.get("simulation_seed")) != str(seed):
        raise RepeatabilityError(f"run {label} simulation seed is not attested")

    schedule_dir = _inside(
        root, str(manifest.get("schedule_dir", "")), f"run {label} schedule"
    )
    freeze = _mapping(
        schedule_dir / "schedule_freeze_manifest.yaml", f"run {label} freeze manifest"
    )
    if freeze.get("schema") != FREEZE_SCHEMA:
        raise RepeatabilityError(f"run {label} has unsupported freeze schema")
    seed_contract = freeze.get("seed_contract")
    mappings = (
        seed_contract.get("launch_argument_columns")
        if isinstance(seed_contract, Mapping)
        else None
    )
    if mappings != {
        "policy_seed": "replicate_seed",
        "simulation_seed": "replicate_seed",
    }:
        raise RepeatabilityError(f"run {label} freeze seed contract is invalid")
    launch_log = ANSI_ESCAPE.sub(
        "", (run / "launch.log").read_text(encoding="utf-8", errors="replace")
    )
    if f"Setting seed value: {seed}" not in launch_log:
        raise RepeatabilityError(f"run {label} lacks Gazebo seed attestation")

    trace_path = run / "policy_trace.jsonl"
    trace = _jsonl(trace_path, f"run {label} policy trace")
    decisions = [item for item in trace if item.get("event") == "decision"]
    finished = [item for item in trace if item.get("event") == "session_finished"]
    if not decisions or len(finished) != 1:
        raise RepeatabilityError(f"run {label} lacks unambiguous policy events")
    first = decisions[0]
    first_payload = first.get("payload")
    finished_payload = finished[0].get("payload")
    if not isinstance(first_payload, Mapping) or not isinstance(finished_payload, Mapping):
        raise RepeatabilityError(f"run {label} policy payload is invalid")
    selected = first_payload.get("selected_candidate")
    target = first_payload.get("target_pose")
    if not isinstance(selected, Mapping) or not isinstance(target, list) or len(target) < 2:
        raise RepeatabilityError(f"run {label} first selected target is invalid")
    execution_key = selected.get("execution_key")
    if not isinstance(execution_key, list) or not execution_key:
        raise RepeatabilityError(f"run {label} first execution key is invalid")

    metrics = _jsonl(run / "evaluation_metrics.jsonl", f"run {label} metrics")
    snapshots = [
        item.get("payload")
        for item in metrics
        if item.get("event") == "metrics_snapshot"
        and isinstance(item.get("payload"), Mapping)
        and item["payload"].get("reason") == "policy_session_finished"
    ]
    if len(snapshots) != 1:
        raise RepeatabilityError(f"run {label} lacks one terminal evaluator snapshot")
    snapshot = snapshots[0]
    source = freeze.get("source")
    if not isinstance(source, Mapping):
        raise RepeatabilityError(f"run {label} freeze source is invalid")

    return RunEvidence(
        label=label,
        run_dir=run.relative_to(root).as_posix(),
        study_id=str(manifest.get("study_id", "")),
        schedule_id=str(manifest.get("schedule_id", "")),
        world_id=str(identity.get("world_id", "")),
        start_id=str(identity.get("start_id", "")),
        method=str(identity.get("method", "")),
        condition=str(identity.get("condition", "")),
        replicate_seed=int(seed),
        repository_commit=str(source.get("repository_commit", "")),
        source_tree_sha256=str(source.get("source_tree_sha256", "")),
        input_fingerprint=_input_fingerprint(freeze, identity),
        trace_sha256=sha256_file(trace_path),
        first_decision_ros_time_s=float(
            _number(first.get("ros_time_ns"), "first decision time")
        ) / 1e9,
        first_map_revision=int(
            _number(first.get("map_revision"), "first map revision", integer=True)
        ),
        first_known_free_cells=int(
            _number(
                first_payload.get("known_free_cells"),
                "first known-free count",
                integer=True,
            )
        ),
        first_execution_key=list(execution_key),
        first_target_x_m=float(_number(target[0], "first target x")),
        first_target_y_m=float(_number(target[1], "first target y")),
        decision_count=int(
            _number(finished_payload.get("decisions_issued"), "decision count", integer=True)
        ),
        navigation_goal_count=int(
            _number(
                _nested(snapshot, "actions", "navigation_goal_count"),
                "goal count",
                integer=True,
            )
        ),
        navigation_success_count=int(
            _number(
                _nested(snapshot, "actions", "navigation_success_count"),
                "success count",
                integer=True,
            )
        ),
        collision_count=int(
            _number(
                _nested(snapshot, "safety", "collision_count"),
                "collision count",
                integer=True,
            )
        ),
        information_coverage=float(
            _number(
                _nested(snapshot, "coverage_endpoints", "c_i_information"),
                "information coverage",
            )
        ),
        topological_coverage=float(
            _number(
                _nested(snapshot, "coverage_endpoints", "c_t_topological"),
                "topological coverage",
            )
        ),
        joint_coverage=float(
            _number(_nested(snapshot, "coverage_endpoints", "joint_min"), "joint coverage")
        ),
        target_recall_proxy=float(
            _number(_nested(snapshot, "targets", "target_recall"), "target recall")
        ),
        ground_truth_travel_m=float(
            _number(
                _nested(snapshot, "ground_truth_motion", "ground_truth_path_length_m"),
                "ground-truth travel",
            )
        ),
        ate_rmse_m=float(
            _number(_nested(snapshot, "ground_truth_motion", "ate_rmse_m"), "ATE RMSE")
        ),
    )


def compare_runs(first: RunEvidence, second: RunEvidence) -> dict[str, Any]:
    matched_fields = (
        "world_id",
        "start_id",
        "method",
        "condition",
        "replicate_seed",
        "repository_commit",
        "source_tree_sha256",
        "input_fingerprint",
    )
    mismatches = [
        field for field in matched_fields if getattr(first, field) != getattr(second, field)
    ]
    if mismatches:
        raise RepeatabilityError(
            "repeatability inputs are not a matched pair: " + ", ".join(mismatches)
        )
    target_separation = math.hypot(
        first.first_target_x_m - second.first_target_x_m,
        first.first_target_y_m - second.first_target_y_m,
    )
    deltas: dict[str, dict[str, float]] = {}
    for field in METRIC_FIELDS:
        left = float(getattr(first, field))
        right = float(getattr(second, field))
        denominator = (abs(left) + abs(right)) / 2.0
        deltas[field] = {
            "run_a": left,
            "run_b": right,
            "absolute_difference": abs(left - right),
            "symmetric_percent_difference": (
                0.0 if denominator == 0.0 else 100.0 * abs(left - right) / denominator
            ),
        }
    return {
        "schema": SCHEMA,
        "design": "descriptive_same_seed_stage0_repeat",
        "seed_control_attested": True,
        "bitwise_or_trajectory_determinism_observed": (
            first.trace_sha256 == second.trace_sha256
        ),
        "same_first_execution_key": (
            first.first_execution_key == second.first_execution_key
        ),
        "first_target_separation_m": target_separation,
        "first_known_free_cell_difference": abs(
            first.first_known_free_cells - second.first_known_free_cells
        ),
        "terminal_outcome_agreement": (
            first.navigation_goal_count == second.navigation_goal_count
            and first.navigation_success_count == second.navigation_success_count
            and first.collision_count == second.collision_count
        ),
        "tolerance_verdict": "not_applicable_no_preregistered_stage0_threshold",
        "inference_boundary": (
            "Seed 103 controls declared RNG inputs but asynchronous sensor, SLAM, "
            "executor and navigation scheduling does not yield bitwise trajectories. "
            "Use matched multi-seed blocks and inferential intervals for method claims."
        ),
        "runs": [asdict(first), asdict(second)],
        "metric_deltas": deltas,
    }


def _write_runs_csv(path: Path, runs: Sequence[RunEvidence]) -> None:
    rows = [asdict(run) for run in runs]
    fields = list(rows[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            row["first_execution_key"] = json.dumps(
                row["first_execution_key"], separators=(",", ":")
            )
            writer.writerow(row)


def _write_deltas_csv(path: Path, comparison: Mapping[str, Any]) -> None:
    deltas = comparison["metric_deltas"]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "metric",
                "run_a",
                "run_b",
                "absolute_difference",
                "symmetric_percent_difference",
            ),
        )
        writer.writeheader()
        for metric in METRIC_FIELDS:
            writer.writerow({"metric": metric, **deltas[metric]})


def _render_figure(path: Path, first: RunEvidence, second: RunEvidence) -> None:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    import numpy as np

    figure = Figure(figsize=(11.0, 7.5), constrained_layout=True)
    FigureCanvasAgg(figure)
    grid = figure.add_gridspec(2, 2)
    labels = (first.label, second.label)
    colors = ("#2563eb", "#dc2626")

    coverage_axis = figure.add_subplot(grid[0, 0])
    coverage_names = ("Information", "Topological", "Joint", "Target proxy")
    first_values = (
        first.information_coverage,
        first.topological_coverage,
        first.joint_coverage,
        first.target_recall_proxy,
    )
    second_values = (
        second.information_coverage,
        second.topological_coverage,
        second.joint_coverage,
        second.target_recall_proxy,
    )
    x = np.arange(len(coverage_names))
    coverage_axis.bar(x - 0.18, first_values, 0.36, label=labels[0], color=colors[0])
    coverage_axis.bar(x + 0.18, second_values, 0.36, label=labels[1], color=colors[1])
    coverage_axis.set_xticks(x, coverage_names, rotation=20, ha="right")
    coverage_axis.set_ylim(0.0, 1.0)
    coverage_axis.set_ylabel("Fraction")
    coverage_axis.set_title("Terminal coverage endpoints")
    coverage_axis.legend()
    coverage_axis.grid(axis="y", alpha=0.25)

    motion_axis = figure.add_subplot(grid[0, 1])
    motion_axis.bar(
        labels,
        (first.ground_truth_travel_m, second.ground_truth_travel_m),
        color=colors,
    )
    motion_axis.set_ylabel("Ground-truth travel [m]")
    motion_axis.set_title("Executed path")
    motion_axis.grid(axis="y", alpha=0.25)
    ate_axis = motion_axis.twinx()
    ate_axis.plot(
        labels,
        (first.ate_rmse_m, second.ate_rmse_m),
        marker="o",
        color="#111827",
        linewidth=2,
    )
    ate_axis.set_ylabel("ATE RMSE [m]")

    input_axis = figure.add_subplot(grid[1, 0])
    input_axis.bar(
        labels,
        (first.first_known_free_cells, second.first_known_free_cells),
        color=colors,
    )
    input_axis.set_ylabel("Known-free cells")
    input_axis.set_title(
        "First decision input "
        f"(t={first.first_decision_ros_time_s:g}/{second.first_decision_ros_time_s:g} s, "
        f"revision={first.first_map_revision}/{second.first_map_revision})"
    )
    input_axis.grid(axis="y", alpha=0.25)

    conclusion_axis = figure.add_subplot(grid[1, 1])
    conclusion_axis.axis("off")
    separation = math.hypot(
        first.first_target_x_m - second.first_target_x_m,
        first.first_target_y_m - second.first_target_y_m,
    )
    conclusion_axis.text(
        0.02,
        0.95,
        "Controlled input, stochastic trajectory",
        va="top",
        fontsize=15,
        fontweight="bold",
    )
    conclusion_axis.text(
        0.02,
        0.78,
        "\n".join(
            (
                f"Frozen Gazebo/policy seed: {first.replicate_seed}",
                "Same first execution key: "
                f"{first.first_execution_key == second.first_execution_key}",
                f"First-target separation: {separation:.3f} m",
                "Nav2 successes: "
                f"{first.navigation_success_count}/{first.navigation_goal_count} and "
                f"{second.navigation_success_count}/{second.navigation_goal_count}",
                f"Collisions: {first.collision_count} and {second.collision_count}",
                "No post-hoc Stage-0 tolerance verdict.",
            )
        ),
        va="top",
        fontsize=11,
        linespacing=1.5,
    )
    figure.suptitle(
        "Gazebo Stage-0 same-seed repeatability audit",
        fontsize=18,
        fontweight="bold",
    )
    figure.savefig(
        path,
        dpi=160,
        facecolor="white",
        metadata={
            "Title": "Gazebo Stage-0 same-seed repeatability audit",
            "Description": "Descriptive development simulation evidence",
        },
    )


def analyze_repeatability(
    *,
    root: Path,
    run_a: Path | str,
    run_b: Path | str,
    output_dir: Path | str,
) -> dict[str, Any]:
    root = root.resolve()
    first = load_run_evidence(root, run_a, label="run_a")
    second = load_run_evidence(root, run_b, label="run_b")
    comparison = compare_runs(first, second)
    output = _inside(root, str(output_dir), "repeatability output")
    if os.path.lexists(output):
        raise RepeatabilityError(f"refusing existing output directory: {output}")
    output.mkdir(parents=True)

    report_path = output / "repeatability_comparison.json"
    runs_path = output / "repeatability_runs.csv"
    deltas_path = output / "repeatability_deltas.csv"
    figure_path = output / "repeatability_figure.png"
    report_path.write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_runs_csv(runs_path, (first, second))
    _write_deltas_csv(deltas_path, comparison)
    _render_figure(figure_path, first, second)
    outputs = (report_path, runs_path, deltas_path, figure_path)
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "development_simulation_not_formal_or_real_robot_evidence": True,
        "inputs": {
            "run_a": first.run_dir,
            "run_b": second.run_dir,
            "input_fingerprint": first.input_fingerprint,
        },
        "outputs": {
            path.name: {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
            for path in outputs
        },
    }
    manifest_path = output / "analysis_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return {
        "output_dir": str(output),
        "comparison": comparison,
        "analysis_manifest": str(manifest_path),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_a", type=Path)
    parser.add_argument("run_b", type=Path)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = analyze_repeatability(
            root=args.root,
            run_a=args.run_a,
            run_b=args.run_b,
            output_dir=args.output_dir,
        )
    except (OSError, RepeatabilityError) as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "output_dir": result["output_dir"],
                "seed_control_attested": result["comparison"]["seed_control_attested"],
                "same_first_execution_key": result["comparison"]["same_first_execution_key"],
                "terminal_outcome_agreement": result["comparison"]["terminal_outcome_agreement"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
