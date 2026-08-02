#!/usr/bin/env python3
"""Create a fail-closed four-world paired descriptive simulation report.

The input must be ``system_sim_runs.csv`` produced by
``analyze_system_sim_experiments.py`` together with its sibling
``analysis_manifest.json``.  Every input row is retained and must participate
in exactly one pair keyed by ``world_id/start_id/condition/replicate_seed``.
Only the frozen SSTG and external-frontier endpoints are reported.  No
threshold, exclusion, imputation, confidence interval, or significance test is
introduced here.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import statistics
import sys
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
INPUT_ANALYSIS_SCHEMA = "sstg_system_sim_analysis/v1"
OUTPUT_SCHEMA = "sstg_system_sim_paired_descriptive/v1"
MANIFEST_SCHEMA = "sstg_system_sim_paired_descriptive_manifest/v1"
EXPECTED_WORLD_COUNT = 4
SSTG_METHOD = "sstg"
EXTERNAL_METHOD = "frontier_mrtsp_dp_external"
METHODS = (SSTG_METHOD, EXTERNAL_METHOD)
PAIR_KEY_FIELDS = (
    "world_id",
    "start_id",
    "condition",
    "replicate_seed",
)
PAIR_CONSISTENCY_FIELDS = (
    "study_id",
    "block_id",
    "site_family",
)
RUN_COMPLETION_FIELDS = {
    "run_manifest_present": "true",
    "execution_status": "terminal_completed",
    "executed": "true",
    "task_completed": "true",
    "artifact_audit_valid": "true",
    "snapshot_present": "true",
    "snapshot_reason": "policy_session_settled",
    "formal_result_eligible": "false",
}
CONTINUOUS_METRICS = (
    "information_coverage",
    "topological_coverage",
    "target_recall_proxy",
    "ground_truth_travel_m",
    "mean_clearance_m",
    "minimum_clearance_m",
    "clearance_q05_m",
    "ate_mean_m",
    "ate_rmse_m",
    "ate_max_m",
)
FRACTION_METRICS = frozenset(
    {
        "information_coverage",
        "topological_coverage",
        "target_recall_proxy",
    }
)
COUNT_METRICS = (
    "collision_count",
    "navigation_technical_failure_count",
)
ALL_METRICS = CONTINUOUS_METRICS + COUNT_METRICS
REQUIRED_INPUT_FIELDS = (
    "study_id",
    "schedule_id",
    "block_id",
    "world_id",
    "site_family",
    "start_id",
    "method",
    "condition",
    "replicate_seed",
    "run_output_dir",
    *RUN_COMPLETION_FIELDS,
    "evidence_error",
    *ALL_METRICS,
)
OUTPUT_IDENTITY_FIELDS = (
    "evidence_tier",
    "formal_result_eligible",
    "delta_definition",
    "study_id",
    "block_id",
    "world_id",
    "site_family",
    "start_id",
    "condition",
    "replicate_seed",
    "sstg_schedule_id",
    "external_schedule_id",
)
OUTPUT_FIELDS = OUTPUT_IDENTITY_FIELDS + tuple(
    field
    for metric in ALL_METRICS
    for field in (f"sstg_{metric}", f"external_{metric}", f"delta_{metric}")
)


class PairedAnalysisError(ValueError):
    """Raised when complete, unambiguous pairing cannot be established."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _inside_root(root: Path, path: Path, label: str) -> Path:
    root = root.resolve()
    candidate = path.expanduser()
    candidate = (
        candidate.resolve() if candidate.is_absolute() else (root / candidate).resolve()
    )
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise PairedAnalysisError(f"{label} escapes repository root: {path}") from error
    return candidate


def _reject_symlink_traversal(root: Path, path: Path, label: str) -> None:
    current = root.resolve()
    for part in path.resolve().relative_to(current).parts:
        current /= part
        if current.is_symlink():
            raise PairedAnalysisError(f"{label} traverses a symlink: {current}")


def _repo_path(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def _display_path(root: Path, path: Path) -> str:
    try:
        return _repo_path(root, path)
    except ValueError:
        return path.resolve().as_posix()


def _read_json_mapping(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise PairedAnalysisError(f"{label} is missing or a symlink: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PairedAnalysisError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise PairedAnalysisError(f"{label} must be a JSON object: {path}")
    return value


def _read_source_rows(path: Path) -> tuple[list[dict[str, str]], bytes]:
    if path.name != "system_sim_runs.csv":
        raise PairedAnalysisError(
            "input must be analyzer-generated system_sim_runs.csv"
        )
    if path.is_symlink() or not path.is_file():
        raise PairedAnalysisError(f"input CSV is missing or a symlink: {path}")
    content = path.read_bytes()
    try:
        text = content.decode("utf-8")
    except UnicodeDecodeError as error:
        raise PairedAnalysisError(f"input CSV is not UTF-8: {path}") from error
    reader = csv.DictReader(io.StringIO(text, newline=""))
    fields = reader.fieldnames
    if not fields:
        raise PairedAnalysisError("input CSV has no header")
    if len(fields) != len(set(fields)):
        raise PairedAnalysisError("input CSV has duplicate header fields")
    missing_fields = sorted(set(REQUIRED_INPUT_FIELDS) - set(fields))
    if missing_fields:
        raise PairedAnalysisError(
            "input CSV lacks required fields: " + ", ".join(missing_fields)
        )
    rows: list[dict[str, str]] = []
    for line_number, raw in enumerate(reader, 2):
        if None in raw:
            raise PairedAnalysisError(f"input CSV line {line_number} has excess cells")
        if any(value is None for value in raw.values()):
            raise PairedAnalysisError(f"input CSV line {line_number} is truncated")
        rows.append({str(key): str(value) for key, value in raw.items()})
    if not rows:
        raise PairedAnalysisError("input CSV has no run rows")
    return rows, content


def _verify_analyzer_provenance(
    input_path: Path, input_content: bytes, row_count: int
) -> tuple[Path, dict[str, Any]]:
    manifest_path = input_path.parent / "analysis_manifest.json"
    manifest = _read_json_mapping(manifest_path, "analyzer manifest")
    if manifest.get("schema") != INPUT_ANALYSIS_SCHEMA:
        raise PairedAnalysisError("analyzer manifest has unsupported schema")
    if manifest.get("evidence_source") != "system_simulation":
        raise PairedAnalysisError("analyzer manifest has wrong evidence source")
    if manifest.get("formal_result_eligible") is not False:
        raise PairedAnalysisError(
            "paired report is restricted to development, non-formal evidence"
        )
    outputs = manifest.get("outputs")
    record = outputs.get(input_path.name) if isinstance(outputs, Mapping) else None
    if not isinstance(record, Mapping):
        raise PairedAnalysisError("analyzer manifest does not register input CSV")
    digest = _sha256_bytes(input_content)
    if record.get("sha256") != digest or record.get("bytes") != len(input_content):
        raise PairedAnalysisError(
            "input CSV disagrees with analyzer manifest hash/size"
        )
    counts = manifest.get("counts")
    if not isinstance(counts, Mapping) or counts.get("scheduled_runs") != row_count:
        raise PairedAnalysisError("input row count disagrees with analyzer manifest")
    return manifest_path, manifest


def _required_text(row: Mapping[str, str], field: str, row_label: str) -> str:
    value = row[field].strip()
    if not value:
        raise PairedAnalysisError(f"{row_label} has missing {field}")
    return value


def _finite_metric(row: Mapping[str, str], field: str, row_label: str) -> float:
    value = _required_text(row, field, row_label)
    try:
        result = float(value)
    except ValueError as error:
        raise PairedAnalysisError(f"{row_label} {field} is not numeric") from error
    if not math.isfinite(result):
        raise PairedAnalysisError(f"{row_label} {field} must be finite")
    if field in FRACTION_METRICS:
        if not 0.0 <= result <= 1.0:
            raise PairedAnalysisError(f"{row_label} {field} must be in [0, 1]")
    elif result < 0.0:
        raise PairedAnalysisError(f"{row_label} {field} must be nonnegative")
    return result


def _count_metric(row: Mapping[str, str], field: str, row_label: str) -> int:
    value = _required_text(row, field, row_label)
    if not re.fullmatch(r"0|[1-9][0-9]*", value):
        raise PairedAnalysisError(
            f"{row_label} {field} must be a canonical nonnegative integer"
        )
    return int(value)


def _validated_run(row: Mapping[str, str], row_number: int) -> dict[str, Any]:
    schedule_id = _required_text(row, "schedule_id", f"row {row_number}")
    label = f"row {row_number} ({schedule_id})"
    method = _required_text(row, "method", label)
    if method not in METHODS:
        raise PairedAnalysisError(f"{label} has unsupported method {method!r}")
    for field in (
        "study_id",
        "block_id",
        "world_id",
        "site_family",
        "start_id",
        "condition",
        "run_output_dir",
    ):
        _required_text(row, field, label)
    seed = _required_text(row, "replicate_seed", label)
    if not re.fullmatch(r"0|[1-9][0-9]*", seed):
        raise PairedAnalysisError(
            f"{label} replicate_seed must be a canonical nonnegative integer"
        )
    for field, expected in RUN_COMPLETION_FIELDS.items():
        actual = row[field].strip()
        if actual != expected:
            raise PairedAnalysisError(
                f"incomplete pair member {label}: {field}={actual!r}, expected {expected!r}"
            )
    if row["evidence_error"].strip():
        raise PairedAnalysisError(
            f"incomplete pair member {label}: evidence_error is set"
        )
    result: dict[str, Any] = dict(row)
    result["replicate_seed_int"] = int(seed)
    for metric in CONTINUOUS_METRICS:
        result[metric] = _finite_metric(row, metric, label)
    for metric in COUNT_METRICS:
        result[metric] = _count_metric(row, metric, label)
    return result


def pair_rows(rows: Sequence[Mapping[str, str]]) -> list[dict[str, Any]]:
    """Validate all rows and return deterministic SSTG-minus-external pairs."""
    indexed: dict[tuple[str, str, str, int], dict[str, dict[str, Any]]] = {}
    studies: set[str] = set()
    for row_number, source in enumerate(rows, 2):
        row = _validated_run(source, row_number)
        key = (
            str(row["world_id"]),
            str(row["start_id"]),
            str(row["condition"]),
            int(row["replicate_seed_int"]),
        )
        by_method = indexed.setdefault(key, {})
        method = str(row["method"])
        if method in by_method:
            rendered = "/".join(str(value) for value in key)
            raise PairedAnalysisError(
                f"duplicate {method} row for frozen pair key {rendered}"
            )
        by_method[method] = row
        studies.add(str(row["study_id"]))
    if len(studies) != 1:
        raise PairedAnalysisError("input rows must belong to exactly one study")
    worlds = {key[0] for key in indexed}
    if len(worlds) != EXPECTED_WORLD_COUNT:
        raise PairedAnalysisError(
            f"expected exactly {EXPECTED_WORLD_COUNT} worlds, found {len(worlds)}"
        )

    paired: list[dict[str, Any]] = []
    for key in sorted(indexed, key=lambda item: (item[0], item[1], item[2], item[3])):
        by_method = indexed[key]
        missing = [method for method in METHODS if method not in by_method]
        if missing:
            rendered = "/".join(str(value) for value in key)
            raise PairedAnalysisError(
                f"incomplete frozen pair {rendered}; missing " + ", ".join(missing)
            )
        sstg = by_method[SSTG_METHOD]
        external = by_method[EXTERNAL_METHOD]
        for field in PAIR_CONSISTENCY_FIELDS:
            if sstg[field] != external[field]:
                raise PairedAnalysisError(
                    f"pair {key} disagrees on {field}: "
                    f"{sstg[field]!r} != {external[field]!r}"
                )
        if sstg["schedule_id"] == external["schedule_id"]:
            raise PairedAnalysisError(f"pair {key} reuses one schedule_id")
        row: dict[str, Any] = {
            "evidence_tier": "development_simulation",
            "formal_result_eligible": False,
            "delta_definition": "sstg_minus_frontier_mrtsp_dp_external",
            "study_id": sstg["study_id"],
            "block_id": sstg["block_id"],
            "world_id": key[0],
            "site_family": sstg["site_family"],
            "start_id": key[1],
            "condition": key[2],
            "replicate_seed": key[3],
            "sstg_schedule_id": sstg["schedule_id"],
            "external_schedule_id": external["schedule_id"],
        }
        for metric in ALL_METRICS:
            left = sstg[metric]
            right = external[metric]
            row[f"sstg_{metric}"] = left
            row[f"external_{metric}"] = right
            row[f"delta_{metric}"] = left - right
        paired.append(row)
    if len(rows) != 2 * len(paired):
        raise PairedAnalysisError("not every input run belongs to exactly one pair")
    return paired


def _csv_bytes(rows: Sequence[Mapping[str, Any]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow(
            {
                field: (
                    str(row[field]).lower()
                    if isinstance(row[field], bool)
                    else row[field]
                )
                for field in OUTPUT_FIELDS
            }
        )
    return stream.getvalue().encode("utf-8")


def _delta_summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for metric in ALL_METRICS:
        values = [float(row[f"delta_{metric}"]) for row in rows]
        summary[metric] = {
            "n_pairs": len(values),
            "mean_delta": statistics.fmean(values),
            "median_delta": statistics.median(values),
            "minimum_delta": min(values),
            "maximum_delta": max(values),
        }
    return summary


def _figure_bytes(rows: Sequence[Mapping[str, Any]]) -> bytes:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MaxNLocator

    panels = (
        ("information_coverage", "Information coverage", "ratio"),
        ("topological_coverage", "Topological coverage", "ratio"),
        ("target_recall_proxy", "Target recall proxy", "ratio"),
        ("ground_truth_travel_m", "Ground-truth travel", "m"),
        ("mean_clearance_m", "Mean clearance", "m"),
        ("minimum_clearance_m", "Minimum clearance", "m"),
        ("clearance_q05_m", "5th-percentile clearance", "m"),
        ("collision_count", "Collision count", "count"),
        ("ate_mean_m", "ATE mean", "m"),
        ("ate_rmse_m", "ATE RMSE", "m"),
        ("ate_max_m", "ATE maximum", "m"),
        ("navigation_technical_failure_count", "Technical failure count", "count"),
    )
    worlds = sorted({str(row["world_id"]) for row in rows})
    palette = ("#2563eb", "#dc2626", "#059669", "#7c3aed")
    colors = {world: palette[index] for index, world in enumerate(worlds)}
    figure, axes = plt.subplots(3, 4, figsize=(18, 10.5))
    figure.subplots_adjust(
        left=0.055,
        right=0.865,
        bottom=0.075,
        top=0.855,
        hspace=0.42,
        wspace=0.28,
    )
    for axis, (metric, title, unit) in zip(axes.flat, panels):
        for row in rows:
            world = str(row["world_id"])
            axis.plot(
                (0.0, 1.0),
                (row[f"external_{metric}"], row[f"sstg_{metric}"]),
                color=colors[world],
                alpha=0.62,
                linewidth=1.35,
                marker="o",
                markersize=4.2,
            )
        mean_delta = statistics.fmean(float(row[f"delta_{metric}"]) for row in rows)
        axis.set_title(title, fontsize=11, fontweight="bold")
        axis.set_xticks((0.0, 1.0), ("External", "SSTG"))
        axis.set_xlim(-0.18, 1.18)
        axis.set_ylabel(unit)
        axis.grid(axis="y", alpha=0.24)
        axis.text(
            0.03,
            0.97,
            f"mean Δ = {mean_delta:.3g}",
            transform=axis.transAxes,
            ha="left",
            va="top",
            fontsize=8.5,
            bbox={
                "boxstyle": "round,pad=0.2",
                "facecolor": "white",
                "alpha": 0.8,
                "edgecolor": "#d1d5db",
            },
        )
        if unit == "count":
            axis.yaxis.set_major_locator(MaxNLocator(integer=True))
    legend = [
        Line2D([0], [0], color=colors[world], marker="o", linewidth=1.5, label=world)
        for world in worlds
    ]
    figure.legend(
        handles=legend,
        loc="center left",
        bbox_to_anchor=(0.88, 0.5),
        ncol=1,
        frameon=False,
        title="World\n(each line is one frozen pair)",
    )
    figure.suptitle(
        "Four-world paired endpoint check — development simulation, descriptive only\n"
        f"Δ = SSTG − external; all {len(rows)} complete pairs retained; no thresholds or tests",
        fontsize=15,
        fontweight="bold",
    )
    stream = io.BytesIO()
    figure.savefig(
        stream,
        format="png",
        dpi=180,
        facecolor="white",
        bbox_inches="tight",
        metadata={
            "Title": "Four-world paired descriptive system-simulation endpoints",
            "Description": (
                "Development simulation only; delta is SSTG minus external; "
                "all complete frozen pairs retained; no thresholds or significance tests"
            ),
        },
    )
    plt.close(figure)
    return stream.getvalue()


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode(
        "utf-8"
    )


def _atomic_write(path: Path, content: bytes) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        temporary.write_bytes(content)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def analyze_paired_runs(
    *, root: Path, input_csv: Path, output_dir: Path
) -> dict[str, Any]:
    """Validate analyzer output and write a separate paired report directory."""
    root = root.resolve()
    source = _inside_root(root, input_csv, "input CSV")
    output = _inside_root(root, output_dir, "paired output directory")
    _reject_symlink_traversal(root, source, "input CSV")
    _reject_symlink_traversal(root, output.parent, "paired output directory")
    try:
        output.relative_to(source.parent)
    except ValueError:
        pass
    else:
        raise PairedAnalysisError(
            "paired output must be separate from the analyzer output directory"
        )
    if os.path.lexists(output):
        raise PairedAnalysisError(f"refusing existing paired output: {output}")

    source_rows, source_content = _read_source_rows(source)
    analyzer_manifest_path, analyzer_manifest = _verify_analyzer_provenance(
        source, source_content, len(source_rows)
    )
    pairs = pair_rows(source_rows)
    study_ids = {str(row["study_id"]) for row in pairs}
    assert len(study_ids) == 1
    study_id = next(iter(study_ids))
    if analyzer_manifest.get("study_id") != study_id:
        raise PairedAnalysisError("study_id disagrees with analyzer manifest")

    report = {
        "schema": OUTPUT_SCHEMA,
        "study_id": study_id,
        "evidence_tier": "development_simulation",
        "formal_result_eligible": False,
        "analysis_role": "descriptive_only",
        "pair_key": list(PAIR_KEY_FIELDS),
        "methods": {"sstg": SSTG_METHOD, "external": EXTERNAL_METHOD},
        "delta_definition": "SSTG - frontier_mrtsp_dp_external",
        "endpoint_policy": {
            "continuous": list(CONTINUOUS_METRICS),
            "counts": list(COUNT_METRICS),
            "missing": "reject entire paired analysis; never drop or impute a run",
            "inference": "none; no significance tests or confidence intervals",
            "thresholds": "none introduced by this analysis",
        },
        "counts": {
            "input_runs": len(source_rows),
            "paired_runs": len(pairs),
            "unique_worlds": len({str(row["world_id"]) for row in pairs}),
        },
        "descriptive_delta_summary": _delta_summary(pairs),
        "paired_runs": pairs,
    }
    derived = {
        "paired_run_deltas.csv": _csv_bytes(pairs),
        "paired_run_deltas.json": _json_bytes(report),
        "paired_endpoints.png": _figure_bytes(pairs),
    }
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "study_id": study_id,
        "evidence_tier": "development_simulation",
        "formal_result_eligible": False,
        "analysis_role": "descriptive_only",
        "pair_key": list(PAIR_KEY_FIELDS),
        "delta_definition": "SSTG - frontier_mrtsp_dp_external",
        "counts": {
            "input_runs": len(source_rows),
            "paired_runs": len(pairs),
            "unique_worlds": EXPECTED_WORLD_COUNT,
        },
        "inputs": {
            "system_sim_runs.csv": {
                "path": _repo_path(root, source),
                "sha256": _sha256_bytes(source_content),
                "bytes": len(source_content),
            },
            "analysis_manifest.json": {
                "path": _repo_path(root, analyzer_manifest_path),
                "sha256": sha256_file(analyzer_manifest_path),
                "bytes": analyzer_manifest_path.stat().st_size,
            },
            "analysis_tool": {
                "path": _display_path(root, Path(__file__)),
                "sha256": sha256_file(Path(__file__)),
            },
        },
        "outputs": {
            name: {"sha256": _sha256_bytes(content), "bytes": len(content)}
            for name, content in derived.items()
        },
    }
    manifest_bytes = _json_bytes(manifest)
    manifest_hash = _sha256_bytes(manifest_bytes)

    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    for name, content in derived.items():
        _atomic_write(output / name, content)
    _atomic_write(output / "paired_analysis_manifest.json", manifest_bytes)
    _atomic_write(
        output / "paired_analysis_manifest.sha256",
        f"{manifest_hash}  paired_analysis_manifest.json\n".encode("ascii"),
    )
    return manifest


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = analyze_paired_runs(
            root=args.root,
            input_csv=args.input_csv,
            output_dir=args.output_dir,
        )
    except (OSError, PairedAnalysisError, ValueError) as error:
        print(f"paired system-simulation analysis failed: {error}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "study_id": result["study_id"],
                "paired_runs": result["counts"]["paired_runs"],
                "output_dir": str(
                    _inside_root(args.root.resolve(), args.output_dir, "output")
                ),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
