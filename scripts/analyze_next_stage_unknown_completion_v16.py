#!/usr/bin/env python3
"""Summarise the next-stage ROS unknown-completion screen.

This is an experiment-only wrapper around the frozen single-run evaluator.
It accepts the parent schedule and optional recovery schedules, retains every
invalid row, and reports descriptive block-level results only.  It does not
modify the core analyzer, policies, or paper.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import statistics
from typing import Any

import yaml

try:
    from scripts.analyze_unknown_completion import (
        AnalysisError,
        METHOD_LABELS,
        analyze_run,
    )
except ModuleNotFoundError as error:
    if error.name != "scripts":
        raise
    from analyze_unknown_completion import (  # type: ignore[no-redef]
        AnalysisError,
        METHOD_LABELS,
        analyze_run,
    )

ROOT = Path(__file__).resolve().parents[1]
THRESHOLD = 0.95


def _read_rows(schedule_dir: Path) -> list[dict[str, str]]:
    path = schedule_dir / "run_schedule.csv"
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        row["_schedule_dir"] = str(schedule_dir)
    return rows


def _failure_class(error: Exception, row: dict[str, str]) -> str:
    text = str(error).lower()
    if "pytorch" in text or "torch" in text or "learning dependencies" in text:
        return "environment_blocked_ans_missing_torch"
    if "session_finished" in text or "terminal_completed" in text:
        return "incomplete_or_interrupted"
    if row.get("method") == "ans_adapted" and "policy" in text:
        return "ans_runtime_failure"
    return "analysis_error"


def _number(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _pair_key(row: dict[str, Any]) -> str:
    """Stable paired-cell identity shared by parent and recovery schedules."""
    return "|".join(
        str(row.get(field, ""))
        for field in ("world_id", "start_id", "condition", "replicate_seed")
    )


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot(output: Path, valid: list[dict[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    methods = ["sstg", "frontier", "nbv", "rrt_adapted", "ans_adapted"]
    labels = [METHOD_LABELS[m] for m in methods]
    colors = {
        "sstg": "#E45756", "frontier": "#4C78A8", "nbv": "#F58518",
        "rrt_adapted": "#54A24B", "ans_adapted": "#B279A2",
    }
    values: dict[str, list[float]] = {method: [] for method in methods}
    aucs: dict[str, list[float]] = {method: [] for method in methods}
    for row in valid:
        method = row["method"]
        if method not in values:
            continue
        distance = _number(row.get("distance_to_95_95_m"))
        auc = _number(row.get("coverage_distance_auc_at_95_95"))
        if distance is not None:
            values[method].append(distance)
        if auc is not None:
            aucs[method].append(auc)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    axes[0].boxplot(
        [values[m] or [float("nan")] for m in methods],
        labels=labels,
        showmeans=True,
    )
    axes[0].set_ylabel("Distance to evaluator-only 95/95 (m)")
    axes[0].set_title("First joint threshold crossing")
    axes[0].tick_params(axis="x", rotation=22)
    for index, method in enumerate(methods, start=1):
        axes[0].scatter(
            [index] * len(values[method]), values[method],
            color=colors[method], alpha=0.65, s=24,
        )
    axes[1].boxplot(
        [aucs[m] or [float("nan")] for m in methods],
        labels=labels,
        showmeans=True,
    )
    axes[1].set_ylabel("Coverage-distance AUC at 95/95")
    axes[1].set_title("Efficiency while reaching the endpoint")
    axes[1].set_ylim(0.0, 1.0)
    axes[1].tick_params(axis="x", rotation=22)
    for index, method in enumerate(methods, start=1):
        axes[1].scatter(
            [index] * len(aucs[method]), aucs[method],
            color=colors[method], alpha=0.65, s=24,
        )
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("schedule_dir", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    schedule_dirs = [path.resolve() for path in args.schedule_dir]
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    all_rows: list[dict[str, str]] = []
    for schedule_dir in schedule_dirs:
        all_rows.extend(_read_rows(schedule_dir))
    seen: set[str] = set()
    valid_records: list[dict[str, Any]] = []
    audit_records: list[dict[str, Any]] = []
    for row in all_rows:
        schedule_id = row["schedule_id"]
        if schedule_id in seen:
            continue
        seen.add(schedule_id)
        pair_key = _pair_key(row)
        try:
            result, _, hashes = analyze_run(
                row,
                sensor_threshold=THRESHOLD,
                topological_threshold=THRESHOLD,
            )
        except (AnalysisError, KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
            audit_records.append({
                "study_id": row.get("study_id", ""),
                "schedule_id": schedule_id,
                "block_id": row.get("block_id", ""),
                "pair_key": pair_key,
                "world_id": row.get("world_id", ""),
                "start_id": row.get("start_id", ""),
                "condition": row.get("condition", ""),
                "replicate_seed": row.get("replicate_seed", ""),
                "method": row.get("method", ""),
                "method_label": METHOD_LABELS.get(row.get("method", ""), row.get("method", "")),
                "run_output_dir": row.get("run_output_dir", ""),
                "status": "invalid",
                "failure_class": _failure_class(error, row),
                "error": str(error),
            })
            continue
        record = asdict(result)
        record.update({
            "study_id": row.get("study_id", ""),
            "block_id": row.get("block_id", ""),
            "pair_key": pair_key,
            "world_id": row.get("world_id", ""),
            "start_id": row.get("start_id", ""),
            "condition": row.get("condition", ""),
            "replicate_seed": row.get("replicate_seed", ""),
            "status": "valid",
            "failure_class": "",
            "input_hashes": json.dumps(hashes, sort_keys=True),
        })
        valid_records.append(record)
    _write_csv(output / "run_audit.csv", audit_records + valid_records)

    # Prefer a valid recovery row over an invalid parent row for a paired cell.
    # Recovery schedules have different block IDs, so pairing is normalized by
    # world/start/condition/seed rather than the schedule-local block ID.
    chosen: dict[tuple[str, str], dict[str, Any]] = {}
    for record in valid_records:
        key = (record["pair_key"], record["method"])
        chosen[key] = record
    pair_context: dict[str, dict[str, str]] = {}
    for row in all_rows:
        pair_context.setdefault(_pair_key(row), row)
    block_rows: list[dict[str, Any]] = []
    for pair_key in sorted(pair_context):
        context = pair_context[pair_key]
        for method in ("sstg", "frontier", "nbv", "rrt_adapted", "ans_adapted"):
            record = chosen.get((pair_key, method))
            block_rows.append({
                "pair_key": pair_key,
                "block_id": "" if record is None else record["block_id"],
                "world_id": context.get("world_id", "") if record is None else record["world_id"],
                "start_id": context.get("start_id", "") if record is None else record["start_id"],
                "condition": context.get("condition", "") if record is None else record["condition"],
                "replicate_seed": context.get("replicate_seed", "") if record is None else record["replicate_seed"],
                "method": method,
                "method_label": METHOD_LABELS[method],
                "status": "valid" if record is not None else "missing_or_invalid",
                "schedule_id": "" if record is None else record["schedule_id"],
                "distance_to_95_95_m": "" if record is None else record["distance_to_95_95_m"],
                "executions_to_95_95": "" if record is None else record["executions_to_95_95"],
                "coverage_distance_auc_at_95_95": "" if record is None else record["coverage_distance_auc_at_95_95"],
                "full_distance_m": "" if record is None else record["full_distance_m"],
                "full_decisions": "" if record is None else record["full_decisions"],
                "full_executions": "" if record is None else record["full_executions"],
                "full_navigation_failures": "" if record is None else record["full_navigation_failures"],
                "terminal_reason": "" if record is None else record["terminal_reason"],
            })
    _write_csv(output / "paired_blocks.csv", block_rows)

    aggregates: list[dict[str, Any]] = []
    for method in ("sstg", "frontier", "nbv", "rrt_adapted", "ans_adapted"):
        records = [record for record in chosen.values() if record["method"] == method]
        def mean(field: str) -> float | None:
            values = [_number(record.get(field)) for record in records]
            values = [value for value in values if value is not None]
            return statistics.fmean(values) if values else None
        def median(field: str) -> float | None:
            values = [_number(record.get(field)) for record in records]
            values = [value for value in values if value is not None]
            return statistics.median(values) if values else None
        aggregates.append({
            "method": method,
            "method_label": METHOD_LABELS[method],
            "valid_blocks": len(records),
            "first_95_95_reached": sum(bool(record["equivalent_95_95_reached"]) for record in records),
            "distance_to_95_95_mean_m": mean("distance_to_95_95_m"),
            "distance_to_95_95_median_m": median("distance_to_95_95_m"),
            "executions_to_95_95_mean": mean("executions_to_95_95"),
            "coverage_distance_auc_mean": mean("coverage_distance_auc_at_95_95"),
            "full_distance_mean_m": mean("full_distance_m"),
            "full_decisions_mean": mean("full_decisions"),
            "full_executions_mean": mean("full_executions"),
            "full_navigation_failures_mean": mean("full_navigation_failures"),
            "native_termination_counts": json.dumps({
                reason: sum(record["terminal_reason"] == reason for record in records)
                for reason in sorted({record["terminal_reason"] for record in records})
            }, sort_keys=True),
        })
    _write_csv(output / "aggregate_by_method.csv", aggregates)
    _plot(output / "first95_efficiency.png", list(chosen.values()))

    invalid = audit_records
    lines = [
        "# Next-stage unknown-completion screen — evidence-bounded summary",
        "",
        "This is a development-only four-world, one-start, one-seed ROS2/Gazebo screen. The evaluator-only first joint `Ci >= 0.95` and `Ct >= 0.95` crossing is the primary endpoint; truth is not provided to policy nodes. Results are descriptive and are not a formal test-set ranking.",
        "",
        f"- Scheduled rows read: {len(all_rows)} (unique IDs: {len(seen)})",
        f"- Valid terminal rows: {len(valid_records)}; invalid/incomplete retained: {len(invalid)}",
        "- Recovery rows are joined by world/start/condition/seed/method; an invalid parent row is never silently counted as a valid result.",
        "- ANS recovery uses the preinstalled Anaconda PyTorch 2.13.0 through the temporary workspace shim; the original missing-PyTorch startup failures remain in `run_audit.csv`.",
        "",
        "## Aggregate descriptive table",
        "",
        "See `aggregate_by_method.csv` and `paired_blocks.csv`. No p-values, confidence intervals, or population-level claims are generated.",
        "",
        "## Interpretation boundary",
        "",
        "SSTG is the primary method. Distance/AUC/action counts describe exploration efficiency; collision, clearance, ATE and Nav2 failures remain secondary execution diagnostics. Any budget or runtime failure is retained as incomplete rather than treated as success.",
        "",
        "![first95_efficiency](first95_efficiency.png)",
        "",
    ]
    (output / "CONCLUSION.md").write_text("\n".join(lines), encoding="utf-8")
    manifest = {
        "schema": "sstg_next_stage_unknown_completion_report/v1",
        "schedules": [str(path) for path in schedule_dirs],
        "thresholds": {"sensor": THRESHOLD, "topological": THRESHOLD},
        "valid_terminal_rows": len(valid_records),
        "invalid_or_incomplete_rows": len(invalid),
        "outputs": ["run_audit.csv", "paired_blocks.csv", "aggregate_by_method.csv", "first95_efficiency.png", "CONCLUSION.md"],
    }
    (output / "analysis_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
