#!/usr/bin/env python3
"""Fail-closed analysis for frozen SSTG ROS 2/Gazebo studies."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import random
import statistics
import sys
from typing import Any, Iterable, Mapping, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCHEDULE_SCHEMA = "sstg_system_sim_run_schedule/v2"
FREEZE_SCHEMA = "sstg_system_sim_schedule_freeze/v2"
RUN_MANIFEST_SCHEMA = "sstg_system_sim_run_launch/v1"
ANALYSIS_SCHEMA = "sstg_system_sim_analysis/v1"
FINAL_EVALUATOR_SNAPSHOT_REASON = "policy_session_settled"
ALLOWED_EXECUTION_STATUS = {
    "reserved",
    "starting",
    "running",
    "terminal_completed",
    "timeout",
    "early_exit",
    "manual_interrupt",
    "artifact_validation_failed",
    "shutdown_failed",
    "launch_error",
}
REQUIRED_SCHEDULE_COLUMNS = (
    "schema",
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
    "formal_result_eligible",
)
REQUIRED_COMPLETION_FILES = (
    "policy_manifest.json",
    "policy_trace.jsonl",
    "evaluation_manifest.json",
    "evaluation_metrics.jsonl",
    "evaluation_observed_policy_trace.jsonl",
    "launch.log",
)
RUN_FIELDS = (
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
    "formal_result_eligible",
    "nominal_condition",
    "run_manifest_present",
    "run_manifest_sha256",
    "execution_status",
    "executed",
    "task_completed",
    "artifact_audit_valid",
    "snapshot_present",
    "snapshot_reason",
    "snapshot_ros_time_ns",
    "information_coverage",
    "topological_coverage",
    "joint_coverage",
    "dual_threshold_success",
    "target_recall_proxy",
    "target_total_count",
    "detected_target_count",
    "ground_truth_travel_m",
    "ground_truth_sample_count",
    "unique_node_count",
    "raw_node_observation_count",
    "duplicate_node_observation_count",
    "redundant_node_fraction",
    "navigation_goal_count",
    "execution_count",
    "navigation_success_count",
    "navigation_failure_count",
    "navigation_canceled_count",
    "navigation_upstream_cancel_count",
    "navigation_adapter_cancel_count",
    "navigation_non_cancel_failure_count",
    "collision_count",
    "collision_free",
    "contact_message_count",
    "mean_clearance_m",
    "minimum_clearance_m",
    "clearance_q05_m",
    "maximum_penetration_depth_m",
    "ate_sample_count",
    "ate_mean_m",
    "ate_rmse_m",
    "ate_max_m",
    "evidence_error",
)
METRICS = (
    ("task_completion", "task_completed"),
    ("collision_free", "collision_free"),
    ("dual_success", "dual_threshold_success"),
    ("information_coverage", "information_coverage"),
    ("topological_coverage", "topological_coverage"),
    ("joint_coverage", "joint_coverage"),
    ("target_recall_proxy", "target_recall_proxy"),
    ("ground_truth_travel_m", "ground_truth_travel_m"),
    ("unique_node_count", "unique_node_count"),
    ("redundant_node_fraction", "redundant_node_fraction"),
    ("collision_count", "collision_count"),
    ("mean_clearance_m", "mean_clearance_m"),
    ("minimum_clearance_m", "minimum_clearance_m"),
    ("clearance_q05_m", "clearance_q05_m"),
    ("ate_rmse_m", "ate_rmse_m"),
)
AGGREGATE_FIELDS = (
    "condition",
    "method",
    "scheduled_runs",
    "executed_runs",
    "terminal_completed_runs",
    "failed_or_incomplete_runs",
    "not_executed_runs",
    "artifact_invalid_runs",
    *(field for metric, _column in METRICS for field in (
        f"{metric}_n_runs",
        f"{metric}_n_seeds",
        f"{metric}_mean",
        f"{metric}_ci95_low",
        f"{metric}_ci95_high",
    )),
)


class AnalysisError(ValueError):
    """Raised when frozen evidence integrity cannot be established."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    content = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(content).hexdigest()


def _resolve_inside(root: Path, value: str | Path, label: str) -> Path:
    candidate = Path(value)
    path = (root / candidate).resolve() if not candidate.is_absolute() else candidate.resolve()
    try:
        path.relative_to(root.resolve())
    except ValueError as error:
        raise AnalysisError(f"{label} must remain under project root: {value}") from error
    return path


def _display_path(root: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _load_mapping(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise AnalysisError(f"missing {label}: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise AnalysisError(f"invalid {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise AnalysisError(f"{label} must be a mapping: {path}")
    return value


def _read_schedule(path: Path) -> list[dict[str, str]]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fields = tuple(reader.fieldnames or ())
            missing = sorted(set(REQUIRED_SCHEDULE_COLUMNS) - set(fields))
            if missing:
                raise AnalysisError(
                    f"run_schedule.csv is missing columns: {', '.join(missing)}"
                )
            rows = list(reader)
    except (OSError, UnicodeError, csv.Error) as error:
        raise AnalysisError(f"cannot read run_schedule.csv: {error}") from error
    if not rows:
        raise AnalysisError("run_schedule.csv contains no runs")
    identifiers = [row["schedule_id"] for row in rows]
    if len(set(identifiers)) != len(identifiers):
        raise AnalysisError("run_schedule.csv contains duplicate schedule_id values")
    output_dirs = [row["run_output_dir"] for row in rows]
    if len(set(output_dirs)) != len(output_dirs):
        raise AnalysisError("run_schedule.csv contains duplicate run_output_dir values")
    for row in rows:
        if row["schema"] != SCHEDULE_SCHEMA:
            raise AnalysisError(f"unsupported schedule schema: {row['schema']!r}")
        try:
            seed = int(row["replicate_seed"])
        except ValueError as error:
            raise AnalysisError(
                f"invalid replicate_seed for {row['schedule_id']}"
            ) from error
        if seed < 0:
            raise AnalysisError(f"negative replicate_seed for {row['schedule_id']}")
        if row["formal_result_eligible"] not in {"true", "false"}:
            raise AnalysisError(
                f"invalid formal_result_eligible for {row['schedule_id']}"
            )
    return rows


def _json_records(path: Path, label: str) -> list[dict[str, Any]]:
    try:
        content = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise AnalysisError(f"cannot read {label}: {error}") from error
    if content and not content.endswith("\n"):
        raise AnalysisError(f"{label} has a truncated final JSONL record")
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(content.splitlines(), start=1):
        if not line:
            raise AnalysisError(f"{label}:{line_number} is blank")
        try:
            value = json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"non-standard JSON constant {value}")
            ))
        except (json.JSONDecodeError, ValueError) as error:
            raise AnalysisError(f"{label}:{line_number} is invalid JSON: {error}") from error
        if not isinstance(value, dict):
            raise AnalysisError(f"{label}:{line_number} is not a JSON object")
        records.append(value)
    return records


def _nested(value: Mapping[str, Any], *paths: Sequence[str]) -> Any:
    for path in paths:
        current: Any = value
        for key in path:
            if not isinstance(current, Mapping) or key not in current:
                current = None
                break
            current = current[key]
        if current is not None:
            return current
    return None


def _number(value: Any, label: str) -> float | int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise AnalysisError(f"{label} must be numeric, not boolean")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise AnalysisError(f"{label} is not numeric: {value!r}") from error
    if not math.isfinite(result):
        raise AnalysisError(f"{label} is non-finite")
    return int(result) if result.is_integer() and isinstance(value, int) else result


def _bounded_fraction(value: Any, label: str) -> float | int | None:
    result = _number(value, label)
    if result is not None and not 0.0 <= float(result) <= 1.0:
        raise AnalysisError(f"{label} must be in [0, 1]")
    return result


def _nonnegative(value: Any, label: str) -> float | int | None:
    result = _number(value, label)
    if result is not None and float(result) < 0.0:
        raise AnalysisError(f"{label} must be non-negative")
    return result


def _boolean(value: Any, label: str) -> bool | None:
    if value is None:
        return None
    if not isinstance(value, bool):
        raise AnalysisError(f"{label} must be true, false, or null")
    return value


def _last_snapshot(
    metrics_path: Path,
    *,
    reason: str | None = None,
) -> dict[str, Any] | None:
    if not metrics_path.is_file():
        return None
    records = _json_records(metrics_path, metrics_path.name)
    snapshots = [
        record.get("payload")
        for record in records
        if record.get("event") == "metrics_snapshot"
        and isinstance(record.get("payload"), dict)
        and (
            reason is None
            or record["payload"].get("reason") == reason
        )
    ]
    return dict(snapshots[-1]) if snapshots else None


def _verify_declared_artifacts(
    run_dir: Path, run_manifest: Mapping[str, Any], completed: bool
) -> tuple[bool | None, list[str]]:
    execution = run_manifest.get("execution")
    if not isinstance(execution, Mapping):
        raise AnalysisError(f"run manifest has no execution mapping: {run_dir}")
    audit = execution.get("artifact_audit")
    if audit is None:
        if completed:
            raise AnalysisError(f"completed run has no artifact audit: {run_dir}")
        return None, ["artifact_audit_absent"]
    if not isinstance(audit, Mapping):
        raise AnalysisError(f"artifact audit is not a mapping: {run_dir}")
    audit_valid = audit.get("valid")
    if not isinstance(audit_valid, bool):
        raise AnalysisError(f"artifact audit valid flag is not boolean: {run_dir}")
    files = audit.get("files")
    if not isinstance(files, Mapping):
        raise AnalysisError(f"artifact audit files is not a mapping: {run_dir}")
    required = (
        tuple(sorted(set(REQUIRED_COMPLETION_FILES) | set(files)))
        if completed
        else tuple(files)
    )
    errors: list[str] = []
    for name in required:
        record = files.get(name)
        if not isinstance(record, Mapping):
            errors.append(f"missing_hash:{name}")
            continue
        relative = Path(str(name))
        if relative.is_absolute() or ".." in relative.parts:
            errors.append(f"unsafe_path:{name}")
            continue
        path = run_dir / relative
        try:
            path.resolve(strict=False).relative_to(run_dir.resolve())
        except ValueError:
            errors.append(f"unsafe_path:{name}")
            continue
        expected = str(record.get("sha256", ""))
        if path.is_symlink() or not path.is_file():
            errors.append(f"missing_file:{name}")
        elif len(expected) != 64 or sha256_file(path) != expected:
            errors.append(f"hash_mismatch:{name}")
    if completed and (not audit_valid or errors):
        raise AnalysisError(
            f"completed run failed artifact verification {run_dir}: "
            + ", ".join(errors or ["audit_valid_false"])
        )
    return audit_valid and not errors, errors


def _extract_snapshot(snapshot: Mapping[str, Any] | None) -> dict[str, Any]:
    result: dict[str, Any] = {}
    if snapshot is None:
        return result
    node_audit = _nested(snapshot, ("topological", "node_audit"))
    node_audit = node_audit if isinstance(node_audit, Mapping) else {}
    raw_nodes = _number(
        node_audit.get("raw_node_observation_count"), "raw node count"
    )
    duplicate_nodes = _number(
        node_audit.get("duplicate_node_observation_count"), "duplicate node count"
    )
    redundancy = None
    if raw_nodes is not None and raw_nodes > 0 and duplicate_nodes is not None:
        redundancy = float(duplicate_nodes) / float(raw_nodes)
    result.update({
        "snapshot_present": True,
        "snapshot_reason": snapshot.get("reason"),
        "snapshot_ros_time_ns": _number(snapshot.get("ros_time_ns"), "snapshot time"),
        "information_coverage": _bounded_fraction(_nested(
            snapshot,
            ("coverage_endpoints", "c_i_information"),
            ("topological", "information_coverage"),
            ("geometric", "geometric_coverage"),
        ), "information coverage"),
        "topological_coverage": _bounded_fraction(_nested(
            snapshot,
            ("coverage_endpoints", "c_t_topological"),
            ("topological", "topological_coverage"),
        ), "topological coverage"),
        "joint_coverage": _bounded_fraction(_nested(
            snapshot,
            ("coverage_endpoints", "joint_min"),
            ("topological", "joint_coverage"),
        ), "joint coverage"),
        "dual_threshold_success": _boolean(_nested(
            snapshot,
            ("coverage_endpoints", "dual_threshold_success"),
            ("topological", "dual_threshold_success"),
        ), "dual threshold success"),
        "target_recall_proxy": _bounded_fraction(_nested(
            snapshot, ("targets", "target_recall")
        ), "target recall proxy"),
        "target_total_count": _number(_nested(
            snapshot, ("targets", "target_total_count")
        ), "target total count"),
        "detected_target_count": _number(_nested(
            snapshot, ("targets", "detected_target_count")
        ), "detected target count"),
        "ground_truth_travel_m": _nonnegative(_nested(
            snapshot, ("ground_truth_motion", "ground_truth_path_length_m")
        ), "ground-truth travel"),
        "ground_truth_sample_count": _number(_nested(
            snapshot, ("ground_truth_motion", "ground_truth_sample_count")
        ), "ground-truth sample count"),
        "unique_node_count": _number(_nested(
            snapshot,
            ("topological", "node_audit", "unique_node_count"),
            ("topological", "unique_node_count"),
        ), "unique node count"),
        "raw_node_observation_count": raw_nodes,
        "duplicate_node_observation_count": duplicate_nodes,
        "redundant_node_fraction": redundancy,
        "navigation_goal_count": _number(_nested(
            snapshot, ("actions", "navigation_goal_count")
        ), "navigation goal count"),
        "execution_count": _number(_nested(
            snapshot, ("actions", "execution_count")
        ), "execution count"),
        "navigation_success_count": _number(_nested(
            snapshot, ("actions", "navigation_success_count")
        ), "navigation success count"),
        "navigation_failure_count": _number(_nested(
            snapshot, ("actions", "navigation_failure_count")
        ), "navigation failure count"),
        "navigation_canceled_count": _number(_nested(
            snapshot, ("actions", "navigation_canceled_count")
        ), "navigation canceled count"),
        "navigation_upstream_cancel_count": _number(_nested(
            snapshot, ("actions", "navigation_upstream_cancel_count")
        ), "navigation upstream cancel count"),
        "navigation_adapter_cancel_count": _number(_nested(
            snapshot, ("actions", "navigation_adapter_cancel_count")
        ), "navigation adapter cancel count"),
        "navigation_non_cancel_failure_count": _number(_nested(
            snapshot, ("actions", "navigation_non_cancel_failure_count")
        ), "navigation non-cancel failure count"),
        "collision_count": _number(_nested(
            snapshot, ("safety", "collision_count")
        ), "collision count"),
        "collision_free": _boolean(_nested(
            snapshot, ("safety", "collision_free")
        ), "collision free"),
        "contact_message_count": _number(_nested(
            snapshot, ("safety", "contact_message_count")
        ), "contact message count"),
        "mean_clearance_m": _number(_nested(
            snapshot,
            ("static_clearance", "footprint_clearance_mean_m"),
            ("clearance", "mean_clearance_m"),
            ("trajectory", "mean_clearance_m"),
        ), "mean clearance"),
        "minimum_clearance_m": _number(_nested(
            snapshot,
            ("static_clearance", "footprint_clearance_min_m"),
            ("clearance", "minimum_clearance_m"),
            ("clearance", "min_clearance_m"),
            ("trajectory", "minimum_clearance_m"),
        ), "minimum clearance"),
        "clearance_q05_m": _number(_nested(
            snapshot,
            ("static_clearance", "footprint_clearance_p05_m"),
            ("clearance", "clearance_q05_m"),
            ("trajectory", "clearance_q05_m"),
        ), "clearance q05"),
        "maximum_penetration_depth_m": _number(_nested(
            snapshot, ("safety", "maximum_reported_penetration_depth_m")
        ), "maximum penetration depth"),
        "ate_sample_count": _number(_nested(
            snapshot, ("ground_truth_motion", "ate_sample_count")
        ), "ATE sample count"),
        "ate_mean_m": _number(_nested(
            snapshot, ("ground_truth_motion", "ate_mean_m")
        ), "ATE mean"),
        "ate_rmse_m": _number(_nested(
            snapshot, ("ground_truth_motion", "ate_rmse_m")
        ), "ATE RMSE"),
        "ate_max_m": _number(_nested(
            snapshot, ("ground_truth_motion", "ate_max_m")
        ), "ATE maximum"),
    })
    for field in (
        "target_total_count",
        "detected_target_count",
        "ground_truth_sample_count",
        "unique_node_count",
        "raw_node_observation_count",
        "duplicate_node_observation_count",
        "navigation_goal_count",
        "execution_count",
        "navigation_success_count",
        "navigation_failure_count",
        "navigation_canceled_count",
        "navigation_upstream_cancel_count",
        "navigation_adapter_cancel_count",
        "navigation_non_cancel_failure_count",
        "collision_count",
        "contact_message_count",
        "mean_clearance_m",
        "minimum_clearance_m",
        "clearance_q05_m",
        "maximum_penetration_depth_m",
        "ate_sample_count",
        "ate_mean_m",
        "ate_rmse_m",
        "ate_max_m",
    ):
        value = result.get(field)
        if value is not None and float(value) < 0.0:
            raise AnalysisError(f"{field} must be non-negative")
    execution = result.get("execution_count")
    success = result.get("navigation_success_count")
    failure = result.get("navigation_failure_count")
    canceled = result.get("navigation_canceled_count")
    upstream_canceled = result.get("navigation_upstream_cancel_count")
    adapter_canceled = result.get("navigation_adapter_cancel_count")
    non_cancel_failure = result.get("navigation_non_cancel_failure_count")
    if None not in (execution, success, failure):
        if float(success) + float(failure) != float(execution):
            raise AnalysisError(
                "navigation success/failure counts do not partition executions"
            )
    if None not in (failure, canceled, non_cancel_failure):
        if float(canceled) + float(non_cancel_failure) != float(failure):
            raise AnalysisError(
                "navigation cancel/non-cancel counts do not partition failures"
            )
    if None not in (canceled, upstream_canceled, adapter_canceled):
        if float(upstream_canceled) + float(adapter_canceled) > float(canceled):
            raise AnalysisError(
                "attributed navigation cancellations exceed all cancellations"
            )
    if raw_nodes is not None and duplicate_nodes is not None:
        if float(duplicate_nodes) > float(raw_nodes):
            raise AnalysisError("duplicate node count exceeds raw node count")
    if result.get("target_total_count") is not None and result.get(
        "detected_target_count"
    ) is not None:
        detected = float(result["detected_target_count"])
        total = float(result["target_total_count"])
        if detected > total:
            raise AnalysisError("detected target count exceeds target total count")
        recall = result.get("target_recall_proxy")
        if recall is not None and total > 0.0:
            if not math.isclose(float(recall), detected / total, abs_tol=1e-9):
                raise AnalysisError("target recall disagrees with detected/total counts")
    information = result.get("information_coverage")
    topological = result.get("topological_coverage")
    joint = result.get("joint_coverage")
    if information is not None and topological is not None and joint is not None:
        if not math.isclose(
            float(joint), min(float(information), float(topological)), abs_tol=1e-9
        ):
            raise AnalysisError("joint coverage disagrees with min(C_I, C_T)")
    return result


def _run_row(
    root: Path,
    schedule_row: Mapping[str, str],
    schedule_sha256: str,
    main_condition: str,
) -> tuple[dict[str, Any], dict[str, str]]:
    run_dir = _resolve_inside(root, schedule_row["run_output_dir"], "run output")
    row: dict[str, Any] = {field: None for field in RUN_FIELDS}
    row.update({field: schedule_row.get(field) for field in (
        "study_id", "schedule_id", "block_id", "world_id", "site_family",
        "start_id", "method", "condition", "replicate_seed", "run_output_dir",
        "formal_result_eligible",
    )})
    row.update({
        "nominal_condition": schedule_row["condition"] == main_condition,
        "run_manifest_present": False,
        "executed": False,
        "task_completed": False,
        "snapshot_present": False,
        "execution_status": "not_executed",
    })
    manifest_path = run_dir / "run_launch_manifest.yaml"
    if not run_dir.exists():
        return row, {}
    if not run_dir.is_dir():
        raise AnalysisError(f"run output is not a directory: {run_dir}")
    if not manifest_path.is_file():
        raise AnalysisError(f"reserved run directory has no run manifest: {run_dir}")
    manifest = _load_mapping(manifest_path, "run launch manifest")
    if manifest.get("schema") != RUN_MANIFEST_SCHEMA:
        raise AnalysisError(f"unsupported run manifest schema: {manifest_path}")
    if manifest.get("schedule_id") != schedule_row["schedule_id"]:
        raise AnalysisError(f"run manifest schedule_id mismatch: {manifest_path}")
    if manifest.get("schedule_sha256") != schedule_sha256:
        raise AnalysisError(f"run manifest schedule hash mismatch: {manifest_path}")
    declared_output = manifest.get("output_dir")
    if declared_output is not None:
        declared_path = _resolve_inside(
            root, str(declared_output), "manifest output"
        )
        if declared_path != run_dir:
            raise AnalysisError(f"run manifest output_dir mismatch: {manifest_path}")
    identity = manifest.get("identity")
    if identity is not None:
        if not isinstance(identity, Mapping):
            raise AnalysisError(
                f"run manifest identity is not a mapping: {manifest_path}"
            )
        for field in (
            "world_id",
            "start_id",
            "method",
            "condition",
            "replicate_seed",
        ):
            if field in identity and str(identity[field]) != str(
                schedule_row[field]
            ):
                raise AnalysisError(
                    f"run manifest identity mismatch for {field}: {manifest_path}"
                )
    execution = manifest.get("execution")
    if not isinstance(execution, Mapping):
        raise AnalysisError(f"run manifest lacks execution mapping: {manifest_path}")
    status = str(execution.get("status", ""))
    if status not in ALLOWED_EXECUTION_STATUS:
        raise AnalysisError(f"unsupported execution status {status!r}: {manifest_path}")
    completed = status == "terminal_completed"
    audit_valid, audit_errors = _verify_declared_artifacts(
        run_dir, manifest, completed
    )
    snapshot = None
    if not audit_errors:
        snapshot = _last_snapshot(
            run_dir / "evaluation_metrics.jsonl",
            reason=FINAL_EVALUATOR_SNAPSHOT_REASON if completed else None,
        )
    if completed and (
        snapshot is None
        or snapshot.get("reason") != FINAL_EVALUATOR_SNAPSHOT_REASON
    ):
        raise AnalysisError(f"completed run lacks final evaluator snapshot: {run_dir}")
    row.update(_extract_snapshot(snapshot))
    row.update({
        "run_manifest_present": True,
        "run_manifest_sha256": sha256_file(manifest_path),
        "execution_status": status,
        "executed": status not in {"reserved", "starting", "launch_error"},
        "task_completed": completed,
        "artifact_audit_valid": audit_valid,
        "evidence_error": "|".join(audit_errors),
    })
    evidence_hashes = {"run_launch_manifest.yaml": sha256_file(manifest_path)}
    files = execution.get("artifact_audit", {}).get("files", {})
    if isinstance(files, Mapping):
        evidence_hashes.update({
            str(name): str(record.get("sha256"))
            for name, record in files.items()
            if isinstance(record, Mapping) and record.get("sha256")
        })
    return row, evidence_hashes


def _percentile(sorted_values: Sequence[float], probability: float) -> float:
    if not sorted_values:
        raise ValueError("percentile requires values")
    position = (len(sorted_values) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[lower]
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def seed_bootstrap_ci(
    values: Sequence[tuple[int, float]], *, resamples: int, seed: int
) -> tuple[float | None, float | None, float | None, int]:
    """Bootstrap seed means, treating worlds/starts within a seed as clustered."""
    if resamples <= 0:
        raise AnalysisError("bootstrap resamples must be positive")
    grouped: dict[int, list[float]] = {}
    for replicate_seed, value in values:
        if math.isfinite(value):
            grouped.setdefault(int(replicate_seed), []).append(float(value))
    if not grouped:
        return None, None, None, 0
    seed_means = [statistics.fmean(grouped[key]) for key in sorted(grouped)]
    estimate = statistics.fmean(seed_means)
    if len(seed_means) == 1:
        return estimate, estimate, estimate, 1
    generator = random.Random(seed)
    samples = sorted(
        statistics.fmean(generator.choice(seed_means) for _ in seed_means)
        for _ in range(resamples)
    )
    return estimate, _percentile(samples, 0.025), _percentile(samples, 0.975), len(seed_means)


def _aggregate(
    runs: Sequence[Mapping[str, Any]], *, resamples: int, seed: int
) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[Mapping[str, Any]]] = {}
    for row in runs:
        grouped.setdefault((str(row["condition"]), str(row["method"])), []).append(row)
    results: list[dict[str, Any]] = []
    for (condition, method), rows in sorted(grouped.items()):
        result: dict[str, Any] = {
            "condition": condition,
            "method": method,
            "scheduled_runs": len(rows),
            "executed_runs": sum(bool(row["executed"]) for row in rows),
            "terminal_completed_runs": sum(bool(row["task_completed"]) for row in rows),
            "failed_or_incomplete_runs": sum(not bool(row["task_completed"]) for row in rows),
            "not_executed_runs": sum(row["execution_status"] == "not_executed" for row in rows),
            "artifact_invalid_runs": sum(row["artifact_audit_valid"] is False for row in rows),
        }
        for metric_index, (metric, column) in enumerate(METRICS):
            values = [
                (int(row["replicate_seed"]), float(row[column]))
                for row in rows
                if row[column] is not None
            ]
            metric_seed = int(hashlib.sha256(
                f"{seed}:{condition}:{method}:{metric}".encode("utf-8")
            ).hexdigest()[:16], 16)
            mean, low, high, seed_count = seed_bootstrap_ci(
                values, resamples=resamples, seed=metric_seed + metric_index
            )
            result.update({
                f"{metric}_n_runs": len(values),
                f"{metric}_n_seeds": seed_count,
                f"{metric}_mean": mean,
                f"{metric}_ci95_low": low,
                f"{metric}_ci95_high": high,
            })
        results.append(result)
    return results


def _csv_bytes(rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({
            field: (
                ""
                if row.get(field) is None
                else str(row.get(field)).lower()
                if isinstance(row.get(field), bool)
                else row.get(field)
            )
            for field in fields
        })
    return stream.getvalue().encode("utf-8")


def _latex_escape(value: str) -> str:
    return value.replace("_", r"\_").replace("%", r"\%")


def _format_metric(row: Mapping[str, Any], metric: str, scale: float = 1.0) -> str:
    mean = row.get(f"{metric}_mean")
    low = row.get(f"{metric}_ci95_low")
    high = row.get(f"{metric}_ci95_high")
    if mean is None:
        return "--"
    return f"{float(mean) * scale:.2f} [{float(low) * scale:.2f}, {float(high) * scale:.2f}]"


def _latex_table(aggregates: Sequence[Mapping[str, Any]], condition: str) -> bytes:
    selected = [row for row in aggregates if row["condition"] == condition]
    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\scriptsize",
        r"\resizebox{\textwidth}{!}{%",
        r"\begin{tabular}{lrrrrrrrrrrrrr}",
        r"\toprule",
        (
            r"Method & Task comp. (\%) & Dual succ. (\%) & Collision-free (\%) "
            r"& $C_I$ & $C_T$ & Joint & Target proxy & GT travel (m) & Nodes "
            r"& Redund. (\%) & Collisions & Min clear. (m) & ATE RMSE (m) \\"
        ),
        r"\midrule",
    ]
    for row in selected:
        cells = [
            _latex_escape(str(row["method"])),
            _format_metric(row, "task_completion", 100.0),
            _format_metric(row, "dual_success", 100.0),
            _format_metric(row, "collision_free", 100.0),
            _format_metric(row, "information_coverage"),
            _format_metric(row, "topological_coverage"),
            _format_metric(row, "joint_coverage"),
            _format_metric(row, "target_recall_proxy"),
            _format_metric(row, "ground_truth_travel_m"),
            _format_metric(row, "unique_node_count"),
            _format_metric(row, "redundant_node_fraction", 100.0),
            _format_metric(row, "collision_count"),
            _format_metric(row, "minimum_clearance_m"),
            _format_metric(row, "ate_rmse_m"),
        ]
        lines.append(" & ".join(cells) + r" \\")
    lines.extend([
        r"\bottomrule",
        r"\end{tabular}%",
        r"}",
        (
            r"\caption{System-simulation results for the "
            + _latex_escape(condition)
            + r" condition. Brackets are seed-cluster bootstrap 95\% intervals. "
            r"Task completion is an artifact-valid terminal run and is distinct "
            r"from evaluator dual-threshold success and collision-free status. "
            r"Target recall is a deterministic geometry proxy. Missing metrics "
            r"are shown as -- and are not imputed.}"
        ),
        r"\label{tab:system-sim-main}",
        r"\end{table*}",
        "",
    ])
    return "\n".join(lines).encode("utf-8")


def _atomic_write(path: Path, content: bytes) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        temporary.write_bytes(content)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def analyze_study(
    *,
    root: Path,
    study_dir: Path,
    output_dir: Path,
    main_condition: str = "nominal",
    bootstrap_resamples: int = 2000,
    bootstrap_seed: int = 20260801,
    force: bool = False,
) -> dict[str, Any]:
    root = root.resolve()
    study_dir = _resolve_inside(root, study_dir, "study directory")
    output_dir = _resolve_inside(root, output_dir, "analysis output directory")
    if bootstrap_resamples <= 0:
        raise AnalysisError("bootstrap resamples must be positive")
    if output_dir.exists() and not output_dir.is_dir():
        raise AnalysisError(f"analysis output is not a directory: {output_dir}")
    existing = list(output_dir.iterdir()) if output_dir.exists() else []
    if existing and not force:
        raise AnalysisError(
            f"refusing non-empty analysis output directory: {output_dir}"
        )
    freeze_path = study_dir / "schedule_freeze_manifest.yaml"
    freeze = _load_mapping(freeze_path, "schedule freeze manifest")
    if freeze.get("schema") != FREEZE_SCHEMA:
        raise AnalysisError(f"unsupported schedule freeze schema: {freeze_path}")
    eligibility = freeze.get("eligibility")
    if not isinstance(eligibility, Mapping) or not isinstance(
        eligibility.get("formal_result_eligible"), bool
    ):
        raise AnalysisError("schedule freeze manifest has invalid eligibility")
    outputs = freeze.get("outputs")
    if not isinstance(outputs, Mapping):
        raise AnalysisError("schedule freeze manifest lacks outputs mapping")
    schedule_name = str(outputs.get("run_schedule", ""))
    schedule_path = (study_dir / schedule_name).resolve()
    try:
        schedule_path.relative_to(study_dir)
    except ValueError as error:
        raise AnalysisError("run schedule escapes study directory") from error
    if not schedule_path.is_file():
        raise AnalysisError(f"missing frozen run schedule: {schedule_path}")
    schedule_sha = sha256_file(schedule_path)
    if schedule_sha != outputs.get("run_schedule_sha256"):
        raise AnalysisError("run schedule hash disagrees with freeze manifest")
    schedule_rows = _read_schedule(schedule_path)
    study_id = str(freeze.get("study_id", ""))
    if not study_id or any(row["study_id"] != study_id for row in schedule_rows):
        raise AnalysisError("schedule study_id disagrees with freeze manifest")
    freeze_formal = bool(eligibility["formal_result_eligible"])
    if any(
        (row["formal_result_eligible"] == "true") != freeze_formal
        for row in schedule_rows
    ):
        raise AnalysisError("schedule formal eligibility disagrees with freeze manifest")

    runs: list[dict[str, Any]] = []
    run_evidence: dict[str, dict[str, str]] = {}
    for schedule_row in schedule_rows:
        row, evidence = _run_row(
            root, schedule_row, schedule_sha, main_condition
        )
        runs.append(row)
        run_evidence[schedule_row["schedule_id"]] = evidence
    aggregates = _aggregate(
        runs, resamples=bootstrap_resamples, seed=bootstrap_seed
    )

    run_bytes = _csv_bytes(runs, RUN_FIELDS)
    aggregate_bytes = _csv_bytes(aggregates, AGGREGATE_FIELDS)
    latex_bytes = _latex_table(aggregates, main_condition)
    derived = {
        "system_sim_runs.csv": run_bytes,
        "system_sim_method_aggregate.csv": aggregate_bytes,
        "system_sim_main_table.tex": latex_bytes,
    }
    output_hashes = {
        name: hashlib.sha256(content).hexdigest() for name, content in derived.items()
    }
    metric_missing_counts = {
        column: sum(row[column] is None for row in runs)
        for _metric, column in METRICS
    }
    manifest: dict[str, Any] = {
        "schema": ANALYSIS_SCHEMA,
        "study_id": study_id,
        "evidence_source": "system_simulation",
        "formal_result_eligible": freeze_formal,
        "definitions": {
            "task_completed": (
                "runner status terminal_completed with artifact-valid final "
                "policy_session_settled evaluator snapshot"
            ),
            "dual_threshold_success": (
                "evaluator C_I and C_T both meet frozen thresholds"
            ),
            "collision_free": (
                "evaluator configured-contact-sensor result; null remains missing"
            ),
            "target_recall_proxy": "deterministic evaluator geometry proxy",
            "continuous_metric_policy": (
                "use observed last snapshot for failed runs; never impute missing values"
            ),
        },
        "bootstrap": {
            "unit": "replicate_seed",
            "within_seed_reduction": "arithmetic mean across worlds/starts",
            "resamples": bootstrap_resamples,
            "seed": bootstrap_seed,
            "interval": "percentile_95",
        },
        "counts": {
            "scheduled_runs": len(runs),
            "executed_runs": sum(bool(row["executed"]) for row in runs),
            "terminal_completed_runs": sum(bool(row["task_completed"]) for row in runs),
            "not_executed_runs": sum(row["execution_status"] == "not_executed" for row in runs),
            "execution_status": {
                status: sum(row["execution_status"] == status for row in runs)
                for status in sorted({str(row["execution_status"]) for row in runs})
            },
            "metric_missing": metric_missing_counts,
        },
        "inputs": {
            "schedule_freeze_manifest": {
                "path": freeze_path.relative_to(root).as_posix(),
                "sha256": sha256_file(freeze_path),
            },
            "run_schedule": {
                "path": schedule_path.relative_to(root).as_posix(),
                "sha256": schedule_sha,
            },
            "run_evidence_sha256": _canonical_sha256(run_evidence),
            "run_evidence": run_evidence,
            "analysis_tool": {
                "path": _display_path(root, Path(__file__).resolve()),
                "sha256": sha256_file(Path(__file__).resolve()),
            },
        },
        "outputs": {
            name: {"sha256": digest, "bytes": len(derived[name])}
            for name, digest in output_hashes.items()
        },
    }
    manifest_bytes = (
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    manifest_sha = hashlib.sha256(manifest_bytes).hexdigest()

    output_dir.mkdir(parents=True, exist_ok=True)
    for name, content in derived.items():
        _atomic_write(output_dir / name, content)
    _atomic_write(output_dir / "analysis_manifest.json", manifest_bytes)
    _atomic_write(
        output_dir / "analysis_manifest.sha256",
        f"{manifest_sha}  analysis_manifest.json\n".encode("ascii"),
    )
    return manifest


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study_dir", type=Path)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--main-condition", default="nominal")
    parser.add_argument("--bootstrap-resamples", type=int, default=2000)
    parser.add_argument("--bootstrap-seed", type=int, default=20260801)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    root = args.root.resolve()
    try:
        study_dir = _resolve_inside(root, args.study_dir, "study directory")
        freeze = _load_mapping(
            study_dir / "schedule_freeze_manifest.yaml", "schedule freeze manifest"
        )
        study_id = str(freeze.get("study_id", study_dir.name))
        output = (
            args.output
            or Path("system_sim_outputs") / "reports" / study_id / "analysis"
        )
        manifest = analyze_study(
            root=root,
            study_dir=study_dir,
            output_dir=output,
            main_condition=args.main_condition,
            bootstrap_resamples=args.bootstrap_resamples,
            bootstrap_seed=args.bootstrap_seed,
            force=args.force,
        )
    except AnalysisError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(json.dumps({
        "study_id": manifest["study_id"],
        "scheduled_runs": manifest["counts"]["scheduled_runs"],
        "terminal_completed_runs": manifest["counts"]["terminal_completed_runs"],
        "formal_result_eligible": manifest["formal_result_eligible"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
