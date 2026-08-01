#!/usr/bin/env python3
"""Plan or execute one frozen ROS 2/Gazebo schedule row safely.

Planning is the default and does not start ROS.  ``--execute`` atomically
reserves the row's unique output directory, writes a launch manifest, and then
invokes ``ros2 launch`` without a shell.  Existing paths, including empty
directories, are never reused because policy and evaluator JSONL files append.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import partial
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

import yaml

try:
    from scripts.generate_system_sim_schedule import (
        CORE_BAG_REQUIRED_TOPICS,
        CORE_BAG_TOPICS,
        EXPERIMENT_BUDGET_FIELDS,
        FREEZE_SCHEMA,
        GAZEBO_SEED_MAX,
        GAZEBO_SEED_MIN,
        SCHEDULE_SCHEMA,
        SEED_LAUNCH_ARGUMENTS,
        ScheduleError,
        sha256_file,
        sha256_tree,
        validate_experiment_budget,
        validate_recording_contract,
    )
except ModuleNotFoundError as error:
    if error.name != "scripts":
        raise
    from generate_system_sim_schedule import (  # type: ignore[no-redef]
        CORE_BAG_REQUIRED_TOPICS,
        CORE_BAG_TOPICS,
        EXPERIMENT_BUDGET_FIELDS,
        FREEZE_SCHEMA,
        GAZEBO_SEED_MAX,
        GAZEBO_SEED_MIN,
        SCHEDULE_SCHEMA,
        SEED_LAUNCH_ARGUMENTS,
        ScheduleError,
        sha256_file,
        sha256_tree,
        validate_experiment_budget,
        validate_recording_contract,
    )


ROOT = Path(__file__).resolve().parents[1]
RUN_MANIFEST_SCHEMA = "sstg_system_sim_run_launch/v1"
DEFAULT_SIGINT_GRACE_S = 15.0
DEFAULT_TERM_GRACE_S = 3.0


class RunnerError(ValueError):
    """Raised when a frozen run cannot be planned or safely launched."""


@dataclass(frozen=True)
class RunPlan:
    root: Path
    schedule_dir: Path
    schedule_id: str
    study_id: str
    schedule_sha256: str
    output_dir: Path
    launch_package: str
    launch_file: str
    launch_arguments: Mapping[str, str]
    experiment_budget: Mapping[str, float | int]
    recording_contract: Mapping[str, Any] | None
    command: tuple[str, ...]
    schedule_row: Mapping[str, str]


@dataclass(frozen=True)
class ProcessOutcome:
    status: str
    process_returncode: int | None
    shutdown_signals: tuple[str, ...]
    wall_elapsed_s: float
    terminal_event_observed: bool


@dataclass(frozen=True)
class ExecutionResult:
    status: str
    exit_code: int
    process_returncode: int | None
    artifact_audit: Mapping[str, Any]
    shutdown_signals: tuple[str, ...]
    wall_elapsed_s: float


REQUIRED_COMPLETION_ARTIFACTS = (
    "policy_manifest.json",
    "policy_trace.jsonl",
    "evaluation_manifest.json",
    "evaluation_metrics.jsonl",
    "evaluation_observed_policy_trace.jsonl",
    "launch.log",
)

PROCESS_DIED_PATTERN = re.compile(
    r"\[ERROR\] \[(?P<process>[^\]]+)\]: process has died .*"
    r"exit code (?P<code>-?\d+)"
)
PROCESS_FINISHED_PATTERN = re.compile(
    r"\[INFO\] \[(?P<process>[^\]]+)\]: process has finished cleanly"
)
FATAL_LAUNCH_MARKERS = (
    "Traceback (most recent call last):",
    "corrupted double-linked list",
    "double free or corruption",
    "terminate called after throwing",
)
SUPERVISOR_SHUTDOWN_BEGIN = "[sstg-runner] coordinated shutdown begin"
SUPERVISOR_SHUTDOWN_END = "[sstg-runner] coordinated shutdown complete signals="
REQUIRED_RUNTIME_PROCESS_PREFIXES = (
    "gazebo-",
    "parameter_bridge-",
    "robot_state_publisher-",
    "image_bridge-",
    "async_slam_toolbox_node-",
    "controller_server-",
    "smoother_server-",
    "planner_server-",
    "route_server-",
    "behavior_server-",
    "bt_navigator-",
    "waypoint_follower-",
    "velocity_smoother-",
    "collision_monitor-",
    "opennav_docking-",
    "lifecycle_manager-",
    "system_eval_node-",
    "policy_node-",
    "sstg_core_bag_recorder-",
)


def _shutdown_log_window(
    lines: Sequence[str],
) -> tuple[int | None, int | None, set[int], list[str]]:
    begin = [
        index for index, line in enumerate(lines)
        if SUPERVISOR_SHUTDOWN_BEGIN in line
    ]
    end = [
        index for index, line in enumerate(lines)
        if SUPERVISOR_SHUTDOWN_END in line
    ]
    if not begin and not end:
        return None, None, set(), []
    if len(begin) != 1 or len(end) != 1 or end[0] <= begin[0]:
        return None, None, set(), ["malformed coordinated shutdown markers"]
    signal_names = lines[end[0]].split(SUPERVISOR_SHUTDOWN_END, 1)[1].split(",")
    signal_names = [name.strip() for name in signal_names if name.strip()]
    allowed_codes: set[int] = set()
    for name in signal_names:
        try:
            allowed_codes.add(-int(signal.Signals[name]))
        except (KeyError, ValueError):
            return None, None, set(), [
                f"unknown coordinated shutdown signal: {name}"
            ]
    return begin[0], end[0], allowed_codes, []


def _launch_log_runtime_errors(path: Path) -> list[str]:
    """Detect early exits and crashes outside a runner-owned shutdown window."""
    try:
        content = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        return [f"cannot inspect runtime log: {error}"]
    lines = content.splitlines()
    shutdown_begin, shutdown_end, allowed_codes, detected = _shutdown_log_window(
        lines
    )
    for marker in FATAL_LAUNCH_MARKERS:
        if marker in content:
            detected.append(f"fatal runtime marker: {marker}")
    for index, line in enumerate(lines):
        finished = PROCESS_FINISHED_PATTERN.search(line)
        if (
            finished is not None
            and finished.group("process").startswith(
                REQUIRED_RUNTIME_PROCESS_PREFIXES
            )
            and (shutdown_begin is None or index < shutdown_begin)
        ):
            detected.append(
                "required process exited before coordinated shutdown: "
                f"{finished.group('process')}"
            )
        match = PROCESS_DIED_PATTERN.search(line)
        if match is None:
            continue
        process = match.group("process")
        code = int(match.group("code"))
        if (
            shutdown_begin is not None
            and shutdown_end is not None
            and shutdown_begin < index < shutdown_end
            and code in allowed_codes
        ):
            continue
        detected.append(f"child process crashed: {process} exit code {code}")
    return list(dict.fromkeys(detected))


def _jsonl_records(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    records: list[dict[str, Any]] = []
    errors: list[str] = []
    try:
        content = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        return records, [f"{path.name}: cannot read JSONL: {error}"]
    if content and not content.endswith("\n"):
        errors.append(f"{path.name}: final JSONL record is not newline-terminated")
    for line_number, line in enumerate(content.splitlines(), start=1):
        if not line.strip():
            errors.append(f"{path.name}:{line_number}: blank JSONL record")
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            errors.append(
                f"{path.name}:{line_number}: invalid JSON: {error.msg}"
            )
            continue
        if not isinstance(value, dict):
            errors.append(f"{path.name}:{line_number}: record is not an object")
            continue
        records.append(value)
    if not records:
        errors.append(f"{path.name}: contains no JSON object records")
    return records, errors


def jsonl_contains_event(path: Path, event: str) -> bool:
    """Inspect only complete JSONL records while a writer may still be active."""
    try:
        content = path.read_text(encoding="utf-8")
    except (FileNotFoundError, OSError, UnicodeError):
        return False
    complete_content = content if content.endswith("\n") else content.rpartition("\n")[0]
    for line in complete_content.splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and value.get("event") == event:
            return True
    return False


def _core_bag_artifacts(
    output_dir: Path,
    contract: Mapping[str, Any],
) -> tuple[dict[str, dict[str, Any]], list[str], dict[str, Any]]:
    """Validate a finalized rosbag2 MCAP and return hashable evidence records."""
    files: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    summary: dict[str, Any] = {
        "required": True,
        "storage_id": None,
        "message_count": 0,
        "duration_ns": 0,
        "topic_message_counts": {},
        "complete": False,
    }
    bag_dir = output_dir / str(contract.get("output", ""))
    if bag_dir.is_symlink() or not bag_dir.is_dir():
        return files, ["core bag directory is missing or is a symlink"], summary
    try:
        resolved_output_dir = output_dir.resolve(strict=True)
        resolved_bag_dir = bag_dir.resolve(strict=True)
        resolved_bag_dir.relative_to(resolved_output_dir)
    except (OSError, ValueError):
        return files, ["core bag directory escapes the run output"], summary
    metadata_path = bag_dir / "metadata.yaml"
    if metadata_path.is_symlink() or not metadata_path.is_file():
        return files, ["core bag metadata.yaml is missing or is a symlink"], summary
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        return files, [f"core bag metadata is invalid: {error}"], summary
    information = (
        metadata.get("rosbag2_bagfile_information")
        if isinstance(metadata, Mapping)
        else None
    )
    if not isinstance(information, Mapping):
        return files, ["core bag metadata lacks rosbag2_bagfile_information"], summary

    storage_id = information.get("storage_identifier")
    summary["storage_id"] = storage_id
    if storage_id != contract.get("storage_id"):
        errors.append("core bag storage identifier disagrees with recording contract")
    message_count = information.get("message_count")
    if type(message_count) is not int or message_count <= 0:
        errors.append("core bag message_count must be a positive integer")
    else:
        summary["message_count"] = message_count
    duration = information.get("duration")
    duration_ns = duration.get("nanoseconds") if isinstance(duration, Mapping) else None
    if type(duration_ns) is not int or duration_ns <= 0:
        errors.append("core bag duration must be positive")
    else:
        summary["duration_ns"] = duration_ns

    topic_counts: dict[str, int] = {}
    topic_records = information.get("topics_with_message_count")
    if not isinstance(topic_records, list):
        errors.append("core bag metadata has no topic message counts")
    else:
        for record in topic_records:
            if not isinstance(record, Mapping):
                errors.append("core bag topic record is not a mapping")
                continue
            topic_metadata = record.get("topic_metadata")
            name = (
                topic_metadata.get("name")
                if isinstance(topic_metadata, Mapping)
                else None
            )
            count = record.get("message_count")
            if not isinstance(name, str) or not name.startswith("/"):
                errors.append("core bag topic record has an invalid name")
                continue
            if type(count) is not int or count < 0:
                errors.append(f"core bag topic {name} has an invalid count")
                continue
            if name in topic_counts:
                errors.append(f"core bag topic is duplicated in metadata: {name}")
                continue
            topic_counts[name] = count
    summary["topic_message_counts"] = topic_counts
    for topic in contract.get("required_nonempty_topics", CORE_BAG_REQUIRED_TOPICS):
        if topic_counts.get(str(topic), 0) <= 0:
            errors.append(f"core bag required topic is empty or absent: {topic}")

    relative_paths = information.get("relative_file_paths")
    if not isinstance(relative_paths, list) or not relative_paths:
        errors.append("core bag metadata names no MCAP files")
        relative_paths = []
    metadata_key = metadata_path.relative_to(output_dir).as_posix()
    try:
        files[metadata_key] = {
            "sha256": sha256_file(metadata_path),
            "size_bytes": metadata_path.stat().st_size,
        }
    except OSError as error:
        errors.append(f"cannot hash core bag metadata: {error}")
    for relative in relative_paths:
        if not isinstance(relative, str):
            errors.append("core bag MCAP path is not a string")
            continue
        candidate = bag_dir / relative
        try:
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(resolved_bag_dir)
        except (OSError, ValueError):
            errors.append(f"core bag MCAP path escapes or is missing: {relative}")
            continue
        if candidate.is_symlink() or not resolved.is_file() or resolved.suffix != ".mcap":
            errors.append(f"core bag path is not a regular MCAP file: {relative}")
            continue
        key = resolved.relative_to(resolved_output_dir).as_posix()
        try:
            size_bytes = resolved.stat().st_size
            if size_bytes <= 0:
                errors.append(f"core bag MCAP file is empty: {relative}")
            files[key] = {
                "sha256": sha256_file(resolved),
                "size_bytes": size_bytes,
            }
        except OSError as error:
            errors.append(f"cannot hash core bag MCAP file {relative}: {error}")
    summary["complete"] = not errors
    return files, errors, summary


def validate_completed_artifacts(
    output_dir: Path,
    *,
    expected_experiment_budget: Mapping[str, float | int] | None = None,
    expected_recording_contract: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Audit the minimum policy/evaluator evidence required for completion."""
    errors: list[str] = []
    files: dict[str, dict[str, Any]] = {}
    for name in REQUIRED_COMPLETION_ARTIFACTS:
        path = output_dir / name
        if not path.is_file():
            if name in REQUIRED_COMPLETION_ARTIFACTS:
                errors.append(f"missing required artifact: {name}")
            continue
        try:
            files[name] = {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        except OSError as error:
            errors.append(f"cannot hash artifact {name}: {error}")

    manifest_expectations = {
        "policy_manifest.json": "sstg_system_sim_policy_manifest/v1",
        "evaluation_manifest.json": "sstg_system_sim_evaluator_manifest/v2",
    }
    manifests: dict[str, dict[str, Any]] = {}
    for name, schema in manifest_expectations.items():
        path = output_dir / name
        if not path.is_file():
            continue
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            errors.append(f"{name}: invalid manifest JSON: {error}")
            continue
        if not isinstance(value, dict):
            errors.append(f"{name}: manifest is not an object")
            continue
        if value.get("schema") != schema:
            errors.append(
                f"{name}: schema {value.get('schema')!r} does not match {schema!r}"
            )
        manifests[name] = value
    policy_manifest = manifests.get("policy_manifest.json")
    if policy_manifest is not None and policy_manifest.get("truth_access") is not False:
        errors.append("policy_manifest.json: truth_access must be false")
    if policy_manifest is not None and expected_experiment_budget is not None:
        parameters = policy_manifest.get("parameters")
        observed_values = (
            {
                field: parameters[field]
                for field in EXPERIMENT_BUDGET_FIELDS
                if field in parameters
            }
            if isinstance(parameters, Mapping)
            else parameters
        )
        try:
            observed_budget = _normalized_experiment_budget(
                observed_values,
                label="policy_manifest.json parameters",
            )
        except RunnerError as error:
            errors.append(str(error))
        else:
            if observed_budget != dict(expected_experiment_budget):
                errors.append(
                    "policy_manifest.json: runtime experiment budget disagrees "
                    "with the frozen launch budget"
                )
    evaluator_manifest = manifests.get("evaluation_manifest.json")
    if (
        evaluator_manifest is not None
        and evaluator_manifest.get("truth_access") != "evaluator_only"
    ):
        errors.append(
            "evaluation_manifest.json: truth_access must be evaluator_only"
        )

    core_bag = {"required": False, "complete": None}
    if expected_recording_contract is not None:
        bag_files, bag_errors, core_bag = _core_bag_artifacts(
            output_dir, expected_recording_contract
        )
        files.update(bag_files)
        errors.extend(bag_errors)

    launch_runtime_errors: list[str] = []
    launch_log_path = output_dir / "launch.log"
    if launch_log_path.is_file():
        launch_runtime_errors = _launch_log_runtime_errors(launch_log_path)
        errors.extend(
            f"launch.log: {runtime_error}"
            for runtime_error in launch_runtime_errors
        )

    jsonl: dict[str, list[dict[str, Any]]] = {}
    for name in (
        "policy_trace.jsonl",
        "evaluation_metrics.jsonl",
        "evaluation_observed_policy_trace.jsonl",
    ):
        path = output_dir / name
        if not path.is_file():
            continue
        records, record_errors = _jsonl_records(path)
        errors.extend(record_errors)
        jsonl[name] = records
        if name in files:
            files[name]["record_count"] = len(records)

    policy_records = jsonl.get("policy_trace.jsonl", [])
    if policy_records and not any(
        record.get("event") == "session_finished" for record in policy_records
    ):
        errors.append("policy_trace.jsonl: session_finished is absent")
    observed_records = jsonl.get("evaluation_observed_policy_trace.jsonl", [])
    if observed_records and not any(
        record.get("event") == "session_finished" for record in observed_records
    ):
        errors.append(
            "evaluation_observed_policy_trace.jsonl: session_finished is absent"
        )
    metric_records = jsonl.get("evaluation_metrics.jsonl", [])
    ingested_terminal = any(
        record.get("event") == "policy_trace_ingested"
        and isinstance(record.get("payload"), dict)
        and record["payload"].get("event") == "session_finished"
        for record in metric_records
    )
    final_snapshot = any(
        record.get("event") == "metrics_snapshot"
        and isinstance(record.get("payload"), dict)
        and record["payload"].get("reason") == "policy_session_finished"
        for record in metric_records
    )
    if metric_records and not ingested_terminal:
        errors.append(
            "evaluation_metrics.jsonl: evaluator did not ingest session_finished"
        )
    if metric_records and not final_snapshot:
        errors.append(
            "evaluation_metrics.jsonl: final policy_session_finished snapshot is absent"
        )

    return {
        "valid": not errors,
        "errors": errors,
        "files": files,
        "completion_checks": {
            "policy_session_finished": any(
                record.get("event") == "session_finished"
                for record in policy_records
            ),
            "evaluator_observed_session_finished": any(
                record.get("event") == "session_finished"
                for record in observed_records
            ),
            "evaluator_ingested_session_finished": ingested_terminal,
            "evaluator_final_snapshot": final_snapshot,
            "launch_log_clean": not launch_runtime_errors,
            "core_bag_complete": core_bag.get("complete"),
        },
        "core_bag": core_bag,
    }


def _resolve_inside(root: Path, value: str | Path, label: str) -> Path:
    candidate = Path(value)
    resolved = (
        (root / candidate).resolve()
        if not candidate.is_absolute()
        else candidate.resolve()
    )
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise RunnerError(f"{label} must remain under project root: {value}") from error
    return resolved


def _load_yaml_mapping(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise RunnerError(f"missing {label}: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise RunnerError(f"invalid YAML in {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise RunnerError(f"{label} must be a YAML mapping: {path}")
    return value


def _schedule_rows(path: Path) -> list[dict[str, str]]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    except (OSError, csv.Error) as error:
        raise RunnerError(f"cannot read frozen run schedule {path}: {error}") from error
    if not rows:
        raise RunnerError(f"frozen run schedule contains no rows: {path}")
    return rows


def ensure_run_output_available(root: Path, output_dir: Path) -> None:
    """Reject every pre-existing path, including an empty directory."""
    output_dir = _resolve_inside(root, output_dir, "run output directory")
    if output_dir.exists() or output_dir.is_symlink():
        raise RunnerError(
            "refusing existing run output path; use a new schedule/run ID: "
            f"{output_dir}"
        )
    ancestor = output_dir.parent
    while not ancestor.exists() and ancestor != root:
        ancestor = ancestor.parent
    if ancestor.exists() and not ancestor.is_dir():
        raise RunnerError(f"run output parent is not a directory: {ancestor}")


def _launch_contract(
    manifest: Mapping[str, Any],
) -> tuple[str, str, dict[str, str], dict[str, str]]:
    launch = manifest.get("launch")
    if not isinstance(launch, Mapping):
        raise RunnerError("schedule freeze manifest has no launch contract")
    package = str(launch.get("package", ""))
    launch_file = str(launch.get("file", ""))
    fixed = launch.get("fixed_arguments")
    columns = launch.get("argument_columns")
    if not package or not launch_file:
        raise RunnerError("launch contract requires package and file")
    if not isinstance(fixed, Mapping) or not isinstance(columns, Mapping):
        raise RunnerError("launch contract requires argument mappings")
    fixed_args = {str(key): str(value) for key, value in fixed.items()}
    column_args = {str(key): str(value) for key, value in columns.items()}
    overlap = sorted(set(fixed_args) & set(column_args))
    if overlap:
        raise RunnerError(f"launch arguments declared twice: {', '.join(overlap)}")
    return package, launch_file, fixed_args, column_args


def _verify_file_record(
    root: Path, record: Any, label: str
) -> None:
    if not isinstance(record, Mapping):
        raise RunnerError(f"freeze manifest has no {label} record")
    path = _resolve_inside(root, str(record.get("path", "")), label)
    expected = str(record.get("sha256", ""))
    if not path.is_file() or not expected or sha256_file(path) != expected:
        raise RunnerError(f"frozen {label} changed or is missing: {path}")


def _verify_frozen_inputs(
    root: Path,
    manifest: Mapping[str, Any],
    source: Mapping[str, Any],
    row: Mapping[str, str],
) -> None:
    source_paths_value = source.get("source_paths")
    if not isinstance(source_paths_value, list) or not source_paths_value:
        raise RunnerError("freeze manifest has no source paths")
    source_paths = [
        _resolve_inside(root, str(path), "source path")
        for path in source_paths_value
    ]
    try:
        current_source_sha = sha256_tree(root, source_paths)
    except (ScheduleError, OSError) as error:
        raise RunnerError(f"cannot verify frozen source tree: {error}") from error
    if current_source_sha != row.get("source_tree_sha256"):
        raise RunnerError("frozen source tree changed after schedule freeze")

    inputs = manifest.get("inputs")
    if not isinstance(inputs, Mapping):
        raise RunnerError("freeze manifest has no inputs mapping")
    _verify_file_record(root, inputs.get("world_registry"), "world registry")
    _verify_file_record(root, inputs.get("shared_stack"), "shared stack")
    _verify_file_record(root, inputs.get("condition"), "condition config")

    methods = inputs.get("methods")
    if not isinstance(methods, list):
        raise RunnerError("freeze manifest has no method records")
    method_records = [
        item
        for item in methods
        if isinstance(item, Mapping) and str(item.get("method")) == row.get("method")
    ]
    if len(method_records) != 1:
        raise RunnerError("schedule method does not match exactly one frozen config")
    _verify_file_record(root, method_records[0], "method config")

    bundle_path = _resolve_inside(root, row.get("world_bundle", ""), "world bundle")
    if not bundle_path.is_dir():
        raise RunnerError(f"frozen world bundle is missing: {bundle_path}")
    try:
        current_bundle_sha = sha256_tree(root, [bundle_path])
    except (ScheduleError, OSError) as error:
        raise RunnerError(f"cannot verify frozen world bundle: {error}") from error
    if current_bundle_sha != row.get("world_bundle_sha256"):
        raise RunnerError("frozen world bundle changed after schedule freeze")


def _normalized_experiment_budget(
    value: Any, *, label: str
) -> dict[str, float | int]:
    try:
        return validate_experiment_budget(value, label=label)
    except ScheduleError as error:
        raise RunnerError(str(error)) from error


def _schedule_row_experiment_budget(
    row: Mapping[str, str],
) -> dict[str, float | int]:
    parsed: dict[str, float | int] = {}
    for field in EXPERIMENT_BUDGET_FIELDS:
        raw = row.get(field)
        if raw is None or raw == "":
            raise RunnerError(f"schedule row lacks experiment budget field: {field}")
        if field == "max_decisions":
            if not raw.isdigit():
                raise RunnerError(
                    "schedule row max_decisions must be a positive integer"
                )
            parsed[field] = int(raw)
        else:
            try:
                parsed[field] = float(raw)
            except ValueError as error:
                raise RunnerError(
                    f"schedule row {field} must be a finite positive number"
                ) from error
    return _normalized_experiment_budget(
        parsed, label="schedule row experiment_budget"
    )


def _experiment_budget_contract(
    *,
    root: Path,
    manifest: Mapping[str, Any],
    row: Mapping[str, str],
    fixed_args: Mapping[str, str],
    column_args: Mapping[str, str],
) -> dict[str, float | int]:
    """Cross-check budget provenance and require row-to-launch passthrough."""
    inputs = manifest.get("inputs")
    if not isinstance(inputs, Mapping):
        raise RunnerError("freeze manifest has no inputs mapping")
    shared_record = inputs.get("shared_stack")
    if not isinstance(shared_record, Mapping):
        raise RunnerError("freeze manifest has no shared stack record")
    shared_path = _resolve_inside(
        root, str(shared_record.get("path", "")), "shared stack"
    )
    shared_stack = _load_yaml_mapping(shared_path, "shared stack")
    declared = _normalized_experiment_budget(
        shared_stack.get("experiment_budget"),
        label="shared_stack.experiment_budget",
    )
    recorded_declared = _normalized_experiment_budget(
        shared_record.get("experiment_budget"),
        label="freeze manifest shared_stack.experiment_budget",
    )
    if recorded_declared != declared:
        raise RunnerError(
            "freeze manifest shared-stack experiment budget disagrees with its input"
        )

    effective = _normalized_experiment_budget(
        manifest.get("experiment_budget"),
        label="freeze manifest experiment_budget",
    )
    provenance = manifest.get("budget_provenance")
    if not isinstance(provenance, Mapping):
        raise RunnerError("freeze manifest has no budget provenance")
    source = provenance.get("source")
    overrides = provenance.get("development_overrides")
    if not isinstance(overrides, Mapping):
        raise RunnerError(
            "budget provenance development_overrides must be a mapping"
        )
    unknown = sorted(
        str(field) for field in overrides if field not in EXPERIMENT_BUDGET_FIELDS
    )
    if unknown:
        raise RunnerError(
            "budget provenance has unknown overrides: " + ", ".join(unknown)
        )

    eligibility = manifest.get("eligibility")
    if not isinstance(eligibility, Mapping):
        raise RunnerError("freeze manifest has no eligibility mapping")
    evidence_tier = eligibility.get("evidence_tier")
    if evidence_tier not in {"development", "formal"}:
        raise RunnerError("freeze manifest has invalid evidence tier")
    if source == "shared_stack":
        if overrides or effective != declared:
            raise RunnerError(
                "shared-stack budget provenance disagrees with the effective budget"
            )
    elif source == "development_override":
        if evidence_tier != "development" or not overrides:
            raise RunnerError(
                "experiment budget overrides are allowed only for development schedules"
            )
        candidate = dict(declared)
        candidate.update(overrides)
        normalized_candidate = _normalized_experiment_budget(
            candidate, label="budget provenance effective experiment_budget"
        )
        if normalized_candidate != effective:
            raise RunnerError(
                "development budget overrides disagree with the effective budget"
            )
    else:
        raise RunnerError(f"unsupported experiment budget source: {source!r}")
    if evidence_tier == "formal" and effective != declared:
        raise RunnerError("formal schedule experiment budget is not frozen")

    row_budget = _schedule_row_experiment_budget(row)
    if row_budget != effective:
        raise RunnerError(
            "schedule row experiment budget disagrees with the freeze manifest"
        )
    for field in EXPERIMENT_BUDGET_FIELDS:
        if field in fixed_args or column_args.get(field) != field:
            raise RunnerError(
                f"launch contract must pass {field} from its schedule column"
            )
    return effective


def _seed_contract(
    *,
    root: Path,
    manifest: Mapping[str, Any],
    row: Mapping[str, str],
    fixed_args: Mapping[str, str],
    column_args: Mapping[str, str],
) -> int:
    """Cross-check shared-stack, freeze, row, and launch RNG provenance."""
    expected = {
        "seed_source": "replicate_seed",
        "valid_range_inclusive": [GAZEBO_SEED_MIN, GAZEBO_SEED_MAX],
        "launch_argument_columns": {
            argument: "replicate_seed" for argument in SEED_LAUNCH_ARGUMENTS
        },
    }
    inputs = manifest.get("inputs")
    if not isinstance(inputs, Mapping):
        raise RunnerError("freeze manifest has no inputs mapping")
    shared_record = inputs.get("shared_stack")
    if not isinstance(shared_record, Mapping):
        raise RunnerError("freeze manifest has no shared stack record")
    shared_path = _resolve_inside(
        root, str(shared_record.get("path", "")), "shared stack"
    )
    shared_stack = _load_yaml_mapping(shared_path, "shared stack")
    physics = shared_stack.get("physics")
    if not isinstance(physics, Mapping):
        raise RunnerError("shared_stack.physics must be a mapping")
    raw_seed_range = physics.get("seed_valid_range_inclusive")
    if not (
        isinstance(raw_seed_range, list)
        and len(raw_seed_range) == 2
        and all(type(bound) is int for bound in raw_seed_range)
    ):
        raise RunnerError("shared-stack seed contract is unsupported")
    shared_contract = {
        "seed_source": physics.get("seed_source"),
        "valid_range_inclusive": raw_seed_range,
        "launch_argument_columns": expected["launch_argument_columns"],
    }
    if shared_contract != expected:
        raise RunnerError("shared-stack seed contract is unsupported")
    if shared_record.get("seed_contract") != expected:
        raise RunnerError(
            "freeze manifest shared-stack seed contract disagrees with its input"
        )
    if manifest.get("seed_contract") != expected:
        raise RunnerError("freeze manifest seed contract is unsupported")

    raw_seed = row.get("replicate_seed", "")
    if not isinstance(raw_seed, str):
        raise RunnerError(
            "schedule row replicate_seed must be a positive signed 32-bit integer"
        )
    try:
        replicate_seed = int(raw_seed)
    except ValueError as error:
        raise RunnerError(
            "schedule row replicate_seed must be a positive signed 32-bit integer"
        ) from error
    if (
        str(replicate_seed) != raw_seed.strip()
        or not GAZEBO_SEED_MIN <= replicate_seed <= GAZEBO_SEED_MAX
    ):
        raise RunnerError(
            "schedule row replicate_seed must be a positive signed 32-bit integer"
        )
    for argument in SEED_LAUNCH_ARGUMENTS:
        if argument in fixed_args or column_args.get(argument) != "replicate_seed":
            raise RunnerError(
                f"launch contract must pass {argument} from replicate_seed"
            )
    return replicate_seed


def _normalized_recording_contract(value: Any, *, label: str) -> dict[str, Any]:
    try:
        return validate_recording_contract(value, label=label)
    except ScheduleError as error:
        raise RunnerError(str(error)) from error


def _recording_contract(
    *,
    root: Path,
    manifest: Mapping[str, Any],
    fixed_args: Mapping[str, str],
    column_args: Mapping[str, str],
) -> dict[str, Any]:
    """Cross-check the shared, frozen, and launch rosbag2 profile."""
    inputs = manifest.get("inputs")
    if not isinstance(inputs, Mapping):
        raise RunnerError("freeze manifest has no inputs mapping")
    shared_record = inputs.get("shared_stack")
    if not isinstance(shared_record, Mapping):
        raise RunnerError("freeze manifest has no shared stack record")
    shared_path = _resolve_inside(
        root, str(shared_record.get("path", "")), "shared stack"
    )
    shared_stack = _load_yaml_mapping(shared_path, "shared stack")
    declared = _normalized_recording_contract(
        shared_stack.get("recording"), label="shared_stack.recording"
    )
    recorded = _normalized_recording_contract(
        shared_record.get("recording_contract"),
        label="freeze manifest shared_stack.recording_contract",
    )
    frozen = _normalized_recording_contract(
        manifest.get("recording_contract"),
        label="freeze manifest recording_contract",
    )
    if recorded != declared or frozen != declared:
        raise RunnerError("freeze manifest recording contract disagrees with its input")
    if fixed_args.get("record_bag") != "true" or "record_bag" in column_args:
        raise RunnerError("launch contract must fix record_bag=true")
    return frozen


def load_run_plan(
    *, root: Path, schedule_dir: Path, schedule_id: str
) -> RunPlan:
    """Load and integrity-check one run without mutating the filesystem."""
    root = root.resolve()
    schedule_dir = _resolve_inside(root, schedule_dir, "schedule directory")
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = _load_yaml_mapping(manifest_path, "schedule freeze manifest")
    if manifest.get("schema") != FREEZE_SCHEMA:
        raise RunnerError(
            f"schedule manifest schema {manifest.get('schema')!r} is not {FREEZE_SCHEMA!r}"
        )
    source = manifest.get("source")
    if not isinstance(source, Mapping):
        raise RunnerError("schedule manifest has no source mapping")
    expected_runner_sha = str(source.get("runner_tool_sha256", ""))
    if not expected_runner_sha or sha256_file(Path(__file__).resolve()) != expected_runner_sha:
        raise RunnerError("schedule runner changed after schedule freeze")

    outputs = manifest.get("outputs")
    if not isinstance(outputs, Mapping):
        raise RunnerError("schedule manifest has no outputs mapping")
    schedule_name = str(outputs.get("run_schedule", ""))
    if not schedule_name:
        raise RunnerError("schedule manifest does not name run_schedule.csv")
    schedule_path = _resolve_inside(schedule_dir, schedule_name, "run schedule")
    try:
        schedule_path.relative_to(schedule_dir)
    except ValueError as error:
        raise RunnerError("run schedule must remain inside the schedule directory") from error
    if not schedule_path.is_file():
        raise RunnerError(f"frozen run schedule is missing: {schedule_path}")
    expected_schedule_sha = str(outputs.get("run_schedule_sha256", ""))
    actual_schedule_sha = sha256_file(schedule_path)
    if not expected_schedule_sha or actual_schedule_sha != expected_schedule_sha:
        raise RunnerError("run_schedule.csv hash disagrees with the freeze manifest")

    matching = [
        row
        for row in _schedule_rows(schedule_path)
        if row.get("schedule_id") == schedule_id
    ]
    if len(matching) != 1:
        raise RunnerError(
            f"schedule_id must match exactly one row, found {len(matching)}: {schedule_id}"
        )
    row = matching[0]
    if row.get("schema") != SCHEDULE_SCHEMA:
        raise RunnerError(f"unsupported schedule row schema: {row.get('schema')!r}")
    study_id = str(manifest.get("study_id", ""))
    if not study_id or row.get("study_id") != study_id:
        raise RunnerError("schedule row study_id disagrees with freeze manifest")
    _verify_frozen_inputs(root, manifest, source, row)

    package, launch_file, fixed_args, column_args = _launch_contract(manifest)
    experiment_budget = _experiment_budget_contract(
        root=root,
        manifest=manifest,
        row=row,
        fixed_args=fixed_args,
        column_args=column_args,
    )
    _seed_contract(
        root=root,
        manifest=manifest,
        row=row,
        fixed_args=fixed_args,
        column_args=column_args,
    )
    recording_contract = _recording_contract(
        root=root,
        manifest=manifest,
        fixed_args=fixed_args,
        column_args=column_args,
    )
    launch_arguments = dict(fixed_args)
    for argument, column in column_args.items():
        value = row.get(column)
        if value is None or value == "":
            raise RunnerError(
                f"schedule row lacks launch value for {argument!r} from {column!r}"
            )
        launch_arguments[argument] = value

    for argument in ("world", "truth_map_yaml", "output_dir"):
        if argument not in launch_arguments:
            raise RunnerError(f"launch contract is missing required argument: {argument}")
    world_path = _resolve_inside(root, launch_arguments["world"], "world SDF")
    truth_map_path = _resolve_inside(
        root, launch_arguments["truth_map_yaml"], "truth map"
    )
    output_dir = _resolve_inside(
        root, launch_arguments["output_dir"], "run output directory"
    )
    if not world_path.is_file():
        raise RunnerError(f"scheduled world SDF is missing: {world_path}")
    if not truth_map_path.is_file():
        raise RunnerError(f"scheduled truth map is missing: {truth_map_path}")
    if sha256_file(world_path) != row.get("world_sdf_sha256"):
        raise RunnerError("scheduled world SDF changed after schedule freeze")
    ensure_run_output_available(root, output_dir)

    launch_arguments["world"] = str(world_path)
    launch_arguments["truth_map_yaml"] = str(truth_map_path)
    launch_arguments["output_dir"] = str(output_dir)
    command = (
        "ros2",
        "launch",
        package,
        launch_file,
        *(f"{key}:={launch_arguments[key]}" for key in sorted(launch_arguments)),
    )
    return RunPlan(
        root=root,
        schedule_dir=schedule_dir,
        schedule_id=schedule_id,
        study_id=study_id,
        schedule_sha256=actual_schedule_sha,
        output_dir=output_dir,
        launch_package=package,
        launch_file=launch_file,
        launch_arguments=launch_arguments,
        experiment_budget=experiment_budget,
        recording_contract=recording_contract,
        command=command,
        schedule_row=row,
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _manifest_value(plan: RunPlan, *, status: str, **updates: Any) -> dict[str, Any]:
    value: dict[str, Any] = {
        "schema": RUN_MANIFEST_SCHEMA,
        "study_id": plan.study_id,
        "schedule_id": plan.schedule_id,
        "schedule_sha256": plan.schedule_sha256,
        "schedule_dir": plan.schedule_dir.relative_to(plan.root).as_posix(),
        "output_dir": plan.output_dir.relative_to(plan.root).as_posix(),
        "experiment_budget": dict(plan.experiment_budget),
        "recording_contract": (
            dict(plan.recording_contract)
            if plan.recording_contract is not None
            else None
        ),
        "launch": {
            "package": plan.launch_package,
            "file": plan.launch_file,
            "arguments": dict(plan.launch_arguments),
            "command": list(plan.command),
        },
        "identity": {
            key: plan.schedule_row[key]
            for key in (
                "world_id",
                "world_name",
                "start_id",
                "method",
                "condition",
                "replicate_seed",
            )
        },
        "execution": {"status": status, **updates},
    }
    return value


def _write_manifest(path: Path, value: Mapping[str, Any]) -> None:
    content = yaml.safe_dump(dict(value), sort_keys=False, allow_unicode=False).encode(
        "utf-8"
    )
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        temporary.write_bytes(content)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def reserve_run_output(plan: RunPlan) -> Path:
    """Atomically reserve a unique run directory and write its launch manifest."""
    ensure_run_output_available(plan.root, plan.output_dir)
    plan.output_dir.parent.mkdir(parents=True, exist_ok=True)
    try:
        plan.output_dir.mkdir()
    except FileExistsError as error:
        raise RunnerError(
            f"run output was reserved concurrently: {plan.output_dir}"
        ) from error
    (plan.output_dir / "media" / "raw").mkdir(parents=True)
    (plan.output_dir / "bags").mkdir()
    manifest_path = plan.output_dir / "run_launch_manifest.yaml"
    _write_manifest(
        manifest_path,
        _manifest_value(plan, status="reserved", reserved_at_utc=_utc_now()),
    )
    return manifest_path


def _process_group_exists(process_group_id: int) -> bool:
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def shutdown_process_group(
    process: subprocess.Popen[Any],
    *,
    sigint_grace_s: float = DEFAULT_SIGINT_GRACE_S,
    term_grace_s: float = DEFAULT_TERM_GRACE_S,
    kill_grace_s: float = 1.0,
    clock: Any = time.monotonic,
    sleeper: Any = time.sleep,
) -> tuple[str, ...]:
    """Ask launch to stop cleanly, then terminate any remaining process group."""
    process_group_id = process.pid
    stages = (
        (signal.SIGINT, sigint_grace_s, False),
        (signal.SIGTERM, term_grace_s, True),
        (signal.SIGKILL, kill_grace_s, True),
    )
    sent: list[str] = []
    for signal_value, grace_s, signal_group in stages:
        process.poll()
        if not _process_group_exists(process_group_id):
            break
        try:
            if signal_group:
                os.killpg(process_group_id, signal_value)
            elif process.returncode is None:
                os.kill(process.pid, signal_value)
            else:
                continue
        except ProcessLookupError:
            continue
        sent.append(signal.Signals(signal_value).name)
        deadline = clock() + max(0.0, grace_s)
        while clock() < deadline:
            process.poll()
            if not _process_group_exists(process_group_id):
                break
            sleeper(min(0.05, max(0.0, deadline - clock())))
        if not _process_group_exists(process_group_id):
            break
    try:
        process.wait(timeout=0.2)
    except subprocess.TimeoutExpired:
        pass
    return tuple(sent)


def shutdown_process_group_logged(
    process: subprocess.Popen[Any],
    *,
    launch_log: Any,
    **shutdown_kwargs: Any,
) -> tuple[str, ...]:
    """Bracket supervisor signals with flushed, runner-owned log markers."""
    launch_log.write(f"{SUPERVISOR_SHUTDOWN_BEGIN}\n".encode("utf-8"))
    launch_log.flush()
    sent = shutdown_process_group(process, **shutdown_kwargs)
    signal_list = ",".join(sent)
    launch_log.write(
        f"{SUPERVISOR_SHUTDOWN_END}{signal_list}\n".encode("utf-8")
    )
    launch_log.flush()
    return sent


def _validate_supervision_parameters(
    *,
    wall_timeout_s: float,
    evaluator_flush_s: float,
    poll_interval_s: float,
    sigint_grace_s: float,
    term_grace_s: float,
) -> None:
    values = (
        (wall_timeout_s, "wall_timeout_s", False),
        (evaluator_flush_s, "evaluator_flush_s", True),
        (poll_interval_s, "poll_interval_s", False),
        (sigint_grace_s, "sigint_grace_s", True),
        (term_grace_s, "term_grace_s", True),
    )
    for value, label, allow_zero in values:
        invalid = (
            not math.isfinite(value)
            or value < 0.0
            or (not allow_zero and value == 0.0)
        )
        if invalid:
            qualifier = "non-negative" if allow_zero else "positive"
            raise RunnerError(f"{label} must be finite and {qualifier}")


def supervise_process(
    process: subprocess.Popen[Any],
    *,
    trace_path: Path,
    wall_timeout_s: float,
    evaluator_flush_s: float = 2.0,
    poll_interval_s: float = 0.2,
    sigint_grace_s: float = DEFAULT_SIGINT_GRACE_S,
    term_grace_s: float = DEFAULT_TERM_GRACE_S,
    clock: Any = time.monotonic,
    sleeper: Any = time.sleep,
    shutdown: Any = shutdown_process_group,
) -> ProcessOutcome:
    """Wait for the policy terminal record rather than for ros2 launch to exit."""
    _validate_supervision_parameters(
        wall_timeout_s=wall_timeout_s,
        evaluator_flush_s=evaluator_flush_s,
        poll_interval_s=poll_interval_s,
        sigint_grace_s=sigint_grace_s,
        term_grace_s=term_grace_s,
    )

    started = clock()
    wall_deadline = started + wall_timeout_s
    terminal_observed = False
    shutdown_signals: tuple[str, ...] = ()
    status = "early_exit"
    try:
        while True:
            if jsonl_contains_event(trace_path, "session_finished"):
                terminal_observed = True
                flush_deadline = min(clock() + evaluator_flush_s, wall_deadline)
                while clock() < flush_deadline and process.poll() is None:
                    sleeper(min(poll_interval_s, flush_deadline - clock()))
                shutdown_signals = shutdown(
                    process,
                    sigint_grace_s=sigint_grace_s,
                    term_grace_s=term_grace_s,
                    clock=clock,
                    sleeper=sleeper,
                )
                status = "terminal_observed"
                break
            if process.poll() is not None:
                shutdown_signals = shutdown(
                    process,
                    sigint_grace_s=sigint_grace_s,
                    term_grace_s=term_grace_s,
                    clock=clock,
                    sleeper=sleeper,
                )
                status = "early_exit"
                break
            if clock() >= wall_deadline:
                shutdown_signals = shutdown(
                    process,
                    sigint_grace_s=sigint_grace_s,
                    term_grace_s=term_grace_s,
                    clock=clock,
                    sleeper=sleeper,
                )
                status = "timeout"
                break
            sleeper(min(poll_interval_s, max(0.0, wall_deadline - clock())))
    except KeyboardInterrupt:
        shutdown_signals = shutdown(
            process,
            sigint_grace_s=sigint_grace_s,
            term_grace_s=term_grace_s,
            clock=clock,
            sleeper=sleeper,
        )
        status = "manual_interrupt"
    return ProcessOutcome(
        status=status,
        process_returncode=process.poll(),
        shutdown_signals=shutdown_signals,
        wall_elapsed_s=max(0.0, clock() - started),
        terminal_event_observed=terminal_observed,
    )


def execute_run(
    plan: RunPlan,
    *,
    wall_timeout_s: float = 1200.0,
    evaluator_flush_s: float = 2.0,
    poll_interval_s: float = 0.2,
    sigint_grace_s: float = DEFAULT_SIGINT_GRACE_S,
    term_grace_s: float = DEFAULT_TERM_GRACE_S,
) -> ExecutionResult:
    """Supervise one launch and audit evidence before declaring completion."""
    _validate_supervision_parameters(
        wall_timeout_s=wall_timeout_s,
        evaluator_flush_s=evaluator_flush_s,
        poll_interval_s=poll_interval_s,
        sigint_grace_s=sigint_grace_s,
        term_grace_s=term_grace_s,
    )
    manifest_path = reserve_run_output(plan)
    started = _utc_now()
    supervisor_started = time.monotonic()
    _write_manifest(
        manifest_path,
        _manifest_value(
            plan,
            status="starting",
            started_at_utc=started,
            wall_timeout_s=wall_timeout_s,
            evaluator_flush_s=evaluator_flush_s,
            sigint_grace_s=sigint_grace_s,
            term_grace_s=term_grace_s,
        ),
    )
    launch_log_path = plan.output_dir / "launch.log"
    process: subprocess.Popen[Any] | None = None
    try:
        with launch_log_path.open("xb") as launch_log:
            process = subprocess.Popen(
                list(plan.command),
                cwd=plan.root,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            _write_manifest(
                manifest_path,
                _manifest_value(
                    plan,
                    status="running",
                    started_at_utc=started,
                    process_id=process.pid,
                    process_group_id=process.pid,
                    wall_timeout_s=wall_timeout_s,
                    evaluator_flush_s=evaluator_flush_s,
                    sigint_grace_s=sigint_grace_s,
                    term_grace_s=term_grace_s,
                ),
            )
            outcome = supervise_process(
                process,
                trace_path=plan.output_dir / "policy_trace.jsonl",
                wall_timeout_s=wall_timeout_s,
                evaluator_flush_s=evaluator_flush_s,
                poll_interval_s=poll_interval_s,
                sigint_grace_s=sigint_grace_s,
                term_grace_s=term_grace_s,
                shutdown=partial(
                    shutdown_process_group_logged,
                    launch_log=launch_log,
                ),
            )
    except KeyboardInterrupt:
        if process is not None and launch_log_path.is_file():
            with launch_log_path.open("ab") as launch_log:
                shutdown_signals = shutdown_process_group_logged(
                    process,
                    launch_log=launch_log,
                    sigint_grace_s=sigint_grace_s,
                    term_grace_s=term_grace_s,
                )
        else:
            shutdown_signals = ()
        artifact_audit = validate_completed_artifacts(
            plan.output_dir,
            expected_experiment_budget=plan.experiment_budget,
            expected_recording_contract=plan.recording_contract,
        )
        wall_elapsed_s = max(0.0, time.monotonic() - supervisor_started)
        _write_manifest(
            manifest_path,
            _manifest_value(
                plan,
                status="manual_interrupt",
                started_at_utc=started,
                finished_at_utc=_utc_now(),
                process_returncode=(
                    process.poll() if process is not None else None
                ),
                supervisor_exit_code=130,
                wall_elapsed_s=wall_elapsed_s,
                sigint_grace_s=sigint_grace_s,
                term_grace_s=term_grace_s,
                shutdown_signals=list(shutdown_signals),
                artifact_audit=artifact_audit,
            ),
        )
        return ExecutionResult(
            status="manual_interrupt",
            exit_code=130,
            process_returncode=(process.poll() if process is not None else None),
            artifact_audit=artifact_audit,
            shutdown_signals=shutdown_signals,
            wall_elapsed_s=wall_elapsed_s,
        )
    except OSError as error:
        if process is not None:
            if launch_log_path.is_file():
                with launch_log_path.open("ab") as launch_log:
                    shutdown_process_group_logged(
                        process,
                        launch_log=launch_log,
                        sigint_grace_s=sigint_grace_s,
                        term_grace_s=term_grace_s,
                    )
            else:
                shutdown_process_group(
                    process,
                    sigint_grace_s=sigint_grace_s,
                    term_grace_s=term_grace_s,
                )
        artifact_audit = validate_completed_artifacts(
            plan.output_dir,
            expected_experiment_budget=plan.experiment_budget,
            expected_recording_contract=plan.recording_contract,
        )
        _write_manifest(
            manifest_path,
            _manifest_value(
                plan,
                status="launch_error",
                started_at_utc=started,
                finished_at_utc=_utc_now(),
                error=str(error),
                sigint_grace_s=sigint_grace_s,
                term_grace_s=term_grace_s,
                artifact_audit=artifact_audit,
            ),
        )
        raise RunnerError(f"could not invoke ros2 launch: {error}") from error

    artifact_audit = validate_completed_artifacts(
        plan.output_dir,
        expected_experiment_budget=plan.experiment_budget,
        expected_recording_contract=plan.recording_contract,
    )
    status = outcome.status
    if status == "terminal_observed":
        if outcome.process_returncode is None:
            status = "shutdown_failed"
        else:
            status = (
                "terminal_completed"
                if artifact_audit["valid"]
                else "artifact_validation_failed"
            )
    exit_codes = {
        "terminal_completed": 0,
        "timeout": 124,
        "early_exit": 3,
        "manual_interrupt": 130,
        "artifact_validation_failed": 4,
        "shutdown_failed": 5,
    }
    exit_code = exit_codes[status]
    _write_manifest(
        manifest_path,
        _manifest_value(
            plan,
            status=status,
            started_at_utc=started,
            finished_at_utc=_utc_now(),
            process_returncode=outcome.process_returncode,
            supervisor_exit_code=exit_code,
            terminal_event_observed=outcome.terminal_event_observed,
            wall_elapsed_s=outcome.wall_elapsed_s,
            wall_timeout_s=wall_timeout_s,
            evaluator_flush_s=evaluator_flush_s,
            sigint_grace_s=sigint_grace_s,
            term_grace_s=term_grace_s,
            shutdown_signals=list(outcome.shutdown_signals),
            artifact_audit=artifact_audit,
        ),
    )
    return ExecutionResult(
        status=status,
        exit_code=exit_code,
        process_returncode=outcome.process_returncode,
        artifact_audit=artifact_audit,
        shutdown_signals=outcome.shutdown_signals,
        wall_elapsed_s=outcome.wall_elapsed_s,
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--schedule-dir", type=Path, required=True)
    parser.add_argument("--schedule-id", required=True)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="reserve the output directory and invoke ros2 launch",
    )
    parser.add_argument(
        "--wall-timeout-s",
        type=float,
        default=1200.0,
        help="hard wall-clock limit including simulator startup (default: 1200)",
    )
    parser.add_argument(
        "--evaluator-flush-s",
        type=float,
        default=2.0,
        help="grace after policy session_finished before group shutdown (default: 2)",
    )
    parser.add_argument(
        "--sigint-grace-s",
        type=float,
        default=DEFAULT_SIGINT_GRACE_S,
        help="grace for ros2 launch lifecycle shutdown (default: 15)",
    )
    parser.add_argument(
        "--term-grace-s",
        type=float,
        default=DEFAULT_TERM_GRACE_S,
        help="grace for residual process-group termination (default: 3)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        plan = load_run_plan(
            root=args.root,
            schedule_dir=args.schedule_dir,
            schedule_id=args.schedule_id,
        )
        if args.execute:
            if shutil.which("ros2") is None:
                raise RunnerError("ros2 is not on PATH; source the ROS 2 setup first")
            result = execute_run(
                plan,
                wall_timeout_s=args.wall_timeout_s,
                evaluator_flush_s=args.evaluator_flush_s,
                sigint_grace_s=args.sigint_grace_s,
                term_grace_s=args.term_grace_s,
            )
            return result.exit_code
        _validate_supervision_parameters(
            wall_timeout_s=args.wall_timeout_s,
            evaluator_flush_s=args.evaluator_flush_s,
            poll_interval_s=0.2,
            sigint_grace_s=args.sigint_grace_s,
            term_grace_s=args.term_grace_s,
        )
    except RunnerError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "mode": "plan_only",
                "schedule_id": plan.schedule_id,
                "output_dir": str(plan.output_dir),
                "command": list(plan.command),
                "experiment_budget": dict(plan.experiment_budget),
                "recording_contract": (
                    dict(plan.recording_contract)
                    if plan.recording_contract is not None
                    else None
                ),
                "supervision": {
                    "wall_timeout_s": args.wall_timeout_s,
                    "evaluator_flush_s": args.evaluator_flush_s,
                    "sigint_grace_s": args.sigint_grace_s,
                    "term_grace_s": args.term_grace_s,
                    "completion_event": "policy_trace.jsonl:session_finished",
                },
                "filesystem_mutated": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
