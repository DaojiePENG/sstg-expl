#!/usr/bin/env python3
"""Plan or execute one frozen ROS 2/Gazebo schedule row safely.

Planning is the default and does not start ROS.  ``--execute`` atomically
reserves the row's unique output directory, writes a launch manifest, and then
invokes ``ros2 launch`` without a shell.  Existing paths, including empty
directories, are never reused because policy and evaluator JSONL files append.
"""
from __future__ import annotations

import argparse
from collections import Counter
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
from xml.etree import ElementTree

import yaml

try:
    from scripts.generate_system_sim_schedule import (
        CORE_BAG_REQUIRED_TOPICS,
        CORE_BAG_TOPIC_TYPES,
        CORE_BAG_TOPICS,
        EXPERIMENT_BUDGET_FIELDS,
        FREEZE_SCHEMA,
        GAZEBO_SEED_MAX,
        GAZEBO_SEED_MIN,
        METHOD_POLICY_DEFAULTS,
        RUNTIME_ADAPTERS,
        SCHEDULE_SCHEMA,
        SEED_LAUNCH_ARGUMENTS,
        ScheduleError,
        sha256_file,
        sha256_tree,
        validate_experiment_budget,
        validate_recording_contract,
        validate_ros_gz_bridge_contract,
        validate_ros_middleware_contract,
    )
except ModuleNotFoundError as error:
    if error.name != "scripts":
        raise
    from generate_system_sim_schedule import (  # type: ignore[no-redef]
        CORE_BAG_REQUIRED_TOPICS,
        CORE_BAG_TOPIC_TYPES,
        CORE_BAG_TOPICS,
        EXPERIMENT_BUDGET_FIELDS,
        FREEZE_SCHEMA,
        GAZEBO_SEED_MAX,
        GAZEBO_SEED_MIN,
        METHOD_POLICY_DEFAULTS,
        RUNTIME_ADAPTERS,
        SCHEDULE_SCHEMA,
        SEED_LAUNCH_ARGUMENTS,
        ScheduleError,
        sha256_file,
        sha256_tree,
        validate_experiment_budget,
        validate_recording_contract,
        validate_ros_gz_bridge_contract,
        validate_ros_middleware_contract,
    )


ROOT = Path(__file__).resolve().parents[1]
RUN_MANIFEST_SCHEMA = "sstg_system_sim_run_launch/v1"
DEFAULT_SIGINT_GRACE_S = 15.0
DEFAULT_TERM_GRACE_S = 3.0
DEFAULT_EVALUATOR_SETTLEMENT_S = 5.0
FINAL_EVALUATOR_SNAPSHOT_REASON = "policy_session_settled"
MCAP_MAGIC = b"\x89MCAP0\r\n"


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
    ros_gz_bridge_contract: Mapping[str, Any] | None = None
    ros_middleware_contract: Mapping[str, Any] | None = None


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
PROCESS_STARTED_PATTERN = re.compile(
    r"\[INFO\] \[(?P<process>[^\]]+)\]: process started with pid"
)
FATAL_LAUNCH_MARKERS = (
    "Traceback (most recent call last):",
    "corrupted double-linked list",
    "double free or corruption",
    "terminate called after throwing",
)
SHUTDOWN_BRIDGE_EXCEPTION_PATTERN = re.compile(
    r"\[(?P<process>parameter_bridge-\d+)\] "
    r"terminate called after throwing an instance of 'std::system_error'"
)
SUPERVISOR_SHUTDOWN_BEGIN = "[sstg-runner] coordinated shutdown begin"
SUPERVISOR_SHUTDOWN_END = "[sstg-runner] coordinated shutdown complete signals="
COMMON_RUNTIME_PROCESS_PREFIXES = (
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
    "sstg_core_bag_recorder-",
)
RUNTIME_ADAPTER_PROCESS_PREFIXES = {
    "sstg_policy": ("policy_node-",),
    "frontier_mrtsp_dp_external": (
        "frontier_explorer-",
        "frontier_action_adapter-",
    ),
}
EXTERNAL_TOPOLOGICAL_VISIT_CONTRACT = "policy_transition_node_v1"
REQUIRED_RUNTIME_PROCESS_PREFIXES = (
    COMMON_RUNTIME_PROCESS_PREFIXES
    + tuple(
        prefix
        for prefixes in RUNTIME_ADAPTER_PROCESS_PREFIXES.values()
        for prefix in prefixes
    )
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


def _known_shutdown_bridge_race_lines(
    lines: Sequence[str],
    *,
    shutdown_begin: int | None,
    shutdown_end: int | None,
) -> tuple[set[int], set[int]]:
    """Identify the narrow ros_gz_bridge shutdown race seen after SIGINT.

    Jazzy's parameter bridge can throw ``std::system_error(Invalid argument)``
    while its executor is being torn down.  This is acceptable only inside a
    runner-owned shutdown window and only when the same bridge subsequently
    exits with SIGABRT.  Runtime occurrences remain fatal.
    """
    if shutdown_begin is None or shutdown_end is None:
        return set(), set()
    ignored_markers: set[int] = set()
    ignored_deaths: set[int] = set()
    for index in range(shutdown_begin + 1, shutdown_end):
        match = SHUTDOWN_BRIDGE_EXCEPTION_PATTERN.search(lines[index])
        if match is None:
            continue
        process = match.group("process")
        has_invalid_argument = any(
            f"[{process}]   what():  Invalid argument" in lines[probe]
            for probe in range(index + 1, shutdown_end)
        )
        death_indexes = [
            probe
            for probe in range(index + 1, shutdown_end)
            if (death := PROCESS_DIED_PATTERN.search(lines[probe])) is not None
            and death.group("process") == process
            and int(death.group("code")) == -int(signal.SIGABRT)
        ]
        if has_invalid_argument and death_indexes:
            ignored_markers.add(index)
            ignored_deaths.add(death_indexes[0])
    return ignored_markers, ignored_deaths


def _launch_log_runtime_errors(
    path: Path, expected_runtime_adapter: str | None = None
) -> list[str]:
    """Detect early exits and crashes outside a runner-owned shutdown window."""
    try:
        content = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        return [f"cannot inspect runtime log: {error}"]
    if (
        expected_runtime_adapter is not None
        and expected_runtime_adapter not in RUNTIME_ADAPTER_PROCESS_PREFIXES
    ):
        return [
            "unsupported expected runtime_adapter: "
            f"{expected_runtime_adapter!r}"
        ]
    lines = content.splitlines()
    required_prefixes = REQUIRED_RUNTIME_PROCESS_PREFIXES
    if expected_runtime_adapter is not None:
        required_prefixes = (
            COMMON_RUNTIME_PROCESS_PREFIXES
            + RUNTIME_ADAPTER_PROCESS_PREFIXES.get(
                expected_runtime_adapter, ()
            )
        )
    shutdown_begin, shutdown_end, allowed_codes, detected = _shutdown_log_window(
        lines
    )
    ignored_fatal_markers, ignored_bridge_deaths = (
        _known_shutdown_bridge_race_lines(
            lines,
            shutdown_begin=shutdown_begin,
            shutdown_end=shutdown_end,
        )
    )
    if expected_runtime_adapter is not None:
        started_processes = {
            match.group("process")
            for line in lines
            if (match := PROCESS_STARTED_PATTERN.search(line)) is not None
        }
        for prefix in RUNTIME_ADAPTER_PROCESS_PREFIXES.get(
            expected_runtime_adapter, ()
        ):
            if not any(
                process.startswith(prefix) for process in started_processes
            ):
                detected.append(
                    "required adapter process did not start: " + prefix
                )
    for index, line in enumerate(lines):
        for marker in FATAL_LAUNCH_MARKERS:
            if marker in line and index not in ignored_fatal_markers:
                detected.append(f"fatal runtime marker: {marker}")
        finished = PROCESS_FINISHED_PATTERN.search(line)
        if (
            finished is not None
            and finished.group("process").startswith(
                required_prefixes
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
        if index in ignored_bridge_deaths:
            continue
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
    complete_content = (
        content if content.endswith("\n") else content.rpartition("\n")[0]
    )
    for line in complete_content.splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and value.get("event") == event:
            return True
    return False


def jsonl_contains_snapshot_reason(path: Path, reason: str) -> bool:
    """Read complete evaluator JSONL records until a snapshot reason appears."""
    try:
        content = path.read_text(encoding="utf-8")
    except (FileNotFoundError, OSError, UnicodeError):
        return False
    complete_content = (
        content if content.endswith("\n") else content.rpartition("\n")[0]
    )
    for line in complete_content.splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        payload = value.get("payload") if isinstance(value, dict) else None
        if (
            isinstance(value, dict)
            and value.get("event") == "metrics_snapshot"
            and isinstance(payload, dict)
            and payload.get("reason") == reason
        ):
            return True
    return False


def _read_core_bag_to_eof(
    bag_dir: Path,
    storage_id: str,
) -> tuple[dict[str, int], dict[str, str], list[str]]:
    """Open the bag through upstream rosbag2 and consume every record."""
    try:
        import rosbag2_py
    except ImportError as error:
        return {}, {}, [f"rosbag2_py is unavailable: {error}"]

    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(bag_dir),
                storage_id=storage_id,
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="",
                output_serialization_format="",
            ),
        )
        topic_types: dict[str, str] = {}
        errors: list[str] = []
        for metadata in reader.get_all_topics_and_types():
            name = str(metadata.name)
            if name in topic_types:
                errors.append(f"rosbag2 reader returned duplicate topic: {name}")
                continue
            topic_types[name] = str(metadata.type)
        counts: Counter[str] = Counter()
        while reader.has_next():
            topic, _serialized_message, _timestamp = reader.read_next()
            counts[str(topic)] += 1
        return dict(counts), topic_types, errors
    except Exception as error:  # rosbag2 storage plugins raise native exceptions.
        return {}, {}, [f"rosbag2 reader could not consume core bag: {error}"]


def _core_bag_artifacts(
    output_dir: Path,
    contract: Mapping[str, Any],
    *,
    expected_runtime_adapter: str | None = None,
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
        "topic_types": {},
        "reader_message_count": 0,
        "reader_verified": False,
        "expected_runtime_adapter": expected_runtime_adapter,
        "required_nonempty_topics": [],
        "runtime_adapter_required_nonempty_topics": [],
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
    topic_types: dict[str, str] = {}
    expected_topic_types = contract.get("topic_types", CORE_BAG_TOPIC_TYPES)
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
            topic_type = (
                topic_metadata.get("type")
                if isinstance(topic_metadata, Mapping)
                else None
            )
            serialization = (
                topic_metadata.get("serialization_format")
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
            expected_type = (
                expected_topic_types.get(name)
                if isinstance(expected_topic_types, Mapping)
                else None
            )
            if not isinstance(topic_type, str) or topic_type != expected_type:
                errors.append(
                    f"core bag topic type disagrees with recording contract: {name}"
                )
            if serialization != "cdr":
                errors.append(f"core bag topic is not CDR serialized: {name}")
            topic_counts[name] = count
            topic_types[name] = str(topic_type)
    summary["topic_message_counts"] = topic_counts
    summary["topic_types"] = topic_types
    if type(message_count) is int and sum(topic_counts.values()) != message_count:
        errors.append("core bag topic counts do not sum to message_count")
    required_topics = list(
        contract.get("required_nonempty_topics", CORE_BAG_REQUIRED_TOPICS)
    )
    runtime_required_topics: list[str] = []
    if expected_runtime_adapter is not None:
        required_by_adapter = contract.get(
            "required_nonempty_topics_by_runtime_adapter",
            {},
        )
        if isinstance(required_by_adapter, Mapping):
            runtime_required_topics = [
                str(topic)
                for topic in required_by_adapter.get(
                    expected_runtime_adapter, ()
                )
            ]
    summary["runtime_adapter_required_nonempty_topics"] = (
        runtime_required_topics
    )
    required_topics.extend(runtime_required_topics)
    required_topics = list(dict.fromkeys(str(topic) for topic in required_topics))
    summary["required_nonempty_topics"] = required_topics
    for topic in required_topics:
        if topic_counts.get(str(topic), 0) <= 0:
            errors.append(f"core bag required topic is empty or absent: {topic}")

    relative_paths = information.get("relative_file_paths")
    if not isinstance(relative_paths, list) or not relative_paths:
        errors.append("core bag metadata names no MCAP files")
        relative_paths = []
    elif len(relative_paths) != len(set(relative_paths)):
        errors.append("core bag metadata contains duplicate MCAP paths")
    file_records = information.get("files")
    recorded_file_paths: list[str] = []
    recorded_file_count = 0
    if not isinstance(file_records, list) or not file_records:
        errors.append("core bag metadata has no per-file records")
    else:
        for record in file_records:
            if not isinstance(record, Mapping):
                errors.append("core bag file record is not a mapping")
                continue
            path = record.get("path")
            count = record.get("message_count")
            if not isinstance(path, str):
                errors.append("core bag file record has an invalid path")
                continue
            if type(count) is not int or count < 0:
                errors.append(f"core bag file {path} has an invalid count")
                continue
            recorded_file_paths.append(path)
            recorded_file_count += count
    if recorded_file_paths != relative_paths:
        errors.append("core bag per-file paths disagree with relative_file_paths")
    if type(message_count) is int and recorded_file_count != message_count:
        errors.append("core bag per-file counts do not sum to message_count")
    metadata_key = metadata_path.relative_to(output_dir).as_posix()
    try:
        files[metadata_key] = {
            "sha256": sha256_file(metadata_path),
            "size_bytes": metadata_path.stat().st_size,
        }
    except OSError as error:
        errors.append(f"cannot hash core bag metadata: {error}")
    verified_mcap_paths = 0
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
            if size_bytes < 2 * len(MCAP_MAGIC):
                errors.append(f"core bag MCAP file is too short: {relative}")
            else:
                with resolved.open("rb") as stream:
                    leading_magic = stream.read(len(MCAP_MAGIC))
                    stream.seek(-len(MCAP_MAGIC), os.SEEK_END)
                    trailing_magic = stream.read(len(MCAP_MAGIC))
                if leading_magic != MCAP_MAGIC or trailing_magic != MCAP_MAGIC:
                    errors.append(f"core bag MCAP framing is invalid: {relative}")
                else:
                    verified_mcap_paths += 1
            files[key] = {
                "sha256": sha256_file(resolved),
                "size_bytes": size_bytes,
            }
        except OSError as error:
            errors.append(f"cannot hash core bag MCAP file {relative}: {error}")
    if relative_paths and verified_mcap_paths == len(relative_paths):
        reader_counts, reader_types, reader_errors = _read_core_bag_to_eof(
            bag_dir, str(contract.get("storage_id", ""))
        )
        errors.extend(reader_errors)
        summary["reader_message_count"] = sum(reader_counts.values())
        if not reader_errors:
            summary["reader_verified"] = True
            all_count_topics = set(topic_counts) | set(reader_counts)
            for topic in sorted(all_count_topics):
                if reader_counts.get(topic, 0) != topic_counts.get(topic, 0):
                    errors.append(
                        f"rosbag2 reader count disagrees with metadata: {topic}"
                    )
            all_type_topics = set(topic_types) | set(reader_types)
            for topic in sorted(all_type_topics):
                if reader_types.get(topic) != topic_types.get(topic):
                    errors.append(
                        f"rosbag2 reader type disagrees with metadata: {topic}"
                    )
    summary["complete"] = not errors
    return files, errors, summary


def validate_completed_artifacts(
    output_dir: Path,
    *,
    expected_experiment_budget: Mapping[str, float | int] | None = None,
    expected_recording_contract: Mapping[str, Any] | None = None,
    expected_runtime_adapter: str | None = None,
) -> dict[str, Any]:
    """Audit the minimum policy/evaluator evidence required for completion."""
    errors: list[str] = []
    if (
        expected_runtime_adapter is not None
        and expected_runtime_adapter not in RUNTIME_ADAPTERS
    ):
        errors.append(
            "unsupported expected runtime_adapter: "
            f"{expected_runtime_adapter!r}"
        )
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
    if (
        policy_manifest is not None
        and expected_runtime_adapter is not None
        and policy_manifest.get("runtime_adapter") != expected_runtime_adapter
    ):
        errors.append(
            "policy_manifest.json: runtime_adapter disagrees with the frozen method"
        )
    if (
        policy_manifest is not None
        and expected_runtime_adapter == "frontier_mrtsp_dp_external"
        and policy_manifest.get("topological_visit_contract") != (
            EXTERNAL_TOPOLOGICAL_VISIT_CONTRACT
        )
    ):
        errors.append(
            "policy_manifest.json: external topological visit contract "
            "is absent or unsupported"
        )
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
    if evaluator_manifest is not None:
        evaluator_parameters = evaluator_manifest.get("parameters")
        if not isinstance(evaluator_parameters, Mapping):
            errors.append(
                "evaluation_manifest.json: parameters must be an object"
            )
        elif evaluator_parameters.get("use_sim_time") is not True:
            errors.append(
                "evaluation_manifest.json: use_sim_time must be true"
            )
        if expected_runtime_adapter == "frontier_mrtsp_dp_external":
            coverage = evaluator_manifest.get("coverage_endpoints")
            contracts = (
                coverage.get("accepted_topological_visit_contracts")
                if isinstance(coverage, Mapping) else None
            )
            if (
                not isinstance(contracts, list)
                or EXTERNAL_TOPOLOGICAL_VISIT_CONTRACT not in contracts
            ):
                errors.append(
                    "evaluation_manifest.json: evaluator does not attest "
                    "the external topological visit contract"
                )

    core_bag = {"required": False, "complete": None}
    if expected_recording_contract is not None:
        bag_files, bag_errors, core_bag = _core_bag_artifacts(
            output_dir,
            expected_recording_contract,
            expected_runtime_adapter=expected_runtime_adapter,
        )
        files.update(bag_files)
        errors.extend(bag_errors)

    launch_runtime_errors: list[str] = []
    launch_log_path = output_dir / "launch.log"
    if launch_log_path.is_file():
        launch_runtime_errors = _launch_log_runtime_errors(
            launch_log_path, expected_runtime_adapter
        )
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
    if policy_records and observed_records and policy_records != observed_records:
        errors.append(
            "evaluation_observed_policy_trace.jsonl: records disagree with "
            "policy_trace.jsonl"
        )
    metric_records = jsonl.get("evaluation_metrics.jsonl", [])
    ingested_terminal = any(
        record.get("event") == "policy_trace_ingested"
        and isinstance(record.get("payload"), dict)
        and record["payload"].get("event") == "session_finished"
        for record in metric_records
    )
    terminal_snapshot = any(
        record.get("event") == "metrics_snapshot"
        and isinstance(record.get("payload"), dict)
        and record["payload"].get("reason") == "policy_session_finished"
        for record in metric_records
    )
    settled_snapshots = [
        record["payload"]
        for record in metric_records
        if record.get("event") == "metrics_snapshot"
        and isinstance(record.get("payload"), dict)
        and record["payload"].get("reason")
        == FINAL_EVALUATOR_SNAPSHOT_REASON
    ]
    settled_snapshot = settled_snapshots[-1] if settled_snapshots else None
    if metric_records and not ingested_terminal:
        errors.append(
            "evaluation_metrics.jsonl: evaluator did not ingest session_finished"
        )
    if metric_records and not terminal_snapshot:
        errors.append(
            "evaluation_metrics.jsonl: policy_session_finished snapshot is absent"
        )
    if metric_records and settled_snapshot is None:
        errors.append(
            "evaluation_metrics.jsonl: final policy_session_settled snapshot is absent"
        )
    if settled_snapshot is not None:
        diagnostics = settled_snapshot.get("diagnostics")
        ground_truth = settled_snapshot.get("ground_truth_motion")
        if not isinstance(diagnostics, Mapping):
            errors.append(
                "evaluation_metrics.jsonl: settled diagnostics are absent"
            )
        else:
            if diagnostics.get("ate_pending_sample_count") != 0:
                errors.append(
                    "evaluation_metrics.jsonl: settled ATE queue is not empty"
                )
            if diagnostics.get("ate_settlement_pending") is not False:
                errors.append(
                    "evaluation_metrics.jsonl: ATE settlement remains pending"
                )
            for field in (
                "trace_rejection_count",
                "topology_trace_rejection_count",
            ):
                value = diagnostics.get(field)
                require_field = (
                    isinstance(policy_manifest, Mapping)
                    and policy_manifest.get("topological_visit_contract")
                    == EXTERNAL_TOPOLOGICAL_VISIT_CONTRACT
                )
                if require_field and type(value) is not int:
                    errors.append(
                        "evaluation_metrics.jsonl: settled evaluator lacks "
                        f"integer {field}"
                    )
                elif value not in (None, 0):
                    errors.append(
                        "evaluation_metrics.jsonl: settled evaluator has "
                        f"nonzero {field}"
                    )
        if not isinstance(ground_truth, Mapping):
            errors.append(
                "evaluation_metrics.jsonl: settled ground-truth metrics are absent"
            )
        elif ground_truth.get("ate_pending_sample_count") != 0:
            errors.append(
                "evaluation_metrics.jsonl: settled ground-truth ATE queue is not empty"
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
            "evaluator_terminal_snapshot": terminal_snapshot,
            "evaluator_settled_snapshot": settled_snapshot is not None,
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
    runtime_adapter = str(row.get("runtime_adapter", ""))
    if runtime_adapter not in RUNTIME_ADAPTERS:
        raise RunnerError(
            f"schedule row has unsupported runtime_adapter: {runtime_adapter!r}"
        )
    if str(method_records[0].get("runtime_adapter", "")) != runtime_adapter:
        raise RunnerError(
            "schedule runtime_adapter disagrees with frozen method config"
        )

    bundle_path = _resolve_inside(root, row.get("world_bundle", ""), "world bundle")
    if not bundle_path.is_dir():
        raise RunnerError(f"frozen world bundle is missing: {bundle_path}")
    try:
        current_bundle_sha = sha256_tree(root, [bundle_path])
    except (ScheduleError, OSError) as error:
        raise RunnerError(f"cannot verify frozen world bundle: {error}") from error
    if current_bundle_sha != row.get("world_bundle_sha256"):
        raise RunnerError("frozen world bundle changed after schedule freeze")


def _method_runtime_contract(
    *,
    root: Path,
    manifest: Mapping[str, Any],
    row: Mapping[str, str],
    fixed_args: Mapping[str, str],
    column_args: Mapping[str, str],
) -> str:
    """Cross-check method YAML, freeze record, CSV, and launch selection."""
    inputs = manifest.get("inputs")
    methods = inputs.get("methods") if isinstance(inputs, Mapping) else None
    if not isinstance(methods, list):
        raise RunnerError("freeze manifest has no method records")
    records = [
        item for item in methods
        if isinstance(item, Mapping)
        and str(item.get("method")) == row.get("method")
    ]
    if len(records) != 1:
        raise RunnerError("schedule method does not match one runtime record")
    record = records[0]
    path = _resolve_inside(
        root, str(record.get("path", "")), "method config"
    )
    config = _load_yaml_mapping(path, "method config")
    if config.get("schema") != "sstg_system_sim_method/v1":
        raise RunnerError("method config schema is not sstg_system_sim_method/v1")
    expected_fields = {
        "method": "method",
        "runtime_adapter": "runtime_adapter",
        "strategy": "strategy",
        "coverage_objective": "coverage_objective",
    }
    for config_field, row_field in expected_fields.items():
        if str(config.get(config_field, "")) != str(row.get(row_field, "")):
            raise RunnerError(
                f"method config {config_field} disagrees with schedule row"
            )
    for field, default in METHOD_POLICY_DEFAULTS.items():
        try:
            config_value = float(config.get(field, default))
            row_value = float(row.get(field, ""))
        except (TypeError, ValueError) as error:
            raise RunnerError(
                f"method policy field {field} must be numeric"
            ) from error
        if (
            not math.isfinite(config_value)
            or not math.isfinite(row_value)
            or config_value < 0.0
            or row_value < 0.0
            or not math.isclose(
                config_value, row_value, rel_tol=0.0, abs_tol=1e-12
            )
        ):
            raise RunnerError(
                f"method config {field} disagrees with schedule row"
            )
    runtime_adapter = str(config.get("runtime_adapter", ""))
    if runtime_adapter not in RUNTIME_ADAPTERS:
        raise RunnerError(
            f"method config has unsupported runtime_adapter: {runtime_adapter!r}"
        )
    if str(record.get("runtime_adapter", "")) != runtime_adapter:
        raise RunnerError(
            "frozen method record runtime_adapter disagrees with method config"
        )
    for argument, column in {
        "runtime_adapter": "runtime_adapter",
        "strategy": "strategy",
        "coverage_objective": "coverage_objective",
        **{name: name for name in METHOD_POLICY_DEFAULTS},
    }.items():
        if argument in fixed_args or column_args.get(argument) != column:
            raise RunnerError(
                f"launch contract must pass {argument} from {column}"
            )
    return runtime_adapter


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


def _normalized_ros_gz_bridge_contract(
    value: Any, *, label: str
) -> dict[str, Any]:
    try:
        return validate_ros_gz_bridge_contract(value, label=label)
    except ScheduleError as error:
        raise RunnerError(str(error)) from error


def _ros_gz_bridge_contract(
    *, root: Path, manifest: Mapping[str, Any]
) -> dict[str, Any]:
    """Cross-check the bridge requirement at every frozen provenance layer."""
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
    declared = _normalized_ros_gz_bridge_contract(
        shared_stack.get("ros_gz_bridge"),
        label="shared_stack.ros_gz_bridge",
    )
    recorded = _normalized_ros_gz_bridge_contract(
        shared_record.get("ros_gz_bridge_contract"),
        label="freeze manifest shared_stack.ros_gz_bridge_contract",
    )
    frozen = _normalized_ros_gz_bridge_contract(
        manifest.get("ros_gz_bridge_contract"),
        label="freeze manifest ros_gz_bridge_contract",
    )
    if recorded != declared or frozen != declared:
        raise RunnerError(
            "freeze manifest ros_gz_bridge contract disagrees with its input"
        )
    return frozen


def _normalized_ros_middleware_contract(
    value: Any, *, label: str
) -> dict[str, Any]:
    try:
        return validate_ros_middleware_contract(value, label=label)
    except ScheduleError as error:
        raise RunnerError(str(error)) from error


def _ros_middleware_contract(
    *, root: Path, manifest: Mapping[str, Any]
) -> dict[str, Any]:
    """Cross-check the RMW requirement at every frozen provenance layer."""
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
    declared = _normalized_ros_middleware_contract(
        shared_stack.get("ros_middleware"),
        label="shared_stack.ros_middleware",
    )
    recorded = _normalized_ros_middleware_contract(
        shared_record.get("ros_middleware_contract"),
        label="freeze manifest shared_stack.ros_middleware_contract",
    )
    frozen = _normalized_ros_middleware_contract(
        manifest.get("ros_middleware_contract"),
        label="freeze manifest ros_middleware_contract",
    )
    if recorded != declared or frozen != declared:
        raise RunnerError(
            "freeze manifest ROS middleware contract disagrees with its input"
        )
    return frozen


def _runtime_command(
    command: Sequence[str], *, label: str
) -> subprocess.CompletedProcess[str]:
    try:
        completed = subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RunnerError(f"could not verify {label}: {error}") from error
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise RunnerError(
            f"could not verify {label} (exit {completed.returncode}): {detail}"
        )
    return completed


def _relative_runtime_path(root: Path, path: Path) -> str:
    try:
        return path.relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def _runtime_apt_version(package: str) -> str:
    result = _runtime_command(
        ["dpkg-query", "-W", "-f=${Version}\\n", package],
        label=f"apt package version for {package}",
    )
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if len(lines) != 1:
        observed = result.stdout.strip() or "<empty>"
        raise RunnerError(
            f"apt package {package} returned an ambiguous version: {observed}"
        )
    return lines[0]


def verify_ros_middleware_runtime(
    root: Path, contract: Mapping[str, Any]
) -> dict[str, Any]:
    """Attest the RMW selection and reject undeclared ROS underlays."""
    root = root.resolve()
    expected = _normalized_ros_middleware_contract(
        contract, label="runtime ROS middleware contract"
    )
    implementation = os.environ.get("RMW_IMPLEMENTATION", "")
    if implementation != expected["implementation"]:
        raise RunnerError(
            "RMW_IMPLEMENTATION must be "
            f"{expected['implementation']}, observed: "
            f"{implementation or '<unset>'}"
        )
    mismatched_required = [
        f"{name}={os.environ.get(name, '<unset>')}"
        for name, value in expected["required_environment"].items()
        if os.environ.get(name) != value
    ]
    if mismatched_required:
        raise RunnerError(
            "required ROS environment does not match the frozen contract: "
            + ", ".join(mismatched_required)
        )
    configured_forbidden = [
        name
        for name in expected["forbidden_environment"]
        if name in os.environ
    ]
    configured_forbidden.extend(
        name
        for name in sorted(os.environ)
        if any(
            name.startswith(prefix)
            for prefix in expected["forbidden_environment_prefixes"]
        )
    )
    configured_forbidden = sorted(set(configured_forbidden))
    if configured_forbidden:
        raise RunnerError(
            "custom middleware environment is forbidden: "
            + ", ".join(configured_forbidden)
        )

    allowed_roots = [
        (
            (root / Path(value)).resolve()
            if not Path(value).is_absolute()
            else Path(value).resolve()
        )
        for value in expected["allowed_prefix_roots"]
    ]
    additional_roots = {
        name: [
            (
                (root / Path(value)).resolve()
                if not Path(value).is_absolute()
                else Path(value).resolve()
            )
            for value in values
        ]
        for name, values in expected[
            "additional_allowed_prefix_roots"
        ].items()
    }
    prefix_environment: dict[str, list[str]] = {}
    outside_prefixes: list[str] = []
    for name in expected["prefix_path_environment"]:
        if name not in os.environ:
            prefix_environment[name] = []
            continue
        paths = os.environ[name].split(os.pathsep)
        if any(not value for value in paths):
            raise RunnerError(
                f"ROS path environment contains an empty segment: {name}"
            )
        prefix_environment[name] = paths
        name_allowed_roots = allowed_roots + additional_roots.get(name, [])
        for value in paths:
            if not Path(value).is_absolute():
                outside_prefixes.append(f"{name}={value} (relative)")
                continue
            resolved = Path(value).resolve()
            if not any(
                resolved.is_relative_to(allowed)
                for allowed in name_allowed_roots
            ):
                outside_prefixes.append(f"{name}={value}")
    if outside_prefixes:
        raise RunnerError(
            "ROS environment contains undeclared underlay paths: "
            + ", ".join(outside_prefixes)
        )

    prefix = Path(expected["required_prefix"]).resolve()
    prefix_result = _runtime_command(
        ["ros2", "pkg", "prefix", expected["package"]],
        label="ROS middleware package prefix",
    )
    prefix_lines = [
        line.strip()
        for line in prefix_result.stdout.splitlines()
        if line.strip()
    ]
    if len(prefix_lines) != 1 or Path(prefix_lines[0]).resolve() != prefix:
        observed = prefix_result.stdout.strip() or "<empty>"
        raise RunnerError(
            f"{expected['package']} must resolve to {prefix}, observed: {observed}"
        )

    package_xml = prefix / "share" / expected["package"] / "package.xml"
    try:
        package_root = ElementTree.parse(package_xml).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise RunnerError(
            f"cannot parse middleware package metadata: {error}"
        ) from error
    version = (package_root.findtext("version") or "").strip()
    if version != expected["required_version"]:
        raise RunnerError(
            f"{expected['package']} version must be {expected['required_version']}, "
            f"observed: {version or '<empty>'}"
        )
    apt_version = _runtime_apt_version(expected["apt_package"])
    if apt_version != expected["apt_version_observed"]:
        raise RunnerError(
            f"{expected['apt_package']} version must be "
            f"{expected['apt_version_observed']}, observed: "
            f"{apt_version or '<empty>'}"
        )

    library = prefix / "lib" / "librmw_fastrtps_cpp.so"
    if not library.is_file():
        raise RunnerError(f"middleware library is missing: {library}")
    library_sha256 = sha256_file(library)
    if library_sha256 != expected["required_library_sha256"]:
        raise RunnerError(
            "middleware library hash must be "
            f"{expected['required_library_sha256']}, observed: {library_sha256}"
        )
    ldd_result = _runtime_command(["ldd", str(library)], label="middleware linkage")
    dependency_patterns = {
        "rmw_fastrtps_shared_cpp": r"librmw_fastrtps_shared_cpp\.so(?:\.\S+)?",
        "fastrtps": r"libfastrtps\.so(?:\.\S+)?",
        "fastcdr": r"libfastcdr\.so(?:\.\S+)?",
    }
    linked_dependencies: dict[str, dict[str, str]] = {}
    for label, pattern in dependency_patterns.items():
        dependency_contract = expected["required_linked_dependencies"][label]
        match = re.search(
            rf"^\s*{pattern}\s*=>\s*(\S+)",
            ldd_result.stdout,
            flags=re.MULTILINE,
        )
        if match is None:
            raise RunnerError(f"middleware linkage does not list {label}")
        linked = Path(match.group(1)).resolve()
        if not linked.is_relative_to(prefix):
            raise RunnerError(
                f"middleware dependency {label} resolves outside {prefix}: {linked}"
            )
        required_library = Path(
            dependency_contract["required_library"]
        ).resolve()
        if linked != required_library:
            raise RunnerError(
                f"middleware dependency {label} must resolve to "
                f"{required_library}, observed: {linked}"
            )
        if not linked.is_file():
            raise RunnerError(f"middleware dependency {label} is missing: {linked}")
        dependency_sha256 = sha256_file(linked)
        if dependency_sha256 != dependency_contract["required_sha256"]:
            raise RunnerError(
                f"middleware dependency {label} hash must be "
                f"{dependency_contract['required_sha256']}, observed: "
                f"{dependency_sha256}"
            )
        dependency_apt_version = _runtime_apt_version(
            dependency_contract["apt_package"]
        )
        if dependency_apt_version != dependency_contract["apt_version"]:
            raise RunnerError(
                f"{dependency_contract['apt_package']} version must be "
                f"{dependency_contract['apt_version']}, observed: "
                f"{dependency_apt_version}"
            )
        linked_dependencies[label] = {
            "path": linked.as_posix(),
            "sha256": dependency_sha256,
            "apt_package": dependency_contract["apt_package"],
            "apt_version": dependency_apt_version,
        }

    return {
        "implementation": implementation,
        "package": expected["package"],
        "version": version,
        "apt_package": expected["apt_package"],
        "apt_version": apt_version,
        "prefix": prefix.as_posix(),
        "library": {
            "path": library.as_posix(),
            "sha256": library_sha256,
        },
        "linked_dependencies": linked_dependencies,
        "environment": {
            "forbidden_variables_set": [],
            "required_variables": dict(expected["required_environment"]),
            "allowed_prefix_roots": [path.as_posix() for path in allowed_roots],
            "additional_allowed_prefix_roots": {
                name: [path.as_posix() for path in paths]
                for name, paths in additional_roots.items()
            },
            "prefix_paths": prefix_environment,
        },
    }


def verify_ros_gz_bridge_runtime(
    root: Path, contract: Mapping[str, Any]
) -> dict[str, Any]:
    """Attest that ROS will execute the pinned, bounds-fixed bridge overlay."""
    root = root.resolve()
    expected = _normalized_ros_gz_bridge_contract(
        contract, label="runtime ros_gz_bridge contract"
    )
    prefix = _resolve_inside(root, expected["required_prefix"], "bridge prefix")
    checkout = _resolve_inside(
        root, expected["source_checkout"], "bridge source checkout"
    )

    prefix_result = _runtime_command(
        ["ros2", "pkg", "prefix", expected["package"]],
        label="ros_gz_bridge package prefix",
    )
    prefix_lines = [
        line.strip()
        for line in prefix_result.stdout.splitlines()
        if line.strip()
    ]
    if len(prefix_lines) != 1 or Path(prefix_lines[0]).resolve() != prefix:
        observed = prefix_result.stdout.strip() or "<empty>"
        raise RunnerError(
            f"ros_gz_bridge must resolve to {prefix}, observed: {observed}"
        )

    package_xml = prefix / "share" / expected["package"] / "package.xml"
    try:
        package_root = ElementTree.parse(package_xml).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise RunnerError(f"cannot parse bridge package metadata: {error}") from error
    version = (package_root.findtext("version") or "").strip()
    if version != expected["required_version"]:
        raise RunnerError(
            "ros_gz_bridge version must be "
            f"{expected['required_version']}, observed: {version or '<empty>'}"
        )

    if not checkout.is_dir():
        raise RunnerError(f"bridge source checkout is missing: {checkout}")
    head = _runtime_command(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"],
        label="ros_gz_bridge source commit",
    ).stdout.strip()
    if head != expected["source_commit"]:
        raise RunnerError(
            f"ros_gz_bridge source commit must be {expected['source_commit']}, "
            f"observed: {head or '<empty>'}"
        )
    tag_commit = _runtime_command(
        ["git", "-C", str(checkout), "rev-list", "-n", "1", expected["source_tag"]],
        label="ros_gz_bridge source tag",
    ).stdout.strip()
    if tag_commit != expected["source_commit"]:
        raise RunnerError(
            f"ros_gz_bridge tag {expected['source_tag']} does not resolve to the "
            "required source commit"
        )
    source_status = _runtime_command(
        [
            "git",
            "-C",
            str(checkout),
            "status",
            "--porcelain",
            "--untracked-files=all",
        ],
        label="ros_gz_bridge source status",
    ).stdout.strip()
    if source_status:
        raise RunnerError("ros_gz_bridge source checkout has local changes")
    _runtime_command(
        [
            "git",
            "-C",
            str(checkout),
            "merge-base",
            "--is-ancestor",
            expected["required_fix_commit"],
            "HEAD",
        ],
        label="ros_gz_bridge required fix ancestry",
    )

    executable = prefix / "lib" / expected["package"] / "parameter_bridge"
    library = prefix / "lib" / "libros_gz_bridge.so"
    for path, label in (
        (executable, "parameter_bridge"),
        (library, "bridge library"),
    ):
        if not path.is_file():
            raise RunnerError(f"{label} is missing from the required overlay: {path}")
    ldd_result = _runtime_command(["ldd", str(executable)], label="bridge linkage")
    linked_match = re.search(
        r"^\s*libros_gz_bridge\.so\s*=>\s*(\S+)",
        ldd_result.stdout,
        flags=re.MULTILINE,
    )
    if linked_match is None:
        raise RunnerError("parameter_bridge linkage does not list libros_gz_bridge.so")
    linked_library = Path(linked_match.group(1))
    if linked_library.resolve() != library.resolve():
        raise RunnerError(
            f"parameter_bridge links {linked_library}, expected overlay {library}"
        )

    return {
        "package": expected["package"],
        "version": version,
        "prefix": _relative_runtime_path(root, prefix),
        "source_checkout": _relative_runtime_path(root, checkout),
        "source_commit": head,
        "source_tag": expected["source_tag"],
        "source_tag_commit": tag_commit,
        "required_fix_commit": expected["required_fix_commit"],
        "required_fix_ancestor": True,
        "source_clean": True,
        "parameter_bridge": {
            "path": _relative_runtime_path(root, executable),
            "sha256": sha256_file(executable),
        },
        "library": {
            "path": _relative_runtime_path(root, library),
            "sha256": sha256_file(library),
        },
        "linked_library": _relative_runtime_path(root, linked_library),
    }


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
    _method_runtime_contract(
        root=root,
        manifest=manifest,
        row=row,
        fixed_args=fixed_args,
        column_args=column_args,
    )
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
    ros_gz_bridge_contract = _ros_gz_bridge_contract(
        root=root,
        manifest=manifest,
    )
    ros_middleware_contract = _ros_middleware_contract(
        root=root,
        manifest=manifest,
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
        ros_gz_bridge_contract=ros_gz_bridge_contract,
        ros_middleware_contract=ros_middleware_contract,
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
        "ros_gz_bridge_contract": (
            dict(plan.ros_gz_bridge_contract)
            if plan.ros_gz_bridge_contract is not None
            else None
        ),
        "ros_middleware_contract": (
            dict(plan.ros_middleware_contract)
            if plan.ros_middleware_contract is not None
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
                "runtime_adapter",
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


def reserve_run_output(
    plan: RunPlan,
    *,
    ros_gz_bridge_runtime: Mapping[str, Any] | None = None,
    ros_middleware_runtime: Mapping[str, Any] | None = None,
) -> Path:
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
    runtime_updates: dict[str, Any] = {}
    if ros_gz_bridge_runtime is not None:
        runtime_updates["ros_gz_bridge_runtime"] = dict(ros_gz_bridge_runtime)
    if ros_middleware_runtime is not None:
        runtime_updates["ros_middleware_runtime"] = dict(ros_middleware_runtime)
    _write_manifest(
        manifest_path,
        _manifest_value(
            plan,
            status="reserved",
            reserved_at_utc=_utc_now(),
            **runtime_updates,
        ),
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
    metrics_path: Path | None = None,
    evaluator_flush_s: float = DEFAULT_EVALUATOR_SETTLEMENT_S,
    poll_interval_s: float = 0.2,
    sigint_grace_s: float = DEFAULT_SIGINT_GRACE_S,
    term_grace_s: float = DEFAULT_TERM_GRACE_S,
    clock: Any = time.monotonic,
    sleeper: Any = time.sleep,
    shutdown: Any = shutdown_process_group,
) -> ProcessOutcome:
    """Wait for policy completion and the evaluator's settled ATE snapshot."""
    _validate_supervision_parameters(
        wall_timeout_s=wall_timeout_s,
        evaluator_flush_s=evaluator_flush_s,
        poll_interval_s=poll_interval_s,
        sigint_grace_s=sigint_grace_s,
        term_grace_s=term_grace_s,
    )

    started = clock()
    wall_deadline = started + wall_timeout_s
    if metrics_path is None:
        metrics_path = trace_path.with_name("evaluation_metrics.jsonl")
    terminal_observed = False
    shutdown_signals: tuple[str, ...] = ()
    status = "early_exit"
    try:
        while True:
            if jsonl_contains_event(trace_path, "session_finished"):
                terminal_observed = True
                flush_deadline = min(clock() + evaluator_flush_s, wall_deadline)
                while clock() < flush_deadline and process.poll() is None:
                    if jsonl_contains_snapshot_reason(
                        metrics_path, FINAL_EVALUATOR_SNAPSHOT_REASON
                    ):
                        break
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
    evaluator_flush_s: float = DEFAULT_EVALUATOR_SETTLEMENT_S,
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
    ros_middleware_runtime = (
        verify_ros_middleware_runtime(plan.root, plan.ros_middleware_contract)
        if plan.ros_middleware_contract is not None
        else None
    )
    ros_gz_bridge_runtime = (
        verify_ros_gz_bridge_runtime(plan.root, plan.ros_gz_bridge_contract)
        if plan.ros_gz_bridge_contract is not None
        else None
    )
    runtime_manifest_update: dict[str, Any] = {}
    if ros_middleware_runtime is not None:
        runtime_manifest_update["ros_middleware_runtime"] = dict(
            ros_middleware_runtime
        )
    if ros_gz_bridge_runtime is not None:
        runtime_manifest_update["ros_gz_bridge_runtime"] = dict(
            ros_gz_bridge_runtime
        )
    manifest_path = reserve_run_output(
        plan,
        ros_gz_bridge_runtime=ros_gz_bridge_runtime,
        ros_middleware_runtime=ros_middleware_runtime,
    )
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
            **runtime_manifest_update,
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
                    **runtime_manifest_update,
                ),
            )
            outcome = supervise_process(
                process,
                trace_path=plan.output_dir / "policy_trace.jsonl",
                metrics_path=plan.output_dir / "evaluation_metrics.jsonl",
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
            expected_runtime_adapter=plan.schedule_row["runtime_adapter"],
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
                **runtime_manifest_update,
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
            expected_runtime_adapter=plan.schedule_row["runtime_adapter"],
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
                **runtime_manifest_update,
            ),
        )
        raise RunnerError(f"could not invoke ros2 launch: {error}") from error

    artifact_audit = validate_completed_artifacts(
        plan.output_dir,
        expected_experiment_budget=plan.experiment_budget,
        expected_recording_contract=plan.recording_contract,
        expected_runtime_adapter=plan.schedule_row["runtime_adapter"],
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
            **runtime_manifest_update,
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
        default=DEFAULT_EVALUATOR_SETTLEMENT_S,
        help=(
            "maximum wait for policy_session_settled after session_finished "
            "before group shutdown (default: 5)"
        ),
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
                "ros_gz_bridge_contract": (
                    dict(plan.ros_gz_bridge_contract)
                    if plan.ros_gz_bridge_contract is not None
                    else None
                ),
                "ros_middleware_contract": (
                    dict(plan.ros_middleware_contract)
                    if plan.ros_middleware_contract is not None
                    else None
                ),
                "supervision": {
                    "wall_timeout_s": args.wall_timeout_s,
                    "evaluator_flush_s": args.evaluator_flush_s,
                    "sigint_grace_s": args.sigint_grace_s,
                    "term_grace_s": args.term_grace_s,
                    "completion_event": "policy_trace.jsonl:session_finished",
                    "evaluator_settlement_event": (
                        "evaluation_metrics.jsonl:policy_session_settled"
                    ),
                },
                "filesystem_mutated": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
