#!/usr/bin/env python3
"""Freeze a matched-block ROS 2/Gazebo experiment schedule without running it.

The generated schedule pairs every selected method within the same world,
start, condition, and replicate seed.  Method order is independently and
deterministically shuffled inside each block.  A companion manifest records
the exact source, configuration, and world hashes used to create the CSV.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT_ROOT = Path("experiments/system_sim")
SCHEDULE_SCHEMA = "sstg_system_sim_run_schedule/v2"
FREEZE_SCHEMA = "sstg_system_sim_schedule_freeze/v2"
ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
EXPERIMENT_BUDGET_FIELDS = (
    "max_duration_s",
    "max_distance_m",
    "max_decisions",
    "goal_timeout_s",
)
METHOD_POLICY_DEFAULTS = {
    "clearance_weight": 1.5,
    "travel_cost_weight": 0.60,
}
GAZEBO_SEED_MIN = 1
GAZEBO_SEED_MAX = 0x7FFFFFFF
SEED_LAUNCH_ARGUMENTS = ("policy_seed", "simulation_seed")
RUNTIME_ADAPTERS = frozenset({
    "sstg_policy",
    "frontier_mrtsp_dp_external",
})
RUNTIME_ADAPTER_METHOD_IDS = {
    "frontier_mrtsp_dp_external": frozenset({
        "frontier_mrtsp_dp_external"
    }),
}
ROS_GZ_BRIDGE_CONTRACT = {
    "package": "ros_gz_bridge",
    "required_version": "1.0.23",
    "repository": "https://github.com/gazebosim/ros_gz",
    "source_tag": "1.0.23",
    "source_commit": "ec3a555b540ac492882d587a09752eb2eeeee3cd",
    "required_fix_commit": "4c6cb80bb30fc0871bbd5ec95761272ce49a150d",
    "source_checkout": "ros2_ws/src/third_party/ros_gz",
    "required_prefix": "ros2_ws/install/ros_gz_bridge",
    "system_apt_version_observed": "1.0.22-1noble.20260615.142443",
    "system_apt_eligible": False,
}
ROS_MIDDLEWARE_CONTRACT = {
    "implementation": "rmw_fastrtps_cpp",
    "package": "rmw_fastrtps_cpp",
    "required_version": "8.4.4",
    "required_prefix": "/opt/ros/jazzy",
    "required_library_sha256": (
        "046375a1ef195094abb57c832b275c385261052cfed3ee044cce70e25a42cef3"
    ),
    "apt_package": "ros-jazzy-rmw-fastrtps-cpp",
    "apt_version_observed": "8.4.4-1noble.20260615.124621",
    "required_linked_dependencies": {
        "rmw_fastrtps_shared_cpp": {
            "apt_package": "ros-jazzy-rmw-fastrtps-shared-cpp",
            "apt_version": "8.4.4-1noble.20260615.124045",
            "required_library": (
                "/opt/ros/jazzy/lib/librmw_fastrtps_shared_cpp.so"
            ),
            "required_sha256": (
                "c1abaceb3433fd4f20b1f5a5fa6686d285659bce8a2bb81ec2d61954a6d490ec"
            ),
        },
        "fastrtps": {
            "apt_package": "ros-jazzy-fastrtps",
            "apt_version": "2.14.6-1noble.20260303.233638",
            "required_library": "/opt/ros/jazzy/lib/libfastrtps.so.2.14.6",
            "required_sha256": (
                "8d39de86a55a92e1be92640a22e6322f099227930d4fc98bd821b6effd7a3eaa"
            ),
        },
        "fastcdr": {
            "apt_package": "ros-jazzy-fastcdr",
            "apt_version": "2.2.7-1noble.20260225.051855",
            "required_library": "/opt/ros/jazzy/lib/libfastcdr.so.2.2.7",
            "required_sha256": (
                "0eeb1f3d1859db07e7551be9df814053a8a4805c9feb9cb990363e43ac45cd69"
            ),
        },
    },
    "custom_underlays_eligible": False,
    "required_environment": {
        "ROS_DISTRO": "jazzy",
        "ROS_DOMAIN_ID": "42",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "SKIP_DEFAULT_XML": "1",
    },
    "forbidden_environment": [
        "CYCLONEDDS_URI",
        "ROS_DISCOVERY_SERVER",
        "ROS_LOCALHOST_ONLY",
        "ROS_STATIC_PEERS",
        "ROS_SUPER_CLIENT",
    ],
    "forbidden_environment_prefixes": [
        "FASTDDS_",
        "FASTRTPS_",
        "RMW_FASTRTPS_",
        "ROS_SECURITY_",
    ],
    "prefix_path_environment": [
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "LD_LIBRARY_PATH",
        "PATH",
        "PKG_CONFIG_PATH",
        "PYTHONPATH",
    ],
    "allowed_prefix_roots": [
        "ros2_ws/install",
        "/opt/ros/jazzy",
    ],
    "additional_allowed_prefix_roots": {
        "PATH": ["/usr/bin", "/usr/sbin"],
        "PYTHONPATH": ["ros2_ws/build"],
    },
}
CORE_BAG_TOPICS = (
    "/clock",
    "/tf",
    "/tf_static",
    "/scan",
    "/imu",
    "/joint_states",
    "/odom",
    "/map",
    "/cmd_vel",
    "/plan",
    "/navigate_to_pose/_action/feedback",
    "/navigate_to_pose/_action/status",
    "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/feedback",
    "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/status",
    "/baseline/frontier_mrtsp_dp/exploration_complete",
    "/explore/frontiers",
    "/policy/decision_trace",
    "/policy/status",
    "/policy/candidates",
    "/evaluation/ground_truth_odom",
    "/evaluation/world_stats",
    "/evaluation/metrics",
    "/evaluation/status",
    "/task_camera/image_raw",
)
CORE_BAG_TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/imu": "sensor_msgs/msg/Imu",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/odom": "nav_msgs/msg/Odometry",
    "/map": "nav_msgs/msg/OccupancyGrid",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/plan": "nav_msgs/msg/Path",
    "/navigate_to_pose/_action/feedback": (
        "nav2_msgs/action/NavigateToPose_FeedbackMessage"
    ),
    "/navigate_to_pose/_action/status": "action_msgs/msg/GoalStatusArray",
    "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/feedback": (
        "nav2_msgs/action/NavigateToPose_FeedbackMessage"
    ),
    "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/status": (
        "action_msgs/msg/GoalStatusArray"
    ),
    "/baseline/frontier_mrtsp_dp/exploration_complete": "std_msgs/msg/Empty",
    "/explore/frontiers": "visualization_msgs/msg/MarkerArray",
    "/policy/decision_trace": "std_msgs/msg/String",
    "/policy/status": "std_msgs/msg/String",
    "/policy/candidates": "visualization_msgs/msg/MarkerArray",
    "/evaluation/ground_truth_odom": "nav_msgs/msg/Odometry",
    "/evaluation/world_stats": "ros_gz_interfaces/msg/WorldStatistics",
    "/evaluation/metrics": "std_msgs/msg/String",
    "/evaluation/status": "std_msgs/msg/String",
    "/task_camera/image_raw": "sensor_msgs/msg/Image",
}
CORE_BAG_REQUIRED_TOPICS = (
    "/clock",
    "/scan",
    "/map",
    "/policy/decision_trace",
    "/evaluation/ground_truth_odom",
    "/evaluation/world_stats",
    "/task_camera/image_raw",
)
CORE_BAG_REQUIRED_TOPICS_BY_RUNTIME_ADAPTER = {
    "sstg_policy": (
        "/navigate_to_pose/_action/feedback",
        "/navigate_to_pose/_action/status",
    ),
    "frontier_mrtsp_dp_external": (
        "/navigate_to_pose/_action/feedback",
        "/navigate_to_pose/_action/status",
        "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/feedback",
        "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/status",
    ),
}
LOCALIZATION_REPORTING_CONTRACT = {
    "schema": "sstg_system_sim_localization_reporting/v1",
    "evidence_source": "evaluator_ground_truth_ate",
    "continuous_metrics": ["ate_mean_m", "ate_rmse_m", "ate_max_m"],
    "analysis_population": "all_scheduled_runs",
    "missing_data_policy": "retain_as_missing_without_imputation",
    "localization_based_run_exclusion": False,
    "method_comparison_role": "secondary_outcome_not_adjustment_covariate",
    "paired_reporting_keys": [
        "world_id",
        "start_id",
        "condition",
        "replicate_seed",
    ],
    "map_to_odom_diagnostic": {
        "source_topic": "/tf",
        "parent_frame": "map",
        "child_frame": "odom",
        "statistic": "largest_adjacent_translation_correction_m",
        "threshold_m": None,
        "role": "descriptive_only",
    },
    "formal_threshold_policy": (
        "externally_anchor_and_freeze_before_test_split_or_report_none"
    ),
}
CSV_FIELDS = (
    "schema",
    "study_id",
    "schedule_id",
    "block_id",
    "block_index",
    "order_position",
    "backend",
    "world_id",
    "site_family",
    "world_split",
    "world_bundle",
    "world_sdf",
    "world_name",
    "start_id",
    "start_x_m",
    "start_y_m",
    "start_yaw_rad",
    "truth_map_yaml",
    "truth_registration_id",
    "truth_to_map_x_m",
    "truth_to_map_y_m",
    "truth_to_map_yaw_rad",
    "method",
    "runtime_adapter",
    "strategy",
    "coverage_objective",
    *METHOD_POLICY_DEFAULTS,
    "condition",
    "replicate_seed",
    "randomization_seed",
    *EXPERIMENT_BUDGET_FIELDS,
    "run_output_dir",
    "evidence_tier",
    "formal_result_eligible",
    "eligibility_reasons",
    "source_tree_sha256",
    "world_bundle_sha256",
    "world_sdf_sha256",
    "method_config_sha256",
    "condition_config_sha256",
    "shared_stack_sha256",
    "world_registry_sha256",
    "run_config_sha256",
)


class ScheduleError(ValueError):
    """Raised when a schedule cannot be frozen from the declared inputs."""


def validate_seed_contract(
    value: Any, *, label: str = "shared_stack.physics"
) -> dict[str, Any]:
    """Validate the common RNG source accepted by Gazebo Harmonic."""
    if not isinstance(value, Mapping):
        raise ScheduleError(f"{label} must be a mapping")
    if value.get("seed_source") != "replicate_seed":
        raise ScheduleError(f"{label}.seed_source must be replicate_seed")
    seed_range = value.get("seed_valid_range_inclusive")
    valid_range = (
        isinstance(seed_range, list)
        and len(seed_range) == 2
        and all(type(bound) is int for bound in seed_range)
        and seed_range == [GAZEBO_SEED_MIN, GAZEBO_SEED_MAX]
    )
    if not valid_range:
        raise ScheduleError(
            f"{label}.seed_valid_range_inclusive must be "
            f"[{GAZEBO_SEED_MIN}, {GAZEBO_SEED_MAX}]"
        )
    return {
        "seed_source": "replicate_seed",
        "valid_range_inclusive": [GAZEBO_SEED_MIN, GAZEBO_SEED_MAX],
        "launch_argument_columns": {
            argument: "replicate_seed" for argument in SEED_LAUNCH_ARGUMENTS
        },
    }


def validate_ros_gz_bridge_contract(
    value: Any, *, label: str = "shared_stack.ros_gz_bridge"
) -> dict[str, Any]:
    """Require the audited official bridge source overlay exactly."""
    if not isinstance(value, Mapping):
        raise ScheduleError(f"{label} must be a mapping")
    missing = [field for field in ROS_GZ_BRIDGE_CONTRACT if field not in value]
    extra = sorted(
        str(field) for field in value if field not in ROS_GZ_BRIDGE_CONTRACT
    )
    if missing:
        raise ScheduleError(f"{label} is missing: {', '.join(missing)}")
    if extra:
        raise ScheduleError(f"{label} has unknown fields: {', '.join(extra)}")
    for field, expected in ROS_GZ_BRIDGE_CONTRACT.items():
        actual = value[field]
        if type(actual) is not type(expected) or actual != expected:
            raise ScheduleError(f"{label}.{field} must be {expected!r}")
    return dict(ROS_GZ_BRIDGE_CONTRACT)


def validate_ros_middleware_contract(
    value: Any, *, label: str = "shared_stack.ros_middleware"
) -> dict[str, Any]:
    """Require one audited RMW and reject host-specific underlays."""
    if not isinstance(value, Mapping):
        raise ScheduleError(f"{label} must be a mapping")
    missing = [field for field in ROS_MIDDLEWARE_CONTRACT if field not in value]
    extra = sorted(
        str(field) for field in value if field not in ROS_MIDDLEWARE_CONTRACT
    )
    if missing:
        raise ScheduleError(f"{label} is missing: {', '.join(missing)}")
    if extra:
        raise ScheduleError(f"{label} has unknown fields: {', '.join(extra)}")
    for field, expected in ROS_MIDDLEWARE_CONTRACT.items():
        actual = value[field]
        if type(actual) is not type(expected) or actual != expected:
            raise ScheduleError(f"{label}.{field} must be {expected!r}")
    return deepcopy(ROS_MIDDLEWARE_CONTRACT)


def validate_recording_contract(
    value: Any, *, label: str = "shared_stack.recording"
) -> dict[str, Any]:
    """Validate the fixed rosbag2/MCAP evidence profile."""
    if not isinstance(value, Mapping):
        raise ScheduleError(f"{label} must be a mapping")
    expected_scalars = {
        "enabled": True,
        "backend": "rosbag2",
        "storage_id": "mcap",
        "storage_preset_profile": "zstd_fast",
        "include_hidden_topics": True,
        "output": "bags/core",
    }
    for field, expected in expected_scalars.items():
        if type(value.get(field)) is not type(expected) or value.get(field) != expected:
            raise ScheduleError(f"{label}.{field} must be {expected!r}")
    topics = value.get("topics")
    if not isinstance(topics, list) or tuple(topics) != CORE_BAG_TOPICS:
        raise ScheduleError(f"{label}.topics must match the frozen core topic list")
    topic_types = value.get("topic_types")
    if not isinstance(topic_types, Mapping) or dict(topic_types) != CORE_BAG_TOPIC_TYPES:
        raise ScheduleError(f"{label}.topic_types must match the frozen ROS types")
    required = value.get("required_nonempty_topics")
    if not isinstance(required, list) or tuple(required) != CORE_BAG_REQUIRED_TOPICS:
        raise ScheduleError(
            f"{label}.required_nonempty_topics must match the frozen required list"
        )
    expected_required_by_adapter = {
        runtime_adapter: list(topics)
        for runtime_adapter, topics in (
            CORE_BAG_REQUIRED_TOPICS_BY_RUNTIME_ADAPTER.items()
        )
    }
    required_by_adapter = value.get(
        "required_nonempty_topics_by_runtime_adapter"
    )
    if (
        not isinstance(required_by_adapter, Mapping)
        or dict(required_by_adapter) != expected_required_by_adapter
    ):
        raise ScheduleError(
            f"{label}.required_nonempty_topics_by_runtime_adapter must match "
            "the frozen runtime-adapter required lists"
        )
    return {
        **expected_scalars,
        "topics": list(CORE_BAG_TOPICS),
        "topic_types": dict(CORE_BAG_TOPIC_TYPES),
        "required_nonempty_topics": list(CORE_BAG_REQUIRED_TOPICS),
        "required_nonempty_topics_by_runtime_adapter": (
            expected_required_by_adapter
        ),
    }


def validate_localization_reporting_contract(
    value: Any, *, label: str = "shared_stack.localization_reporting"
) -> dict[str, Any]:
    """Freeze continuous localization reporting without post-hoc exclusion."""
    if not isinstance(value, Mapping):
        raise ScheduleError(f"{label} must be a mapping")
    if dict(value) != LOCALIZATION_REPORTING_CONTRACT:
        raise ScheduleError(f"{label} must match the frozen localization contract")
    return deepcopy(LOCALIZATION_REPORTING_CONTRACT)


def validate_experiment_budget(
    value: Any, *, label: str = "experiment_budget"
) -> dict[str, float | int]:
    """Normalize the required policy limits, rejecting disabled or mistyped values."""
    if not isinstance(value, Mapping):
        raise ScheduleError(f"{label} must be a mapping")
    missing = [field for field in EXPERIMENT_BUDGET_FIELDS if field not in value]
    extra = sorted(
        str(field) for field in value if field not in EXPERIMENT_BUDGET_FIELDS
    )
    if missing:
        raise ScheduleError(f"{label} is missing: {', '.join(missing)}")
    if extra:
        raise ScheduleError(f"{label} has unknown fields: {', '.join(extra)}")

    max_decisions = value["max_decisions"]
    if isinstance(max_decisions, bool) or not isinstance(max_decisions, int):
        raise ScheduleError(f"{label}.max_decisions must be a positive integer")
    if max_decisions <= 0:
        raise ScheduleError(f"{label}.max_decisions must be a positive integer")

    normalized: dict[str, float | int] = {"max_decisions": max_decisions}
    for field in ("max_duration_s", "max_distance_m", "goal_timeout_s"):
        number = _finite_number(value[field], f"{label}.{field}")
        if number <= 0.0:
            raise ScheduleError(f"{label}.{field} must be positive")
        normalized[field] = number
    if normalized["goal_timeout_s"] > normalized["max_duration_s"]:
        raise ScheduleError(
            f"{label}.goal_timeout_s must not exceed max_duration_s"
        )
    return {
        field: normalized[field]
        for field in EXPERIMENT_BUDGET_FIELDS
    }


def _effective_experiment_budget(
    shared_stack: Mapping[str, Any],
    evidence_tier: str,
    overrides: Mapping[str, Any] | None,
) -> tuple[dict[str, float | int], dict[str, float | int]]:
    declared = validate_experiment_budget(
        shared_stack.get("experiment_budget"),
        label="shared_stack.experiment_budget",
    )
    explicit = dict(overrides or {})
    unknown = sorted(
        str(field) for field in explicit if field not in EXPERIMENT_BUDGET_FIELDS
    )
    if unknown:
        raise ScheduleError(
            "unknown experiment budget overrides: " + ", ".join(unknown)
        )
    if evidence_tier == "formal" and explicit:
        raise ScheduleError(
            "formal schedules cannot override the frozen shared-stack "
            "experiment budget"
        )
    effective = dict(declared)
    effective.update(explicit)
    normalized = validate_experiment_budget(
        effective, label="effective experiment_budget"
    )
    return normalized, {
        field: normalized[field]
        for field in EXPERIMENT_BUDGET_FIELDS
        if field in explicit
    }


def inverse_spawn_transform(
    x_m: float, y_m: float, yaw_deg: float
) -> dict[str, float]:
    """Return ``T_map_truth = inverse(T_world_spawn)`` for a planar start pose.

    SLAM initializes its map frame at the robot spawn pose.  Static truth is in
    the Gazebo world frame, so evaluator coordinates require the inverse spawn
    transform, not merely the negated translation when the start yaw is nonzero.
    """
    values = (
        _finite_number(x_m, "start x_m"),
        _finite_number(y_m, "start y_m"),
        _finite_number(yaw_deg, "start yaw_deg"),
    )
    x_m, y_m, yaw_deg = values
    spawn_yaw = math.radians(yaw_deg)
    inverse_yaw = -spawn_yaw
    cosine = math.cos(inverse_yaw)
    sine = math.sin(inverse_yaw)
    inverse_x = -(cosine * x_m - sine * y_m)
    inverse_y = -(sine * x_m + cosine * y_m)

    def clean(value: float) -> float:
        return 0.0 if abs(value) < 1e-15 else value

    return {
        "spawn_yaw_rad": clean(spawn_yaw),
        "truth_to_map_x_m": clean(inverse_x),
        "truth_to_map_y_m": clean(inverse_y),
        "truth_to_map_yaw_rad": clean(inverse_yaw),
    }


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ScheduleError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ScheduleError(f"{label} must be a finite number") from error
    if not math.isfinite(result):
        raise ScheduleError(f"{label} must be a finite number")
    return result


def _format_float(value: float) -> str:
    """Use a stable round-trippable representation in CSV launch fields."""
    value = 0.0 if abs(value) < 1e-15 else value
    return format(value, ".17g")


def _sdf_world_name(path: Path) -> str:
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as error:
        raise ScheduleError(f"cannot parse world SDF {path}: {error}") from error
    worlds = root.findall("world")
    if len(worlds) != 1:
        raise ScheduleError(
            f"world SDF must contain exactly one top-level <world>: {path}"
        )
    return _require_id(worlds[0].get("name"), "world_name")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _path_label(root: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _source_files(paths: Iterable[Path]) -> list[Path]:
    files: dict[Path, Path] = {}
    for raw_path in paths:
        path = raw_path.resolve()
        if not path.exists():
            raise ScheduleError(f"source path does not exist: {raw_path}")
        candidates = [path] if path.is_file() else path.rglob("*")
        for candidate in candidates:
            if not candidate.is_file():
                continue
            if (
                ".git" in candidate.parts
                or "__pycache__" in candidate.parts
                or candidate.suffix == ".pyc"
            ):
                continue
            files[candidate.resolve()] = candidate.resolve()
    if not files:
        raise ScheduleError("source fingerprint contains no files")
    return sorted(files.values(), key=lambda item: item.as_posix())


def sha256_tree(root: Path, paths: Iterable[Path]) -> str:
    """Hash path names and bytes for a deterministic multi-path fingerprint."""
    digest = hashlib.sha256()
    for path in _source_files(paths):
        label = _path_label(root, path).encode("utf-8")
        digest.update(len(label).to_bytes(8, "big"))
        digest.update(label)
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def _canonical_hash(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _resolve_inside(root: Path, value: str | Path) -> Path:
    path = Path(value)
    resolved = (root / path).resolve() if not path.is_absolute() else path.resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise ScheduleError(f"input path must remain under project root: {value}") from error
    return resolved


def _display_path(root: Path, path: Path) -> str:
    return _path_label(root, path)


def _load_mapping(path: Path, expected_schema: str) -> dict[str, Any]:
    if not path.is_file():
        raise ScheduleError(f"missing input file: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ScheduleError(f"invalid YAML in {path}: {error}") from error
    if not isinstance(value, dict):
        raise ScheduleError(f"expected a YAML mapping in {path}")
    if value.get("schema") != expected_schema:
        raise ScheduleError(
            f"{path} has schema {value.get('schema')!r}; expected {expected_schema!r}"
        )
    return value


def _require_id(value: Any, label: str) -> str:
    if value is None or isinstance(value, bool):
        raise ScheduleError(f"invalid {label}: {value!r}")
    text = str(value)
    if not ID_PATTERN.fullmatch(text):
        raise ScheduleError(f"invalid {label}: {text!r}")
    return text


def _contains_tbd(value: Any) -> bool:
    if isinstance(value, Mapping):
        return any(_contains_tbd(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return any(_contains_tbd(item) for item in value)
    return isinstance(value, str) and "TBD" in value.upper()


def _git_identity(root: Path, relevant_paths: Sequence[Path]) -> dict[str, Any]:
    def run(arguments: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["git", "-C", str(root), *arguments],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )

    try:
        commit_result = run(["rev-parse", "HEAD"])
    except (OSError, subprocess.TimeoutExpired):
        return {"repository_commit": None, "repository_dirty": True}
    if commit_result.returncode != 0:
        return {"repository_commit": None, "repository_dirty": True}

    pathspecs: list[str] = []
    for path in relevant_paths:
        try:
            pathspecs.append(path.resolve().relative_to(root.resolve()).as_posix())
        except ValueError:
            return {
                "repository_commit": commit_result.stdout.strip(),
                "repository_dirty": True,
            }
    try:
        status_result = run(
            ["status", "--porcelain", "--untracked-files=all", "--", *pathspecs]
        )
    except (OSError, subprocess.TimeoutExpired):
        return {
            "repository_commit": commit_result.stdout.strip(),
            "repository_dirty": True,
        }
    return {
        "repository_commit": commit_result.stdout.strip(),
        "repository_dirty": status_result.returncode != 0 or bool(status_result.stdout),
    }


def _config_path(root: Path, category: str, value: str | Path) -> Path:
    candidate = Path(value)
    if candidate.suffix or candidate.parent != Path("."):
        return _resolve_inside(root, candidate)
    return _resolve_inside(
        root, EXPERIMENT_ROOT / "configs" / category / f"{candidate.name}.yaml"
    )


def _world_specs(
    root: Path,
    registry: Mapping[str, Any],
    selected_world_ids: Sequence[str] | None,
    start_policy: str,
) -> list[dict[str, Any]]:
    entries = registry.get("worlds")
    if not isinstance(entries, list) or not entries:
        raise ScheduleError("world registry must contain a non-empty worlds list")
    indexed: dict[str, Mapping[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping):
            raise ScheduleError("each world registry entry must be a mapping")
        world_id = _require_id(entry.get("world_id"), "world_id")
        if world_id in indexed:
            raise ScheduleError(f"duplicate world_id in registry: {world_id}")
        indexed[world_id] = entry

    wanted = sorted(indexed) if not selected_world_ids else sorted(
        {_require_id(item, "world_id") for item in selected_world_ids}
    )
    missing = sorted(set(wanted) - set(indexed))
    if missing:
        raise ScheduleError(f"unknown world IDs: {', '.join(missing)}")

    results: list[dict[str, Any]] = []
    for world_id in wanted:
        entry = indexed[world_id]
        bundle = _resolve_inside(root, str(entry.get("bundle", "")))
        if not bundle.is_dir():
            raise ScheduleError(f"world bundle is not a directory: {bundle}")
        metadata_path = bundle / "metadata.yaml"
        world_path = bundle / "world.sdf"
        starts_path = bundle / "starts.yaml"
        targets_path = bundle / "targets.yaml"
        truth_map_path = bundle / "evaluation" / "truth_map.yaml"
        metadata = _load_mapping(metadata_path, "sstg_system_sim_world/v1")
        starts = _load_mapping(starts_path, "sstg_system_sim_starts/v1")
        if not world_path.is_file():
            raise ScheduleError(f"world bundle has no world.sdf: {bundle}")
        if not truth_map_path.is_file():
            raise ScheduleError(f"world bundle has no evaluation truth map: {bundle}")
        world_name = _sdf_world_name(world_path)
        start_entries = starts.get("starts")
        if not isinstance(start_entries, list) or not start_entries:
            raise ScheduleError(f"world {world_id} has no registered start poses")
        parsed_starts: list[dict[str, Any]] = []
        for item in start_entries:
            if not isinstance(item, Mapping):
                raise ScheduleError(f"world {world_id} has a non-mapping start pose")
            start_id = _require_id(item.get("start_id"), "start_id")
            x_m = _finite_number(item.get("x_m"), f"{world_id}/{start_id} x_m")
            y_m = _finite_number(item.get("y_m"), f"{world_id}/{start_id} y_m")
            yaw_deg = _finite_number(
                item.get("yaw_deg"), f"{world_id}/{start_id} yaw_deg"
            )
            parsed_starts.append(
                {
                    "start_id": start_id,
                    "x_m": x_m,
                    "y_m": y_m,
                    "yaw_deg": yaw_deg,
                    **inverse_spawn_transform(x_m, y_m, yaw_deg),
                }
            )
        start_ids = [item["start_id"] for item in parsed_starts]
        if len(set(start_ids)) != len(start_ids):
            raise ScheduleError(f"world {world_id} has invalid or duplicate start IDs")
        selected_starts = (
            [parsed_starts[0]]
            if start_policy == "first"
            else sorted(parsed_starts, key=lambda item: item["start_id"])
        )

        crosschecks = {
            "world_id": world_id,
            "backend": entry.get("backend"),
            "split": entry.get("split"),
            "site_family": entry.get("site_family"),
            "formal_result_eligible": entry.get("formal_result_eligible"),
        }
        for key, expected in crosschecks.items():
            if metadata.get(key) != expected:
                raise ScheduleError(
                    f"world {world_id} registry/metadata mismatch for {key}: "
                    f"{expected!r} != {metadata.get(key)!r}"
                )
        if starts.get("world_id") != world_id:
            raise ScheduleError(f"starts.yaml world_id mismatch for {world_id}")
        if targets_path.is_file():
            targets = _load_mapping(targets_path, "sstg_system_sim_targets/v1")
            if targets.get("world_id") != world_id:
                raise ScheduleError(f"targets.yaml world_id mismatch for {world_id}")

        results.append(
            {
                "world_id": world_id,
                "backend": str(entry.get("backend")),
                "split": str(entry.get("split")),
                "site_family": str(entry.get("site_family")),
                "bundle": bundle,
                "bundle_display": _display_path(root, bundle),
                "world_sdf_display": _display_path(root, world_path),
                "world_name": world_name,
                "truth_map_display": _display_path(root, truth_map_path),
                "formal_result_eligible": entry.get("formal_result_eligible") is True,
                "starts": selected_starts,
                "sha256": {
                    "bundle": sha256_tree(root, [bundle]),
                    "world.sdf": sha256_file(world_path),
                    "metadata.yaml": sha256_file(metadata_path),
                    "starts.yaml": sha256_file(starts_path),
                    "targets.yaml": (
                        sha256_file(targets_path) if targets_path.is_file() else None
                    ),
                    "evaluation/truth_map.yaml": sha256_file(truth_map_path),
                },
            }
        )
    return results


def _formal_reasons(
    evidence_tier: str,
    shared_stack: Mapping[str, Any],
    condition: Mapping[str, Any],
    methods: Sequence[Mapping[str, Any]],
    worlds: Sequence[Mapping[str, Any]],
    git_identity: Mapping[str, Any],
) -> list[str]:
    reasons: list[str] = []
    if evidence_tier != "formal":
        reasons.append("evidence_tier_is_development")
    if shared_stack.get("freeze_status") != "frozen":
        reasons.append(f"shared_stack_not_frozen:{shared_stack.get('freeze_status')}")
    if condition.get("status") != "frozen":
        reasons.append(f"condition_not_frozen:{condition.get('status')}")
    for method in methods:
        if method.get("status") != "frozen":
            reasons.append(f"method_not_frozen:{method.get('method')}:{method.get('status')}")
        if method.get("formal_method_eligible") is not True:
            reasons.append(
                f"method_not_formal_eligible:{method.get('method')}"
            )
    for world in worlds:
        if world.get("split") != "test":
            reasons.append(f"world_not_test_split:{world.get('world_id')}:{world.get('split')}")
        if world.get("formal_result_eligible") is not True:
            reasons.append(f"world_not_formal_eligible:{world.get('world_id')}")
    if not git_identity.get("repository_commit"):
        reasons.append("source_repository_commit_unavailable")
    if git_identity.get("repository_dirty") is not False:
        reasons.append("source_paths_dirty")
    return reasons


def _method_order(method_ids: Sequence[str], block_id: str, seed: int) -> list[str]:
    material = f"sstg-system-sim-method-order/v1\0{seed}\0{block_id}\0"
    return sorted(
        method_ids,
        key=lambda method: hashlib.sha256(
            f"{material}{method}".encode("utf-8")
        ).digest(),
    )


def _csv_bytes(rows: Sequence[Mapping[str, Any]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow(row)
    return stream.getvalue().encode("utf-8")


def _atomic_write(path: Path, content: bytes) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        temporary.write_bytes(content)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def planned_run_output_paths(
    root: Path, rows: Sequence[Mapping[str, Any]]
) -> list[Path]:
    """Resolve and validate the one-to-one schedule-to-output mapping."""
    paths: list[Path] = []
    labels: set[str] = set()
    for row in rows:
        schedule_id = str(row.get("schedule_id", ""))
        output_label = str(row.get("run_output_dir", ""))
        if not schedule_id or not output_label:
            raise ScheduleError("each schedule row requires an ID and run output dir")
        if output_label in labels:
            raise ScheduleError(f"duplicate run output directory: {output_label}")
        labels.add(output_label)
        paths.append(_resolve_inside(root, output_label))
    if len(paths) != len(rows):
        raise ScheduleError("each schedule run must have a unique output directory")
    return paths


def refuse_preexisting_run_outputs(root: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    """Reject even empty pre-existing run directories to prevent log reuse."""
    existing = [path for path in planned_run_output_paths(root, rows) if path.exists()]
    if existing:
        raise ScheduleError(
            "refusing pre-existing run output paths (empty paths are also reserved): "
            + ", ".join(_display_path(root, path) for path in existing)
        )


def freeze_schedule(
    *,
    root: Path,
    study_id: str,
    output_dir: Path,
    world_registry_path: Path,
    shared_stack_path: Path,
    method_paths: Sequence[Path],
    condition_path: Path,
    world_ids: Sequence[str] | None,
    replicate_seeds: Sequence[int],
    randomization_seed: int,
    evidence_tier: str = "development",
    start_policy: str = "first",
    budget_overrides: Mapping[str, Any] | None = None,
    source_paths: Sequence[Path] | None = None,
    run_output_root: Path | None = None,
    force: bool = False,
) -> dict[str, Any]:
    """Create a frozen schedule and manifest, returning the manifest mapping."""
    root = root.resolve()
    study_id = _require_id(study_id, "study_id")
    if evidence_tier not in {"development", "formal"}:
        raise ScheduleError("evidence_tier must be development or formal")
    if start_policy not in {"first", "all"}:
        raise ScheduleError("start_policy must be first or all")
    if any(
        isinstance(seed, bool) or not isinstance(seed, int)
        for seed in replicate_seeds
    ):
        raise ScheduleError(
            "replicate seeds must be positive signed 32-bit integers for Gazebo"
        )
    seeds = sorted(set(replicate_seeds))
    if not seeds:
        raise ScheduleError("at least one replicate seed is required")
    if len(seeds) != len(replicate_seeds):
        raise ScheduleError("replicate seeds must be unique")
    if any(not GAZEBO_SEED_MIN <= seed <= GAZEBO_SEED_MAX for seed in seeds):
        raise ScheduleError(
            "replicate seeds must be positive signed 32-bit integers for Gazebo"
        )
    if randomization_seed < 0:
        raise ScheduleError("randomization seed must be a non-negative integer")

    world_registry_path = _resolve_inside(root, world_registry_path)
    shared_stack_path = _resolve_inside(root, shared_stack_path)
    condition_path = _resolve_inside(root, condition_path)
    method_paths = [_resolve_inside(root, path) for path in method_paths]
    if not method_paths:
        raise ScheduleError("at least one method config is required")

    registry = _load_mapping(
        world_registry_path, "sstg_system_sim_world_registry/v1"
    )
    shared_stack = _load_mapping(
        shared_stack_path, "sstg_system_sim_shared_stack/v1"
    )
    seed_contract = validate_seed_contract(shared_stack.get("physics"))
    ros_gz_bridge_contract = validate_ros_gz_bridge_contract(
        shared_stack.get("ros_gz_bridge")
    )
    ros_middleware_contract = validate_ros_middleware_contract(
        shared_stack.get("ros_middleware")
    )
    recording_contract = validate_recording_contract(
        shared_stack.get("recording")
    )
    localization_reporting_contract = (
        validate_localization_reporting_contract(
            shared_stack.get("localization_reporting")
        )
    )
    experiment_budget, applied_budget_overrides = _effective_experiment_budget(
        shared_stack, evidence_tier, budget_overrides
    )
    condition = _load_mapping(condition_path, "sstg_system_sim_condition/v1")
    if _contains_tbd(condition) or "not_runnable" in str(condition.get("status", "")):
        raise ScheduleError(
            f"condition {condition.get('condition')!r} is not runnable or contains TBD values"
        )
    condition_id = _require_id(condition.get("condition"), "condition")

    methods: list[dict[str, Any]] = []
    method_records: list[dict[str, str]] = []
    for path in method_paths:
        config = _load_mapping(path, "sstg_system_sim_method/v1")
        method_id = _require_id(config.get("method"), "method")
        methods.append(config)
        method_records.append(
            {
                "method": method_id,
                "runtime_adapter": _require_id(
                    config.get("runtime_adapter"), "runtime_adapter"
                ),
                "path": _display_path(root, path),
                "sha256": sha256_file(path),
            }
        )
    if len({item["method"] for item in method_records}) != len(method_records):
        raise ScheduleError("method configs declare duplicate method IDs")
    paired = sorted(zip(methods, method_records), key=lambda pair: pair[1]["method"])
    methods = [item[0] for item in paired]
    method_records = [item[1] for item in paired]
    method_launch: dict[str, dict[str, Any]] = {}
    for config, record in paired:
        method_id = record["method"]
        runtime_adapter = record["runtime_adapter"]
        if runtime_adapter not in RUNTIME_ADAPTERS:
            raise ScheduleError(
                f"unsupported runtime_adapter for {method_id}: "
                f"{runtime_adapter!r}"
            )
        allowed_method_ids = RUNTIME_ADAPTER_METHOD_IDS.get(runtime_adapter)
        if allowed_method_ids is not None and method_id not in allowed_method_ids:
            raise ScheduleError(
                f"runtime_adapter {runtime_adapter!r} requires method ID in "
                f"{sorted(allowed_method_ids)!r}; found {method_id!r}"
            )
        method_launch[method_id] = {
            "runtime_adapter": runtime_adapter,
            "strategy": _require_id(config.get("strategy"), "strategy"),
            "coverage_objective": _require_id(
                config.get("coverage_objective"), "coverage_objective"
            ),
            **{
                name: _finite_number(
                    config.get(name, default), f"{method_id}.{name}"
                )
                for name, default in METHOD_POLICY_DEFAULTS.items()
            },
        }
        if any(
            method_launch[method_id][name] < 0.0
            for name in METHOD_POLICY_DEFAULTS
        ):
            raise ScheduleError(
                f"method {method_id} policy weights must be non-negative"
            )

    worlds = _world_specs(root, registry, world_ids, start_policy)
    backend = str(shared_stack.get("backend"))
    if backend != "gazebo_harmonic":
        raise ScheduleError(
            f"this schedule freezer requires gazebo_harmonic, found {backend!r}"
        )
    mismatched_backends = [
        world["world_id"] for world in worlds if world["backend"] != backend
    ]
    if mismatched_backends:
        raise ScheduleError(
            f"world backend differs from shared stack ({backend}): "
            + ", ".join(mismatched_backends)
        )

    if source_paths is None:
        source_paths = [root / "src", root / "ros2_ws" / "src"]
    source_paths = [_resolve_inside(root, path) for path in source_paths]
    if run_output_root is None:
        run_output_root = Path("system_sim_outputs") / "runs" / study_id
    run_output_root = _resolve_inside(root, run_output_root)
    source_tree_sha256 = sha256_tree(root, source_paths)
    relevant_paths = [
        *source_paths,
        world_registry_path,
        shared_stack_path,
        condition_path,
        *method_paths,
        *(world["bundle"] for world in worlds),
    ]
    tool_path = Path(__file__).resolve()
    runner_tool_path = tool_path.with_name("run_system_sim_schedule.py")
    if not runner_tool_path.is_file():
        raise ScheduleError(f"missing schedule runner: {runner_tool_path}")
    try:
        tool_path.relative_to(root)
    except ValueError:
        pass
    else:
        relevant_paths.append(tool_path)
    try:
        runner_tool_path.relative_to(root)
    except ValueError:
        pass
    else:
        relevant_paths.append(runner_tool_path)
    runner_tool_sha = sha256_file(runner_tool_path)
    git = _git_identity(root, relevant_paths)
    formal_reasons = _formal_reasons(
        evidence_tier, shared_stack, condition, methods, worlds, git
    )
    formal_eligible = not formal_reasons
    if evidence_tier == "formal" and not formal_eligible:
        raise ScheduleError(
            "formal schedule refused because freeze gates failed: "
            + "; ".join(formal_reasons)
        )

    registry_sha = sha256_file(world_registry_path)
    shared_sha = sha256_file(shared_stack_path)
    condition_sha = sha256_file(condition_path)
    study_config_sha = _canonical_hash(
        {
            "world_registry": registry_sha,
            "shared_stack": shared_sha,
            "condition": condition_sha,
            "methods": method_records,
            "run_output_root": _display_path(root, run_output_root),
            "runner_tool_sha256": runner_tool_sha,
            "experiment_budget": experiment_budget,
            "ros_gz_bridge_contract": ros_gz_bridge_contract,
            "ros_middleware_contract": ros_middleware_contract,
        }
    )
    method_hashes = {item["method"]: item["sha256"] for item in method_records}
    method_ids = [item["method"] for item in method_records]

    rows: list[dict[str, Any]] = []
    block_index = 0
    for world in worlds:
        for start in world["starts"]:
            start_id = start["start_id"]
            for replicate_seed in seeds:
                block_index += 1
                block_id = (
                    f"{study_id}__{world['world_id']}__{start_id}__"
                    f"{condition_id}__seed_{replicate_seed}"
                )
                ordered_methods = _method_order(
                    method_ids, block_id, randomization_seed
                )
                for order_position, method_id in enumerate(ordered_methods, start=1):
                    row_reasons = list(formal_reasons)
                    schedule_id = (
                        f"{block_id}__order_{order_position:02d}__{method_id}"
                    )
                    run_output_dir = run_output_root / schedule_id
                    run_config_sha = _canonical_hash(
                        {
                            "shared_stack": shared_sha,
                            "condition": condition_sha,
                            "method": method_hashes[method_id],
                            "world": world["sha256"]["bundle"],
                            "start": start,
                            "replicate_seed": replicate_seed,
                            "experiment_budget": experiment_budget,
                            "output_dir": _display_path(root, run_output_dir),
                            "runner_tool_sha256": runner_tool_sha,
                        }
                    )
                    rows.append(
                        {
                            "schema": SCHEDULE_SCHEMA,
                            "study_id": study_id,
                            "schedule_id": schedule_id,
                            "block_id": block_id,
                            "block_index": block_index,
                            "order_position": order_position,
                            "backend": backend,
                            "world_id": world["world_id"],
                            "site_family": world["site_family"],
                            "world_split": world["split"],
                            "world_bundle": world["bundle_display"],
                            "world_sdf": world["world_sdf_display"],
                            "world_name": world["world_name"],
                            "start_id": start_id,
                            "start_x_m": _format_float(start["x_m"]),
                            "start_y_m": _format_float(start["y_m"]),
                            "start_yaw_rad": _format_float(start["spawn_yaw_rad"]),
                            "truth_map_yaml": world["truth_map_display"],
                            "truth_registration_id": (
                                f"{world['world_id']}:{start_id}:inverse_spawn_pose"
                            ),
                            "truth_to_map_x_m": _format_float(
                                start["truth_to_map_x_m"]
                            ),
                            "truth_to_map_y_m": _format_float(
                                start["truth_to_map_y_m"]
                            ),
                            "truth_to_map_yaw_rad": _format_float(
                                start["truth_to_map_yaw_rad"]
                            ),
                            "method": method_id,
                            "runtime_adapter": method_launch[method_id][
                                "runtime_adapter"
                            ],
                            "strategy": method_launch[method_id]["strategy"],
                            "coverage_objective": method_launch[method_id][
                                "coverage_objective"
                            ],
                            **{
                                name: _format_float(
                                    float(method_launch[method_id][name])
                                )
                                for name in METHOD_POLICY_DEFAULTS
                            },
                            "condition": condition_id,
                            "replicate_seed": replicate_seed,
                            "randomization_seed": randomization_seed,
                            "max_duration_s": _format_float(
                                float(experiment_budget["max_duration_s"])
                            ),
                            "max_distance_m": _format_float(
                                float(experiment_budget["max_distance_m"])
                            ),
                            "max_decisions": experiment_budget["max_decisions"],
                            "goal_timeout_s": _format_float(
                                float(experiment_budget["goal_timeout_s"])
                            ),
                            "run_output_dir": _display_path(root, run_output_dir),
                            "evidence_tier": evidence_tier,
                            "formal_result_eligible": str(formal_eligible).lower(),
                            "eligibility_reasons": "|".join(row_reasons),
                            "source_tree_sha256": source_tree_sha256,
                            "world_bundle_sha256": world["sha256"]["bundle"],
                            "world_sdf_sha256": world["sha256"]["world.sdf"],
                            "method_config_sha256": method_hashes[method_id],
                            "condition_config_sha256": condition_sha,
                            "shared_stack_sha256": shared_sha,
                            "world_registry_sha256": registry_sha,
                            "run_config_sha256": run_config_sha,
                        }
                    )

    refuse_preexisting_run_outputs(root, rows)

    schedule_bytes = _csv_bytes(rows)
    schedule_sha = hashlib.sha256(schedule_bytes).hexdigest()
    manifest: dict[str, Any] = {
        "schema": FREEZE_SCHEMA,
        "study_id": study_id,
        "backend": backend,
        "eligibility": {
            "evidence_tier": evidence_tier,
            "formal_result_eligible": formal_eligible,
            "reasons": formal_reasons,
        },
        "design": {
            "matching_unit": [
                "world_id",
                "start_id",
                "condition",
                "replicate_seed",
            ],
            "randomization_unit": "block_id",
            "randomized_factor": "method",
            "method_order_algorithm": "sha256-key-sort/v1",
            "methods": method_ids,
            "condition": condition_id,
            "replicate_seeds": seeds,
            "start_policy": start_policy,
            "run_output_root": _display_path(root, run_output_root),
            "block_count": block_index,
            "methods_per_block": len(method_ids),
            "scheduled_run_count": len(rows),
        },
        "randomization": {"seed": randomization_seed},
        "seed_contract": seed_contract,
        "ros_gz_bridge_contract": ros_gz_bridge_contract,
        "ros_middleware_contract": ros_middleware_contract,
        "recording_contract": recording_contract,
        "localization_reporting_contract": localization_reporting_contract,
        "experiment_budget": experiment_budget,
        "budget_provenance": {
            "source": (
                "development_override"
                if applied_budget_overrides
                else "shared_stack"
            ),
            "development_overrides": applied_budget_overrides,
        },
        "source": {
            **git,
            "source_tree_sha256": source_tree_sha256,
            "source_paths": [_display_path(root, path) for path in source_paths],
            "tool_path": _display_path(root, tool_path),
            "tool_sha256": sha256_file(tool_path),
            "runner_tool_path": _display_path(root, runner_tool_path),
            "runner_tool_sha256": runner_tool_sha,
            "tree_hash_algorithm": "sha256(path-length || path || file-sha256)/v1",
        },
        "inputs": {
            "study_config_sha256": study_config_sha,
            "world_registry": {
                "path": _display_path(root, world_registry_path),
                "sha256": registry_sha,
            },
            "shared_stack": {
                "path": _display_path(root, shared_stack_path),
                "sha256": shared_sha,
                "freeze_status": shared_stack.get("freeze_status"),
                "seed_contract": validate_seed_contract(
                    shared_stack.get("physics")
                ),
                "ros_gz_bridge_contract": validate_ros_gz_bridge_contract(
                    shared_stack.get("ros_gz_bridge")
                ),
                "ros_middleware_contract": validate_ros_middleware_contract(
                    shared_stack.get("ros_middleware")
                ),
                "recording_contract": validate_recording_contract(
                    shared_stack.get("recording")
                ),
                "localization_reporting_contract": (
                    validate_localization_reporting_contract(
                        shared_stack.get("localization_reporting")
                    )
                ),
                "experiment_budget": validate_experiment_budget(
                    shared_stack.get("experiment_budget"),
                    label="shared_stack.experiment_budget",
                ),
            },
            "condition": {
                "condition": condition_id,
                "path": _display_path(root, condition_path),
                "sha256": condition_sha,
                "status": condition.get("status"),
            },
            "methods": [
                {**record, "status": config.get("status")}
                for config, record in zip(methods, method_records)
            ],
            "worlds": [
                {
                    "world_id": world["world_id"],
                    "site_family": world["site_family"],
                    "split": world["split"],
                    "formal_result_eligible": world["formal_result_eligible"],
                    "bundle": world["bundle_display"],
                    "world_sdf": world["world_sdf_display"],
                    "world_name": world["world_name"],
                    "truth_map_yaml": world["truth_map_display"],
                    "starts": world["starts"],
                    "sha256": world["sha256"],
                }
                for world in worlds
            ],
        },
        "launch": {
            "package": "sstg_nav_bringup",
            "file": "system_sim.launch.py",
            "fixed_arguments": {
                "headless": "true",
                "rviz": "false",
                "evaluator": "true",
                "record_bag": "true",
            },
            "argument_columns": {
                "world": "world_sdf",
                "world_name": "world_name",
                "start_x": "start_x_m",
                "start_y": "start_y_m",
                "start_yaw": "start_yaw_rad",
                "output_dir": "run_output_dir",
                "runtime_adapter": "runtime_adapter",
                "strategy": "strategy",
                "coverage_objective": "coverage_objective",
                "clearance_weight": "clearance_weight",
                "travel_cost_weight": "travel_cost_weight",
                "policy_seed": "replicate_seed",
                "simulation_seed": "replicate_seed",
                "max_duration_s": "max_duration_s",
                "max_distance_m": "max_distance_m",
                "max_decisions": "max_decisions",
                "goal_timeout_s": "goal_timeout_s",
                "truth_map_yaml": "truth_map_yaml",
                "truth_registration_id": "truth_registration_id",
                "truth_to_map_x_m": "truth_to_map_x_m",
                "truth_to_map_y_m": "truth_to_map_y_m",
                "truth_to_map_yaw_rad": "truth_to_map_yaw_rad",
            },
        },
        "outputs": {
            "run_schedule": "run_schedule.csv",
            "run_schedule_sha256": schedule_sha,
        },
        "execution": {
            "simulator_invoked": False,
            "status": "not_started",
        },
    }
    manifest_bytes = yaml.safe_dump(
        manifest, sort_keys=False, allow_unicode=False
    ).encode("utf-8")

    output_dir = _resolve_inside(root, output_dir)
    schedule_path = output_dir / "run_schedule.csv"
    manifest_path = output_dir / "schedule_freeze_manifest.yaml"
    existing = [path for path in (schedule_path, manifest_path) if path.exists()]
    if existing and not force:
        raise ScheduleError(
            "refusing to overwrite frozen artifacts: "
            + ", ".join(_display_path(root, path) for path in existing)
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    _atomic_write(schedule_path, schedule_bytes)
    _atomic_write(manifest_path, manifest_bytes)
    return manifest


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--study-id", required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--world-registry",
        type=Path,
        default=EXPERIMENT_ROOT / "registries/worlds.yaml",
    )
    parser.add_argument(
        "--shared-stack",
        type=Path,
        default=EXPERIMENT_ROOT / "configs/shared_stack.yaml",
    )
    parser.add_argument(
        "--world",
        dest="worlds",
        action="append",
        help="world_id to include (repeatable; defaults to every registry world)",
    )
    parser.add_argument(
        "--method",
        dest="methods",
        action="append",
        required=True,
        help="method ID or config path (repeatable)",
    )
    parser.add_argument("--condition", required=True, help="condition ID or config path")
    parser.add_argument(
        "--replicate-seed", type=int, action="append", required=True
    )
    parser.add_argument("--randomization-seed", type=int, required=True)
    parser.add_argument(
        "--evidence-tier",
        choices=("development", "formal"),
        default="development",
    )
    parser.add_argument(
        "--start-policy", choices=("first", "all"), default="first"
    )
    parser.add_argument(
        "--max-duration-s",
        type=float,
        help="development-only override of the shared-stack policy duration",
    )
    parser.add_argument(
        "--max-distance-m",
        type=float,
        help="development-only override of the shared-stack travel budget",
    )
    parser.add_argument(
        "--max-decisions",
        type=int,
        help="development-only override of the shared-stack action budget",
    )
    parser.add_argument(
        "--goal-timeout-s",
        type=float,
        help="development-only override of the shared-stack per-goal timeout",
    )
    parser.add_argument(
        "--source-path",
        type=Path,
        action="append",
        help="source path to fingerprint (repeatable; defaults to src and ros2_ws/src)",
    )
    parser.add_argument(
        "--run-output-root",
        type=Path,
        help=(
            "directory containing per-run artifacts "
            "(defaults to system_sim_outputs/runs/<study-id>)"
        ),
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    root = args.root.resolve()
    output_dir = args.output_dir or (
        EXPERIMENT_ROOT / "studies" / args.study_id
    )
    try:
        manifest = freeze_schedule(
            root=root,
            study_id=args.study_id,
            output_dir=output_dir,
            world_registry_path=args.world_registry,
            shared_stack_path=args.shared_stack,
            method_paths=[_config_path(root, "methods", item) for item in args.methods],
            condition_path=_config_path(root, "conditions", args.condition),
            world_ids=args.worlds,
            replicate_seeds=args.replicate_seed,
            randomization_seed=args.randomization_seed,
            evidence_tier=args.evidence_tier,
            start_policy=args.start_policy,
            budget_overrides={
                field: getattr(args, field)
                for field in EXPERIMENT_BUDGET_FIELDS
                if getattr(args, field) is not None
            },
            source_paths=args.source_path,
            run_output_root=args.run_output_root,
            force=args.force,
        )
    except ScheduleError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "study_id": manifest["study_id"],
                "scheduled_runs": manifest["design"]["scheduled_run_count"],
                "formal_result_eligible": manifest["eligibility"][
                    "formal_result_eligible"
                ],
                "experiment_budget": manifest["experiment_budget"],
                "schedule_sha256": manifest["outputs"]["run_schedule_sha256"],
                "simulator_invoked": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
