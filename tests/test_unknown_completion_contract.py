"""Machine-check the procedural-to-ROS unknown-completion equivalence contract."""
from dataclasses import asdict
import hashlib
from pathlib import Path

import yaml

from sstg_explorer import SensorConfig, UnknownExplorerConfig


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_PATH = ROOT / "experiments/system_sim/configs/unknown_completion.yaml"
POLICY_PATH = ROOT / "ros2_ws/src/sstg_policy_ros/config/policy.yaml"


def _yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_reference_and_checkpoint_are_exactly_frozen():
    protocol = _yaml(PROTOCOL_PATH)
    reference = protocol["procedural_reference"]
    checkpoint = protocol["ans_checkpoint"]

    assert _sha256(ROOT / reference["manifest"]) == reference["manifest_sha256"]
    assert _sha256(ROOT / checkpoint["path"]) == checkpoint["sha256"]


def test_five_ros_methods_map_one_to_one_to_procedural_algorithms():
    protocol = _yaml(PROTOCOL_PATH)
    mappings = protocol["methods"]

    assert [item["procedural_algorithm"] for item in mappings] == [
        "frontier", "nbv", "rrt", "ans", "sstg"
    ]
    assert len({item["strategy"] for item in mappings}) == 5
    for item in mappings:
        assert item["native_termination_rule"].startswith("no_")
        method = _yaml(ROOT / item["config"])
        assert method["method"] == item["method"]
        assert method["strategy"] == item["strategy"]
        assert method["runtime_adapter"] == "sstg_policy"
        assert method["unknown_completion_protocol"] == "unknown_completion_v1"


def test_ros_policy_matches_shared_procedural_algorithm_config():
    protocol = _yaml(PROTOCOL_PATH)
    shared = protocol["shared_policy"]
    sensor = protocol["sensor_adaptation"]
    fail_safes = protocol["fail_safes"]
    ros = _yaml(POLICY_PATH)["sstg_policy"]["ros__parameters"]

    field_map = {
        "coverage_objective": "coverage_objective",
        "topological_radius_m": "topological_radius_m",
        "topological_merge_distance_m": "topological_merge_distance_m",
        "target_sensor_coverage": "target_sensor_coverage",
        "target_topological_coverage": "target_topological_coverage",
        "information_gain_weight": "information_gain_weight",
        "topological_gain_weight": "topological_gain_weight",
        "robot_radius_m": "robot_radius_m",
        "safety_margin_m": "safety_margin_m",
        "minimum_goal_clearance_m": "minimum_goal_clearance_m",
        "preferred_clearance_m": "preferred_clearance_m",
        "target_spacing_m": "target_spacing_m",
        "min_gain_cells": "min_gain_cells",
        "min_topological_gain_cells": "min_topological_gain_cells",
        "max_frontier_candidates": "max_frontier_candidates",
        "random_candidates": "random_candidates",
        "exact_gain_budget": "exact_gain_budget",
        "clearance_weight": "clearance_weight",
        "travel_cost_weight": "travel_cost_weight",
        "spacing_weight": "spacing_weight",
        "multi_frontier": "multi_frontier",
        "use_topological_vantages": "use_topological_vantages",
        "require_known_footprint": "require_known_footprint",
    }
    for protocol_name, ros_name in field_map.items():
        assert ros[ros_name] == shared[protocol_name]

    assert ros["termination_mode"] == protocol["policy_contract"]["termination_mode"]
    assert ros["online_exhaustion_confirmations"] == protocol[
        "policy_contract"
    ]["completion_confirmation"]["consecutive_new_map_revisions"]
    assert ros["lidar_fov_deg"] == sensor["field_of_view_deg"]
    assert ros["lidar_range_m"] == sensor["max_range_m"]
    assert ros["lidar_angular_resolution_deg"] == sensor["angular_resolution_deg"]
    for name in ("max_decisions", "max_distance_m", "max_duration_s", "goal_timeout_s"):
        assert ros[name] == fail_safes[name]
    assert ros["ans_checkpoint"] == protocol["ans_checkpoint"]["path"]


def test_protocol_constructs_the_same_shared_explorer_configuration():
    protocol = _yaml(PROTOCOL_PATH)
    shared = protocol["shared_policy"]
    sensor = protocol["sensor_adaptation"]
    config = UnknownExplorerConfig(
        strategy="sstg",
        sensor=SensorConfig(
            sensor["field_of_view_deg"],
            sensor["max_range_m"],
            sensor["angular_resolution_deg"],
        ),
        target_coverage=shared["target_sensor_coverage"],
        coverage_objective=shared["coverage_objective"],
        topological_radius=shared["topological_radius_m"],
        topological_merge_distance=shared["topological_merge_distance_m"],
        target_topological_coverage=shared["target_topological_coverage"],
        termination_mode=protocol["policy_contract"]["termination_mode"],
        online_exhaustion_confirmations=protocol["policy_contract"][
            "completion_confirmation"
        ]["consecutive_new_map_revisions"],
        information_gain_weight=shared["information_gain_weight"],
        topological_gain_weight=shared["topological_gain_weight"],
        max_decisions=protocol["fail_safes"]["max_decisions"],
        robot_radius=shared["robot_radius_m"],
        safety_margin=shared["safety_margin_m"],
        minimum_goal_clearance=shared["minimum_goal_clearance_m"],
        preferred_clearance=shared["preferred_clearance_m"],
        target_spacing=shared["target_spacing_m"],
        scan_interval=shared["scan_interval_m"],
        min_gain_cells=shared["min_gain_cells"],
        min_topological_gain_cells=shared["min_topological_gain_cells"],
        max_frontier_candidates=shared["max_frontier_candidates"],
        random_candidates=shared["random_candidates"],
        exact_gain_budget=shared["exact_gain_budget"],
        clearance_weight=shared["clearance_weight"],
        travel_cost_weight=shared["travel_cost_weight"],
        spacing_weight=shared["spacing_weight"],
        multi_frontier=shared["multi_frontier"],
        use_topological_vantages=shared["use_topological_vantages"],
        require_known_footprint=shared["require_known_footprint"],
    )

    assert asdict(config)["termination_mode"] == "candidate_exhaustion"
    assert config.robot_radius == 0.24
    assert config.minimum_goal_clearance == 0.40
    assert config.max_decisions == 80
    assert config.online_exhaustion_confirmations == 3
