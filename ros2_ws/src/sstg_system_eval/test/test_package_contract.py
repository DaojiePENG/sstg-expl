import math
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[2]


def test_evaluator_config_keeps_truth_outputs_out_of_policy_namespace():
    config = yaml.safe_load(
        (PACKAGE / "config/evaluator.yaml").read_text(encoding="utf-8")
    )["sstg_system_eval"]["ros__parameters"]
    topic_access = yaml.safe_load(
        (
            REPOSITORY / "experiments/system_sim/configs/topic_access.yaml"
        ).read_text(encoding="utf-8")
    )

    assert config["map_topic"] == "/map"
    assert config["use_sim_time"] is True
    assert config["trace_topic"].startswith("/policy/")
    assert config["metrics_topic"].startswith("/evaluation/")
    assert config["status_topic"].startswith("/evaluation/")
    assert config["ground_truth_odom_topic"].startswith("/evaluation/")
    assert config["contacts_topic"].startswith("/evaluation/")
    assert config["world_stats_topic"].startswith("/evaluation/")
    assert config["path_frame"] == "odom"
    assert config["topological_radius_m"] == 2.0
    assert config["metrics_topic"] in topic_access["evaluator"]["may_read"] or (
        "/evaluation/*" in topic_access["evaluator"]["may_read"]
    )
    forbidden = tuple(topic_access["policy"]["forbidden_prefixes"])
    assert config["metrics_topic"].startswith(forbidden)
    assert config["status_topic"].startswith(forbidden)
    assert config["ground_truth_odom_topic"].startswith(forbidden)
    assert config["contacts_topic"].startswith(forbidden)
    assert config["world_stats_topic"].startswith(forbidden)
    assert config["robot_clearance_radius_m"] > 0.0
    assert config["allow_existing_output"] is False
    assert (
        config["world_stats_topic"]
        in topic_access["evaluator"]["may_read"]
        or "/evaluation/*" in topic_access["evaluator"]["may_read"]
    )


def test_node_has_no_policy_publications_or_command_interfaces():
    source = (PACKAGE / "sstg_system_eval/evaluator_node.py").read_text(
        encoding="utf-8"
    )

    assert 'create_publisher(\n            String, self.metrics_topic' in source
    assert 'create_publisher(\n            String, self.status_topic' in source
    assert "create_service(" not in source
    assert "ActionClient(" not in source
    assert '"policy_topics_published": []' in source
    assert "Contacts, self.contacts_topic" in source
    assert "WorldStatistics," in source
    assert "self.world_stats_topic" in source
    assert "Odometry," in source
    assert "self.ground_truth_odom_topic" in source
    assert '"primary_travel_metric": "ground_truth_path_length_m"' in source
    assert "create_subscription(\n            OccupancyGrid" in source
    assert "create_subscription(\n            String, self.trace_topic" in source
    assert "/cmd_vel" not in source
    assert "NavigateToPose" not in source
    assert "/task_camera" not in source
    assert "sensor_msgs" not in source


def test_manifest_declares_truth_metrics_and_proxy_limitations():
    source = (PACKAGE / "sstg_system_eval/evaluator_node.py").read_text(
        encoding="utf-8"
    )
    metrics = (PACKAGE / "sstg_system_eval/metrics.py").read_text(
        encoding="utf-8"
    )

    assert '"schema": "sstg_system_sim_evaluator_manifest/v2"' in source
    assert '"truth_alignment": "T_map_truth"' in source
    assert '"model": "deterministic_geometry_proxy_v1"' in source
    assert '"image_detector": False' in source
    assert '"raw_metric": "raw_static_obstacle_distance_*_m"' in source
    assert '"source": self.world_stats_topic' in source
    assert "name-token attribution" in metrics
    assert "2-D truth occupancy makes LOS conservative" in metrics


def test_evaluator_launch_derives_targets_and_rejects_output_reuse_by_default():
    source = (PACKAGE / "launch/evaluator.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"evaluator_params_file", default_value=default_params' in source
    assert 'LaunchConfiguration("evaluator_params_file")' in source
    assert 'DeclareLaunchArgument("params_file"' not in source
    assert 'DeclareLaunchArgument("targets_yaml", default_value="")' in source
    assert 'DeclareLaunchArgument("use_sim_time", default_value="true")' in source
    assert '"use_sim_time": ParameterValue(' in source
    assert (
        'DeclareLaunchArgument("allow_existing_output", default_value="false")'
        in source
    )
    assert '"allow_existing_output": ParameterValue(' in source
    assert "default_targets" not in source


def test_evaluator_fails_closed_without_simulation_time():
    source = (PACKAGE / "sstg_system_eval/evaluator_node.py").read_text(
        encoding="utf-8"
    )

    assert 'self.get_parameter("use_sim_time").value' in source
    assert "use_sim_time must be true for timestamp-paired simulation metrics" in source


def test_frozen_camera_proxy_has_valid_frozen_geometry():
    config = yaml.safe_load(
        (PACKAGE / "config/evaluator.yaml").read_text(encoding="utf-8")
    )["sstg_system_eval"]["ros__parameters"]
    for name in (
        "camera_x_offset_m",
        "camera_y_offset_m",
        "camera_height_m",
        "camera_yaw_offset_rad",
        "camera_pitch_rad",
        "camera_horizontal_fov_rad",
        "camera_vertical_fov_rad",
        "camera_minimum_range_m",
        "camera_maximum_range_m",
    ):
        assert math.isfinite(config[name])
    assert 0.0 < config["camera_horizontal_fov_rad"] < 2.0 * math.pi
    assert 0.0 < config["camera_vertical_fov_rad"] < math.pi
    assert 0.0 <= config["camera_minimum_range_m"]
    assert (
        config["camera_maximum_range_m"]
        > config["camera_minimum_range_m"]
    )


def test_clearance_radius_is_positive_and_frozen_in_config():
    config = yaml.safe_load(
        (PACKAGE / "config/evaluator.yaml").read_text(encoding="utf-8")
    )["sstg_system_eval"]["ros__parameters"]
    assert config["robot_clearance_radius_m"] == 0.24
