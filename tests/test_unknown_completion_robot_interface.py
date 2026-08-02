from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_robot_interface_is_a_driver_only_replacement_boundary():
    path = ROOT / "experiments/system_sim/configs/unknown_completion_robot_interface.yaml"
    contract = yaml.safe_load(path.read_text(encoding="utf-8"))

    assert contract["schema"] == "sstg_unknown_completion_robot_interface/v1"
    assert contract["policy_boundary"]["direct_scan_access"] is False
    assert contract["policy_boundary"]["direct_imu_access"] is False
    assert contract["policy_boundary"]["direct_cmd_vel_access"] is False
    assert contract["real_robot_launch"]["launches_gazebo"] is False
    assert contract["real_robot_launch"]["launches_truth_evaluator"] is False
    assert set(contract["driver_replacement_only"]["unchanged_components"]) >= {
        "slam_toolbox", "nav2", "sstg_policy_ros", "rosbag2"
    }


def test_real_robot_launch_has_standard_ros2_io_and_no_simulator_or_truth():
    path = ROOT / "ros2_ws/src/sstg_nav_bringup/launch/unknown_completion_robot.launch.py"
    source = path.read_text(encoding="utf-8")

    for topic in ("/scan", "/imu", "/odom", "/cmd_vel", "/map"):
        assert f'"{topic}"' in source
    assert "nav2_bringup" in source
    assert "slam_toolbox" in source
    assert "sstg_policy_ros" in source
    assert "sstg_gazebo" not in source
    assert "sstg_system_eval" not in source
    assert "use_sim_time\": False" in source


def test_slam_range_matches_unknown_completion_sensor_profile():
    protocol = yaml.safe_load(
        (ROOT / "experiments/system_sim/configs/unknown_completion.yaml").read_text(
            encoding="utf-8"
        )
    )
    slam = yaml.safe_load(
        (ROOT / "ros2_ws/src/sstg_nav_bringup/config/slam_toolbox.yaml").read_text(
            encoding="utf-8"
        )
    )["slam_toolbox"]["ros__parameters"]

    assert slam["max_laser_range"] == protocol["sensor_adaptation"]["max_range_m"]
    assert slam["scan_buffer_maximum_scan_distance"] == protocol["sensor_adaptation"]["max_range_m"]
