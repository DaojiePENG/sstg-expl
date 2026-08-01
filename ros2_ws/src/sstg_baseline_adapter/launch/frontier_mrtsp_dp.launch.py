from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    adapter_share = get_package_share_directory("sstg_baseline_adapter")
    upstream_share = get_package_share_directory("frontier_exploration_ros2")

    adapter_params = LaunchConfiguration("adapter_params")
    upstream_params = LaunchConfiguration("upstream_params")
    output_dir = LaunchConfiguration("output_dir")
    policy_seed = LaunchConfiguration("policy_seed")
    max_duration_s = LaunchConfiguration("max_duration_s")
    max_distance_m = LaunchConfiguration("max_distance_m")
    max_decisions = LaunchConfiguration("max_decisions")
    goal_timeout_s = LaunchConfiguration("goal_timeout_s")

    default_adapter_params = os.path.join(
        adapter_share, "config", "frontier_mrtsp_dp.yaml"
    )
    default_upstream_params = os.path.join(
        upstream_share, "config", "params.yaml"
    )

    upstream = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            upstream_params,
            adapter_params,
            {"use_sim_time": True},
        ],
    )
    adapter = Node(
        package="sstg_baseline_adapter",
        executable="frontier_action_adapter",
        name="frontier_baseline_adapter",
        output="screen",
        parameters=[
            adapter_params,
            {
                "output_dir": ParameterValue(output_dir, value_type=str),
                # Recorded for pairing provenance only. The pinned upstream
                # algorithm exposes no policy RNG and does not consume it.
                "policy_seed": ParameterValue(policy_seed, value_type=int),
                "max_duration_s": ParameterValue(
                    max_duration_s, value_type=float
                ),
                "max_distance_m": ParameterValue(
                    max_distance_m, value_type=float
                ),
                "max_decisions": ParameterValue(
                    max_decisions, value_type=int
                ),
                "goal_timeout_s": ParameterValue(
                    goal_timeout_s, value_type=float
                ),
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "adapter_params", default_value=default_adapter_params
        ),
        DeclareLaunchArgument(
            "upstream_params", default_value=default_upstream_params
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="system_sim_outputs/runs/development/manual",
        ),
        DeclareLaunchArgument("policy_seed"),
        DeclareLaunchArgument("max_duration_s"),
        DeclareLaunchArgument("max_distance_m"),
        DeclareLaunchArgument("max_decisions"),
        DeclareLaunchArgument("goal_timeout_s"),
        upstream,
        adapter,
    ])
