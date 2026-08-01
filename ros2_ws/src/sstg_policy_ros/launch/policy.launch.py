from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory("sstg_policy_ros")
    default_params = os.path.join(package_share, "config", "policy.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("strategy", default_value="sstg"),
        DeclareLaunchArgument("coverage_objective", default_value="joint"),
        DeclareLaunchArgument("policy_seed", default_value="42"),
        DeclareLaunchArgument(
            "output_dir",
            default_value="system_sim_outputs/runs/development/manual",
        ),
        Node(
            package="sstg_policy_ros",
            executable="policy_node",
            name="sstg_policy",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "strategy": ParameterValue(
                        LaunchConfiguration("strategy"), value_type=str
                    ),
                    "coverage_objective": ParameterValue(
                        LaunchConfiguration("coverage_objective"), value_type=str
                    ),
                    "policy_seed": ParameterValue(
                        LaunchConfiguration("policy_seed"), value_type=int
                    ),
                    "output_dir": ParameterValue(
                        LaunchConfiguration("output_dir"), value_type=str
                    ),
                },
            ],
        ),
    ])
