import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    evaluator_share = get_package_share_directory("sstg_system_eval")
    gazebo_share = get_package_share_directory("sstg_gazebo")
    default_params = os.path.join(evaluator_share, "config", "evaluator.yaml")
    default_truth = os.path.join(
        gazebo_share,
        "worlds",
        "development",
        "multi_room_office",
        "dev_office_01",
        "evaluation",
        "truth_map.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("truth_map_yaml", default_value=default_truth),
        DeclareLaunchArgument("targets_yaml", default_value=""),
        DeclareLaunchArgument(
            "truth_registration_id",
            default_value="dev_office_01:start_southwest:inverse_spawn_pose",
        ),
        DeclareLaunchArgument("truth_to_map_x_m", default_value="6.5"),
        DeclareLaunchArgument("truth_to_map_y_m", default_value="4.5"),
        DeclareLaunchArgument("truth_to_map_yaw_rad", default_value="0.0"),
        DeclareLaunchArgument(
            "output_dir",
            default_value="system_sim_outputs/runs/development/manual",
        ),
        DeclareLaunchArgument("allow_existing_output", default_value="false"),
        Node(
            package="sstg_system_eval",
            executable="system_eval_node",
            name="sstg_system_eval",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "truth_map_yaml": LaunchConfiguration("truth_map_yaml"),
                    "targets_yaml": LaunchConfiguration("targets_yaml"),
                    "truth_registration_id": LaunchConfiguration(
                        "truth_registration_id"
                    ),
                    "truth_to_map_x_m": ParameterValue(
                        LaunchConfiguration("truth_to_map_x_m"), value_type=float
                    ),
                    "truth_to_map_y_m": ParameterValue(
                        LaunchConfiguration("truth_to_map_y_m"), value_type=float
                    ),
                    "truth_to_map_yaw_rad": ParameterValue(
                        LaunchConfiguration("truth_to_map_yaw_rad"),
                        value_type=float,
                    ),
                    "output_dir": LaunchConfiguration("output_dir"),
                    "allow_existing_output": ParameterValue(
                        LaunchConfiguration("allow_existing_output"),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
