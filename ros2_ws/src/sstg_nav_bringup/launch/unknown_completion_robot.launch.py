"""Hardware-agnostic unknown-completion bring-up (drivers run externally)."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


ROBOT_BAG_TOPICS = (
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
    "/policy/decision_trace",
    "/policy/status",
    "/policy/candidates",
)


def generate_launch_description():
    nav2_share = get_package_share_directory("nav2_bringup")
    slam_share = get_package_share_directory("slam_toolbox")
    policy_share = get_package_share_directory("sstg_policy_ros")
    bringup_share = get_package_share_directory("sstg_nav_bringup")

    output_dir = LaunchConfiguration("output_dir")
    nav2_params = LaunchConfiguration("nav2_params")
    slam_params = LaunchConfiguration("slam_params")
    policy_params = LaunchConfiguration("policy_params")
    strategy = LaunchConfiguration("strategy")
    policy_seed = LaunchConfiguration("policy_seed")
    max_duration_s = LaunchConfiguration("max_duration_s")
    max_distance_m = LaunchConfiguration("max_distance_m")
    max_decisions = LaunchConfiguration("max_decisions")
    goal_timeout_s = LaunchConfiguration("goal_timeout_s")
    record_bag = LaunchConfiguration("record_bag")
    rviz = LaunchConfiguration("rviz")

    default_nav2 = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_slam = os.path.join(bringup_share, "config", "slam_toolbox.yaml")
    default_policy = os.path.join(policy_share, "config", "policy.yaml")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": slam_params,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": nav2_params,
            "use_composition": "False",
        }.items(),
    )
    policy = Node(
        package="sstg_policy_ros",
        executable="policy_node",
        name="sstg_policy",
        output="screen",
        parameters=[
            policy_params,
            {
                "use_sim_time": False,
                "output_dir": ParameterValue(output_dir, value_type=str),
                "strategy": ParameterValue(strategy, value_type=str),
                "policy_seed": ParameterValue(policy_seed, value_type=int),
                "max_duration_s": ParameterValue(max_duration_s, value_type=float),
                "max_distance_m": ParameterValue(max_distance_m, value_type=float),
                "max_decisions": ParameterValue(max_decisions, value_type=int),
                "goal_timeout_s": ParameterValue(goal_timeout_s, value_type=float),
            },
        ],
    )
    recorder = ExecuteProcess(
        name="sstg_unknown_completion_robot_bag_recorder",
        cmd=[
            "ros2", "bag", "record", "--storage", "mcap",
            "--storage-preset-profile", "zstd_fast",
            "--output", PathJoinSubstitution([output_dir, "bags", "core"]),
            "--disable-keyboard-controls", "--include-hidden-topics",
            "--node-name", "sstg_unknown_completion_robot_bag_recorder",
            "--topics", *ROBOT_BAG_TOPICS,
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bringup_share, "rviz", "system_sim.rviz")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value="real_robot_outputs/unknown_completion/manual",
        ),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2),
        DeclareLaunchArgument("slam_params", default_value=default_slam),
        DeclareLaunchArgument("policy_params", default_value=default_policy),
        DeclareLaunchArgument("strategy", default_value="sstg"),
        DeclareLaunchArgument("policy_seed", default_value="42"),
        DeclareLaunchArgument("max_duration_s", default_value="900.0"),
        DeclareLaunchArgument("max_distance_m", default_value="150.0"),
        DeclareLaunchArgument("max_decisions", default_value="80"),
        DeclareLaunchArgument("goal_timeout_s", default_value="180.0"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        slam,
        navigation,
        policy,
        recorder,
        rviz_node,
    ])
