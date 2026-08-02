from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


CORE_BAG_STORAGE_ID = "mcap"
CORE_BAG_STORAGE_PRESET = "zstd_fast"
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


def generate_launch_description():
    gazebo_share = get_package_share_directory("sstg_gazebo")
    nav2_share = get_package_share_directory("nav2_bringup")
    policy_share = get_package_share_directory("sstg_policy_ros")
    baseline_adapter_share = get_package_share_directory(
        "sstg_baseline_adapter"
    )
    evaluator_share = get_package_share_directory("sstg_system_eval")
    slam_share = get_package_share_directory("slam_toolbox")
    bringup_share = get_package_share_directory("sstg_nav_bringup")

    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_z = LaunchConfiguration("start_z")
    start_yaw = LaunchConfiguration("start_yaw")
    output_dir = LaunchConfiguration("output_dir")
    record_bag = LaunchConfiguration("record_bag")
    nav2_params = LaunchConfiguration("nav2_params")
    slam_params = LaunchConfiguration("slam_params")
    policy_params = LaunchConfiguration("policy_params")
    runtime_adapter = LaunchConfiguration("runtime_adapter")
    evaluator_params = LaunchConfiguration("evaluator_params")
    strategy = LaunchConfiguration("strategy")
    coverage_objective = LaunchConfiguration("coverage_objective")
    clearance_weight = LaunchConfiguration("clearance_weight")
    travel_cost_weight = LaunchConfiguration("travel_cost_weight")
    policy_seed = LaunchConfiguration("policy_seed")
    simulation_seed = LaunchConfiguration("simulation_seed")
    max_duration_s = LaunchConfiguration("max_duration_s")
    max_distance_m = LaunchConfiguration("max_distance_m")
    max_decisions = LaunchConfiguration("max_decisions")
    goal_timeout_s = LaunchConfiguration("goal_timeout_s")
    evaluator_enabled = LaunchConfiguration("evaluator")
    truth_map_yaml = LaunchConfiguration("truth_map_yaml")
    truth_registration_id = LaunchConfiguration("truth_registration_id")
    truth_to_map_x_m = LaunchConfiguration("truth_to_map_x_m")
    truth_to_map_y_m = LaunchConfiguration("truth_to_map_y_m")
    truth_to_map_yaw_rad = LaunchConfiguration("truth_to_map_yaw_rad")

    default_world = os.path.join(
        gazebo_share,
        "worlds", "development", "multi_room_office", "dev_office_01", "world.sdf",
    )
    default_slam = os.path.join(bringup_share, "config", "slam_toolbox.yaml")
    default_nav2 = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_policy = os.path.join(policy_share, "config", "policy.yaml")
    default_evaluator = os.path.join(evaluator_share, "config", "evaluator.yaml")
    default_truth = os.path.join(
        gazebo_share,
        "worlds", "development", "multi_room_office", "dev_office_01",
        "evaluation", "truth_map.yaml",
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": world,
            "world_name": world_name,
            "headless": headless,
            "start_x": start_x,
            "start_y": start_y,
            "start_z": start_z,
            "start_yaw": start_yaw,
            "simulation_seed": simulation_seed,
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
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
                "output_dir": ParameterValue(output_dir, value_type=str),
                "strategy": ParameterValue(strategy, value_type=str),
                "coverage_objective": ParameterValue(
                    coverage_objective, value_type=str
                ),
                "clearance_weight": ParameterValue(
                    clearance_weight, value_type=float
                ),
                "travel_cost_weight": ParameterValue(
                    travel_cost_weight, value_type=float
                ),
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
        condition=IfCondition(
            EqualsSubstitution(runtime_adapter, "sstg_policy")
        ),
    )
    frontier_baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                baseline_adapter_share,
                "launch",
                "frontier_mrtsp_dp.launch.py",
            )
        ),
        launch_arguments={
            "output_dir": output_dir,
            "policy_seed": policy_seed,
            "max_duration_s": max_duration_s,
            "max_distance_m": max_distance_m,
            "max_decisions": max_decisions,
            "goal_timeout_s": goal_timeout_s,
        }.items(),
        condition=IfCondition(EqualsSubstitution(
            runtime_adapter, "frontier_mrtsp_dp_external"
        )),
    )
    evaluator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(evaluator_share, "launch", "evaluator.launch.py")
        ),
        launch_arguments={
            "evaluator_params_file": evaluator_params,
            "use_sim_time": "true",
            "truth_map_yaml": truth_map_yaml,
            "truth_registration_id": truth_registration_id,
            "truth_to_map_x_m": truth_to_map_x_m,
            "truth_to_map_y_m": truth_to_map_y_m,
            "truth_to_map_yaw_rad": truth_to_map_yaw_rad,
            "output_dir": output_dir,
        }.items(),
        condition=IfCondition(evaluator_enabled),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", os.path.join(bringup_share, "rviz", "system_sim.rviz")
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )
    core_bag_recorder = ExecuteProcess(
        name="sstg_core_bag_recorder",
        cmd=[
            "ros2",
            "bag",
            "record",
            "--storage",
            CORE_BAG_STORAGE_ID,
            "--storage-preset-profile",
            CORE_BAG_STORAGE_PRESET,
            "--output",
            PathJoinSubstitution([output_dir, "bags", "core"]),
            "--disable-keyboard-controls",
            "--use-sim-time",
            "--include-hidden-topics",
            "--node-name",
            "sstg_core_bag_recorder",
            "--topics",
            *CORE_BAG_TOPICS,
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("world_name", default_value="dev_office_01"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start_x", default_value="-6.5"),
        DeclareLaunchArgument("start_y", default_value="-4.5"),
        DeclareLaunchArgument("start_z", default_value="0.01"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "output_dir",
            default_value="system_sim_outputs/runs/development/manual",
        ),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2),
        DeclareLaunchArgument("slam_params", default_value=default_slam),
        DeclareLaunchArgument("policy_params", default_value=default_policy),
        DeclareLaunchArgument(
            "runtime_adapter",
            default_value="sstg_policy",
            choices=["sstg_policy", "frontier_mrtsp_dp_external"],
            description="frozen method-specific policy runtime adapter",
        ),
        DeclareLaunchArgument("evaluator_params", default_value=default_evaluator),
        DeclareLaunchArgument("strategy", default_value="sstg"),
        DeclareLaunchArgument("coverage_objective", default_value="joint"),
        DeclareLaunchArgument("clearance_weight", default_value="1.5"),
        DeclareLaunchArgument("travel_cost_weight", default_value="0.60"),
        DeclareLaunchArgument(
            "policy_seed",
            description="required frozen policy random-number seed",
        ),
        DeclareLaunchArgument(
            "simulation_seed",
            description="required frozen Gazebo random-number seed",
        ),
        DeclareLaunchArgument(
            "max_duration_s",
            description="required frozen policy duration budget",
        ),
        DeclareLaunchArgument(
            "max_distance_m",
            description="required frozen policy travel budget",
        ),
        DeclareLaunchArgument(
            "max_decisions",
            description="required frozen policy action budget",
        ),
        DeclareLaunchArgument(
            "goal_timeout_s",
            description="required frozen per-goal timeout",
        ),
        DeclareLaunchArgument("evaluator", default_value="true"),
        DeclareLaunchArgument("truth_map_yaml", default_value=default_truth),
        DeclareLaunchArgument(
            "truth_registration_id",
            default_value="dev_office_01:start_southwest:inverse_spawn_pose",
        ),
        DeclareLaunchArgument("truth_to_map_x_m", default_value="6.5"),
        DeclareLaunchArgument("truth_to_map_y_m", default_value="4.5"),
        DeclareLaunchArgument("truth_to_map_yaw_rad", default_value="0.0"),
        simulation,
        core_bag_recorder,
        slam,
        navigation,
        evaluator,
        policy,
        frontier_baseline,
        rviz_node,
    ])
