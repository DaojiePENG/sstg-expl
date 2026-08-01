"""Compose SSTG worlds with Nav2's released TurtleBot3 simulation."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from sstg_gazebo.instrumented_tb3 import prepare_instrumented_tb3_sdf


def _is_gazebo_process(action):
    """Match the Gazebo process action created by ros_gz_sim."""
    process_name = getattr(action.process_description, "final_name", None) or ""
    return process_name.startswith("gazebo-")


def _shutdown_on_unexpected_gazebo_exit(event, context):
    """Keep Gazebo required without emitting a second shutdown on SIGINT."""
    if context.is_shutdown:
        return None
    return Shutdown(
        reason=(
            "Gazebo exited before launch shutdown "
            f"(return code {event.returncode})"
        )
    )


def _gazebo_exit_guard():
    return RegisterEventHandler(
        OnProcessExit(
            target_action=_is_gazebo_process,
            on_exit=_shutdown_on_unexpected_gazebo_exit,
        )
    )


def _validated_simulation_seed(value):
    """Return a Gazebo-compatible deterministic RNG seed."""
    try:
        seed = int(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(
            "simulation_seed must be a positive signed 32-bit integer"
        ) from error
    if str(seed) != str(value).strip() or not 1 <= seed <= 0x7FFFFFFF:
        raise RuntimeError(
            "simulation_seed must be a positive signed 32-bit integer"
        )
    return seed


def _launch_setup(context):
    package_share = get_package_share_directory("sstg_gazebo")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    tb3_share = get_package_share_directory("nav2_minimal_tb3_sim")
    world = LaunchConfiguration("world").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    start_x = LaunchConfiguration("start_x").perform(context)
    start_y = LaunchConfiguration("start_y").perform(context)
    start_z = LaunchConfiguration("start_z").perform(context)
    start_yaw = LaunchConfiguration("start_yaw").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_sdf_xacro = LaunchConfiguration("robot_sdf_xacro").perform(context)
    simulation_seed = _validated_simulation_seed(
        LaunchConfiguration("simulation_seed").perform(context)
    )
    headless_args = "-s --headless-rendering " if headless else ""
    gz_args = f"-r {headless_args}-v 3 --seed {simulation_seed} {world}"
    world_stats_topic = f"/world/{world_name}/stats"
    prepared_robot = prepare_instrumented_tb3_sdf(
        robot_sdf_xacro,
        namespace="",
    )
    rendered_robot_sdf = str(prepared_robot.output_path)
    camera_sensor_root = (
        f"/world/{world_name}/model/{robot_name}/link/camera_link/sensor/"
        "intel_realsense_r200_depth"
    )
    depth_image_topic = f"{camera_sensor_root}/depth_image"
    camera_info_topic = f"{camera_sensor_root}/camera_info"

    # Robot description, primary bridge and spawn semantics come directly
    # from the released Nav2 TurtleBot3 package.
    upstream_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_share, "launch", "spawn_tb3.launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "robot_name": robot_name,
            "robot_sdf": rendered_robot_sdf,
            "x_pose": start_x,
            "y_pose": start_y,
            "z_pose": start_z,
            "roll": "0.0",
            "pitch": "0.0",
            "yaw": start_yaw,
        }.items(),
    )
    upstream_urdf = os.path.join(
        tb3_share, "urdf", "turtlebot3_waffle.urdf"
    )

    return [
        LogInfo(
            msg=(
                "Prepared instrumented upstream TB3 SDF: "
                f"{rendered_robot_sdf} "
                f"(xacro_sha256={prepared_robot.source_xacro_sha256}, "
                f"contacts={prepared_robot.instrumentation.contact_sensor_count}, "
                "imu_backport="
                f"{prepared_robot.instrumentation.imu_joint_backported}, "
                f"simulation_seed={simulation_seed})"
            )
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(tb3_share, "models")
        ),
        _gazebo_exit_guard(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": gz_args,
                # Upstream's unconditional Shutdown() runs even when Gazebo
                # exits because the enclosing launch is already stopping.
                "on_exit_shutdown": "false",
            }.items(),
        ),
        upstream_spawn,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": open(
                    upstream_urdf, encoding="utf-8"
                ).read(),
            }],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="sstg_evaluation_bridge",
            output="screen",
            parameters=[{
                "config_file": os.path.join(
                    package_share, "config", "evaluation_bridge.yaml"
                ),
                "use_sim_time": True,
            }],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="sstg_auxiliary_bridge",
            output="screen",
            arguments=[
                world_stats_topic
                + "@ros_gz_interfaces/msg/WorldStatistics[gz.msgs.WorldStatistics",
                camera_info_topic
                + "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                (world_stats_topic, "/evaluation/world_stats"),
                (camera_info_topic, "/task_camera/camera_info"),
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="sstg_task_camera_bridge",
            output="screen",
            arguments=[depth_image_topic],
            remappings=[(depth_image_topic, "/task_camera/image_raw")],
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("sstg_gazebo")
    tb3_share = get_package_share_directory("nav2_minimal_tb3_sim")
    default_world = os.path.join(
        package_share,
        "worlds", "development", "multi_room_office", "dev_office_01",
        "world.sdf",
    )
    default_robot_sdf = os.path.join(
        tb3_share, "urdf", "gz_waffle.sdf.xacro"
    )
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("world_name", default_value="dev_office_01"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("start_x", default_value="-6.5"),
        DeclareLaunchArgument("start_y", default_value="-4.5"),
        DeclareLaunchArgument("start_z", default_value="0.01"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("simulation_seed", default_value="1"),
        DeclareLaunchArgument(
            "robot_name", default_value="turtlebot3_waffle"
        ),
        DeclareLaunchArgument(
            "robot_sdf_xacro", default_value=default_robot_sdf
        ),
        OpaqueFunction(function=_launch_setup),
    ])
