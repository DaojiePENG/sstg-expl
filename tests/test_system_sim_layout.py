"""Static integrity gates for the embodied-simulation source tree."""
from collections import deque
import hashlib
import importlib.util
import math
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from xml.etree import ElementTree

import numpy as np
import pytest
import yaml

from ros2_ws.src.sstg_gazebo.sstg_gazebo import instrumented_tb3
from ros2_ws.src.sstg_gazebo.sstg_gazebo.instrumented_tb3 import (
    CONTACT_SENSOR_PREFIX,
    CONTACT_TOPIC,
    GROUND_TRUTH_ODOM_TOPIC,
    IMU_BACKPORT_COMMIT,
    InstrumentationError,
    UPSTREAM_RELEASE_XACRO_SHA256,
    instrument_rendered_tb3_sdf,
    prepare_instrumented_tb3_sdf,
    render_tb3_xacro,
)
from scripts.generate_gazebo_scene_world import generate as generate_scene_world
from scripts.generate_gazebo_truth_map import collision_boxes, rasterize


ROOT = Path(__file__).resolve().parents[1]
WORLD_BUNDLE = (
    ROOT / "ros2_ws/src/sstg_gazebo/worlds/development/"
    "multi_room_office/dev_office_01"
)
SIM_LAUNCH_PATH = ROOT / "ros2_ws/src/sstg_gazebo/launch/sim.launch.py"
SYSTEM_LAUNCH_PATH = (
    ROOT / "ros2_ws/src/sstg_nav_bringup/launch/system_sim.launch.py"
)
RVIZ_PATH = ROOT / "ros2_ws/src/sstg_nav_bringup/rviz/system_sim.rviz"
SLAM_PARAMS_PATH = (
    ROOT / "ros2_ws/src/sstg_nav_bringup/config/slam_toolbox.yaml"
)


RENDERED_TB3_FIXTURE = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="turtlebot3_waffle">
    <pose>0 0 0 0 0 0</pose>
    <link name="base_footprint"/>
    <link name="base_link">
      <inertial><mass>1.0</mass></inertial>
      <collision name="base_collision">
        <pose>-0.064 0 0.048 0 0 0</pose>
        <geometry><box><size>0.265 0.265 0.089</size></box></geometry>
      </collision>
      <visual name="base_visual">
        <geometry><mesh><uri>package://upstream/waffle_base.dae</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>
      </visual>
    </link>
    <link name="imu_link">
      <sensor name="tb3_imu" type="imu">
        <always_on>true</always_on><topic>/imu</topic>
      </sensor>
    </link>
    <link name="base_scan">
      <collision name="lidar_sensor_collision">
        <geometry><cylinder><radius>0.0508</radius><length>0.055</length></cylinder></geometry>
      </collision>
      <sensor name="hls_lfcd_lds" type="gpu_lidar">
        <topic>/scan</topic><ray><range><max>20.0</max></range></ray>
      </sensor>
    </link>
    <joint name="base_joint" type="fixed">
      <parent>base_footprint</parent><child>base_link</child>
    </joint>
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent><child>base_scan</child>
    </joint>
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.287</wheel_separation>
      <wheel_radius>0.033</wheel_radius>
      <max_linear_velocity>0.46</max_linear_velocity>
      <topic>/cmd_vel</topic>
    </plugin>
  </model>
</sdf>
"""


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_sim_launch_module():
    pytest.importorskip("launch")
    package_source = ROOT / "ros2_ws/src/sstg_gazebo"
    sys.path.insert(0, str(package_source))
    try:
        spec = importlib.util.spec_from_file_location(
            "sstg_gazebo_sim_launch_test", SIM_LAUNCH_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(package_source))


def _load_system_launch_module():
    pytest.importorskip("launch")
    spec = importlib.util.spec_from_file_location(
        "sstg_system_sim_launch_test", SYSTEM_LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _xml_signature(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_xml_signature(child) for child in list(element)),
    )


def _world_cell(metadata, x_m, y_m):
    resolution = float(metadata["truth_resolution_m"])
    origin_x = float(metadata["origin_m"]["x"])
    origin_y = float(metadata["origin_m"]["y"])
    return (
        int(math.floor((float(y_m) - origin_y) / resolution)),
        int(math.floor((float(x_m) - origin_x) / resolution)),
    )


def _inflate_occupied(occupied, resolution, clearance_m):
    """Conservatively inflate occupied cell centers without SciPy."""
    inflated = occupied.copy()
    radius_cells = int(math.ceil(clearance_m / resolution))
    height, width = occupied.shape
    for row_delta in range(-radius_cells, radius_cells + 1):
        for column_delta in range(-radius_cells, radius_cells + 1):
            if math.hypot(row_delta, column_delta) * resolution > clearance_m:
                continue
            source_rows = slice(max(0, -row_delta), min(height, height - row_delta))
            source_columns = slice(
                max(0, -column_delta), min(width, width - column_delta)
            )
            target_rows = slice(max(0, row_delta), min(height, height + row_delta))
            target_columns = slice(
                max(0, column_delta), min(width, width + column_delta)
            )
            inflated[target_rows, target_columns] |= occupied[
                source_rows, source_columns
            ]
    return inflated


def _reachable_cells(traversable, start):
    height, width = traversable.shape
    visited = np.zeros_like(traversable, dtype=bool)
    pending = deque([start])
    visited[start] = True
    while pending:
        row, column = pending.popleft()
        for row_delta, column_delta in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            neighbor = (row + row_delta, column + column_delta)
            if not (0 <= neighbor[0] < height and 0 <= neighbor[1] < width):
                continue
            if traversable[neighbor] and not visited[neighbor]:
                visited[neighbor] = True
                pending.append(neighbor)
    return visited


def test_system_sim_yaml_and_xml_are_parseable():
    bases = [ROOT / "experiments/system_sim", ROOT / "ros2_ws/src"]
    vendor_root = ROOT / "ros2_ws/src/third_party"
    for base in bases:
        for path in base.rglob("*.yaml"):
            if path.is_relative_to(vendor_root):
                continue
            yaml.safe_load(path.read_text(encoding="utf-8"))
        for pattern in ("*.xml", "*.sdf"):
            for path in base.rglob(pattern):
                if path.is_relative_to(vendor_root):
                    continue
                ElementTree.parse(path)


def test_policy_topic_allowlist_excludes_evaluator_truth():
    access = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/topic_access.yaml"
    ).read_text(encoding="utf-8"))
    subscriptions = access["policy"]["subscriptions"]
    assert not any(topic.startswith("/evaluation/") for topic in subscriptions)
    assert "/map" in subscriptions


def test_shared_slam_profile_excludes_observed_corridor_alias_jumps():
    params = yaml.safe_load(SLAM_PARAMS_PATH.read_text(encoding="utf-8"))[
        "slam_toolbox"
    ]["ros__parameters"]
    stack = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    mapping = stack["shared_autonomy"]

    assert mapping["mapping_profile"] == "corridor_alias_hardened_v1"
    assert mapping["mapping_profile_rationale"][
        "applies_equally_to_all_methods"
    ] is True
    observed = mapping["mapping_profile_rationale"][
        "observed_false_translation_corrections_m"
    ]
    assert params["do_loop_closing"] is True
    assert params["loop_search_maximum_distance"] < min(observed)
    assert params["loop_search_space_dimension"] <= 2.0 * params[
        "loop_search_maximum_distance"
    ]
    assert params["loop_match_minimum_chain_size"] >= 20
    assert params["loop_match_maximum_variance_coarse"] <= 1.0
    assert params["loop_match_minimum_response_coarse"] >= 0.45
    assert params["loop_match_minimum_response_fine"] >= 0.55


def test_released_nav2_turtlebot_is_the_default_robot():
    stack = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    robot = stack["robot"]
    source = SIM_LAUNCH_PATH.read_text(encoding="utf-8")
    package = ElementTree.parse(
        ROOT / "ros2_ws/src/sstg_gazebo/package.xml"
    ).getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    buildtools = {node.text for node in package.findall("buildtool_depend")}
    cmake = (
        ROOT / "ros2_ws/src/sstg_gazebo/CMakeLists.txt"
    ).read_text(encoding="utf-8")

    assert robot["profile"] == "nav2_minimal_turtlebot3_waffle"
    assert robot["upstream_package"] == "nav2_minimal_tb3_sim"
    assert robot["upstream_version"] == "1.0.1-1noble.20260616.074421"
    assert robot["upstream_license"] == "Apache-2.0"
    assert "nav2_minimal_tb3_sim" in dependencies
    assert "xacro" in dependencies
    assert "ament_cmake_python" in buildtools
    assert "ament_python_install_package(${PROJECT_NAME})" in cmake
    assert "install(FILES UPSTREAM.md" in cmake
    assert "install(DIRECTORY config launch worlds" in cmake
    assert 'get_package_share_directory("nav2_minimal_tb3_sim")' in source
    assert '"launch", "spawn_tb3.launch.py"' in source
    assert '"urdf", "gz_waffle.sdf.xacro"' in source
    assert "prepare_instrumented_tb3_sdf(" in source
    assert "ExecuteProcess" not in source
    assert "tb3_evaluation_overlay" not in source
    assert "spawn_sstg_diffbot" not in source
    assert not (
        ROOT / "ros2_ws/src/sstg_gazebo/models/sstg_diffbot"
    ).exists()
    assert not (ROOT / "ros2_ws/src/sstg_gazebo/models").exists()


def test_runtime_tb3_derivative_has_only_allowlisted_instrumentation():
    upstream = ElementTree.fromstring(RENDERED_TB3_FIXTURE)
    upstream_model = upstream.find("model")
    original_collisions = [
        _xml_signature(node) for node in upstream_model.findall("link/collision")
    ]
    original_sensors = [
        _xml_signature(node) for node in upstream_model.findall("link/sensor")
    ]
    original_meshes = [
        _xml_signature(node) for node in upstream_model.findall(".//mesh")
    ]
    original_diff_drive = _xml_signature(upstream_model.find(
        "plugin[@name='gz::sim::systems::DiffDrive']"
    ))

    result = instrument_rendered_tb3_sdf(RENDERED_TB3_FIXTURE)
    derivative = ElementTree.fromstring(result.xml)
    model = derivative.find("model")

    assert result.upstream_structure_preserved is True
    assert result.imu_joint_backported is True
    assert result.collision_count == 2
    assert result.contact_sensor_count == result.collision_count
    assert [
        _xml_signature(node) for node in model.findall("link/collision")
    ] == original_collisions
    assert [
        _xml_signature(node) for node in model.findall("link/sensor")
        if not node.get("name", "").startswith(CONTACT_SENSOR_PREFIX)
    ] == original_sensors
    assert [
        _xml_signature(node) for node in model.findall(".//mesh")
    ] == original_meshes
    assert _xml_signature(model.find(
        "plugin[@name='gz::sim::systems::DiffDrive']"
    )) == original_diff_drive

    for link in model.findall("link"):
        collisions = {node.get("name") for node in link.findall("collision")}
        contacts = {
            sensor.findtext("contact/collision")
            for sensor in link.findall("sensor[@type='contact']")
            if sensor.get("name", "").startswith(CONTACT_SENSOR_PREFIX)
        }
        assert contacts == collisions
        for sensor in link.findall("sensor[@type='contact']"):
            assert sensor.findtext("contact/topic") == CONTACT_TOPIC

    imu_joint = model.find("joint[@name='imu_joint']")
    assert IMU_BACKPORT_COMMIT == "b9d523ad7ea0e98174627fdefeb4b1ae9b515063"
    assert imu_joint.get("type") == "fixed"
    assert imu_joint.findtext("parent") == "base_link"
    assert imu_joint.findtext("child") == "imu_link"
    assert imu_joint.findtext("pose") == "0.0 0 0.068 0 0 0"
    odometry = model.find(
        "plugin[@name='gz::sim::systems::OdometryPublisher']"
    )
    assert odometry.findtext("odom_topic") == GROUND_TRUTH_ODOM_TOPIC
    assert model.find(
        "plugin[@name='gz::sim::systems::DetachableJoint']"
    ) is None


def test_instrumenter_rejects_a_different_model_identity():
    unexpected = RENDERED_TB3_FIXTURE.replace(
        'model name="turtlebot3_waffle"', 'model name="locally_authored_robot"'
    )
    with pytest.raises(InstrumentationError, match="not the audited upstream"):
        instrument_rendered_tb3_sdf(unexpected)


def test_prepare_rejects_an_unfrozen_xacro_before_execution(
    monkeypatch, tmp_path
):
    source = tmp_path / "gz_waffle.sdf.xacro"
    source.write_text("not the frozen upstream xacro", encoding="utf-8")
    called = False

    def must_not_run(*args, **kwargs):
        nonlocal called
        called = True
        raise AssertionError("xacro must not run after a source hash mismatch")

    monkeypatch.setattr(instrumented_tb3.subprocess, "run", must_not_run)
    with pytest.raises(InstrumentationError, match="differs from the frozen"):
        prepare_instrumented_tb3_sdf(source, output_directory=tmp_path)
    assert called is False
    assert len(UPSTREAM_RELEASE_XACRO_SHA256) == 64


def test_runtime_tb3_derivative_keeps_an_existing_upstream_imu_joint():
    upstream = RENDERED_TB3_FIXTURE.replace(
        "    <plugin filename=\"gz-sim-diff-drive-system\"",
        '''    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent><child>imu_link</child>
      <pose>0.0 0 0.068 0 0 0</pose>
    </joint>
    <plugin filename="gz-sim-diff-drive-system"''',
        1,
    )

    result = instrument_rendered_tb3_sdf(upstream)
    model = ElementTree.fromstring(result.xml).find("model")
    joints = model.findall("joint[@name='imu_joint']")

    assert result.imu_joint_backported is False
    assert len(joints) == 1
    assert joints[0].findtext("pose") == "0.0 0 0.068 0 0 0"


def test_xacro_render_helper_is_synchronous_and_passes_namespace(
    monkeypatch, tmp_path
):
    source = tmp_path / "gz_waffle.sdf.xacro"
    executable = tmp_path / "xacro"
    source.write_text("upstream xacro fixture", encoding="utf-8")
    executable.write_text("test executable fixture", encoding="utf-8")
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(
            command, 0, stdout=RENDERED_TB3_FIXTURE.encode("utf-8")
        )

    monkeypatch.setattr(instrumented_tb3.subprocess, "run", fake_run)
    monkeypatch.setenv("SSTG_XACRO_INHERITED", "inherited")
    rendered = render_tb3_xacro(
        source,
        namespace="robot_1",
        xacro_executable=executable,
        environment={"SSTG_XACRO_OVERRIDE": "override"},
    )

    assert rendered == RENDERED_TB3_FIXTURE.encode("utf-8")
    assert calls[0][0] == [
        str(executable), str(source), "namespace:=robot_1"
    ]
    assert calls[0][1]["check"] is True
    assert calls[0][1]["capture_output"] is True
    assert calls[0][1]["env"]["SSTG_XACRO_INHERITED"] == "inherited"
    assert calls[0][1]["env"]["SSTG_XACRO_OVERRIDE"] == "override"


def test_installed_upstream_tb3_derivative_preserves_release_contract(tmp_path):
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    except (ImportError, LookupError, EnvironmentError):
        pytest.skip("nav2_minimal_tb3_sim is not installed")
    xacro_executable = shutil.which("xacro")
    if not xacro_executable:
        pytest.skip("xacro executable is not installed")

    prepared = prepare_instrumented_tb3_sdf(
        share / "urdf/gz_waffle.sdf.xacro",
        output_directory=tmp_path,
        xacro_executable=xacro_executable,
    )
    report = prepared.instrumentation
    model = ElementTree.parse(prepared.output_path).getroot().find("model")

    assert report.upstream_structure_preserved is True
    assert prepared.source_xacro_sha256 == UPSTREAM_RELEASE_XACRO_SHA256
    assert report.contact_sensor_count == report.collision_count == 7
    assert report.imu_joint_backported is True
    assert model.find("joint[@name='imu_joint']") is not None
    assert model.find(
        "plugin[@name='gz::sim::systems::OdometryPublisher']"
    ) is not None


def test_floor_surface_and_default_spawn_match_upstream_tb3_contract():
    world = ElementTree.parse(WORLD_BUNDLE / "world.sdf").getroot()
    floor = world.find(".//model[@name='floor']")
    floor_pose = [float(value) for value in floor.findtext("pose").split()]
    floor_size = [
        float(value)
        for value in floor.findtext("link/collision/geometry/box/size").split()
    ]
    source = SIM_LAUNCH_PATH.read_text(encoding="utf-8")

    assert math.isclose(
        floor_pose[2] + floor_size[2] / 2.0, 0.0, abs_tol=1e-12
    )
    assert 'DeclareLaunchArgument("start_z", default_value="0.01")' in source


def test_headless_launch_keeps_gpu_sensor_rendering_and_dynamic_world_stats():
    source = SIM_LAUNCH_PATH.read_text(encoding="utf-8")
    world = ElementTree.parse(WORLD_BUNDLE / "world.sdf").getroot().find("world")

    assert 'headless_args = "-s --headless-rendering " if headless else ""' in source
    assert 'LaunchConfiguration("world_name")' in source
    assert (
        f'DeclareLaunchArgument("world_name", default_value="{world.get("name")}")'
        in source
    )
    assert 'world_stats_topic = f"/world/{world_name}/stats"' in source
    assert (
        '"@ros_gz_interfaces/msg/WorldStatistics[gz.msgs.WorldStatistics"'
        in source
    )
    assert '(world_stats_topic, "/evaluation/world_stats")' in source
    assert "--seed {simulation_seed}" in source
    assert "_validated_simulation_seed" in source
    assert 'name="sstg_world_stats_bridge"' in source
    assert "CameraInfo" not in source
    assert 'name="sstg_task_camera_info_bridge"' not in source


def test_simulation_seed_validation_is_fail_closed():
    module = _load_sim_launch_module()

    assert module._validated_simulation_seed("1") == 1
    assert module._validated_simulation_seed("2147483647") == 0x7FFFFFFF
    for invalid in ("-1", "0", "1.0", "2147483648", "4294967295", "seed"):
        with pytest.raises(RuntimeError, match="positive signed 32-bit"):
            module._validated_simulation_seed(invalid)


def test_gazebo_world_path_is_shell_quoted_for_upstream_launch():
    module = _load_sim_launch_module()
    world = "/tmp/system sim/world's scene.sdf"

    arguments = module._gazebo_arguments(
        world, headless=True, simulation_seed=103
    )

    assert shlex.split(arguments)[-1] == world
    assert shlex.split(arguments)[:-1] == [
        "-r", "-s", "--headless-rendering", "-v", "3", "--seed", "103"
    ]


def test_gazebo_exit_guard_replaces_upstream_unconditional_shutdown():
    source = SIM_LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"on_exit_shutdown": "false"' in source
    assert '"on_exit_shutdown": "true"' not in source
    assert "OnProcessExit" in source
    assert "if context.is_shutdown:" in source


def test_coordinated_launch_shutdown_does_not_shutdown_ros_twice():
    module = _load_sim_launch_module()
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.actions import Shutdown, TimerAction
    from launch.event_handlers import OnShutdown

    shutdown_reasons = []

    def shutdown_non_idempotent_adapter(event, context):
        shutdown_reasons.append(event.reason)
        if len(shutdown_reasons) > 1:
            raise RuntimeError(
                "Cannot shutdown a ROS adapter that is not running"
            )

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnShutdown(
            on_shutdown=shutdown_non_idempotent_adapter
        )),
        module._gazebo_exit_guard(),
        ExecuteProcess(
            cmd=[sys.executable, "-c", "import time; time.sleep(60)"],
            name="gazebo",
        ),
        TimerAction(
            period=0.1,
            actions=[Shutdown(reason="coordinated test shutdown")],
        ),
    ]))

    assert service.run() == 0
    assert shutdown_reasons == ["coordinated test shutdown"]


def test_unexpected_gazebo_exit_still_shuts_down_the_launch():
    module = _load_sim_launch_module()
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.actions import Shutdown, TimerAction
    from launch.event_handlers import OnShutdown

    shutdown_reasons = []
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: shutdown_reasons.append(
                event.reason
            )
        )),
        module._gazebo_exit_guard(),
        ExecuteProcess(
            cmd=[sys.executable, "-c", "raise SystemExit(7)"],
            name="gazebo",
        ),
        ExecuteProcess(
            cmd=[sys.executable, "-c", "import time; time.sleep(60)"],
            name="sentinel",
        ),
        TimerAction(
            period=2.0,
            actions=[Shutdown(reason="unexpected-exit guard timed out")],
        ),
    ]))

    assert service.run() == 0
    assert shutdown_reasons == [
        "Gazebo exited before launch shutdown (return code 7)"
    ]


def test_target_panel_pose_and_local_x_normal_match_registry():
    targets = yaml.safe_load((WORLD_BUNDLE / "targets.yaml").read_text())
    world = ElementTree.parse(WORLD_BUNDLE / "world.sdf").getroot()
    models = {
        model.get("name"): model
        for model in world.findall("world/model")
        if model.get("name", "").startswith("target_panel_")
    }
    assert set(models) == {entry["target_id"] for entry in targets["targets"]}
    for entry in targets["targets"]:
        model = models[entry["target_id"]]
        pose = [float(value) for value in model.findtext("pose").split()]
        assert pose[:3] == [entry["x_m"], entry["y_m"], entry["z_m"]]
        expected_yaw = math.radians(entry["surface_normal_yaw_deg"])
        yaw_error = math.atan2(
            math.sin(pose[5] - expected_yaw), math.cos(pose[5] - expected_yaw)
        )
        assert math.isclose(yaw_error, 0.0, abs_tol=1e-6)
        size = [
            float(value)
            for value in model.findtext("link/visual/geometry/box/size").split()
        ]
        assert size[0] < min(size[1:])


def test_development_world_is_not_formal_evidence():
    metadata = yaml.safe_load((
        WORLD_BUNDLE / "metadata.yaml"
    ).read_text(encoding="utf-8"))
    assert metadata["split"] == "development"
    assert metadata["formal_result_eligible"] is False
    assert metadata["truth_access"] == "evaluator_only"


def test_evaluator_truth_map_is_frozen_against_world_bundle():
    manifest = yaml.safe_load((
        WORLD_BUNDLE / "evaluation/truth_map_manifest.yaml"
    ).read_text(encoding="utf-8"))
    assert manifest["shape"] == [240, 320]
    assert manifest["occupied_cells"] > 0
    assert manifest["free_cells"] > manifest["occupied_cells"]
    assert manifest["target_count"] == 4
    for relative, expected in manifest["sha256"].items():
        path = (
            WORLD_BUNDLE / "evaluation" / relative
            if relative.startswith("truth_map") else WORLD_BUNDLE / relative
        )
        assert _sha256(path) == expected


def test_policy_cannot_read_ground_truth_pose_or_contacts():
    access = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/topic_access.yaml"
    ).read_text(encoding="utf-8"))
    bridge = yaml.safe_load((
        ROOT / "ros2_ws/src/sstg_gazebo/config/evaluation_bridge.yaml"
    ).read_text(encoding="utf-8"))
    bridged = {entry["ros_topic_name"] for entry in bridge}
    assert bridged == {
        "/evaluation/ground_truth_odom", "/evaluation/contacts"
    }
    assert "/evaluation/" in access["policy"]["forbidden_prefixes"]
    evaluator_truth = bridged | {"/evaluation/world_stats"}
    assert not evaluator_truth.intersection(access["policy"]["subscriptions"])


def test_system_rviz_exposes_policy_sensor_and_navigation_state():
    config = RVIZ_PATH.read_text(encoding="utf-8")
    launch = SYSTEM_LAUNCH_PATH.read_text(encoding="utf-8")
    for topic in (
        "/map", "/scan", "/plan", "/policy/candidates",
        "/task_camera/image_raw",
    ):
        assert topic in config
    assert '"system_sim.rviz"' in launch


def test_upstream_depth_camera_uses_its_scoped_gazebo_topics():
    source = SIM_LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"intel_realsense_r200_depth"' in source
    assert 'depth_image_topic = f"{camera_sensor_root}/depth_image"' in source
    assert 'remappings=[(depth_image_topic, "/task_camera/image_raw")]' in source
    assert '"/task_camera/camera_info"' not in source
    assert 'arguments=["/camera"]' not in source


def test_frozen_nav2_parameters_stay_inside_upstream_tb3_limits():
    nav2 = yaml.safe_load((
        ROOT / "ros2_ws/src/sstg_nav_bringup/config/nav2_params.yaml"
    ).read_text(encoding="utf-8"))
    shared = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    policy = yaml.safe_load((
        ROOT / "ros2_ws/src/sstg_policy_ros/config/policy.yaml"
    ).read_text(encoding="utf-8"))
    unknown_completion = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/unknown_completion.yaml"
    ).read_text(encoding="utf-8"))
    controller = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    controller_server = nav2["controller_server"]["ros__parameters"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    robot = shared["robot"]

    assert controller["vx_max"] <= robot["max_linear_velocity_mps"]
    assert controller["wz_max"] <= robot["max_angular_velocity_radps"]
    assert controller["ax_max"] <= robot["max_linear_acceleration_mps2"]
    assert controller["az_max"] <= robot["max_angular_acceleration_radps2"]
    assert smoother["max_velocity"] == [0.4, 0.0, 1.1]
    assert smoother["max_accel"] == [0.6, 0.0, 1.8]
    assert smoother["max_decel"] == [-0.6, 0.0, -1.8]
    footprint = robot["collision_footprint"]
    conservative_radius = float(robot["policy_conservative_radius_m"])
    assert conservative_radius == (
        float(footprint["radius_m"]) + float(footprint["padding_m"])
    )
    assert nav2["local_costmap"]["local_costmap"]["ros__parameters"][
        "robot_radius"
    ] == footprint["radius_m"]
    assert controller["CostCritic"]["consider_footprint"] is False
    assert controller_server["progress_checker"][
        "required_movement_radius"
    ] < controller_server["general_goal_checker"]["xy_goal_tolerance"]
    assert controller_server["general_goal_checker"][
        "xy_goal_tolerance"
    ] <= nav2["global_costmap"]["global_costmap"]["ros__parameters"][
        "resolution"
    ]
    assert policy["sstg_policy"]["ros__parameters"]["robot_radius_m"] == (
        unknown_completion["shared_policy"]["robot_radius_m"]
    )


def test_system_sim_budget_matches_policy_and_is_required_by_launch():
    shared = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    policy = yaml.safe_load((
        ROOT / "ros2_ws/src/sstg_policy_ros/config/policy.yaml"
    ).read_text(encoding="utf-8"))["sstg_policy"]["ros__parameters"]
    launch = SYSTEM_LAUNCH_PATH.read_text(encoding="utf-8")

    assert shared["physics"]["seed_valid_range_inclusive"] == [1, 0x7FFFFFFF]

    for field, value in shared["experiment_budget"].items():
        assert policy[field] == value
        assert f'{field} = LaunchConfiguration("{field}")' in launch
        assert f'"{field}": ParameterValue(' in launch
        required_declaration = (
            'DeclareLaunchArgument(\n'
            f'            "{field}",\n'
            '            description="required frozen'
        )
        assert required_declaration in launch

    for seed_name in ("policy_seed", "simulation_seed"):
        required_declaration = (
            'DeclareLaunchArgument(\n'
            f'            "{seed_name}",\n'
            '            description="required frozen'
        )
        assert required_declaration in launch


def test_system_launch_isolates_evaluator_parameters_and_simulation_clock():
    launch = SYSTEM_LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'evaluator_params = LaunchConfiguration("evaluator_params")' in launch
    assert '"evaluator_params_file": evaluator_params' in launch
    assert '"use_sim_time": "true"' in launch
    assert 'DeclareLaunchArgument("evaluator_params",' in launch
    assert '"params_file": evaluator_params' not in launch


def test_system_launch_selects_one_frozen_runtime_adapter():
    launch = SYSTEM_LAUNCH_PATH.read_text(encoding="utf-8")
    package = ElementTree.parse(
        ROOT / "ros2_ws/src/sstg_nav_bringup/package.xml"
    ).getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}

    assert 'runtime_adapter = LaunchConfiguration("runtime_adapter")' in launch
    assert 'EqualsSubstitution(runtime_adapter, "sstg_policy")' in launch
    assert 'runtime_adapter, "frontier_mrtsp_dp_external"' in launch
    assert '"frontier_mrtsp_dp.launch.py"' in launch
    assert "sstg_baseline_adapter" in dependencies


def test_method_configs_distinguish_internal_and_public_runtime_origins():
    method_root = ROOT / "experiments/system_sim/configs/methods"
    internal = {
        name: yaml.safe_load((method_root / f"{name}.yaml").read_text())
        for name in ("sstg", "frontier", "nbv", "rrt_adapted")
    }
    external = yaml.safe_load(
        (method_root / "frontier_mrtsp_dp_external.yaml").read_text()
    )

    assert all(
        config["runtime_adapter"] == "sstg_policy"
        for config in internal.values()
    )
    assert all(
        config["independent_public_baseline"] is False
        for config in internal.values()
    )
    assert external["runtime_adapter"] == "frontier_mrtsp_dp_external"
    assert external["independent_public_baseline"] is True
    assert external["formal_method_eligible"] is False
    assert external["policy_seed_applicable"] is False


def test_core_rosbag_profile_matches_shared_stack_and_is_enabled(monkeypatch):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    shared = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    module = _load_system_launch_module()
    launch = SYSTEM_LAUNCH_PATH.read_text(encoding="utf-8")
    package = ElementTree.parse(
        ROOT / "ros2_ws/src/sstg_nav_bringup/package.xml"
    ).getroot()
    dependencies = {node.text for node in package.findall("exec_depend")}
    recording = shared["recording"]

    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package_name: f"/tmp/{package_name}",
    )

    assert recording["enabled"] is True
    assert recording["storage_id"] == module.CORE_BAG_STORAGE_ID
    assert recording["storage_preset_profile"] == module.CORE_BAG_STORAGE_PRESET
    assert recording["include_hidden_topics"] is True
    assert recording["topics"] == list(module.CORE_BAG_TOPICS)
    assert set(recording["topic_types"]) == set(recording["topics"])
    assert 'DeclareLaunchArgument("record_bag", default_value="true")' in launch
    assert 'name="sstg_core_bag_recorder"' in launch
    assert 'PathJoinSubstitution([output_dir, "bags", "core"])' in launch
    assert {"rosbag2", "rosbag2_storage_mcap"}.issubset(dependencies)

    context = LaunchContext()
    context.launch_configurations["output_dir"] = "/tmp/system_sim_test"
    recorder_commands = [
        [perform_substitutions(context, argument) for argument in entity.cmd]
        for entity in module.generate_launch_description().entities
        if type(entity).__name__ == "ExecuteProcess"
    ]
    assert len(recorder_commands) == 1
    command = recorder_commands[0]
    topics_index = command.index("--topics")
    assert command[topics_index + 1:] == recording["topics"]

    hidden_action_topics = [
        topic for topic in recording["topics"] if "/_action/" in topic
    ]
    assert hidden_action_topics == [
        "/navigate_to_pose/_action/feedback",
        "/navigate_to_pose/_action/status",
        "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/feedback",
        "/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/status",
    ]
    assert recording["required_nonempty_topics_by_runtime_adapter"] == {
        "sstg_policy": hidden_action_topics[:2],
        "frontier_mrtsp_dp_external": hidden_action_topics,
    }
    assert command.count("--include-hidden-topics") == int(
        recording["include_hidden_topics"]
    )
    assert command.index("--include-hidden-topics") < topics_index


def test_development_registry_has_four_distinct_generated_scene_families():
    registry = yaml.safe_load((
        ROOT / "experiments/system_sim/registries/worlds.yaml"
    ).read_text(encoding="utf-8"))
    worlds = registry["worlds"]
    assert len(worlds) == 4
    assert len({entry["world_id"] for entry in worlds}) == 4
    assert {entry["site_family"] for entry in worlds} == {
        "multi_room_office", "dense_laboratory", "warehouse_aisles",
        "corridor_alcoves",
    }
    for entry in worlds:
        bundle = ROOT / entry["bundle"]
        metadata = yaml.safe_load((bundle / "metadata.yaml").read_text())
        starts = yaml.safe_load((bundle / "starts.yaml").read_text())
        targets = yaml.safe_load((bundle / "targets.yaml").read_text())
        sdf_world = ElementTree.parse(bundle / "world.sdf").getroot().find("world")
        manifest = yaml.safe_load((
            bundle / "evaluation/truth_map_manifest.yaml"
        ).read_text())
        assert metadata["world_id"] == entry["world_id"]
        assert metadata["site_family"] == entry["site_family"]
        assert metadata["formal_result_eligible"] is False
        assert sdf_world.get("name") == entry["world_id"]
        assert len(starts["starts"]) >= 2
        assert len(targets["targets"]) >= 4
        assert manifest["world_id"] == entry["world_id"]
        for relative, expected in manifest["sha256"].items():
            path = (
                bundle / "evaluation" / relative
                if relative.startswith("truth_map") else bundle / relative
            )
            assert _sha256(path) == expected
        if (bundle / "scene.yaml").exists():
            assert generate_scene_world(bundle, check=True) == []


def test_development_scenes_are_connected_for_frozen_robot_and_targets():
    registry = yaml.safe_load((
        ROOT / "experiments/system_sim/registries/worlds.yaml"
    ).read_text(encoding="utf-8"))
    shared = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    clearance_m = float(shared["robot"]["policy_conservative_radius_m"])
    for entry in registry["worlds"]:
        bundle = ROOT / entry["bundle"]
        metadata = yaml.safe_load((bundle / "metadata.yaml").read_text())
        starts = yaml.safe_load((bundle / "starts.yaml").read_text())["starts"]
        targets = yaml.safe_load((bundle / "targets.yaml").read_text())["targets"]
        occupied, resolution, _ = rasterize(
            metadata, collision_boxes(bundle / "world.sdf", slice_z=0.15)
        )
        traversable = ~_inflate_occupied(occupied, resolution, clearance_m)
        first_start = _world_cell(metadata, starts[0]["x_m"], starts[0]["y_m"])
        assert traversable[first_start], entry["world_id"]
        reachable = _reachable_cells(traversable, first_start)
        assert reachable.sum() / traversable.sum() > 0.999, entry["world_id"]
        for start in starts:
            cell = _world_cell(metadata, start["x_m"], start["y_m"])
            assert reachable[cell], (entry["world_id"], start["start_id"])
        for target in targets:
            normal = math.radians(float(target["surface_normal_yaw_deg"]))
            visible_reachable_pose = False
            for distance_m in np.arange(0.6, 4.01, 0.1):
                x_m = float(target["x_m"]) + math.cos(normal) * distance_m
                y_m = float(target["y_m"]) + math.sin(normal) * distance_m
                row, column = _world_cell(metadata, x_m, y_m)
                if not (
                    0 <= row < reachable.shape[0]
                    and 0 <= column < reachable.shape[1]
                    and reachable[row, column]
                ):
                    continue
                ray_clear = True
                for ray_distance in np.arange(0.05, distance_m, resolution / 2.0):
                    ray_x = float(target["x_m"]) + math.cos(normal) * ray_distance
                    ray_y = float(target["y_m"]) + math.sin(normal) * ray_distance
                    if occupied[_world_cell(metadata, ray_x, ray_y)]:
                        ray_clear = False
                        break
                if ray_clear:
                    visible_reachable_pose = True
                    break
            assert visible_reachable_pose, (
                entry["world_id"], target["target_id"]
            )
