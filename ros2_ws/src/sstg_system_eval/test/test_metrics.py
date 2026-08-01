import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from sstg_system_eval.metrics import (
    ActionTraceAccumulator,
    BeliefGrid,
    CameraGeometry,
    CollisionAccumulator,
    GroundTruthMotionAccumulator,
    TargetRecallAccumulator,
    TargetSpec,
    TruthClearanceAccumulator,
    TopologicalNodeAccumulator,
    TrajectoryAccumulator,
    TruthGrid,
    WorldStatisticsAccumulator,
    compute_geometric_metrics,
    compute_topological_metrics,
    evaluate_target_visibility,
    load_target_registry,
    load_truth_map,
    transform_planar_point,
    transform_truth_grid,
)


def _truth(free, occupied=None, resolution=1.0, origin=(0.0, 0.0)):
    free = np.asarray(free, dtype=bool)
    if occupied is None:
        occupied = ~free
    return TruthGrid(
        free=free,
        occupied=np.asarray(occupied, dtype=bool),
        resolution=resolution,
        origin=origin,
    )


def _record(event, payload, ros_time_ns=1):
    return json.dumps({
        "event": event,
        "ros_time_ns": ros_time_ns,
        "map_revision": 1,
        "payload": payload,
    }, sort_keys=True)


def test_load_generated_truth_map_preserves_ros_row_order():
    repository = Path(__file__).resolve().parents[4]
    truth_yaml = repository / (
        "ros2_ws/src/sstg_gazebo/worlds/development/multi_room_office/"
        "dev_office_01/evaluation/truth_map.yaml"
    )
    truth = load_truth_map(truth_yaml)

    assert truth.shape == (240, 320)
    assert truth.resolution == pytest.approx(0.05)
    assert truth.origin == (-8.0, -6.0)
    manifest = yaml.safe_load(
        (truth_yaml.parent / "truth_map_manifest.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert int(np.count_nonzero(truth.free)) == manifest["free_cells"]
    assert int(np.count_nonzero(truth.occupied)) == manifest["occupied_cells"]
    assert truth.source_sha256


def test_geometric_metrics_score_truth_free_known_free_only():
    truth = _truth(
        [[True, True], [False, True]],
        [[False, False], [True, False]],
    )
    belief = BeliefGrid(
        data=np.asarray([[0, -1], [0, 100]], dtype=np.int16),
        resolution=1.0,
        origin=(0.0, 0.0),
    )

    result = compute_geometric_metrics(truth, belief)

    assert result["truth_free_total_cells"] == 3
    assert result["truth_free_known_cells"] == 2
    assert result["truth_free_known_free_cells"] == 1
    assert result["truth_free_known_occupied_cells"] == 1
    assert result["truth_occupied_known_free_cells"] == 1
    assert result["geometric_coverage"] == pytest.approx(1.0 / 3.0)
    assert result["truth_free_observed_fraction"] == pytest.approx(2.0 / 3.0)
    assert result["known_free_precision_on_truth"] == pytest.approx(0.5)


def test_geometric_projection_supports_changed_origin_and_resolution():
    truth = _truth([[True, True, True, True]], occupied=[[False] * 4])
    belief = BeliefGrid(
        data=np.asarray([[0]], dtype=np.int16),
        resolution=2.0,
        origin=(1.0, 0.0),
    )

    result = compute_geometric_metrics(truth, belief)

    assert result["truth_free_in_belief_extent_cells"] == 2
    assert result["truth_free_known_free_cells"] == 2
    assert result["geometric_coverage"] == pytest.approx(0.5)
    assert result["truth_free_extent_fraction"] == pytest.approx(0.5)


def test_truth_registration_transforms_origin_and_orientation():
    truth = _truth([[True]], origin=(1.0, 0.0))

    registered = transform_truth_grid(
        truth, translation=(2.0, 3.0), yaw=np.pi / 2.0
    )

    assert registered.origin == pytest.approx((2.0, 4.0))
    assert registered.origin_yaw == pytest.approx(np.pi / 2.0)
    belief = BeliefGrid(
        data=np.asarray([[0]], dtype=np.int16),
        resolution=1.0,
        origin=(1.0, 4.0),
    )
    assert compute_geometric_metrics(registered, belief)[
        "geometric_coverage"
    ] == 1.0


def test_planar_transform_and_ground_truth_path_ate_are_auditable():
    assert transform_planar_point(
        (1.0, 0.0), (2.0, 3.0), np.pi / 2.0
    ) == pytest.approx((2.0, 4.0))
    motion = GroundTruthMotionAccumulator(minimum_step_m=0.05)

    assert not motion.add_ground_truth(10, 0.0, 0.0)
    assert motion.add_ground_truth(20, 3.0, 4.0)
    assert motion.add_ate_pair(10, (0.0, 0.0), (1.0, 0.0))
    assert motion.add_ate_pair(20, (2.0, 0.0), (4.0, 0.0))
    assert not motion.add_ate_pair(20, (2.0, 0.0), (99.0, 0.0))

    snapshot = motion.snapshot()
    assert snapshot["status"] == "available"
    assert snapshot["ground_truth_path_length_m"] == pytest.approx(5.0)
    assert snapshot["ate_sample_count"] == 2
    assert snapshot["ate_mean_m"] == pytest.approx(1.5)
    assert snapshot["ate_rmse_m"] == pytest.approx(np.sqrt(2.5))
    assert snapshot["ate_max_m"] == pytest.approx(2.0)


def test_truth_clearance_reports_raw_and_radius_reduced_statistics():
    free = np.ones((9, 10), dtype=bool)
    free[4, 6] = False
    truth = _truth(free, occupied=~free)
    clearance = TruthClearanceAccumulator(truth, robot_radius_m=0.5)

    assert clearance.add(4.5, 4.5) == pytest.approx(1.0)
    assert clearance.add(5.5, 4.5) == pytest.approx(0.0)
    assert clearance.add(-0.1, 4.5) == pytest.approx(0.0)

    snapshot = clearance.snapshot()
    assert snapshot["clearance_sample_count"] == 3
    assert snapshot["outside_truth_extent_sample_count"] == 1
    assert snapshot["outside_truth_extent_fraction"] == pytest.approx(1 / 3)
    assert snapshot["raw_static_obstacle_distance_min_m"] == 0.0
    assert snapshot["raw_static_obstacle_distance_p05_m"] == pytest.approx(0.05)
    assert snapshot["raw_static_obstacle_distance_mean_m"] == pytest.approx(
        2.0 / 3.0
    )
    assert snapshot["footprint_clearance_min_m"] == 0.0
    assert snapshot["footprint_clearance_p05_m"] == 0.0
    assert snapshot["footprint_clearance_mean_m"] == pytest.approx(1.0 / 3.0)
    assert snapshot["outside_samples_are_zero_clearance"] is True


def test_truth_clearance_respects_rotated_truth_origin():
    free = np.ones((9, 10), dtype=bool)
    free[4, 6] = False
    truth = TruthGrid(
        free=free,
        occupied=~free,
        resolution=1.0,
        origin=(10.0, 20.0),
        origin_yaw=np.pi / 2.0,
    )
    clearance = TruthClearanceAccumulator(truth, robot_radius_m=0.5)

    assert clearance.add(5.5, 24.5) == pytest.approx(1.0)


def test_truth_clearance_treats_unknown_as_obstacle():
    free = np.asarray([[True, True, False]], dtype=bool)
    occupied = np.asarray([[False, False, False]], dtype=bool)
    truth = _truth(free, occupied=occupied)
    clearance = TruthClearanceAccumulator(truth, robot_radius_m=0.25)

    assert clearance.add(1.5, 0.5) == pytest.approx(0.25)
    snapshot = clearance.snapshot()
    assert snapshot["raw_static_obstacle_distance_min_m"] == 0.5
    assert snapshot["unknown_truth_cells_are_obstacles"] is True


def test_world_statistics_accumulates_clock_rtf_pause_and_iterations():
    statistics = WorldStatisticsAccumulator()
    statistics.ingest(
        sim_time_ns=0,
        pause_time_ns=0,
        real_time_ns=0,
        paused=False,
        iterations=0,
        model_count=18,
        real_time_factor=1.0,
        step_size_ns=4_000_000,
        stepping=False,
    )
    statistics.ingest(
        sim_time_ns=1_000_000_000,
        pause_time_ns=0,
        real_time_ns=2_000_000_000,
        paused=False,
        iterations=250,
        model_count=18,
        real_time_factor=0.5,
        step_size_ns=4_000_000,
        stepping=False,
    )
    statistics.ingest(
        sim_time_ns=1_000_000_000,
        pause_time_ns=1_000_000_000,
        real_time_ns=3_000_000_000,
        paused=True,
        iterations=250,
        model_count=18,
        real_time_factor=0.0,
        step_size_ns=0,
        stepping=False,
    )

    snapshot = statistics.snapshot()
    assert snapshot["status"] == "available"
    assert snapshot["world_stats_sample_count"] == 3
    assert snapshot["sim_time_elapsed_ns"] == 1_000_000_000
    assert snapshot["real_time_elapsed_ns"] == 3_000_000_000
    assert snapshot["paused_latest"] is True
    assert snapshot["paused_sample_fraction"] == pytest.approx(1.0 / 3.0)
    assert snapshot["iterations_latest"] == 250
    assert snapshot["iteration_delta"] == 250
    assert snapshot["reported_real_time_factor_mean"] == pytest.approx(0.5)
    assert snapshot["observed_delta_real_time_factor_mean"] == pytest.approx(
        0.25
    )


def test_world_statistics_flags_nonmonotonic_clock_and_iteration():
    statistics = WorldStatisticsAccumulator()
    common = {
        "pause_time_ns": 0,
        "paused": False,
        "model_count": 1,
        "real_time_factor": 1.0,
        "step_size_ns": 1,
        "stepping": False,
    }
    statistics.ingest(
        sim_time_ns=10, real_time_ns=10, iterations=10, **common
    )
    statistics.ingest(
        sim_time_ns=9, real_time_ns=9, iterations=9, **common
    )
    statistics.ingest(
        sim_time_ns=9, real_time_ns=9, iterations=9, **common
    )

    snapshot = statistics.snapshot()
    assert snapshot["status"] == "degraded_nonmonotonic_clock"
    assert snapshot["nonmonotonic_sim_time_count"] == 1
    assert snapshot["nonmonotonic_real_time_count"] == 1
    assert snapshot["nonmonotonic_iteration_count"] == 1
    assert snapshot["sim_time_elapsed_ns"] == 0
    assert snapshot["real_time_elapsed_ns"] == 0
    assert snapshot["iteration_delta"] == 0


def test_inverse_spawn_registration_handles_nonzero_spawn_yaw():
    spawn_translation = np.asarray([2.0, 3.0])
    spawn_yaw = np.pi / 2.0
    inverse_yaw = -spawn_yaw
    rotation = np.asarray([
        [np.cos(inverse_yaw), -np.sin(inverse_yaw)],
        [np.sin(inverse_yaw), np.cos(inverse_yaw)],
    ])
    inverse_translation = -(rotation @ spawn_translation)

    assert transform_planar_point(
        spawn_translation,
        inverse_translation,
        inverse_yaw,
    ) == pytest.approx((0.0, 0.0))


def test_collision_accumulator_filters_support_contacts_and_debounces():
    collisions = CollisionAccumulator(
        robot_name_tokens=["sstg_diffbot", "base_collision", "left_wheel"],
        ground_name_tokens=["floor", "ground_plane"],
        event_separation_s=1.0,
    )

    assert collisions.ingest(0, []) == 0
    assert collisions.ingest(10, [
        (
            "sstg_diffbot::left_wheel::collision",
            "floor::floor_link::collision",
            0.002,
        ),
    ]) == 0
    wall = (
        "sstg_diffbot::base_link::base_collision",
        "wall_west::wall::c",
        0.003,
    )
    assert collisions.ingest(20, [wall, wall]) == 1
    assert collisions.ingest(30, [wall]) == 0
    assert collisions.ingest(40, []) == 0
    assert collisions.ingest(50, [wall]) == 0
    assert collisions.ingest(1_000_000_050, []) == 0
    assert collisions.ingest(1_000_000_060, [wall]) == 1

    snapshot = collisions.snapshot()
    assert snapshot["status"] == "available"
    assert snapshot["collision_count"] == 2
    assert snapshot["collision_free"] is False
    assert snapshot["ignored_ground_contact_count"] == 1
    assert snapshot["raw_contact_count"] == 6
    assert snapshot["maximum_reported_penetration_depth_m"] == 0.003


def test_collision_accumulator_is_conservative_for_unattributed_contacts():
    collisions = CollisionAccumulator(["robot"], ["floor"])

    assert collisions.ingest(1, [
        ("unknown_a", "wall", 0.1),
        ("robot::wheel", "robot::base", 0.1),
        ("", "wall", 0.1),
    ]) == 0
    snapshot = collisions.snapshot()
    assert snapshot["collision_free"] is None
    assert snapshot["status"] == "degraded_unverified_contact_stream"
    assert snapshot["contact_attribution_complete"] is False
    assert snapshot["ignored_unattributed_contact_count"] == 1
    assert snapshot["ignored_self_contact_count"] == 1
    assert snapshot["malformed_contact_count"] == 1


def test_collision_free_is_unverified_after_nonmonotonic_contact_stamp():
    collisions = CollisionAccumulator(["robot"], ["floor"])

    assert collisions.ingest(100, []) == 0
    assert collisions.ingest(90, []) == 0

    snapshot = collisions.snapshot()
    assert snapshot["collision_free"] is None
    assert snapshot["contact_temporal_order_complete"] is False
    assert snapshot["contact_nonmonotonic_stamp_count"] == 1


def test_target_registry_and_visibility_proxy_use_fov_facing_and_los():
    repository = Path(__file__).resolve().parents[4]
    registry = repository / (
        "ros2_ws/src/sstg_gazebo/worlds/development/multi_room_office/"
        "dev_office_01/targets.yaml"
    )
    world_id, targets, digest = load_target_registry(registry)
    assert world_id == "dev_office_01"
    assert len(targets) == 4
    assert digest

    free = np.ones((2, 6), dtype=bool)
    truth = _truth(free, occupied=np.zeros_like(free))
    target = TargetSpec(
        target_id="panel",
        class_name="inspection_panel",
        x_m=4.5,
        y_m=0.5,
        z_m=0.5,
        surface_normal_yaw_rad=np.pi,
    )
    camera = CameraGeometry(
        x_offset_m=0.0,
        height_m=0.5,
        horizontal_fov_rad=np.pi / 2.0,
        vertical_fov_rad=np.pi / 2.0,
        maximum_range_m=10.0,
        los_endpoint_clearance_m=0.1,
    )

    visible = evaluate_target_visibility(
        truth, target, (0.5, 0.5, 0.0), camera
    )
    assert visible["visible"] is True
    assert visible["reason"] == "visible"
    assert evaluate_target_visibility(
        truth, target, (0.5, 0.5, np.pi), camera
    )["reason"] == "horizontal_fov"

    blocked_free = free.copy()
    blocked_free[0, 2] = False
    blocked_truth = _truth(
        blocked_free,
        occupied=~blocked_free,
    )
    assert evaluate_target_visibility(
        blocked_truth, target, (0.5, 0.5, 0.0), camera
    )["reason"] == "occluded_2d_truth"


def test_target_recall_records_first_seen_time_once():
    free = np.ones((2, 6), dtype=bool)
    truth = _truth(free, occupied=np.zeros_like(free))
    target = TargetSpec(
        "panel", "inspection_panel", 4.5, 0.5, 0.5, np.pi
    )
    camera = CameraGeometry(
        x_offset_m=0.0,
        height_m=0.5,
        horizontal_fov_rad=np.pi / 2.0,
        vertical_fov_rad=np.pi / 2.0,
        maximum_range_m=10.0,
    )
    recall = TargetRecallAccumulator(truth, [target], camera)
    recall.begin_session(500_000_000)

    assert recall.ingest(400_000_000, (0.5, 0.5, 0.0)) == ()
    assert recall.ingest(1_000_000_000, (0.5, 0.5, np.pi)) == ()
    assert recall.ingest(2_500_000_000, (0.5, 0.5, 0.0)) == ("panel",)
    assert recall.ingest(3_000_000_000, (0.5, 0.5, 0.0)) == ()

    snapshot = recall.snapshot()
    assert snapshot["target_recall"] == 1.0
    assert snapshot["detected_target_ids"] == ["panel"]
    assert snapshot["first_detections"]["panel"][
        "first_seen_elapsed_s"
    ] == pytest.approx(2.0)
    assert snapshot["time_origin_ros_time_ns"] == 500_000_000
    assert snapshot["pre_origin_pose_count"] == 1
    assert snapshot["detection_model_is_image_detector"] is False


def test_topological_coverage_and_dual_threshold_success():
    truth = _truth([[True, True, True, True, True]], occupied=[[False] * 5])

    result = compute_topological_metrics(
        truth,
        node_positions=[(2.5, 0.5)],
        radius_m=1.0,
        information_coverage=0.8,
        information_target=0.75,
        topological_target=0.6,
    )

    assert result["truth_free_topologically_covered_cells"] == 3
    assert result["topological_coverage"] == pytest.approx(0.6)
    assert result["joint_coverage"] == pytest.approx(0.6)
    assert result["information_target_met"] is True
    assert result["topological_target_met"] is True
    assert result["dual_threshold_success"] is True


def test_topological_coverage_waits_for_information_snapshot():
    truth = _truth([[True]])

    result = compute_topological_metrics(
        truth, [(0.5, 0.5)], radius_m=0.1, information_coverage=None
    )

    assert result["topological_coverage"] == 1.0
    assert result["joint_coverage"] is None
    assert result["dual_threshold_success"] is None


def test_topological_nodes_use_initial_and_successfully_created_reached_poses():
    nodes = TopologicalNodeAccumulator(deduplication_tolerance_m=0.05)
    initial = {
        "event": "session_started",
        "payload": {"nodes": [{"id": 0, "position": [1.0, 2.0]}]},
    }
    created = {
        "event": "execution",
        "payload": {
            "decision_id": 1,
            "succeeded": True,
            "topological_node_created": True,
            "reached_pose": [3.0, 4.0, 90.0],
        },
    }
    duplicate = {
        "event": "execution",
        "payload": {
            "decision_id": 2,
            "succeeded": True,
            "topological_node_created": True,
            "reached_pose": [3.03, 4.0, 90.0],
        },
    }
    failed = {
        "event": "execution",
        "payload": {
            "decision_id": 3,
            "succeeded": False,
            "topological_node_created": True,
            "reached_pose": [8.0, 8.0, 0.0],
        },
    }
    merged = {
        "event": "execution",
        "payload": {
            "decision_id": 4,
            "succeeded": True,
            "topological_node_created": False,
            "reached_pose": [9.0, 9.0, 0.0],
        },
    }

    assert nodes.ingest_record(initial) == 1
    assert nodes.ingest_record(created) == 1
    assert nodes.ingest_record(duplicate) == 0
    assert nodes.ingest_record(failed) == 0
    assert nodes.ingest_record(merged) == 0
    snapshot = nodes.snapshot()
    assert snapshot["raw_node_observation_count"] == 3
    assert snapshot["initial_node_observation_count"] == 1
    assert snapshot["execution_node_observation_count"] == 2
    assert snapshot["duplicate_node_observation_count"] == 1
    assert snapshot["unique_node_count"] == 2
    assert nodes.positions == [(1.0, 2.0), (3.0, 4.0)]


def test_load_truth_map_rejects_invalid_thresholds(tmp_path):
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\xfe")
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(yaml.safe_dump({
        "image": "map.pgm",
        "resolution": 1.0,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.1,
        "free_thresh": 0.2,
    }), encoding="utf-8")

    with pytest.raises(ValueError, match="thresholds"):
        load_truth_map(map_yaml)


def test_trajectory_accumulator_filters_jitter_and_handles_time_reset():
    trajectory = TrajectoryAccumulator(minimum_step_m=0.05)

    assert not trajectory.add(10, 0.0, 0.0)
    assert not trajectory.add(20, 0.01, 0.0)
    assert trajectory.add(30, 0.11, 0.0)
    assert not trajectory.add(5, 100.0, 100.0)
    assert trajectory.add(6, 100.0, 100.2)

    snapshot = trajectory.snapshot()
    assert snapshot["tf_sample_count"] == 5
    assert snapshot["tf_moving_segment_count"] == 2
    assert snapshot["tf_time_reset_count"] == 1
    assert snapshot["tf_path_length_m"] == pytest.approx(0.31)


def test_action_trace_accumulator_counts_and_recomputes_path_length():
    actions = ActionTraceAccumulator()
    decision = _record("decision", {
        "decision_id": 1,
        "status": "navigate",
        "decision_time_ms": 12.5,
    })
    execution = _record("execution", {
        "decision_id": 1,
        "succeeded": True,
        "reason": "nav2_status_4",
        "translation_m": 6.0,
        "rotation_deg": 90.0,
        "path": [[0.0, 0.0], [3.0, 4.0]],
    }, ros_time_ns=2)

    assert actions.ingest(decision)
    assert not actions.ingest(decision)
    assert actions.ingest(execution)
    snapshot = actions.snapshot()

    assert snapshot["accepted_trace_events"] == 2
    assert snapshot["decision_count"] == 1
    assert snapshot["navigation_goal_count"] == 1
    assert snapshot["execution_count"] == 1
    assert snapshot["navigation_success_rate"] == 1.0
    assert snapshot["decision_time_ms_mean"] == 12.5
    assert snapshot["trace_reported_translation_m"] == 6.0
    assert snapshot["trace_recomputed_path_length_m"] == 5.0
    assert snapshot["trace_translation_disagreement_m"] == 1.0
    assert snapshot["execution_reasons"] == {"nav2_status_4": 1}


def test_action_trace_rejects_malformed_motion_payload():
    actions = ActionTraceAccumulator()
    with pytest.raises(ValueError, match="non-negative"):
        actions.ingest(_record("execution", {
            "decision_id": 1,
            "succeeded": False,
            "reason": "failed",
            "translation_m": -1.0,
            "rotation_deg": 0.0,
            "path": [],
        }))


def test_action_trace_rejects_nonstandard_json_nan():
    actions = ActionTraceAccumulator()
    with pytest.raises(ValueError, match="non-standard JSON"):
        actions.ingest(
            '{"event":"unknown","payload":{"value":NaN}}'
        )
