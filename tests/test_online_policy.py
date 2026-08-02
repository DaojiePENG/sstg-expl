"""Tests for the hidden-truth-free embodied policy boundary."""
import numpy as np
import pytest

from sstg_explorer import (
    OccupancyGrid,
    OnlineExplorerSession,
    SensorConfig,
    UnknownExplorerConfig,
)


def _known_room(size=100, resolution=0.1):
    data = np.zeros((size, size), dtype=np.int8)
    data[:3, :] = 100
    data[-3:, :] = 100
    data[:, :3] = 100
    data[:, -3:] = 100
    return OccupancyGrid(data, resolution, (0.0, 0.0))


def _session(start=(5.0, 5.0, 0.0), **config_overrides):
    return OnlineExplorerSession(
        UnknownExplorerConfig(
            strategy="sstg",
            sensor=SensorConfig(360.0, 8.0, 1.0),
            coverage_objective="joint",
            termination_mode="candidate_exhaustion",
            topological_radius=2.0,
            max_decisions=20,
            seed=7,
            **config_overrides,
        ),
        start,
    )


def test_online_session_selects_goal_without_ground_truth_argument():
    session = _session()
    decision = session.propose(_known_room(), map_revision=3)

    assert decision.status == "navigate"
    assert decision.map_revision == 3
    assert decision.target_pose is not None
    assert decision.known_free_cells > 0
    assert 0.0 < decision.known_topological_coverage < 1.0
    assert decision.selected_candidate["kind"] == "coverage_gap"
    assert decision.generated_candidates


def test_goal_clearance_is_endpoint_only_and_filters_all_candidates():
    session = OnlineExplorerSession(
        UnknownExplorerConfig(
            strategy="sstg",
            sensor=SensorConfig(360.0, 8.0, 1.0),
            coverage_objective="joint",
            termination_mode="candidate_exhaustion",
            robot_radius=0.24,
            minimum_goal_clearance=1.0,
            seed=7,
        ),
        (5.0, 5.0, 0.0),
    )

    decision = session.propose(_known_room(), map_revision=1)

    assert decision.status == "navigate"
    assert decision.generated_candidates
    assert all(
        candidate["clearance"] + 1e-9 >= 1.0
        for candidate in decision.generated_candidates
    )


def test_online_session_requires_execution_result_before_next_goal():
    session = _session()
    belief = _known_room()
    decision = session.propose(belief)

    with pytest.raises(RuntimeError):
        session.propose(belief)

    record = session.record_execution(
        decision.decision_id,
        succeeded=True,
        reached_pose=decision.target_pose,
    )
    assert record.succeeded
    assert record.topological_node_created
    assert len(session.nodes) == 2
    assert session.pending_decision is None
    assert session.summary()["successful_executions"] == 1


def test_failed_goal_is_audited_and_not_added_as_observation_node():
    session = _session()
    belief = _known_room()
    decision = session.propose(belief)
    record = session.record_execution(
        decision.decision_id,
        succeeded=False,
        reached_pose=decision.current_pose,
        executed_path=[decision.current_pose[:2]],
        reason="controller_failed",
    )

    assert not record.succeeded
    assert not record.topological_node_created
    assert len(session.nodes) == 1
    assert session.execution_records[0].reason == "controller_failed"
    assert session.summary()["total_distance_m"] == pytest.approx(0.0)


def test_failed_goal_still_counts_actual_execution_cost():
    session = _session()
    decision = session.propose(_known_room())
    session.record_execution(
        decision.decision_id,
        succeeded=False,
        reached_pose=(5.5, 5.0, 30.0),
        executed_path=[(5.0, 5.0), (5.25, 5.0), (5.5, 5.0)],
        reason="controller_aborted_after_motion",
    )

    summary = session.summary()
    assert summary["total_distance_m"] == pytest.approx(0.5)
    assert summary["total_rotation_deg"] == pytest.approx(30.0)
    assert len(summary["nodes"]) == 1


def test_failed_goal_suppresses_neighboring_embodied_targets():
    session = _session(failed_goal_suppression_radius=20.0)
    belief = _known_room()
    failed = session.propose(belief, map_revision=1)

    record = session.record_execution(
        failed.decision_id,
        succeeded=False,
        reached_pose=failed.current_pose,
        reason="controller_aborted",
    )
    next_decision = session.propose(belief, map_revision=2)

    assert record.failure_neighborhood_recorded
    assert session.summary()["failed_goal_neighborhoods"] == [
        list(failed.target_pose[:2])
    ]
    assert any(
        candidate["status"] == "pruned_navigation_failure_neighborhood"
        for candidate in next_decision.generated_candidates
    )
    assert not next_decision.active_candidates


def test_adapter_cancel_does_not_poison_failed_goal_neighborhood():
    session = _session(failed_goal_suppression_radius=0.8)
    decision = session.propose(_known_room())

    record = session.record_execution(
        decision.decision_id,
        succeeded=False,
        reached_pose=decision.current_pose,
        reason="nav2_status_5:distance_budget",
        suppress_failed_target=False,
    )

    assert not record.failure_neighborhood_recorded
    assert session.summary()["failed_goal_neighborhoods"] == []


def test_execution_path_in_odom_is_not_mixed_with_map_endpoints():
    session = _session(start=(5.0, 5.0, 0.0))
    decision = session.propose(_known_room())
    record = session.record_execution(
        decision.decision_id,
        succeeded=True,
        reached_pose=decision.target_pose,
        executed_path=[(0.0, 0.0), (0.3, 0.4)],
        executed_path_frame="odom",
    )

    assert record.path_frame == "odom"
    assert record.path == [(0.0, 0.0), (0.3, 0.4)]
    assert record.translation_m == pytest.approx(0.5)
    assert session.summary()["total_distance_m"] == pytest.approx(0.5)


def test_online_completion_does_not_claim_truth_coverage():
    belief = OccupancyGrid(
        np.array([[100, 100, 100], [100, 0, 100], [100, 100, 100]], dtype=np.int8),
        1.0,
        (0.0, 0.0),
    )
    session = _session(start=(1.5, 1.5, 0.0))
    first = session.propose(belief, map_revision=1)
    repeated = session.propose(belief, map_revision=1)
    second = session.propose(belief, map_revision=2)
    decision = session.propose(belief, map_revision=3)

    assert first.status == "confirming"
    assert repeated.exhaustion_confirmation == 1
    assert second.exhaustion_confirmation == 2
    assert decision.status == "complete"
    assert decision.reason == "candidate_exhaustion"
    assert decision.exhaustion_confirmation == 3
    assert "truth" not in decision.to_dict()
    assert session.summary()["termination_reason"] == "candidate_exhaustion"


def test_sstg_native_completion_skips_topology_only_tail():
    session = _session(target_topological_coverage=0.01)
    belief = _known_room()

    first = session.propose(belief, map_revision=1)
    second = session.propose(belief, map_revision=2)
    complete = session.propose(belief, map_revision=3)

    assert first.status == "confirming"
    assert first.reason == "sstg_frontier_topology_convergence_pending"
    assert first.native_completion_trigger == (
        "sstg_frontier_topology_convergence"
    )
    assert first.active_candidates
    assert not any(
        candidate["kind"] == "frontier"
        and candidate["predicted_gain"] >= session.config.min_gain_cells
        for candidate in first.active_candidates
    )
    assert second.exhaustion_confirmation == 2
    assert complete.status == "complete"
    assert complete.reason == "candidate_exhaustion"
    assert session.summary()["native_completion_trigger"] == (
        "sstg_frontier_topology_convergence"
    )


def test_sstg_native_completion_never_ignores_informative_frontier():
    belief = _known_room()
    belief.data[30:70, 70:97] = -1
    session = _session(target_topological_coverage=0.01)

    decision = session.propose(belief, map_revision=1)

    assert decision.status == "navigate"
    assert decision.native_completion_trigger is None
    assert any(
        candidate["kind"] == "frontier"
        and candidate["predicted_gain"] >= session.config.min_gain_cells
        for candidate in decision.active_candidates
    )


def test_goal_tolerance_pose_is_bridged_back_to_conservative_safe_space():
    belief = _known_room()
    # This measured pose is known free but lies inside the 0.3 m erosion band,
    # as can happen when Nav2 accepts a nearby frontier goal within tolerance.
    session = _session(start=(0.35, 5.0, 0.0))

    decision = session.propose(belief)

    assert decision.status == "navigate"
    assert decision.reason == "goal_selected"
    assert len(decision.active_candidates) > 1
    assert decision.planned_path[0] == pytest.approx((0.35, 5.0))


def test_new_native_candidate_resets_completion_confirmation():
    enclosed_data = np.full((100, 100), 100, dtype=np.int8)
    enclosed_data[15, 15] = 0
    enclosed = OccupancyGrid(
        enclosed_data,
        0.1,
        (0.0, 0.0),
    )
    session = _session(start=(1.55, 1.55, 0.0))
    assert session.propose(enclosed, map_revision=1).status == "confirming"
    assert session.propose(enclosed, map_revision=2).exhaustion_confirmation == 2

    decision = session.propose(_known_room(), map_revision=3)

    assert decision.status == "navigate"
    assert decision.exhaustion_confirmation == 0


def test_online_session_rejects_truth_target_termination():
    config = UnknownExplorerConfig(termination_mode="coverage_target")
    with pytest.raises(ValueError, match="has no truth"):
        OnlineExplorerSession(config, (1.0, 1.0, 0.0))


def test_map_transform_uses_floor_below_negative_boundary():
    belief = _known_room()
    assert belief.world_to_grid(-0.01, 1.0)[1] == -1
    assert not belief.is_valid_world(-0.01, 1.0)


def test_session_rejects_resolution_change_but_allows_origin_growth():
    session = _session()
    first = session.propose(_known_room(), map_revision=1)
    session.record_execution(
        first.decision_id, True, first.target_pose, first.planned_path
    )
    grown = OccupancyGrid(
        np.zeros((140, 140), dtype=np.int8), 0.1, (-2.0, -2.0)
    )
    second = session.propose(grown, map_revision=2)
    assert second.map_revision == 2

    session.record_execution(
        second.decision_id, False, second.current_pose, reason="test"
    )
    changed = OccupancyGrid(grown.data, 0.2, grown.origin)
    with pytest.raises(ValueError, match="resolution changed"):
        session.propose(changed, map_revision=3)


def test_ans_temporal_memory_is_reprojected_when_map_extent_grows():
    policy = _session().policy
    previous = OccupancyGrid(
        np.zeros((4, 5), dtype=np.int8), 0.1, (0.0, 0.0)
    )
    policy._previous_known_mask = np.zeros(previous.shape, dtype=bool)
    policy._previous_known_mask[1, 2] = True
    policy._previous_known_origin = previous.origin
    policy._previous_known_resolution = previous.resolution
    grown = OccupancyGrid(
        np.zeros((8, 10), dtype=np.int8), 0.1, (-0.3, -0.2)
    )

    reprojected = policy._previous_known_in_current_grid(grown)

    assert reprojected.shape == grown.shape
    assert np.count_nonzero(reprojected) == 1
    assert reprojected[3, 5]
