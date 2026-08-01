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


def _session(start=(5.0, 5.0, 0.0)):
    return OnlineExplorerSession(
        UnknownExplorerConfig(
            strategy="sstg",
            sensor=SensorConfig(360.0, 8.0, 1.0),
            coverage_objective="joint",
            topological_radius=2.0,
            max_decisions=20,
            seed=7,
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
    decision = session.propose(belief)

    assert decision.status == "complete"
    assert decision.reason == "candidate_exhaustion"
    assert "truth" not in decision.to_dict()


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
