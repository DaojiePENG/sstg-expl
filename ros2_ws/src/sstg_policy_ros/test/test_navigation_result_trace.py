from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus

from sstg_policy_ros.policy_node import (
    SSTGPolicyNode,
    _navigation_result_metadata,
)


@pytest.mark.parametrize(
    "termination_reason",
    ["action_budget", "distance_budget", "time_budget"],
)
def test_canceled_budget_result_uses_shared_session_termination_contract(
    termination_reason,
):
    metadata = _navigation_result_metadata(
        GoalStatus.STATUS_CANCELED,
        termination_reason,
    )

    assert metadata == {
        "nav2_status": GoalStatus.STATUS_CANCELED,
        "cancel_origin": "adapter_session_termination",
        "termination_reason": termination_reason,
    }


@pytest.mark.parametrize(
    ("nav2_status", "termination_reason"),
    [
        (GoalStatus.STATUS_SUCCEEDED, "distance_budget"),
        (GoalStatus.STATUS_CANCELED, None),
        (GoalStatus.STATUS_ABORTED, "distance_budget"),
        (None, "distance_budget"),
    ],
)
def test_only_canceled_budget_result_is_attributed_to_session_termination(
    nav2_status,
    termination_reason,
):
    metadata = _navigation_result_metadata(nav2_status, termination_reason)

    assert metadata == {
        "nav2_status": nav2_status,
        "cancel_origin": None,
    }


def test_goal_result_callback_forwards_nav2_terminal_status():
    calls = []
    node = SimpleNamespace(
        _finish_navigation=lambda *args, **kwargs: calls.append((args, kwargs))
    )
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
    )

    SSTGPolicyNode._goal_result_callback(node, future)

    assert calls == [
        (
            (False, f"nav2_status_{GoalStatus.STATUS_CANCELED}"),
            {"nav2_status": GoalStatus.STATUS_CANCELED},
        )
    ]


def test_finish_navigation_emits_distance_budget_cancel_causality():
    traces = []
    statuses = []
    recorded = []

    class Session:
        pending_decision = SimpleNamespace(decision_id=7)
        termination_reason = "running"

        def record_execution(self, decision_id, succeeded, pose, **kwargs):
            recorded.append((decision_id, succeeded, pose, kwargs))
            return SimpleNamespace(to_dict=lambda: {
                "decision_id": decision_id,
                "succeeded": succeeded,
                "reason": kwargs["reason"],
                "path": list(kwargs["executed_path"]),
                "translation_m": 1.0,
                "rotation_deg": 0.0,
            })

        @staticmethod
        def summary():
            return {"execution_count": 1}

        def terminate(self, reason):
            self.termination_reason = reason

    now = SimpleNamespace(nanoseconds=123_000_000)
    node = SimpleNamespace(
        execution_frame="odom",
        execution_path=[(0.0, 0.0)],
        session=Session(),
        termination_requested_reason="distance_budget",
        busy=True,
        goal_handle=object(),
        goal_started=object(),
        map_settle_s=1.0,
        _lookup_pose=lambda frame=None: (
            (0.9, 0.1, 2.0) if frame == "odom" else (1.0, 0.2, 3.0)
        ),
        _append_trace=lambda event, payload: traces.append((event, payload)),
        _publish_status=(
            lambda state, detail="": statuses.append((state, detail))
        ),
        get_clock=lambda: SimpleNamespace(now=lambda: now),
    )

    SSTGPolicyNode._finish_navigation(
        node,
        False,
        f"nav2_status_{GoalStatus.STATUS_CANCELED}",
        nav2_status=GoalStatus.STATUS_CANCELED,
    )

    assert recorded[0][3]["reason"] == "nav2_status_5:distance_budget"
    execution = next(
        payload for event, payload in traces if event == "execution"
    )
    assert execution["nav2_status"] == GoalStatus.STATUS_CANCELED
    assert execution["cancel_origin"] == "adapter_session_termination"
    assert execution["termination_reason"] == "distance_budget"
    assert execution["reason"] == "nav2_status_5:distance_budget"
    assert ("budget_reached", {"reason": "distance_budget"}) in traces
    assert statuses[-1] == ("BUDGET_EXHAUSTED", "distance_budget")
    assert node.termination_requested_reason is None
