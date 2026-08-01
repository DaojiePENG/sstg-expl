from collections import deque
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import ExternalShutdownException

from sstg_system_eval import evaluator_node


@pytest.mark.parametrize(
    "interruption", [KeyboardInterrupt, ExternalShutdownException, RuntimeError]
)
def test_main_cleans_up_after_executor_shutdown(monkeypatch, interruption):
    events = []

    class FakeNode:
        def _emit_snapshot(self, reason, *, publish):
            events.append(("snapshot", reason, publish))

        def destroy_node(self):
            events.append("destroy_node")

    monkeypatch.setattr(
        evaluator_node.rclpy, "init", lambda args=None: events.append("init")
    )
    monkeypatch.setattr(evaluator_node, "SystemEvaluatorNode", FakeNode)

    def interrupt(_node):
        events.append("spin")
        raise interruption("shutdown")

    monkeypatch.setattr(evaluator_node.rclpy, "spin", interrupt)
    monkeypatch.setattr(
        evaluator_node.rclpy,
        "ok",
        lambda: interruption is not RuntimeError,
    )
    monkeypatch.setattr(
        evaluator_node.rclpy,
        "try_shutdown",
        lambda: events.append("try_shutdown"),
    )

    evaluator_node.main(args=["--test"])

    assert events == [
        "init",
        "spin",
        ("snapshot", "evaluator_shutdown", False),
        "destroy_node",
        "try_shutdown",
    ]


def test_main_does_not_hide_runtime_error_while_context_is_live(monkeypatch):
    class FakeNode:
        def _emit_snapshot(self, _reason, *, publish):
            del publish

        def destroy_node(self):
            pass

    monkeypatch.setattr(evaluator_node.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(evaluator_node, "SystemEvaluatorNode", FakeNode)
    monkeypatch.setattr(
        evaluator_node.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(RuntimeError("live failure")),
    )
    monkeypatch.setattr(evaluator_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(evaluator_node.rclpy, "try_shutdown", lambda: None)

    with pytest.raises(RuntimeError, match="live failure"):
        evaluator_node.main()


def test_ate_settlement_waits_for_deadline_and_empty_queue():
    emitted = []
    now_ns = [199]
    fake = SimpleNamespace(
        _ate_session_finalizing=True,
        _ate_settlement_deadline_ns=200,
        _pending_ate=deque([(100, 1.0, 2.0)]),
        _now_ns=lambda: now_ns[0],
        _emit_snapshot=emitted.append,
    )

    assert (
        evaluator_node.SystemEvaluatorNode._maybe_emit_ate_settlement(fake)
        is False
    )
    fake._pending_ate.clear()
    assert (
        evaluator_node.SystemEvaluatorNode._maybe_emit_ate_settlement(fake)
        is False
    )
    now_ns[0] = 200

    assert (
        evaluator_node.SystemEvaluatorNode._maybe_emit_ate_settlement(fake)
        is True
    )
    assert fake._ate_session_finalizing is False
    assert emitted == ["policy_session_settled"]


def test_runtime_simulation_clock_cannot_be_disabled():
    rejected = evaluator_node.SystemEvaluatorNode._guard_simulation_clock(
        [SimpleNamespace(name="use_sim_time", value=False)]
    )
    accepted = evaluator_node.SystemEvaluatorNode._guard_simulation_clock(
        [SimpleNamespace(name="known_free_threshold", value=37)]
    )

    assert rejected.successful is False
    assert "cannot be disabled" in rejected.reason
    assert accepted.successful is True


def test_node_rejects_wall_time_before_creating_artifacts(tmp_path):
    output = tmp_path / "must_not_exist"
    rclpy.init(args=[
        "--ros-args",
        "-p",
        "use_sim_time:=false",
        "-p",
        f"output_dir:={output}",
    ])
    try:
        with pytest.raises(ValueError, match="use_sim_time must be true"):
            evaluator_node.SystemEvaluatorNode()
    finally:
        rclpy.try_shutdown()

    assert not output.exists()
