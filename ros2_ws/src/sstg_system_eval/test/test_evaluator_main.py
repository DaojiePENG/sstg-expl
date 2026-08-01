import pytest
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
