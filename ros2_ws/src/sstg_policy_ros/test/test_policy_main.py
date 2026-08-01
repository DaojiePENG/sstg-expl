import pytest
from rclpy.executors import ExternalShutdownException

from sstg_policy_ros import policy_node


@pytest.mark.parametrize(
    "interruption", [KeyboardInterrupt, ExternalShutdownException, RuntimeError]
)
def test_main_uses_idempotent_shutdown_after_executor_interrupt(
    monkeypatch, interruption
):
    events = []

    class FakeNode:
        def destroy_node(self):
            events.append("destroy_node")

    monkeypatch.setattr(
        policy_node.rclpy, "init", lambda args=None: events.append("init")
    )
    monkeypatch.setattr(policy_node, "SSTGPolicyNode", FakeNode)

    def interrupt(_node):
        events.append("spin")
        raise interruption

    monkeypatch.setattr(policy_node.rclpy, "spin", interrupt)
    monkeypatch.setattr(
        policy_node.rclpy,
        "ok",
        lambda: interruption is not RuntimeError,
    )
    monkeypatch.setattr(
        policy_node.rclpy,
        "try_shutdown",
        lambda: events.append("try_shutdown"),
    )

    policy_node.main(args=["--test"])

    assert events == ["init", "spin", "destroy_node", "try_shutdown"]


def test_main_does_not_hide_runtime_error_while_context_is_live(monkeypatch):
    class FakeNode:
        def destroy_node(self):
            pass

    monkeypatch.setattr(policy_node.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(policy_node, "SSTGPolicyNode", FakeNode)
    monkeypatch.setattr(
        policy_node.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(RuntimeError("live failure")),
    )
    monkeypatch.setattr(policy_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(policy_node.rclpy, "try_shutdown", lambda: None)

    with pytest.raises(RuntimeError, match="live failure"):
        policy_node.main()
