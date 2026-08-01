"""ROS graph tests for the external frontier NavigateToPose proxy.

These tests deliberately use real ROS 2 actions, services, simulated time,
and TF.  Unit-testing helper methods alone would miss the two races that can
change the upstream baseline's behavior: overlapping goals with out-of-order
results, and cancellation before the downstream action accepts a goal.
"""
from __future__ import annotations

import json
from pathlib import Path
import threading
import time
import uuid

import pytest

pytest.importorskip("rclpy", reason="requires a sourced ROS 2 workspace")
pytest.importorskip(
    "frontier_exploration_ros2.srv",
    reason="requires the built external frontier workspace",
)

from action_msgs.msg import GoalStatus
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty
from tf2_ros import TransformBroadcaster

from sstg_baseline_adapter.frontier_action_adapter import FrontierActionAdapter


def _wait_until(predicate, timeout_s=8.0, description="condition"):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for {description}")


def _future_result(future, timeout_s=8.0, description="ROS future"):
    _wait_until(future.done, timeout_s, description)
    return future.result()


class FakeSharedStack(Node):
    """Minimum Nav2/lifecycle/map/TF/control surface used by the adapter."""

    def __init__(self, context: Context, token: str) -> None:
        super().__init__(f"fake_shared_stack_{token}", context=context)
        self.token = token
        self.map_topic = f"/{token}/map"
        self.map_frame = f"{token}_map"
        self.odom_frame = f"{token}_odom"
        self.base_frame = f"{token}_base"
        self.nav2_action_name = f"/{token}/navigate_to_pose"
        self.lifecycle_service = f"/{token}/bt_navigator/get_state"
        self.control_service = f"/{token}/control_exploration"
        self.completion_topic = f"/{token}/exploration_complete"

        self.first_started = threading.Event()
        self.release_first = threading.Event()
        self.delayed_accept_entered = threading.Event()
        self.delayed_accept_release = threading.Event()
        self.late_accept_entered = threading.Event()
        self.late_accept_release = threading.Event()
        self.delayed_execute_started = threading.Event()
        self.downstream_cancel_seen = threading.Event()
        self.hung_execute_started = threading.Event()
        self.hung_release = threading.Event()
        self.hung_terminal = threading.Event()
        self.terminal_order: list[int] = []
        self.cancel_targets: list[int] = []
        self.control_actions: list[int] = []
        self._clock_ns = 1_000_000_000

        callback_group = ReentrantCallbackGroup()
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.clock_publisher = self.create_publisher(Clock, "/clock", clock_qos)
        self.map_publisher = self.create_publisher(
            OccupancyGrid, self.map_topic, latched_qos
        )
        self.completion_publisher = self.create_publisher(
            Empty, self.completion_topic, latched_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_service(
            GetState,
            self.lifecycle_service,
            self._lifecycle_state,
            callback_group=callback_group,
        )
        self.create_service(
            ControlExploration,
            self.control_service,
            self._control,
            callback_group=callback_group,
        )
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            self.nav2_action_name,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=callback_group,
        )
        self.world_timer = self.create_timer(0.01, self._publish_world)

    def _publish_world(self) -> None:
        self._clock_ns += 20_000_000
        clock = Clock()
        clock.clock.sec = self._clock_ns // 1_000_000_000
        clock.clock.nanosec = self._clock_ns % 1_000_000_000
        self.clock_publisher.publish(clock)

        transforms = []
        for parent, child in (
            (self.map_frame, self.odom_frame),
            (self.odom_frame, self.base_frame),
        ):
            transform = TransformStamped()
            transform.header.stamp = clock.clock
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf_broadcaster.sendTransform(transforms)

        grid = OccupancyGrid()
        grid.header.stamp = clock.clock
        grid.header.frame_id = self.map_frame
        grid.info.resolution = 1.0
        grid.info.width = 3
        grid.info.height = 3
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 9
        self.map_publisher.publish(grid)

    def _lifecycle_state(self, request, response):
        del request
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        response.current_state.label = "active"
        return response

    def _control(self, request, response):
        self.control_actions.append(int(request.action))
        response.accepted = True
        response.scheduled = False
        if request.action == ControlExploration.Request.ACTION_START:
            response.state = ControlExploration.Request.STATE_RUNNING
            response.message = "fake upstream started"
        else:
            response.state = ControlExploration.Request.STATE_IDLE
            response.message = "fake upstream stopped"
        return response

    def _goal(self, goal_request):
        target = round(goal_request.pose.pose.position.x)
        if target == 3:
            self.delayed_accept_entered.set()
            self.delayed_accept_release.wait(timeout=5.0)
        if target == 6:
            self.late_accept_entered.set()
            self.late_accept_release.wait(timeout=5.0)
        if target == 4:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        target = round(goal_handle.request.pose.pose.position.x)
        self.cancel_targets.append(target)
        if target == 5:
            return CancelResponse.REJECT
        self.downstream_cancel_seen.set()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        target = round(goal_handle.request.pose.pose.position.x)
        result = NavigateToPose.Result()
        if target == 1:
            self.first_started.set()
            if not self.release_first.wait(timeout=5.0):
                result.error_code = 90
                result.error_msg = "test did not release first goal"
            else:
                result.error_code = 41
                result.error_msg = "superseded first goal"
            goal_handle.abort()
            self.terminal_order.append(1)
            return result
        if target == 2:
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = 1.25
            feedback.current_pose = goal_handle.request.pose
            goal_handle.publish_feedback(feedback)
            result.error_code = 0
            goal_handle.succeed()
            self.terminal_order.append(2)
            return result
        if target in (7, 8):
            result.error_code = 0
            goal_handle.succeed()
            self.terminal_order.append(target)
            return result
        if target == 5:
            self.hung_execute_started.set()
            self.hung_release.wait(timeout=5.0)
            result.error_code = 92
            result.error_msg = "released rejected-cancel goal"
            goal_handle.abort()
            self.terminal_order.append(target)
            self.hung_terminal.set()
            return result

        self.delayed_execute_started.set()
        deadline = time.monotonic() + 5.0
        while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.005)
        if goal_handle.is_cancel_requested:
            result.error_code = 7
            result.error_msg = "canceled by proxy"
            goal_handle.canceled()
        else:
            result.error_code = 91
            result.error_msg = "cancel never arrived"
            goal_handle.abort()
        self.terminal_order.append(target)
        return result

    def unblock(self) -> None:
        self.release_first.set()
        self.delayed_accept_release.set()
        self.late_accept_release.set()
        self.hung_release.set()

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()


class RuntimeHarness:
    def __init__(self, output_dir: Path, *, max_decisions: int = 10) -> None:
        self.context = Context()
        self.token = "proxy_" + uuid.uuid4().hex[:10]
        domain_id = 100 + int(self.token[-4:], 16) % 100
        self.context.init(args=[], domain_id=domain_id)
        self.fake = FakeSharedStack(self.context, self.token)
        self.proxy_action_name = f"/{self.token}/proxy_navigate_to_pose"
        overrides = {
            "use_sim_time": True,
            "auto_start": True,
            "output_dir": str(output_dir),
            "map_topic": self.fake.map_topic,
            "map_frame": self.fake.map_frame,
            "execution_frame": self.fake.odom_frame,
            "base_frame": self.fake.base_frame,
            "proxy_action_name": self.proxy_action_name,
            "nav2_action_name": self.fake.nav2_action_name,
            "nav2_lifecycle_state_service": self.fake.lifecycle_service,
            "control_service_name": self.fake.control_service,
            "completion_topic": self.fake.completion_topic,
            "monitor_period_s": 0.01,
            "start_delay_s": 0.01,
            "max_duration_s": 120.0,
            "max_distance_m": 100.0,
            "max_decisions": max_decisions,
            "goal_timeout_s": 30.0,
            "cancel_grace_s": 0.2,
        }
        self.adapter = FrontierActionAdapter(
            context=self.context,
            parameter_overrides=[
                Parameter(name=name, value=value)
                for name, value in overrides.items()
            ],
        )
        self.client_node = Node(
            f"upstream_client_{self.token}", context=self.context
        )
        self.client = ActionClient(
            self.client_node, NavigateToPose, self.proxy_action_name
        )
        self.executor = MultiThreadedExecutor(
            num_threads=8, context=self.context
        )
        for node in (self.fake, self.adapter, self.client_node):
            self.executor.add_node(node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            name=f"executor_{self.token}",
            daemon=True,
        )
        self.spin_thread.start()
        _wait_until(
            self.client.server_is_ready,
            description="proxy action discovery",
        )
        _wait_until(
            lambda: any(
                record["event"] == "session_started"
                for record in self.trace_records()
            ),
            description="adapter session start",
        )

    def trace_records(self):
        path = self.adapter.trace_path
        if not path.exists():
            return []
        snapshot = path.read_text(encoding="utf-8")
        lines = snapshot.splitlines()
        if snapshot and not snapshot.endswith("\n"):
            lines = lines[:-1]
        return [
            json.loads(line)
            for line in lines
            if line.strip()
        ]

    def send_goal(self, x: float, feedback_callback=None):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.fake.map_frame
        goal.pose.pose.position.x = x
        goal.pose.pose.orientation.w = 1.0
        future = self.client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        handle = _future_result(future, description=f"proxy goal {x} response")
        assert handle.accepted
        return handle

    def finish_from_upstream(self):
        self.fake.completion_publisher.publish(Empty())
        _wait_until(
            lambda: sum(
                record["event"] == "session_finished"
                for record in self.trace_records()
            ) == 1,
            description="single finalized session",
        )
        _wait_until(
            lambda: ControlExploration.Request.ACTION_STOP
            in self.fake.control_actions,
            description="upstream STOP service request",
        )

    def close(self) -> None:
        self.fake.unblock()
        if self.fake.hung_execute_started.is_set():
            _wait_until(
                self.fake.hung_terminal.is_set,
                description="hung fake action cleanup",
            )
        if self.adapter._stop_request_sent:
            _wait_until(
                lambda: self.adapter._stop_response_received,
                description="adapter STOP response callback",
            )
        for context in list(self.adapter._goals.values()):
            if context.cancel_sent:
                _wait_until(
                    lambda item=context: item.cancel_response_received,
                    description="downstream cancel response callback",
                )
            if context.downstream_goal is not None:
                _wait_until(
                    lambda item=context: (
                        item.downstream_result_response_received
                    ),
                    description="downstream result response callback",
                )
        # Stop sources of new work, then put executor barriers behind callbacks
        # already queued by action/service futures before destroying entities.
        self.adapter.timer.cancel()
        self.fake.world_timer.cancel()
        time.sleep(0.05)
        barriers = [self.executor.create_task(lambda: None) for _ in range(8)]
        for barrier in barriers:
            _future_result(barrier, description="executor teardown barrier")
        self.executor.shutdown(timeout_sec=5.0)
        self.spin_thread.join(timeout=5.0)
        self.client.destroy()
        self.client_node.destroy_node()
        self.adapter.destroy_node()
        self.fake.destroy_node()
        if rclpy.ok(context=self.context):
            self.context.shutdown()


@pytest.fixture
def runtime_harness(tmp_path):
    harness = RuntimeHarness(tmp_path / "policy")
    try:
        yield harness
    finally:
        harness.close()


def test_overlapping_goals_forward_feedback_and_keep_out_of_order_identity(
    runtime_harness,
):
    harness = runtime_harness
    feedback_distances = []
    first = harness.send_goal(1.0)
    _wait_until(harness.fake.first_started.is_set, description="first execution")
    second = harness.send_goal(
        2.0,
        feedback_callback=lambda message: feedback_distances.append(
            message.feedback.distance_remaining
        ),
    )

    second_result = _future_result(
        second.get_result_async(), description="second proxy result"
    )
    harness.fake.release_first.set()
    first_result = _future_result(
        first.get_result_async(), description="first proxy result"
    )
    _wait_until(lambda: feedback_distances, description="forwarded feedback")

    assert second_result.status == GoalStatus.STATUS_SUCCEEDED
    assert second_result.result.error_code == 0
    assert first_result.status == GoalStatus.STATUS_ABORTED
    assert first_result.result.error_code == 41
    assert first_result.result.error_msg == "superseded first goal"
    assert feedback_distances == pytest.approx([1.25])
    assert harness.fake.terminal_order[:2] == [2, 1]

    _wait_until(
        lambda: len([
            record for record in harness.trace_records()
            if record["event"] == "execution"
        ]) == 2,
        description="two execution trace records",
    )
    executions = [
        record["payload"] for record in harness.trace_records()
        if record["event"] == "execution"
    ]
    assert [record["decision_id"] for record in executions] == [2, 1]
    by_decision = {record["decision_id"]: record for record in executions}
    assert by_decision[2]["succeeded"] is True
    assert by_decision[1]["succeeded"] is False
    assert by_decision[1]["upstream_preemption"] is True
    assert by_decision[1]["cancel_origin"] == "nav2_native_preemption"
    assert "superseded_by_2" in by_decision[1]["reason"]
    assert by_decision[1]["nav2_error_code"] == 41
    for execution in executions:
        assert execution["upstream_goal_uuid"]
        assert execution["downstream_goal_uuid"]
        assert execution["upstream_goal_uuid"] != execution["downstream_goal_uuid"]

    harness.finish_from_upstream()
    manifest = json.loads(harness.adapter.manifest_path.read_text())
    assert manifest["runtime_adapter"] == "frontier_mrtsp_dp_external"
    assert manifest["adapter_contract"] == "navigate_to_pose_transparent_proxy_v1"
    assert harness.fake.control_actions[0] == ControlExploration.Request.ACTION_START
    assert ControlExploration.Request.ACTION_STOP in harness.fake.control_actions


def test_upstream_cancel_is_latched_until_downstream_goal_acceptance(
    runtime_harness,
):
    harness = runtime_harness
    goal = harness.send_goal(3.0)
    _wait_until(
        harness.fake.delayed_accept_entered.is_set,
        description="blocked downstream goal acceptance",
    )

    cancel_response = _future_result(
        goal.cancel_goal_async(), description="upstream cancel response"
    )
    assert len(cancel_response.goals_canceling) == 1
    harness.fake.delayed_accept_release.set()
    _wait_until(
        harness.fake.delayed_execute_started.is_set,
        description="delayed downstream execution",
    )
    wrapped = _future_result(
        goal.get_result_async(), description="canceled proxy result"
    )

    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert wrapped.result.error_code == 7
    assert harness.fake.downstream_cancel_seen.is_set()
    _wait_until(
        lambda: any(
            record["event"] == "execution"
            for record in harness.trace_records()
        ),
        description="cancel execution trace",
    )
    execution = next(
        record["payload"] for record in harness.trace_records()
        if record["event"] == "execution"
    )
    assert execution["nav2_status"] == GoalStatus.STATUS_CANCELED
    assert execution["cancel_origin"] == "upstream_cancel_request"
    assert execution["succeeded"] is False

    harness.finish_from_upstream()
    assert sum(
        record["event"] == "session_finished"
        for record in harness.trace_records()
    ) == 1


def test_downstream_rejection_is_explicitly_traced_and_aborts_upstream(
    runtime_harness,
):
    harness = runtime_harness
    goal = harness.send_goal(4.0)
    wrapped = _future_result(
        goal.get_result_async(), description="rejected downstream result"
    )

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    _wait_until(
        lambda: any(
            record["event"] == "execution"
            for record in harness.trace_records()
        ),
        description="rejection execution trace",
    )
    execution = next(
        record["payload"] for record in harness.trace_records()
        if record["event"] == "execution"
    )
    assert execution["reason"] == "downstream_goal_rejected"
    assert execution["downstream_goal_uuid"] is None
    assert execution["nav2_status"] is None
    assert execution["cancel_origin"] is None
    assert execution["succeeded"] is False

    harness.finish_from_upstream()


def test_rejected_downstream_cancel_forces_audited_local_terminal(
    runtime_harness,
):
    harness = runtime_harness
    goal = harness.send_goal(5.0)
    _wait_until(
        harness.fake.hung_execute_started.is_set,
        description="non-terminating downstream execution",
    )
    cancel_response = _future_result(
        goal.cancel_goal_async(), description="upstream cancel response"
    )
    assert len(cancel_response.goals_canceling) == 1

    wrapped = _future_result(
        goal.get_result_async(), description="cancel grace local terminal"
    )
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert harness.fake.cancel_targets == [5]
    records = harness.trace_records()
    cancel_response_record = next(
        record["payload"] for record in records
        if record["event"] == "downstream_cancel_response"
    )
    assert cancel_response_record["return_code"] == 0
    assert cancel_response_record["accepted"] is False
    assert cancel_response_record["return_code_semantics"] == (
        "rejected_empty_goal_list"
    )
    assert cancel_response_record["goals_canceling"] == 0
    forced = next(
        record["payload"] for record in records
        if record["event"] == "cancel_grace_expired"
    )
    assert forced["cancel_origin"] == "upstream_cancel_request"
    assert forced["cancel_response"] == {
        "return_code": 0,
        "goals_canceling": 0,
    }
    execution = next(
        record["payload"] for record in records
        if record["event"] == "execution"
    )
    assert execution["reason"] == (
        "cancel_grace_expired:upstream_cancel_request"
    )
    harness.finish_from_upstream()


def test_late_downstream_acceptance_honors_already_forced_cancel(
    runtime_harness,
):
    harness = runtime_harness
    goal = harness.send_goal(6.0)
    _wait_until(
        harness.fake.late_accept_entered.is_set,
        description="blocked late downstream acceptance",
    )
    cancel_response = _future_result(
        goal.cancel_goal_async(), description="late-accept upstream cancel"
    )
    assert len(cancel_response.goals_canceling) == 1
    wrapped = _future_result(
        goal.get_result_async(), description="pre-accept local terminal"
    )
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    forced = next(
        record["payload"] for record in harness.trace_records()
        if record["event"] == "cancel_grace_expired"
    )
    assert forced["cancel_response"] is None

    harness.fake.late_accept_release.set()
    _wait_until(
        harness.fake.downstream_cancel_seen.is_set,
        description="late accepted downstream cancel forwarding",
    )
    _wait_until(
        lambda: 6 in harness.fake.terminal_order,
        description="late accepted downstream terminal",
    )
    harness.finish_from_upstream()


def test_pending_acceptance_reserves_budget_and_delays_session_finalization(
    tmp_path,
):
    harness = RuntimeHarness(tmp_path / "budget_policy", max_decisions=1)
    lock_held = False
    try:
        harness.adapter._acceptance_lock.acquire()
        lock_held = True

        first_goal = NavigateToPose.Goal()
        first_goal.pose.header.frame_id = harness.fake.map_frame
        first_goal.pose.pose.position.x = 7.0
        first_goal.pose.pose.orientation.w = 1.0
        first_future = harness.client.send_goal_async(first_goal)
        _wait_until(
            lambda: harness.adapter._pending_goal_acceptances == 1,
            description="reserved decision slot",
        )

        second_goal = NavigateToPose.Goal()
        second_goal.pose.header.frame_id = harness.fake.map_frame
        second_goal.pose.pose.position.x = 8.0
        second_goal.pose.pose.orientation.w = 1.0
        second_handle = _future_result(
            harness.client.send_goal_async(second_goal),
            description="over-budget goal rejection",
        )
        assert second_handle.accepted is False
        assert not any(
            record["event"] == "session_finished"
            for record in harness.trace_records()
        )

        harness.adapter._acceptance_lock.release()
        lock_held = False
        first_handle = _future_result(
            first_future, description="reserved goal acceptance"
        )
        assert first_handle.accepted is True
        _future_result(
            first_handle.get_result_async(),
            description="reserved goal terminal result",
        )
        _wait_until(
            lambda: any(
                record["event"] == "session_finished"
                for record in harness.trace_records()
            ),
            description="post-registration session finalization",
        )
        decisions = [
            record for record in harness.trace_records()
            if record["event"] == "decision"
        ]
        assert len(decisions) == 1
        assert decisions[0]["payload"]["decision_id"] == 1
        finished = next(
            record["payload"] for record in harness.trace_records()
            if record["event"] == "session_finished"
        )
        assert finished["decision_count"] == 1
        assert finished["termination_reason"] == "action_budget"
    finally:
        if lock_held:
            harness.adapter._acceptance_lock.release()
        harness.close()
