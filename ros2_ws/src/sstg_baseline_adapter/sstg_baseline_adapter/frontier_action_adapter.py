"""Lifecycle, budget, trace, and action proxy for a pinned frontier baseline.

The independently maintained explorer remains the only target-selection
implementation.  This node exposes a separate NavigateToPose server to that
explorer, forwards every overlapping goal to the shared Nav2 server, and owns
only the experiment contract around those actions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
from typing import Any, Optional

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .contracts import jsonable, path_length, shortest_rotation_deg, validate_budget


UPSTREAM_COMPONENT_ID = "frontier_mrtsp_dp_external_v1_6_1"
UPSTREAM_REPOSITORY = "https://github.com/mertgulerx/frontier_exploration_ros2"
UPSTREAM_TAG = "v1.6.1"
UPSTREAM_COMMIT = "b0fad500e5c81ad3154f0469ca283b2702a3f90c"
ALGORITHM_IDENTITY = "wfd_decision_map_mrtsp_bounded_horizon_dp_with_preemption"


def _yaw_from_quaternion(rotation: Any) -> float:
    siny = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny, cosy)


def _pose_from_transform(transform: Any) -> tuple[float, float, float]:
    return (
        float(transform.translation.x),
        float(transform.translation.y),
        _yaw_from_quaternion(transform.rotation),
    )


def _pose_message_values(message: PoseStamped) -> list[float]:
    position = message.pose.position
    orientation = message.pose.orientation
    return [
        float(position.x),
        float(position.y),
        _yaw_from_quaternion(orientation),
    ]


def _uuid_hex(value: Any) -> str:
    return bytes(value.uuid).hex()


@dataclass
class ForwardedGoal:
    key: bytes
    decision_id: int
    server_goal: Any
    accepted_ros_time_ns: int
    target_pose: list[float]
    target_frame: str
    execution_path: list[list[float]] = field(default_factory=list)
    map_path: list[list[float]] = field(default_factory=list)
    downstream_goal: Any = None
    downstream_status: Optional[int] = None
    downstream_result: Any = None
    transport_error: str = ""
    cancel_sent: bool = False
    cancel_origin: Optional[str] = None
    cancel_requested_ros_time_ns: Optional[int] = None
    cancel_response_code: Optional[int] = None
    cancel_response_goal_count: Optional[int] = None
    cancel_response_received: bool = False
    downstream_result_response_received: bool = False
    local_terminal_forced: bool = False
    timeout_requested: bool = False
    superseded_by_decision_id: Optional[int] = None
    distance_accounted: bool = False
    completed: bool = False
    result_event: threading.Event = field(default_factory=threading.Event)


class FrontierActionAdapter(Node):
    """Forward Nav2 actions while enforcing the common experiment contract."""

    def __init__(
        self,
        *,
        context=None,
        parameter_overrides=None,
    ) -> None:
        super().__init__(
            "frontier_baseline_adapter",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self._declare_parameters()
        self._validate_parameters()

        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        budget = validate_budget({
            name: self.get_parameter(name).value
            for name in (
                "max_duration_s",
                "max_distance_m",
                "max_decisions",
                "goal_timeout_s",
            )
        })
        self.max_duration_s = float(budget["max_duration_s"])
        self.max_distance_m = float(budget["max_distance_m"])
        self.max_decisions = int(budget["max_decisions"])
        self.goal_timeout_s = float(budget["goal_timeout_s"])
        self.cancel_grace_s = float(
            self.get_parameter("cancel_grace_s").value
        )
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.execution_frame = str(
            self.get_parameter("execution_frame").value
        )
        self.proxy_action_name = str(
            self.get_parameter("proxy_action_name").value
        )
        self.nav2_action_name = str(
            self.get_parameter("nav2_action_name").value
        )
        self.control_service_name = str(
            self.get_parameter("control_service_name").value
        )
        self.nav2_lifecycle_state_service = str(
            self.get_parameter("nav2_lifecycle_state_service").value
        )
        self.completion_topic = str(
            self.get_parameter("completion_topic").value
        )
        self.method_id = str(self.get_parameter("method_id").value)

        self._state_lock = threading.RLock()
        self._acceptance_lock = threading.Lock()
        self._trace_lock = threading.Lock()
        self._goals: dict[bytes, ForwardedGoal] = {}
        self._current_goal_key: Optional[bytes] = None
        self._decision_count = 0
        self._pending_goal_acceptances = 0
        self._execution_count = 0
        self._success_count = 0
        self._completed_distance_m = 0.0
        self._topological_nodes: list[list[float]] = []
        self._map_received = False
        self._map_revision = 0
        self._nav2_lifecycle_active = False
        self._nav2_state_future = None
        self._nav2_state_last_request_ns: Optional[int] = None
        self._enabled = bool(self.get_parameter("auto_start").value)
        self._start_request_pending = False
        self._session_active = False
        self._session_started_ns: Optional[int] = None
        self._session_initial_pose: Optional[list[float]] = None
        self._pending_initial_pose: Optional[list[float]] = None
        self._termination_reason: Optional[str] = None
        self._stop_request_sent = False
        self._stop_response_received = False
        self._finished = False

        self._prepare_output()
        self.add_on_set_parameters_callback(self._guard_simulation_clock)

        callback_group = ReentrantCallbackGroup()
        trace_qos = QoSProfile(depth=200)
        trace_qos.reliability = ReliabilityPolicy.RELIABLE
        trace_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        completion_qos = QoSProfile(depth=1)
        completion_qos.reliability = ReliabilityPolicy.RELIABLE
        completion_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.trace_publisher = self.create_publisher(
            String, "/policy/decision_trace", trace_qos
        )
        self.status_publisher = self.create_publisher(
            String, "/policy/status", status_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/policy/candidates", 10
        )
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Empty,
            self.completion_topic,
            self._completion_callback,
            completion_qos,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger, "/policy/start", self._policy_start_callback,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger, "/policy/stop", self._policy_stop_callback,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger, "/policy/reset", self._policy_reset_callback,
            callback_group=callback_group,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            self.nav2_action_name,
            callback_group=callback_group,
        )
        self.control_client = self.create_client(
            ControlExploration,
            self.control_service_name,
            callback_group=callback_group,
        )
        self.nav2_lifecycle_client = self.create_client(
            GetState,
            self.nav2_lifecycle_state_service,
            callback_group=callback_group,
        )
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            self.proxy_action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self.timer = self.create_timer(
            float(self.get_parameter("monitor_period_s").value),
            self._tick,
            callback_group=callback_group,
        )
        self._publish_status("WAIT_STACK")

    def _declare_parameters(self) -> None:
        defaults = {
            "use_sim_time": True,
            "auto_start": True,
            "method_id": "frontier_mrtsp_dp_external",
            "map_topic": "/map",
            "map_frame": "map",
            "base_frame": "base_footprint",
            "execution_frame": "odom",
            "proxy_action_name": "/baseline/frontier_mrtsp_dp/navigate_to_pose",
            "nav2_action_name": "/navigate_to_pose",
            "nav2_lifecycle_state_service": "/bt_navigator/get_state",
            "control_service_name": "/control_exploration",
            "completion_topic": (
                "/baseline/frontier_mrtsp_dp/exploration_complete"
            ),
            "monitor_period_s": 0.1,
            "start_delay_s": 0.5,
            "goal_timeout_s": 180.0,
            "cancel_grace_s": 5.0,
            "max_duration_s": 900.0,
            "max_distance_m": 150.0,
            "max_decisions": 100,
            "topological_merge_distance_m": 0.25,
            "policy_seed": 42,
            "allow_existing_output": False,
            "output_dir": "system_sim_outputs/runs/development/manual",
        }
        for name, value in defaults.items():
            # rclpy's TimeSource declares use_sim_time while Node is being
            # constructed.  Keep the explicit experiment default above, but
            # do not redeclare parameters that the base class already owns.
            if not self.has_parameter(name):
                self.declare_parameter(name, value)

    def _validate_parameters(self) -> None:
        if self.get_parameter("use_sim_time").value is not True:
            raise ValueError("use_sim_time must be true for simulation baselines")
        for name in (
            "method_id",
            "map_topic",
            "map_frame",
            "base_frame",
            "execution_frame",
            "proxy_action_name",
            "nav2_action_name",
            "nav2_lifecycle_state_service",
            "control_service_name",
            "completion_topic",
        ):
            if not str(self.get_parameter(name).value).strip():
                raise ValueError(f"{name} must be non-empty")
        if self.get_parameter("proxy_action_name").value == self.get_parameter(
            "nav2_action_name"
        ).value:
            raise ValueError("proxy_action_name must differ from nav2_action_name")
        for name in ("monitor_period_s", "start_delay_s", "cancel_grace_s"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        merge_distance = float(
            self.get_parameter("topological_merge_distance_m").value
        )
        if not math.isfinite(merge_distance) or merge_distance < 0.0:
            raise ValueError(
                "topological_merge_distance_m must be finite and non-negative"
            )

    @staticmethod
    def _guard_simulation_clock(parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "use_sim_time" and parameter.value is not True:
                return SetParametersResult(
                    successful=False,
                    reason="use_sim_time cannot be disabled during a simulation run",
                )
        return SetParametersResult(successful=True)

    def _prepare_output(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.trace_path = self.output_dir / "policy_trace.jsonl"
        self.manifest_path = self.output_dir / "policy_manifest.json"
        if not bool(self.get_parameter("allow_existing_output").value):
            existing = [
                path for path in (self.trace_path, self.manifest_path)
                if path.exists()
            ]
            if existing:
                names = ", ".join(path.name for path in existing)
                raise FileExistsError(
                    f"refusing to reuse policy output artifacts: {names}"
                )
        manifest = {
            "schema": "sstg_system_sim_policy_manifest/v1",
            "node": self.get_fully_qualified_name(),
            "evidence_source": "system_simulation",
            "truth_access": False,
            "map_topic": str(self.get_parameter("map_topic").value),
            "map_frame": str(self.get_parameter("map_frame").value),
            "base_frame": str(self.get_parameter("base_frame").value),
            "execution_frame": str(
                self.get_parameter("execution_frame").value
            ),
            "navigate_action": str(
                self.get_parameter("nav2_action_name").value
            ),
            "runtime_adapter": "frontier_mrtsp_dp_external",
            "adapter_contract": "navigate_to_pose_transparent_proxy_v1",
            "upstream": {
                "component_id": UPSTREAM_COMPONENT_ID,
                "repository": UPSTREAM_REPOSITORY,
                "release_tag": UPSTREAM_TAG,
                "commit": UPSTREAM_COMMIT,
                "algorithm_identity": ALGORITHM_IDENTITY,
                "source_modified": False,
            },
            "policy_seed_applicable": False,
            "parameters": {
                name: self.get_parameter(name).value
                for name in self._parameters
            },
        }
        self.manifest_path.write_text(
            json.dumps(jsonable(manifest), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _append_trace(self, event: str, payload: Any) -> None:
        record = {
            "event": event,
            "ros_time_ns": self.get_clock().now().nanoseconds,
            "map_revision": self._map_revision,
            "payload": jsonable(payload),
        }
        encoded = json.dumps(record, sort_keys=True, allow_nan=False)
        with self._trace_lock:
            with self.trace_path.open("a", encoding="utf-8") as stream:
                stream.write(encoded + "\n")
        message = String()
        message.data = encoded
        self.trace_publisher.publish(message)

    def _publish_status(self, state: str, detail: str = "") -> None:
        message = String()
        message.data = json.dumps({
            "state": state,
            "detail": detail,
            "runtime_adapter": "navigate_to_pose_transparent_proxy_v1",
        }, sort_keys=True)
        self.status_publisher.publish(message)

    def _lookup_pose(self, target_frame: str) -> Optional[list[float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        return list(_pose_from_transform(transform.transform))

    def _map_callback(self, message: OccupancyGrid) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().error(
                f"Ignoring map frame {message.header.frame_id!r}; "
                f"expected {self.map_frame!r}"
            )
            return
        with self._state_lock:
            self._map_received = True
            self._map_revision += 1

    def _policy_start_callback(self, request, response):
        del request
        with self._state_lock:
            if self._finished:
                response.success = False
                response.message = "one-session output is already finalized"
                return response
            self._enabled = True
        response.success = True
        response.message = "baseline start enabled; waiting for shared stack"
        self._publish_status("START_REQUESTED")
        return response

    def _policy_stop_callback(self, request, response):
        del request
        self._request_termination("manual_stop")
        response.success = True
        response.message = "baseline stop and active-goal cancellation requested"
        return response

    def _policy_reset_callback(self, request, response):
        del request
        with self._state_lock:
            allowed = not self._session_active and not self._finished
            if allowed:
                self._start_request_pending = False
        response.success = allowed
        response.message = (
            "cold-idle readiness reset"
            if allowed else
            "one-session experiment cannot reset after start"
        )
        return response

    def _request_upstream_start(self) -> None:
        initial_pose = self._lookup_pose(self.map_frame)
        if initial_pose is None:
            return
        with self._state_lock:
            if self._start_request_pending or self._session_active or self._finished:
                return
            self._start_request_pending = True
            self._pending_initial_pose = initial_pose
        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_START
        request.delay_seconds = float(self.get_parameter("start_delay_s").value)
        request.quit_after_stop = False
        future = self.control_client.call_async(request)
        future.add_done_callback(self._upstream_start_response)
        self._publish_status("STARTING_UPSTREAM")

    def _upstream_start_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            with self._state_lock:
                self._start_request_pending = False
                self._pending_initial_pose = None
            self.get_logger().error(f"Upstream start service failed: {error}")
            return
        if response is None or not response.accepted:
            with self._state_lock:
                self._start_request_pending = False
                self._pending_initial_pose = None
            detail = "empty response" if response is None else response.message
            self.get_logger().warn(f"Upstream start rejected: {detail}")
            return
        with self._state_lock:
            initial_pose = self._pending_initial_pose
        if initial_pose is None:
            with self._state_lock:
                self._start_request_pending = False
                self._pending_initial_pose = None
            self.get_logger().error("Upstream start accepted without map-frame TF")
            self._request_termination("start_tf_lost")
            return
        now_ns = self.get_clock().now().nanoseconds
        with self._state_lock:
            self._session_active = True
            self._session_started_ns = now_ns
            self._session_initial_pose = initial_pose
            self._topological_nodes = [initial_pose[:2]]
            self._pending_initial_pose = None
            self._start_request_pending = False
        self._append_trace("session_started", {
            "method": self.method_id,
            "runtime_adapter": "navigate_to_pose_transparent_proxy_v1",
            "upstream_component_id": UPSTREAM_COMPONENT_ID,
            "nodes": [{"id": 0, "position": initial_pose[:2]}],
            "experiment_budget": {
                "max_duration_s": self.max_duration_s,
                "max_distance_m": self.max_distance_m,
                "max_decisions": self.max_decisions,
                "goal_timeout_s": self.goal_timeout_s,
            },
            "initial_pose": initial_pose,
            "start_delay_s": float(self.get_parameter("start_delay_s").value),
        })
        self._publish_status("RUNNING", response.message)

    def _goal_callback(self, goal_request) -> GoalResponse:
        pose = goal_request.pose
        values = _pose_message_values(pose)
        valid = (
            pose.header.frame_id == self.map_frame
            and all(math.isfinite(value) for value in values)
        )
        budget_reached = False
        with self._state_lock:
            accept = (
                valid
                and self._session_active
                and self._termination_reason is None
                and not self._finished
            )
            reserved_decisions = (
                self._decision_count + self._pending_goal_acceptances
            )
            if accept and reserved_decisions >= self.max_decisions:
                accept = False
                budget_reached = True
            elif accept:
                self._pending_goal_acceptances += 1
        if budget_reached:
            self._request_termination("action_budget")
        if not valid:
            self.get_logger().error(
                "Rejected upstream goal with non-finite pose or wrong map frame"
            )
        return GoalResponse.ACCEPT if accept else GoalResponse.REJECT

    def _handle_accepted_callback(self, goal_handle) -> None:
        # ActionServer may invoke accepted callbacks concurrently.  Serialize
        # registration through execute() so decision IDs, supersession links,
        # and downstream dispatch order cannot be inverted by TF lookup time.
        with self._acceptance_lock:
            self._register_accepted_goal(goal_handle)

    def _register_accepted_goal(self, goal_handle) -> None:
        key = bytes(goal_handle.goal_id.uuid)
        execution_pose = self._lookup_pose(self.execution_frame)
        map_pose = self._lookup_pose(self.map_frame)
        target = _pose_message_values(goal_handle.request.pose)
        with self._state_lock:
            if self._pending_goal_acceptances <= 0:
                self.get_logger().error(
                    "Accepted goal had no reserved experiment decision slot"
                )
            else:
                self._pending_goal_acceptances -= 1
            self._decision_count += 1
            decision_id = self._decision_count
            if self._current_goal_key in self._goals:
                previous = self._goals[self._current_goal_key]
                if not previous.completed:
                    previous.superseded_by_decision_id = decision_id
                    if execution_pose is not None:
                        self._append_path_sample(previous, execution_pose, map_pose)
            context = ForwardedGoal(
                key=key,
                decision_id=decision_id,
                server_goal=goal_handle,
                accepted_ros_time_ns=self.get_clock().now().nanoseconds,
                target_pose=target,
                target_frame=goal_handle.request.pose.header.frame_id,
            )
            if execution_pose is not None:
                self._append_path_sample(context, execution_pose, map_pose)
            self._goals[key] = context
            self._current_goal_key = key
        self._append_trace("decision", {
            "decision_id": decision_id,
            "status": "navigate",
            "reason": "upstream_navigate_to_pose_request",
            "target_pose": target,
            "target_frame": context.target_frame,
            "upstream_goal_uuid": _uuid_hex(goal_handle.goal_id),
            "decision_time_ms": None,
            "decision_time_semantics": (
                "unavailable_upstream_internal_compute_time"
            ),
            "upstream_component_id": UPSTREAM_COMPONENT_ID,
        })
        self._publish_target_marker(goal_handle.request.pose, decision_id)
        self._publish_status("NAVIGATING", str(decision_id))
        goal_handle.execute()

    @staticmethod
    def _append_path_sample(
        context: ForwardedGoal,
        execution_pose: list[float],
        map_pose: Optional[list[float]],
    ) -> None:
        point = [float(execution_pose[0]), float(execution_pose[1])]
        if not context.execution_path or context.execution_path[-1] != point:
            context.execution_path.append(point)
        if map_pose is not None:
            mapped = [float(value) for value in map_pose]
            if not context.map_path or context.map_path[-1] != mapped:
                context.map_path.append(mapped)

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        key = bytes(goal_handle.goal_id.uuid)
        with self._state_lock:
            context = self._goals.get(key)
        if context is not None:
            # GoalHandle.is_cancel_requested becomes false after canceled() is
            # called, so preserve the first causal event for the final trace.
            self._cancel_downstream(context, "upstream_cancel_request")
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        key = bytes(goal_handle.goal_id.uuid)
        with self._state_lock:
            context = self._goals.get(key)
        if context is None:
            goal_handle.abort()
            return NavigateToPose.Result()
        try:
            future = self.nav2_client.send_goal_async(
                goal_handle.request,
                feedback_callback=lambda feedback, goal_key=key: (
                    self._forward_feedback(goal_key, feedback)
                ),
            )
            future.add_done_callback(
                lambda result, goal_key=key: self._downstream_goal_response(
                    goal_key, result
                )
            )
        except Exception as error:
            context.transport_error = f"send_goal:{error}"
            context.result_event.set()

        while not context.result_event.wait(0.02):
            if not rclpy.ok(context=self.context):
                context.transport_error = "context_shutdown"
                break
            elapsed_ns = (
                self.get_clock().now().nanoseconds
                - context.accepted_ros_time_ns
            )
            if (
                elapsed_ns >= int(self.goal_timeout_s * 1e9)
                and not context.timeout_requested
            ):
                context.timeout_requested = True
                self._append_trace("goal_timeout", {
                    "decision_id": context.decision_id,
                    "goal_timeout_s": self.goal_timeout_s,
                })
            with self._state_lock:
                terminate = self._termination_reason is not None
            if goal_handle.is_cancel_requested:
                self._cancel_downstream(context, "upstream_cancel_request")
            elif context.timeout_requested:
                self._cancel_downstream(context, "adapter_goal_timeout")
            elif terminate:
                self._cancel_downstream(context, "adapter_session_termination")

            cancel_started_ns = context.cancel_requested_ros_time_ns
            now_ns = self.get_clock().now().nanoseconds
            if (
                cancel_started_ns is not None
                and now_ns >= cancel_started_ns
                and now_ns - cancel_started_ns
                >= int(self.cancel_grace_s * 1e9)
            ):
                with self._state_lock:
                    if not context.local_terminal_forced:
                        context.local_terminal_forced = True
                        context.transport_error = (
                            "cancel_grace_expired:"
                            f"{context.cancel_origin}"
                        )
                        cancel_response_code = context.cancel_response_code
                        cancel_response_goal_count = (
                            context.cancel_response_goal_count
                        )
                        force_now = True
                    else:
                        cancel_response_code = None
                        cancel_response_goal_count = None
                        force_now = False
                if cancel_response_code is not None or (
                    cancel_response_goal_count is not None
                ):
                    response_evidence = {
                        "return_code": cancel_response_code,
                        "goals_canceling": cancel_response_goal_count,
                    }
                else:
                    response_evidence = None
                if force_now:
                    self._append_trace("cancel_grace_expired", {
                        "decision_id": context.decision_id,
                        "cancel_origin": context.cancel_origin,
                        "cancel_grace_s": self.cancel_grace_s,
                        "cancel_response": response_evidence,
                    })
                    context.result_event.set()

        result = context.downstream_result or NavigateToPose.Result()
        status = context.downstream_status
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if succeeded and goal_handle.is_active:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested and goal_handle.is_active:
            goal_handle.canceled()
        elif goal_handle.is_active:
            goal_handle.abort()

        self._record_execution(context, succeeded)
        with self._state_lock:
            context.completed = True
            if self._current_goal_key == key:
                self._current_goal_key = None
            should_stop_for_actions = (
                self._decision_count >= self.max_decisions
                and self._termination_reason is None
            )
        if should_stop_for_actions:
            self._request_termination("action_budget")
        self._finish_session_if_quiescent()
        return result

    def _downstream_goal_response(self, key: bytes, future) -> None:
        with self._state_lock:
            context = self._goals.get(key)
        if context is None:
            return
        try:
            downstream_goal = future.result()
        except Exception as error:
            context.transport_error = f"goal_response:{error}"
            context.result_event.set()
            return
        if downstream_goal is None or not downstream_goal.accepted:
            context.transport_error = "downstream_goal_rejected"
            context.result_event.set()
            return
        context.downstream_goal = downstream_goal
        with self._state_lock:
            terminate = self._termination_reason is not None
            latched_cancel_origin = context.cancel_origin
        if latched_cancel_origin is not None:
            self._cancel_downstream(context, latched_cancel_origin)
        elif context.server_goal.is_cancel_requested:
            self._cancel_downstream(context, "upstream_cancel_request")
        elif context.timeout_requested:
            self._cancel_downstream(context, "adapter_goal_timeout")
        elif terminate:
            self._cancel_downstream(context, "adapter_session_termination")
        try:
            result_future = downstream_goal.get_result_async()
            result_future.add_done_callback(
                lambda result, goal_key=key: self._downstream_result(
                    goal_key, result
                )
            )
        except Exception as error:
            context.transport_error = f"get_result_request:{error}"
            context.result_event.set()

    def _downstream_result(self, key: bytes, future) -> None:
        with self._state_lock:
            context = self._goals.get(key)
        if context is None:
            return
        try:
            wrapped = future.result()
            with self._state_lock:
                context.downstream_status = int(wrapped.status)
                context.downstream_result = wrapped.result
        except Exception as error:
            with self._state_lock:
                context.transport_error = f"get_result:{error}"
        finally:
            with self._state_lock:
                context.downstream_result_response_received = True
            context.result_event.set()

    def _forward_feedback(self, key: bytes, feedback) -> None:
        with self._state_lock:
            context = self._goals.get(key)
        if context is None or context.completed or not context.server_goal.is_active:
            return
        try:
            context.server_goal.publish_feedback(feedback.feedback)
        except Exception as error:
            self.get_logger().warn(f"Could not forward Nav2 feedback: {error}")

    def _cancel_downstream(
        self, context: ForwardedGoal, origin: str
    ) -> None:
        with self._state_lock:
            if context.cancel_origin is None:
                context.cancel_origin = origin
                context.cancel_requested_ros_time_ns = (
                    self.get_clock().now().nanoseconds
                )
            if context.cancel_sent or context.downstream_goal is None:
                return
            context.cancel_sent = True
            downstream_goal = context.downstream_goal
        try:
            future = downstream_goal.cancel_goal_async()
            future.add_done_callback(
                lambda result, goal_key=context.key: (
                    self._downstream_cancel_response(goal_key, result)
                )
            )
        except Exception as error:
            context.transport_error = f"cancel:{error}"
            context.result_event.set()

    def _downstream_cancel_response(self, key: bytes, future) -> None:
        """Record whether Nav2 actually transitioned the goal to canceling."""
        try:
            response = future.result()
        except Exception as error:
            with self._state_lock:
                context = self._goals.get(key)
                if context is not None:
                    context.cancel_response_received = True
                completed = context is None or context.completed
            if not completed:
                self.get_logger().warn(
                    f"Downstream cancellation response failed: {error}"
                )
            return
        return_code = int(response.return_code)
        goal_count = len(response.goals_canceling)
        with self._state_lock:
            context = self._goals.get(key)
            if context is None:
                return
            context.cancel_response_code = return_code
            context.cancel_response_goal_count = goal_count
            context.cancel_response_received = True
            decision_id = context.decision_id
            cancel_origin = context.cancel_origin
            completed = context.completed
        if completed:
            return
        accepted = (
            return_code == CancelGoal.Response.ERROR_NONE
            and goal_count > 0
        )
        if accepted:
            semantics = "accepted"
        elif return_code == CancelGoal.Response.ERROR_NONE:
            # rclpy can preserve ERROR_NONE after its user callback removes
            # every rejected goal from goals_canceling.  The empty list is the
            # operative evidence that no goal entered CANCELING.
            semantics = "rejected_empty_goal_list"
        else:
            semantics = {
                CancelGoal.Response.ERROR_REJECTED: "rejected",
                CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID: "unknown_goal_id",
                CancelGoal.Response.ERROR_GOAL_TERMINATED: "goal_terminated",
            }.get(return_code, "unknown")
        self._append_trace("downstream_cancel_response", {
            "decision_id": decision_id,
            "cancel_origin": cancel_origin,
            "return_code": return_code,
            "accepted": accepted,
            "return_code_semantics": semantics,
            "goals_canceling": goal_count,
        })

    def _record_execution(self, context: ForwardedGoal, succeeded: bool) -> None:
        execution_pose = self._lookup_pose(self.execution_frame)
        reached_pose = self._lookup_pose(self.map_frame)
        with self._state_lock:
            if execution_pose is not None:
                self._append_path_sample(context, execution_pose, reached_pose)
            translation = path_length(context.execution_path)
            self._completed_distance_m += translation
            context.distance_accounted = True
            self._execution_count += 1
            self._success_count += int(succeeded)
            termination = self._termination_reason
            topological_node_created = False
            if succeeded and reached_pose is not None:
                merge_distance = float(
                    self.get_parameter("topological_merge_distance_m").value
                )
                endpoint = reached_pose[:2]
                topological_node_created = all(
                    math.hypot(
                        endpoint[0] - node[0], endpoint[1] - node[1]
                    ) > merge_distance + 1e-12
                    for node in self._topological_nodes
                )
                if topological_node_created:
                    self._topological_nodes.append(endpoint)
        rotation = (
            shortest_rotation_deg(
                context.map_path[0][2], context.map_path[-1][2]
            )
            if len(context.map_path) >= 2 else 0.0
        )
        if context.transport_error:
            reason = context.transport_error
        elif context.timeout_requested:
            reason = "goal_timeout"
        elif termination is not None:
            reason = f"nav2_status_{context.downstream_status}:{termination}"
        elif context.superseded_by_decision_id is not None:
            reason = (
                f"nav2_status_{context.downstream_status}:"
                f"superseded_by_{context.superseded_by_decision_id}"
            )
        elif context.cancel_origin is not None:
            reason = (
                f"nav2_status_{context.downstream_status}:"
                f"{context.cancel_origin}"
            )
        else:
            reason = f"nav2_status_{context.downstream_status}"
        nav2_error_code = (
            None if context.downstream_result is None else
            int(getattr(context.downstream_result, "error_code", 0))
        )
        nav2_error_message = (
            "" if context.downstream_result is None else
            str(getattr(context.downstream_result, "error_msg", ""))
        )
        downstream_uuid = (
            None if context.downstream_goal is None else
            _uuid_hex(context.downstream_goal.goal_id)
        )
        cancel_origin = context.cancel_origin
        if cancel_origin is None and context.superseded_by_decision_id is not None:
            cancel_origin = "nav2_native_preemption"
        self._append_trace("execution", {
            "decision_id": context.decision_id,
            "succeeded": bool(succeeded),
            "topological_node_created": topological_node_created,
            "commanded_pose": context.target_pose,
            "reached_pose": reached_pose or context.target_pose,
            "translation_m": translation,
            "rotation_deg": rotation,
            "path": context.execution_path,
            "executed_path_frame": self.execution_frame,
            "reason": reason,
            "nav2_status": context.downstream_status,
            "nav2_error_code": nav2_error_code,
            "nav2_error_message": nav2_error_message,
            "upstream_goal_uuid": _uuid_hex(context.server_goal.goal_id),
            "downstream_goal_uuid": downstream_uuid,
            "cancel_origin": cancel_origin,
            "upstream_preemption": context.superseded_by_decision_id is not None,
        })

    def _publish_target_marker(
        self, pose: PoseStamped, decision_id: int
    ) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        target = Marker()
        target.header = pose.header
        target.ns = "external_frontier_target"
        target.id = decision_id
        target.type = Marker.ARROW
        target.action = Marker.ADD
        target.pose = pose.pose
        target.scale.x = 0.55
        target.scale.y = 0.10
        target.scale.z = 0.10
        target.color.r = 0.15
        target.color.g = 0.75
        target.color.b = 1.0
        target.color.a = 1.0
        markers.markers.append(target)
        self.marker_publisher.publish(markers)

    def _completion_callback(self, message: Empty) -> None:
        del message
        with self._state_lock:
            active = self._session_active and not self._finished
        if active:
            self._append_trace("upstream_completion_observed", {
                "semantics": "frontier_exhaustion_not_coverage_success",
                "upstream_component_id": UPSTREAM_COMPONENT_ID,
            })
            self._request_termination("frontier_exhausted")

    def _request_termination(self, reason: str) -> None:
        with self._state_lock:
            if self._finished or not self._session_active:
                return
            first_request = self._termination_reason is None
            if first_request:
                self._termination_reason = reason
            contexts = [
                context for context in self._goals.values()
                if not context.completed
            ]
        if first_request:
            if reason in {"action_budget", "distance_budget", "time_budget"}:
                self._append_trace("budget_cancel_requested", {"reason": reason})
            self._publish_status("STOPPING", reason)
        for context in contexts:
            self._cancel_downstream(context, "adapter_session_termination")
        self._request_upstream_stop()
        self._finish_session_if_quiescent()

    def _request_upstream_stop(self) -> None:
        with self._state_lock:
            if self._stop_request_sent or not self.control_client.service_is_ready():
                return
            self._stop_request_sent = True
        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_STOP
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        future = self.control_client.call_async(request)
        future.add_done_callback(self._upstream_stop_response)

    def _upstream_stop_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Upstream stop service failed: {error}")
        else:
            if response is not None and not response.accepted:
                self.get_logger().warn(
                    f"Upstream stop rejected: {response.message}"
                )
        finally:
            with self._state_lock:
                self._stop_response_received = True

    def _finish_session_if_quiescent(self) -> None:
        with self._state_lock:
            if (
                self._finished
                or self._termination_reason is None
                or self._pending_goal_acceptances > 0
                or any(not context.completed for context in self._goals.values())
            ):
                return
            reason = self._termination_reason
            self._finished = True
            self._session_active = False
            summary = {
                "method": self.method_id,
                "termination_reason": reason,
                "decision_count": self._decision_count,
                "execution_count": self._execution_count,
                "navigation_success_count": self._success_count,
                "navigation_failure_count": (
                    self._execution_count - self._success_count
                ),
                "total_distance_m": self._completed_distance_m,
                "nodes": [
                    {"id": index, "position": position}
                    for index, position in enumerate(self._topological_nodes)
                ],
            }
        if reason in {"action_budget", "distance_budget", "time_budget"}:
            self._append_trace("budget_reached", {"reason": reason})
        self._append_trace("session_finished", summary)
        terminal_state = (
            "COMPLETE" if reason == "frontier_exhausted" else
            "BUDGET_EXHAUSTED" if reason.endswith("_budget") else
            "STOPPED"
        )
        self._publish_status(terminal_state, reason)

    def _active_distance_m(self) -> float:
        with self._state_lock:
            unaccounted = sum(
                path_length(context.execution_path)
                for context in self._goals.values()
                if not context.distance_accounted
            )
            return self._completed_distance_m + unaccounted

    def _poll_nav2_lifecycle(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        with self._state_lock:
            future_pending = self._nav2_state_future is not None
            last_request_ns = self._nav2_state_last_request_ns
            cached_active = self._nav2_lifecycle_active
        if not self.nav2_lifecycle_client.service_is_ready():
            with self._state_lock:
                self._nav2_lifecycle_active = False
            return False
        request_due = (
            last_request_ns is None
            or now_ns < last_request_ns
            or now_ns - last_request_ns >= 1_000_000_000
        )
        if not future_pending and request_due:
            future = self.nav2_lifecycle_client.call_async(GetState.Request())
            with self._state_lock:
                self._nav2_state_future = future
                self._nav2_state_last_request_ns = now_ns
            future.add_done_callback(self._nav2_lifecycle_response)
        return cached_active

    def _nav2_lifecycle_response(self, future) -> None:
        active = False
        try:
            response = future.result()
            if response is not None:
                state = response.current_state
                active = int(state.id) == 3 and str(state.label).lower() == "active"
        except Exception as error:
            self.get_logger().debug(f"Nav2 lifecycle query failed: {error}")
        with self._state_lock:
            self._nav2_lifecycle_active = active
            self._nav2_state_future = None

    def _tick(self) -> None:
        with self._state_lock:
            enabled = self._enabled
            active = self._session_active
            finished = self._finished
            map_received = self._map_received
            started_ns = self._session_started_ns
            termination = self._termination_reason
            current = self._goals.get(self._current_goal_key)
        if finished:
            return
        if not active:
            nav2_lifecycle_active = self._poll_nav2_lifecycle()
            ready = (
                enabled
                and map_received
                and self.nav2_client.server_is_ready()
                and nav2_lifecycle_active
                and self.control_client.service_is_ready()
                and self._lookup_pose(self.map_frame) is not None
                and self._lookup_pose(self.execution_frame) is not None
            )
            if ready:
                self._request_upstream_start()
            else:
                self._publish_status("WAIT_STACK")
            return

        if current is not None and not current.completed:
            execution_pose = self._lookup_pose(self.execution_frame)
            map_pose = self._lookup_pose(self.map_frame)
            if execution_pose is not None:
                with self._state_lock:
                    self._append_path_sample(current, execution_pose, map_pose)

        if termination is not None:
            self._request_upstream_stop()
            self._finish_session_if_quiescent()
            return
        now_ns = self.get_clock().now().nanoseconds
        if (
            started_ns is not None
            and now_ns - started_ns >= int(self.max_duration_s * 1e9)
        ):
            self._request_termination("time_budget")
            return
        if self._active_distance_m() >= self.max_distance_m:
            self._request_termination("distance_budget")

    def destroy_node(self) -> None:
        self.action_server.destroy()
        self.nav2_client.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[FrontierActionAdapter] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = FrontierActionAdapter()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
