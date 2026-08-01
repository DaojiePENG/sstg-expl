"""ROS 2 state machine that executes incremental SSTG goals through Nav2."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Optional

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from sstg_explorer import (
    OnlineExplorerSession,
    SensorConfig,
    UnknownExplorerConfig,
)

from .conversions import (
    occupancy_grid_from_msg,
    pose2d_from_transform,
    target_pose_message,
)
from .readiness import LifecycleActiveGate, ReadinessResult


def _validated_policy_budget(
    values: dict[str, Any],
) -> dict[str, float | int]:
    """Reject disabled or malformed runtime limits before creating artifacts."""
    normalized: dict[str, float | int] = {}
    for field in ("max_duration_s", "max_distance_m", "goal_timeout_s"):
        value = values[field]
        if isinstance(value, bool):
            raise ValueError(f"{field} must be a finite positive number")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{field} must be a finite positive number"
            ) from error
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{field} must be a finite positive number")
        normalized[field] = number
    max_decisions = values["max_decisions"]
    if (
        isinstance(max_decisions, bool)
        or not isinstance(max_decisions, int)
        or max_decisions <= 0
    ):
        raise ValueError("max_decisions must be a positive integer")
    normalized["max_decisions"] = max_decisions
    if normalized["goal_timeout_s"] > normalized["max_duration_s"]:
        raise ValueError("goal_timeout_s must not exceed max_duration_s")
    return normalized


def _jsonable(value: Any) -> Any:
    """Convert trace content to strict JSON without hiding non-finite values."""
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "item"):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class SSTGPolicyNode(Node):
    """WAIT_MAP/TF -> PLAN -> NAVIGATE -> RECORD -> PLAN state machine."""

    def __init__(self) -> None:
        super().__init__("sstg_policy")
        self._declare_parameters()
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.execution_frame = str(
            self.get_parameter("execution_frame").value
        )
        self.navigate_action = str(self.get_parameter("navigate_action").value)
        navigate_lifecycle_node = str(
            self.get_parameter("navigate_lifecycle_node").value
        ).strip().rstrip("/")
        if not navigate_lifecycle_node:
            raise ValueError("navigate_lifecycle_node must be non-empty")
        self.navigate_lifecycle_node = navigate_lifecycle_node
        self.navigate_lifecycle_state_service = (
            f"{navigate_lifecycle_node}/get_state"
        )
        self.map_settle_s = float(self.get_parameter("map_settle_s").value)
        experiment_budget = _validated_policy_budget({
            name: self.get_parameter(name).value
            for name in (
                "max_duration_s",
                "max_distance_m",
                "max_decisions",
                "goal_timeout_s",
            )
        })
        self.goal_timeout_s = float(experiment_budget["goal_timeout_s"])
        self.max_distance_m = float(experiment_budget["max_distance_m"])
        self.max_duration_s = float(experiment_budget["max_duration_s"])
        self.max_decisions = int(experiment_budget["max_decisions"])
        self.output_dir = Path(str(self.get_parameter("output_dir").value))

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGridMsg, self.map_topic, self._map_callback, map_qos
        )
        trace_qos = QoSProfile(depth=200)
        trace_qos.reliability = ReliabilityPolicy.RELIABLE
        trace_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.trace_publisher = self.create_publisher(
            String, "/policy/decision_trace", trace_qos
        )
        self.status_publisher = self.create_publisher(
            String, "/policy/status", status_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/policy/candidates", 10
        )
        self.create_service(Trigger, "/policy/start", self._start_callback)
        self.create_service(Trigger, "/policy/stop", self._stop_callback)
        self.create_service(Trigger, "/policy/reset", self._reset_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self, NavigateToPose, self.navigate_action
        )
        self.nav_lifecycle_client = self.create_client(
            GetState, self.navigate_lifecycle_state_service
        )
        self.nav_lifecycle_gate = LifecycleActiveGate(
            self.nav_lifecycle_client, self.navigate_lifecycle_state_service
        )
        self.timer = self.create_timer(
            float(self.get_parameter("decision_period_s").value), self._tick
        )

        self.running = bool(self.get_parameter("auto_start").value)
        self.session: Optional[OnlineExplorerSession] = None
        self.latest_belief = None
        self.map_revision = 0
        self.last_map_time = None
        self.settle_until_ns = 0
        self.busy = False
        self.goal_handle = None
        self.goal_started = None
        self.session_started_ns = None
        self.termination_requested_reason = None
        self.execution_path = []
        self.expected_resolution = None
        self._prepare_output()
        self._publish_status("WAIT_MAP_TF")

    def _declare_parameters(self) -> None:
        defaults = {
            "auto_start": False,
            "strategy": "sstg",
            "coverage_objective": "joint",
            "map_topic": "/map",
            "map_frame": "map",
            "base_frame": "base_footprint",
            "execution_frame": "odom",
            "navigate_action": "/navigate_to_pose",
            "navigate_lifecycle_node": "/bt_navigator",
            "decision_period_s": 0.5,
            "map_settle_s": 1.0,
            "goal_timeout_s": 180.0,
            "topological_radius_m": 2.0,
            "topological_merge_distance_m": 0.25,
            "target_sensor_coverage": 0.95,
            "target_topological_coverage": 0.95,
            "lidar_fov_deg": 360.0,
            "lidar_range_m": 20.0,
            "lidar_angular_resolution_deg": 1.0,
            "robot_radius_m": 0.24,
            "safety_margin_m": 0.0,
            "preferred_clearance_m": 0.5,
            "target_spacing_m": 2.0,
            "information_gain_weight": 0.40,
            "topological_gain_weight": 0.60,
            "spacing_weight": 0.30,
            "min_gain_cells": 8,
            "min_topological_gain_cells": 8,
            "max_frontier_candidates": 48,
            "random_candidates": 24,
            "exact_gain_budget": 18,
            "multi_frontier": True,
            "use_topological_vantages": True,
            "require_known_footprint": True,
            "policy_seed": 42,
            "max_decisions": 100,
            "max_distance_m": 150.0,
            "max_duration_s": 900.0,
            "allow_existing_output": False,
            "output_dir": "system_sim_outputs/runs/development/manual",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _policy_config(self) -> UnknownExplorerConfig:
        return UnknownExplorerConfig(
            strategy=str(self.get_parameter("strategy").value),
            coverage_objective=str(
                self.get_parameter("coverage_objective").value
            ),
            sensor=SensorConfig(
                field_of_view_deg=float(
                    self.get_parameter("lidar_fov_deg").value
                ),
                max_range=float(self.get_parameter("lidar_range_m").value),
                angular_resolution_deg=float(
                    self.get_parameter("lidar_angular_resolution_deg").value
                ),
            ),
            topological_radius=float(
                self.get_parameter("topological_radius_m").value
            ),
            topological_merge_distance=float(
                self.get_parameter("topological_merge_distance_m").value
            ),
            target_coverage=float(
                self.get_parameter("target_sensor_coverage").value
            ),
            target_topological_coverage=float(
                self.get_parameter("target_topological_coverage").value
            ),
            information_gain_weight=float(
                self.get_parameter("information_gain_weight").value
            ),
            topological_gain_weight=float(
                self.get_parameter("topological_gain_weight").value
            ),
            robot_radius=float(self.get_parameter("robot_radius_m").value),
            safety_margin=float(
                self.get_parameter("safety_margin_m").value
            ),
            preferred_clearance=float(
                self.get_parameter("preferred_clearance_m").value
            ),
            target_spacing=float(
                self.get_parameter("target_spacing_m").value
            ),
            min_gain_cells=int(self.get_parameter("min_gain_cells").value),
            min_topological_gain_cells=int(
                self.get_parameter("min_topological_gain_cells").value
            ),
            max_frontier_candidates=int(
                self.get_parameter("max_frontier_candidates").value
            ),
            random_candidates=int(
                self.get_parameter("random_candidates").value
            ),
            exact_gain_budget=int(
                self.get_parameter("exact_gain_budget").value
            ),
            spacing_weight=float(
                self.get_parameter("spacing_weight").value
            ),
            multi_frontier=bool(
                self.get_parameter("multi_frontier").value
            ),
            use_topological_vantages=bool(
                self.get_parameter("use_topological_vantages").value
            ),
            require_known_footprint=bool(
                self.get_parameter("require_known_footprint").value
            ),
            seed=int(self.get_parameter("policy_seed").value),
            max_decisions=self.max_decisions,
            verbose=False,
        )

    def _prepare_output(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.trace_path = self.output_dir / "policy_trace.jsonl"
        manifest_path = self.output_dir / "policy_manifest.json"
        if not bool(self.get_parameter("allow_existing_output").value):
            existing = [
                path for path in (self.trace_path, manifest_path)
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
            "map_topic": self.map_topic,
            "map_frame": self.map_frame,
            "base_frame": self.base_frame,
            "execution_frame": self.execution_frame,
            "navigate_action": self.navigate_action,
            "runtime_adapter": "sstg_policy",
            "parameters": {
                name: self.get_parameter(name).value
                for name in self._parameters
            },
        }
        manifest_path.write_text(
            json.dumps(_jsonable(manifest), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _append_trace(self, event: str, payload: Any) -> None:
        record = {
            "event": event,
            "ros_time_ns": self.get_clock().now().nanoseconds,
            "map_revision": self.map_revision,
            "payload": _jsonable(payload),
        }
        encoded = json.dumps(record, sort_keys=True, allow_nan=False)
        with self.trace_path.open("a", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
        message = String()
        message.data = encoded
        self.trace_publisher.publish(message)

    def _publish_status(self, state: str, detail: str = "") -> None:
        message = String()
        message.data = json.dumps({"state": state, "detail": detail})
        self.status_publisher.publish(message)

    def _map_callback(self, message: OccupancyGridMsg) -> None:
        try:
            self.latest_belief = occupancy_grid_from_msg(
                message, expected_resolution=self.expected_resolution
            )
            if self.expected_resolution is None:
                self.expected_resolution = self.latest_belief.resolution
        except ValueError as error:
            self.get_logger().error(f"Rejected /map revision: {error}")
            self._append_trace("map_rejected", {"reason": str(error)})
            return
        self.map_revision += 1
        self.last_map_time = self.get_clock().now()

    def _lookup_pose(self, target_frame: Optional[str] = None):
        target_frame = target_frame or self.map_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            self.get_logger().debug(f"Waiting for TF: {error}")
            return None
        return pose2d_from_transform(transform.transform)

    def _nav2_ready_for_dispatch(self) -> ReadinessResult:
        """Require both action discovery and a fresh ACTIVE lifecycle state."""
        if not self.nav_client.server_is_ready():
            self.nav_lifecycle_gate.reset()
            return ReadinessResult(
                False,
                f"NavigateToPose action {self.navigate_action!r} is unavailable",
            )
        return self.nav_lifecycle_gate.poll()

    def _start_callback(self, request, response):
        del request
        self.running = True
        response.success = True
        response.message = "policy execution enabled"
        self._publish_status("START_REQUESTED")
        return response

    def _stop_callback(self, request, response):
        del request
        self.running = False
        self.nav_lifecycle_gate.reset()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        response.success = True
        response.message = "policy stopped; active goal cancellation requested"
        self._publish_status("STOP_REQUESTED")
        return response

    def _reset_callback(self, request, response):
        del request
        if self.busy:
            response.success = False
            response.message = "stop and wait for the active goal before reset"
            return response
        self.nav_lifecycle_gate.reset()
        self._append_trace("session_reset", {
            "previous_summary": self.session.summary() if self.session else None,
        })
        self.session = None
        self.session_started_ns = None
        self.termination_requested_reason = None
        self.expected_resolution = None
        response.success = True
        response.message = "policy session reset"
        self._publish_status("RESET")
        return response

    def _tick(self) -> None:
        if not self.running or self.latest_belief is None:
            return
        pose = self._lookup_pose()
        if pose is None:
            self._publish_status("WAIT_MAP_TF")
            return
        if self.busy:
            execution_pose = self._lookup_pose(self.execution_frame)
            if (
                execution_pose is not None
                and (
                    not self.execution_path
                    or self.execution_path[-1] != execution_pose[:2]
                )
            ):
                self.execution_path.append(execution_pose[:2])
            budget_reason = self._budget_reason(include_active_path=True)
            if (
                budget_reason is not None
                and self.termination_requested_reason is None
                and self.goal_handle is not None
            ):
                self.termination_requested_reason = budget_reason
                self._append_trace("budget_cancel_requested", {
                    "reason": budget_reason,
                    "pose": pose,
                })
                self.goal_handle.cancel_goal_async()
            if (
                self.goal_started is not None
                and (self.get_clock().now() - self.goal_started).nanoseconds
                > self.goal_timeout_s * 1e9
                and self.goal_handle is not None
            ):
                self._append_trace("goal_timeout", {"pose": pose})
                self.goal_handle.cancel_goal_async()
                self.goal_started = None
            return
        if self.get_clock().now().nanoseconds < self.settle_until_ns:
            self._publish_status("MAP_SETTLE")
            return
        nav2_readiness = self._nav2_ready_for_dispatch()
        if not nav2_readiness.ready:
            self._publish_status("WAIT_NAV2", nav2_readiness.detail)
            return
        if self.session is None:
            self.session = OnlineExplorerSession(self._policy_config(), pose)
            self.session_started_ns = self.get_clock().now().nanoseconds
            self._append_trace("session_started", self.session.summary())
        budget_reason = self._budget_reason()
        if budget_reason is not None:
            self.running = False
            self._publish_status("BUDGET_EXHAUSTED", budget_reason)
            self._append_trace("budget_reached", {"reason": budget_reason})
            self._append_trace("session_finished", self.session.summary())
            return

        try:
            decision = self.session.propose(
                self.latest_belief, pose, map_revision=self.map_revision
            )
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(f"Policy decision failed: {error}")
            self._append_trace("decision_error", {"reason": str(error)})
            self.running = False
            self._publish_status("ERROR", str(error))
            return
        self._append_trace("decision", decision.to_dict())
        self._publish_markers(decision)
        if decision.status != "navigate":
            self.running = False
            self._publish_status(decision.status.upper(), decision.reason)
            self._append_trace("session_finished", self.session.summary())
            return
        goal = NavigateToPose.Goal()
        goal.pose = target_pose_message(
            *decision.target_pose,
            frame_id=self.map_frame,
            stamp=self.get_clock().now().to_msg(),
        )
        self.busy = True
        execution_pose = self._lookup_pose(self.execution_frame)
        self.execution_path = (
            [execution_pose[:2]] if execution_pose is not None else []
        )
        self.goal_started = self.get_clock().now()
        self._publish_status("NAVIGATING", str(decision.decision_id))
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    @staticmethod
    def _path_length(points) -> float:
        return float(sum(
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(points[:-1], points[1:])
        ))

    def _budget_reason(self, include_active_path: bool = False):
        if self.session is None:
            return None
        if len(self.session.execution_records) >= self.max_decisions:
            return "action_budget"
        distance = self.session.total_distance_m
        if include_active_path:
            distance += self._path_length(self.execution_path)
        if self.max_distance_m > 0.0 and distance >= self.max_distance_m:
            return "distance_budget"
        if self.max_duration_s > 0.0 and self.session_started_ns is not None:
            elapsed_ns = self.get_clock().now().nanoseconds - self.session_started_ns
            if elapsed_ns >= int(self.max_duration_s * 1e9):
                return "time_budget"
        return None

    def _goal_response_callback(self, future) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as error:  # rclpy action transport failure
            self._finish_navigation(False, f"goal_transport_error:{error}")
            return
        if not self.goal_handle.accepted:
            self.nav_lifecycle_gate.reset()
            self._finish_navigation(False, "goal_rejected")
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        try:
            wrapped = future.result()
            succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED
            reason = f"nav2_status_{wrapped.status}"
        except Exception as error:  # rclpy action transport failure
            succeeded = False
            reason = f"result_transport_error:{error}"
        self._finish_navigation(succeeded, reason)

    def _finish_navigation(self, succeeded: bool, reason: str) -> None:
        pose = self._lookup_pose()
        execution_pose = self._lookup_pose(self.execution_frame)
        if (
            execution_pose is not None
            and (
                not self.execution_path
                or self.execution_path[-1] != execution_pose[:2]
            )
        ):
            self.execution_path.append(execution_pose[:2])
        if pose is None and self.session is not None:
            pose = self.session.current_pose
        if self.session is None or self.session.pending_decision is None:
            self._append_trace("orphan_navigation_result", {"reason": reason})
        else:
            decision_id = self.session.pending_decision.decision_id
            record = self.session.record_execution(
                decision_id,
                succeeded,
                pose,
                executed_path=self.execution_path,
                executed_path_frame=self.execution_frame,
                reason=reason,
            )
            self._append_trace("execution", record.to_dict())
        self.busy = False
        self.goal_handle = None
        self.goal_started = None
        self.execution_path = []
        self.last_map_time = self.get_clock().now()
        self.settle_until_ns = (
            self.last_map_time.nanoseconds + int(self.map_settle_s * 1e9)
        )
        if self.termination_requested_reason is not None:
            budget_reason = self.termination_requested_reason
            self.termination_requested_reason = None
            self.running = False
            self._append_trace("budget_reached", {"reason": budget_reason})
            self._append_trace("session_finished", self.session.summary())
            self._publish_status("BUDGET_EXHAUSTED", budget_reason)
        else:
            self._publish_status("MAP_SETTLE", reason)

    def _publish_markers(self, decision) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()

        for index, candidate in enumerate(decision.active_candidates):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = "active_candidates"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(candidate["target"][0])
            marker.pose.position.y = float(candidate["target"][1])
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.a = 0.65
            markers.markers.append(marker)

        if decision.target_pose is not None:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = "selected_goal"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = target_pose_message(
                *decision.target_pose, self.map_frame, stamp
            ).pose
            marker.scale.x = 0.55
            marker.scale.y = 0.10
            marker.scale.z = 0.10
            marker.color.g = 1.0
            marker.color.a = 1.0
            markers.markers.append(marker)

        if self.session is not None:
            for index, node in enumerate(self.session.nodes):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = stamp
                marker.ns = "topological_nodes"
                marker.id = index
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = float(node["position"][0])
                marker.pose.position.y = float(node["position"][1])
                marker.pose.position.z = 0.04
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = 0.18
                marker.scale.z = 0.08
                marker.color.b = 1.0
                marker.color.a = 0.9
                markers.markers.append(marker)
        self.marker_publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SSTGPolicyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Jazzy can surface RCLError from spin after its signal handler has
        # already invalidated the context.  Preserve failures while ROS is live.
        if rclpy.ok():
            raise
    finally:
        try:
            if node is not None:
                try:
                    node.destroy_node()
                except RuntimeError:
                    if rclpy.ok():
                        raise
        finally:
            # ROS signal handling may already have shut down the default
            # context.  try_shutdown is idempotent in that case.
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
