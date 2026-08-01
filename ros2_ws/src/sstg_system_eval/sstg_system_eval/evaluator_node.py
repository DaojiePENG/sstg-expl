"""ROS 2 evaluator process with exclusive access to simulation truth."""
from __future__ import annotations

import hashlib
import json
import math
from collections import deque
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts, WorldStatistics
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .artifacts import prepare_output_directory
from .metrics import (
    ActionTraceAccumulator,
    BeliefGrid,
    CameraGeometry,
    CollisionAccumulator,
    GroundTruthMotionAccumulator,
    TargetRecallAccumulator,
    TruthClearanceAccumulator,
    TopologicalNodeAccumulator,
    TrajectoryAccumulator,
    WorldStatisticsAccumulator,
    compute_geometric_metrics,
    compute_topological_metrics,
    load_target_registry,
    load_truth_map,
    transform_planar_point,
    transform_truth_grid,
)


def _strict_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":"))


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("pose has a zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class SystemEvaluatorNode(Node):
    """Score `/map`, sample TF and audit public policy trace events."""

    def __init__(self) -> None:
        super().__init__("sstg_system_eval")
        self._declare_parameters()
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.trace_topic = str(self.get_parameter("trace_topic").value)
        self.metrics_topic = str(self.get_parameter("metrics_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.ground_truth_odom_topic = str(
            self.get_parameter("ground_truth_odom_topic").value
        )
        self.contacts_topic = str(self.get_parameter("contacts_topic").value)
        self.world_stats_topic = str(
            self.get_parameter("world_stats_topic").value
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.path_frame = str(self.get_parameter("path_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ground_truth_frame = str(
            self.get_parameter("ground_truth_frame").value
        )
        self.ground_truth_child_frame = str(
            self.get_parameter("ground_truth_child_frame").value
        )
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        allow_existing_output = self.get_parameter(
            "allow_existing_output"
        ).value
        if not isinstance(allow_existing_output, bool):
            raise ValueError("allow_existing_output must be boolean")
        self.allow_existing_output = allow_existing_output
        self.known_free_threshold = int(
            self.get_parameter("known_free_threshold").value
        )
        self.publish_period_ns = int(
            float(self.get_parameter("metrics_publish_period_s").value) * 1e9
        )
        self._validate_boundary()

        truth_path = str(self.get_parameter("truth_map_yaml").value).strip()
        if not truth_path:
            raise ValueError("truth_map_yaml is required for the evaluator")
        self.truth = load_truth_map(truth_path)
        self.registration_id = str(
            self.get_parameter("truth_registration_id").value
        ).strip()
        if not self.registration_id:
            raise ValueError(
                "truth_registration_id must document the truth-to-map transform"
            )
        self.truth_to_map = (
            float(self.get_parameter("truth_to_map_x_m").value),
            float(self.get_parameter("truth_to_map_y_m").value),
            float(self.get_parameter("truth_to_map_yaw_rad").value),
        )
        self.registered_truth = transform_truth_grid(
            self.truth,
            self.truth_to_map[:2],
            self.truth_to_map[2],
        )
        self.trajectory = TrajectoryAccumulator(
            float(self.get_parameter("tf_minimum_step_m").value)
        )
        self.ground_truth_minimum_step_m = float(
            self.get_parameter("ground_truth_minimum_step_m").value
        )
        self.ground_truth_motion = GroundTruthMotionAccumulator(
            self.ground_truth_minimum_step_m
        )
        self.clearance = TruthClearanceAccumulator(
            self.truth,
            float(self.get_parameter("robot_clearance_radius_m").value),
        )
        self.world_statistics = WorldStatisticsAccumulator()
        self.collisions = CollisionAccumulator(
            self.get_parameter("robot_collision_name_tokens").value,
            self.get_parameter("ground_collision_name_tokens").value,
            float(
                self.get_parameter("collision_event_separation_s").value
            ),
            float(self.get_parameter("collision_minimum_depth_m").value),
        )
        targets_path = str(self.get_parameter("targets_yaml").value).strip()
        if not targets_path:
            targets_path = str(
                Path(truth_path).expanduser().resolve().parent.parent
                / "targets.yaml"
            )
        (
            self.target_world_id,
            target_specs,
            self.targets_sha256,
        ) = load_target_registry(targets_path)
        self.targets_yaml = str(Path(targets_path).expanduser().resolve())
        self.camera_geometry = CameraGeometry(
            x_offset_m=float(
                self.get_parameter("camera_x_offset_m").value
            ),
            y_offset_m=float(
                self.get_parameter("camera_y_offset_m").value
            ),
            height_m=float(self.get_parameter("camera_height_m").value),
            yaw_offset_rad=float(
                self.get_parameter("camera_yaw_offset_rad").value
            ),
            pitch_rad=float(self.get_parameter("camera_pitch_rad").value),
            horizontal_fov_rad=float(
                self.get_parameter("camera_horizontal_fov_rad").value
            ),
            vertical_fov_rad=float(
                self.get_parameter("camera_vertical_fov_rad").value
            ),
            minimum_range_m=float(
                self.get_parameter("camera_minimum_range_m").value
            ),
            maximum_range_m=float(
                self.get_parameter("camera_maximum_range_m").value
            ),
            maximum_incidence_rad=float(
                self.get_parameter("target_maximum_incidence_rad").value
            ),
            los_endpoint_clearance_m=float(
                self.get_parameter("target_los_endpoint_clearance_m").value
            ),
        )
        self.target_recall = TargetRecallAccumulator(
            self.truth, target_specs, self.camera_geometry
        )
        self.actions = ActionTraceAccumulator()
        self.topological_nodes = TopologicalNodeAccumulator(
            float(
                self.get_parameter(
                    "topological_node_dedup_tolerance_m"
                ).value
            )
        )
        self.topological_radius_m = float(
            self.get_parameter("topological_radius_m").value
        )
        self.information_coverage_target = float(
            self.get_parameter("information_coverage_target").value
        )
        self.topological_coverage_target = float(
            self.get_parameter("topological_coverage_target").value
        )
        self.latest_geometric: Optional[dict] = None
        self.map_revision = 0
        self.map_rejection_count = 0
        self.trace_rejection_count = 0
        self.topology_trace_rejection_count = 0
        self.last_map_stamp_ns: Optional[int] = None
        self.last_emit_ns: Optional[int] = None
        self.tf_wait_count = 0
        self.ground_truth_rejection_count = 0
        self.contact_rejection_count = 0
        self.world_stats_rejection_count = 0
        self.ate_tf_wait_count = 0
        self.ate_tf_drop_count = 0
        self._last_ground_truth_stamp_ns: Optional[int] = None
        self._pending_ate = deque()
        self.target_session_active = False
        self.ate_pairing_delay_ns = int(
            float(self.get_parameter("ate_pairing_delay_s").value) * 1e9
        )
        self.ate_tf_expiration_ns = int(
            float(self.get_parameter("ate_tf_expiration_s").value) * 1e9
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.metrics_publisher = self.create_publisher(
            String, self.metrics_topic, 20
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, status_qos
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos
        )
        trace_qos = QoSProfile(depth=200)
        trace_qos.reliability = ReliabilityPolicy.RELIABLE
        trace_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, self.trace_topic, self._trace_callback, trace_qos
        )
        sensor_qos = QoSProfile(depth=50)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry,
            self.ground_truth_odom_topic,
            self._ground_truth_callback,
            sensor_qos,
        )
        self.create_subscription(
            Contacts, self.contacts_topic, self._contacts_callback, sensor_qos
        )
        self.create_subscription(
            WorldStatistics,
            self.world_stats_topic,
            self._world_stats_callback,
            sensor_qos,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(
            float(self.get_parameter("tf_sample_period_s").value),
            self._sample_tf,
        )
        self._prepare_output()
        self._append_event("evaluator_started", self._manifest())
        self._publish_status("WAIT_MAP_TF")

    def _declare_parameters(self) -> None:
        defaults = {
            "truth_map_yaml": "",
            "truth_registration_id": "",
            "truth_to_map_x_m": 0.0,
            "truth_to_map_y_m": 0.0,
            "truth_to_map_yaw_rad": 0.0,
            "map_topic": "/map",
            "trace_topic": "/policy/decision_trace",
            "metrics_topic": "/evaluation/metrics",
            "status_topic": "/evaluation/status",
            "ground_truth_odom_topic": "/evaluation/ground_truth_odom",
            "contacts_topic": "/evaluation/contacts",
            "world_stats_topic": "/evaluation/world_stats",
            "map_frame": "map",
            "path_frame": "odom",
            "base_frame": "base_footprint",
            "ground_truth_frame": "world",
            "ground_truth_child_frame": "base_footprint_truth",
            "known_free_threshold": 50,
            "tf_sample_period_s": 0.1,
            "tf_minimum_step_m": 0.002,
            "ground_truth_minimum_step_m": 0.002,
            "robot_clearance_radius_m": 0.24,
            "ate_pairing_delay_s": 0.15,
            "ate_tf_expiration_s": 2.0,
            "robot_collision_name_tokens": [
                "sstg_tb3_evaluation_overlay",
                "footprint_probe",
                "turtlebot3_waffle",
                "base_collision",
                "wheel_left",
                "wheel_right",
                "caster_back",
            ],
            "ground_collision_name_tokens": ["floor", "ground_plane"],
            "collision_event_separation_s": 1.0,
            "collision_minimum_depth_m": 0.0,
            "targets_yaml": "",
            "camera_x_offset_m": 0.133,
            "camera_y_offset_m": -0.094,
            "camera_height_m": 0.214,
            "camera_yaw_offset_rad": 0.0,
            "camera_pitch_rad": 0.0,
            "camera_horizontal_fov_rad": 1.047,
            "camera_vertical_fov_rad": 0.8171093547878163,
            "camera_minimum_range_m": 0.001,
            "camera_maximum_range_m": 5.0,
            "target_maximum_incidence_rad": 1.3962634016,
            "target_los_endpoint_clearance_m": 0.10,
            "topological_radius_m": 2.0,
            "topological_node_dedup_tolerance_m": 0.01,
            "information_coverage_target": 0.95,
            "topological_coverage_target": 0.95,
            "metrics_publish_period_s": 1.0,
            "output_dir": "system_sim_outputs/runs/development/manual",
            "allow_existing_output": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _validate_boundary(self) -> None:
        if not self.metrics_topic.startswith("/evaluation/"):
            raise ValueError("metrics_topic must remain under /evaluation/")
        if not self.status_topic.startswith("/evaluation/"):
            raise ValueError("status_topic must remain under /evaluation/")
        if not self.ground_truth_odom_topic.startswith("/evaluation/"):
            raise ValueError(
                "ground_truth_odom_topic must remain under /evaluation/"
            )
        if not self.contacts_topic.startswith("/evaluation/"):
            raise ValueError("contacts_topic must remain under /evaluation/")
        if not self.world_stats_topic.startswith("/evaluation/"):
            raise ValueError("world_stats_topic must remain under /evaluation/")
        if not self.trace_topic.startswith("/policy/"):
            raise ValueError("trace_topic must remain under /policy/")
        if self.map_topic != "/map":
            raise ValueError("the current audit contract requires map_topic=/map")
        if not all((
            self.map_frame,
            self.path_frame,
            self.base_frame,
            self.ground_truth_frame,
            self.ground_truth_child_frame,
        )):
            raise ValueError("all evaluator frame parameters must be non-empty")
        if not 1 <= self.known_free_threshold <= 100:
            raise ValueError("known_free_threshold must be in [1, 100]")
        tf_period = float(self.get_parameter("tf_sample_period_s").value)
        publish_period = float(
            self.get_parameter("metrics_publish_period_s").value
        )
        if not math.isfinite(tf_period) or tf_period <= 0.0:
            raise ValueError("tf_sample_period_s must be positive and finite")
        if not math.isfinite(publish_period) or publish_period <= 0.0:
            raise ValueError(
                "metrics_publish_period_s must be positive and finite"
            )
        for name in (
            "ate_pairing_delay_s",
            "ate_tf_expiration_s",
            "ground_truth_minimum_step_m",
            "robot_clearance_radius_m",
            "collision_minimum_depth_m",
            "target_los_endpoint_clearance_m",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be non-negative and finite")
        if float(self.get_parameter("robot_clearance_radius_m").value) <= 0.0:
            raise ValueError("robot_clearance_radius_m must be positive")
        if float(self.get_parameter("ate_tf_expiration_s").value) <= float(
            self.get_parameter("ate_pairing_delay_s").value
        ):
            raise ValueError(
                "ate_tf_expiration_s must exceed ate_pairing_delay_s"
            )
        collision_gap = float(
            self.get_parameter("collision_event_separation_s").value
        )
        if not math.isfinite(collision_gap) or collision_gap <= 0.0:
            raise ValueError(
                "collision_event_separation_s must be positive and finite"
            )
        for name in (
            "information_coverage_target",
            "topological_coverage_target",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f"{name} must be in (0, 1]")
        radius = float(self.get_parameter("topological_radius_m").value)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("topological_radius_m must be positive and finite")

    def _prepare_output(self) -> None:
        owned_artifact_names = (
            "evaluation_metrics.jsonl",
            "evaluation_observed_policy_trace.jsonl",
            "evaluation_manifest.json",
        )
        self.output_dir = prepare_output_directory(
            self.output_dir,
            self.allow_existing_output,
            owned_artifact_names,
        )
        self.metrics_path = self.output_dir / "evaluation_metrics.jsonl"
        self.observed_trace_path = (
            self.output_dir / "evaluation_observed_policy_trace.jsonl"
        )
        self.manifest_path = self.output_dir / "evaluation_manifest.json"
        mode = "w" if self.allow_existing_output else "x"
        with self.manifest_path.open(mode, encoding="utf-8") as stream:
            stream.write(
                json.dumps(self._manifest(), indent=2, sort_keys=True) + "\n"
            )

    def _manifest(self) -> dict:
        return {
            "schema": "sstg_system_sim_evaluator_manifest/v2",
            "node": self.get_fully_qualified_name(),
            "evidence_source": "system_simulation",
            "truth_access": "evaluator_only",
            "truth_map_yaml": self.truth.source_yaml,
            "truth_map_bundle_sha256": self.truth.source_sha256,
            "truth_map_shape": list(self.truth.shape),
            "truth_map_resolution_m": self.truth.resolution,
            "truth_registration": {
                "registration_id": self.registration_id,
                "semantics": "T_map_truth",
                "x_m": self.truth_to_map[0],
                "y_m": self.truth_to_map[1],
                "yaw_rad": self.truth_to_map[2],
            },
            "subscribed_topics": [
                self.map_topic,
                self.trace_topic,
                self.ground_truth_odom_topic,
                self.contacts_topic,
                self.world_stats_topic,
                "/tf",
                "/tf_static",
            ],
            "published_topics": [self.metrics_topic, self.status_topic],
            "policy_topics_published": [],
            "primary_travel_source": self.ground_truth_odom_topic,
            "primary_travel_interval": (
                "accepted policy session_started through session_finished"
            ),
            "estimated_odometry_diagnostic_source": (
                f"tf:{self.path_frame}->{self.base_frame}"
            ),
            "planar_ate": {
                "truth_source": self.ground_truth_odom_topic,
                "truth_alignment": "T_map_truth",
                "estimate_source": f"tf:{self.map_frame}->{self.base_frame}",
                "pairing": (
                    "ground-truth stamp with delayed exact-time TF lookup"
                ),
            },
            "coverage_endpoints": {
                "information": "geometric_coverage",
                "topological": "truth-free cells within topological_radius_m",
                "joint": "min(information_coverage, topological_coverage)",
                "success": "both frozen thresholds met",
            },
            "target_recall": {
                "status": "implemented_geometry_proxy",
                "primary_metric": "targets.target_recall",
                "targets_yaml": self.targets_yaml,
                "targets_sha256": self.targets_sha256,
                "target_world_id": self.target_world_id,
                "coordinate_frame": self.ground_truth_frame,
                "model": "deterministic_geometry_proxy_v1",
                "image_detector": False,
                "time_origin": "accepted policy session_started ros_time_ns",
                "active_interval": "session_started through session_finished",
            },
            "safety_metric": {
                "source": self.contacts_topic,
                "semantics": (
                    "debounced attributed robot/non-ground collision-pair "
                    "onsets"
                ),
                "floor_contacts_excluded": True,
                "name_filtering_is_runtime_audited": True,
                "collision_free_scope": (
                    "configured_contact_sensor_collisions; null when "
                    "attribution or timestamp ordering is incomplete"
                ),
            },
            "static_clearance_metric": {
                "truth_source": self.truth.source_yaml,
                "pose_source": self.ground_truth_odom_topic,
                "raw_metric": "raw_static_obstacle_distance_*_m",
                "footprint_metric": "footprint_clearance_*_m",
                "robot_clearance_radius_m": self.clearance.robot_radius_m,
                "robot_radius_semantics": (
                    "frozen shared-stack conservative clearance-radius "
                    "parameter including navigation footprint padding"
                ),
            },
            "simulation_clock_diagnostics": {
                "source": self.world_stats_topic,
                "message_type": "ros_gz_interfaces/msg/WorldStatistics",
                "primary_fields": [
                    "sim_time_latest_ns",
                    "reported_real_time_factor_latest",
                    "paused_latest",
                    "iterations_latest",
                ],
            },
            "artifact_reuse": {
                "allow_existing_output": self.allow_existing_output,
                "default": "fail_closed",
            },
            "output_files": {
                "metrics_jsonl": "evaluation_metrics.jsonl",
                "observed_policy_trace_jsonl": (
                    "evaluation_observed_policy_trace.jsonl"
                ),
            },
            "parameters": {
                name: self.get_parameter(name).value for name in self._parameters
            },
        }

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _append_event(self, event: str, payload: Any) -> None:
        record = {
            "schema": "sstg_system_sim_evaluator_event/v1",
            "event": event,
            "ros_time_ns": self._now_ns(),
            "map_revision": self.map_revision,
            "payload": payload,
        }
        with self.metrics_path.open("a", encoding="utf-8") as stream:
            stream.write(_strict_json(record) + "\n")

    def _publish_status(self, state: str, detail: str = "") -> None:
        message = String()
        message.data = _strict_json({
            "schema": "sstg_system_sim_evaluator_status/v1",
            "state": state,
            "detail": detail,
            "map_revision": self.map_revision,
            "map_rejection_count": self.map_rejection_count,
            "trace_rejection_count": self.trace_rejection_count,
            "topology_trace_rejection_count": (
                self.topology_trace_rejection_count
            ),
            "tf_sample_count": self.trajectory.sample_count,
            "ground_truth_sample_count": (
                self.ground_truth_motion.path.sample_count
            ),
            "ground_truth_rejection_count": (
                self.ground_truth_rejection_count
            ),
            "contact_message_count": self.collisions.message_count,
            "collision_count": self.collisions.collision_event_count,
            "world_stats_sample_count": self.world_statistics.sample_count,
            "world_stats_rejection_count": self.world_stats_rejection_count,
        })
        self.status_publisher.publish(message)

    def _snapshot(self, reason: str) -> dict:
        information_coverage = (
            None
            if self.latest_geometric is None
            else self.latest_geometric["geometric_coverage"]
        )
        topological = compute_topological_metrics(
            self.registered_truth,
            self.topological_nodes.positions,
            self.topological_radius_m,
            information_coverage=information_coverage,
            information_target=self.information_coverage_target,
            topological_target=self.topological_coverage_target,
        )
        topological["node_audit"] = self.topological_nodes.snapshot()
        target_metrics = self.target_recall.snapshot()
        target_metrics["policy_session_active"] = self.target_session_active
        if target_metrics["time_origin_ros_time_ns"] is None:
            target_metrics["status"] = "waiting_for_policy_session"
        ground_truth_metrics = self.ground_truth_motion.snapshot()
        ground_truth_sample_count = ground_truth_metrics[
            "ground_truth_sample_count"
        ]
        ground_truth_metrics.update({
            "policy_session_active": self.target_session_active,
            "ate_pending_sample_count": len(self._pending_ate),
            "ate_dropped_sample_count": self.ate_tf_drop_count,
            "ate_pairing_fraction_of_ground_truth_samples": (
                None
                if ground_truth_sample_count == 0
                else ground_truth_metrics["ate_sample_count"]
                / ground_truth_sample_count
            ),
        })
        return {
            "schema": "sstg_system_sim_evaluator_snapshot/v2",
            "ros_time_ns": self._now_ns(),
            "reason": reason,
            "map_revision": self.map_revision,
            "last_map_stamp_ns": self.last_map_stamp_ns,
            "truth_map_bundle_sha256": self.truth.source_sha256,
            "geometric": self.latest_geometric,
            "topological": topological,
            "coverage_endpoints": {
                "c_i_information": topological["information_coverage"],
                "c_t_topological": topological["topological_coverage"],
                "joint_min": topological["joint_coverage"],
                "dual_threshold_success": topological[
                    "dual_threshold_success"
                ],
            },
            "trajectory": {
                "role": "estimated_odometry_diagnostic",
                "path_source": f"tf:{self.path_frame}->{self.base_frame}",
                **self.trajectory.snapshot(),
            },
            "actions": self.actions.snapshot(),
            "diagnostics": {
                "map_rejection_count": self.map_rejection_count,
                "trace_rejection_count": self.trace_rejection_count,
                "topology_trace_rejection_count": (
                    self.topology_trace_rejection_count
                ),
                "tf_wait_count": self.tf_wait_count,
                "ground_truth_rejection_count": (
                    self.ground_truth_rejection_count
                ),
                "contact_rejection_count": self.contact_rejection_count,
                "world_stats_rejection_count": (
                    self.world_stats_rejection_count
                ),
                "ate_tf_wait_count": self.ate_tf_wait_count,
                "ate_tf_drop_count": self.ate_tf_drop_count,
                "ate_pending_sample_count": len(self._pending_ate),
            },
            "targets": target_metrics,
            "safety": self.collisions.snapshot(),
            "static_clearance": self.clearance.snapshot(),
            "simulation_clock": self.world_statistics.snapshot(),
            "ground_truth_motion": {
                "primary_travel_metric": "ground_truth_path_length_m",
                "source": self.ground_truth_odom_topic,
                "source_frame": self.ground_truth_frame,
                "ate_truth_alignment": "T_map_truth",
                "ate_estimate_source": (
                    f"tf:{self.map_frame}->{self.base_frame}"
                ),
                **ground_truth_metrics,
            },
        }

    def _emit_snapshot(self, reason: str, publish: bool = True) -> None:
        snapshot = self._snapshot(reason)
        self._append_event("metrics_snapshot", snapshot)
        if not publish or not rclpy.ok(context=self.context):
            self.last_emit_ns = snapshot["ros_time_ns"]
            return
        message = String()
        message.data = _strict_json(snapshot)
        self.metrics_publisher.publish(message)
        self.last_emit_ns = snapshot["ros_time_ns"]
        state = "RUNNING" if self.latest_geometric is not None else "WAIT_MAP"
        self._publish_status(state, reason)

    def _belief_from_message(self, message: OccupancyGrid) -> BeliefGrid:
        width = int(message.info.width)
        height = int(message.info.height)
        if width <= 0 or height <= 0:
            raise ValueError("occupancy-grid dimensions must be positive")
        if len(message.data) != width * height:
            raise ValueError(
                f"occupancy-grid has {len(message.data)} cells; "
                f"expected {width * height}"
            )
        orientation = message.info.origin.orientation
        yaw = _quaternion_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        return BeliefGrid(
            data=np.asarray(message.data, dtype=np.int16).reshape((height, width)),
            resolution=float(message.info.resolution),
            origin=(
                float(message.info.origin.position.x),
                float(message.info.origin.position.y),
            ),
            origin_yaw=yaw,
        )

    def _map_callback(self, message: OccupancyGrid) -> None:
        try:
            if message.header.frame_id != self.map_frame:
                raise ValueError(
                    f"occupancy-grid frame is {message.header.frame_id!r}; "
                    f"expected {self.map_frame!r}"
                )
            belief = self._belief_from_message(message)
            geometric = compute_geometric_metrics(
                self.registered_truth, belief, self.known_free_threshold
            )
        except (TypeError, ValueError) as error:
            self.map_rejection_count += 1
            self._append_event("map_rejected", {"reason": str(error)})
            self.get_logger().error(f"Rejected map snapshot: {error}")
            self._publish_status("MAP_REJECTED", str(error))
            return
        self.map_revision += 1
        self.last_map_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        self.latest_geometric = geometric
        self._emit_snapshot("map_update")

    def _trace_callback(self, message: String) -> None:
        try:
            accepted = self.actions.ingest(message.data)
        except (KeyError, TypeError, ValueError) as error:
            self.trace_rejection_count += 1
            self._append_event("trace_rejected", {
                "reason": str(error),
                "encoded_length": len(message.data),
                "sha256": hashlib.sha256(
                    message.data.encode("utf-8")
                ).hexdigest(),
            })
            self.get_logger().error(f"Rejected policy trace: {error}")
            self._publish_status("TRACE_REJECTED", str(error))
            return
        if not accepted:
            return
        if self.actions.latest_record is None:
            raise RuntimeError("accepted trace did not retain its parsed record")
        event = self.actions.latest_record["event"]
        if event == "session_started":
            try:
                session_time_ns = int(
                    self.actions.latest_record.get("ros_time_ns")
                )
                if session_time_ns < 0:
                    raise ValueError("negative session timestamp")
            except (TypeError, ValueError):
                session_time_ns = self._now_ns()
            self.ground_truth_motion = GroundTruthMotionAccumulator(
                self.ground_truth_minimum_step_m
            )
            self.clearance.reset_samples()
            self._pending_ate.clear()
            self._last_ground_truth_stamp_ns = None
            self.ate_tf_wait_count = 0
            self.ate_tf_drop_count = 0
            self.target_recall.begin_session(session_time_ns)
            self.target_session_active = True
        elif event == "session_finished":
            self.target_session_active = False
        try:
            self.topological_nodes.ingest_record(self.actions.latest_record)
        except (TypeError, ValueError) as error:
            self.topology_trace_rejection_count += 1
            self._append_event("topology_trace_rejected", {
                "event": event,
                "reason": str(error),
            })
            self.get_logger().error(f"Rejected topology trace fields: {error}")
        with self.observed_trace_path.open("a", encoding="utf-8") as stream:
            stream.write(_strict_json(self.actions.latest_record) + "\n")
        self._append_event("policy_trace_ingested", {
            "event": event,
            "accepted_trace_events": self.actions.accepted_trace_events,
            "sha256": hashlib.sha256(
                message.data.encode("utf-8")
            ).hexdigest(),
        })
        if event in {
            "session_started",
            "execution",
            "decision_error",
            "session_finished",
        }:
            self._emit_snapshot(f"policy_{event}")

    def _message_stamp_ns(self, stamp) -> int:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return stamp_ns if stamp_ns > 0 else self._now_ns()

    def _ground_truth_callback(self, message: Odometry) -> None:
        try:
            if message.header.frame_id != self.ground_truth_frame:
                raise ValueError(
                    f"ground-truth odometry frame is "
                    f"{message.header.frame_id!r}; expected "
                    f"{self.ground_truth_frame!r}"
                )
            if message.child_frame_id != self.ground_truth_child_frame:
                raise ValueError(
                    f"ground-truth odometry child frame is "
                    f"{message.child_frame_id!r}; expected "
                    f"{self.ground_truth_child_frame!r}"
                )
            stamp_ns = self._message_stamp_ns(message.header.stamp)
            position = message.pose.pose.position
            truth_x = float(position.x)
            truth_y = float(position.y)
            if not math.isfinite(truth_x) or not math.isfinite(truth_y):
                raise ValueError("ground-truth position must be finite")
            orientation = message.pose.pose.orientation
            truth_yaw = _quaternion_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
        except (TypeError, ValueError) as error:
            self.ground_truth_rejection_count += 1
            self._append_event("ground_truth_rejected", {"reason": str(error)})
            self.get_logger().error(f"Rejected ground-truth odometry: {error}")
            self._publish_status("GROUND_TRUTH_REJECTED", str(error))
            return

        if not self.target_session_active:
            return

        if (
            self._last_ground_truth_stamp_ns is not None
            and stamp_ns < self._last_ground_truth_stamp_ns
        ):
            self.ate_tf_drop_count += len(self._pending_ate)
            self._pending_ate.clear()
        self._last_ground_truth_stamp_ns = stamp_ns
        moved = self.ground_truth_motion.add_ground_truth(
            stamp_ns, truth_x, truth_y
        )
        self.clearance.add(truth_x, truth_y)
        truth_in_map = transform_planar_point(
            (truth_x, truth_y), self.truth_to_map[:2], self.truth_to_map[2]
        )
        self._pending_ate.append((stamp_ns, truth_in_map[0], truth_in_map[1]))
        newly_detected = (
            self.target_recall.ingest(
                stamp_ns, (truth_x, truth_y, truth_yaw)
            )
            if self.target_session_active
            else ()
        )
        if newly_detected:
            self._append_event("targets_first_seen", {
                "target_ids": list(newly_detected),
                "target_recall": self.target_recall.snapshot()["target_recall"],
            })
            self._emit_snapshot("target_first_seen")
        elif moved or self.ground_truth_motion.path.sample_count == 1:
            self._maybe_emit_periodic("ground_truth_periodic")

    def _resolve_pending_ate(self) -> int:
        now_ns = self._now_ns()
        accepted = 0
        while self._pending_ate:
            stamp_ns, truth_x, truth_y = self._pending_ate[0]
            age_ns = now_ns - stamp_ns
            if age_ns < self.ate_pairing_delay_ns:
                break
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(
                        nanoseconds=stamp_ns,
                        clock_type=ClockType.ROS_TIME,
                    ),
                    timeout=Duration(seconds=0.0),
                )
            except TransformException as error:
                self.ate_tf_wait_count += 1
                if age_ns >= self.ate_tf_expiration_ns:
                    self._pending_ate.popleft()
                    self.ate_tf_drop_count += 1
                    if self.ate_tf_drop_count == 1 or (
                        self.ate_tf_drop_count % 100 == 0
                    ):
                        self.get_logger().warning(
                            "Dropped expired ground-truth ATE sample: "
                            f"{error}"
                        )
                    continue
                break
            self._pending_ate.popleft()
            if self.ground_truth_motion.add_ate_pair(
                stamp_ns,
                (truth_x, truth_y),
                (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                ),
            ):
                accepted += 1
        return accepted

    def _contacts_callback(self, message: Contacts) -> None:
        try:
            stamp_ns = self._message_stamp_ns(message.header.stamp)
            contacts = []
            for contact in message.contacts:
                depths = [float(depth) for depth in contact.depths]
                contacts.append((
                    str(contact.collision1.name),
                    str(contact.collision2.name),
                    max(depths, default=0.0),
                ))
            new_events = self.collisions.ingest(stamp_ns, contacts)
        except (AttributeError, TypeError, ValueError) as error:
            self.contact_rejection_count += 1
            self._append_event("contacts_rejected", {"reason": str(error)})
            self.get_logger().error(f"Rejected contacts message: {error}")
            self._publish_status("CONTACTS_REJECTED", str(error))
            return
        if new_events:
            self._append_event("collision_onset", {
                "new_collision_event_count": new_events,
                "collision_count": self.collisions.collision_event_count,
            })
            self._emit_snapshot("collision_onset")

    @staticmethod
    def _time_value_ns(value, label: str) -> int:
        seconds = int(value.sec)
        nanoseconds = int(value.nanosec)
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise ValueError(f"{label} is not a valid non-negative time")
        return seconds * 1_000_000_000 + nanoseconds

    def _world_stats_callback(self, message: WorldStatistics) -> None:
        previous_paused = self.world_statistics.snapshot()["paused_latest"]
        try:
            self.world_statistics.ingest(
                sim_time_ns=self._time_value_ns(
                    message.sim_time, "simulation time"
                ),
                pause_time_ns=self._time_value_ns(
                    message.pause_time, "pause time"
                ),
                real_time_ns=self._time_value_ns(
                    message.real_time, "real time"
                ),
                paused=bool(message.paused),
                iterations=int(message.iterations),
                model_count=int(message.model_count),
                real_time_factor=float(message.real_time_factor),
                step_size_ns=self._time_value_ns(
                    message.step_size, "step size"
                ),
                stepping=bool(message.stepping),
            )
        except (AttributeError, TypeError, ValueError) as error:
            self.world_stats_rejection_count += 1
            self._append_event("world_stats_rejected", {"reason": str(error)})
            self.get_logger().error(f"Rejected world statistics: {error}")
            self._publish_status("WORLD_STATS_REJECTED", str(error))
            return
        if previous_paused is None or previous_paused != bool(message.paused):
            self._emit_snapshot("world_stats_state_change")
        else:
            self._maybe_emit_periodic("world_stats_periodic")

    def _maybe_emit_periodic(self, reason: str) -> None:
        now_ns = self._now_ns()
        elapsed = (
            None if self.last_emit_ns is None else now_ns - self.last_emit_ns
        )
        if elapsed is None or elapsed < 0 or elapsed >= self.publish_period_ns:
            self._emit_snapshot(reason)

    def _sample_tf(self) -> None:
        ate_pairs = self._resolve_pending_ate()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            self.tf_wait_count += 1
            if self.tf_wait_count == 1 or self.tf_wait_count % 100 == 0:
                self.get_logger().debug(f"Waiting for evaluator TF: {error}")
                self._publish_status("WAIT_TF", str(error))
            return
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        if stamp_ns == 0:
            stamp_ns = self._now_ns()
        moved = self.trajectory.add(
            stamp_ns,
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        if not moved and not ate_pairs:
            return
        self._maybe_emit_periodic("motion_periodic")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SystemEvaluatorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # rclpy may report a pybind take_message conversion error after the
        # signal handler has already invalidated the context.  Do not hide the
        # same exception while ROS is still live.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            try:
                node._emit_snapshot("evaluator_shutdown", publish=False)
            except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
                pass
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
