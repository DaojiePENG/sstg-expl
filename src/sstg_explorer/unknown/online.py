"""Incremental belief-only policy interface for embodied robot execution.

The procedural benchmark owns a hidden ground-truth map, simulates sensing and
executes complete paths inside :meth:`UnknownMapExplorer.explore`.  A ROS robot
cannot use that interface: it receives an incrementally built occupancy grid,
asks for one goal, executes it through the navigation stack, and then reports
the outcome.  This module provides that stateful boundary without accepting a
ground-truth map at all.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from sstg_explorer.map import OccupancyGrid

from .explorer import UnknownExplorerConfig, UnknownMapExplorer


Pose2D = Tuple[float, float, float]
Point2D = Tuple[float, float]


@dataclass
class OnlineDecision:
    """One policy decision made from the current belief and estimated pose."""

    decision_id: int
    map_revision: Optional[int]
    status: str
    reason: str
    current_pose: Pose2D
    target_pose: Optional[Pose2D]
    selected_candidate: Optional[Dict]
    planned_path: List[Point2D]
    generated_candidates: List[Dict]
    active_candidates: List[Dict]
    known_free_cells: int
    known_topological_coverage: float
    native_termination_rule: str
    native_completion_trigger: Optional[str]
    native_completion_topological_threshold: Optional[float]
    exhaustion_confirmation: int
    exhaustion_confirmations_required: int
    decision_time_ms: float

    def to_dict(self) -> Dict:
        """Return a JSON-serializable representation."""
        return asdict(self)


@dataclass
class ExecutionRecord:
    """Outcome of executing one :class:`OnlineDecision`."""

    decision_id: int
    succeeded: bool
    reason: str
    commanded_pose: Pose2D
    reached_pose: Pose2D
    path: List[Point2D]
    path_frame: str
    translation_m: float
    rotation_deg: float
    failure_neighborhood_recorded: bool
    topological_node_created: bool
    topological_node_id: Optional[int]

    def to_dict(self) -> Dict:
        """Return a JSON-serializable representation."""
        return asdict(self)


class OnlineExplorerSession:
    """Stateful, hidden-truth-free adapter for ROS or another robot runtime.

    ``propose`` consumes only the online belief map and estimated pose.
    ``record_execution`` consumes the navigation result.  Neither method has a
    ground-truth argument; truth coverage remains an evaluator responsibility.
    """

    _NATIVE_TERMINATION_RULES = {
        "frontier": "no_reachable_frontier_or_joint_gap_above_gain",
        "nbv": "no_nbv_candidate_above_information_or_topological_gain",
        "rrt": "no_rrt_expansion_above_information_or_topological_gain",
        "ans": "no_ans_guided_candidate_above_information_or_topological_gain",
        "sstg": (
            "no_reachable_informative_frontier_and_"
            "known_topological_target_plus_one_cell_margin_met"
        ),
    }

    def __init__(self, config: UnknownExplorerConfig, start_pose: Pose2D):
        if config.termination_mode != "candidate_exhaustion":
            raise ValueError(
                "online exploration requires termination_mode="
                "'candidate_exhaustion' because policy-visible state has no truth"
            )
        self.config = config
        self.policy = UnknownMapExplorer(config)
        self.current_pose = self._normalize_pose(start_pose)
        self.nodes: List[Dict] = [{
            "id": 0,
            "position": self.current_pose[:2],
            "orientation": self.current_pose[2],
            "timestamp": 0,
        }]
        self.oriented_views: List[Dict] = [{
            "id": 0,
            "position": self.current_pose[:2],
            "orientation": self.current_pose[2],
            "timestamp": 0,
            "topological_node_id": 0,
        }]
        self.execution_records: List[ExecutionRecord] = []
        self.total_distance_m = 0.0
        self.total_rotation_deg = 0.0
        self._next_decision_id = 1
        self._pending: Optional[OnlineDecision] = None
        self._map_resolution: Optional[float] = None
        self._exhaustion_confirmation = 0
        self._last_exhaustion_map_revision: Optional[int] = None
        self._native_completion_trigger: Optional[str] = None
        self.termination_reason = "running"

    @staticmethod
    def _normalize_pose(pose: Sequence[float]) -> Pose2D:
        if len(pose) != 3:
            raise ValueError("pose must contain x, y and heading degrees")
        values = tuple(float(value) for value in pose)
        if not all(np.isfinite(values)):
            raise ValueError("pose values must be finite")
        return values[0], values[1], values[2] % 360.0

    @staticmethod
    def _path_length(path: Sequence[Point2D]) -> float:
        return float(sum(
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(path[:-1], path[1:])
        ))

    @property
    def positions(self) -> List[Point2D]:
        return [tuple(node["position"]) for node in self.nodes]

    @property
    def pending_decision(self) -> Optional[OnlineDecision]:
        return self._pending

    def _coverage_state(self, belief: OccupancyGrid) -> Tuple[int, float]:
        known_free = int(np.sum(belief.get_free_space_mask()))
        _, _, ratio = self.policy._known_topological_state(
            belief, self.positions
        )
        return known_free, ratio

    def _sstg_frontier_topology_converged(
        self,
        active: Sequence[Dict],
        known_topological_coverage: float,
        belief_resolution: float,
    ) -> bool:
        """Apply SSTG's belief-only native stopping condition.

        A locally tiny map can appear fully covered, so the topological target
        is never sufficient by itself.  At least one reachable frontier with
        predicted information gain keeps exploration active.  Once those
        frontiers are exhausted and the already-known free space meets the
        frozen topological target, remaining topology-only gap candidates are
        tail refinement rather than evidence of another unknown room.
        """
        if self.config.strategy != "sstg":
            return False
        if known_topological_coverage + 1e-12 < (
            self._sstg_native_topological_threshold(belief_resolution)
        ):
            return False
        return not any(
            candidate.get("kind") == "frontier"
            and int(candidate.get("predicted_gain", 0))
            >= self.config.min_gain_cells
            for candidate in active
        )

    def _sstg_native_topological_threshold(
        self, belief_resolution: float
    ) -> float:
        """Add one normalized map cell of conservative belief-side margin."""
        return min(
            1.0,
            self.config.target_topological_coverage
            + float(belief_resolution) / self.config.topological_radius,
        )

    def propose(
        self,
        belief: OccupancyGrid,
        current_pose: Optional[Pose2D] = None,
        map_revision: Optional[int] = None,
    ) -> OnlineDecision:
        """Propose one reachable goal using only policy-visible state.

        A pending decision must be resolved with :meth:`record_execution`
        before another goal is requested.  Completion means the policy has no
        remaining informative or topological-gap candidate; it is not a claim
        about evaluator-only ground-truth coverage.
        """
        if self._pending is not None:
            raise RuntimeError(
                "record the pending execution before requesting another goal"
            )
        if current_pose is not None:
            self.current_pose = self._normalize_pose(current_pose)
        if self._map_resolution is None:
            self._map_resolution = float(belief.resolution)
        elif not math.isclose(
            belief.resolution, self._map_resolution, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError("belief map resolution changed within one session")
        if not belief.is_valid_world(*self.current_pose[:2]):
            raise ValueError("current pose is outside the belief map")

        started = time.perf_counter()
        current = self.current_pose[:2]
        heading = self.current_pose[2]
        known_free_cells, known_topology = self._coverage_state(belief)
        planner, safe, reachable, cost_map = self.policy._known_safe_planner(
            belief, current
        )
        ans_predicted = (
            self.policy._ans_predicted_cell(belief, current, heading)
            if self.config.strategy == "ans" else None
        )
        raw = self.policy._raw_candidates(
            belief,
            current,
            heading,
            safe,
            reachable,
            cost_map,
            self.positions,
        )
        active, traced = self.policy._evaluate_candidates(
            belief, raw, ans_predicted
        )
        selected = self.policy._select_candidate(
            active, belief, ans_predicted
        )

        native_completion_trigger: Optional[str] = None
        if self._sstg_frontier_topology_converged(
            active, known_topology, belief.resolution
        ):
            selected = None
            native_completion_trigger = "sstg_frontier_topology_convergence"
        elif selected is None:
            native_completion_trigger = "candidate_exhaustion"
        navigation_failure_exhaustion = bool(
            selected is None
            and not active
            and any(
                candidate.get("status")
                == "pruned_navigation_failure_neighborhood"
                for candidate in traced
            )
        )
        if navigation_failure_exhaustion:
            native_completion_trigger = "navigation_failure_exhaustion"

        status = (
            "failed" if navigation_failure_exhaustion else "confirming"
        )
        reason = (
            "navigation_failure_exhaustion"
            if navigation_failure_exhaustion
            else (
                f"{native_completion_trigger}_pending"
                if native_completion_trigger is not None
                else "candidate_exhaustion_pending"
            )
        )
        target_pose = None
        planned_path: List[Point2D] = []
        if selected is not None:
            self._exhaustion_confirmation = 0
            self._last_exhaustion_map_revision = None
            self._native_completion_trigger = None
            path = self.policy._plan(
                planner, current, selected, belief.data.size
            )
            if path:
                status = "navigate"
                reason = "goal_selected"
                planned_path = [tuple(map(float, point)) for point in path]
                target_pose = (
                    float(selected["target"][0]),
                    float(selected["target"][1]),
                    float(selected["heading"]) % 360.0,
                )
            else:
                status = "stalled"
                reason = "selected_candidate_unreachable"
        elif navigation_failure_exhaustion:
            self._native_completion_trigger = native_completion_trigger
        elif (
            map_revision is None
            or map_revision != self._last_exhaustion_map_revision
        ):
            self._exhaustion_confirmation += 1
            self._last_exhaustion_map_revision = map_revision
            self._native_completion_trigger = native_completion_trigger
        if (
            selected is None
            and not navigation_failure_exhaustion
            and self._exhaustion_confirmation
            >= self.config.online_exhaustion_confirmations
        ):
            status = "complete"
            reason = "candidate_exhaustion"

        decision = OnlineDecision(
            decision_id=self._next_decision_id,
            map_revision=map_revision,
            status=status,
            reason=reason,
            current_pose=self.current_pose,
            target_pose=target_pose,
            selected_candidate=(dict(selected) if selected is not None else None),
            planned_path=planned_path,
            generated_candidates=[dict(candidate) for candidate in traced],
            active_candidates=[dict(candidate) for candidate in active],
            known_free_cells=known_free_cells,
            known_topological_coverage=known_topology,
            native_termination_rule=self._NATIVE_TERMINATION_RULES[
                self.config.strategy
            ],
            native_completion_trigger=native_completion_trigger,
            native_completion_topological_threshold=(
                self._sstg_native_topological_threshold(belief.resolution)
                if self.config.strategy == "sstg" else None
            ),
            exhaustion_confirmation=self._exhaustion_confirmation,
            exhaustion_confirmations_required=(
                self.config.online_exhaustion_confirmations
            ),
            decision_time_ms=(time.perf_counter() - started) * 1000.0,
        )
        self._next_decision_id += 1
        if decision.status == "navigate":
            self._pending = decision
        elif decision.status != "confirming":
            self.termination_reason = decision.reason
        return decision

    def terminate(self, reason: str) -> None:
        """Record an evaluator-neutral external fail-safe termination."""
        normalized = str(reason).strip()
        if not normalized:
            raise ValueError("termination reason must be non-empty")
        self.termination_reason = normalized

    def record_execution(
        self,
        decision_id: int,
        succeeded: bool,
        reached_pose: Pose2D,
        executed_path: Optional[Sequence[Point2D]] = None,
        reason: str = "navigation_result",
        executed_path_frame: str = "map",
        suppress_failed_target: bool = True,
    ) -> ExecutionRecord:
        """Record a navigation outcome and update execution-aware memory.

        ``suppress_failed_target`` must be false for transport/rejection errors
        and adapter-owned cancellations: those events do not establish that the
        commanded spatial neighborhood is physically difficult to reach.
        """
        if self._pending is None or self._pending.decision_id != decision_id:
            raise ValueError("decision_id does not match the pending decision")
        decision = self._pending
        if decision.target_pose is None or decision.selected_candidate is None:
            raise RuntimeError("pending navigation decision has no target")

        reached = self._normalize_pose(reached_pose)
        path_frame = str(executed_path_frame).strip()
        if not path_frame:
            raise ValueError("executed_path_frame must be non-empty")
        source_path = (
            decision.planned_path if executed_path is None else executed_path
        )
        path = [
            (float(point[0]), float(point[1]))
            for point in source_path
        ]
        # Map-frame paths can be completed with the estimated start and end
        # poses.  A path sampled in odom (or another continuous execution
        # frame) must not be mixed with map-frame coordinates when a SLAM loop
        # closure changes map -> odom.
        if path_frame == "map":
            if not path:
                path = [self.current_pose[:2], reached[:2]]
            elif path[0] != self.current_pose[:2]:
                path.insert(0, self.current_pose[:2])
            if path[-1] != reached[:2]:
                path.append(reached[:2])

        translation = self._path_length(path)
        rotation = self.policy._angle_delta(
            reached[2], self.current_pose[2]
        )
        node_created = False
        node_id: Optional[int] = None
        execution_key = tuple(
            decision.selected_candidate.get("execution_key", ())
        )
        # Do not immediately retry a navigation failure forever.  The result
        # remains explicitly visible in ``execution_records``.
        if execution_key:
            self.policy._executed_candidate_keys.add(execution_key)
        failure_neighborhood_recorded = bool(
            not succeeded
            and suppress_failed_target
            and self.config.failed_goal_suppression_radius > 0.0
        )
        if failure_neighborhood_recorded:
            self.policy._failed_goal_positions.append(
                tuple(map(float, decision.target_pose[:2]))
            )

        # Physical execution cost is incurred even when Nav2 ultimately
        # aborts or a goal is canceled.  Keeping only successful travel would
        # make system-level method comparisons optimistically biased.
        self.total_distance_m += translation
        self.total_rotation_deg += rotation

        if succeeded:
            nearest_id = min(
                range(len(self.nodes)),
                key=lambda index: math.hypot(
                    reached[0] - self.nodes[index]["position"][0],
                    reached[1] - self.nodes[index]["position"][1],
                ),
            )
            nearest_distance = math.hypot(
                reached[0] - self.nodes[nearest_id]["position"][0],
                reached[1] - self.nodes[nearest_id]["position"][1],
            )
            node_created = (
                nearest_distance >= self.config.topological_merge_distance
            )
            if node_created:
                node_id = len(self.nodes)
                self.nodes.append({
                    "id": node_id,
                    "position": reached[:2],
                    "orientation": reached[2],
                    "timestamp": decision_id,
                })
            else:
                node_id = nearest_id
            self.oriented_views.append({
                "id": len(self.oriented_views),
                "position": reached[:2],
                "orientation": reached[2],
                "timestamp": decision_id,
                "topological_node_id": node_id,
            })
        record = ExecutionRecord(
            decision_id=decision_id,
            succeeded=bool(succeeded),
            reason=str(reason),
            commanded_pose=decision.target_pose,
            reached_pose=reached,
            path=path,
            path_frame=path_frame,
            translation_m=translation,
            rotation_deg=rotation,
            failure_neighborhood_recorded=failure_neighborhood_recorded,
            topological_node_created=node_created,
            topological_node_id=node_id,
        )
        self.execution_records.append(record)
        self.current_pose = reached
        self._pending = None
        return record

    def summary(self) -> Dict:
        """Return evaluator-neutral execution state for manifests and logs."""
        return {
            "strategy": self.config.strategy,
            "coverage_objective": self.config.coverage_objective,
            "termination_mode": self.config.termination_mode,
            "termination_reason": self.termination_reason,
            "native_termination_rule": self._NATIVE_TERMINATION_RULES[
                self.config.strategy
            ],
            "native_completion_trigger": self._native_completion_trigger,
            "exhaustion_confirmation": self._exhaustion_confirmation,
            "exhaustion_confirmations_required": (
                self.config.online_exhaustion_confirmations
            ),
            "decisions_issued": self._next_decision_id - 1,
            "executions": len(self.execution_records),
            "successful_executions": sum(
                record.succeeded for record in self.execution_records
            ),
            "failed_goal_suppression_radius_m": (
                self.config.failed_goal_suppression_radius
            ),
            "failed_goal_neighborhoods": [
                list(position)
                for position in self.policy._failed_goal_positions
            ],
            "total_distance_m": self.total_distance_m,
            "total_rotation_deg": self.total_rotation_deg,
            "nodes": [dict(node) for node in self.nodes],
            "oriented_views": [dict(view) for view in self.oriented_views],
        }
