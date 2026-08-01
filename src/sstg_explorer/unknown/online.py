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
    translation_m: float
    rotation_deg: float
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

    def __init__(self, config: UnknownExplorerConfig, start_pose: Pose2D):
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

        status = "complete"
        reason = "candidate_exhaustion"
        target_pose = None
        planned_path: List[Point2D] = []
        if selected is not None:
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
            decision_time_ms=(time.perf_counter() - started) * 1000.0,
        )
        self._next_decision_id += 1
        if decision.status == "navigate":
            self._pending = decision
        return decision

    def record_execution(
        self,
        decision_id: int,
        succeeded: bool,
        reached_pose: Pose2D,
        executed_path: Optional[Sequence[Point2D]] = None,
        reason: str = "navigation_result",
    ) -> ExecutionRecord:
        """Record a navigation outcome and update the online topology."""
        if self._pending is None or self._pending.decision_id != decision_id:
            raise ValueError("decision_id does not match the pending decision")
        decision = self._pending
        if decision.target_pose is None or decision.selected_candidate is None:
            raise RuntimeError("pending navigation decision has no target")

        reached = self._normalize_pose(reached_pose)
        path = [
            (float(point[0]), float(point[1]))
            for point in (executed_path or decision.planned_path)
        ]
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
            translation_m=translation,
            rotation_deg=rotation,
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
            "decisions_issued": self._next_decision_id - 1,
            "executions": len(self.execution_records),
            "successful_executions": sum(
                record.succeeded for record in self.execution_records
            ),
            "total_distance_m": self.total_distance_m,
            "total_rotation_deg": self.total_rotation_deg,
            "nodes": [dict(node) for node in self.nodes],
            "oriented_views": [dict(view) for view in self.oriented_views],
        }
