"""Common online unknown-map protocol and exploration-policy adapters."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import (
    binary_dilation,
    binary_erosion,
    distance_transform_edt,
    label,
)

from sstg_explorer.baselines.active_neural_slam import ActiveNeuralSLAMExplorer
from sstg_explorer.map import OccupancyGrid
from sstg_explorer.planning.astar import AStarPlanner
from sstg_explorer.sensing import RaycastSensor, SensorConfig


UNKNOWN_STRATEGIES = ("frontier", "nbv", "rrt", "ans", "sstg")


@dataclass
class UnknownExplorerConfig:
    """Configuration shared by every policy in the unknown-map benchmark."""

    strategy: str = "sstg"
    sensor: SensorConfig = field(default_factory=SensorConfig)
    target_coverage: float = 0.95
    max_decisions: int = 80
    robot_radius: float = 0.3
    safety_margin: float = 0.0
    preferred_clearance: float = 0.5
    target_spacing: float = 2.0
    scan_interval: float = 1.0
    min_gain_cells: int = 8
    max_frontier_candidates: int = 48
    random_candidates: int = 24
    exact_gain_budget: int = 18
    clearance_weight: float = 1.5
    spacing_weight: float = 0.0
    multi_frontier: bool = True
    use_topological_vantages: bool = True
    require_known_footprint: bool = True
    seed: int = 42
    checkpoint: Optional[str] = None
    verbose: bool = False

    def __post_init__(self):
        if self.strategy not in UNKNOWN_STRATEGIES:
            raise ValueError(f"Unknown strategy: {self.strategy}")
        if not 0.0 < self.target_coverage <= 1.0:
            raise ValueError("target_coverage must be in (0, 1]")


class UnknownMapExplorer:
    """Explore a hidden static grid through a belief-map-only policy interface.

    The ground-truth grid is held by the sensor/evaluator. Candidate generation,
    utility, reachability and planning receive only the incrementally observed
    belief grid. This separation is the central invariant of the protocol.
    """

    requires_occupancy_grid = True
    DISPLAY_NAMES = {
        "frontier": "Frontier-Unknown",
        "nbv": "NBV-Unknown",
        "rrt": "RRT-Unknown (adapted)",
        "ans": "ANS-Global Unknown (adapted)",
        "sstg": "SSTG-Explorer Unknown",
    }

    def __init__(self, config: Optional[UnknownExplorerConfig] = None, **kwargs):
        self.config = config or UnknownExplorerConfig(**kwargs)
        self.name = self.DISPLAY_NAMES[self.config.strategy]
        self.sensor = RaycastSensor(self.config.sensor)
        self.rng = np.random.default_rng(self.config.seed)
        self._candidate_ids: Dict[Tuple, int] = {}
        self._next_candidate_id = 0
        self._previous_candidate_keys = set()
        self._ans = None
        self._previous_known_mask = None
        if self.config.strategy == "ans":
            if not self.config.checkpoint:
                raise ValueError("ANS unknown policy requires checkpoint=...")
            self._ans = ActiveNeuralSLAMExplorer(
                checkpoint=self.config.checkpoint,
                r_view=2.0,
                target_coverage=self.config.target_coverage,
                r_robot=self.config.robot_radius,
                d_safe=self.config.safety_margin,
            )

    @staticmethod
    def _heading(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return float(np.degrees(np.arctan2(b[1] - a[1], b[0] - a[0])) % 360.0)

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        return float(abs((a - b + 180.0) % 360.0 - 180.0))

    def _candidate_id(self, key: Tuple) -> int:
        if key not in self._candidate_ids:
            self._candidate_ids[key] = self._next_candidate_id
            self._next_candidate_id += 1
        return self._candidate_ids[key]

    def _known_safe_planner(
        self,
        belief: OccupancyGrid,
        current: Tuple[float, float],
    ) -> Tuple[AStarPlanner, np.ndarray, np.ndarray, np.ndarray]:
        """Build a planner that traverses known free cells only."""
        known_free = belief.get_free_space_mask()
        inflation_cells = int(np.ceil(
            (self.config.robot_radius + self.config.safety_margin) /
            belief.resolution
        ))
        yy, xx = np.ogrid[
            -inflation_cells:inflation_cells + 1,
            -inflation_cells:inflation_cells + 1,
        ]
        footprint = xx * xx + yy * yy <= inflation_cells * inflation_cells
        # A robot centre is traversable only when its complete footprint is
        # observed free.  Merely inflating known obstacles would treat an
        # unobserved obstacle beside a ray as safe and could trap the robot
        # after the next scan.
        if self.config.require_known_footprint:
            safe = binary_erosion(
                known_free, structure=footprint, border_value=0
            )
        else:
            inflated_known_obstacles = binary_dilation(
                belief.get_occupied_mask(), structure=footprint
            )
            safe = known_free & (~inflated_known_obstacles)
        current_cell = belief.world_to_grid(*current)
        safe[current_cell] = True

        planning_data = np.full(belief.shape, 100, dtype=np.int8)
        planning_data[safe] = 0
        planning_grid = OccupancyGrid(
            planning_data, belief.resolution, belief.origin
        )
        planner = AStarPlanner(planning_grid, robot_radius=0.0, safety_margin=0.0)
        cost_map = planner.compute_cost_map(current)

        # MCP's fully-connected graph admits diagonal corner cutting whereas
        # the execution A* deliberately forbids it.  Use a conservative
        # four-connected component as the policy reachability invariant, then
        # mask the geodesic map with the same invariant.  Every candidate
        # labelled reachable is therefore executable by the stricter planner.
        components, _ = label(safe, structure=np.array([
            [0, 1, 0],
            [1, 1, 1],
            [0, 1, 0],
        ], dtype=bool))
        current_component = int(components[current_cell])
        reachable = (
            components == current_component
            if current_component > 0 else np.zeros_like(safe)
        )
        cost_map[~reachable] = np.inf
        return planner, safe, reachable, cost_map

    def _spatial_representatives(
        self,
        mask: np.ndarray,
        cost_map: np.ndarray,
        clearance: np.ndarray,
        max_count: int,
        resolution: float,
    ) -> List[Tuple[int, int]]:
        """Deterministic farthest-point representatives of a belief region."""
        indices = np.argwhere(mask & np.isfinite(cost_map))
        if not len(indices) or max_count <= 0:
            return []

        costs = cost_map[indices[:, 0], indices[:, 1]]
        clearances = clearance[indices[:, 0], indices[:, 1]]
        finite_cost = np.nan_to_num(costs, nan=0.0, posinf=0.0)
        seed_score = finite_cost + 0.25 * clearances
        chosen = [int(np.argmax(seed_score))]
        delta = indices - indices[chosen[0]]
        min_squared = np.einsum("ij,ij->i", delta, delta).astype(float)
        spacing_cells = self.config.target_spacing / max(resolution, 1e-9)

        while len(chosen) < min(max_count, len(indices)):
            next_index = int(np.argmax(min_squared))
            if math.sqrt(float(min_squared[next_index])) < spacing_cells:
                break
            chosen.append(next_index)
            delta = indices - indices[next_index]
            squared = np.einsum("ij,ij->i", delta, delta)
            min_squared = np.minimum(min_squared, squared)
        return [tuple(map(int, indices[index])) for index in chosen]

    def _nearest_unknown_heading(
        self,
        row: int,
        col: int,
        nearest_unknown: np.ndarray,
        grid: OccupancyGrid,
    ) -> float:
        unknown_row = int(nearest_unknown[0, row, col])
        unknown_col = int(nearest_unknown[1, row, col])
        return self._heading(
            grid.grid_to_world(row, col),
            grid.grid_to_world(unknown_row, unknown_col),
        )

    def _optimistic_gain(
        self,
        candidate: Dict,
        unknown_xy: np.ndarray,
    ) -> int:
        if unknown_xy.size == 0:
            return 0
        target = np.asarray(candidate["target"], dtype=float)
        delta = unknown_xy - target
        squared = np.einsum("ij,ij->i", delta, delta)
        in_range = squared <= self.config.sensor.max_range ** 2
        fov = self.config.sensor.field_of_view_deg
        if fov < 360.0 - 1e-9:
            angles = np.degrees(np.arctan2(delta[:, 1], delta[:, 0])) % 360.0
            heading = float(candidate["heading"])
            angle_delta = np.abs((angles - heading + 180.0) % 360.0 - 180.0)
            in_range &= angle_delta <= fov / 2.0
        return int(np.sum(in_range))

    def _ans_predicted_cell(
        self,
        belief: OccupancyGrid,
        current: Tuple[float, float],
        heading: float,
    ) -> Optional[Tuple[int, int]]:
        if self._ans is None:
            return None
        import torch

        map_cells = self._ans.map_cells
        local_cells = self._ans.local_cells
        offset_r = (map_cells - belief.height) // 2
        offset_c = (map_cells - belief.width) // 2
        if offset_r < 0 or offset_c < 0:
            return None
        known = ~belief.get_unknown_mask()
        occupied = belief.get_occupied_mask()
        full = np.zeros((4, map_cells, map_cells), dtype=np.float32)
        region = (
            slice(offset_r, offset_r + belief.height),
            slice(offset_c, offset_c + belief.width),
        )
        full[0][region] = occupied
        full[1][region] = known
        if self._previous_known_mask is not None:
            full[3][region] = self._previous_known_mask
        current_cell = belief.world_to_grid(*current)
        cr, cc = current_cell[0] + offset_r, current_cell[1] + offset_c
        full[2, max(0, cr - 1):cr + 2, max(0, cc - 1):cc + 2] = 1.0
        local = self._ans._crop(full, (cr, cc), local_cells)
        pooled = full.reshape(4, local_cells, 2, local_cells, 2).max(axis=(2, 4))
        policy_input = np.concatenate((local, pooled), axis=0)
        orientation = int(((heading + 180.0) % 360.0) / 5.0)
        with torch.no_grad():
            action = self._ans.policy.deterministic_goal(
                torch.from_numpy(policy_input[None]),
                torch.tensor([[orientation]], dtype=torch.long),
            )[0].numpy()
        predicted_local = np.clip(
            (action * (local_cells - 1)).astype(int), 0, local_cells - 1
        )
        predicted = (
            cr - local_cells // 2 + int(predicted_local[0]) - offset_r,
            cc - local_cells // 2 + int(predicted_local[1]) - offset_c,
        )
        self._previous_known_mask = known.copy()
        return predicted

    def _raw_candidates(
        self,
        belief: OccupancyGrid,
        current: Tuple[float, float],
        current_heading: float,
        safe: np.ndarray,
        reachable: np.ndarray,
        cost_map: np.ndarray,
        explored_positions: Sequence[Tuple[float, float]],
    ) -> List[Dict]:
        unknown = belief.get_unknown_mask()
        if not np.any(unknown):
            return []
        _, nearest_unknown = distance_transform_edt(
            ~unknown, return_indices=True
        )
        known_obstacles = belief.get_occupied_mask()
        clearance = distance_transform_edt(~known_obstacles) * belief.resolution
        unknown_distance = distance_transform_edt(~unknown) * belief.resolution
        frontier_band = (
            self.config.robot_radius + 2.0 * belief.resolution
        )
        frontier_mask = (
            safe & reachable &
            (unknown_distance <= frontier_band)
        )
        components, count = label(frontier_mask, structure=np.ones((3, 3)))
        candidates: List[Dict] = []

        component_sizes = np.bincount(components.ravel())
        if self.config.strategy == "sstg" and self.config.multi_frontier:
            # A large occlusion boundary is often one connected component.  A
            # single centroid can miss every doorway/aisle, so SSTG retains a
            # spatially discrete set of representatives over the full frontier.
            frontier_cells = self._spatial_representatives(
                frontier_mask, cost_map, clearance,
                max_count=self.config.max_frontier_candidates // 2,
                resolution=belief.resolution,
            )
        else:
            # Classic baselines keep one representative per connected cluster.
            frontier_cells = []
            clusters = []
            for component_id in range(1, count + 1):
                indices = np.argwhere(components == component_id)
                if len(indices):
                    clusters.append(indices)
            clusters.sort(key=len, reverse=True)
            for indices in clusters[:self.config.max_frontier_candidates]:
                center = np.mean(indices, axis=0)
                local_clearance = clearance[indices[:, 0], indices[:, 1]]
                center_cost = np.linalg.norm(indices - center, axis=1)
                pick = int(np.argmax(
                    local_clearance - center_cost * belief.resolution * 0.1
                ))
                frontier_cells.append(tuple(map(int, indices[pick])))

        for row, col in frontier_cells:
            heading = self._nearest_unknown_heading(
                row, col, nearest_unknown, belief
            )
            target = belief.grid_to_world(row, col)
            key = (row, col, int(round(heading)) % 360, "frontier")
            candidates.append({
                "frontier_id": self._candidate_id(key), "key": key,
                "target": target, "heading": heading, "kind": "frontier",
                "path_cost": float(cost_map[row, col]),
                "clearance": float(clearance[row, col]),
                "cluster_size": int(component_sizes[components[row, col]]),
                "status": "generated",
            })

        if (
            self.config.strategy == "sstg" and
            self.config.use_topological_vantages
        ):
            # Deterministic topological vantages inside known-safe space let the
            # policy look through occluded doorways without pretending that an
            # unknown cell is traversable.  They are belief-derived, unlike a
            # ground-truth uniform grid.
            vantage_cells = self._spatial_representatives(
                reachable, cost_map, clearance,
                max_count=self.config.max_frontier_candidates // 2,
                resolution=belief.resolution,
            )
            for row, col in vantage_cells:
                target = belief.grid_to_world(row, col)
                heading = self._nearest_unknown_heading(
                    row, col, nearest_unknown, belief
                )
                key = (row, col, int(round(heading)) % 360, "vantage")
                candidates.append({
                    "frontier_id": self._candidate_id(key), "key": key,
                    "target": target, "heading": heading,
                    "kind": "topological_vantage",
                    "path_cost": float(cost_map[row, col]),
                    "clearance": float(clearance[row, col]),
                    "cluster_size": 1, "status": "generated",
                })

        # Directional sensors may gain information by rotating without adding
        # translational path length. These remain separate oriented viewpoints.
        if self.config.sensor.field_of_view_deg < 360.0 - 1e-9:
            rotation_step = max(45.0, self.config.sensor.field_of_view_deg / 2.0)
            current_row, current_col = belief.world_to_grid(*current)
            for heading in np.arange(0.0, 360.0, rotation_step):
                if self._angle_delta(float(heading), current_heading) < 1.0:
                    continue
                key = (
                    current_row, current_col, int(round(heading)) % 360,
                    "rotation",
                )
                candidates.append({
                    "frontier_id": self._candidate_id(key), "key": key,
                    "target": current, "heading": float(heading),
                    "kind": "rotation", "path_cost": 0.0,
                    "clearance": float(clearance[current_row, current_col]),
                    "cluster_size": 1, "status": "generated",
                })

        if self.config.strategy in ("nbv", "rrt"):
            indices = np.argwhere(reachable)
            if len(indices):
                sample_count = min(self.config.random_candidates, len(indices))
                chosen = self.rng.choice(len(indices), size=sample_count, replace=False)
                for row, col in indices[chosen]:
                    row, col = int(row), int(col)
                    heading = self._nearest_unknown_heading(
                        row, col, nearest_unknown, belief
                    )
                    key = (row, col, int(round(heading)) % 360, "sampled")
                    candidates.append({
                        "frontier_id": self._candidate_id(key), "key": key,
                        "target": belief.grid_to_world(row, col),
                        "heading": heading, "kind": "sampled",
                        "path_cost": float(cost_map[row, col]),
                        "clearance": float(clearance[row, col]),
                        "cluster_size": 1, "status": "generated",
                    })

        # The frontier and topological samplers can represent the same
        # oriented grid pose.  Keep the first (frontier has precedence) so the
        # exact-gain budget and trace do not contain duplicate hypotheses.
        unique_candidates = {}
        for candidate in candidates:
            dedup_key = (
                tuple(np.round(candidate["target"], 6)),
                round(float(candidate["heading"]) % 360.0, 4),
            )
            unique_candidates.setdefault(dedup_key, candidate)
        candidates = list(unique_candidates.values())

        unknown_rows, unknown_cols = np.nonzero(unknown)
        unknown_xy = np.column_stack((
            belief.origin[0] + (unknown_cols + 0.5) * belief.resolution,
            belief.origin[1] + (unknown_rows + 0.5) * belief.resolution,
        ))
        for candidate in candidates:
            candidate["optimistic_gain"] = self._optimistic_gain(
                candidate, unknown_xy
            )
            distances = [
                math.hypot(
                    candidate["target"][0] - old[0],
                    candidate["target"][1] - old[1],
                )
                for old in explored_positions
            ]
            candidate["nearest_viewpoint_distance"] = (
                float(min(distances)) if distances else float("inf")
            )
        return candidates

    def _evaluate_candidates(
        self,
        belief: OccupancyGrid,
        candidates: List[Dict],
        ans_predicted: Optional[Tuple[int, int]],
    ) -> Tuple[List[Dict], List[Dict]]:
        """Spend exact occlusion-aware gain computation on a bounded shortlist."""
        if not candidates:
            return [], []
        preliminary = np.asarray([
            candidate["optimistic_gain"] /
            (1.0 + candidate["path_cost"] / max(self.config.sensor.max_range, 1e-6))
            for candidate in candidates
        ])
        shortlist = set(np.argsort(preliminary)[-self.config.exact_gain_budget:].tolist())
        closest = np.argsort([candidate["path_cost"] for candidate in candidates])[:6]
        shortlist.update(map(int, closest))
        shortlist.update(
            index for index, candidate in enumerate(candidates)
            if candidate["kind"] == "rotation"
        )
        if ans_predicted is not None:
            nearest = min(
                range(len(candidates)),
                key=lambda index: (
                    belief.world_to_grid(*candidates[index]["target"])[0] - ans_predicted[0]
                ) ** 2 + (
                    belief.world_to_grid(*candidates[index]["target"])[1] - ans_predicted[1]
                ) ** 2,
            )
            shortlist.add(int(nearest))

        active, traced = [], []
        for index, candidate in enumerate(candidates):
            candidate = dict(candidate)
            candidate.pop("key", None)
            if index not in shortlist:
                candidate["status"] = "pruned_evaluation_budget"
                traced.append(candidate)
                continue
            gain = self.sensor.predict_unknown_gain(
                belief, candidate["target"], candidate["heading"]
            )
            candidate["predicted_gain"] = int(gain)
            if gain < self.config.min_gain_cells:
                candidate["status"] = "pruned_gain"
                traced.append(candidate)
                continue
            candidate["status"] = "active"
            active.append(candidate)
            traced.append(candidate)
        return active, traced

    def _select_candidate(
        self,
        active: List[Dict],
        belief: OccupancyGrid,
        ans_predicted: Optional[Tuple[int, int]],
    ) -> Optional[Dict]:
        if not active:
            return None
        max_gain = max(candidate["predicted_gain"] for candidate in active)
        strategy = self.config.strategy
        non_rotation = [candidate for candidate in active if candidate["kind"] != "rotation"]
        for candidate in active:
            gain = candidate["predicted_gain"] / max(max_gain, 1)
            cost = candidate["path_cost"] / max(self.config.sensor.max_range, 1e-6)
            clearance = min(
                candidate["clearance"] /
                max(self.config.preferred_clearance, 1e-6),
                1.0,
            )
            spacing = min(
                candidate["nearest_viewpoint_distance"] /
                max(self.config.target_spacing, 1e-6),
                1.0,
            )
            if candidate["kind"] == "rotation":
                spacing = 0.35
            if strategy == "frontier":
                if non_rotation and candidate["kind"] == "rotation":
                    score = -1e6
                else:
                    score = -candidate["path_cost"] + gain * 0.05
            elif strategy == "nbv":
                score = gain - 0.25 * cost
            elif strategy == "rrt":
                score = (0.05 + gain) / (1.0 + cost)
                score *= float(self.rng.uniform(0.75, 1.25))
            elif strategy == "ans" and ans_predicted is not None:
                row, col = belief.world_to_grid(*candidate["target"])
                predicted_distance = math.hypot(
                    row - ans_predicted[0], col - ans_predicted[1]
                ) * belief.resolution
                score = -predicted_distance / max(self.config.sensor.max_range, 1e-6) + 0.25 * gain
            else:
                # Additive normalization avoids suppressing a high-gain remote
                # doorway to zero, while explicitly rewarding clearance and
                # spatial separation from accepted viewpoints.
                score = (
                    1.20 * gain
                    - 0.15 * cost
                    + self.config.spacing_weight * spacing
                    + 0.20 * self.config.clearance_weight * clearance
                )
            candidate["priority"] = float(score)
        return max(active, key=lambda candidate: candidate["priority"])

    def _plan(
        self,
        planner: AStarPlanner,
        current: Tuple[float, float],
        selected: Dict,
        grid_size: int,
    ) -> Optional[List[Tuple[float, float]]]:
        if math.hypot(
            selected["target"][0] - current[0],
            selected["target"][1] - current[1],
        ) < planner.grid.resolution:
            return [current]
        path = planner.plan(
            current, selected["target"],
            max_iterations=max(10000, grid_size),
        )
        if path:
            path[0] = current
        return path

    def _scan_path(
        self,
        truth: OccupancyGrid,
        belief: OccupancyGrid,
        path: List[Tuple[float, float]],
        final_heading: float,
    ) -> Tuple[List[List[float]], List[List[float]], int, int]:
        """Observe continuously along a path and return compact map updates."""
        new_values: Dict[int, int] = {}
        scan_poses: List[List[float]] = []
        visible_total = 0
        if len(path) > 1:
            for start, end in zip(path[:-1], path[1:]):
                distance = math.hypot(end[0] - start[0], end[1] - start[1])
                count = max(1, int(np.ceil(distance / self.config.scan_interval)))
                heading = self._heading(start, end)
                for fraction in np.linspace(1.0 / count, 1.0, count):
                    point = (
                        start[0] + fraction * (end[0] - start[0]),
                        start[1] + fraction * (end[1] - start[1]),
                    )
                    observation = self.sensor.observe(
                        truth, belief, point, heading
                    )
                    visible_total += len(observation.visible_flat_indices)
                    scan_poses.append([float(point[0]), float(point[1]), heading])
                    for flat in observation.new_flat_indices:
                        new_values[int(flat)] = int(belief.data.ravel()[flat])

        final_position = path[-1]
        observation = self.sensor.observe(
            truth, belief, final_position, final_heading
        )
        visible_total += len(observation.visible_flat_indices)
        scan_poses.append([
            float(final_position[0]), float(final_position[1]), float(final_heading)
        ])
        for flat in observation.new_flat_indices:
            new_values[int(flat)] = int(belief.data.ravel()[flat])
        updates = [[flat, value] for flat, value in sorted(new_values.items())]
        return updates, scan_poses, visible_total, len(new_values)

    def _execution_rotation(
        self,
        path: List[Tuple[float, float]],
        initial_heading: float,
        final_heading: float,
    ) -> float:
        """Yaw accumulated while aligning with path segments and final view."""
        headings = []
        for start, end in zip(path[:-1], path[1:]):
            if math.hypot(end[0] - start[0], end[1] - start[1]) > 1e-9:
                headings.append(self._heading(start, end))
        headings.append(float(final_heading))
        rotation = 0.0
        previous = float(initial_heading)
        for value in headings:
            rotation += self._angle_delta(value, previous)
            previous = value
        return rotation

    @staticmethod
    def _coverage(truth: OccupancyGrid, belief: OccupancyGrid) -> Dict[str, float]:
        truth_free = truth.get_free_space_mask()
        truth_occupied = truth.get_occupied_mask()
        known_free = belief.get_free_space_mask()
        known_occupied = belief.get_occupied_mask()
        known = ~belief.get_unknown_mask()
        return {
            "free_coverage": float(np.sum(known_free & truth_free) / max(np.sum(truth_free), 1)),
            "occupied_recall": float(np.sum(known_occupied & truth_occupied) / max(np.sum(truth_occupied), 1)),
            "known_ratio": float(np.mean(known)),
        }

    def explore(
        self,
        occupancy_grid: OccupancyGrid,
        start_pose: Tuple[float, float, float],
        visualizer=None,
    ) -> Dict:
        """Run online exploration; ``occupancy_grid`` is hidden truth."""
        started = time.perf_counter()
        truth = occupancy_grid
        belief = OccupancyGrid(
            np.full(truth.shape, -1, dtype=np.int8),
            truth.resolution,
            truth.origin,
        )
        current = (float(start_pose[0]), float(start_pose[1]))
        heading = float(start_pose[2] % 360.0)
        nodes = [{
            "id": 0, "position": current, "orientation": heading,
            "timestamp": 0,
        }]
        positions = [current]
        paths: List[List[Tuple[float, float]]] = []
        steps: List[Dict] = []
        total_distance = 0.0
        total_rotation = 0.0
        scan_count = 0
        in_place_rotations = 0
        stagnant_steps = 0
        termination_reason = "running"
        self._candidate_ids = {}
        self._next_candidate_id = 0
        self._previous_candidate_keys = set()
        self._previous_known_mask = None

        initial_observation = self.sensor.observe(truth, belief, current, heading)
        scan_count += 1
        initial_updates = [
            [int(flat), int(belief.data.ravel()[flat])]
            for flat in initial_observation.new_flat_indices
        ]
        initial_coverage = self._coverage(truth, belief)
        steps.append({
            "trace_id": 0, "iteration": 0, "event": "initial_observation",
            "current_pose": [current[0], current[1], heading],
            "selected_frontier": None, "path": [],
            "translation_m": 0.0, "rotation_deg": 0.0,
            "explored_nodes": [dict(nodes[0])],
            "generated_candidates": [], "new_frontiers": [],
            "active_frontiers": [], "observed_updates": initial_updates,
            "scan_poses": [[current[0], current[1], heading]],
            "visible_cell_count": int(len(initial_observation.visible_flat_indices)),
            "new_observed_count": int(len(initial_updates)),
            "coverage_before": 0.0,
            "coverage_after": initial_coverage["free_coverage"],
            "coverage_gain": initial_coverage["free_coverage"],
            "known_ratio": initial_coverage["known_ratio"],
            "occupied_recall": initial_coverage["occupied_recall"],
            "sensor": {
                "fov_deg": self.config.sensor.field_of_view_deg,
                "range_m": self.config.sensor.max_range,
            },
        })

        for decision in range(1, self.config.max_decisions + 1):
            before = self._coverage(truth, belief)
            if before["free_coverage"] >= self.config.target_coverage:
                termination_reason = "coverage_target"
                break
            planner, safe, reachable, cost_map = self._known_safe_planner(
                belief, current
            )
            ans_predicted = self._ans_predicted_cell(
                belief, current, heading
            ) if self.config.strategy == "ans" else None
            raw = self._raw_candidates(
                belief, current, heading, safe, reachable, cost_map, positions
            )
            active, traced = self._evaluate_candidates(
                belief, raw, ans_predicted
            )
            selected = self._select_candidate(active, belief, ans_predicted)
            active_keys = {
                (
                    tuple(np.round(candidate["target"], 4)),
                    round(candidate["heading"], 2), candidate["kind"],
                )
                for candidate in active
            }
            new_frontiers = [
                candidate for candidate in active
                if (
                    tuple(np.round(candidate["target"], 4)),
                    round(candidate["heading"], 2), candidate["kind"],
                ) not in self._previous_candidate_keys
            ]
            self._previous_candidate_keys = active_keys
            if selected is None:
                termination_reason = "no_informative_candidate"
                steps.append({
                    "trace_id": len(steps), "iteration": decision,
                    "event": termination_reason,
                    "current_pose": [current[0], current[1], heading],
                    "selected_frontier": None, "path": [],
                    "translation_m": 0.0, "rotation_deg": 0.0,
                    "explored_nodes": [dict(node) for node in nodes],
                    "generated_candidates": traced,
                    "new_frontiers": new_frontiers,
                    "active_frontiers": active, "observed_updates": [],
                    "scan_poses": [], "visible_cell_count": 0,
                    "new_observed_count": 0,
                    "coverage_before": before["free_coverage"],
                    "coverage_after": before["free_coverage"],
                    "coverage_gain": 0.0,
                    "known_ratio": before["known_ratio"],
                    "occupied_recall": before["occupied_recall"],
                    "sensor": {
                        "fov_deg": self.config.sensor.field_of_view_deg,
                        "range_m": self.config.sensor.max_range,
                    },
                })
                break
            path = self._plan(planner, current, selected, truth.data.size)
            if path is None:
                selected = dict(selected)
                selected["status"] = "unreachable"
                stagnant_steps += 1
                steps.append({
                    "trace_id": len(steps), "iteration": decision,
                    "event": "selected_unreachable",
                    "current_pose": [current[0], current[1], heading],
                    "selected_frontier": selected, "path": [],
                    "translation_m": 0.0, "rotation_deg": 0.0,
                    "explored_nodes": [dict(node) for node in nodes],
                    "generated_candidates": traced,
                    "new_frontiers": new_frontiers,
                    "active_frontiers": active, "observed_updates": [],
                    "scan_poses": [], "visible_cell_count": 0,
                    "new_observed_count": 0,
                    "coverage_before": before["free_coverage"],
                    "coverage_after": before["free_coverage"],
                    "coverage_gain": 0.0,
                    "known_ratio": before["known_ratio"],
                    "occupied_recall": before["occupied_recall"],
                    "sensor": {
                        "fov_deg": self.config.sensor.field_of_view_deg,
                        "range_m": self.config.sensor.max_range,
                    },
                })
                if stagnant_steps >= 3:
                    termination_reason = "repeated_unreachable"
                    break
                continue

            travel = planner.get_path_length(path)
            rotation = self._execution_rotation(
                path, heading, selected["heading"]
            )
            if travel < belief.resolution:
                in_place_rotations += 1
            total_distance += travel
            total_rotation += rotation
            updates, scan_poses, visible_count, new_count = self._scan_path(
                truth, belief, path, selected["heading"]
            )
            scan_count += len(scan_poses)
            current = tuple(map(float, selected["target"]))
            heading = float(selected["heading"] % 360.0)
            positions.append(current)
            paths.append(path)
            nodes.append({
                "id": len(nodes), "position": current,
                "orientation": heading, "timestamp": decision,
            })
            after = self._coverage(truth, belief)
            stagnant_steps = stagnant_steps + 1 if new_count == 0 else 0
            selected = dict(selected)
            selected["status"] = "selected"
            steps.append({
                "trace_id": len(steps), "iteration": decision,
                "event": "viewpoint_accepted",
                "current_pose": [current[0], current[1], heading],
                "selected_frontier": selected, "path": path,
                "translation_m": float(travel),
                "rotation_deg": float(rotation),
                "explored_nodes": [dict(node) for node in nodes],
                "generated_candidates": traced,
                "new_frontiers": new_frontiers,
                "active_frontiers": active,
                "observed_updates": updates, "scan_poses": scan_poses,
                "visible_cell_count": int(visible_count),
                "new_observed_count": int(new_count),
                "coverage_before": before["free_coverage"],
                "coverage_after": after["free_coverage"],
                "coverage_gain": after["free_coverage"] - before["free_coverage"],
                "known_ratio": after["known_ratio"],
                "occupied_recall": after["occupied_recall"],
                "sensor": {
                    "fov_deg": self.config.sensor.field_of_view_deg,
                    "range_m": self.config.sensor.max_range,
                },
            })
            if self.config.verbose:
                print(
                    f"[{self.name}] step={decision} coverage={after['free_coverage']:.1%} "
                    f"new={new_count} distance={total_distance:.1f}m"
                )
            if stagnant_steps >= 5:
                termination_reason = "stagnation"
                break
        else:
            termination_reason = "max_decisions"

        final = self._coverage(truth, belief)
        if final["free_coverage"] >= self.config.target_coverage:
            termination_reason = "coverage_target"
        elapsed = time.perf_counter() - started
        return {
            "nodes": nodes,
            "metadata": {
                "protocol": "unknown_static_grid_occlusion_aware",
                "strategy": self.config.strategy,
                "sensor_fov_deg": self.config.sensor.field_of_view_deg,
                "sensor_range_m": self.config.sensor.max_range,
                "angular_resolution_deg": self.config.sensor.angular_resolution_deg,
                "coverage_ratio": final["free_coverage"],
                "known_ratio": final["known_ratio"],
                "occupied_recall": final["occupied_recall"],
                "total_distance": total_distance,
                "total_rotation_deg": total_rotation,
                "num_nodes": len(nodes), "scan_count": scan_count,
                "in_place_rotations": in_place_rotations,
                "total_time": elapsed,
                "termination_reason": termination_reason,
                "paths": paths,
            },
            "steps": steps,
            "paths": paths,
            "belief_final": belief.data,
            "success": final["free_coverage"] >= self.config.target_coverage,
            "algorithm": self.name,
        }
