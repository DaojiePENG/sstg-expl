"""Adapter for the pretrained Active Neural SLAM global exploration policy.

This uses the released ICLR 2020 global-policy checkpoint while replacing the
Habitat RGB mapper and local controller with the common occupancy-grid map and
A* planner used by this repository. Results are therefore labelled
``ANS-Global (adapted)`` rather than claimed as the full published ANS system.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.ndimage import label

from .base_explorer import BaseExplorer
from ..core.coverage_analyzer import CoverageAnalyzer
from ..map.occupancy_grid import OccupancyGrid
from ..planning.astar import AStarPlanner

try:
    import torch
    import torch.nn as nn
except ImportError:  # Optional dependency; fail with an actionable message.
    torch = None
    nn = None


if nn is not None:
    class _Flatten(nn.Module):
        def forward(self, value):
            return value.reshape(value.size(0), -1)


    class _AddBias(nn.Module):
        def __init__(self, bias):
            super().__init__()
            self._bias = nn.Parameter(bias.unsqueeze(1))

        def forward(self, value):
            return value + self._bias.t().view(1, -1)


    class _ANSNetwork(nn.Module):
        def __init__(self):
            super().__init__()
            self.main = nn.Sequential(
                nn.MaxPool2d(2),
                nn.Conv2d(8, 32, 3, padding=1), nn.ReLU(),
                nn.MaxPool2d(2),
                nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(),
                nn.MaxPool2d(2),
                nn.Conv2d(64, 128, 3, padding=1), nn.ReLU(),
                nn.MaxPool2d(2),
                nn.Conv2d(128, 64, 3, padding=1), nn.ReLU(),
                nn.Conv2d(64, 32, 3, padding=1), nn.ReLU(),
                _Flatten(),
            )
            self.linear1 = nn.Linear(7208, 256)
            self.linear2 = nn.Linear(256, 256)
            self.critic_linear = nn.Linear(256, 1)
            self.orientation_emb = nn.Embedding(72, 8)

        def forward(self, inputs, orientation):
            features = self.main(inputs)
            embedding = self.orientation_emb(orientation).squeeze(1)
            features = torch.cat((features, embedding), dim=1)
            features = torch.relu(self.linear1(features))
            return torch.relu(self.linear2(features))


    class _DiagGaussian(nn.Module):
        def __init__(self):
            super().__init__()
            self.fc_mean = nn.Linear(256, 2)
            self.logstd = _AddBias(torch.zeros(2))


    class _ANSPolicy(nn.Module):
        def __init__(self):
            super().__init__()
            self.network = _ANSNetwork()
            self.dist = _DiagGaussian()

        def deterministic_goal(self, inputs, orientation):
            features = self.network(inputs, orientation)
            return torch.sigmoid(self.dist.fc_mean(features))


class ActiveNeuralSLAMExplorer(BaseExplorer):
    """Pretrained ANS global policy adapted to the common grid protocol."""

    requires_occupancy_grid = True
    map_cells = 480
    local_cells = 240

    def __init__(
        self,
        checkpoint: str,
        r_view: float = 2.0,
        target_coverage: float = 0.95,
        r_robot: float = 0.3,
        d_safe: float = 0.2,
        max_iterations: int = 200,
    ):
        super().__init__("ANS-Global (adapted)")
        if torch is None:
            raise ImportError(
                "ANS-Global requires PyTorch. Install the 'learning' extra or "
                "run scripts/setup_learning_baselines.py."
            )
        checkpoint_path = Path(checkpoint).expanduser().resolve()
        if not checkpoint_path.exists():
            raise FileNotFoundError(
                f"ANS checkpoint not found: {checkpoint_path}. Run "
                "scripts/setup_learning_baselines.py first."
            )
        torch.set_num_threads(1)
        self.policy = _ANSPolicy()
        state = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        self.policy.load_state_dict(state)
        self.policy.eval()
        self.checkpoint = str(checkpoint_path)
        self.r_view = r_view
        self.target_coverage = target_coverage
        self.r_robot = r_robot
        self.d_safe = d_safe
        self.max_iterations = max_iterations

    @staticmethod
    def _crop(array: np.ndarray, center: Tuple[int, int], size: int) -> np.ndarray:
        """Fixed-size crop with zero padding."""
        channels, height, width = array.shape
        half = size // 2
        top, left = center[0] - half, center[1] - half
        result = np.zeros((channels, size, size), dtype=np.float32)
        src_t, src_l = max(0, top), max(0, left)
        src_b, src_r = min(height, top + size), min(width, left + size)
        dst_t, dst_l = src_t - top, src_l - left
        result[:, dst_t:dst_t + src_b - src_t, dst_l:dst_l + src_r - src_l] = \
            array[:, src_t:src_b, src_l:src_r]
        return result

    def explore(
        self,
        occupancy_grid: OccupancyGrid,
        start_pose: Tuple[float, float, float],
        visualizer: Optional[object] = None,
    ) -> Dict:
        started = time.perf_counter()
        analyzer = CoverageAnalyzer(occupancy_grid)
        planner = AStarPlanner(occupancy_grid, self.r_robot, self.d_safe)
        safe = planner.planning_grid.get_free_space_mask()
        start_cell = occupancy_grid.world_to_grid(start_pose[0], start_pose[1])
        components, _ = label(safe)
        reachable = components == components[start_cell]
        nodes = [(float(start_pose[0]), float(start_pose[1]))]
        paths = []
        steps = []
        total_distance = 0.0

        # Center arbitrary benchmark maps in the 24 m × 24 m ANS canvas.
        offset_r = (self.map_cells - occupancy_grid.height) // 2
        offset_c = (self.map_cells - occupancy_grid.width) // 2
        if offset_r < 0 or offset_c < 0:
            raise ValueError("ANS adapter supports maps no larger than 480×480 cells")

        for iteration in range(self.max_iterations):
            covered = analyzer.compute_coverage_map(nodes, self.r_view)
            coverage_before = analyzer.compute_coverage_ratio(nodes, self.r_view)
            if coverage_before >= self.target_coverage:
                break
            current_cell = occupancy_grid.world_to_grid(*nodes[-1])

            full = np.zeros((4, self.map_cells, self.map_cells), dtype=np.float32)
            region = (slice(offset_r, offset_r + occupancy_grid.height),
                      slice(offset_c, offset_c + occupancy_grid.width))
            full[0][region] = occupancy_grid.get_occupied_mask() & covered
            full[1][region] = covered
            full[3][region] = analyzer.compute_coverage_map(nodes[:-1], self.r_view) if len(nodes) > 1 else 0
            cr, cc = current_cell[0] + offset_r, current_cell[1] + offset_c
            full[2, max(0, cr-1):cr+2, max(0, cc-1):cc+2] = 1.0
            local = self._crop(full, (cr, cc), self.local_cells)
            pooled = full.reshape(4, self.local_cells, 2, self.local_cells, 2).max(axis=(2, 4))
            policy_input = np.concatenate((local, pooled), axis=0)
            orientation = int(((start_pose[2] + 180.0) % 360.0) / 5.0)
            with torch.no_grad():
                action = self.policy.deterministic_goal(
                    torch.from_numpy(policy_input[None]),
                    torch.tensor([[orientation]], dtype=torch.long),
                )[0].numpy()

            predicted_local = np.clip((action * (self.local_cells - 1)).astype(int), 0, self.local_cells - 1)
            predicted_canvas = (
                cr - self.local_cells // 2 + int(predicted_local[0]),
                cc - self.local_cells // 2 + int(predicted_local[1]),
            )
            predicted_grid = (
                predicted_canvas[0] - offset_r,
                predicted_canvas[1] - offset_c,
            )

            available = reachable & safe & (~covered)
            if not np.any(available):
                break
            candidates = np.argwhere(available)
            distances = np.sum((candidates - np.asarray(predicted_grid)) ** 2, axis=1)
            ordered = candidates[np.argsort(distances)]
            path = None
            goal = None
            for row, col in ordered[:200]:
                point = occupancy_grid.grid_to_world(int(row), int(col))
                candidate_path = planner.plan(
                    nodes[-1], point,
                    max_iterations=max(occupancy_grid.data.size, 10000),
                )
                if candidate_path is not None:
                    goal, path = point, candidate_path
                    break
            if path is None or goal is None:
                break

            path_length = planner.get_path_length(path)
            total_distance += path_length
            nodes.append(goal)
            paths.append(path)
            coverage_after = analyzer.compute_coverage_ratio(nodes, self.r_view)
            steps.append({
                'trace_id': iteration,
                'iteration': iteration + 1,
                'event': 'ans_global_goal',
                'current_position': goal,
                'selected_frontier': {
                    'frontier_id': iteration,
                    'origin': nodes[-2], 'target': goal, 'angle': 0.0,
                    'priority': 1.0, 'kind': 'learned_global_goal',
                },
                'path': path,
                'executed_paths': [list(segment) for segment in paths],
                'explored_nodes': list(nodes),
                'generated_candidates': [{
                    'target': goal, 'origin': nodes[-2], 'angle': 0.0,
                    'kind': 'learned_global_goal', 'status': 'added',
                    'policy_action': action.tolist(),
                    'raw_predicted_grid': predicted_grid,
                }],
                'new_frontiers': [], 'active_frontiers': [],
                'coverage_before': coverage_before,
                'coverage_after': coverage_after,
                'coverage_gain': coverage_after - coverage_before,
                'queue_size': 0, 'recovery_round': 0,
            })

        coverage = analyzer.compute_coverage_ratio(nodes, self.r_view)
        self.nodes = nodes
        self.total_distance = total_distance
        self.coverage_ratio = coverage
        return {
            'nodes': [
                {'id': index, 'position': point, 'orientation': 0.0, 'timestamp': index}
                for index, point in enumerate(nodes)
            ],
            'metadata': {
                'coverage_ratio': coverage,
                'total_distance': total_distance,
                'num_nodes': len(nodes),
                'total_time': time.perf_counter() - started,
                'checkpoint': self.checkpoint,
                'adapter': 'released ANS global policy + common occupancy/A* protocol',
                'paths': paths,
            },
            'steps': steps,
            'success': coverage >= self.target_coverage,
            'algorithm': self.name,
        }
