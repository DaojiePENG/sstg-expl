"""
RRT-based Explorer - Baseline exploration algorithm.

Reference:
    LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for
    path planning." Technical Report TR 98-11, Computer Science Dept.,
    Iowa State University.

    Umari, H., & Mukhopadhyay, S. (2017). "Autonomous robotic exploration
    based on multiple rapidly-exploring randomized trees." In IEEE/RSJ
    International Conference on Intelligent Robots and Systems (IROS).

This algorithm uses RRT to incrementally explore the environment by
randomly sampling points and extending the tree towards unexplored regions.
"""

import time
from typing import Dict, List, Optional, Tuple
import numpy as np

from .base_explorer import BaseExplorer
from ..map.occupancy_grid import OccupancyGrid


class RRTExplorer(BaseExplorer):
    """
    RRT-based exploration algorithm.

    Uses Rapidly-exploring Random Trees to incrementally explore the
    environment. The tree is grown by sampling random points and extending
    branches towards them, naturally biasing towards unexplored regions.

    Args:
        max_iterations: Maximum number of RRT iterations
        step_size: Maximum extension distance per iteration
        r_view: Sensor view radius (meters)
        goal_sample_rate: Probability of sampling towards frontiers
        target_coverage: Target coverage ratio to achieve
    """

    def __init__(
        self,
        max_iterations: int = 1000,
        step_size: float = 1.0,
        r_view: float = 2.0,
        goal_sample_rate: float = 0.1,
        target_coverage: float = 0.95,
        **kwargs
    ):
        """Initialize RRT Explorer."""
        config = {
            'max_iterations': max_iterations,
            'step_size': step_size,
            'r_view': r_view,
            'goal_sample_rate': goal_sample_rate,
            'target_coverage': target_coverage,
        }
        config.update(kwargs)
        super().__init__(name='RRT', config=config)

        self.max_iterations = max_iterations
        self.step_size = step_size
        self.r_view = r_view
        self.goal_sample_rate = goal_sample_rate
        self.target_coverage = target_coverage

        self.tree_nodes = []  # RRT tree nodes
        self.tree_parents = []  # Parent indices

    def explore(
        self,
        occupancy_grid: np.ndarray,
        start_pose: Tuple[float, float, float],
        visualizer: Optional[object] = None
    ) -> Dict:
        """
        Explore using RRT-based algorithm.

        Args:
            occupancy_grid: 2D occupancy grid
            start_pose: Starting pose (x, y, theta)
            visualizer: Optional visualizer

        Returns:
            Exploration results dictionary
        """
        start_time = time.time()
        self.reset()

        # Get OccupancyGrid object
        if isinstance(occupancy_grid, OccupancyGrid):
            grid = occupancy_grid
        else:
            # occupancy_grid is numpy array
            grid = OccupancyGrid(data=occupancy_grid, resolution=0.05, origin=(0.0, 0.0))

        # Initialize tree with start node
        start_pos = np.array(start_pose[:2])
        self.tree_nodes = [start_pos]
        self.tree_parents = [-1]  # Root has no parent

        # Initialize coverage map
        coverage_map = np.zeros_like(occupancy_grid, dtype=bool)
        self._mark_coverage(coverage_map, grid, start_pos, self.r_view)

        # Record start node
        self.nodes.append({
            'position': tuple(start_pos),
            'theta': start_pose[2],
            'coverage_area': np.sum(coverage_map) * (grid.resolution ** 2)
        })

        # RRT iterations
        for iteration in range(self.max_iterations):
            # Sample random point
            if np.random.random() < self.goal_sample_rate:
                # Sample towards frontier (unexplored boundary)
                sample = self._sample_frontier(coverage_map, grid)
            else:
                # Uniform random sample
                sample = self._sample_free_space(grid)

            if sample is None:
                continue

            # Find nearest node in tree
            nearest_idx = self._nearest_node(sample)
            nearest = self.tree_nodes[nearest_idx]

            # Extend towards sample
            new_node = self._extend(nearest, sample, self.step_size)

            # Check collision
            if not self._is_collision_free(grid, nearest, new_node):
                continue

            # Add to tree
            self.tree_nodes.append(new_node)
            self.tree_parents.append(nearest_idx)

            # Update travel distance
            dist = np.linalg.norm(new_node - nearest)
            self.total_distance += dist

            # Mark coverage
            old_coverage = np.sum(coverage_map)
            self._mark_coverage(coverage_map, grid, new_node, self.r_view)
            new_coverage = np.sum(coverage_map)

            # Record node if it added new coverage
            if new_coverage > old_coverage:
                self.nodes.append({
                    'position': tuple(new_node),
                    'theta': 0.0,
                    'coverage_area': new_coverage * (grid.resolution ** 2)
                })

                # Visualize if available
                if visualizer is not None:
                    self.coverage_ratio = self._compute_coverage_ratio(
                        coverage_map, grid
                    )
                    visualizer.update(
                        current_position=tuple(new_node) + (0.0,),
                        nodes=self.nodes,
                        frontiers=[],
                        coverage_ratio=self.coverage_ratio
                    )

            # Check termination
            self.coverage_ratio = self._compute_coverage_ratio(
                coverage_map, grid
            )
            if self.coverage_ratio >= self.target_coverage:
                break

        computation_time = time.time() - start_time

        return {
            'nodes': self.nodes,
            'metadata': {
                'total_distance': self.total_distance,
                'coverage_ratio': self.coverage_ratio,
                'num_nodes': len(self.nodes),
                'computation_time': computation_time,
                'success': True,
                'iterations': iteration + 1
            },
            'success': True,
            'algorithm': self.name
        }

    def _sample_free_space(self, grid: OccupancyGrid) -> Optional[np.ndarray]:
        """
        Sample random point in free space.

        Args:
            grid: OccupancyGrid object

        Returns:
            Sampled point (x, y) or None if sampling fails
        """
        max_attempts = 50
        for _ in range(max_attempts):
            x = np.random.uniform(0, grid.width * grid.resolution)
            y = np.random.uniform(0, grid.height * grid.resolution)
            row, col = grid.world_to_grid(x, y)
            if (0 <= row < grid.height and 0 <= col < grid.width and
                grid.data[row, col] < 50):
                return np.array([x, y])
        return None

    def _sample_frontier(
        self,
        coverage_map: np.ndarray,
        grid: OccupancyGrid
    ) -> Optional[np.ndarray]:
        """
        Sample point near coverage frontier.

        Args:
            coverage_map: Boolean coverage map
            grid: OccupancyGrid object

        Returns:
            Sampled frontier point or None
        """
        # Find frontier cells (free, uncovered, adjacent to covered)
        free_mask = grid.data < 50
        uncovered_mask = ~coverage_map

        # Simple frontier: uncovered free cells
        frontier_mask = free_mask & uncovered_mask

        frontier_indices = np.argwhere(frontier_mask)
        if len(frontier_indices) == 0:
            return self._sample_free_space(grid)

        # Randomly select a frontier cell
        idx = np.random.randint(len(frontier_indices))
        row, col = frontier_indices[idx]
        x, y = grid.grid_to_world(row, col)

        return np.array([x, y])

    def _nearest_node(self, point: np.ndarray) -> int:
        """
        Find nearest node in tree to given point.

        Args:
            point: Query point (x, y)

        Returns:
            Index of nearest node
        """
        distances = [np.linalg.norm(node - point) for node in self.tree_nodes]
        return int(np.argmin(distances))

    def _extend(
        self,
        from_node: np.ndarray,
        to_point: np.ndarray,
        max_dist: float
    ) -> np.ndarray:
        """
        Extend from node towards point by at most max_dist.

        Args:
            from_node: Starting node
            to_point: Target point
            max_dist: Maximum extension distance

        Returns:
            New node position
        """
        direction = to_point - from_node
        distance = np.linalg.norm(direction)

        if distance <= max_dist:
            return to_point
        else:
            return from_node + (direction / distance) * max_dist

    def _is_collision_free(
        self,
        grid: OccupancyGrid,
        from_pos: np.ndarray,
        to_pos: np.ndarray,
        num_checks: int = 10
    ) -> bool:
        """
        Check if path from from_pos to to_pos is collision-free.

        Args:
            grid: OccupancyGrid object
            from_pos: Start position
            to_pos: End position
            num_checks: Number of intermediate points to check

        Returns:
            True if collision-free, False otherwise
        """
        for i in range(num_checks + 1):
            t = i / num_checks
            pos = from_pos * (1 - t) + to_pos * t
            row, col = grid.world_to_grid(pos[0], pos[1])

            if not (0 <= row < grid.height and 0 <= col < grid.width):
                return False
            if grid.data[row, col] >= 50:  # Obstacle
                return False

        return True

    def _mark_coverage(
        self,
        coverage_map: np.ndarray,
        grid: OccupancyGrid,
        center: np.ndarray,
        radius: float
    ):
        """Mark circular coverage region on coverage map."""
        cx, cy = center
        center_row, center_col = grid.world_to_grid(cx, cy)
        radius_cells = int(radius / grid.resolution)

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr**2 + dc**2 <= radius_cells**2:
                    r = center_row + dr
                    c = center_col + dc
                    if 0 <= r < grid.height and 0 <= c < grid.width:
                        if grid.data[r, c] < 50:
                            coverage_map[r, c] = True

    def _compute_coverage_ratio(
        self,
        coverage_map: np.ndarray,
        grid: OccupancyGrid
    ) -> float:
        """Compute coverage ratio."""
        free_space = (grid.data < 50).sum()
        covered = coverage_map.sum()
        return covered / max(free_space, 1)

    def reset(self):
        """Reset explorer state."""
        super().reset()
        self.tree_nodes = []
        self.tree_parents = []
