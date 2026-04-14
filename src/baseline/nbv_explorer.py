"""
Next-Best-View (NBV) Explorer - Information-gain based exploration.

Reference:
    Connolly, C. (1985). "The determination of next best views."
    In Proceedings. 1985 IEEE international conference on robotics and
    automation (Vol. 2, pp. 432-435).

    Gonzalez-Banos, H. H., & Latombe, J. C. (2002). "Navigation strategies
    for exploring indoor environments." The International Journal of
    Robotics Research, 21(10-11), 829-848.

    Bircher, A., et al. (2016). "Structural inspection path planning via
    iterative viewpoint resampling with application to aerial robotics."
    In 2016 IEEE International Conference on Robotics and Automation (ICRA).

This algorithm selects the next viewpoint based on information gain,
balancing between exploration benefit and travel cost.
"""

import time
from typing import Dict, List, Optional, Tuple
import numpy as np

from .base_explorer import BaseExplorer
from ..map.occupancy_grid import OccupancyGrid


class NextBestViewExplorer(BaseExplorer):
    """
    Next-Best-View (NBV) exploration algorithm.

    Selects the next viewpoint by maximizing information gain (expected
    new coverage area) while penalizing travel distance. This balances
    between exploration efficiency and travel cost.

    Args:
        r_view: Sensor view radius (meters)
        n_candidates: Number of candidate viewpoints to sample
        target_coverage: Target coverage ratio to achieve
        max_iterations: Maximum exploration iterations
        info_weight: Weight for information gain term
        dist_weight: Weight for distance cost term
    """

    def __init__(
        self,
        r_view: float = 2.0,
        n_candidates: int = 50,
        target_coverage: float = 0.95,
        max_iterations: int = 1000,
        info_weight: float = 1.0,
        dist_weight: float = 0.1,
        **kwargs
    ):
        """Initialize NBV Explorer."""
        config = {
            'r_view': r_view,
            'n_candidates': n_candidates,
            'target_coverage': target_coverage,
            'max_iterations': max_iterations,
            'info_weight': info_weight,
            'dist_weight': dist_weight,
        }
        config.update(kwargs)
        super().__init__(name='NBV', config=config)

        self.r_view = r_view
        self.n_candidates = n_candidates
        self.target_coverage = target_coverage
        self.max_iterations = max_iterations
        self.info_weight = info_weight
        self.dist_weight = dist_weight

    def explore(
        self,
        occupancy_grid: np.ndarray,
        start_pose: Tuple[float, float, float],
        visualizer: Optional[object] = None
    ) -> Dict:
        """
        Explore using Next-Best-View algorithm.

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

        # Initialize with start position
        current_pos = np.array(start_pose[:2])

        # Initialize coverage map
        coverage_map = np.zeros_like(occupancy_grid, dtype=bool)
        self._mark_coverage(coverage_map, grid, current_pos, self.r_view)

        # Record start node
        self.nodes.append({
            'position': tuple(current_pos),
            'theta': start_pose[2],
            'coverage_area': np.sum(coverage_map) * (grid.resolution ** 2)
        })

        # NBV exploration loop
        for iteration in range(self.max_iterations):
            # Generate candidate viewpoints
            candidates = self._generate_candidates(coverage_map, grid, self.n_candidates)

            if len(candidates) == 0:
                # No more candidates, exploration complete
                break

            # Evaluate candidates and select best
            best_candidate = self._select_best_view(
                candidates, current_pos, coverage_map, grid
            )

            if best_candidate is None:
                break

            # Travel to best viewpoint
            travel_dist = np.linalg.norm(best_candidate - current_pos)
            self.total_distance += travel_dist
            current_pos = best_candidate

            # Mark coverage from new position
            self._mark_coverage(coverage_map, grid, current_pos, self.r_view)

            # Record node
            self.nodes.append({
                'position': tuple(current_pos),
                'theta': 0.0,
                'coverage_area': np.sum(coverage_map) * (grid.resolution ** 2)
            })

            # Visualize if available
            if visualizer is not None:
                self.coverage_ratio = self._compute_coverage_ratio(coverage_map, grid)
                visualizer.update(
                    current_position=tuple(current_pos) + (0.0,),
                    nodes=self.nodes,
                    frontiers=[],
                    coverage_ratio=self.coverage_ratio
                )

            # Check termination
            self.coverage_ratio = self._compute_coverage_ratio(coverage_map, grid)
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

    def _generate_candidates(
        self,
        coverage_map: np.ndarray,
        grid: OccupancyGrid,
        n_candidates: int
    ) -> List[np.ndarray]:
        """
        Generate candidate viewpoints in uncovered free space.

        Args:
            coverage_map: Boolean coverage map
            grid: OccupancyGrid object
            n_candidates: Number of candidates to generate

        Returns:
            List of candidate positions (x, y)
        """
        free_mask = grid.data < 50
        uncovered_mask = ~coverage_map

        # Find uncovered free cells
        uncovered_free = free_mask & uncovered_mask
        indices = np.argwhere(uncovered_free)

        if len(indices) == 0:
            return []

        # Randomly sample candidates
        n_samples = min(n_candidates, len(indices))
        sampled_indices = indices[np.random.choice(
            len(indices), size=n_samples, replace=False
        )]

        candidates = []
        for row, col in sampled_indices:
            x, y = grid.grid_to_world(row, col)
            candidates.append(np.array([x, y]))

        return candidates

    def _select_best_view(
        self,
        candidates: List[np.ndarray],
        current_pos: np.ndarray,
        coverage_map: np.ndarray,
        grid: OccupancyGrid
    ) -> Optional[np.ndarray]:
        """
        Select best viewpoint based on information gain vs. travel cost.

        The utility function is:
            U(v) = info_weight * IG(v) - dist_weight * dist(current, v)

        where IG(v) is the expected information gain (new coverage area).

        Args:
            candidates: List of candidate positions
            current_pos: Current position
            coverage_map: Current coverage map
            grid: OccupancyGrid object

        Returns:
            Best candidate position or None
        """
        if len(candidates) == 0:
            return None

        best_utility = -np.inf
        best_candidate = None

        for candidate in candidates:
            # Compute information gain (expected new coverage)
            info_gain = self._compute_information_gain(
                candidate, coverage_map, grid, self.r_view
            )

            # Compute travel distance
            distance = np.linalg.norm(candidate - current_pos)

            # Compute utility
            utility = self.info_weight * info_gain - self.dist_weight * distance

            if utility > best_utility:
                best_utility = utility
                best_candidate = candidate

        return best_candidate

    def _compute_information_gain(
        self,
        position: np.ndarray,
        coverage_map: np.ndarray,
        grid: OccupancyGrid,
        radius: float
    ) -> float:
        """
        Compute expected information gain from viewpoint.

        Information gain is the number of new free cells that would be
        covered from this viewpoint.

        Args:
            position: Candidate position (x, y)
            coverage_map: Current coverage map
            grid: OccupancyGrid object
            radius: Sensor radius

        Returns:
            Information gain (number of new cells)
        """
        px, py = position
        center_row, center_col = grid.world_to_grid(px, py)
        radius_cells = int(radius / grid.resolution)

        new_cells = 0

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr**2 + dc**2 <= radius_cells**2:
                    r = center_row + dr
                    c = center_col + dc
                    if 0 <= r < grid.height and 0 <= c < grid.width:
                        # Check if this cell would add new coverage
                        if grid.data[r, c] < 50 and not coverage_map[r, c]:
                            new_cells += 1

        return float(new_cells)

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
