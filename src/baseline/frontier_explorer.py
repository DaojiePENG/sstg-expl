"""
Frontier-based Exploration - Classic baseline algorithm.

Reference:
    Yamauchi, B. (1997). "A frontier-based approach for autonomous exploration."
    In Proceedings 1997 IEEE International Symposium on Computational
    Intelligence in Robotics and Automation CIRA'97.

    Yamauchi, B. (1998). "Frontier-based exploration using multiple robots."
    In Proceedings of the second international conference on Autonomous agents
    (pp. 47-53).

This is the classic frontier-based exploration algorithm that selects
the nearest frontier (boundary between explored and unexplored regions)
as the next exploration target.
"""

import time
from typing import Dict, List, Optional, Tuple
import numpy as np
from scipy import ndimage

from .base_explorer import BaseExplorer
from ..map.occupancy_grid import OccupancyGrid


class FrontierExplorer(BaseExplorer):
    """
    Classic frontier-based exploration algorithm.

    Repeatedly selects the nearest frontier point (boundary between
    explored free space and unexplored unknown space) as the next
    exploration target until target coverage is achieved.

    Args:
        r_view: Sensor view radius (meters)
        target_coverage: Target coverage ratio to achieve
        max_iterations: Maximum exploration iterations
        frontier_min_size: Minimum frontier cluster size (cells)
    """

    def __init__(
        self,
        r_view: float = 2.0,
        target_coverage: float = 0.95,
        max_iterations: int = 1000,
        frontier_min_size: int = 5,
        **kwargs
    ):
        """Initialize Frontier Explorer."""
        config = {
            'r_view': r_view,
            'target_coverage': target_coverage,
            'max_iterations': max_iterations,
            'frontier_min_size': frontier_min_size,
        }
        config.update(kwargs)
        super().__init__(name='Frontier', config=config)

        self.r_view = r_view
        self.target_coverage = target_coverage
        self.max_iterations = max_iterations
        self.frontier_min_size = frontier_min_size

    def explore(
        self,
        occupancy_grid: np.ndarray,
        start_pose: Tuple[float, float, float],
        visualizer: Optional[object] = None
    ) -> Dict:
        """
        Explore using frontier-based algorithm.

        Args:
            occupancy_grid: 2D occupancy grid
            start_pose: Starting pose (x, y, theta)
            visualizer: Optional visualizer

        Returns:
            Exploration results dictionary
        """
        start_time = time.time()
        self.reset()

        # Create OccupancyGrid wrapper
        grid = OccupancyGrid(occupancy_grid, resolution=0.05)

        # Initialize with start position
        current_pos = np.array(start_pose[:2])

        # Initialize coverage and explored maps
        coverage_map = np.zeros_like(occupancy_grid, dtype=bool)
        explored_map = np.zeros_like(occupancy_grid, dtype=bool)

        # Mark initial coverage
        self._mark_coverage(coverage_map, explored_map, grid, current_pos, self.r_view)

        # Record start node
        self.nodes.append({
            'position': tuple(current_pos),
            'theta': start_pose[2],
            'coverage_area': np.sum(coverage_map) * (grid.resolution ** 2)
        })

        # Frontier exploration loop
        for iteration in range(self.max_iterations):
            # Detect frontiers
            frontiers = self._detect_frontiers(
                explored_map, grid, self.frontier_min_size
            )

            if len(frontiers) == 0:
                # No more frontiers, exploration complete
                break

            # Select nearest frontier
            frontier_pos = self._select_nearest_frontier(frontiers, current_pos)

            if frontier_pos is None:
                break

            # Travel to frontier
            travel_dist = np.linalg.norm(frontier_pos - current_pos)
            self.total_distance += travel_dist
            current_pos = frontier_pos

            # Mark coverage from new position
            self._mark_coverage(coverage_map, explored_map, grid, current_pos, self.r_view)

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
                    frontiers=[{'position': tuple(f), 'priority': 1.0} for f in frontiers],
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

    def _detect_frontiers(
        self,
        explored_map: np.ndarray,
        grid: OccupancyGrid,
        min_size: int = 5
    ) -> List[np.ndarray]:
        """
        Detect frontier points (boundary of explored region).

        A frontier cell is:
        - Free space (not occupied)
        - Not yet explored
        - Adjacent to explored free space

        Args:
            explored_map: Boolean map of explored cells
            grid: OccupancyGrid object
            min_size: Minimum frontier cluster size

        Returns:
            List of frontier positions (x, y) in world coordinates
        """
        free_mask = grid.data < 50
        unexplored_mask = ~explored_map

        # Frontier candidates: free AND unexplored
        frontier_candidates = free_mask & unexplored_mask

        # Check adjacency to explored cells
        # Dilate explored map to find neighbors
        explored_dilated = ndimage.binary_dilation(explored_map, iterations=1)

        # Frontier: unexplored free cells adjacent to explored cells
        frontier_mask = frontier_candidates & explored_dilated

        # Cluster frontiers and filter by size
        labeled, num_clusters = ndimage.label(frontier_mask)

        frontiers = []
        for cluster_id in range(1, num_clusters + 1):
            cluster_mask = labeled == cluster_id
            cluster_size = np.sum(cluster_mask)

            if cluster_size >= min_size:
                # Find centroid of cluster
                indices = np.argwhere(cluster_mask)
                centroid_idx = np.mean(indices, axis=0).astype(int)
                row, col = centroid_idx
                x, y = grid.grid_to_world(row, col)
                frontiers.append(np.array([x, y]))

        return frontiers

    def _select_nearest_frontier(
        self,
        frontiers: List[np.ndarray],
        current_pos: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        Select the nearest frontier to current position.

        Args:
            frontiers: List of frontier positions
            current_pos: Current position

        Returns:
            Selected frontier position or None
        """
        if len(frontiers) == 0:
            return None

        distances = [np.linalg.norm(f - current_pos) for f in frontiers]
        nearest_idx = np.argmin(distances)
        return frontiers[nearest_idx]

    def _mark_coverage(
        self,
        coverage_map: np.ndarray,
        explored_map: np.ndarray,
        grid: OccupancyGrid,
        center: np.ndarray,
        radius: float
    ):
        """
        Mark circular coverage and explored regions.

        Args:
            coverage_map: Boolean coverage map (free space covered)
            explored_map: Boolean explored map (all cells in sensor range)
            grid: OccupancyGrid object
            center: Center position (x, y)
            radius: Sensor radius
        """
        cx, cy = center
        center_row, center_col = grid.world_to_grid(cx, cy)
        radius_cells = int(radius / grid.resolution)

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr**2 + dc**2 <= radius_cells**2:
                    r = center_row + dr
                    c = center_col + dc
                    if 0 <= r < grid.height and 0 <= c < grid.width:
                        # Mark as explored (regardless of occupancy)
                        explored_map[r, c] = True
                        # Mark coverage only for free space
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
