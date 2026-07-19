"""
Uniform Grid Sampling - Baseline exploration algorithm.

Reference:
    Choset, H. (2001). "Coverage for robotics–A survey of recent results."
    Annals of mathematics and artificial intelligence, 31(1-4), 113-126.

This is the simplest baseline: sample the environment on a uniform grid
with fixed spacing, visiting each grid point that is in free space.
"""

import time
from typing import Dict, List, Optional, Tuple
import numpy as np
from scipy.spatial import cKDTree

from .base_explorer import BaseExplorer
from ..map.occupancy_grid import OccupancyGrid
from ..planning.astar import AStarPlanner


class UniformGridExplorer(BaseExplorer):
    """
    Uniform grid sampling exploration algorithm.

    Samples the environment on a regular grid with specified spacing.
    Nodes are visited in a systematic order (row-by-row or column-by-column).

    This serves as a simple baseline that provides predictable coverage
    but does not optimize for travel distance.

    Args:
        grid_spacing: Distance between grid points (meters)
        r_view: Sensor view radius (meters)
        visit_order: 'row' or 'column' or 'nearest' for visiting order
    """

    def __init__(
        self,
        grid_spacing: float = 2.0,
        r_view: float = 2.0,
        visit_order: str = 'row',
        target_coverage: float = 0.95,
        r_robot: float = 0.3,
        d_safe: float = 0.2,
        **kwargs
    ):
        """Initialize Uniform Grid Explorer."""
        config = {
            'grid_spacing': grid_spacing,
            'r_view': r_view,
            'visit_order': visit_order,
        }
        config.update(kwargs)
        super().__init__(name='Uniform Grid', config=config)

        self.grid_spacing = grid_spacing
        self.r_view = r_view
        self.visit_order = visit_order
        self.target_coverage = target_coverage
        self.r_robot = r_robot
        self.d_safe = d_safe

    def explore(
        self,
        occupancy_grid: np.ndarray,
        start_pose: Tuple[float, float, float],
        visualizer: Optional[object] = None
    ) -> Dict:
        """
        Explore using uniform grid sampling.

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

        # All benchmark methods use the same inflated planning map and A*.
        planner = AStarPlanner(grid, self.r_robot, self.d_safe)
        grid_points = self._generate_grid_points(planner.planning_grid)

        if len(grid_points) == 0:
            return {
                'nodes': [],
                'metadata': {
                    'total_distance': 0.0,
                    'coverage_ratio': 0.0,
                    'num_nodes': 0,
                    'computation_time': time.time() - start_time,
                    'success': False
                },
                'success': False,
                'algorithm': self.name
            }

        # Order grid points based on visit strategy
        ordered_points = self._order_grid_points(
            grid_points, start_pose[:2], method=self.visit_order
        )

        # Visit each grid point through executable A* paths.
        current_pos = np.array(start_pose[:2])
        coverage_map = np.zeros_like(grid.data, dtype=bool)
        self._mark_coverage(coverage_map, grid, current_pos, self.r_view)
        self.nodes.append({
            'position': tuple(current_pos),
            'theta': start_pose[2],
            'coverage_area': np.sum(coverage_map) * (grid.resolution ** 2),
        })
        paths = []
        rejected_goals = 0

        for i, point in enumerate(ordered_points):
            if np.linalg.norm(point - current_pos) < grid.resolution:
                continue
            path = planner.plan(
                tuple(current_pos), tuple(point),
                max_iterations=max(10000, grid.data.size),
            )
            if path is None:
                rejected_goals += 1
                continue
            self.total_distance += planner.get_path_length(path)
            current_pos = point
            paths.append(path)

            # Mark coverage
            self._mark_coverage(coverage_map, grid, point, self.r_view)

            # Record node
            node = {
                'position': tuple(point),
                'theta': 0.0,  # Orientation not used
                'coverage_area': np.sum(coverage_map) * (grid.resolution ** 2)
            }
            self.nodes.append(node)

            # Visualize if available
            if visualizer is not None:
                visualizer.update(
                    current_position=tuple(point) + (0.0,),
                    nodes=self.nodes,
                    frontiers=[],
                    coverage_ratio=self._compute_coverage_ratio(
                        coverage_map, grid
                    )
                )

            self.coverage_ratio = self._compute_coverage_ratio(coverage_map, grid)
            if self.coverage_ratio >= self.target_coverage:
                break

        # Compute final coverage
        self.coverage_ratio = self._compute_coverage_ratio(coverage_map, grid)

        computation_time = time.time() - start_time

        return {
            'nodes': self.nodes,
            'metadata': {
                'total_distance': self.total_distance,
                'coverage_ratio': self.coverage_ratio,
                'num_nodes': len(self.nodes),
                'computation_time': computation_time,
                'success': self.coverage_ratio >= self.target_coverage,
                'paths': paths,
                'rejected_unreachable_goals': rejected_goals,
                'execution_protocol': 'shared inflated-grid A*',
            },
            'success': self.coverage_ratio >= self.target_coverage,
            'algorithm': self.name
        }

    def _generate_grid_points(
        self,
        grid: OccupancyGrid
    ) -> List[np.ndarray]:
        """
        Generate candidate grid points in free space.

        Args:
            grid: OccupancyGrid object

        Returns:
            List of (x, y) points in world coordinates
        """
        # Get map bounds
        width = grid.width * grid.resolution
        height = grid.height * grid.resolution

        # Generate grid coordinates
        # Center the lattice cells instead of anchoring points on wall cells.
        x_coords = np.arange(self.grid_spacing / 2.0, width, self.grid_spacing)
        y_coords = np.arange(self.grid_spacing / 2.0, height, self.grid_spacing)

        safe_indices = np.argwhere(grid.data < 50)
        if len(safe_indices) == 0:
            return []
        safe_points = np.asarray([
            grid.grid_to_world(int(row), int(col))
            for row, col in safe_indices
        ])
        safe_tree = cKDTree(safe_points)

        # Create grid mesh
        grid_points = []
        for x in x_coords:
            for y in y_coords:
                # Project an obstructed ideal lattice point to the closest
                # safe cell, but never farther than one half-cell diagonal.
                distance, index = safe_tree.query([x, y])
                if distance <= self.grid_spacing / np.sqrt(2.0):
                    point = safe_points[int(index)].copy()
                    if not any(np.linalg.norm(point - old) < grid.resolution for old in grid_points):
                        grid_points.append(point)

        return grid_points

    def _order_grid_points(
        self,
        points: List[np.ndarray],
        start_pos: Tuple[float, float],
        method: str = 'row'
    ) -> List[np.ndarray]:
        """
        Order grid points for visiting.

        Args:
            points: List of grid points
            start_pos: Starting position
            method: 'row', 'column', or 'nearest'

        Returns:
            Ordered list of points
        """
        if method == 'row':
            # Sort by y coordinate, then x
            return sorted(points, key=lambda p: (p[1], p[0]))
        elif method == 'column':
            # Sort by x coordinate, then y
            return sorted(points, key=lambda p: (p[0], p[1]))
        elif method == 'nearest':
            # Greedy nearest neighbor
            ordered = []
            remaining = points.copy()
            current = np.array(start_pos)

            while remaining:
                # Find nearest unvisited point
                distances = [np.linalg.norm(p - current) for p in remaining]
                idx = np.argmin(distances)
                nearest = remaining.pop(idx)
                ordered.append(nearest)
                current = nearest

            return ordered
        else:
            # Default: row order
            return sorted(points, key=lambda p: (p[1], p[0]))

    def _mark_coverage(
        self,
        coverage_map: np.ndarray,
        grid: OccupancyGrid,
        center: np.ndarray,
        radius: float
    ):
        """
        Mark circular coverage region on coverage map.

        Args:
            coverage_map: Boolean coverage map
            grid: OccupancyGrid object
            center: Center position (x, y)
            radius: Coverage radius
        """
        cx, cy = center
        center_row, center_col = grid.world_to_grid(cx, cy)
        radius_cells = int(radius / grid.resolution)

        # Mark circle
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr**2 + dc**2 <= radius_cells**2:
                    r = center_row + dr
                    c = center_col + dc
                    if 0 <= r < grid.height and 0 <= c < grid.width:
                        if grid.data[r, c] < 50:  # Free space
                            coverage_map[r, c] = True

    def _compute_coverage_ratio(
        self,
        coverage_map: np.ndarray,
        grid: OccupancyGrid
    ) -> float:
        """
        Compute coverage ratio.

        Args:
            coverage_map: Boolean coverage map
            grid: OccupancyGrid object

        Returns:
            Coverage ratio [0, 1]
        """
        free_space = (grid.data < 50).sum()
        covered = coverage_map.sum()
        return covered / max(free_space, 1)
