"""
Collision checking for SSTG Explorer.
"""
from typing import Tuple, List, Optional
from enum import Enum
import numpy as np

from src.map.occupancy_grid import OccupancyGrid
from src.utils.geometry import (
    euclidean_distance,
    sample_line,
    point_in_circle
)


class CollisionType(Enum):
    """Types of collision."""
    FREE = 0          # No collision
    HARD_OBSTACLE = 1  # Collision with obstacle
    SOFT_OBSTACLE = 2  # Too close to explored node


class CollisionChecker:
    """
    Collision checker for exploration.

    Checks collisions against:
    1. Hard obstacles (from occupancy grid)
    2. Soft obstacles (already explored nodes)
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        r_robot: float,
        d_safe: float = 0.1,
        obstacle_threshold: int = 50
    ):
        """
        Initialize collision checker.

        Args:
            occupancy_grid: Occupancy grid map.
            r_robot: Robot radius in meters.
            d_safe: Safety margin in meters.
            obstacle_threshold: Occupancy threshold for obstacles.
        """
        self.grid = occupancy_grid
        self.r_robot = r_robot
        self.d_safe = d_safe
        self.obstacle_threshold = obstacle_threshold

        # Create inflated grid for robot footprint
        self.inflated_grid = occupancy_grid.inflate_obstacles(
            r_robot + d_safe
        )

    def check_point(
        self,
        point: Tuple[float, float],
        use_inflated: bool = True
    ) -> bool:
        """
        Check if a point is collision-free.

        Args:
            point: Point to check (x, y).
            use_inflated: Use inflated grid (accounts for robot size).

        Returns:
            True if free, False if occupied.
        """
        grid = self.inflated_grid if use_inflated else self.grid

        # Check if within grid bounds
        if not grid.is_valid_world(point[0], point[1]):
            return False

        # Check occupancy
        return grid.is_free(point[0], point[1], self.obstacle_threshold)

    def check_path(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        resolution: Optional[float] = None
    ) -> bool:
        """
        Check if a straight-line path is collision-free.

        Args:
            start: Start point (x, y).
            end: End point (x, y).
            resolution: Sampling resolution. Defaults to grid resolution.

        Returns:
            True if path is free, False if collision detected.
        """
        if resolution is None:
            resolution = self.grid.resolution

        # Sample points along the path
        points = sample_line(start, end, resolution)

        # Check each point
        for point in points:
            if not self.check_point(point):
                return False

        return True

    def check_against_explored(
        self,
        point: Tuple[float, float],
        explored_nodes: List[Tuple[float, float]],
        d_repel: float,
        r_view: Optional[float] = None
    ) -> Tuple[bool, float]:
        """
        Check if point is within coverage of explored nodes.

        IMPORTANT: We now use r_view (coverage radius) instead of d_repel (spacing control)
        to determine if a point is already covered. This fixes the bug where frontiers
        in the range [d_repel, r_view] were incorrectly treated as FREE when they were
        actually already covered.

        Args:
            point: Point to check (x, y).
            explored_nodes: List of explored positions (x, y).
            d_repel: Repulsion distance (r_view - overlap) - kept for backwards compatibility.
            r_view: Coverage radius. If None, falls back to d_repel (old behavior).

        Returns:
            Tuple of (is_covered, min_distance):
            - is_covered: True if within r_view of any explored node (already covered).
            - min_distance: Distance to nearest explored node.
        """
        if not explored_nodes:
            return (False, float('inf'))

        # Find minimum distance to explored nodes
        min_dist = min(
            euclidean_distance(point, explored)
            for explored in explored_nodes
        )

        # FIX: Use r_view (actual coverage) instead of d_repel (spacing preference)
        coverage_radius = r_view if r_view is not None else d_repel
        is_covered = min_dist < coverage_radius

        return (is_covered, min_dist)

    def check_collision_type(
        self,
        point: Tuple[float, float],
        explored_nodes: List[Tuple[float, float]],
        d_repel: float,
        r_view: Optional[float] = None
    ) -> Tuple[CollisionType, float]:
        """
        Determine collision type for a point.

        FIXED: Now correctly uses r_view (coverage radius) to check if a point is
        already within the explored area's coverage. Points within r_view are marked
        as SOFT_OBSTACLE with strength based on distance from nearest explored node.

        Args:
            point: Point to check (x, y).
            explored_nodes: List of explored positions.
            d_repel: Repulsion distance (for spacing control).
            r_view: Coverage radius. If None, falls back to d_repel (old behavior).

        Returns:
            Tuple of (collision_type, exploration_strength):
            - collision_type: Type of collision (FREE/HARD_OBSTACLE/SOFT_OBSTACLE).
            - exploration_strength: Priority multiplier [0, 1].
                                   1.0 for FREE, 0.0 for HARD_OBSTACLE,
                                   distance/r_view for SOFT_OBSTACLE (within coverage).
        """
        # Check hard obstacle first
        if not self.check_point(point):
            return (CollisionType.HARD_OBSTACLE, 0.0)

        # Check if within coverage of explored nodes (use r_view, not d_repel!)
        is_covered, min_dist = self.check_against_explored(
            point, explored_nodes, d_repel, r_view
        )

        if is_covered:
            # Point is within r_view coverage of an explored node
            # Exploration strength decreases as we get closer to explored nodes
            # At distance=0: strength=0 (completely covered)
            # At distance=r_view: strength→1.0 (edge of coverage)
            coverage_radius = r_view if r_view is not None else d_repel
            strength = min_dist / coverage_radius if coverage_radius > 0 else 0.0

            # Apply additional penalty if within spacing distance (d_repel)
            # This encourages the algorithm to prefer frontiers farther from existing nodes
            if r_view is not None and min_dist < d_repel:
                # Within both coverage and spacing distance
                # Further reduce strength to discourage over-sampling
                spacing_penalty = 0.5  # Reduce by 50%
                strength *= spacing_penalty

            return (CollisionType.SOFT_OBSTACLE, strength)

        return (CollisionType.FREE, 1.0)

    def find_nearest_obstacle(
        self,
        point: Tuple[float, float],
        max_distance: float
    ) -> Optional[Tuple[float, float]]:
        """
        Find nearest obstacle within max_distance.

        Args:
            point: Query point (x, y).
            max_distance: Maximum search distance in meters.

        Returns:
            Position of nearest obstacle, or None if none found.
        """
        # Convert to grid coordinates
        i_center, j_center = self.grid.world_to_grid(point[0], point[1])

        # Search radius in cells
        search_cells = int(np.ceil(max_distance / self.grid.resolution))

        # Get occupied mask
        occupied_mask = self.grid.get_occupied_mask(self.obstacle_threshold)

        min_dist_sq = float('inf')
        nearest_obstacle = None

        # Search in a square region
        for di in range(-search_cells, search_cells + 1):
            for dj in range(-search_cells, search_cells + 1):
                i = i_center + di
                j = j_center + dj

                if not self.grid.is_valid_grid(i, j):
                    continue

                if occupied_mask[i, j]:
                    obstacle_pos = self.grid.grid_to_world(i, j)
                    dist_sq = (
                        (obstacle_pos[0] - point[0]) ** 2 +
                        (obstacle_pos[1] - point[1]) ** 2
                    )

                    if dist_sq < min_dist_sq:
                        min_dist_sq = dist_sq
                        nearest_obstacle = obstacle_pos

        return nearest_obstacle

    def get_obstacle_points_in_radius(
        self,
        center: Tuple[float, float],
        radius: float
    ) -> List[Tuple[float, float]]:
        """
        Get all obstacle points within a radius.

        Args:
            center: Center point (x, y).
            radius: Search radius in meters.

        Returns:
            List of obstacle positions (x, y).
        """
        # Convert to grid coordinates
        i_center, j_center = self.grid.world_to_grid(center[0], center[1])

        # Search radius in cells
        search_cells = int(np.ceil(radius / self.grid.resolution))

        # Get occupied mask
        occupied_mask = self.grid.get_occupied_mask(self.obstacle_threshold)

        obstacle_points = []

        # Search in a square region
        for di in range(-search_cells, search_cells + 1):
            for dj in range(-search_cells, search_cells + 1):
                i = i_center + di
                j = j_center + dj

                if not self.grid.is_valid_grid(i, j):
                    continue

                if occupied_mask[i, j]:
                    obstacle_pos = self.grid.grid_to_world(i, j)

                    # Check if within radius
                    if euclidean_distance(center, obstacle_pos) <= radius:
                        obstacle_points.append(obstacle_pos)

        return obstacle_points

    def update_grid(self, new_grid: OccupancyGrid):
        """
        Update the occupancy grid (for dynamic environments).

        Args:
            new_grid: New occupancy grid.
        """
        self.grid = new_grid
        self.inflated_grid = new_grid.inflate_obstacles(
            self.r_robot + self.d_safe
        )

    def __repr__(self) -> str:
        return (
            f"CollisionChecker(r_robot={self.r_robot:.2f}m, "
            f"d_safe={self.d_safe:.2f}m, "
            f"grid={self.grid})"
        )
