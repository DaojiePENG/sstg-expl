"""
A* path planning algorithm for SSTG Explorer.

Provides efficient pathfinding in occupancy grid maps with obstacle avoidance.
"""
import heapq
from typing import Tuple, List, Optional, Set
import numpy as np

from src.map.occupancy_grid import OccupancyGrid


class AStarPlanner:
    """
    A* path planner for 2D occupancy grid maps.

    Uses 8-connected grid (allows diagonal movement) and Euclidean distance heuristic.
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        robot_radius: float = 0.3,
        obstacle_threshold: int = 50
    ):
        """
        Initialize A* planner.

        Args:
            occupancy_grid: Occupancy grid map.
            robot_radius: Robot radius in meters (for inflation).
            obstacle_threshold: Occupancy value threshold (0-100).
        """
        self.grid = occupancy_grid
        self.robot_radius = robot_radius
        self.obstacle_threshold = obstacle_threshold

        # 8-connected neighbors (including diagonals)
        self.neighbors_8 = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        # Movement costs (diagonal = sqrt(2) ≈ 1.414)
        self.move_cost_straight = 1.0
        self.move_cost_diagonal = 1.414

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        max_iterations: int = 10000
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path from start to goal using A*.

        Args:
            start: Start position in world coordinates (x, y).
            goal: Goal position in world coordinates (x, y).
            max_iterations: Maximum number of iterations.

        Returns:
            List of waypoints in world coordinates, or None if no path found.
        """
        # Convert to grid coordinates
        start_grid = self.grid.world_to_grid(start[0], start[1])
        goal_grid = self.grid.world_to_grid(goal[0], goal[1])

        # Check if start/goal are valid
        if not self._is_valid_cell(start_grid[0], start_grid[1]):
            return None
        if not self._is_valid_cell(goal_grid[0], goal_grid[1]):
            return None

        # A* search
        path_grid = self._astar_search(start_grid, goal_grid, max_iterations)

        if path_grid is None:
            return None

        # Convert path back to world coordinates
        path_world = [
            self.grid.grid_to_world(cell[0], cell[1])
            for cell in path_grid
        ]

        # Simplify path (remove redundant waypoints)
        path_world = self._simplify_path(path_world)

        return path_world

    def _astar_search(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        max_iterations: int
    ) -> Optional[List[Tuple[int, int]]]:
        """
        Internal A* search in grid coordinates.

        Args:
            start: Start cell (row, col).
            goal: Goal cell (row, col).
            max_iterations: Maximum iterations.

        Returns:
            List of cells from start to goal, or None if no path.
        """
        # Priority queue: (f_score, counter, cell)
        # counter ensures FIFO for equal f_scores
        counter = 0
        open_set = [(0.0, counter, start)]
        heapq.heapify(open_set)

        # Track visited cells
        closed_set: Set[Tuple[int, int]] = set()

        # Cost from start to cell
        g_score = {start: 0.0}

        # Parent pointers for path reconstruction
        came_from = {}

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            # Pop cell with lowest f_score
            _, _, current = heapq.heappop(open_set)

            # Goal reached
            if current == goal:
                return self._reconstruct_path(came_from, current)

            # Skip if already visited
            if current in closed_set:
                continue

            closed_set.add(current)

            # Explore neighbors
            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set:
                    continue

                # Compute tentative g_score
                move_cost = self._get_move_cost(current, neighbor)
                tentative_g = g_score[current] + move_cost

                # Update if this is a better path
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    h_score = self._heuristic(neighbor, goal)
                    f_score = tentative_g + h_score

                    came_from[neighbor] = current

                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        # No path found
        return None

    def _get_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors of a cell."""
        row, col = cell
        neighbors = []

        for dr, dc in self.neighbors_8:
            new_row = row + dr
            new_col = col + dc

            if self._is_valid_cell(new_row, new_col):
                neighbors.append((new_row, new_col))

        return neighbors

    def _is_valid_cell(self, row: int, col: int) -> bool:
        """Check if a cell is within bounds and not an obstacle."""
        # Check bounds
        if row < 0 or row >= self.grid.height:
            return False
        if col < 0 or col >= self.grid.width:
            return False

        # Check occupancy
        if self.grid.data[row, col] >= self.obstacle_threshold:
            return False

        return True

    def _get_move_cost(
        self,
        from_cell: Tuple[int, int],
        to_cell: Tuple[int, int]
    ) -> float:
        """Get movement cost between adjacent cells."""
        dr = abs(to_cell[0] - from_cell[0])
        dc = abs(to_cell[1] - from_cell[1])

        # Diagonal move
        if dr == 1 and dc == 1:
            return self.move_cost_diagonal

        # Straight move
        return self.move_cost_straight

    def _heuristic(
        self,
        cell: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> float:
        """
        Heuristic function (Euclidean distance).

        Euclidean is admissible and more accurate than Manhattan
        for 8-connected grids.
        """
        dr = goal[0] - cell[0]
        dc = goal[1] - cell[1]

        # Convert to world distance for better accuracy
        world_dist = np.sqrt(dr**2 + dc**2) * self.grid.resolution

        return world_dist

    def _reconstruct_path(
        self,
        came_from: dict,
        current: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """Reconstruct path from parent pointers."""
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        # Reverse to get start -> goal order
        path.reverse()

        return path

    def _simplify_path(
        self,
        path: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """
        Simplify path by removing redundant waypoints.

        Uses line-of-sight check: if we can go directly from waypoint i
        to waypoint j, then all waypoints in between are redundant.

        Args:
            path: Original path (list of world coordinates).

        Returns:
            Simplified path.
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Try to skip as many waypoints as possible
            farthest_idx = current_idx + 1

            for next_idx in range(current_idx + 2, len(path)):
                if self._has_line_of_sight(path[current_idx], path[next_idx]):
                    farthest_idx = next_idx
                else:
                    break

            simplified.append(path[farthest_idx])
            current_idx = farthest_idx

        return simplified

    def _has_line_of_sight(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float]
    ) -> bool:
        """
        Check if there's a clear line of sight between two points.

        Uses Bresenham's line algorithm to check all cells along the line.

        Args:
            start: Start point (world coordinates).
            end: End point (world coordinates).

        Returns:
            True if line of sight is clear.
        """
        # Convert to grid coordinates
        start_grid = self.grid.world_to_grid(start[0], start[1])
        end_grid = self.grid.world_to_grid(end[0], end[1])

        # Bresenham's line algorithm
        x0, y0 = start_grid[1], start_grid[0]  # (col, row)
        x1, y1 = end_grid[1], end_grid[0]

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        x, y = x0, y0

        while True:
            # Check if current cell is valid
            if not self._is_valid_cell(y, x):  # (row, col)
                return False

            # Reached end
            if x == x1 and y == y1:
                break

            # Move to next cell
            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x += sx

            if e2 < dx:
                err += dx
                y += sy

        return True

    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """
        Compute total length of a path.

        Args:
            path: List of waypoints.

        Returns:
            Total path length in meters.
        """
        if len(path) < 2:
            return 0.0

        total_length = 0.0

        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            total_length += np.sqrt(dx**2 + dy**2)

        return total_length

    def check_path_feasibility(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        max_length: float = float('inf')
    ) -> bool:
        """
        Check if a path exists and is within max_length.

        Args:
            start: Start position.
            goal: Goal position.
            max_length: Maximum acceptable path length.

        Returns:
            True if feasible path exists.
        """
        path = self.plan(start, goal)

        if path is None:
            return False

        path_length = self.get_path_length(path)

        return path_length <= max_length


# Utility functions for integration with Explorer

def create_astar_planner(
    occupancy_grid: OccupancyGrid,
    robot_radius: float = 0.3,
    obstacle_threshold: int = 50
) -> AStarPlanner:
    """
    Factory function to create A* planner.

    Args:
        occupancy_grid: Occupancy grid map.
        robot_radius: Robot radius in meters.
        obstacle_threshold: Occupancy threshold.

    Returns:
        Configured A* planner instance.
    """
    return AStarPlanner(occupancy_grid, robot_radius, obstacle_threshold)
