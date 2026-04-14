"""
Distance field for fast collision checking and distance queries.

Uses Euclidean Distance Transform (EDT) to precompute distances to obstacles.
"""
import numpy as np
from scipy import ndimage
from typing import Tuple, Optional

from src.map.occupancy_grid import OccupancyGrid


class DistanceField:
    """
    Distance field for efficient distance queries to obstacles.

    Precomputes the distance from every free cell to the nearest obstacle
    using Euclidean Distance Transform (EDT). This enables O(1) distance queries.
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        obstacle_threshold: int = 50,
        max_distance: Optional[float] = None
    ):
        """
        Initialize distance field.

        Args:
            occupancy_grid: Occupancy grid map.
            obstacle_threshold: Occupancy value threshold (0-100).
            max_distance: Maximum distance to compute (in meters).
                         None means compute full distance field.
        """
        self.grid = occupancy_grid
        self.obstacle_threshold = obstacle_threshold
        self.max_distance = max_distance

        # Compute distance field
        self._compute_distance_field()

    def _compute_distance_field(self):
        """Compute Euclidean distance transform."""
        # Create binary obstacle map (True = free, False = obstacle)
        free_space = self.grid.data < self.obstacle_threshold

        # Compute Euclidean distance transform
        # Returns distance in pixels
        distance_pixels = ndimage.distance_transform_edt(free_space)

        # Convert to meters
        self.distance_field = distance_pixels * self.grid.resolution

        # Apply max distance threshold if specified
        if self.max_distance is not None:
            self.distance_field = np.minimum(
                self.distance_field,
                self.max_distance
            )

        # Also compute for obstacles (distance to free space)
        # Useful for some queries
        obstacle_space = ~free_space
        distance_pixels_obs = ndimage.distance_transform_edt(obstacle_space)
        self.distance_to_free = distance_pixels_obs * self.grid.resolution

    def get_distance(
        self,
        x: float,
        y: float,
        return_gradient: bool = False
    ) -> float:
        """
        Get distance to nearest obstacle at a world coordinate.

        Args:
            x: X coordinate in meters.
            y: Y coordinate in meters.
            return_gradient: If True, also return gradient direction.

        Returns:
            Distance to nearest obstacle in meters.
            If return_gradient=True, returns (distance, gradient_x, gradient_y).
        """
        # Convert to grid coordinates
        row, col = self.grid.world_to_grid(x, y)

        # Check bounds
        if not self._is_in_bounds(row, col):
            return 0.0 if not return_gradient else (0.0, 0.0, 0.0)

        # Get distance
        distance = self.distance_field[row, col]

        if not return_gradient:
            return distance

        # Compute gradient (direction to nearest obstacle)
        grad_x, grad_y = self._compute_gradient(row, col)

        return distance, grad_x, grad_y

    def get_distance_grid(
        self,
        x: float,
        y: float
    ) -> float:
        """
        Get distance using grid coordinates (faster, no conversion).

        Args:
            x: X in grid coordinates.
            y: Y in grid coordinates.

        Returns:
            Distance in meters.
        """
        row, col = int(y), int(x)

        if not self._is_in_bounds(row, col):
            return 0.0

        return self.distance_field[row, col]

    def is_collision_free(
        self,
        x: float,
        y: float,
        safety_distance: float = 0.0
    ) -> bool:
        """
        Check if a point is collision-free.

        Args:
            x: X coordinate in meters.
            y: Y coordinate in meters.
            safety_distance: Required clearance to obstacles (meters).

        Returns:
            True if point is at least safety_distance from obstacles.
        """
        distance = self.get_distance(x, y)
        return distance >= safety_distance

    def check_path_clearance(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        safety_distance: float = 0.0,
        num_samples: int = 20
    ) -> bool:
        """
        Check if a straight-line path has sufficient clearance.

        Args:
            start: Start position (x, y).
            end: End position (x, y).
            safety_distance: Required clearance.
            num_samples: Number of points to check along path.

        Returns:
            True if entire path is collision-free.
        """
        # Sample points along the line
        x_samples = np.linspace(start[0], end[0], num_samples)
        y_samples = np.linspace(start[1], end[1], num_samples)

        for x, y in zip(x_samples, y_samples):
            if not self.is_collision_free(x, y, safety_distance):
                return False

        return True

    def get_safe_region_radius(
        self,
        x: float,
        y: float
    ) -> float:
        """
        Get radius of largest circle centered at (x,y) that is obstacle-free.

        Args:
            x: X coordinate in meters.
            y: Y coordinate in meters.

        Returns:
            Radius of safe region in meters.
        """
        return self.get_distance(x, y)

    def find_nearest_obstacle(
        self,
        x: float,
        y: float
    ) -> Optional[Tuple[float, float]]:
        """
        Find the nearest obstacle point to a given position.

        Args:
            x: X coordinate in meters.
            y: Y coordinate in meters.

        Returns:
            (x, y) of nearest obstacle, or None if not found.
        """
        # Convert to grid
        row, col = self.grid.world_to_grid(x, y)

        if not self._is_in_bounds(row, col):
            return None

        # Get distance
        distance = self.distance_field[row, col]

        if distance == 0:
            # Already at obstacle
            return (x, y)

        # Get gradient direction
        _, grad_x, grad_y = self.get_distance(x, y, return_gradient=True)

        # Move in gradient direction by distance
        nearest_x = x + grad_x * distance
        nearest_y = y + grad_y * distance

        return (nearest_x, nearest_y)

    def _compute_gradient(
        self,
        row: int,
        col: int
    ) -> Tuple[float, float]:
        """
        Compute gradient of distance field at a grid cell.

        Args:
            row: Row index.
            col: Column index.

        Returns:
            (grad_x, grad_y) - normalized gradient vector.
        """
        # Central difference for gradient estimation
        grad_row = 0.0
        grad_col = 0.0

        if row > 0 and row < self.grid.height - 1:
            grad_row = (self.distance_field[row+1, col] -
                       self.distance_field[row-1, col]) / (2 * self.grid.resolution)

        if col > 0 and col < self.grid.width - 1:
            grad_col = (self.distance_field[row, col+1] -
                       self.distance_field[row, col-1]) / (2 * self.grid.resolution)

        # Normalize
        magnitude = np.sqrt(grad_row**2 + grad_col**2)

        if magnitude > 1e-6:
            grad_row /= magnitude
            grad_col /= magnitude

        # Convert row/col gradient to x/y gradient
        # row increases downward (y decreases), col increases rightward (x increases)
        grad_x = grad_col
        grad_y = -grad_row

        return grad_x, grad_y

    def _is_in_bounds(self, row: int, col: int) -> bool:
        """Check if grid coordinates are in bounds."""
        return (0 <= row < self.grid.height and
                0 <= col < self.grid.width)

    def get_clearance_map(
        self,
        min_clearance: float = 0.0
    ) -> np.ndarray:
        """
        Get binary map of cells with sufficient clearance.

        Args:
            min_clearance: Minimum required clearance (meters).

        Returns:
            Binary array (True = sufficient clearance).
        """
        return self.distance_field >= min_clearance

    def visualize_distance_field(
        self,
        max_distance: Optional[float] = None
    ) -> np.ndarray:
        """
        Get distance field visualization array.

        Args:
            max_distance: Maximum distance to display (for better contrast).

        Returns:
            Normalized distance field for visualization.
        """
        vis_field = self.distance_field.copy()

        if max_distance is not None:
            vis_field = np.minimum(vis_field, max_distance)

        # Normalize to [0, 1]
        max_val = vis_field.max()
        if max_val > 0:
            vis_field = vis_field / max_val

        return vis_field

    def get_statistics(self) -> dict:
        """
        Get statistics about the distance field.

        Returns:
            Dictionary with statistics.
        """
        free_cells = self.distance_field > 0

        stats = {
            'min_distance': float(self.distance_field[free_cells].min()) if free_cells.any() else 0.0,
            'max_distance': float(self.distance_field.max()),
            'mean_distance': float(self.distance_field[free_cells].mean()) if free_cells.any() else 0.0,
            'std_distance': float(self.distance_field[free_cells].std()) if free_cells.any() else 0.0,
            'num_free_cells': int(free_cells.sum()),
            'num_obstacle_cells': int((~free_cells).sum())
        }

        return stats


def create_distance_field(
    occupancy_grid: OccupancyGrid,
    obstacle_threshold: int = 50,
    max_distance: Optional[float] = None
) -> DistanceField:
    """
    Factory function to create distance field.

    Args:
        occupancy_grid: Occupancy grid map.
        obstacle_threshold: Occupancy threshold.
        max_distance: Maximum distance to compute (None = full field).

    Returns:
        Computed distance field.
    """
    return DistanceField(occupancy_grid, obstacle_threshold, max_distance)
