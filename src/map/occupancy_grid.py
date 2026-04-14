"""
Occupancy grid map representation for SSTG Explorer.
"""
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class OccupancyGrid:
    """
    2D occupancy grid map.

    Represents the environment as a grid of cells, where each cell has an
    occupancy probability.

    Coordinate system:
    - Grid coordinates (i, j): integer indices in the grid array.
    - World coordinates (x, y): real-world positions in meters.
    - Origin: world coordinates of grid[0, 0].

    Cell values:
    - 0: Free space
    - 100: Occupied (obstacle)
    - -1: Unknown
    """

    data: np.ndarray  # Grid data (height x width)
    resolution: float  # Meters per cell
    origin: Tuple[float, float]  # World coordinates of grid[0, 0]

    def __post_init__(self):
        """Validate grid data."""
        if self.data.ndim != 2:
            raise ValueError("Grid data must be 2D array")
        if self.resolution <= 0:
            raise ValueError("Resolution must be positive")

    @property
    def width(self) -> int:
        """Get grid width in cells."""
        return self.data.shape[1]

    @property
    def height(self) -> int:
        """Get grid height in cells."""
        return self.data.shape[0]

    @property
    def shape(self) -> Tuple[int, int]:
        """Get grid shape (height, width)."""
        return self.data.shape

    @property
    def world_width(self) -> float:
        """Get grid width in meters."""
        return self.width * self.resolution

    @property
    def world_height(self) -> float:
        """Get grid height in meters."""
        return self.height * self.resolution

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid indices.

        Args:
            x: World X coordinate (meters).
            y: World Y coordinate (meters).

        Returns:
            Grid indices (i, j) where i is row (Y), j is column (X).
        """
        j = int((x - self.origin[0]) / self.resolution)
        i = int((y - self.origin[1]) / self.resolution)
        return (i, j)

    def grid_to_world(self, i: int, j: int) -> Tuple[float, float]:
        """
        Convert grid indices to world coordinates (center of cell).

        Args:
            i: Grid row index (Y).
            j: Grid column index (X).

        Returns:
            World coordinates (x, y) of cell center.
        """
        x = self.origin[0] + (j + 0.5) * self.resolution
        y = self.origin[1] + (i + 0.5) * self.resolution
        return (x, y)

    def is_valid_grid(self, i: int, j: int) -> bool:
        """
        Check if grid indices are within bounds.

        Args:
            i: Grid row index.
            j: Grid column index.

        Returns:
            True if indices are valid, False otherwise.
        """
        return 0 <= i < self.height and 0 <= j < self.width

    def is_valid_world(self, x: float, y: float) -> bool:
        """
        Check if world coordinates are within grid bounds.

        Args:
            x: World X coordinate.
            y: World Y coordinate.

        Returns:
            True if coordinates are valid, False otherwise.
        """
        i, j = self.world_to_grid(x, y)
        return self.is_valid_grid(i, j)

    def get_value(self, x: float, y: float, default: int = -1) -> int:
        """
        Get occupancy value at world coordinates.

        Args:
            x: World X coordinate.
            y: World Y coordinate.
            default: Default value if coordinates are out of bounds.

        Returns:
            Occupancy value (0=free, 100=occupied, -1=unknown).
        """
        i, j = self.world_to_grid(x, y)
        if not self.is_valid_grid(i, j):
            return default
        return int(self.data[i, j])

    def set_value(self, x: float, y: float, value: int):
        """
        Set occupancy value at world coordinates.

        Args:
            x: World X coordinate.
            y: World Y coordinate.
            value: Occupancy value to set.
        """
        i, j = self.world_to_grid(x, y)
        if self.is_valid_grid(i, j):
            self.data[i, j] = value

    def is_occupied(self, x: float, y: float, threshold: int = 50) -> bool:
        """
        Check if a world position is occupied.

        Args:
            x: World X coordinate.
            y: World Y coordinate.
            threshold: Occupancy threshold (>= threshold means occupied).

        Returns:
            True if occupied, False otherwise.
        """
        value = self.get_value(x, y, default=0)
        return value >= threshold

    def is_free(self, x: float, y: float, threshold: int = 50) -> bool:
        """
        Check if a world position is free.

        Args:
            x: World X coordinate.
            y: World Y coordinate.
            threshold: Occupancy threshold (< threshold means free).

        Returns:
            True if free, False otherwise.
        """
        value = self.get_value(x, y, default=100)
        return 0 <= value < threshold

    def get_free_space_mask(self, threshold: int = 50) -> np.ndarray:
        """
        Get binary mask of free space.

        Args:
            threshold: Occupancy threshold.

        Returns:
            Boolean array where True = free space.
        """
        return (self.data >= 0) & (self.data < threshold)

    def get_occupied_mask(self, threshold: int = 50) -> np.ndarray:
        """
        Get binary mask of occupied space.

        Args:
            threshold: Occupancy threshold.

        Returns:
            Boolean array where True = occupied space.
        """
        return self.data >= threshold

    def get_unknown_mask(self) -> np.ndarray:
        """
        Get binary mask of unknown space.

        Returns:
            Boolean array where True = unknown space.
        """
        return self.data < 0

    def compute_free_space_area(self, threshold: int = 50) -> float:
        """
        Compute total area of free space in square meters.

        Args:
            threshold: Occupancy threshold.

        Returns:
            Free space area in m².
        """
        free_mask = self.get_free_space_mask(threshold)
        num_free_cells = np.sum(free_mask)
        cell_area = self.resolution ** 2
        return num_free_cells * cell_area

    def inflate_obstacles(self, inflation_radius: float) -> 'OccupancyGrid':
        """
        Create a new grid with inflated obstacles.

        Args:
            inflation_radius: Inflation radius in meters.

        Returns:
            New OccupancyGrid with inflated obstacles.
        """
        from scipy.ndimage import binary_dilation

        # Convert radius to cells
        inflation_cells = int(np.ceil(inflation_radius / self.resolution))

        # Create structuring element (circular)
        y, x = np.ogrid[-inflation_cells:inflation_cells+1,
                        -inflation_cells:inflation_cells+1]
        struct_elem = (x**2 + y**2 <= inflation_cells**2)

        # Dilate occupied regions
        occupied_mask = self.get_occupied_mask()
        inflated_mask = binary_dilation(occupied_mask, structure=struct_elem)

        # Create new grid
        new_data = self.data.copy()
        new_data[inflated_mask] = 100

        return OccupancyGrid(
            data=new_data,
            resolution=self.resolution,
            origin=self.origin
        )

    @classmethod
    def create_empty(
        cls,
        width: float,
        height: float,
        resolution: float,
        origin: Optional[Tuple[float, float]] = None
    ) -> 'OccupancyGrid':
        """
        Create an empty occupancy grid (all free space).

        Args:
            width: Grid width in meters.
            height: Grid height in meters.
            resolution: Cell resolution in meters.
            origin: Origin in world coordinates. Defaults to (0, 0).

        Returns:
            New OccupancyGrid filled with free space (0).
        """
        if origin is None:
            origin = (0.0, 0.0)

        grid_width = int(np.ceil(width / resolution))
        grid_height = int(np.ceil(height / resolution))

        data = np.zeros((grid_height, grid_width), dtype=np.int8)

        return cls(data=data, resolution=resolution, origin=origin)

    @classmethod
    def from_image(
        cls,
        image: np.ndarray,
        resolution: float,
        origin: Optional[Tuple[float, float]] = None,
        occupied_thresh: int = 200,
        free_thresh: int = 50
    ) -> 'OccupancyGrid':
        """
        Create occupancy grid from an image.

        Args:
            image: Image array (grayscale, 0-255).
            resolution: Cell resolution in meters.
            origin: Origin in world coordinates. Defaults to (0, 0).
            occupied_thresh: Pixel values >= this are occupied.
            free_thresh: Pixel values < this are free.

        Returns:
            New OccupancyGrid.
        """
        if origin is None:
            origin = (0.0, 0.0)

        # Convert image to occupancy values
        data = np.zeros_like(image, dtype=np.int8)
        data[image >= occupied_thresh] = 100
        data[image < free_thresh] = 0
        data[(image >= free_thresh) & (image < occupied_thresh)] = -1

        return cls(data=data, resolution=resolution, origin=origin)

    def copy(self) -> 'OccupancyGrid':
        """Create a deep copy of the grid."""
        return OccupancyGrid(
            data=self.data.copy(),
            resolution=self.resolution,
            origin=self.origin
        )

    def __repr__(self) -> str:
        return (
            f"OccupancyGrid(shape={self.shape}, "
            f"resolution={self.resolution:.3f}m, "
            f"world_size=({self.world_width:.2f}m, {self.world_height:.2f}m), "
            f"origin={self.origin})"
        )
