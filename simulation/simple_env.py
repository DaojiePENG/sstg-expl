"""
Simple simulation environments for testing SSTG Explorer.
"""
import numpy as np
from typing import Tuple, Optional
from abc import ABC, abstractmethod

from src.map.occupancy_grid import OccupancyGrid


class Environment(ABC):
    """Base class for simulation environments."""

    def __init__(
        self,
        width: float,
        height: float,
        resolution: float = 0.05
    ):
        """
        Initialize environment.

        Args:
            width: Environment width in meters.
            height: Environment height in meters.
            resolution: Map resolution in meters/pixel.
        """
        self.width = width
        self.height = height
        self.resolution = resolution

        self.grid_width = int(np.ceil(width / resolution))
        self.grid_height = int(np.ceil(height / resolution))

        self.occupancy_grid: Optional[OccupancyGrid] = None

    @abstractmethod
    def generate_map(self) -> np.ndarray:
        """
        Generate occupancy map.

        Returns:
            2D array with occupancy values (0=free, 100=occupied).
        """
        pass

    def get_occupancy_map(self) -> OccupancyGrid:
        """
        Get occupancy grid map.

        Returns:
            OccupancyGrid object.
        """
        if self.occupancy_grid is None:
            map_data = self.generate_map()
            self.occupancy_grid = OccupancyGrid(
                data=map_data,
                resolution=self.resolution,
                origin=(0.0, 0.0)
            )
        return self.occupancy_grid

    def get_start_pose(self) -> Tuple[float, float, float]:
        """
        Get default start pose.

        Returns:
            Start pose (x, y, theta) in meters and degrees.
        """
        return (self.width / 2, self.height / 2, 0.0)


class EmptyRoom(Environment):
    """Simple empty rectangular room."""

    def __init__(
        self,
        width: float = 10.0,
        height: float = 10.0,
        wall_thickness: float = 0.3,
        resolution: float = 0.05
    ):
        """
        Initialize empty room.

        Args:
            width: Room width in meters.
            height: Room height in meters.
            wall_thickness: Wall thickness in meters.
            resolution: Map resolution.
        """
        super().__init__(width, height, resolution)
        self.wall_thickness = wall_thickness

    def generate_map(self) -> np.ndarray:
        """Generate empty room with walls."""
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        # Add walls
        wall_cells = int(self.wall_thickness / self.resolution)

        # Top and bottom walls
        map_data[:wall_cells, :] = 100
        map_data[-wall_cells:, :] = 100

        # Left and right walls
        map_data[:, :wall_cells] = 100
        map_data[:, -wall_cells:] = 100

        return map_data


class RoomWithObstacles(Environment):
    """Room with randomly placed rectangular obstacles."""

    def __init__(
        self,
        width: float = 10.0,
        height: float = 10.0,
        num_obstacles: int = 5,
        wall_thickness: float = 0.3,
        resolution: float = 0.05,
        seed: Optional[int] = None
    ):
        """
        Initialize room with obstacles.

        Args:
            width: Room width.
            height: Room height.
            num_obstacles: Number of random obstacles.
            wall_thickness: Wall thickness.
            resolution: Map resolution.
            seed: Random seed for reproducibility.
        """
        super().__init__(width, height, resolution)
        self.num_obstacles = num_obstacles
        self.wall_thickness = wall_thickness
        self.seed = seed

    def generate_map(self) -> np.ndarray:
        """Generate room with random obstacles."""
        if self.seed is not None:
            np.random.seed(self.seed)

        # Start with empty room
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        # Add walls
        wall_cells = int(self.wall_thickness / self.resolution)
        map_data[:wall_cells, :] = 100
        map_data[-wall_cells:, :] = 100
        map_data[:, :wall_cells] = 100
        map_data[:, -wall_cells:] = 100

        # Add random rectangular obstacles
        for _ in range(self.num_obstacles):
            # Random size (0.5m to 2m)
            obs_width = np.random.uniform(0.5, 2.0)
            obs_height = np.random.uniform(0.5, 2.0)

            # Random position (avoid edges)
            margin = 1.5  # meters from edge
            x = np.random.uniform(margin, self.width - margin - obs_width)
            y = np.random.uniform(margin, self.height - margin - obs_height)

            # Convert to grid coordinates
            i_start = int(y / self.resolution)
            i_end = int((y + obs_height) / self.resolution)
            j_start = int(x / self.resolution)
            j_end = int((x + obs_width) / self.resolution)

            # Place obstacle
            map_data[i_start:i_end, j_start:j_end] = 100

        return map_data


class Corridor(Environment):
    """Long narrow corridor environment."""

    def __init__(
        self,
        length: float = 20.0,
        width: float = 2.0,
        wall_thickness: float = 0.3,
        resolution: float = 0.05
    ):
        """
        Initialize corridor.

        Args:
            length: Corridor length.
            width: Corridor width.
            wall_thickness: Wall thickness.
            resolution: Map resolution.
        """
        super().__init__(length, width, resolution)
        self.wall_thickness = wall_thickness

    def generate_map(self) -> np.ndarray:
        """Generate corridor map."""
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        # Add walls
        wall_cells = int(self.wall_thickness / self.resolution)
        map_data[:wall_cells, :] = 100
        map_data[-wall_cells:, :] = 100
        map_data[:, :wall_cells] = 100
        map_data[:, -wall_cells:] = 100

        return map_data

    def get_start_pose(self) -> Tuple[float, float, float]:
        """Start at one end of corridor."""
        return (1.0, self.height / 2, 0.0)


class MultipleRooms(Environment):
    """Environment with multiple connected rooms."""

    def __init__(
        self,
        width: float = 15.0,
        height: float = 10.0,
        wall_thickness: float = 0.3,
        resolution: float = 0.05
    ):
        """
        Initialize multiple rooms environment.

        Args:
            width: Total width.
            height: Total height.
            wall_thickness: Wall thickness.
            resolution: Map resolution.
        """
        super().__init__(width, height, resolution)
        self.wall_thickness = wall_thickness

    def generate_map(self) -> np.ndarray:
        """Generate map with 3 connected rooms."""
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        wall_cells = int(self.wall_thickness / self.resolution)

        # Outer walls
        map_data[:wall_cells, :] = 100
        map_data[-wall_cells:, :] = 100
        map_data[:, :wall_cells] = 100
        map_data[:, -wall_cells:] = 100

        # Vertical dividing walls (creating 3 rooms)
        room_width = self.grid_width // 3

        # First dividing wall (with door)
        j1 = room_width
        map_data[:, j1:j1+wall_cells] = 100
        door_start = self.grid_height // 3
        door_end = 2 * self.grid_height // 3
        map_data[door_start:door_end, j1:j1+wall_cells] = 0  # Door

        # Second dividing wall (with door)
        j2 = 2 * room_width
        map_data[:, j2:j2+wall_cells] = 100
        door_start = self.grid_height // 4
        door_end = 3 * self.grid_height // 4
        map_data[door_start:door_end, j2:j2+wall_cells] = 0  # Door

        return map_data

    def get_start_pose(self) -> Tuple[float, float, float]:
        """Start in the first room."""
        return (self.width / 6, self.height / 2, 0.0)


class ComplexApartment(Environment):
    """Complex apartment-like environment."""

    def __init__(
        self,
        width: float = 12.0,
        height: float = 12.0,
        wall_thickness: float = 0.3,
        resolution: float = 0.05
    ):
        """
        Initialize complex apartment.

        Args:
            width: Apartment width.
            height: Apartment height.
            wall_thickness: Wall thickness.
            resolution: Map resolution.
        """
        super().__init__(width, height, resolution)
        self.wall_thickness = wall_thickness

    def generate_map(self) -> np.ndarray:
        """Generate apartment-like layout."""
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        wall_cells = int(self.wall_thickness / self.resolution)

        # Outer walls
        map_data[:wall_cells, :] = 100
        map_data[-wall_cells:, :] = 100
        map_data[:, :wall_cells] = 100
        map_data[:, -wall_cells:] = 100

        # Central corridor (horizontal)
        corridor_top = self.grid_height // 2 - 10
        corridor_bottom = self.grid_height // 2 + 10

        # Vertical walls creating rooms
        for j in [self.grid_width // 3, 2 * self.grid_width // 3]:
            map_data[corridor_bottom:, j:j+wall_cells] = 100
            map_data[:corridor_top, j:j+wall_cells] = 100

            # Doors
            door_top = corridor_top + (corridor_bottom - corridor_top) // 3
            door_bottom = corridor_top + 2 * (corridor_bottom - corridor_top) // 3
            map_data[door_top:door_bottom, j:j+wall_cells] = 0

        # Add some furniture (small obstacles)
        # Table in first room
        i_table = self.grid_height // 4
        j_table = self.grid_width // 6
        table_size = int(0.8 / self.resolution)
        map_data[i_table:i_table+table_size, j_table:j_table+table_size] = 100

        return map_data

    def get_start_pose(self) -> Tuple[float, float, float]:
        """Start in corridor."""
        return (self.width / 2, self.height / 2, 0.0)


# Environment factory
def create_environment(env_type: str, **kwargs) -> Environment:
    """
    Factory function to create environments.

    Args:
        env_type: Type of environment ('empty', 'obstacles', 'corridor',
                                       'multiple_rooms', 'apartment').
        **kwargs: Additional arguments passed to environment constructor.

    Returns:
        Environment instance.
    """
    env_map = {
        'empty': EmptyRoom,
        'obstacles': RoomWithObstacles,
        'corridor': Corridor,
        'multiple_rooms': MultipleRooms,
        'apartment': ComplexApartment
    }

    if env_type not in env_map:
        raise ValueError(f"Unknown environment type: {env_type}. "
                        f"Available: {list(env_map.keys())}")

    return env_map[env_type](**kwargs)
