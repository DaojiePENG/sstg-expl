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

        # Get start position to avoid placing obstacles there
        start_x, start_y, _ = self.get_start_pose()
        safe_radius = 0.8  # Keep 0.8m radius around start clear

        # Add random rectangular obstacles
        attempts = 0
        max_attempts = self.num_obstacles * 10  # Prevent infinite loop
        placed = 0

        while placed < self.num_obstacles and attempts < max_attempts:
            attempts += 1

            # Random size (0.5m to 2m)
            obs_width = np.random.uniform(0.5, 2.0)
            obs_height = np.random.uniform(0.5, 2.0)

            # Random position (avoid edges)
            margin = 1.5  # meters from edge
            x = np.random.uniform(margin, self.width - margin - obs_width)
            y = np.random.uniform(margin, self.height - margin - obs_height)

            # Check if obstacle overlaps with start position safe zone
            obs_center_x = x + obs_width / 2
            obs_center_y = y + obs_height / 2
            dist_to_start = np.sqrt((obs_center_x - start_x)**2 + (obs_center_y - start_y)**2)

            # Also check corners
            corners = [
                (x, y), (x + obs_width, y),
                (x, y + obs_height), (x + obs_width, y + obs_height)
            ]
            min_corner_dist = min(
                np.sqrt((cx - start_x)**2 + (cy - start_y)**2)
                for cx, cy in corners
            )

            # Skip if too close to start position
            if dist_to_start < safe_radius or min_corner_dist < safe_radius:
                continue

            # Convert to grid coordinates
            i_start = int(y / self.resolution)
            i_end = int((y + obs_height) / self.resolution)
            j_start = int(x / self.resolution)
            j_end = int((x + obs_width) / self.resolution)

            # Place obstacle
            map_data[i_start:i_end, j_start:j_end] = 100
            placed += 1

        if placed < self.num_obstacles:
            print(f"Warning: Only placed {placed}/{self.num_obstacles} obstacles "
                  f"(couldn't find valid positions for others)")

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


class LShapedCorridor(Environment):
    """L-shaped narrow corridor environment."""

    def __init__(
        self,
        corridor_length: float = 10.0,
        corridor_width: float = 2.5,
        wall_thickness: float = 0.3,
        resolution: float = 0.05
    ):
        total_width = corridor_length + corridor_width + wall_thickness
        total_height = corridor_length + corridor_width + wall_thickness
        super().__init__(total_width, total_height, resolution)
        self.corridor_length = corridor_length
        self.corridor_width = corridor_width
        self.wall_thickness = wall_thickness

    def generate_map(self) -> np.ndarray:
        map_data = np.full((self.grid_height, self.grid_width), 100, dtype=np.int8)
        wc = int(self.wall_thickness / self.resolution)
        cw = int(self.corridor_width / self.resolution)
        cl = int(self.corridor_length / self.resolution)

        # Horizontal corridor (bottom-left to right)
        y_start = wc
        y_end = wc + cw
        x_start = wc
        x_end = wc + cl
        map_data[y_start:y_end, x_start:x_end] = 0

        # Vertical corridor (from junction going up)
        jx_start = x_end - cw
        jx_end = x_end
        jy_start = y_start
        jy_end = y_start + cl
        map_data[jy_start:jy_end, jx_start:jx_end] = 0

        return map_data

    def get_start_pose(self) -> Tuple[float, float, float]:
        wt = self.wall_thickness
        cw = self.corridor_width
        return (wt + 1.0, wt + cw / 2, 0.0)


class MazeEnvironment(Environment):
    """Maze with corridors and dead ends."""

    def __init__(
        self,
        width: float = 12.0,
        height: float = 12.0,
        passage_width: float = 2.0,
        wall_thickness: float = 0.3,
        resolution: float = 0.05
    ):
        super().__init__(width, height, resolution)
        self.passage_width = passage_width
        self.wall_thickness = wall_thickness

    def generate_map(self) -> np.ndarray:
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        wc = int(self.wall_thickness / self.resolution)
        pw = int(self.passage_width / self.resolution)

        # Outer walls
        map_data[:wc, :] = 100
        map_data[-wc:, :] = 100
        map_data[:, :wc] = 100
        map_data[:, -wc:] = 100

        H, W = self.grid_height, self.grid_width

        # Horizontal wall 1: from left wall to ~70% width, at 1/3 height
        y1 = H // 3
        map_data[y1:y1+wc, wc:int(W*0.7)] = 100
        # Opening near center
        ox = int(W * 0.35)
        map_data[y1:y1+wc, ox:ox+pw] = 0

        # Horizontal wall 2: from ~30% to right wall, at 2/3 height
        y2 = 2 * H // 3
        map_data[y2:y2+wc, int(W*0.3):W-wc] = 100
        # Opening near center
        ox2 = int(W * 0.6)
        map_data[y2:y2+wc, ox2:ox2+pw] = 0

        # Vertical wall 1: from top to ~60% height, at 1/2 width
        x1 = W // 2
        map_data[wc:int(H*0.6), x1:x1+wc] = 100
        # Opening
        oy1 = int(H * 0.2)
        map_data[oy1:oy1+pw, x1:x1+wc] = 0

        # Vertical wall 2: from 40% to bottom, at 1/4 width
        x2 = W // 4
        map_data[int(H*0.4):H-wc, x2:x2+wc] = 100
        # Opening
        oy2 = int(H * 0.7)
        map_data[oy2:oy2+pw, x2:x2+wc] = 0

        # Vertical wall 3: from 50% to bottom, at 3/4 width
        x3 = 3 * W // 4
        map_data[int(H*0.5):H-wc, x3:x3+wc] = 100
        # Opening
        oy3 = int(H * 0.8)
        map_data[oy3:oy3+pw, x3:x3+wc] = 0

        return map_data

    def get_start_pose(self) -> Tuple[float, float, float]:
        return (1.5, 1.5, 0.0)


class DenseObstacles(Environment):
    """Room with many small, densely packed obstacles."""

    def __init__(
        self,
        width: float = 10.0,
        height: float = 10.0,
        num_obstacles: int = 15,
        wall_thickness: float = 0.3,
        resolution: float = 0.05,
        seed: int = 43
    ):
        super().__init__(width, height, resolution)
        self.num_obstacles = num_obstacles
        self.wall_thickness = wall_thickness
        self.seed = seed

    def generate_map(self) -> np.ndarray:
        np.random.seed(self.seed)
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        wc = int(self.wall_thickness / self.resolution)

        # Outer walls
        map_data[:wc, :] = 100
        map_data[-wc:, :] = 100
        map_data[:, :wc] = 100
        map_data[:, -wc:] = 100

        start_x, start_y, _ = self.get_start_pose()
        safe_radius = 0.8

        placed = 0
        attempts = 0
        while placed < self.num_obstacles and attempts < self.num_obstacles * 15:
            attempts += 1
            obs_w = np.random.uniform(0.3, 1.2)
            obs_h = np.random.uniform(0.3, 1.2)
            margin = 1.0
            x = np.random.uniform(margin, self.width - margin - obs_w)
            y = np.random.uniform(margin, self.height - margin - obs_h)

            cx, cy = x + obs_w/2, y + obs_h/2
            if np.sqrt((cx - start_x)**2 + (cy - start_y)**2) < safe_radius:
                continue

            i0 = int(y / self.resolution)
            i1 = int((y + obs_h) / self.resolution)
            j0 = int(x / self.resolution)
            j1 = int((x + obs_w) / self.resolution)
            map_data[i0:i1, j0:j1] = 100
            placed += 1

        return map_data


class NarrowPassages(Environment):
    """Multiple rooms connected by very narrow doorways."""

    def __init__(
        self,
        width: float = 15.0,
        height: float = 10.0,
        door_width: float = 0.8,
        wall_thickness: float = 0.3,
        resolution: float = 0.05
    ):
        super().__init__(width, height, resolution)
        self.door_width = door_width
        self.wall_thickness = wall_thickness

    def generate_map(self) -> np.ndarray:
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        wc = int(self.wall_thickness / self.resolution)
        dw = int(self.door_width / self.resolution)
        H, W = self.grid_height, self.grid_width

        # Outer walls
        map_data[:wc, :] = 100
        map_data[-wc:, :] = 100
        map_data[:, :wc] = 100
        map_data[:, -wc:] = 100

        # Vertical wall at 1/3 width (full height, narrow door at center)
        x1 = W // 3
        map_data[:, x1:x1+wc] = 100
        door_c = H // 2
        map_data[door_c - dw//2:door_c + dw//2, x1:x1+wc] = 0

        # Vertical wall at 2/3 width (full height, narrow door at 1/3 height)
        x2 = 2 * W // 3
        map_data[:, x2:x2+wc] = 100
        door_c2 = H // 3
        map_data[door_c2 - dw//2:door_c2 + dw//2, x2:x2+wc] = 0

        # Horizontal wall at 1/2 height in left room (narrow door at center of left room)
        y1 = H // 2
        map_data[y1:y1+wc, wc:x1] = 100
        door_cx = (wc + x1) // 2
        map_data[y1:y1+wc, door_cx - dw//2:door_cx + dw//2] = 0

        # Horizontal wall at 1/2 height in right room (narrow door)
        map_data[y1:y1+wc, x2+wc:W-wc] = 100
        door_cx2 = (x2 + wc + W - wc) // 2
        map_data[y1:y1+wc, door_cx2 - dw//2:door_cx2 + dw//2] = 0

        return map_data

    def get_start_pose(self) -> Tuple[float, float, float]:
        return (self.width / 6, self.height / 4, 0.0)


class Warehouse(Environment):
    """Warehouse with regular shelf rows and random clutter."""

    def __init__(
        self,
        width: float = 15.0,
        height: float = 12.0,
        wall_thickness: float = 0.3,
        resolution: float = 0.05,
        seed: int = 44
    ):
        super().__init__(width, height, resolution)
        self.wall_thickness = wall_thickness
        self.seed = seed

    def generate_map(self) -> np.ndarray:
        np.random.seed(self.seed)
        map_data = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        wc = int(self.wall_thickness / self.resolution)

        # Outer walls
        map_data[:wc, :] = 100
        map_data[-wc:, :] = 100
        map_data[:, :wc] = 100
        map_data[:, -wc:] = 100

        # Regular shelf rows (4 rows of shelves)
        shelf_width = 0.4  # meters
        shelf_length = 4.0
        aisle_width = 2.5
        margin_x = 2.0
        margin_y = 2.0

        sw = int(shelf_width / self.resolution)
        sl = int(shelf_length / self.resolution)

        y_pos = margin_y
        while y_pos + shelf_width < self.height - margin_y:
            yi = int(y_pos / self.resolution)
            # Place shelf segments along x with gaps
            x_pos = margin_x
            while x_pos + shelf_length < self.width - margin_x:
                xi = int(x_pos / self.resolution)
                map_data[yi:yi+sw, xi:xi+sl] = 100
                x_pos += shelf_length + 1.5  # gap between segments
            y_pos += aisle_width

        # Add random small clutter (5 small boxes)
        start_x, start_y, _ = self.get_start_pose()
        for _ in range(5):
            for _attempt in range(20):
                bw = np.random.uniform(0.2, 0.5)
                bh = np.random.uniform(0.2, 0.5)
                bx = np.random.uniform(1.5, self.width - 1.5 - bw)
                by = np.random.uniform(1.5, self.height - 1.5 - bh)
                if np.sqrt((bx+bw/2-start_x)**2 + (by+bh/2-start_y)**2) > 1.0:
                    bi = int(by / self.resolution)
                    bj = int(bx / self.resolution)
                    bei = int((by+bh) / self.resolution)
                    bej = int((bx+bw) / self.resolution)
                    map_data[bi:bei, bj:bej] = 100
                    break

        return map_data

    def get_start_pose(self) -> Tuple[float, float, float]:
        return (1.0, self.height / 2, 0.0)


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
        'apartment': ComplexApartment,
        'l_corridor': LShapedCorridor,
        'maze': MazeEnvironment,
        'dense_obstacles': DenseObstacles,
        'narrow_passages': NarrowPassages,
        'warehouse': Warehouse,
    }

    if env_type not in env_map:
        raise ValueError(f"Unknown environment type: {env_type}. "
                        f"Available: {list(env_map.keys())}")

    return env_map[env_type](**kwargs)
