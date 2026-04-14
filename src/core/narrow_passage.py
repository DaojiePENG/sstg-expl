"""
Narrow passage detection and adaptive sampling for SSTG Explorer.

Detects narrow passages using distance field and adapts exploration density.
"""
import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass

from src.map.occupancy_grid import OccupancyGrid
from src.map.distance_field import DistanceField


@dataclass
class PassageInfo:
    """Information about a narrow passage."""
    position: Tuple[float, float]  # Center position
    width: float  # Passage width in meters
    direction: float  # Passage direction in degrees
    is_narrow: bool  # Whether it's considered narrow
    recommended_d_theta: float  # Recommended angular sampling


class NarrowPassageDetector:
    """
    Detector for narrow passages that adapts exploration density.

    Uses distance field to identify narrow regions and recommends
    finer angular sampling (smaller d_theta) in these areas.
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        distance_field: DistanceField,
        robot_radius: float = 0.3,
        narrow_threshold: float = 1.0,  # Passage width threshold
        base_d_theta: float = 45.0,
        min_d_theta: float = 15.0
    ):
        """
        Initialize narrow passage detector.

        Args:
            occupancy_grid: Occupancy grid map.
            distance_field: Precomputed distance field.
            robot_radius: Robot radius in meters.
            narrow_threshold: Width threshold for narrow passages (meters).
            base_d_theta: Base angular interval (degrees).
            min_d_theta: Minimum angular interval for very narrow passages.
        """
        self.grid = occupancy_grid
        self.distance_field = distance_field
        self.robot_radius = robot_radius
        self.narrow_threshold = narrow_threshold
        self.base_d_theta = base_d_theta
        self.min_d_theta = min_d_theta

        # Passage classification thresholds
        self.very_narrow_threshold = narrow_threshold * 0.5
        self.moderately_narrow_threshold = narrow_threshold * 1.5

    def detect_passage(
        self,
        position: Tuple[float, float]
    ) -> PassageInfo:
        """
        Detect passage characteristics at a position.

        Args:
            position: Position to check (x, y) in meters.

        Returns:
            PassageInfo object with passage characteristics.
        """
        x, y = position

        # Get distance to nearest obstacle (approximate passage width / 2)
        clearance = self.distance_field.get_distance(x, y)

        # Estimate passage width (2 * clearance)
        passage_width = 2.0 * clearance

        # Determine if narrow
        is_narrow = passage_width < self.narrow_threshold

        # Compute passage direction (perpendicular to gradient)
        distance, grad_x, grad_y = self.distance_field.get_distance(
            x, y, return_gradient=True
        )

        # Passage direction is perpendicular to gradient
        # Gradient points towards obstacles, passage is perpendicular
        passage_angle = np.arctan2(-grad_x, grad_y)  # In radians
        passage_direction = np.degrees(passage_angle) % 360

        # Recommend d_theta based on passage width
        recommended_d_theta = self._compute_adaptive_d_theta(passage_width)

        return PassageInfo(
            position=position,
            width=passage_width,
            direction=passage_direction,
            is_narrow=is_narrow,
            recommended_d_theta=recommended_d_theta
        )

    def _compute_adaptive_d_theta(self, passage_width: float) -> float:
        """
        Compute adaptive angular sampling based on passage width.

        Narrower passages require finer angular resolution to ensure
        frontiers can find the passage direction.

        Args:
            passage_width: Width of passage in meters.

        Returns:
            Recommended d_theta in degrees.
        """
        if passage_width < self.very_narrow_threshold:
            # Very narrow: use finest resolution
            return self.min_d_theta

        elif passage_width < self.narrow_threshold:
            # Narrow: interpolate between min and base
            ratio = (passage_width - self.very_narrow_threshold) / \
                    (self.narrow_threshold - self.very_narrow_threshold)
            d_theta = self.min_d_theta + ratio * (self.base_d_theta - self.min_d_theta)
            return d_theta

        elif passage_width < self.moderately_narrow_threshold:
            # Moderately narrow: slightly finer than base
            ratio = (passage_width - self.narrow_threshold) / \
                    (self.moderately_narrow_threshold - self.narrow_threshold)
            d_theta = 0.75 * self.base_d_theta + ratio * 0.25 * self.base_d_theta
            return d_theta

        else:
            # Wide open: use base resolution
            return self.base_d_theta

    def get_adaptive_directions(
        self,
        position: Tuple[float, float],
        base_d_theta: Optional[float] = None
    ) -> List[float]:
        """
        Get adaptive direction angles for frontier generation.

        Args:
            position: Current position (x, y).
            base_d_theta: Base angular interval (uses config if None).

        Returns:
            List of angles in degrees [0, 360).
        """
        if base_d_theta is None:
            base_d_theta = self.base_d_theta

        # Detect passage characteristics
        passage = self.detect_passage(position)

        # Use recommended d_theta
        d_theta = passage.recommended_d_theta

        # Generate angles
        n_directions = int(360 / d_theta)
        angles = [i * d_theta for i in range(n_directions)]

        return angles

    def is_in_narrow_passage(
        self,
        position: Tuple[float, float],
        threshold_multiplier: float = 1.0
    ) -> bool:
        """
        Check if a position is in a narrow passage.

        Args:
            position: Position to check (x, y).
            threshold_multiplier: Multiplier for narrow threshold.

        Returns:
            True if position is in a narrow passage.
        """
        passage = self.detect_passage(position)
        adjusted_threshold = self.narrow_threshold * threshold_multiplier
        return passage.width < adjusted_threshold

    def find_narrow_regions(
        self,
        sample_spacing: float = 0.5
    ) -> List[Tuple[float, float]]:
        """
        Find all narrow regions in the map.

        Args:
            sample_spacing: Spacing between sample points (meters).

        Returns:
            List of positions in narrow regions.
        """
        narrow_positions = []

        # Sample the map at regular intervals
        x_min, x_max = self.grid.origin[0], \
                       self.grid.origin[0] + self.grid.world_width
        y_min, y_max = self.grid.origin[1], \
                       self.grid.origin[1] + self.grid.world_height

        x_samples = np.arange(x_min, x_max, sample_spacing)
        y_samples = np.arange(y_min, y_max, sample_spacing)

        for x in x_samples:
            for y in y_samples:
                # Check if position is free space
                if not self.distance_field.is_collision_free(x, y, 0.0):
                    continue

                # Check if it's a narrow passage
                passage = self.detect_passage((x, y))

                if passage.is_narrow:
                    narrow_positions.append((x, y))

        return narrow_positions

    def get_passage_width_map(self) -> np.ndarray:
        """
        Get map of passage widths (2 * distance field).

        Returns:
            Array of passage widths in meters.
        """
        return 2.0 * self.distance_field.distance_field

    def visualize_narrow_regions(
        self,
        threshold_multiplier: float = 1.0
    ) -> np.ndarray:
        """
        Create binary map of narrow regions for visualization.

        Args:
            threshold_multiplier: Multiplier for narrow threshold.

        Returns:
            Binary array (True = narrow region).
        """
        passage_width_map = self.get_passage_width_map()
        adjusted_threshold = self.narrow_threshold * threshold_multiplier

        # Mark narrow regions (free space with width < threshold)
        free_space = self.distance_field.distance_field > 0
        narrow_map = (passage_width_map < adjusted_threshold) & free_space

        return narrow_map

    def get_statistics(self) -> dict:
        """
        Get statistics about narrow passages in the map.

        Returns:
            Dictionary with passage statistics.
        """
        # Sample narrow regions
        narrow_positions = self.find_narrow_regions(sample_spacing=0.5)

        passage_widths = []
        for pos in narrow_positions:
            passage = self.detect_passage(pos)
            passage_widths.append(passage.width)

        if len(passage_widths) == 0:
            return {
                'num_narrow_regions': 0,
                'min_width': 0.0,
                'max_width': 0.0,
                'mean_width': 0.0,
                'narrowest_position': None
            }

        passage_widths = np.array(passage_widths)
        min_idx = np.argmin(passage_widths)

        stats = {
            'num_narrow_regions': len(narrow_positions),
            'min_width': float(passage_widths.min()),
            'max_width': float(passage_widths.max()),
            'mean_width': float(passage_widths.mean()),
            'std_width': float(passage_widths.std()),
            'narrowest_position': narrow_positions[min_idx]
        }

        return stats


class AdaptiveExplorationHelper:
    """
    Helper class for adaptive exploration with narrow passage support.

    Provides utilities to adjust exploration parameters based on
    local passage characteristics.
    """

    def __init__(
        self,
        detector: NarrowPassageDetector,
        enable_adaptive: bool = True
    ):
        """
        Initialize adaptive exploration helper.

        Args:
            detector: Narrow passage detector instance.
            enable_adaptive: Whether to enable adaptive behavior.
        """
        self.detector = detector
        self.enable_adaptive = enable_adaptive

    def get_sampling_density(
        self,
        position: Tuple[float, float],
        base_n_directions: int = 8
    ) -> int:
        """
        Get recommended number of sampling directions at a position.

        Args:
            position: Current position.
            base_n_directions: Base number of directions.

        Returns:
            Recommended number of directions.
        """
        if not self.enable_adaptive:
            return base_n_directions

        passage = self.detector.detect_passage(position)

        # Scale number of directions based on passage width
        if passage.is_narrow:
            # In narrow passages, double or triple the sampling
            base_d_theta = 360.0 / base_n_directions
            adaptive_d_theta = passage.recommended_d_theta

            scale_factor = base_d_theta / adaptive_d_theta
            n_directions = int(base_n_directions * scale_factor)

            return n_directions
        else:
            return base_n_directions

    def should_refine_local_search(
        self,
        position: Tuple[float, float]
    ) -> bool:
        """
        Check if local search refinement is recommended.

        Args:
            position: Current position.

        Returns:
            True if refinement is recommended.
        """
        if not self.enable_adaptive:
            return False

        return self.detector.is_in_narrow_passage(position)


def create_narrow_passage_detector(
    occupancy_grid: OccupancyGrid,
    distance_field: DistanceField,
    robot_radius: float = 0.3,
    narrow_threshold: float = 1.0,
    base_d_theta: float = 45.0
) -> NarrowPassageDetector:
    """
    Factory function to create narrow passage detector.

    Args:
        occupancy_grid: Occupancy grid map.
        distance_field: Precomputed distance field.
        robot_radius: Robot radius in meters.
        narrow_threshold: Width threshold for narrow passages.
        base_d_theta: Base angular interval.

    Returns:
        Configured detector instance.
    """
    return NarrowPassageDetector(
        occupancy_grid,
        distance_field,
        robot_radius,
        narrow_threshold,
        base_d_theta
    )
