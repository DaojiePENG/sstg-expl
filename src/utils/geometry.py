"""
Geometry utility functions for SSTG Explorer.

Provides basic geometric calculations for 2D navigation.
"""
import numpy as np
from typing import Tuple, List
import math


def compute_target_point(
    position: Tuple[float, float],
    distance: float,
    angle_deg: float
) -> Tuple[float, float]:
    """
    Compute a target point at given distance and angle from a position.

    Args:
        position: Base position (x, y).
        distance: Distance to target point.
        angle_deg: Angle in degrees (0° = East, 90° = North).

    Returns:
        Target position (x, y).
    """
    angle_rad = np.deg2rad(angle_deg)
    x = position[0] + distance * np.cos(angle_rad)
    y = position[1] + distance * np.sin(angle_rad)
    return (float(x), float(y))


def euclidean_distance(
    p1: Tuple[float, float],
    p2: Tuple[float, float]
) -> float:
    """
    Compute Euclidean distance between two points.

    Args:
        p1: First point (x, y).
        p2: Second point (x, y).

    Returns:
        Distance between p1 and p2.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


def angle_between_points(
    p1: Tuple[float, float],
    p2: Tuple[float, float]
) -> float:
    """
    Compute angle from p1 to p2 in degrees.

    Args:
        p1: Start point (x, y).
        p2: End point (x, y).

    Returns:
        Angle in degrees [0, 360) (0° = East, 90° = North).
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle_rad = math.atan2(dy, dx)
    angle_deg = np.rad2deg(angle_rad)

    # Normalize to [0, 360)
    return angle_deg % 360.0


def normalize_angle(angle_deg: float) -> float:
    """
    Normalize angle to [0, 360) range.

    Args:
        angle_deg: Angle in degrees.

    Returns:
        Normalized angle in [0, 360).
    """
    return angle_deg % 360.0


def angle_difference(angle1_deg: float, angle2_deg: float) -> float:
    """
    Compute the minimum angular difference between two angles.

    Args:
        angle1_deg: First angle in degrees.
        angle2_deg: Second angle in degrees.

    Returns:
        Angular difference in degrees [0, 180].
    """
    diff = abs(angle1_deg - angle2_deg) % 360.0
    return min(diff, 360.0 - diff)


def line_segment_intersection(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    p4: Tuple[float, float]
) -> bool:
    """
    Check if line segment p1-p2 intersects with line segment p3-p4.

    Args:
        p1: Start of first segment.
        p2: End of first segment.
        p3: Start of second segment.
        p4: End of second segment.

    Returns:
        True if segments intersect, False otherwise.
    """
    def ccw(A, B, C):
        """Check if three points are in counter-clockwise order."""
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)


def point_to_line_distance(
    point: Tuple[float, float],
    line_start: Tuple[float, float],
    line_end: Tuple[float, float]
) -> float:
    """
    Compute minimum distance from a point to a line segment.

    Args:
        point: Point (x, y).
        line_start: Start of line segment.
        line_end: End of line segment.

    Returns:
        Minimum distance from point to line segment.
    """
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    # Vector from line_start to line_end
    dx = x2 - x1
    dy = y2 - y1

    # Handle degenerate case (line_start == line_end)
    if dx == 0 and dy == 0:
        return euclidean_distance(point, line_start)

    # Parameter t of the projection of point onto the line
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))

    # Closest point on the line segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return euclidean_distance(point, (closest_x, closest_y))


def sample_line(
    start: Tuple[float, float],
    end: Tuple[float, float],
    resolution: float
) -> List[Tuple[float, float]]:
    """
    Sample points along a line segment at given resolution.

    Args:
        start: Start point (x, y).
        end: End point (x, y).
        resolution: Distance between samples.

    Returns:
        List of sampled points along the line.
    """
    dist = euclidean_distance(start, end)
    if dist < resolution:
        return [start, end]

    num_samples = int(np.ceil(dist / resolution))
    samples = []

    for i in range(num_samples + 1):
        t = i / num_samples
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        samples.append((x, y))

    return samples


def point_in_circle(
    point: Tuple[float, float],
    center: Tuple[float, float],
    radius: float
) -> bool:
    """
    Check if a point is inside a circle.

    Args:
        point: Point to check (x, y).
        center: Circle center (x, y).
        radius: Circle radius.

    Returns:
        True if point is inside circle, False otherwise.
    """
    return euclidean_distance(point, center) <= radius


def circle_overlap(
    center1: Tuple[float, float],
    radius1: float,
    center2: Tuple[float, float],
    radius2: float
) -> bool:
    """
    Check if two circles overlap.

    Args:
        center1: Center of first circle (x, y).
        radius1: Radius of first circle.
        center2: Center of second circle (x, y).
        radius2: Radius of second circle.

    Returns:
        True if circles overlap, False otherwise.
    """
    return euclidean_distance(center1, center2) <= (radius1 + radius2)


def find_longest_free_sector(
    sector_status: List[bool],
    d_theta: float
) -> Tuple[int, int, float]:
    """
    Find the longest continuous sequence of free sectors.

    Args:
        sector_status: List of booleans indicating if each sector is free.
                      True = free, False = occupied/explored.
        d_theta: Angular interval between sectors in degrees.

    Returns:
        Tuple of (start_index, end_index, center_angle):
        - start_index: Starting sector index of longest free sequence.
        - end_index: Ending sector index (inclusive) of longest free sequence.
        - center_angle: Center angle of the sequence in degrees.
        Returns (-1, -1, -1.0) if no free sectors exist.
    """
    n = len(sector_status)
    if n == 0 or not any(sector_status):
        return (-1, -1, -1.0)

    # Double the array to handle wrap-around
    doubled = sector_status + sector_status

    max_length = 0
    max_start = -1

    current_length = 0
    current_start = -1

    for i in range(2 * n):
        if doubled[i]:  # Free sector
            if current_length == 0:
                current_start = i
            current_length += 1

            # Update max if current is longer
            if current_length > max_length and i < n + max_start:
                # Avoid counting the same sequence twice
                if current_start < n:
                    max_length = current_length
                    max_start = current_start
        else:
            current_length = 0
            current_start = -1

    if max_length == 0:
        return (-1, -1, -1.0)

    # Get indices within original array
    start_idx = max_start % n
    end_idx = (max_start + max_length - 1) % n

    # Compute center angle
    start_angle = start_idx * d_theta
    end_angle = end_idx * d_theta

    # Handle wrap-around case
    if end_idx < start_idx:
        # Wrapped around 0°
        angle_span = (360 - start_angle) + end_angle
        center_angle = (start_angle + angle_span / 2) % 360
    else:
        center_angle = (start_angle + end_angle) / 2

    return (start_idx, end_idx, center_angle)


def rotation_matrix_2d(angle_deg: float) -> np.ndarray:
    """
    Create a 2D rotation matrix.

    Args:
        angle_deg: Rotation angle in degrees.

    Returns:
        2x2 rotation matrix.
    """
    angle_rad = np.deg2rad(angle_deg)
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([[c, -s], [s, c]])


def transform_point(
    point: Tuple[float, float],
    translation: Tuple[float, float],
    rotation_deg: float = 0.0
) -> Tuple[float, float]:
    """
    Apply 2D transformation (rotation + translation) to a point.

    Args:
        point: Point to transform (x, y).
        translation: Translation (dx, dy).
        rotation_deg: Rotation angle in degrees (applied first).

    Returns:
        Transformed point (x, y).
    """
    p = np.array(point)

    # Apply rotation
    if rotation_deg != 0.0:
        R = rotation_matrix_2d(rotation_deg)
        p = R @ p

    # Apply translation
    p = p + np.array(translation)

    return (float(p[0]), float(p[1]))
