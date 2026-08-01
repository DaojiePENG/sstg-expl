"""Strict conversions between ROS messages and the SSTG policy core."""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Transform
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg

from sstg_explorer.map import OccupancyGrid


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw in radians from a normalized or near-normalized quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return an XYZW quaternion for planar yaw in radians."""
    half = float(yaw) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def occupancy_grid_from_msg(
    message: OccupancyGridMsg,
    expected_resolution: Optional[float] = None,
    origin_yaw_tolerance: float = 1e-6,
) -> OccupancyGrid:
    """Convert a row-major ROS occupancy grid without changing evidence values."""
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    if width <= 0 or height <= 0:
        raise ValueError("occupancy grid dimensions must be positive")
    if resolution <= 0.0:
        raise ValueError("occupancy grid resolution must be positive")
    if len(message.data) != width * height:
        raise ValueError(
            f"occupancy data has {len(message.data)} cells; expected {width * height}"
        )
    if expected_resolution is not None and not math.isclose(
        resolution, float(expected_resolution), rel_tol=0.0, abs_tol=1e-12
    ):
        raise ValueError("occupancy grid resolution changed within a run")

    orientation = message.info.origin.orientation
    origin_yaw = quaternion_to_yaw(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    if abs(origin_yaw) > origin_yaw_tolerance:
        raise ValueError(
            "rotated occupancy-grid origins are not supported by the policy core"
        )
    raw = np.asarray(message.data, dtype=np.int16)
    if np.any((raw < -1) | (raw > 100)):
        raise ValueError("occupancy values must remain within ROS range [-1, 100]")
    data = raw.astype(np.int8, copy=True).reshape((height, width))
    return OccupancyGrid(
        data=data,
        resolution=resolution,
        origin=(
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
        ),
    )


def pose2d_from_transform(transform: Transform) -> Tuple[float, float, float]:
    """Convert a ROS transform to ``(x, y, heading_degrees)``."""
    rotation = transform.rotation
    yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
    return (
        float(transform.translation.x),
        float(transform.translation.y),
        math.degrees(yaw) % 360.0,
    )


def target_pose_message(
    x: float,
    y: float,
    heading_degrees: float,
    frame_id: str,
    stamp,
) -> PoseStamped:
    """Build a planar Nav2 target pose."""
    message = PoseStamped()
    message.header.frame_id = str(frame_id)
    if stamp is not None:
        message.header.stamp = stamp
    message.pose = Pose()
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    qx, qy, qz, qw = yaw_to_quaternion(math.radians(heading_degrees))
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    message.pose.orientation.w = qw
    return message
