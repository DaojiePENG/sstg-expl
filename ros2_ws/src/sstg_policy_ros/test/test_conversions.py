import math

import pytest
from nav_msgs.msg import OccupancyGrid

from sstg_policy_ros.conversions import (
    occupancy_grid_from_msg,
    quaternion_to_yaw,
    target_pose_message,
)


def _message():
    message = OccupancyGrid()
    message.info.width = 3
    message.info.height = 2
    message.info.resolution = 0.5
    message.info.origin.position.x = -1.0
    message.info.origin.position.y = -2.0
    message.info.origin.orientation.w = 1.0
    message.data = [-1, 0, 100, 4, 5, 6]
    return message


def test_occupancy_conversion_preserves_row_major_values_and_origin():
    grid = occupancy_grid_from_msg(_message())
    assert grid.shape == (2, 3)
    assert grid.origin == (-1.0, -2.0)
    assert grid.data.tolist() == [[-1, 0, 100], [4, 5, 6]]


def test_occupancy_conversion_rejects_rotated_origin_and_bad_length():
    message = _message()
    message.info.origin.orientation.z = math.sin(0.1)
    message.info.origin.orientation.w = math.cos(0.1)
    with pytest.raises(ValueError, match="rotated"):
        occupancy_grid_from_msg(message)

    message = _message()
    message.data.pop()
    with pytest.raises(ValueError, match="expected 6"):
        occupancy_grid_from_msg(message)


def test_target_pose_heading_round_trip():
    pose = target_pose_message(1.0, 2.0, 135.0, "map", None)
    yaw = quaternion_to_yaw(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    assert math.degrees(yaw) == pytest.approx(135.0)
