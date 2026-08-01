"""Tests for offline task-depth video rendering from the core MCAP."""
from __future__ import annotations

from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from scripts.render_system_sim_bag_media import RenderError
from scripts.render_system_sim_depth_video import (
    VIDEO_NAME,
    colorize_depth,
    depth_array_from_message,
    render_depth_video,
)


def test_depth_array_respects_stride_and_byte_order() -> None:
    little = np.asarray([[1.0, 2.0, 99.0], [3.0, 4.0, 99.0]], dtype="<f4")
    message = SimpleNamespace(
        width=2,
        height=2,
        step=12,
        encoding="32FC1",
        is_bigendian=0,
        data=little.tobytes(),
    )
    np.testing.assert_allclose(
        depth_array_from_message(message),
        np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32),
    )

    big = np.asarray([[5.0, 6.0], [7.0, 8.0]], dtype=">f4")
    message.step = 8
    message.is_bigendian = 1
    message.data = big.tobytes()
    np.testing.assert_allclose(depth_array_from_message(message), big)


def test_depth_array_and_color_limits_fail_closed() -> None:
    message = SimpleNamespace(
        width=2,
        height=2,
        step=8,
        encoding="rgb8",
        is_bigendian=0,
        data=b"\0" * 16,
    )
    with pytest.raises(RenderError, match="encoding must be 32FC1"):
        depth_array_from_message(message)
    with pytest.raises(RenderError, match="finite and increasing"):
        colorize_depth(
            np.ones((2, 2)), depth_min_m=5.0, depth_max_m=1.0
        )


def test_colorize_depth_marks_invalid_pixels_black() -> None:
    depth = np.asarray([[0.0, 0.5], [2.0, np.inf]], dtype=np.float32)
    rgb = colorize_depth(depth, depth_min_m=0.05, depth_max_m=5.0)

    assert rgb.shape == (2, 2, 3)
    assert rgb.dtype == np.uint8
    assert np.array_equal(rgb[0, 0], [0, 0, 0])
    assert np.array_equal(rgb[1, 1], [0, 0, 0])
    assert not np.array_equal(rgb[0, 1], rgb[1, 0])


def _write_depth_run(tmp_path: Path) -> Path:
    rosbag2_py = pytest.importorskip("rosbag2_py")
    rclpy_serialization = pytest.importorskip("rclpy.serialization")
    sensor_messages = pytest.importorskip("sensor_msgs.msg")

    run = tmp_path / "run"
    run.mkdir()
    manifest = {
        "schema": "sstg_system_sim_run_launch/v1",
        "study_id": "depth_video_test",
        "schedule_id": "depth_video_test__seed_1",
        "launch": {
            "arguments": {
                "truth_registration_id": "test:identity",
                "truth_to_map_x_m": "0",
                "truth_to_map_y_m": "0",
                "truth_to_map_yaw_rad": "0",
            }
        },
        "identity": {
            "world_id": "test_world",
            "method": "sstg",
            "condition": "nominal",
            "replicate_seed": "1",
        },
        "execution": {"status": "terminal_completed"},
    }
    (run / "run_launch_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )

    bag = run / "bags/core"
    bag.parent.mkdir()
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0,
            name="/task_camera/image_raw",
            type="sensor_msgs/msg/Image",
            serialization_format="cdr",
        )
    )
    for index in range(3):
        depth = np.asarray(
            [[0.5 + index, 1.0], [2.0, np.inf]], dtype="<f4"
        )
        message = sensor_messages.Image()
        message.width = 2
        message.height = 2
        message.encoding = "32FC1"
        message.is_bigendian = 0
        message.step = 8
        message.data = depth.tobytes()
        writer.write(
            "/task_camera/image_raw",
            rclpy_serialization.serialize_message(message),
            1_000_000 + index * 200_000_000,
        )
    writer.close()
    return run


@pytest.mark.skipif(
    shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None,
    reason="ffmpeg runtime is unavailable",
)
def test_render_depth_video_creates_verified_h264_and_refuses_overwrite(
    tmp_path: Path,
) -> None:
    run = _write_depth_run(tmp_path)

    result = render_depth_video(run)

    output = run / "media/raw" / VIDEO_NAME
    assert output.is_file()
    assert output.stat().st_size == result["bytes"]
    assert result["codec"] == "h264"
    assert result["frame_count"] == 3
    assert result["width"] == 640
    assert result["height"] == 560
    assert len(result["sha256"]) == 64
    with pytest.raises(RenderError, match="refusing to overwrite"):
        render_depth_video(run)
