#!/usr/bin/env python3
"""Render the recorded task-depth stream as an auditable H.264 video.

The input is the frozen ``/task_camera/image_raw`` 32FC1 stream in a system
simulation core MCAP.  Frames use a fixed 0.05--5.0 m color scale and carry a
development-simulation label.  Existing output is never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any, Sequence

import numpy as np

try:
    from scripts.render_system_sim_bag_media import (
        RenderError,
        _protected_child,
        load_run_context,
    )
except ModuleNotFoundError as error:
    if error.name != "scripts":
        raise
    from render_system_sim_bag_media import (  # type: ignore[no-redef]
        RenderError,
        _protected_child,
        load_run_context,
    )


OUTPUT_SCHEMA = "sstg_system_sim_depth_video_render/v1"
DEPTH_TOPIC = "/task_camera/image_raw"
DEPTH_TOPIC_TYPE = "sensor_msgs/msg/Image"
VIDEO_NAME = "task_camera_depth.mp4"
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 560
CAMERA_HEIGHT = 480
DEFAULT_DEPTH_MIN_M = 0.05
DEFAULT_DEPTH_MAX_M = 5.0
DEFAULT_FPS = 5.0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def depth_array_from_message(message: Any) -> np.ndarray:
    """Decode one ROS Image while respecting byte order and row stride."""
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    if width <= 0 or height <= 0:
        raise RenderError("task-depth image dimensions must be positive")
    if str(message.encoding) != "32FC1":
        raise RenderError(
            f"task-depth encoding must be 32FC1, got {message.encoding!r}"
        )
    if int(message.is_bigendian) not in (0, 1):
        raise RenderError("task-depth is_bigendian must be zero or one")
    if step < width * 4 or step % 4 != 0:
        raise RenderError("task-depth row stride is inconsistent with 32FC1")
    payload = memoryview(message.data)
    if len(payload) != height * step:
        raise RenderError("task-depth payload size disagrees with height and step")
    dtype = ">f4" if int(message.is_bigendian) else "<f4"
    values = np.frombuffer(payload, dtype=dtype).reshape((height, step // 4))
    return np.asarray(values[:, :width], dtype=np.float32).copy()


def colorize_depth(
    depth_m: np.ndarray,
    *,
    depth_min_m: float,
    depth_max_m: float,
) -> np.ndarray:
    """Map metric depth to RGB with invalid pixels fixed to black."""
    if (
        not math.isfinite(depth_min_m)
        or not math.isfinite(depth_max_m)
        or depth_min_m < 0.0
        or depth_max_m <= depth_min_m
    ):
        raise RenderError("depth color limits must be finite and increasing")
    values = np.asarray(depth_m, dtype=np.float32)
    if values.ndim != 2 or values.size == 0:
        raise RenderError("task-depth frame must be a non-empty 2D array")
    valid = np.isfinite(values) & (values > 0.0)
    normalized = np.clip(
        (values - depth_min_m) / (depth_max_m - depth_min_m), 0.0, 1.0
    )
    from matplotlib import colormaps

    rgb = np.asarray(colormaps["turbo"](normalized, bytes=True)[..., :3]).copy()
    rgb[~valid] = 0
    return rgb


def _font(size: int) -> Any:
    from PIL import ImageFont

    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def compose_video_frame(
    rgb: np.ndarray,
    *,
    caption: str,
    elapsed_s: float,
    depth_min_m: float,
    depth_max_m: float,
) -> np.ndarray:
    """Upscale a depth frame and attach stable labels plus a metric legend."""
    from PIL import Image, ImageDraw

    frame = Image.fromarray(np.asarray(rgb, dtype=np.uint8), mode="RGB")
    frame = frame.resize((OUTPUT_WIDTH, CAMERA_HEIGHT), Image.Resampling.NEAREST)
    canvas = Image.new("RGB", (OUTPUT_WIDTH, OUTPUT_HEIGHT), "#111827")
    canvas.paste(frame, (0, 40))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (12, 9),
        f"DEVELOPMENT SIMULATION | depth | {caption} | t={elapsed_s:06.2f}s",
        fill="white",
        font=_font(16),
    )

    gradient_values = np.linspace(0.0, 1.0, 360, dtype=np.float32)[None, :]
    from matplotlib import colormaps

    gradient = colormaps["turbo"](gradient_values, bytes=True)[..., :3]
    gradient_image = Image.fromarray(gradient, mode="RGB").resize((360, 14))
    canvas.paste(gradient_image, (18, 530))
    draw.rectangle((18, 530, 378, 544), outline="white", width=1)
    draw.text(
        (392, 526),
        f"{depth_min_m:.2f} m to {depth_max_m:.2f} m | invalid: black",
        fill="white",
        font=_font(13),
    )
    return np.asarray(canvas, dtype=np.uint8)


def video_output_path(run_dir: Path) -> Path:
    run = Path(run_dir).resolve()
    output = _protected_child(
        run,
        run / "media" / "raw" / VIDEO_NAME,
        "task-depth video output",
        must_exist=False,
    )
    if os.path.lexists(output):
        raise RenderError(f"refusing to overwrite existing media: {output}")
    return output


def _video_tools() -> tuple[str, str]:
    ffmpeg = shutil.which("ffmpeg")
    ffprobe = shutil.which("ffprobe")
    if ffmpeg is None or ffprobe is None:
        raise RenderError("ffmpeg and ffprobe are required for depth-video rendering")
    return ffmpeg, ffprobe


def _probe_video(ffprobe: str, path: Path, expected_frames: int) -> dict[str, Any]:
    command = [
        ffprobe,
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-count_frames",
        "-show_entries",
        "stream=codec_name,width,height,nb_read_frames,duration",
        "-of",
        "json",
        str(path),
    ]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        raise RenderError(f"ffprobe rejected rendered video: {result.stderr.strip()}")
    try:
        payload = json.loads(result.stdout)
        stream = payload["streams"][0]
        frames = int(stream["nb_read_frames"])
    except (IndexError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise RenderError("ffprobe returned incomplete video metadata") from error
    if stream.get("codec_name") != "h264":
        raise RenderError("rendered task-depth video is not H.264")
    if int(stream.get("width", 0)) != OUTPUT_WIDTH:
        raise RenderError("rendered task-depth video width is incorrect")
    if int(stream.get("height", 0)) != OUTPUT_HEIGHT:
        raise RenderError("rendered task-depth video height is incorrect")
    if frames != expected_frames:
        raise RenderError("rendered task-depth frame count is incorrect")
    return {
        "codec": "h264",
        "width": OUTPUT_WIDTH,
        "height": OUTPUT_HEIGHT,
        "frame_count": frames,
        "duration_s": float(stream["duration"]),
    }


def render_depth_video(
    run_dir: Path | str,
    *,
    bag: Path | str | None = None,
    depth_min_m: float = DEFAULT_DEPTH_MIN_M,
    depth_max_m: float = DEFAULT_DEPTH_MAX_M,
    fps: float = DEFAULT_FPS,
) -> dict[str, Any]:
    if not math.isfinite(fps) or fps <= 0.0:
        raise RenderError("video fps must be finite and positive")
    context = load_run_context(run_dir, bag)
    output = video_output_path(context.run_dir)
    ffmpeg, ffprobe = _video_tools()
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import (
            ConverterOptions,
            SequentialReader,
            StorageFilter,
            StorageOptions,
        )
        from sensor_msgs.msg import Image as ImageMessage
    except ImportError as error:
        raise RenderError(
            "ROS bag runtime unavailable; source the ROS 2 Jazzy environment"
        ) from error

    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=str(context.bag_dir), storage_id="mcap"),
            ConverterOptions("cdr", "cdr"),
        )
    except Exception as error:
        raise RenderError(f"cannot open core MCAP bag: {error}") from error
    topic_types = {
        metadata.name: metadata.type for metadata in reader.get_all_topics_and_types()
    }
    if topic_types.get(DEPTH_TOPIC) != DEPTH_TOPIC_TYPE:
        raise RenderError(
            f"required bag topic {DEPTH_TOPIC} must have type {DEPTH_TOPIC_TYPE}"
        )
    reader.set_filter(StorageFilter(topics=[DEPTH_TOPIC]))

    output.parent.mkdir(parents=True, exist_ok=True)
    output = video_output_path(context.run_dir)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".mp4", dir=output.parent
    )
    os.close(descriptor)
    os.unlink(temporary_name)
    temporary = Path(temporary_name)
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-video_size",
        f"{OUTPUT_WIDTH}x{OUTPUT_HEIGHT}",
        "-framerate",
        f"{fps:g}",
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(temporary),
    ]
    process: subprocess.Popen[bytes] | None = None
    first_timestamp_ns: int | None = None
    last_timestamp_ns: int | None = None
    frame_count = 0
    caption = (
        f"{context.identity.get('world_id', 'unknown')} | "
        f"seed {context.identity.get('replicate_seed', 'unknown')}"
    )
    try:
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        if process.stdin is None or process.stderr is None:
            raise RenderError("failed to open ffmpeg pipes")
        while reader.has_next():
            topic, encoded, timestamp_ns = reader.read_next()
            if topic != DEPTH_TOPIC:
                raise RenderError(f"unexpected filtered bag topic: {topic}")
            timestamp_ns = int(timestamp_ns)
            if last_timestamp_ns is not None and timestamp_ns < last_timestamp_ns:
                raise RenderError("task-depth bag timestamps are not ordered")
            if first_timestamp_ns is None:
                first_timestamp_ns = timestamp_ns
            last_timestamp_ns = timestamp_ns
            message = deserialize_message(encoded, ImageMessage)
            depth = depth_array_from_message(message)
            rgb = colorize_depth(
                depth,
                depth_min_m=depth_min_m,
                depth_max_m=depth_max_m,
            )
            frame = compose_video_frame(
                rgb,
                caption=caption,
                elapsed_s=(timestamp_ns - first_timestamp_ns) / 1e9,
                depth_min_m=depth_min_m,
                depth_max_m=depth_max_m,
            )
            process.stdin.write(frame.tobytes(order="C"))
            frame_count += 1
        process.stdin.close()
        stderr = process.stderr.read().decode("utf-8", errors="replace")
        returncode = process.wait()
        if returncode != 0:
            raise RenderError(f"ffmpeg failed with code {returncode}: {stderr.strip()}")
        if frame_count < 2 or first_timestamp_ns is None or last_timestamp_ns is None:
            raise RenderError("task-depth stream must contain at least two frames")
        sim_duration_s = (last_timestamp_ns - first_timestamp_ns) / 1e9
        expected_duration_s = (frame_count - 1) / fps
        tolerance_s = max(0.25, expected_duration_s * 0.05)
        if abs(sim_duration_s - expected_duration_s) > tolerance_s:
            raise RenderError(
                "task-depth timestamps disagree with the frozen video frame rate"
            )
        probe = _probe_video(ffprobe, temporary, frame_count)
        with temporary.open("rb") as stream:
            header = stream.read(12)
        if len(header) < 12 or header[4:8] != b"ftyp":
            raise RenderError("rendered task-depth output is not an MP4 file")
        output_sha256 = sha256_file(temporary)
        output_bytes = temporary.stat().st_size
        try:
            os.link(temporary, output)
        except FileExistsError as error:
            raise RenderError(f"refusing to overwrite existing media: {output}") from error
        result = {
            "schema": OUTPUT_SCHEMA,
            "evidence_label": "Development simulation evidence",
            "output": str(output),
            "sha256": output_sha256,
            "bytes": output_bytes,
            "topic": DEPTH_TOPIC,
            "topic_type": DEPTH_TOPIC_TYPE,
            "encoding": "32FC1",
            "depth_range_m": [depth_min_m, depth_max_m],
            "fps": fps,
            "simulation_duration_s": sim_duration_s,
            **probe,
        }
        return result
    except BrokenPipeError as error:
        details = ""
        if process is not None and process.stderr is not None:
            details = process.stderr.read().decode("utf-8", errors="replace").strip()
        raise RenderError(f"ffmpeg closed its input early: {details}") from error
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--bag",
        type=Path,
        help="Core bag directory relative to run_dir (default: bags/core)",
    )
    parser.add_argument("--depth-min-m", type=float, default=DEFAULT_DEPTH_MIN_M)
    parser.add_argument("--depth-max-m", type=float, default=DEFAULT_DEPTH_MAX_M)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = render_depth_video(
            args.run_dir,
            bag=args.bag,
            depth_min_m=args.depth_min_m,
            depth_max_m=args.depth_max_m,
            fps=args.fps,
        )
    except (OSError, RenderError) as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
