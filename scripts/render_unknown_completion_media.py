#!/usr/bin/env python3
"""Render the sole ROS2/Gazebo unknown-completion evidence bundle per run.

The renderer combines the recorded belief-map timeline, belief-only policy
trace, evaluator-only truth metrics, and registered truth map.  It never feeds
truth back to the policy.  Existing output is never overwritten.
"""
from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Mapping, Sequence

import numpy as np

try:
    from scripts.render_system_sim_bag_media import (
        BagEvidence,
        GridSnapshot,
        RenderError,
        RunContext,
        _protected_child,
        grid_from_message,
        load_run_context,
        read_core_bag,
        transform_truth_points,
    )
except ModuleNotFoundError as error:
    if error.name != "scripts":
        raise
    from render_system_sim_bag_media import (  # type: ignore[no-redef]
        BagEvidence,
        GridSnapshot,
        RenderError,
        RunContext,
        _protected_child,
        grid_from_message,
        load_run_context,
        read_core_bag,
        transform_truth_points,
    )


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_SOURCE = ROOT / "ros2_ws/src/sstg_system_eval"
if str(EVALUATOR_SOURCE) not in sys.path:
    sys.path.insert(0, str(EVALUATOR_SOURCE))
from sstg_system_eval.metrics import (  # noqa: E402
    BeliefGrid,
    TruthGrid,
    load_truth_map,
    transform_truth_grid,
)


OUTPUT_SCHEMA = "sstg_unknown_completion_media/v1"
TRACE_SCHEMA = "sstg_unknown_completion_endpoint_trace/v1"
OUTPUT_DIRECTORY = "unknown_completion"
FRAME_WIDTH_PX = 1440
FRAME_HEIGHT_PX = 960


@dataclass(frozen=True)
class TraceEvent:
    event: str
    ros_time_ns: int
    payload: Mapping[str, Any]


@dataclass(frozen=True)
class CoverageSample:
    ros_time_ns: int
    distance_m: float
    sensor: float
    topological: float

    @property
    def joint(self) -> float:
        return min(self.sensor, self.topological)


@dataclass(frozen=True)
class Endpoint:
    decision_id: int
    ros_time_ns: int
    commanded_pose: tuple[float, float, float] | None
    reached_pose: tuple[float, float, float]
    succeeded: bool
    reason: str
    topological_node_created: bool
    cumulative_distance_m: float


def _finite_pose(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise RenderError(f"{label} must contain x, y and heading")
    if len(value) < 3:
        raise RenderError(f"{label} must contain x, y and heading")
    pose = tuple(float(item) for item in value[:3])
    if not all(math.isfinite(item) for item in pose):
        raise RenderError(f"{label} contains non-finite values")
    return pose


def read_jsonl_events(path: Path, *, label: str) -> list[TraceEvent]:
    if not path.is_file():
        raise RenderError(f"missing {label}: {path}")
    events: list[TraceEvent] = []
    previous_time = -1
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            value = json.loads(raw)
        except json.JSONDecodeError as error:
            raise RenderError(f"invalid {label} JSON on line {line_number}") from error
        if not isinstance(value, Mapping) or not isinstance(value.get("payload"), Mapping):
            raise RenderError(f"invalid {label} event on line {line_number}")
        timestamp = int(value.get("ros_time_ns", -1))
        if timestamp < previous_time:
            raise RenderError(f"{label} timestamps are not ordered")
        previous_time = timestamp
        events.append(TraceEvent(str(value.get("event", "")), timestamp, value["payload"]))
    if not events:
        raise RenderError(f"{label} is empty")
    return events


def policy_trace(run_dir: Path) -> list[TraceEvent]:
    return read_jsonl_events(run_dir / "policy_trace.jsonl", label="policy trace")


def coverage_trace(run_dir: Path) -> list[CoverageSample]:
    events = read_jsonl_events(
        run_dir / "evaluation_metrics.jsonl", label="evaluation metrics"
    )
    samples: list[CoverageSample] = []
    for event in events:
        if event.event != "metrics_snapshot":
            continue
        payload = event.payload
        core = payload.get("core_policy")
        motion = payload.get("ground_truth_motion")
        if not isinstance(core, Mapping) or not isinstance(motion, Mapping):
            continue
        sensor_block = core.get("truth_sensor")
        topology_block = core.get("truth_topological")
        if not isinstance(sensor_block, Mapping) or not isinstance(topology_block, Mapping):
            continue
        values = (
            motion.get("ground_truth_path_length_m"),
            sensor_block.get("truth_sensor_coverage"),
            topology_block.get("topological_coverage"),
        )
        if any(value is None for value in values):
            continue
        distance, sensor, topology = (float(value) for value in values)
        if not all(math.isfinite(value) for value in (distance, sensor, topology)):
            continue
        if not (distance >= 0.0 and 0.0 <= sensor <= 1.0 and 0.0 <= topology <= 1.0):
            continue
        sample = CoverageSample(event.ros_time_ns, distance, sensor, topology)
        if samples and math.isclose(samples[-1].distance_m, distance, abs_tol=1e-9):
            samples[-1] = sample
        else:
            samples.append(sample)
    if not samples:
        raise RenderError("evaluation metrics contain no available coverage samples")
    return samples


def extract_decisions(events: Sequence[TraceEvent]) -> list[TraceEvent]:
    decisions = [event for event in events if event.event == "decision"]
    ids = [int(event.payload.get("decision_id", -1)) for event in decisions]
    if ids != sorted(ids) or len(ids) != len(set(ids)):
        raise RenderError("policy decision IDs must be unique and ordered")
    return decisions


def extract_endpoints(events: Sequence[TraceEvent]) -> list[Endpoint]:
    starts = [event for event in events if event.event == "session_started"]
    if len(starts) != 1:
        raise RenderError("policy trace must contain exactly one session_started")
    start_nodes = starts[0].payload.get("nodes")
    if not isinstance(start_nodes, list) or not start_nodes:
        raise RenderError("session_started lacks the initial node")
    start_node = start_nodes[0]
    start_pose = _finite_pose(
        [*start_node.get("position", []), start_node.get("orientation", 0.0)],
        "initial node",
    )
    endpoints = [Endpoint(
        decision_id=0,
        ros_time_ns=starts[0].ros_time_ns,
        commanded_pose=None,
        reached_pose=start_pose,
        succeeded=True,
        reason="session_started",
        topological_node_created=True,
        cumulative_distance_m=0.0,
    )]
    cumulative = 0.0
    for event in events:
        if event.event != "execution":
            continue
        payload = event.payload
        decision_id = int(payload.get("decision_id", -1))
        if decision_id <= 0:
            raise RenderError("execution lacks a positive decision_id")
        translation = float(payload.get("translation_m", 0.0))
        if not math.isfinite(translation) or translation < 0.0:
            raise RenderError("execution translation must be finite and non-negative")
        cumulative += translation
        commanded_raw = payload.get("commanded_pose")
        endpoints.append(Endpoint(
            decision_id=decision_id,
            ros_time_ns=event.ros_time_ns,
            commanded_pose=(
                None if commanded_raw is None
                else _finite_pose(commanded_raw, "commanded pose")
            ),
            reached_pose=_finite_pose(payload.get("reached_pose"), "reached pose"),
            succeeded=bool(payload.get("succeeded", False)),
            reason=str(payload.get("reason", "")),
            topological_node_created=bool(
                payload.get("topological_node_created", False)
            ),
            cumulative_distance_m=cumulative,
        ))
    return endpoints


def read_map_timeline(context: RunContext) -> list[GridSnapshot]:
    try:
        from nav_msgs.msg import OccupancyGrid
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    except ImportError as error:
        raise RenderError("ROS bag runtime unavailable; source the ROS 2 workspace") from error
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(context.bag_dir), storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic_types.get("/map") != "nav_msgs/msg/OccupancyGrid":
        raise RenderError("core bag lacks nav_msgs/msg/OccupancyGrid on /map")
    reader.set_filter(StorageFilter(topics=["/map"]))
    timeline: list[GridSnapshot] = []
    while reader.has_next():
        topic, encoded, timestamp_ns = reader.read_next()
        if topic != "/map":
            raise RenderError(f"unexpected filtered topic: {topic}")
        timeline.append(
            grid_from_message(
                deserialize_message(encoded, OccupancyGrid), int(timestamp_ns)
            )
        )
    if not timeline:
        raise RenderError("core bag contains no map snapshots")
    return timeline


def snapshot_at(timeline: Sequence[GridSnapshot], ros_time_ns: int) -> GridSnapshot:
    timestamps = [item.bag_timestamp_ns for item in timeline]
    index = max(0, bisect.bisect_right(timestamps, ros_time_ns) - 1)
    return timeline[index]


def coverage_at(samples: Sequence[CoverageSample], ros_time_ns: int) -> CoverageSample:
    timestamps = [item.ros_time_ns for item in samples]
    index = min(len(samples) - 1, bisect.bisect_left(timestamps, ros_time_ns))
    return samples[index]


def _project_belief_on_truth(
    truth: TruthGrid, belief: GridSnapshot
) -> tuple[np.ndarray, np.ndarray]:
    height, width = truth.shape
    rows, columns = np.indices((height, width), dtype=np.float64)
    local_x = (columns + 0.5) * truth.resolution
    local_y = (rows + 0.5) * truth.resolution
    tc, ts = math.cos(truth.origin_yaw), math.sin(truth.origin_yaw)
    world_x = truth.origin[0] + tc * local_x - ts * local_y
    world_y = truth.origin[1] + ts * local_x + tc * local_y
    dx, dy = world_x - belief.origin_x_m, world_y - belief.origin_y_m
    bc, bs = math.cos(belief.origin_yaw_rad), math.sin(belief.origin_yaw_rad)
    local_bx = bc * dx + bs * dy
    local_by = -bs * dx + bc * dy
    cols = np.floor(local_bx / belief.resolution_m).astype(np.int64)
    rows = np.floor(local_by / belief.resolution_m).astype(np.int64)
    inside = (
        (rows >= 0) & (rows < belief.height) &
        (cols >= 0) & (cols < belief.width)
    )
    projected = np.full(truth.shape, -2, dtype=np.int16)
    projected[inside] = belief.data[rows[inside], cols[inside]]
    return projected, inside


def remaining_truth_masks(
    truth: TruthGrid, belief: GridSnapshot
) -> tuple[np.ndarray, np.ndarray]:
    """Return truth-free cells still unknown and cells misclassified occupied."""
    projected, inside = _project_belief_on_truth(truth, belief)
    still_unknown = truth.free & (~inside | (projected < 0))
    false_occupied = truth.free & inside & (projected >= 50)
    return still_unknown, false_occupied


def _grid_transform(axis: Any, grid: GridSnapshot) -> Any:
    from matplotlib.transforms import Affine2D

    return (
        Affine2D().rotate(grid.origin_yaw_rad).translate(
            grid.origin_x_m, grid.origin_y_m
        ) + axis.transData
    )


def _truth_transform(axis: Any, truth: TruthGrid) -> Any:
    from matplotlib.transforms import Affine2D

    return (
        Affine2D().rotate(truth.origin_yaw).translate(*truth.origin)
        + axis.transData
    )


def _draw_map(
    axis: Any,
    grid: GridSnapshot,
    truth: TruthGrid,
    endpoints: Sequence[Endpoint],
    *,
    decision: TraceEvent | None,
    show_remaining: bool,
) -> None:
    from matplotlib.colors import ListedColormap

    classified = np.where(grid.data < 0, 0, np.where(grid.data >= 50, 2, 1))
    axis.imshow(
        classified,
        origin="lower",
        interpolation="nearest",
        extent=(0, grid.width * grid.resolution_m, 0, grid.height * grid.resolution_m),
        transform=_grid_transform(axis, grid),
        cmap=ListedColormap(["#9ca3af", "#f8fafc", "#111827"]),
        vmin=0,
        vmax=2,
        zorder=1,
    )
    if show_remaining:
        unknown, false_occupied = remaining_truth_masks(truth, grid)
        overlay = np.zeros((*truth.shape, 4), dtype=np.float32)
        overlay[unknown] = (0.86, 0.08, 0.44, 0.58)
        overlay[false_occupied] = (0.96, 0.50, 0.09, 0.70)
        axis.imshow(
            overlay,
            origin="lower",
            interpolation="nearest",
            extent=(0, truth.shape[1] * truth.resolution, 0, truth.shape[0] * truth.resolution),
            transform=_truth_transform(axis, truth),
            zorder=2,
        )

    if endpoints:
        path = np.asarray([item.reached_pose[:2] for item in endpoints])
        axis.plot(path[:, 0], path[:, 1], color="#2563eb", linewidth=1.5, zorder=4)
        for item in endpoints:
            x, y, heading = item.reached_pose
            color = "#16a34a" if item.succeeded else "#dc2626"
            marker = "o" if item.succeeded else "x"
            axis.scatter(x, y, s=38, marker=marker, color=color, zorder=6)
            axis.annotate(
                str(item.decision_id), (x, y), xytext=(4, 4),
                textcoords="offset points", fontsize=7, fontweight="bold",
                color="#0f172a", bbox={"facecolor": "white", "alpha": 0.75, "pad": 0.6},
                zorder=8,
            )
            radians = math.radians(heading)
            axis.arrow(
                x, y, 0.35 * math.cos(radians), 0.35 * math.sin(radians),
                width=0.015, head_width=0.13, length_includes_head=True,
                color=color, alpha=0.85, zorder=7,
            )

    if decision is not None:
        active = decision.payload.get("active_candidates", [])
        if isinstance(active, list):
            ranked = sorted(
                (item for item in active if isinstance(item, Mapping)),
                key=lambda item: float(item.get("priority", -math.inf)),
                reverse=True,
            )[:24]
            for index, candidate in enumerate(ranked, 1):
                target = candidate.get("target")
                if not isinstance(target, Sequence) or len(target) < 2:
                    continue
                x, y = float(target[0]), float(target[1])
                axis.scatter(x, y, marker="+", s=36, color="#f59e0b", zorder=5)
                axis.annotate(f"C{index}", (x, y), fontsize=5.5, color="#92400e", zorder=6)
        selected = decision.payload.get("target_pose")
        if isinstance(selected, Sequence) and len(selected) >= 3:
            x, y, heading = (float(value) for value in selected[:3])
            radians = math.radians(heading)
            axis.arrow(
                x, y, 0.65 * math.cos(radians), 0.65 * math.sin(radians),
                width=0.025, head_width=0.22, length_includes_head=True,
                color="#7c3aed", zorder=9,
            )
            axis.scatter(x, y, marker="*", s=130, color="#7c3aed", zorder=9)

    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.grid(color="#64748b", alpha=0.18, linewidth=0.5)


def render_map_png(
    context: RunContext,
    grid: GridSnapshot,
    truth: TruthGrid,
    endpoints: Sequence[Endpoint],
    coverage: CoverageSample,
    *,
    decision: TraceEvent | None,
    title: str,
    show_remaining: bool,
) -> bytes:
    from io import BytesIO
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D

    figure = Figure(figsize=(12, 8), dpi=120)
    FigureCanvasAgg(figure)
    axis = figure.add_axes([0.07, 0.12, 0.66, 0.78])
    _draw_map(
        axis, grid, truth, endpoints, decision=decision,
        show_remaining=show_remaining,
    )
    unknown, false_occupied = remaining_truth_masks(truth, grid)
    total_free = max(1, int(np.count_nonzero(truth.free)))
    detail = (
        f"C_i = {coverage.sensor:.1%}\n"
        f"C_t = {coverage.topological:.1%}\n"
        f"joint = {coverage.joint:.1%}\n\n"
        f"distance = {coverage.distance_m:.2f} m\n"
        f"endpoints = {len(endpoints)}\n"
        f"truth-free still unknown =\n"
        f"{np.count_nonzero(unknown) / total_free:.1%}\n\n"
        "success target = 95% / 95%\n"
        "(evaluation only)"
    )
    figure.text(0.76, 0.77, detail, fontsize=10.5, va="top")
    figure.text(
        0.76, 0.50,
        "Endpoint labels\n0 = start\n1..N = executed decisions\n\n"
        "Arrows = reached heading\n"
        "C1..C24 = active candidates\n"
        "Purple star = selected goal",
        fontsize=9.0, va="top",
    )
    handles = [
        Patch(facecolor="#9ca3af", label="belief unknown"),
        Patch(facecolor="#f8fafc", edgecolor="#94a3b8", label="belief free"),
        Patch(facecolor="#111827", label="belief occupied"),
        Patch(facecolor="#db146f", alpha=0.58, label="truth-free still unknown"),
        Patch(facecolor="#f57f17", alpha=0.70, label="truth-free mapped occupied"),
        Line2D([0], [0], color="#2563eb", label="executed endpoint path"),
    ]
    figure.legend(handles=handles, loc="lower right", bbox_to_anchor=(0.97, 0.14), fontsize=8)
    figure.suptitle(title, fontsize=17, fontweight="bold", y=0.975)
    figure.text(
        0.5, 0.94,
        f"UNKNOWN-COMPLETION DEVELOPMENT SIM | {context.identity.get('world_id')} | "
        f"{context.identity.get('method')} | seed {context.identity.get('replicate_seed')}",
        ha="center", fontsize=10, color="#7f1d1d",
    )
    figure.text(0.5, 0.035, "Truth overlay is evaluator-only and never enters policy decisions.", ha="center", fontsize=8)
    stream = BytesIO()
    figure.savefig(stream, format="png", dpi=120, facecolor="white")
    return stream.getvalue()


def render_coverage_png(
    context: RunContext,
    samples: Sequence[CoverageSample],
    endpoints: Sequence[Endpoint],
) -> bytes:
    from io import BytesIO
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    figure = Figure(figsize=(11, 6.5), dpi=140)
    FigureCanvasAgg(figure)
    axis = figure.add_axes([0.10, 0.14, 0.84, 0.72])
    distance = np.asarray([item.distance_m for item in samples])
    sensor = np.asarray([item.sensor for item in samples])
    topology = np.asarray([item.topological for item in samples])
    axis.plot(distance, sensor, color="#2563eb", linewidth=2.2, label="C_i truth-sensor")
    axis.plot(distance, topology, color="#16a34a", linewidth=2.2, label="C_t endpoints")
    axis.plot(distance, np.minimum(sensor, topology), color="#7c3aed", linewidth=1.6, linestyle="--", label="joint min")
    axis.axhline(0.95, color="#dc2626", linewidth=1.2, linestyle=":", label="95% target")
    for endpoint in endpoints:
        sample = coverage_at(samples, endpoint.ros_time_ns)
        axis.scatter(sample.distance_m, sample.joint, s=24, color="#7c3aed", zorder=5)
        axis.annotate(str(endpoint.decision_id), (sample.distance_m, sample.joint), xytext=(3, 4), textcoords="offset points", fontsize=7)
    axis.set_xlim(left=0.0)
    axis.set_ylim(0.0, 1.02)
    axis.set_xlabel("ground-truth travel distance [m]")
    axis.set_ylabel("coverage")
    axis.grid(alpha=0.25)
    axis.legend(loc="lower right")
    figure.suptitle("Unknown-completion coverage evolution", fontsize=16, fontweight="bold")
    figure.text(0.5, 0.92, f"{context.identity.get('world_id')} | {context.identity.get('method')} | numbered at executed endpoints", ha="center", fontsize=9)
    stream = BytesIO()
    figure.savefig(stream, format="png", dpi=140, facecolor="white")
    return stream.getvalue()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_endpoints_csv(
    path: Path,
    endpoints: Sequence[Endpoint],
    coverage: Sequence[CoverageSample],
) -> None:
    fields = [
        "schema", "decision_id", "ros_time_ns", "succeeded", "reason",
        "commanded_x_m", "commanded_y_m", "commanded_heading_deg",
        "reached_x_m", "reached_y_m", "reached_heading_deg",
        "topological_node_created", "cumulative_trace_distance_m",
        "truth_sensor_coverage", "truth_topological_coverage", "joint_coverage",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for endpoint in endpoints:
            sample = coverage_at(coverage, endpoint.ros_time_ns)
            commanded = endpoint.commanded_pose or (None, None, None)
            writer.writerow({
                "schema": TRACE_SCHEMA,
                "decision_id": endpoint.decision_id,
                "ros_time_ns": endpoint.ros_time_ns,
                "succeeded": int(endpoint.succeeded),
                "reason": endpoint.reason,
                "commanded_x_m": commanded[0],
                "commanded_y_m": commanded[1],
                "commanded_heading_deg": commanded[2],
                "reached_x_m": endpoint.reached_pose[0],
                "reached_y_m": endpoint.reached_pose[1],
                "reached_heading_deg": endpoint.reached_pose[2],
                "topological_node_created": int(endpoint.topological_node_created),
                "cumulative_trace_distance_m": endpoint.cumulative_distance_m,
                "truth_sensor_coverage": sample.sensor,
                "truth_topological_coverage": sample.topological,
                "joint_coverage": sample.joint,
            })


def _encode_video(frame_dir: Path, output: Path, fps: float) -> dict[str, Any]:
    ffmpeg = shutil.which("ffmpeg")
    ffprobe = shutil.which("ffprobe")
    if ffmpeg is None or ffprobe is None:
        raise RenderError("ffmpeg and ffprobe are required for decision video")
    command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
        "-framerate", f"{fps:g}", "-i", str(frame_dir / "decision_%03d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(output),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise RenderError(f"ffmpeg failed: {result.stderr.strip()}")
    probe = subprocess.run(
        [ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
         "-show_entries", "stream=codec_name,width,height,nb_read_frames,duration",
         "-of", "json", str(output)],
        capture_output=True, text=True, check=False,
    )
    if probe.returncode != 0:
        raise RenderError(f"ffprobe rejected video: {probe.stderr.strip()}")
    metadata = json.loads(probe.stdout)["streams"][0]
    if metadata.get("codec_name") != "h264":
        raise RenderError("decision video codec is not H.264")
    return {
        "codec": "h264",
        "width": int(metadata["width"]),
        "height": int(metadata["height"]),
        "frame_count": int(metadata["nb_read_frames"]),
        "duration_s": float(metadata["duration"]),
        "fps": fps,
    }


def render_unknown_completion_bundle(
    run_dir: Path | str,
    *,
    bag: Path | str | None = None,
    fps: float = 1.0,
) -> dict[str, Any]:
    if not math.isfinite(fps) or fps <= 0.0:
        raise RenderError("fps must be finite and positive")
    context = load_run_context(run_dir, bag)
    output = _protected_child(
        context.run_dir,
        context.run_dir / "media" / OUTPUT_DIRECTORY,
        "unknown-completion media output",
        must_exist=False,
    )
    if os.path.lexists(output):
        raise RenderError(f"refusing to overwrite existing media bundle: {output}")
    events = policy_trace(context.run_dir)
    decisions = extract_decisions(events)
    endpoints = extract_endpoints(events)
    coverage = coverage_trace(context.run_dir)
    timeline = read_map_timeline(context)
    bag_evidence: BagEvidence = read_core_bag(context, include_scan=False)
    evaluation_manifest_path = context.run_dir / "evaluation_manifest.json"
    evaluation_manifest = json.loads(evaluation_manifest_path.read_text(encoding="utf-8"))
    truth_path = Path(str(evaluation_manifest.get("truth_map_yaml", "")))
    truth = load_truth_map(truth_path)
    truth = transform_truth_grid(
        truth,
        (context.registration.x_m, context.registration.y_m),
        context.registration.yaw_rad,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".unknown_completion.", dir=output.parent))
    try:
        frame_dir = temporary / "decision_frames"
        frame_dir.mkdir()
        rendered_decisions = decisions or [None]
        for index, decision in enumerate(rendered_decisions):
            timestamp = (
                endpoints[-1].ros_time_ns if decision is None else decision.ros_time_ns
            )
            grid = snapshot_at(timeline, timestamp)
            visible_endpoints = [item for item in endpoints if item.ros_time_ns <= timestamp]
            sample = coverage_at(coverage, timestamp)
            payload = render_map_png(
                context, grid, truth, visible_endpoints, sample,
                decision=decision,
                title=("Terminal unknown-completion state" if decision is None else f"Decision {decision.payload.get('decision_id')}"),
                show_remaining=True,
            )
            (frame_dir / f"decision_{index:03d}.png").write_bytes(payload)

        final_grid = timeline[-1]
        final_coverage = coverage[-1]
        final_decision = decisions[-1] if decisions else None
        (temporary / "numbered_final_state.png").write_bytes(
            render_map_png(
                context, final_grid, truth, endpoints, final_coverage,
                decision=final_decision,
                title="Terminal unknown-completion state",
                show_remaining=True,
            )
        )
        (temporary / "coverage_evolution.png").write_bytes(
            render_coverage_png(context, coverage, endpoints)
        )
        _write_endpoints_csv(temporary / "numbered_endpoints.csv", endpoints, coverage)
        video_metadata = _encode_video(
            frame_dir, temporary / "decision_sequence.mp4", fps
        )
        truth_path_map = transform_truth_points(
            bag_evidence.truth_points, context.registration
        )
        termination_reason = "unknown"
        for event in reversed(events):
            if event.event not in {"session_finished", "budget_reached"}:
                continue
            candidate = event.payload.get(
                "termination_reason", event.payload.get("reason")
            )
            if candidate:
                termination_reason = str(candidate)
                break
        manifest = {
            "schema": OUTPUT_SCHEMA,
            "evidence_label": "unknown-completion development simulation",
            "truth_access": "renderer_and_evaluator_only",
            "policy_truth_access": False,
            "study_id": context.study_id,
            "schedule_id": context.schedule_id,
            "identity": dict(context.identity),
            "termination_reason": termination_reason,
            "decision_count": len(decisions),
            "endpoint_count_including_start": len(endpoints),
            "final_coverage": {
                "sensor": final_coverage.sensor,
                "topological": final_coverage.topological,
                "joint": final_coverage.joint,
                "distance_m": final_coverage.distance_m,
            },
            "registered_truth_path_length_m": float(
                np.sum(np.linalg.norm(np.diff(truth_path_map, axis=0), axis=1))
            ),
            "video": video_metadata,
        }
        artifact_paths = sorted(
            path for path in temporary.rglob("*") if path.is_file()
        )
        manifest["artifacts"] = {
            str(path.relative_to(temporary)): {
                "sha256": _sha256(path), "bytes": path.stat().st_size
            }
            for path in artifact_paths
        }
        (temporary / "render_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        try:
            os.rename(temporary, output)
        except FileExistsError as error:
            raise RenderError(f"refusing to overwrite existing media bundle: {output}") from error
        return {**manifest, "output_dir": str(output)}
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--fps", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = render_unknown_completion_bundle(
            args.run_dir, bag=args.bag, fps=args.fps
        )
    except (ImportError, OSError, RenderError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
