#!/usr/bin/env python3
"""Render auditable development media directly from a ROS 2 core bag.

The default artifact is ``media/raw/final_state.png`` below a validated
system-simulation run directory.  Pass ``--sensor-sanity`` to additionally
render the final ``/scan`` sample.  Existing outputs are never overwritten.

Runtime dependencies are ROS 2 Jazzy's ``rosbag2_py`` / CDR message support,
the MCAP storage plugin, NumPy, Matplotlib, Pillow and PyYAML.  Source the ROS
and workspace setup files before invoking this script.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from io import BytesIO
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


RUN_SCHEMA = "sstg_system_sim_run_launch/v1"
OUTPUT_SCHEMA = "sstg_system_sim_bag_media_render/v1"
DEVELOPMENT_LABEL = "Development simulation evidence"
FINAL_STATE_NAME = "final_state.png"
SENSOR_SANITY_NAME = "sensor_sanity.png"
EXPECTED_TOPICS = {
    "/map": "nav_msgs/msg/OccupancyGrid",
    "/evaluation/ground_truth_odom": "nav_msgs/msg/Odometry",
    "/scan": "sensor_msgs/msg/LaserScan",
}


class RenderError(ValueError):
    """Raised when a bag render would be ambiguous or unsafe."""


@dataclass(frozen=True)
class TruthRegistration:
    registration_id: str
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class RunContext:
    run_dir: Path
    bag_dir: Path
    study_id: str
    schedule_id: str
    identity: Mapping[str, Any]
    execution_status: str
    registration: TruthRegistration


@dataclass(frozen=True)
class GridSnapshot:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float
    frame_id: str
    bag_timestamp_ns: int
    data: np.ndarray

    def __post_init__(self) -> None:
        values = np.asarray(self.data, dtype=np.int16)
        if self.width <= 0 or self.height <= 0:
            raise RenderError("map dimensions must be positive")
        if values.shape != (self.height, self.width):
            raise RenderError("map data does not match declared dimensions")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise RenderError("map resolution must be positive and finite")
        if not self.frame_id:
            raise RenderError("map frame_id must be non-empty")
        if not np.all((values >= -1) & (values <= 100)):
            raise RenderError("map occupancy values must be within [-1, 100]")
        object.__setattr__(self, "data", values)


@dataclass(frozen=True)
class ScanSnapshot:
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    frame_id: str
    bag_timestamp_ns: int
    ranges_m: np.ndarray

    def __post_init__(self) -> None:
        ranges = np.asarray(self.ranges_m, dtype=np.float64)
        if ranges.ndim != 1 or ranges.size == 0:
            raise RenderError("scan ranges must be a non-empty vector")
        fields = (
            self.angle_min_rad,
            self.angle_increment_rad,
            self.range_min_m,
            self.range_max_m,
        )
        if not all(math.isfinite(value) for value in fields):
            raise RenderError("scan geometry must be finite")
        if self.angle_increment_rad == 0.0:
            raise RenderError("scan angle increment must be non-zero")
        if not 0.0 <= self.range_min_m < self.range_max_m:
            raise RenderError("scan range bounds are invalid")
        if not self.frame_id:
            raise RenderError("scan frame_id must be non-empty")
        object.__setattr__(self, "ranges_m", ranges)


@dataclass(frozen=True)
class BagEvidence:
    final_map: GridSnapshot
    truth_points: np.ndarray
    truth_frame_id: str
    final_scan: ScanSnapshot | None
    topic_message_counts: Mapping[str, int]

    def __post_init__(self) -> None:
        points = np.asarray(self.truth_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (2,) or points.shape[0] == 0:
            raise RenderError("ground-truth path must contain XY points")
        if not np.all(np.isfinite(points)):
            raise RenderError("ground-truth path contains non-finite positions")
        if not self.truth_frame_id:
            raise RenderError("ground-truth frame_id must be non-empty")
        object.__setattr__(self, "truth_points", points)


def _finite_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RenderError(f"{label} must be numeric") from error
    if not math.isfinite(result):
        raise RenderError(f"{label} must be finite")
    return result


def _mapping(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink():
        raise RenderError(f"{label} must not be a symlink: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RenderError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise RenderError(f"{label} must be a YAML mapping: {path}")
    return value


def _protected_child(
    root: Path,
    candidate: Path,
    label: str,
    *,
    must_exist: bool,
) -> Path:
    """Resolve one path below root while rejecting traversal and symlinks."""
    root = root.resolve()
    lexical = Path(os.path.abspath(os.fspath(candidate.expanduser())))
    try:
        relative = lexical.relative_to(root)
    except ValueError as error:
        raise RenderError(f"{label} escapes run directory: {candidate}") from error
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise RenderError(f"{label} traverses a symlink: {current}")
    resolved = lexical.resolve(strict=False)
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise RenderError(f"{label} escapes run directory: {candidate}") from error
    if must_exist and not resolved.exists():
        raise RenderError(f"{label} does not exist: {resolved}")
    return resolved


def load_run_context(run_dir: Path | str, bag: Path | str | None = None) -> RunContext:
    supplied_run = Path(run_dir).expanduser()
    if supplied_run.is_symlink():
        raise RenderError(f"run directory must not be a symlink: {supplied_run}")
    run = supplied_run.resolve()
    if not run.is_dir():
        raise RenderError(f"run directory does not exist: {run}")
    manifest_path = _protected_child(
        run, run / "run_launch_manifest.yaml", "run launch manifest", must_exist=True
    )
    manifest = _mapping(manifest_path, "run launch manifest")
    if manifest.get("schema") != RUN_SCHEMA:
        raise RenderError("unsupported run launch manifest schema")
    study_id = str(manifest.get("study_id", "")).strip()
    schedule_id = str(manifest.get("schedule_id", "")).strip()
    if not study_id or not schedule_id:
        raise RenderError("run launch manifest lacks study_id or schedule_id")
    launch = manifest.get("launch")
    arguments = launch.get("arguments") if isinstance(launch, Mapping) else None
    if not isinstance(arguments, Mapping):
        raise RenderError("run launch manifest lacks launch arguments")
    registration_id = str(arguments.get("truth_registration_id", "")).strip()
    if not registration_id:
        raise RenderError("truth_registration_id is required for map/path overlay")
    registration = TruthRegistration(
        registration_id=registration_id,
        x_m=_finite_float(arguments.get("truth_to_map_x_m"), "truth_to_map_x_m"),
        y_m=_finite_float(arguments.get("truth_to_map_y_m"), "truth_to_map_y_m"),
        yaw_rad=_finite_float(
            arguments.get("truth_to_map_yaw_rad"), "truth_to_map_yaw_rad"
        ),
    )
    if bag is None:
        bag_candidate = run / "bags" / "core"
    else:
        supplied_bag = Path(bag).expanduser()
        bag_candidate = (
            supplied_bag if supplied_bag.is_absolute() else run / supplied_bag
        )
    bag_dir = _protected_child(
        run, bag_candidate, "core bag directory", must_exist=True
    )
    if not bag_dir.is_dir():
        raise RenderError(f"core bag path is not a directory: {bag_dir}")
    metadata = _protected_child(
        run, bag_dir / "metadata.yaml", "bag metadata", must_exist=True
    )
    if not metadata.is_file():
        raise RenderError(f"bag metadata is not a regular file: {metadata}")
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if not mcap_files:
        raise RenderError(f"core bag has no MCAP file: {bag_dir}")
    for mcap in mcap_files:
        checked = _protected_child(run, mcap, "MCAP file", must_exist=True)
        if not checked.is_file():
            raise RenderError(f"MCAP path is not a regular file: {checked}")
    identity = manifest.get("identity", {})
    if not isinstance(identity, Mapping):
        raise RenderError("run identity must be a mapping")
    execution = manifest.get("execution", {})
    execution_status = (
        str(execution.get("status", "unknown")).strip()
        if isinstance(execution, Mapping)
        else "unknown"
    )
    return RunContext(
        run_dir=run,
        bag_dir=bag_dir,
        study_id=study_id,
        schedule_id=schedule_id,
        identity=dict(identity),
        execution_status=execution_status or "unknown",
        registration=registration,
    )


def _quaternion_yaw(quaternion: Any, label: str) -> float:
    values = np.asarray(
        [
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
        ],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise RenderError(f"{label} quaternion must be finite")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise RenderError(f"{label} quaternion has zero length")
    x, y, z, w = values / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def grid_from_message(message: Any, bag_timestamp_ns: int) -> GridSnapshot:
    info = message.info
    width = int(info.width)
    height = int(info.height)
    data = np.asarray(message.data, dtype=np.int16)
    if data.size != width * height:
        raise RenderError("map message payload size is inconsistent")
    return GridSnapshot(
        width=width,
        height=height,
        resolution_m=_finite_float(info.resolution, "map resolution"),
        origin_x_m=_finite_float(info.origin.position.x, "map origin x"),
        origin_y_m=_finite_float(info.origin.position.y, "map origin y"),
        origin_yaw_rad=_quaternion_yaw(info.origin.orientation, "map origin"),
        frame_id=str(message.header.frame_id).strip(),
        bag_timestamp_ns=int(bag_timestamp_ns),
        data=data.reshape((height, width)),
    )


def scan_from_message(message: Any, bag_timestamp_ns: int) -> ScanSnapshot:
    return ScanSnapshot(
        angle_min_rad=_finite_float(message.angle_min, "scan angle_min"),
        angle_increment_rad=_finite_float(
            message.angle_increment, "scan angle_increment"
        ),
        range_min_m=_finite_float(message.range_min, "scan range_min"),
        range_max_m=_finite_float(message.range_max, "scan range_max"),
        frame_id=str(message.header.frame_id).strip(),
        bag_timestamp_ns=int(bag_timestamp_ns),
        ranges_m=np.asarray(message.ranges, dtype=np.float64),
    )


def transform_truth_points(
    points: np.ndarray, registration: TruthRegistration
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (2,):
        raise RenderError("truth points must be an N by 2 array")
    cosine = math.cos(registration.yaw_rad)
    sine = math.sin(registration.yaw_rad)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + np.asarray([registration.x_m, registration.y_m])


def read_core_bag(context: RunContext, *, include_scan: bool) -> BagEvidence:
    """Read only selected topics; large contact streams are never deserialized."""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import (
            ConverterOptions,
            SequentialReader,
            StorageFilter,
            StorageOptions,
        )
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RenderError(
            "ROS bag runtime unavailable; source /opt/ros/jazzy/setup.bash and "
            "install rosbag2_py plus the MCAP storage plugin"
        ) from error

    selected = ["/map", "/evaluation/ground_truth_odom"]
    if include_scan:
        selected.append("/scan")
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
    for topic in selected:
        actual = topic_types.get(topic)
        expected = EXPECTED_TOPICS[topic]
        if actual != expected:
            raise RenderError(
                f"required bag topic {topic} has type {actual!r}; expected {expected!r}"
            )
    message_types = {topic: get_message(topic_types[topic]) for topic in selected}
    reader.set_filter(StorageFilter(topics=selected))

    final_map: GridSnapshot | None = None
    final_scan: ScanSnapshot | None = None
    truth_points: list[tuple[float, float]] = []
    truth_frame_id: str | None = None
    counts = {topic: 0 for topic in selected}
    last_bag_timestamp_ns: int | None = None
    while reader.has_next():
        try:
            topic, encoded, timestamp_ns = reader.read_next()
            timestamp_ns = int(timestamp_ns)
            if (
                last_bag_timestamp_ns is not None
                and timestamp_ns < last_bag_timestamp_ns
            ):
                raise RenderError("selected bag messages are not timestamp ordered")
            last_bag_timestamp_ns = timestamp_ns
            message = deserialize_message(encoded, message_types[topic])
            counts[topic] += 1
            if topic == "/map":
                final_map = grid_from_message(message, timestamp_ns)
            elif topic == "/evaluation/ground_truth_odom":
                frame_id = str(message.header.frame_id).strip()
                if not frame_id:
                    raise RenderError("ground-truth odometry frame_id is empty")
                if truth_frame_id is None:
                    truth_frame_id = frame_id
                elif frame_id != truth_frame_id:
                    raise RenderError("ground-truth odometry frame_id changed in bag")
                x = _finite_float(message.pose.pose.position.x, "truth odometry x")
                y = _finite_float(message.pose.pose.position.y, "truth odometry y")
                truth_points.append((x, y))
            elif topic == "/scan":
                final_scan = scan_from_message(message, timestamp_ns)
        except RenderError:
            raise
        except Exception as error:
            raise RenderError(
                "cannot decode selected bag message at "
                f"{last_bag_timestamp_ns}: {error}"
            ) from error
    if final_map is None:
        raise RenderError("core bag contains no /map message")
    if not truth_points or truth_frame_id is None:
        raise RenderError("core bag contains no ground-truth odometry path")
    if include_scan and final_scan is None:
        raise RenderError("core bag contains no /scan message")
    return BagEvidence(
        final_map=final_map,
        truth_points=np.asarray(truth_points, dtype=np.float64),
        truth_frame_id=truth_frame_id,
        final_scan=final_scan,
        topic_message_counts=counts,
    )


def _png_bytes(figure: Any, description: str) -> bytes:
    stream = BytesIO()
    figure.savefig(
        stream,
        format="png",
        dpi=160,
        facecolor="white",
        metadata={
            "Title": DEVELOPMENT_LABEL,
            "Description": description,
            "Software": "SSTG offline ROS 2 bag renderer",
        },
    )
    encoded = stream.getvalue()
    if not encoded.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RenderError("renderer did not produce a valid PNG signature")
    return encoded


def _run_caption(context: RunContext) -> str:
    identity = context.identity
    fields = [
        str(identity.get("world_id", "unknown-world")),
        str(identity.get("method", "unknown-method")),
        str(identity.get("condition", "unknown-condition")),
        f"seed {identity.get('replicate_seed', 'unknown')}",
    ]
    return " • ".join(fields)


def render_final_state_png(context: RunContext, evidence: BagEvidence) -> bytes:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.colors import ListedColormap
    from matplotlib.figure import Figure
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    from matplotlib.transforms import Affine2D

    grid = evidence.final_map
    path = transform_truth_points(evidence.truth_points, context.registration)
    classified = np.where(grid.data < 0, 0, np.where(grid.data >= 50, 2, 1))
    colors = ListedColormap(["#9ca3af", "#f8fafc", "#111827"])

    figure = Figure(figsize=(10.0, 8.0), constrained_layout=False)
    FigureCanvasAgg(figure)
    axis = figure.add_axes([0.08, 0.13, 0.84, 0.72])
    local_extent = (
        0.0,
        grid.width * grid.resolution_m,
        0.0,
        grid.height * grid.resolution_m,
    )
    grid_transform = (
        Affine2D()
        .rotate(grid.origin_yaw_rad)
        .translate(grid.origin_x_m, grid.origin_y_m)
        + axis.transData
    )
    axis.imshow(
        classified,
        origin="lower",
        interpolation="nearest",
        extent=local_extent,
        transform=grid_transform,
        cmap=colors,
        vmin=0,
        vmax=2,
        zorder=1,
    )
    axis.plot(
        path[:, 0],
        path[:, 1],
        color="#dc2626",
        linewidth=1.8,
        alpha=0.9,
        zorder=3,
        label="Ground-truth path (T_map_truth)",
    )
    axis.scatter(path[0, 0], path[0, 1], marker="o", s=65, color="#16a34a", zorder=4)
    axis.scatter(path[-1, 0], path[-1, 1], marker="*", s=150, color="#2563eb", zorder=5)

    corners = np.asarray(
        [
            [0.0, 0.0],
            [local_extent[1], 0.0],
            [local_extent[1], local_extent[3]],
            [0.0, local_extent[3]],
        ]
    )
    cosine = math.cos(grid.origin_yaw_rad)
    sine = math.sin(grid.origin_yaw_rad)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    map_corners = corners @ rotation.T + np.asarray(
        [
            grid.origin_x_m,
            grid.origin_y_m,
        ]
    )
    all_x = np.concatenate((map_corners[:, 0], path[:, 0]))
    all_y = np.concatenate((map_corners[:, 1], path[:, 1]))
    span = max(float(np.ptp(all_x)), float(np.ptp(all_y)), 1.0)
    margin = max(0.5, 0.04 * span)
    axis.set_xlim(float(np.min(all_x)) - margin, float(np.max(all_x)) + margin)
    axis.set_ylim(float(np.min(all_y)) - margin, float(np.max(all_y)) + margin)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel(f"{grid.frame_id} x [m]")
    axis.set_ylabel(f"{grid.frame_id} y [m]")
    axis.grid(color="#64748b", alpha=0.18, linewidth=0.6)

    path_length_m = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    counts = evidence.topic_message_counts
    detail = (
        f"Final /map: {grid.width}×{grid.height} @ {grid.resolution_m:g} m  |  "
        f"map messages: {counts.get('/map', 0)}\n"
        f"Truth samples: {counts.get('/evaluation/ground_truth_odom', 0)}  |  "
        f"whole-bag path: {path_length_m:.2f} m\n"
        f"Frames: {evidence.truth_frame_id} → {grid.frame_id}  |  "
        f"run status: {context.execution_status}\n"
        f"Registration: {context.registration.registration_id}"
    )
    axis.text(
        0.01,
        0.01,
        detail,
        transform=axis.transAxes,
        fontsize=7.8,
        va="bottom",
        ha="left",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "alpha": 0.88},
        zorder=10,
    )
    handles = [
        Patch(facecolor="#9ca3af", label="Unknown"),
        Patch(facecolor="#f8fafc", edgecolor="#94a3b8", label="Known free"),
        Patch(facecolor="#111827", label="Occupied (≥50)"),
        Line2D([0], [0], color="#dc2626", linewidth=2, label="Truth path"),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor="#16a34a", label="Start"
        ),
        Line2D(
            [0],
            [0],
            marker="*",
            color="none",
            markerfacecolor="#2563eb",
            markersize=10,
            label="Final",
        ),
    ]
    axis.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.92)
    figure.suptitle(
        DEVELOPMENT_LABEL,
        x=0.5,
        y=0.975,
        fontsize=18,
        fontweight="bold",
        color="#b91c1c",
    )
    figure.text(0.5, 0.925, _run_caption(context), ha="center", fontsize=11)
    figure.text(
        0.5,
        0.89,
        "Offline core-MCAP rendering • not real-robot or formal evidence",
        ha="center",
        fontsize=9.5,
        color="#475569",
    )
    figure.text(
        0.5,
        0.055,
        f"schedule: {context.schedule_id}",
        ha="center",
        fontsize=7.5,
        color="#64748b",
    )
    return _png_bytes(
        figure,
        f"{DEVELOPMENT_LABEL}; final occupancy map and registered ground-truth path",
    )


def render_sensor_sanity_png(context: RunContext, evidence: BagEvidence) -> bytes:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    scan = evidence.final_scan
    if scan is None:
        raise RenderError("sensor sanity rendering requires a /scan sample")
    angles = (
        scan.angle_min_rad + np.arange(scan.ranges_m.size) * scan.angle_increment_rad
    )
    valid = (
        np.isfinite(scan.ranges_m)
        & (scan.ranges_m >= scan.range_min_m)
        & (scan.ranges_m <= scan.range_max_m)
    )
    if not np.any(valid):
        raise RenderError("final /scan sample contains no valid in-range returns")
    x = scan.ranges_m[valid] * np.cos(angles[valid])
    y = scan.ranges_m[valid] * np.sin(angles[valid])

    figure = Figure(figsize=(8.0, 8.0), constrained_layout=False)
    FigureCanvasAgg(figure)
    axis = figure.add_axes([0.10, 0.13, 0.80, 0.72])
    scatter = axis.scatter(
        x,
        y,
        c=scan.ranges_m[valid],
        s=12,
        cmap="viridis",
        vmin=scan.range_min_m,
        vmax=scan.range_max_m,
    )
    axis.scatter([0.0], [0.0], marker="^", s=100, color="#dc2626", label="LiDAR")
    limit = scan.range_max_m * 1.05
    axis.set_xlim(-limit, limit)
    axis.set_ylim(-limit, limit)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel(f"{scan.frame_id} x [m]")
    axis.set_ylabel(f"{scan.frame_id} y [m]")
    axis.grid(color="#64748b", alpha=0.25)
    axis.legend(loc="upper right")
    colorbar = figure.colorbar(scatter, ax=axis, fraction=0.046, pad=0.04)
    colorbar.set_label("range [m]")
    valid_count = int(np.count_nonzero(valid))
    axis.set_title(
        f"Final /scan sample: {valid_count}/{scan.ranges_m.size} valid returns "
        f"({valid_count / scan.ranges_m.size:.1%})",
        fontsize=11,
    )
    figure.suptitle(
        DEVELOPMENT_LABEL,
        x=0.5,
        y=0.975,
        fontsize=18,
        fontweight="bold",
        color="#b91c1c",
    )
    figure.text(0.5, 0.925, _run_caption(context), ha="center", fontsize=10)
    figure.text(
        0.5,
        0.055,
        "Offline final LaserScan rendering • not real-robot or formal evidence",
        ha="center",
        fontsize=8.5,
        color="#475569",
    )
    return _png_bytes(
        figure,
        f"{DEVELOPMENT_LABEL}; final LaserScan sensor-sanity rendering",
    )


def media_output_paths(
    run_dir: Path, *, include_sensor_sanity: bool
) -> dict[str, Path]:
    supplied_run = Path(run_dir).expanduser()
    if supplied_run.is_symlink():
        raise RenderError(f"run directory must not be a symlink: {supplied_run}")
    run = supplied_run.resolve()
    if not run.is_dir():
        raise RenderError(f"run directory does not exist: {run}")
    output_dir = _protected_child(
        run, run / "media" / "raw", "media output directory", must_exist=False
    )
    outputs = {FINAL_STATE_NAME: output_dir / FINAL_STATE_NAME}
    if include_sensor_sanity:
        outputs[SENSOR_SANITY_NAME] = output_dir / SENSOR_SANITY_NAME
    for name, output in outputs.items():
        checked = _protected_child(
            run, output, f"media output {name}", must_exist=False
        )
        if os.path.lexists(checked):
            raise RenderError(f"refusing to overwrite existing media: {checked}")
    return outputs


def _publish_complete_file(path: Path, encoded: bytes) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.stem}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary_name, path)
        except FileExistsError as error:
            raise RenderError(
                f"refusing to overwrite existing media: {path}"
            ) from error
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def publish_media_pngs(
    run_dir: Path,
    payloads: Mapping[str, bytes],
    *,
    include_sensor_sanity: bool,
) -> dict[str, Path]:
    expected = {FINAL_STATE_NAME}
    if include_sensor_sanity:
        expected.add(SENSOR_SANITY_NAME)
    if set(payloads) != expected:
        raise RenderError(f"rendered payload set must be {sorted(expected)}")
    for name, encoded in payloads.items():
        if not isinstance(encoded, bytes) or not encoded.startswith(
            b"\x89PNG\r\n\x1a\n"
        ):
            raise RenderError(f"rendered payload is not a PNG: {name}")
    outputs = media_output_paths(run_dir, include_sensor_sanity=include_sensor_sanity)
    output_dir = next(iter(outputs.values())).parent
    output_dir.mkdir(parents=True, exist_ok=True)
    # Revalidate after mkdir in case an existing parent was raced into place.
    outputs = media_output_paths(run_dir, include_sensor_sanity=include_sensor_sanity)
    created: list[Path] = []
    try:
        for name in sorted(outputs):
            _publish_complete_file(outputs[name], payloads[name])
            created.append(outputs[name])
    except BaseException:
        for path in created:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        raise
    return outputs


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--bag",
        type=Path,
        help="Core bag directory relative to run_dir (default: bags/core)",
    )
    parser.add_argument(
        "--sensor-sanity",
        action="store_true",
        help="Also render final /scan to media/raw/sensor_sanity.png",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        context = load_run_context(args.run_dir, args.bag)
        # Refuse all existing requested outputs before reading a potentially
        # large bag or constructing any figures.
        media_output_paths(context.run_dir, include_sensor_sanity=args.sensor_sanity)
        evidence = read_core_bag(context, include_scan=args.sensor_sanity)
        payloads = {FINAL_STATE_NAME: render_final_state_png(context, evidence)}
        if args.sensor_sanity:
            payloads[SENSOR_SANITY_NAME] = render_sensor_sanity_png(context, evidence)
        outputs = publish_media_pngs(
            context.run_dir,
            payloads,
            include_sensor_sanity=args.sensor_sanity,
        )
    except (ImportError, OSError, RenderError) as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "schema": OUTPUT_SCHEMA,
                "evidence_label": DEVELOPMENT_LABEL,
                "bag_dir": str(context.bag_dir),
                "outputs": {name: str(path) for name, path in outputs.items()},
                "topic_message_counts": dict(evidence.topic_message_counts),
                "truth_registration_id": context.registration.registration_id,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
