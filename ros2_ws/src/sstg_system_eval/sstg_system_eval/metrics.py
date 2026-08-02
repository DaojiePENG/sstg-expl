"""Pure metric primitives used by the ROS evaluator node.

This module intentionally has no ROS imports.  The ground-truth grid is only
used to score a belief snapshot; none of its data is exposed through a policy
interface.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

import numpy as np
import yaml
from scipy.spatial import cKDTree


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"policy trace contains non-standard JSON constant {value}")


def _finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _fraction(numerator: int, denominator: int) -> Optional[float]:
    if denominator <= 0:
        return None
    return float(numerator) / float(denominator)


def _next_pgm_token(raw: bytes, index: int) -> Tuple[bytes, int]:
    size = len(raw)
    while index < size:
        if raw[index:index + 1] == b"#":
            newline = raw.find(b"\n", index + 1)
            if newline < 0:
                raise ValueError("unterminated PGM comment")
            index = newline + 1
        elif raw[index:index + 1].isspace():
            index += 1
        else:
            break
    start = index
    while index < size and not raw[index:index + 1].isspace():
        if raw[index:index + 1] == b"#":
            break
        index += 1
    if start == index:
        raise ValueError("missing PGM header token")
    return raw[start:index], index


def _read_pgm(path: Path) -> Tuple[np.ndarray, int]:
    raw = path.read_bytes()
    magic, index = _next_pgm_token(raw, 0)
    width_token, index = _next_pgm_token(raw, index)
    height_token, index = _next_pgm_token(raw, index)
    max_token, index = _next_pgm_token(raw, index)
    if magic not in {b"P2", b"P5"}:
        raise ValueError("truth image must be a P2 or P5 PGM")
    width = int(width_token)
    height = int(height_token)
    max_value = int(max_token)
    if width <= 0 or height <= 0:
        raise ValueError("PGM dimensions must be positive")
    if not 0 < max_value <= 65535:
        raise ValueError("PGM max value must be in [1, 65535]")
    count = width * height

    if magic == b"P2":
        values = []
        for _ in range(count):
            token, index = _next_pgm_token(raw, index)
            values.append(int(token))
        pixels = np.asarray(values, dtype=np.uint32)
    else:
        if index >= len(raw) or not raw[index:index + 1].isspace():
            raise ValueError("P5 header is missing the raster separator")
        if raw[index:index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
        bytes_per_sample = 1 if max_value < 256 else 2
        expected_bytes = count * bytes_per_sample
        raster = raw[index:index + expected_bytes]
        if len(raster) != expected_bytes:
            raise ValueError("truncated PGM raster")
        dtype = np.uint8 if bytes_per_sample == 1 else np.dtype(">u2")
        pixels = np.frombuffer(raster, dtype=dtype, count=count).astype(
            np.uint32, copy=False
        )
    if np.any(pixels > max_value):
        raise ValueError("PGM sample exceeds declared max value")
    return pixels.reshape((height, width)), max_value


@dataclass(frozen=True)
class TruthGrid:
    """Ground-truth occupancy masks in ROS row order (bottom row first)."""

    free: np.ndarray
    occupied: np.ndarray
    resolution: float
    origin: Tuple[float, float]
    origin_yaw: float = 0.0
    source_yaml: str = ""
    source_sha256: str = ""

    def __post_init__(self) -> None:
        free = np.asarray(self.free, dtype=bool)
        occupied = np.asarray(self.occupied, dtype=bool)
        if free.ndim != 2 or free.shape != occupied.shape or free.size == 0:
            raise ValueError("truth masks must be non-empty matching 2-D arrays")
        if np.any(free & occupied):
            raise ValueError("truth free and occupied masks overlap")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("truth resolution must be positive and finite")
        if not all(math.isfinite(float(value)) for value in self.origin):
            raise ValueError("truth origin must be finite")
        if not math.isfinite(self.origin_yaw):
            raise ValueError("truth origin yaw must be finite")
        object.__setattr__(self, "free", free)
        object.__setattr__(self, "occupied", occupied)

    @property
    def shape(self) -> Tuple[int, int]:
        return self.free.shape


@dataclass(frozen=True)
class BeliefGrid:
    """One ROS occupancy-grid snapshot in row-major map coordinates."""

    data: np.ndarray
    resolution: float
    origin: Tuple[float, float]
    origin_yaw: float = 0.0

    def __post_init__(self) -> None:
        data = np.asarray(self.data, dtype=np.int16)
        if data.ndim != 2 or data.size == 0:
            raise ValueError("belief data must be a non-empty 2-D array")
        if np.any((data < -1) | (data > 100)):
            raise ValueError("belief occupancy must be within ROS range [-1, 100]")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("belief resolution must be positive and finite")
        if not all(math.isfinite(float(value)) for value in self.origin):
            raise ValueError("belief origin must be finite")
        if not math.isfinite(self.origin_yaw):
            raise ValueError("belief origin yaw must be finite")
        object.__setattr__(self, "data", data)


def load_truth_map(map_yaml: Path | str) -> TruthGrid:
    """Load a Nav2/map_server YAML+PGM pair as an evaluator truth grid."""
    yaml_path = Path(map_yaml).expanduser().resolve()
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("truth map YAML must contain a mapping")
    required = {
        "image",
        "resolution",
        "origin",
        "negate",
        "occupied_thresh",
        "free_thresh",
    }
    missing = sorted(required - document.keys())
    if missing:
        raise ValueError(f"truth map YAML is missing: {', '.join(missing)}")
    image_path = Path(str(document["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    pixels_top_first, max_value = _read_pgm(image_path.resolve())
    pixels = np.flipud(pixels_top_first).astype(np.float64)
    negate = int(document["negate"])
    if negate not in {0, 1}:
        raise ValueError("truth map negate must be 0 or 1")
    occupancy_probability = (
        pixels / max_value if negate else (max_value - pixels) / max_value
    )
    occupied_threshold = _finite_float(
        document["occupied_thresh"], "occupied_thresh"
    )
    free_threshold = _finite_float(document["free_thresh"], "free_thresh")
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise ValueError("truth map thresholds must satisfy 0 <= free < occupied <= 1")
    origin = document["origin"]
    if not isinstance(origin, Sequence) or len(origin) != 3:
        raise ValueError("truth map origin must be [x, y, yaw]")
    digest = hashlib.sha256()
    digest.update(yaml_path.read_bytes())
    digest.update(image_path.resolve().read_bytes())
    return TruthGrid(
        free=occupancy_probability < free_threshold,
        occupied=occupancy_probability > occupied_threshold,
        resolution=_finite_float(document["resolution"], "resolution"),
        origin=(
            _finite_float(origin[0], "origin x"),
            _finite_float(origin[1], "origin y"),
        ),
        origin_yaw=_finite_float(origin[2], "origin yaw"),
        source_yaml=str(yaml_path),
        source_sha256=digest.hexdigest(),
    )


def transform_truth_grid(
    truth: TruthGrid,
    translation: Tuple[float, float],
    yaw: float,
) -> TruthGrid:
    """Express a truth grid in another frame using ``T_target_truth``."""
    tx = _finite_float(translation[0], "truth transform x")
    ty = _finite_float(translation[1], "truth transform y")
    yaw = _finite_float(yaw, "truth transform yaw")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    origin_x = tx + cosine * truth.origin[0] - sine * truth.origin[1]
    origin_y = ty + sine * truth.origin[0] + cosine * truth.origin[1]
    return TruthGrid(
        free=truth.free,
        occupied=truth.occupied,
        resolution=truth.resolution,
        origin=(origin_x, origin_y),
        origin_yaw=truth.origin_yaw + yaw,
        source_yaml=truth.source_yaml,
        source_sha256=truth.source_sha256,
    )


def transform_planar_point(
    point: Sequence[float],
    translation: Sequence[float],
    yaw: float,
) -> Tuple[float, float]:
    """Apply ``T_target_source`` to one planar point."""
    if isinstance(point, (str, bytes)):
        raise ValueError("planar point must contain x and y")
    if isinstance(translation, (str, bytes)):
        raise ValueError("planar translation must contain x and y")
    try:
        if len(point) < 2:
            raise ValueError("planar point must contain x and y")
        if len(translation) < 2:
            raise ValueError("planar translation must contain x and y")
    except TypeError as error:
        raise ValueError(
            "planar point and translation must contain x and y"
        ) from error
    x = _finite_float(point[0], "planar point x")
    y = _finite_float(point[1], "planar point y")
    tx = _finite_float(translation[0], "planar transform x")
    ty = _finite_float(translation[1], "planar transform y")
    yaw = _finite_float(yaw, "planar transform yaw")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
    )


def compute_geometric_metrics(
    truth: TruthGrid,
    belief: BeliefGrid,
    known_free_threshold: int = 50,
) -> Dict[str, Any]:
    """Score the belief on ground-truth cell centers.

    ``geometric_coverage`` is the fraction of all truth-free cells currently
    represented as known-free by the SLAM map.  Raw cell counts accompany all
    ratios so map extent and classification errors remain auditable.
    """
    if not 1 <= int(known_free_threshold) <= 100:
        raise ValueError("known_free_threshold must be in [1, 100]")
    height, width = truth.shape
    rows, columns = np.indices((height, width), dtype=np.float64)
    local_x = (columns + 0.5) * truth.resolution
    local_y = (rows + 0.5) * truth.resolution
    truth_cos = math.cos(truth.origin_yaw)
    truth_sin = math.sin(truth.origin_yaw)
    world_x = truth.origin[0] + truth_cos * local_x - truth_sin * local_y
    world_y = truth.origin[1] + truth_sin * local_x + truth_cos * local_y

    delta_x = world_x - belief.origin[0]
    delta_y = world_y - belief.origin[1]
    belief_cos = math.cos(belief.origin_yaw)
    belief_sin = math.sin(belief.origin_yaw)
    belief_local_x = belief_cos * delta_x + belief_sin * delta_y
    belief_local_y = -belief_sin * delta_x + belief_cos * delta_y
    belief_columns = np.floor(belief_local_x / belief.resolution).astype(np.int64)
    belief_rows = np.floor(belief_local_y / belief.resolution).astype(np.int64)
    in_extent = (
        (belief_columns >= 0)
        & (belief_columns < belief.data.shape[1])
        & (belief_rows >= 0)
        & (belief_rows < belief.data.shape[0])
    )
    projected = np.full(truth.shape, -2, dtype=np.int16)
    projected[in_extent] = belief.data[
        belief_rows[in_extent], belief_columns[in_extent]
    ]
    known = in_extent & (projected >= 0)
    known_free = known & (projected < int(known_free_threshold))
    known_occupied = known & ~known_free
    truth_known = truth.free | truth.occupied
    truth_unknown = ~truth_known

    truth_free_total = int(np.count_nonzero(truth.free))
    truth_occupied_total = int(np.count_nonzero(truth.occupied))
    truth_unknown_total = int(np.count_nonzero(truth_unknown))
    truth_free_in_extent = int(np.count_nonzero(truth.free & in_extent))
    truth_free_known = int(np.count_nonzero(truth.free & known))
    truth_free_known_free = int(np.count_nonzero(truth.free & known_free))
    truth_free_known_occupied = int(np.count_nonzero(truth.free & known_occupied))
    truth_occupied_known_free = int(np.count_nonzero(truth.occupied & known_free))
    known_free_on_truth = int(np.count_nonzero(truth_known & known_free))
    known_on_truth = int(np.count_nonzero(truth_known & known))
    cell_area = truth.resolution * truth.resolution

    return {
        "schema": "sstg_system_sim_geometric_metrics/v1",
        "truth_resolution_m": float(truth.resolution),
        "truth_free_total_cells": truth_free_total,
        "truth_occupied_total_cells": truth_occupied_total,
        "truth_unknown_total_cells": truth_unknown_total,
        "truth_cells_in_belief_extent": int(np.count_nonzero(in_extent)),
        "truth_free_in_belief_extent_cells": truth_free_in_extent,
        "truth_free_known_cells": truth_free_known,
        "truth_free_known_free_cells": truth_free_known_free,
        "truth_free_known_occupied_cells": truth_free_known_occupied,
        "truth_occupied_known_free_cells": truth_occupied_known_free,
        "belief_known_on_truth_cells": known_on_truth,
        "belief_known_free_on_truth_cells": known_free_on_truth,
        "truth_free_total_area_m2": truth_free_total * cell_area,
        "truth_free_known_free_area_m2": truth_free_known_free * cell_area,
        "geometric_coverage": _fraction(
            truth_free_known_free, truth_free_total
        ),
        "truth_free_observed_fraction": _fraction(
            truth_free_known, truth_free_total
        ),
        "truth_free_extent_fraction": _fraction(
            truth_free_in_extent, truth_free_total
        ),
        "known_free_precision_on_truth": _fraction(
            truth_free_known_free, known_free_on_truth
        ),
    }


def compute_topological_metrics(
    truth: TruthGrid,
    node_positions: Sequence[Sequence[float]],
    radius_m: float,
    information_coverage: Optional[float] = None,
    information_target: float = 0.95,
    topological_target: float = 0.95,
) -> Dict[str, Any]:
    """Cover registered truth-free cells with metric disks around real nodes."""
    radius_m = _finite_float(radius_m, "topological radius")
    information_target = _finite_float(
        information_target, "information coverage target"
    )
    topological_target = _finite_float(
        topological_target, "topological coverage target"
    )
    if radius_m <= 0.0:
        raise ValueError("topological radius must be positive")
    if not 0.0 < information_target <= 1.0:
        raise ValueError("information coverage target must be in (0, 1]")
    if not 0.0 < topological_target <= 1.0:
        raise ValueError("topological coverage target must be in (0, 1]")
    if information_coverage is not None:
        information_coverage = _finite_float(
            information_coverage, "information coverage"
        )
        if not 0.0 <= information_coverage <= 1.0:
            raise ValueError("information coverage must be in [0, 1]")

    parsed_nodes = []
    for index, position in enumerate(node_positions):
        if (
            not isinstance(position, Sequence)
            or isinstance(position, (str, bytes))
            or len(position) < 2
        ):
            raise ValueError(f"topological node {index} must contain x and y")
        parsed_nodes.append((
            _finite_float(position[0], f"topological node {index} x"),
            _finite_float(position[1], f"topological node {index} y"),
        ))

    covered = np.zeros(truth.shape, dtype=bool)
    cosine = math.cos(truth.origin_yaw)
    sine = math.sin(truth.origin_yaw)
    padding = int(math.ceil(radius_m / truth.resolution)) + 1
    radius_squared = radius_m * radius_m + 1e-12
    height, width = truth.shape
    for x, y in parsed_nodes:
        delta_x = x - truth.origin[0]
        delta_y = y - truth.origin[1]
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        center_column = int(math.floor(local_x / truth.resolution))
        center_row = int(math.floor(local_y / truth.resolution))
        row0 = max(0, center_row - padding)
        row1 = min(height, center_row + padding + 1)
        column0 = max(0, center_column - padding)
        column1 = min(width, center_column + padding + 1)
        if row0 >= row1 or column0 >= column1:
            continue
        rows = np.arange(row0, row1, dtype=np.float64)[:, None]
        columns = np.arange(column0, column1, dtype=np.float64)[None, :]
        cell_x = (columns + 0.5) * truth.resolution
        cell_y = (rows + 0.5) * truth.resolution
        disk = (
            (cell_x - local_x) ** 2 + (cell_y - local_y) ** 2
            <= radius_squared
        )
        covered[row0:row1, column0:column1] |= disk

    truth_free_total = int(np.count_nonzero(truth.free))
    topologically_covered = int(np.count_nonzero(covered & truth.free))
    topological_coverage = _fraction(topologically_covered, truth_free_total)
    joint_coverage = (
        None
        if information_coverage is None or topological_coverage is None
        else min(information_coverage, topological_coverage)
    )
    information_target_met = (
        None
        if information_coverage is None
        else information_coverage >= information_target
    )
    topological_target_met = (
        None
        if topological_coverage is None
        else topological_coverage >= topological_target
    )
    dual_success = (
        None
        if information_target_met is None or topological_target_met is None
        else information_target_met and topological_target_met
    )
    cell_area = truth.resolution * truth.resolution
    return {
        "schema": "sstg_system_sim_topological_metrics/v1",
        "topological_radius_m": radius_m,
        "unique_node_count": len(parsed_nodes),
        "truth_free_total_cells": truth_free_total,
        "truth_free_topologically_covered_cells": topologically_covered,
        "truth_free_topologically_covered_area_m2": (
            topologically_covered * cell_area
        ),
        "information_coverage": information_coverage,
        "topological_coverage": topological_coverage,
        "joint_coverage": joint_coverage,
        "information_coverage_target": information_target,
        "topological_coverage_target": topological_target,
        "information_target_met": information_target_met,
        "topological_target_met": topological_target_met,
        "dual_threshold_success": dual_success,
    }


def _polyline_length(points: Sequence[Sequence[float]]) -> float:
    parsed = []
    for index, point in enumerate(points):
        if not isinstance(point, Sequence) or len(point) < 2:
            raise ValueError(f"path point {index} must contain x and y")
        parsed.append((
            _finite_float(point[0], f"path[{index}].x"),
            _finite_float(point[1], f"path[{index}].y"),
        ))
    return float(sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(parsed[:-1], parsed[1:])
    ))


class TrajectoryAccumulator:
    """Accumulate evaluator-sampled TF path length without bridging resets."""

    def __init__(self, minimum_step_m: float = 0.0):
        self.minimum_step_m = _finite_float(minimum_step_m, "minimum_step_m")
        if self.minimum_step_m < 0.0:
            raise ValueError("minimum_step_m must be non-negative")
        self.sample_count = 0
        self.moving_segment_count = 0
        self.time_reset_count = 0
        self.path_length_m = 0.0
        self._last_time_ns: Optional[int] = None
        self._distance_anchor: Optional[Tuple[float, float]] = None

    def add(self, time_ns: int, x: float, y: float) -> bool:
        time_ns = int(time_ns)
        x = _finite_float(x, "trajectory x")
        y = _finite_float(y, "trajectory y")
        self.sample_count += 1
        if self._last_time_ns is None or self._distance_anchor is None:
            self._last_time_ns = time_ns
            self._distance_anchor = (x, y)
            return False
        if time_ns < self._last_time_ns:
            self.time_reset_count += 1
            self._last_time_ns = time_ns
            self._distance_anchor = (x, y)
            return False
        if time_ns == self._last_time_ns:
            return False
        self._last_time_ns = time_ns
        anchor_x, anchor_y = self._distance_anchor
        distance = math.hypot(x - anchor_x, y - anchor_y)
        if distance < self.minimum_step_m:
            return False
        self._distance_anchor = (x, y)
        self.path_length_m += distance
        self.moving_segment_count += 1
        return True

    def snapshot(self) -> Dict[str, Any]:
        return {
            "tf_sample_count": self.sample_count,
            "tf_moving_segment_count": self.moving_segment_count,
            "tf_time_reset_count": self.time_reset_count,
            "tf_path_length_m": self.path_length_m,
            "tf_minimum_step_m": self.minimum_step_m,
        }


class GroundTruthMotionAccumulator:
    """Aggregate physical travel and time-paired planar localization error."""

    def __init__(self, minimum_step_m: float = 0.0):
        self.path = TrajectoryAccumulator(minimum_step_m)
        self.ate_sample_count = 0
        self.ate_time_reset_count = 0
        self._ate_error_sum_m = 0.0
        self._ate_squared_error_sum_m2 = 0.0
        self._ate_max_error_m: Optional[float] = None
        self._last_ate_time_ns: Optional[int] = None

    def add_ground_truth(self, time_ns: int, x: float, y: float) -> bool:
        return self.path.add(time_ns, x, y)

    def add_ate_pair(
        self,
        time_ns: int,
        truth_in_map: Sequence[float],
        estimated_in_map: Sequence[float],
    ) -> bool:
        """Add one timestamp-matched map-frame estimate/truth pair."""
        time_ns = int(time_ns)
        truth = transform_planar_point(truth_in_map, (0.0, 0.0), 0.0)
        estimate = transform_planar_point(
            estimated_in_map, (0.0, 0.0), 0.0
        )
        if self._last_ate_time_ns is not None:
            if time_ns == self._last_ate_time_ns:
                return False
            if time_ns < self._last_ate_time_ns:
                self.ate_time_reset_count += 1
        self._last_ate_time_ns = time_ns
        error = math.hypot(estimate[0] - truth[0], estimate[1] - truth[1])
        self.ate_sample_count += 1
        self._ate_error_sum_m += error
        self._ate_squared_error_sum_m2 += error * error
        self._ate_max_error_m = (
            error
            if self._ate_max_error_m is None
            else max(self._ate_max_error_m, error)
        )
        return True

    def snapshot(self) -> Dict[str, Any]:
        path = self.path.snapshot()
        mean = (
            None
            if self.ate_sample_count == 0
            else self._ate_error_sum_m / self.ate_sample_count
        )
        rmse = (
            None
            if self.ate_sample_count == 0
            else math.sqrt(
                self._ate_squared_error_sum_m2 / self.ate_sample_count
            )
        )
        return {
            "status": (
                "available"
                if path["tf_sample_count"] > 0
                else "waiting_for_ground_truth"
            ),
            "ground_truth_sample_count": path["tf_sample_count"],
            "ground_truth_moving_segment_count": path[
                "tf_moving_segment_count"
            ],
            "ground_truth_time_reset_count": path["tf_time_reset_count"],
            "ground_truth_path_length_m": path["tf_path_length_m"],
            "ground_truth_minimum_step_m": path["tf_minimum_step_m"],
            "ate_sample_count": self.ate_sample_count,
            "ate_status": (
                "available"
                if self.ate_sample_count > 0
                else "waiting_for_timestamp_paired_map_tf"
            ),
            "ate_time_reset_count": self.ate_time_reset_count,
            "ate_mean_m": mean,
            "ate_rmse_m": rmse,
            "ate_max_m": self._ate_max_error_m,
        }


class TruthClearanceAccumulator:
    """Sample conservative footprint clearance in a static truth grid.

    Non-free cells (occupied or unknown) are closed square obstacles. The map
    exterior is also treated as unknown obstacle. Point-to-square distance is
    evaluated exactly in the truth grid's local coordinates, then the frozen
    circular robot radius is subtracted and the result is clamped at zero.
    """

    def __init__(self, truth: TruthGrid, robot_radius_m: float):
        self.truth = truth
        self.robot_radius_m = _finite_float(
            robot_radius_m, "robot clearance radius"
        )
        if self.robot_radius_m < 0.0:
            raise ValueError("robot clearance radius must be non-negative")
        non_free_rows, non_free_columns = np.nonzero(~truth.free)
        if non_free_rows.size == 0:
            raise ValueError(
                "truth clearance requires at least one non-free cell"
            )
        centers = np.column_stack((
            (non_free_columns.astype(np.float64) + 0.5) * truth.resolution,
            (non_free_rows.astype(np.float64) + 0.5) * truth.resolution,
        ))
        self._obstacle_centers = centers
        self._obstacle_tree = cKDTree(centers)
        self.reset_samples()

    def reset_samples(self) -> None:
        """Start a fresh policy-session clearance series."""
        self.pose_sample_count = 0
        self.outside_truth_extent_sample_count = 0
        self._raw_distances = []
        self._clearances = []

    def _truth_local(self, x: float, y: float) -> Tuple[float, float]:
        delta_x = x - self.truth.origin[0]
        delta_y = y - self.truth.origin[1]
        cosine = math.cos(self.truth.origin_yaw)
        sine = math.sin(self.truth.origin_yaw)
        return (
            cosine * delta_x + sine * delta_y,
            -sine * delta_x + cosine * delta_y,
        )

    def add(self, x: float, y: float) -> float:
        x = _finite_float(x, "ground-truth clearance x")
        y = _finite_float(y, "ground-truth clearance y")
        self.pose_sample_count += 1
        local_x, local_y = self._truth_local(x, y)
        height, width = self.truth.shape
        extent_x = width * self.truth.resolution
        extent_y = height * self.truth.resolution
        if not (0.0 <= local_x < extent_x and 0.0 <= local_y < extent_y):
            self.outside_truth_extent_sample_count += 1
            raw_distance = 0.0
            footprint_clearance = 0.0
            self._raw_distances.append(raw_distance)
            self._clearances.append(footprint_clearance)
            return footprint_clearance

        query = np.asarray([local_x, local_y], dtype=np.float64)
        nearest_center_distance = float(self._obstacle_tree.query(query)[0])
        half_cell = 0.5 * self.truth.resolution
        search_radius = nearest_center_distance + half_cell
        candidate_indices = self._obstacle_tree.query_ball_point(
            query, search_radius + 1e-12
        )
        candidates = self._obstacle_centers[candidate_indices]
        offsets = np.maximum(np.abs(candidates - query) - half_cell, 0.0)
        obstacle_distance = float(np.min(np.hypot(
            offsets[:, 0], offsets[:, 1]
        )))
        boundary_distance = min(
            local_x,
            extent_x - local_x,
            local_y,
            extent_y - local_y,
        )
        center_clearance = min(obstacle_distance, boundary_distance)
        footprint_clearance = max(
            0.0, center_clearance - self.robot_radius_m
        )
        self._raw_distances.append(center_clearance)
        self._clearances.append(footprint_clearance)
        return footprint_clearance

    def snapshot(self) -> Dict[str, Any]:
        values = np.asarray(self._clearances, dtype=np.float64)
        raw_values = np.asarray(self._raw_distances, dtype=np.float64)
        sample_count = int(values.size)
        return {
            "status": "available" if sample_count else "waiting_for_path",
            "semantics": (
                "per in-session ground-truth pose; exact distance to static "
                "non-free truth-cell squares or map exterior, minus circular "
                "robot radius and clamped at zero"
            ),
            "raw_distance_semantics": (
                "robot-center distance to nearest non-free truth-cell square "
                "or map exterior"
            ),
            "footprint_clearance_formula": (
                "max(0, raw_static_obstacle_distance_m - "
                "robot_clearance_radius_m)"
            ),
            "robot_clearance_radius_m": self.robot_radius_m,
            "clearance_sample_count": sample_count,
            "outside_truth_extent_sample_count": (
                self.outside_truth_extent_sample_count
            ),
            "outside_truth_extent_fraction": _fraction(
                self.outside_truth_extent_sample_count, sample_count
            ),
            "raw_static_obstacle_distance_min_m": (
                None if sample_count == 0 else float(np.min(raw_values))
            ),
            "raw_static_obstacle_distance_p05_m": (
                None
                if sample_count == 0
                else float(np.percentile(raw_values, 5.0))
            ),
            "raw_static_obstacle_distance_mean_m": (
                None if sample_count == 0 else float(np.mean(raw_values))
            ),
            "footprint_clearance_min_m": (
                None if sample_count == 0 else float(np.min(values))
            ),
            "footprint_clearance_p05_m": (
                None
                if sample_count == 0
                else float(np.percentile(values, 5.0))
            ),
            "footprint_clearance_mean_m": (
                None if sample_count == 0 else float(np.mean(values))
            ),
            "outside_samples_are_zero_clearance": True,
            "unknown_truth_cells_are_obstacles": True,
            "map_exterior_is_obstacle": True,
            "limitations": [
                "static 2-D truth occupancy only; dynamic obstacles and "
                "vertical clearance are not represented",
                "statistics are pose-sample weighted, not distance- or "
                "time-weighted",
            ],
        }


class WorldStatisticsAccumulator:
    """Aggregate Gazebo simulation-clock and real-time diagnostics."""

    def __init__(self):
        self.sample_count = 0
        self.paused_sample_count = 0
        self.nonmonotonic_sim_time_count = 0
        self.nonmonotonic_real_time_count = 0
        self.nonmonotonic_iteration_count = 0
        self.stalled_unpaused_sample_count = 0
        self._reported_rtfs = []
        self._observed_rtfs = []
        self._last_valid_sim_time_ns: Optional[int] = None
        self._last_valid_real_time_ns: Optional[int] = None
        self._last_valid_iterations: Optional[int] = None
        self._first_sim_time_ns: Optional[int] = None
        self._first_real_time_ns: Optional[int] = None
        self._first_iterations: Optional[int] = None
        self._last_sim_time_ns: Optional[int] = None
        self._last_real_time_ns: Optional[int] = None
        self._last_pause_time_ns: Optional[int] = None
        self._last_iterations: Optional[int] = None
        self._last_paused: Optional[bool] = None
        self._last_stepping: Optional[bool] = None
        self._last_model_count: Optional[int] = None
        self._last_step_size_ns: Optional[int] = None
        self._last_reported_rtf: Optional[float] = None

    @staticmethod
    def _nonnegative_int(value: Any, label: str) -> int:
        result = int(value)
        if result < 0:
            raise ValueError(f"{label} must be non-negative")
        return result

    def ingest(
        self,
        *,
        sim_time_ns: int,
        pause_time_ns: int,
        real_time_ns: int,
        paused: bool,
        iterations: int,
        model_count: int,
        real_time_factor: float,
        step_size_ns: int,
        stepping: bool,
    ) -> None:
        sim_time_ns = self._nonnegative_int(sim_time_ns, "simulation time")
        pause_time_ns = self._nonnegative_int(pause_time_ns, "pause time")
        real_time_ns = self._nonnegative_int(real_time_ns, "real time")
        iterations = self._nonnegative_int(iterations, "iterations")
        model_count = self._nonnegative_int(model_count, "model count")
        step_size_ns = self._nonnegative_int(step_size_ns, "step size")
        if not isinstance(paused, bool) or not isinstance(stepping, bool):
            raise ValueError("paused and stepping must be boolean")
        real_time_factor = _finite_float(
            real_time_factor, "reported real-time factor"
        )
        if real_time_factor < 0.0:
            raise ValueError("reported real-time factor must be non-negative")

        if self._last_sim_time_ns is not None:
            sim_delta = sim_time_ns - self._last_sim_time_ns
            real_delta = real_time_ns - self._last_real_time_ns
            iteration_delta = iterations - self._last_iterations
            nonmonotonic_sim = sim_delta < 0
            nonmonotonic_real = real_delta < 0
            nonmonotonic_iteration = iteration_delta < 0
            self.nonmonotonic_sim_time_count += int(nonmonotonic_sim)
            self.nonmonotonic_real_time_count += int(nonmonotonic_real)
            self.nonmonotonic_iteration_count += int(nonmonotonic_iteration)
            self.stalled_unpaused_sample_count += int(
                sim_delta == 0 and not paused
            )
            if real_delta > 0 and sim_delta >= 0:
                self._observed_rtfs.append(sim_delta / real_delta)
            if (
                not nonmonotonic_sim
                and sim_time_ns >= self._last_valid_sim_time_ns
            ):
                self._last_valid_sim_time_ns = sim_time_ns
            if (
                not nonmonotonic_real
                and real_time_ns >= self._last_valid_real_time_ns
            ):
                self._last_valid_real_time_ns = real_time_ns
            if (
                not nonmonotonic_iteration
                and iterations >= self._last_valid_iterations
            ):
                self._last_valid_iterations = iterations
        else:
            self._first_sim_time_ns = sim_time_ns
            self._first_real_time_ns = real_time_ns
            self._first_iterations = iterations
            self._last_valid_sim_time_ns = sim_time_ns
            self._last_valid_real_time_ns = real_time_ns
            self._last_valid_iterations = iterations

        self.sample_count += 1
        self.paused_sample_count += int(paused)
        self._reported_rtfs.append(real_time_factor)
        self._last_sim_time_ns = sim_time_ns
        self._last_real_time_ns = real_time_ns
        self._last_pause_time_ns = pause_time_ns
        self._last_iterations = iterations
        self._last_paused = paused
        self._last_stepping = stepping
        self._last_model_count = model_count
        self._last_step_size_ns = step_size_ns
        self._last_reported_rtf = real_time_factor

    @staticmethod
    def _mean_or_none(values: Sequence[float]) -> Optional[float]:
        return None if not values else float(np.mean(values))

    def snapshot(self) -> Dict[str, Any]:
        degraded = any((
            self.nonmonotonic_sim_time_count,
            self.nonmonotonic_real_time_count,
            self.nonmonotonic_iteration_count,
        ))
        return {
            "status": (
                "waiting_for_world_stats"
                if self.sample_count == 0
                else (
                    "degraded_nonmonotonic_clock"
                    if degraded
                    else "available"
                )
            ),
            "world_stats_sample_count": self.sample_count,
            "sim_time_first_ns": self._first_sim_time_ns,
            "sim_time_latest_ns": self._last_sim_time_ns,
            "sim_time_elapsed_ns": (
                None
                if self.sample_count == 0
                else self._last_valid_sim_time_ns - self._first_sim_time_ns
            ),
            "real_time_first_ns": self._first_real_time_ns,
            "real_time_latest_ns": self._last_real_time_ns,
            "real_time_elapsed_ns": (
                None
                if self.sample_count == 0
                else self._last_valid_real_time_ns - self._first_real_time_ns
            ),
            "pause_time_latest_ns": self._last_pause_time_ns,
            "paused_latest": self._last_paused,
            "stepping_latest": self._last_stepping,
            "paused_sample_count": self.paused_sample_count,
            "paused_sample_fraction": _fraction(
                self.paused_sample_count, self.sample_count
            ),
            "iterations_first": self._first_iterations,
            "iterations_latest": self._last_iterations,
            "iteration_delta": (
                None
                if self.sample_count == 0
                else self._last_valid_iterations - self._first_iterations
            ),
            "model_count_latest": self._last_model_count,
            "step_size_latest_ns": self._last_step_size_ns,
            "reported_real_time_factor_latest": self._last_reported_rtf,
            "reported_real_time_factor_mean": self._mean_or_none(
                self._reported_rtfs
            ),
            "observed_delta_real_time_factor_mean": self._mean_or_none(
                self._observed_rtfs
            ),
            "observed_delta_real_time_factor_sample_count": len(
                self._observed_rtfs
            ),
            "nonmonotonic_sim_time_count": (
                self.nonmonotonic_sim_time_count
            ),
            "nonmonotonic_real_time_count": (
                self.nonmonotonic_real_time_count
            ),
            "nonmonotonic_iteration_count": (
                self.nonmonotonic_iteration_count
            ),
            "stalled_unpaused_sample_count": (
                self.stalled_unpaused_sample_count
            ),
        }


class CollisionAccumulator:
    """Count debounced robot-obstacle contact-pair episodes.

    Gazebo may emit the same contact pair at every physics/sensor update.  A
    collision event is therefore the onset of a meaningful pair, not a raw
    contact point or message. A pair remains active until it has not been seen
    for ``event_separation_s``. This works when several link-local contact
    sensors publish interleaved messages on the same topic: an empty message
    from one sensor cannot end another sensor's collision episode.
    """

    def __init__(
        self,
        robot_name_tokens: Sequence[str],
        ground_name_tokens: Sequence[str],
        event_separation_s: float = 1.0,
        minimum_depth_m: float = 0.0,
    ):
        self.robot_name_tokens = self._tokens(
            robot_name_tokens, "robot collision name tokens"
        )
        self.ground_name_tokens = self._tokens(
            ground_name_tokens, "ground collision name tokens"
        )
        self.event_separation_ns = int(
            _finite_float(event_separation_s, "collision event separation")
            * 1e9
        )
        if self.event_separation_ns <= 0:
            raise ValueError("collision event separation must be positive")
        self.minimum_depth_m = _finite_float(
            minimum_depth_m, "collision minimum depth"
        )
        if self.minimum_depth_m < 0.0:
            raise ValueError("collision minimum depth must be non-negative")
        self.message_count = 0
        self.raw_contact_count = 0
        self.collision_event_count = 0
        self.time_reset_count = 0
        self.ignored_ground_contact_count = 0
        self.ignored_self_contact_count = 0
        self.ignored_unattributed_contact_count = 0
        self.ignored_below_depth_contact_count = 0
        self.malformed_contact_count = 0
        self.max_penetration_depth_m: Optional[float] = None
        self._active_pairs = set()
        self._last_seen_ns: Dict[Tuple[str, str], int] = {}
        self._pair_event_counts: Dict[str, int] = {}
        self._last_message_time_ns: Optional[int] = None

    @staticmethod
    def _tokens(values: Sequence[str], label: str) -> Tuple[str, ...]:
        if (
            not isinstance(values, Sequence)
            or isinstance(values, (str, bytes))
        ):
            raise ValueError(f"{label} must be a sequence")
        tokens = tuple(
            str(value).strip().casefold()
            for value in values
            if str(value).strip()
        )
        if not tokens:
            raise ValueError(f"{label} must not be empty")
        return tokens

    @staticmethod
    def _contains(name: str, tokens: Sequence[str]) -> bool:
        folded = name.casefold()
        return any(token in folded for token in tokens)

    def ingest(
        self,
        time_ns: int,
        contacts: Iterable[Sequence[Any]],
    ) -> int:
        """Ingest ``(collision1_name, collision2_name, max_depth_m)`` rows."""
        time_ns = int(time_ns)
        self.message_count += 1
        if (
            self._last_message_time_ns is not None
            and time_ns < self._last_message_time_ns
        ):
            self.time_reset_count += 1
            time_ns = self._last_message_time_ns
        self._last_message_time_ns = time_ns
        expired_pairs = {
            pair
            for pair in self._active_pairs
            if time_ns - self._last_seen_ns.get(pair, time_ns)
            >= self.event_separation_ns
        }
        self._active_pairs.difference_update(expired_pairs)
        for pair in expired_pairs:
            self._last_seen_ns.pop(pair, None)
        current_pairs = set()
        new_events = 0
        for raw in contacts:
            self.raw_contact_count += 1
            if (
                not isinstance(raw, Sequence)
                or isinstance(raw, (str, bytes))
                or len(raw) < 2
            ):
                self.malformed_contact_count += 1
                continue
            name1 = str(raw[0]).strip()
            name2 = str(raw[1]).strip()
            if not name1 or not name2:
                self.malformed_contact_count += 1
                continue
            try:
                depth = (
                    0.0
                    if len(raw) < 3 or raw[2] is None
                    else _finite_float(raw[2], "contact penetration depth")
                )
            except (TypeError, ValueError):
                self.malformed_contact_count += 1
                continue
            if depth < 0.0:
                self.malformed_contact_count += 1
                continue
            self.max_penetration_depth_m = (
                depth
                if self.max_penetration_depth_m is None
                else max(self.max_penetration_depth_m, depth)
            )
            robot1 = self._contains(name1, self.robot_name_tokens)
            robot2 = self._contains(name2, self.robot_name_tokens)
            if robot1 and robot2:
                self.ignored_self_contact_count += 1
                continue
            if not robot1 and not robot2:
                self.ignored_unattributed_contact_count += 1
                continue
            other = name2 if robot1 else name1
            if self._contains(other, self.ground_name_tokens):
                self.ignored_ground_contact_count += 1
                continue
            if depth + 1e-12 < self.minimum_depth_m:
                self.ignored_below_depth_contact_count += 1
                continue
            pair = tuple(sorted((name1, name2)))
            already_current = pair in current_pairs
            current_pairs.add(pair)
            if not already_current and (
                pair not in self._active_pairs
            ):
                self.collision_event_count += 1
                new_events += 1
                encoded_pair = " <-> ".join(pair)
                self._pair_event_counts[encoded_pair] = (
                    self._pair_event_counts.get(encoded_pair, 0) + 1
                )
            self._last_seen_ns[pair] = time_ns
            self._active_pairs.add(pair)
        return new_events

    def snapshot(self) -> Dict[str, Any]:
        attribution_complete = (
            self.malformed_contact_count == 0
            and self.ignored_unattributed_contact_count == 0
        )
        temporal_order_complete = self.time_reset_count == 0
        verification_complete = (
            attribution_complete and temporal_order_complete
        )
        if self.collision_event_count > 0:
            collision_free = False
        elif self.message_count == 0 or not verification_complete:
            collision_free = None
        else:
            collision_free = True
        return {
            "status": (
                "waiting_for_contacts"
                if self.message_count == 0
                else (
                    "available"
                    if verification_complete
                    else "degraded_unverified_contact_stream"
                )
            ),
            "collision_count": self.collision_event_count,
            "collision_free": collision_free,
            "collision_free_scope": "configured_contact_sensor_collisions",
            "contact_attribution_complete": attribution_complete,
            "contact_temporal_order_complete": temporal_order_complete,
            "collision_count_semantics": (
                "debounced onset count of attributed robot/non-ground "
                "collision-name pairs; pair ends after event_separation_s "
                "without an observation"
            ),
            "contact_message_count": self.message_count,
            "raw_contact_count": self.raw_contact_count,
            "active_meaningful_pair_count": len(self._active_pairs),
            "ignored_ground_contact_count": self.ignored_ground_contact_count,
            "ignored_self_contact_count": self.ignored_self_contact_count,
            "ignored_unattributed_contact_count": (
                self.ignored_unattributed_contact_count
            ),
            "ignored_below_depth_contact_count": (
                self.ignored_below_depth_contact_count
            ),
            "malformed_contact_count": self.malformed_contact_count,
            "contact_nonmonotonic_stamp_count": self.time_reset_count,
            "maximum_reported_penetration_depth_m": (
                self.max_penetration_depth_m
            ),
            "pair_event_counts": dict(sorted(self._pair_event_counts.items())),
            "minimum_depth_m": self.minimum_depth_m,
            "event_separation_s": self.event_separation_ns / 1e9,
            "limitations": [
                "name-token attribution; empty entity names are not counted",
                "all configured floor/ground pairs are excluded, including "
                "possible abnormal chassis-floor contacts",
                "a bridge gap longer than event_separation_s can split one "
                "sustained contact episode",
                "coverage is limited to collision geometries exported by the "
                "configured Gazebo contact sensor",
                "non-monotonic contact stamps are clamped and make a "
                "zero-collision result unverified",
            ],
        }


@dataclass(frozen=True)
class TargetSpec:
    target_id: str
    class_name: str
    x_m: float
    y_m: float
    z_m: float
    surface_normal_yaw_rad: float


@dataclass(frozen=True)
class CameraGeometry:
    x_offset_m: float = 0.2
    y_offset_m: float = 0.0
    height_m: float = 0.27
    yaw_offset_rad: float = 0.0
    pitch_rad: float = 0.0
    horizontal_fov_rad: float = 1.22173
    vertical_fov_rad: float = 0.9671376131
    minimum_range_m: float = 0.1
    maximum_range_m: float = 8.0
    maximum_incidence_rad: float = math.radians(80.0)
    los_endpoint_clearance_m: float = 0.1

    def __post_init__(self) -> None:
        for name in (
            "x_offset_m",
            "y_offset_m",
            "height_m",
            "yaw_offset_rad",
            "pitch_rad",
            "horizontal_fov_rad",
            "vertical_fov_rad",
            "minimum_range_m",
            "maximum_range_m",
            "maximum_incidence_rad",
            "los_endpoint_clearance_m",
        ):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name))
        if not 0.0 < self.horizontal_fov_rad < 2.0 * math.pi:
            raise ValueError("camera horizontal FOV must be in (0, 2*pi)")
        if not 0.0 < self.vertical_fov_rad < math.pi:
            raise ValueError("camera vertical FOV must be in (0, pi)")
        if self.minimum_range_m < 0.0:
            raise ValueError("camera minimum range must be non-negative")
        if self.maximum_range_m <= self.minimum_range_m:
            raise ValueError("camera maximum range must exceed minimum range")
        if not 0.0 <= self.maximum_incidence_rad <= math.pi:
            raise ValueError("maximum target incidence must be in [0, pi]")
        if self.los_endpoint_clearance_m < 0.0:
            raise ValueError("LOS endpoint clearance must be non-negative")


def load_target_registry(
    targets_yaml: Path | str,
) -> Tuple[str, Tuple[TargetSpec, ...], str]:
    """Load and hash an evaluator-only target registry."""
    path = Path(targets_yaml).expanduser().resolve()
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("target registry must contain a mapping")
    if not str(document.get("schema", "")).startswith(
        "sstg_system_sim_targets/"
    ):
        raise ValueError("target registry has an unsupported schema")
    world_id = str(document.get("world_id", "")).strip()
    if not world_id:
        raise ValueError("target registry world_id is required")
    rows = document.get("targets")
    if not isinstance(rows, list) or not rows:
        raise ValueError("target registry must contain at least one target")
    targets = []
    identifiers = set()
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"target {index} must be a mapping")
        target_id = str(row.get("target_id", "")).strip()
        if not target_id or target_id in identifiers:
            raise ValueError("target IDs must be non-empty and unique")
        identifiers.add(target_id)
        targets.append(TargetSpec(
            target_id=target_id,
            class_name=str(row.get("class", "unspecified")),
            x_m=_finite_float(row.get("x_m"), f"target {target_id} x"),
            y_m=_finite_float(row.get("y_m"), f"target {target_id} y"),
            z_m=_finite_float(row.get("z_m"), f"target {target_id} z"),
            surface_normal_yaw_rad=math.radians(_finite_float(
                row.get("surface_normal_yaw_deg"),
                f"target {target_id} surface normal yaw",
            )),
        ))
    return world_id, tuple(targets), hashlib.sha256(path.read_bytes()).hexdigest()


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _truth_los_clear(
    truth: TruthGrid,
    start: Tuple[float, float],
    end: Tuple[float, float],
    endpoint_clearance_m: float,
) -> bool:
    """Conservative 2-D LOS: occupied, unknown and out-of-map cells block."""
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    checked_distance = max(0.0, distance - endpoint_clearance_m)
    if checked_distance <= 0.0:
        return True
    step = max(1e-6, truth.resolution * 0.5)
    sample_count = max(1, int(math.ceil(checked_distance / step)))
    cosine = math.cos(truth.origin_yaw)
    sine = math.sin(truth.origin_yaw)
    for index in range(1, sample_count + 1):
        travel = min(index * step, checked_distance)
        fraction = travel / distance
        x = start[0] + fraction * (end[0] - start[0])
        y = start[1] + fraction * (end[1] - start[1])
        delta_x = x - truth.origin[0]
        delta_y = y - truth.origin[1]
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = int(math.floor(local_x / truth.resolution))
        row = int(math.floor(local_y / truth.resolution))
        if not (0 <= row < truth.shape[0] and 0 <= column < truth.shape[1]):
            return False
        if not truth.free[row, column]:
            return False
    return True


def evaluate_target_visibility(
    truth: TruthGrid,
    target: TargetSpec,
    base_pose_truth: Sequence[float],
    camera: CameraGeometry,
) -> Dict[str, Any]:
    """Evaluate the frozen deterministic camera/occupancy visibility proxy."""
    if (
        not isinstance(base_pose_truth, Sequence)
        or isinstance(base_pose_truth, (str, bytes))
        or len(base_pose_truth) < 3
    ):
        raise ValueError("base truth pose must contain x, y and yaw")
    base_x = _finite_float(base_pose_truth[0], "base truth x")
    base_y = _finite_float(base_pose_truth[1], "base truth y")
    base_yaw = _finite_float(base_pose_truth[2], "base truth yaw")
    cosine, sine = math.cos(base_yaw), math.sin(base_yaw)
    camera_x = (
        base_x + cosine * camera.x_offset_m - sine * camera.y_offset_m
    )
    camera_y = (
        base_y + sine * camera.x_offset_m + cosine * camera.y_offset_m
    )
    camera_yaw = base_yaw + camera.yaw_offset_rad
    delta_x = target.x_m - camera_x
    delta_y = target.y_m - camera_y
    horizontal_range = math.hypot(delta_x, delta_y)
    range_m = math.hypot(horizontal_range, target.z_m - camera.height_m)
    horizontal_angle = _wrap_angle(math.atan2(delta_y, delta_x) - camera_yaw)
    vertical_angle = math.atan2(
        target.z_m - camera.height_m, horizontal_range
    ) - camera.pitch_rad
    target_to_camera_bearing = math.atan2(camera_y - target.y_m, camera_x - target.x_m)
    incidence = abs(_wrap_angle(
        target_to_camera_bearing - target.surface_normal_yaw_rad
    ))
    reason = "visible"
    if not camera.minimum_range_m <= range_m <= camera.maximum_range_m:
        reason = "range"
    elif abs(horizontal_angle) > camera.horizontal_fov_rad * 0.5 + 1e-12:
        reason = "horizontal_fov"
    elif abs(vertical_angle) > camera.vertical_fov_rad * 0.5 + 1e-12:
        reason = "vertical_fov"
    elif incidence > camera.maximum_incidence_rad + 1e-12:
        reason = "backface_or_oblique"
    elif not _truth_los_clear(
        truth,
        (camera_x, camera_y),
        (target.x_m, target.y_m),
        camera.los_endpoint_clearance_m,
    ):
        reason = "occluded_2d_truth"
    return {
        "visible": reason == "visible",
        "reason": reason,
        "range_m": range_m,
        "horizontal_angle_rad": horizontal_angle,
        "vertical_angle_rad": vertical_angle,
        "surface_incidence_rad": incidence,
        "camera_truth_position": [camera_x, camera_y, camera.height_m],
    }


class TargetRecallAccumulator:
    """First-seen target recall from ground-truth trajectory geometry only."""

    def __init__(
        self,
        truth: TruthGrid,
        targets: Sequence[TargetSpec],
        camera: CameraGeometry,
    ):
        if not targets:
            raise ValueError("at least one target is required")
        identifiers = [target.target_id for target in targets]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("target IDs must be unique")
        self.truth = truth
        self.targets = tuple(targets)
        self.camera = camera
        self.pose_sample_count = 0
        self.pre_origin_pose_count = 0
        self.time_reset_count = 0
        self._first_pose_time_ns: Optional[int] = None
        self._time_origin_ns: Optional[int] = None
        self._last_pose_time_ns: Optional[int] = None
        self._detections: Dict[str, Dict[str, Any]] = {}

    def begin_session(self, time_ns: int) -> None:
        """Freeze a policy-session time origin and clear prior detections."""
        self._time_origin_ns = int(time_ns)
        self._first_pose_time_ns = None
        self._last_pose_time_ns = None
        self.pose_sample_count = 0
        self.pre_origin_pose_count = 0
        self._detections.clear()

    def ingest(
        self, time_ns: int, base_pose_truth: Sequence[float]
    ) -> Tuple[str, ...]:
        time_ns = int(time_ns)
        if self._last_pose_time_ns is not None and time_ns < self._last_pose_time_ns:
            self.time_reset_count += 1
            self._first_pose_time_ns = None
            self._time_origin_ns = None
            self._detections.clear()
        if self._time_origin_ns is not None and time_ns < self._time_origin_ns:
            self.pre_origin_pose_count += 1
            return ()
        self._last_pose_time_ns = time_ns
        if self._first_pose_time_ns is None:
            self._first_pose_time_ns = time_ns
        if self._time_origin_ns is None:
            self._time_origin_ns = self._first_pose_time_ns
        self.pose_sample_count += 1
        newly_detected = []
        for target in self.targets:
            if target.target_id in self._detections:
                continue
            evidence = evaluate_target_visibility(
                self.truth, target, base_pose_truth, self.camera
            )
            if not evidence["visible"]:
                continue
            evidence = dict(evidence)
            evidence.update({
                "target_id": target.target_id,
                "first_seen_ros_time_ns": time_ns,
                "first_seen_elapsed_s": (
                    time_ns - self._time_origin_ns
                ) / 1e9,
            })
            self._detections[target.target_id] = evidence
            newly_detected.append(target.target_id)
        return tuple(newly_detected)

    def snapshot(self) -> Dict[str, Any]:
        detected_ids = sorted(self._detections)
        return {
            "status": (
                "available"
                if self.pose_sample_count
                else "waiting_for_ground_truth"
            ),
            "detection_model": "deterministic_geometry_proxy_v1",
            "detection_model_is_image_detector": False,
            "target_total_count": len(self.targets),
            "detected_target_count": len(detected_ids),
            "detected_target_ids": detected_ids,
            "target_recall": len(detected_ids) / len(self.targets),
            "pose_sample_count": self.pose_sample_count,
            "pre_origin_pose_count": self.pre_origin_pose_count,
            "target_time_reset_count": self.time_reset_count,
            "time_origin_ros_time_ns": self._time_origin_ns,
            "first_detections": {
                target_id: dict(self._detections[target_id])
                for target_id in detected_ids
            },
            "camera_geometry": {
                "x_offset_m": self.camera.x_offset_m,
                "y_offset_m": self.camera.y_offset_m,
                "height_m": self.camera.height_m,
                "yaw_offset_rad": self.camera.yaw_offset_rad,
                "pitch_rad": self.camera.pitch_rad,
                "horizontal_fov_rad": self.camera.horizontal_fov_rad,
                "vertical_fov_rad": self.camera.vertical_fov_rad,
                "minimum_range_m": self.camera.minimum_range_m,
                "maximum_range_m": self.camera.maximum_range_m,
                "maximum_incidence_rad": self.camera.maximum_incidence_rad,
                "los_endpoint_clearance_m": (
                    self.camera.los_endpoint_clearance_m
                ),
            },
            "limitations": [
                "trajectory-only geometric proxy; no camera pixels or detector",
                "2-D truth occupancy makes LOS conservative and does not model "
                "object height or transparency",
                "visibility at a sampled pose counts as detection with no "
                "blur, illumination, dwell-time or confidence model",
                "FOV, range and facing tests use the target center point, not "
                "the panel's projected area",
            ],
        }


class TopologicalNodeAccumulator:
    """Extract actual created topology nodes from accepted policy traces."""

    def __init__(self, deduplication_tolerance_m: float = 0.01):
        self.deduplication_tolerance_m = _finite_float(
            deduplication_tolerance_m, "node deduplication tolerance"
        )
        if self.deduplication_tolerance_m < 0.0:
            raise ValueError("node deduplication tolerance must be non-negative")
        self.raw_node_observation_count = 0
        self.initial_node_observation_count = 0
        self.execution_node_observation_count = 0
        self.duplicate_node_observation_count = 0
        self.topological_node_event_count = 0
        self.topological_node_merged_event_count = 0
        self.topological_node_trigger_counts: Dict[str, int] = {}
        self._topological_events: Dict[str, Dict[str, Any]] = {}
        self.nodes = []

    @staticmethod
    def _position(value: Any, label: str) -> Tuple[float, float]:
        if (
            not isinstance(value, Sequence)
            or isinstance(value, (str, bytes))
            or len(value) < 2
        ):
            raise ValueError(f"{label} must contain x and y")
        return (
            _finite_float(value[0], f"{label} x"),
            _finite_float(value[1], f"{label} y"),
        )

    def _add(
        self,
        position: Tuple[float, float],
        source: str,
        source_id: Any,
        trigger: str,
    ) -> bool:
        self.raw_node_observation_count += 1
        if source == "session_started":
            self.initial_node_observation_count += 1
        else:
            self.execution_node_observation_count += 1
        duplicate_of = next((
            node["unique_node_index"]
            for node in self.nodes
            if math.hypot(
                position[0] - node["x_m"], position[1] - node["y_m"]
            ) <= self.deduplication_tolerance_m + 1e-12
        ), None)
        if duplicate_of is not None:
            self.duplicate_node_observation_count += 1
            return False
        self.nodes.append({
            "unique_node_index": len(self.nodes),
            "x_m": position[0],
            "y_m": position[1],
            "source": source,
            "source_id": source_id,
            "trigger": trigger,
        })
        return True

    def _ingest_topological_node_event(self, payload: Dict[str, Any]) -> int:
        event_id = payload.get("event_id")
        trigger = payload.get("trigger")
        confirmation = payload.get("confirmation")
        created = payload.get("created")
        if not isinstance(event_id, str) or not event_id:
            raise ValueError("topological_node event_id must be non-empty")
        if event_id in self._topological_events:
            raise ValueError("duplicate topological_node event_id")
        if trigger not in {
            "navigation_succeeded",
            "upstream_cancel_request",
            "nav2_native_preemption",
        }:
            raise ValueError("unsupported topological_node trigger")
        if not isinstance(confirmation, str) or not confirmation:
            raise ValueError("topological_node confirmation must be non-empty")
        allowed_confirmations = {
            "navigation_succeeded": {"downstream_result_succeeded"},
            "nav2_native_preemption": {"replacement_goal_accepted"},
            "upstream_cancel_request": {
                "downstream_cancel_accepted",
                "downstream_result_canceled",
                "replacement_goal_accepted",
            },
        }
        if confirmation not in allowed_confirmations[trigger]:
            raise ValueError(
                "topological_node confirmation disagrees with its trigger"
            )
        if not isinstance(created, bool):
            raise ValueError("topological_node created must be boolean")
        decision_id = int(payload.get("decision_id"))
        causal_ros_time_ns = int(payload.get("causal_ros_time_ns"))
        node_id = int(payload.get("node_id"))
        if decision_id <= 0 or causal_ros_time_ns < 0 or node_id < 0:
            raise ValueError("topological_node identifiers must be non-negative")
        position = self._position(payload.get("pose"), "topological_node pose")
        nearest_distance = _finite_float(
            payload.get("nearest_node_distance_m"),
            "topological_node nearest distance",
        )
        merge_distance = _finite_float(
            payload.get("merge_distance_m"),
            "topological_node merge distance",
        )
        if nearest_distance < 0.0 or merge_distance < 0.0:
            raise ValueError("topological_node distances must be non-negative")
        if not self.nodes:
            raise ValueError("topological_node event requires an initial node")
        recomputed_distances = [
            math.hypot(
                position[0] - node["x_m"], position[1] - node["y_m"]
            )
            for node in self.nodes
        ]
        nearest_index = min(
            range(len(recomputed_distances)), key=recomputed_distances.__getitem__
        )
        recomputed_nearest = recomputed_distances[nearest_index]
        if not math.isclose(
            nearest_distance, recomputed_nearest, rel_tol=0.0, abs_tol=1e-6
        ):
            raise ValueError(
                "topological_node nearest distance disagrees with prior nodes"
            )
        expected_created = recomputed_nearest >= merge_distance
        if created != expected_created:
            raise ValueError(
                "topological_node created flag disagrees with merge threshold"
            )
        expected_node_id = len(self.nodes) if created else nearest_index
        if node_id != expected_node_id:
            raise ValueError("topological_node node_id disagrees with node order")
        self.topological_node_event_count += 1
        self.topological_node_merged_event_count += int(not created)
        self.topological_node_trigger_counts[trigger] = (
            self.topological_node_trigger_counts.get(trigger, 0) + 1
        )
        event_record = {
            "decision_id": decision_id,
            "trigger": trigger,
            "created": created,
            "pose": position,
        }
        self._topological_events[event_id] = event_record
        if not created:
            return 0
        return int(self._add(
            position, "topological_node", event_id, trigger
        ))

    def ingest_record(self, record: Dict[str, Any]) -> int:
        event = record.get("event")
        payload = record.get("payload")
        if not isinstance(payload, dict):
            raise ValueError("topology trace payload must be an object")
        candidates = []
        if event == "session_started":
            nodes = payload.get("nodes")
            if not isinstance(nodes, list) or not nodes:
                raise ValueError("session_started must contain at least one node")
            for index, node in enumerate(nodes):
                if not isinstance(node, dict):
                    raise ValueError("session_started nodes must be objects")
                candidates.append((
                    self._position(node.get("position"), f"initial node {index}"),
                    "session_started",
                    node.get("id", index),
                    "initial_pose",
                ))
        elif event == "topological_node":
            return self._ingest_topological_node_event(payload)
        elif event == "execution":
            succeeded = payload.get("succeeded")
            created = payload.get("topological_node_created")
            if not isinstance(succeeded, bool):
                raise ValueError("execution succeeded must be boolean")
            if not isinstance(created, bool):
                raise ValueError("execution topological_node_created must be boolean")
            node_event_id = payload.get("topological_node_event_id")
            if node_event_id is not None:
                if not isinstance(node_event_id, str):
                    raise ValueError(
                        "execution topological_node_event_id must be a string"
                    )
                node_event = self._topological_events.get(node_event_id)
                if node_event is None:
                    raise ValueError(
                        "execution references an unseen topological_node event"
                    )
                if (
                    node_event["decision_id"] != int(payload.get("decision_id"))
                    or node_event["created"] != created
                    or node_event["trigger"] != payload.get(
                        "topological_node_trigger"
                    )
                    or node_event["pose"] != self._position(
                        payload.get("topological_node_pose"),
                        "execution topological_node_pose",
                    )
                ):
                    raise ValueError(
                        "execution disagrees with its topological_node event"
                    )
                return 0
            if succeeded and created:
                candidates.append((
                    self._position(payload.get("reached_pose"), "reached_pose"),
                    "execution",
                    payload.get("decision_id"),
                    "legacy_navigation_succeeded",
                ))
        return sum(self._add(*candidate) for candidate in candidates)

    @property
    def positions(self):
        return [(node["x_m"], node["y_m"]) for node in self.nodes]

    def snapshot(self) -> Dict[str, Any]:
        return {
            "raw_node_observation_count": self.raw_node_observation_count,
            "initial_node_observation_count": self.initial_node_observation_count,
            "execution_node_observation_count": (
                self.execution_node_observation_count
            ),
            "duplicate_node_observation_count": (
                self.duplicate_node_observation_count
            ),
            "topological_node_event_count": self.topological_node_event_count,
            "topological_node_merged_event_count": (
                self.topological_node_merged_event_count
            ),
            "topological_node_trigger_counts": dict(sorted(
                self.topological_node_trigger_counts.items()
            )),
            "unique_node_count": len(self.nodes),
            "node_deduplication_tolerance_m": self.deduplication_tolerance_m,
            "unique_nodes": [dict(node) for node in self.nodes],
        }


class ActionTraceAccumulator:
    """Validate and aggregate the policy's public JSON trace events."""

    def __init__(self):
        self._seen = set()
        self.latest_record: Optional[Dict[str, Any]] = None
        self.accepted_trace_events = 0
        self.decision_count = 0
        self.navigation_goal_count = 0
        self.execution_count = 0
        self.navigation_success_count = 0
        self.navigation_failure_count = 0
        self.navigation_canceled_count = 0
        self.navigation_upstream_cancel_count = 0
        self.navigation_adapter_cancel_count = 0
        self.navigation_non_cancel_failure_count = 0
        self.navigation_policy_transition_count = 0
        self.navigation_technical_failure_count = 0
        self.decision_error_count = 0
        self.session_finished_count = 0
        self.decision_time_ms_total = 0.0
        self.decision_time_observed_count = 0
        self.decision_time_unavailable_count = 0
        self.trace_reported_translation_m = 0.0
        self.trace_recomputed_path_length_m = 0.0
        self.trace_reported_rotation_deg = 0.0
        self.latest_decision_id: Optional[int] = None
        self.execution_reasons: Dict[str, int] = {}

    def ingest(self, encoded: str) -> bool:
        try:
            record = json.loads(encoded, parse_constant=_reject_json_constant)
        except json.JSONDecodeError as error:
            raise ValueError(f"policy trace is not valid JSON: {error.msg}") from error
        if not isinstance(record, dict):
            raise ValueError("policy trace record must be a JSON object")
        event = record.get("event")
        payload = record.get("payload")
        if not isinstance(event, str) or not isinstance(payload, dict):
            raise ValueError("policy trace requires string event and object payload")
        identity = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        if identity in self._seen:
            return False

        if event == "decision":
            decision_id = int(payload["decision_id"])
            raw_decision_time = payload.get("decision_time_ms")
            if raw_decision_time is None:
                if payload.get("decision_time_semantics") != (
                    "unavailable_upstream_internal_compute_time"
                ):
                    raise ValueError(
                        "unavailable decision_time_ms requires an explicit "
                        "upstream-unavailable semantic"
                    )
                self.decision_time_unavailable_count += 1
            else:
                decision_time = _finite_float(
                    raw_decision_time, "decision_time_ms"
                )
                if decision_time < 0.0:
                    raise ValueError("decision_time_ms must be non-negative")
                self.decision_time_ms_total += decision_time
                self.decision_time_observed_count += 1
            status = str(payload.get("status", ""))
            self.decision_count += 1
            self.navigation_goal_count += int(status == "navigate")
            self.latest_decision_id = decision_id
        elif event == "execution":
            decision_id = int(payload["decision_id"])
            succeeded = payload.get("succeeded")
            if not isinstance(succeeded, bool):
                raise ValueError("execution succeeded must be boolean")
            translation = _finite_float(
                payload.get("translation_m", 0.0), "translation_m"
            )
            rotation = _finite_float(
                payload.get("rotation_deg", 0.0), "rotation_deg"
            )
            if translation < 0.0 or rotation < 0.0:
                raise ValueError("execution motion metrics must be non-negative")
            path = payload.get("path", [])
            if not isinstance(path, list):
                raise ValueError("execution path must be a list")
            reason = str(payload.get("reason", "unspecified"))
            recomputed = _polyline_length(path)
            failed = not succeeded
            canceled = failed and payload.get("nav2_status") == 5
            cancel_origin = payload.get("cancel_origin")
            topological_eligibility = payload.get(
                "topological_node_eligibility"
            )
            if topological_eligibility not in {
                None,
                "navigation_succeeded",
                "policy_transition",
                "ineligible_action_outcome",
            }:
                raise ValueError(
                    "execution has unsupported topological_node_eligibility"
                )
            policy_transition = topological_eligibility == "policy_transition"
            if policy_transition and (
                payload.get("topological_node_trigger") not in {
                    "upstream_cancel_request",
                    "nav2_native_preemption",
                }
                or not isinstance(payload.get("topological_node_event_id"), str)
            ):
                raise ValueError(
                    "policy transition execution lacks its causal node event"
                )
            if topological_eligibility == "navigation_succeeded" and (
                not succeeded
                or payload.get("topological_node_trigger") != (
                    "navigation_succeeded"
                )
                or not isinstance(payload.get("topological_node_event_id"), str)
            ):
                raise ValueError(
                    "navigation-succeeded topology execution lacks its node event"
                )
            if topological_eligibility == "ineligible_action_outcome" and (
                payload.get("topological_node_event_id") is not None
                or payload.get("topological_node_created") is True
            ):
                raise ValueError(
                    "ineligible topology execution references a created node"
                )
            adapter_session_stop = (
                cancel_origin == "adapter_session_termination"
            )
            self.execution_count += 1
            self.navigation_success_count += int(succeeded)
            self.navigation_failure_count += int(failed)
            self.navigation_canceled_count += int(canceled)
            self.navigation_non_cancel_failure_count += int(
                failed and not canceled
            )
            self.navigation_policy_transition_count += int(policy_transition)
            self.navigation_technical_failure_count += int(
                failed and not policy_transition and not adapter_session_stop
            )
            if canceled and isinstance(cancel_origin, str):
                if cancel_origin in {
                    "upstream_cancel_request",
                    "nav2_native_preemption",
                }:
                    self.navigation_upstream_cancel_count += 1
                elif cancel_origin.startswith("adapter_"):
                    self.navigation_adapter_cancel_count += 1
            self.trace_reported_translation_m += translation
            self.trace_recomputed_path_length_m += recomputed
            self.trace_reported_rotation_deg += rotation
            self.latest_decision_id = decision_id
            self.execution_reasons[reason] = self.execution_reasons.get(reason, 0) + 1
        elif event == "decision_error":
            self.decision_error_count += 1
        elif event == "session_finished":
            self.session_finished_count += 1

        self._seen.add(identity)
        self.latest_record = record
        self.accepted_trace_events += 1
        return True

    def snapshot(self) -> Dict[str, Any]:
        success_rate = _fraction(
            self.navigation_success_count, self.execution_count
        )
        average_decision_time = (
            self.decision_time_ms_total / self.decision_time_observed_count
            if self.decision_time_observed_count else None
        )
        return {
            "accepted_trace_events": self.accepted_trace_events,
            "decision_count": self.decision_count,
            "navigation_goal_count": self.navigation_goal_count,
            "execution_count": self.execution_count,
            "navigation_success_count": self.navigation_success_count,
            "navigation_failure_count": self.navigation_failure_count,
            "navigation_canceled_count": self.navigation_canceled_count,
            "navigation_upstream_cancel_count": (
                self.navigation_upstream_cancel_count
            ),
            "navigation_adapter_cancel_count": (
                self.navigation_adapter_cancel_count
            ),
            "navigation_non_cancel_failure_count": (
                self.navigation_non_cancel_failure_count
            ),
            "navigation_policy_transition_count": (
                self.navigation_policy_transition_count
            ),
            "navigation_technical_failure_count": (
                self.navigation_technical_failure_count
            ),
            "navigation_success_rate": success_rate,
            "decision_error_count": self.decision_error_count,
            "session_finished_count": self.session_finished_count,
            "decision_time_ms_total": self.decision_time_ms_total,
            "decision_time_ms_mean": average_decision_time,
            "decision_time_observed_count": self.decision_time_observed_count,
            "decision_time_unavailable_count": (
                self.decision_time_unavailable_count
            ),
            "trace_reported_translation_m": self.trace_reported_translation_m,
            "trace_recomputed_path_length_m": self.trace_recomputed_path_length_m,
            "trace_translation_disagreement_m": (
                self.trace_reported_translation_m
                - self.trace_recomputed_path_length_m
            ),
            "trace_reported_rotation_deg": self.trace_reported_rotation_deg,
            "latest_decision_id": self.latest_decision_id,
            "execution_reasons": dict(sorted(self.execution_reasons.items())),
        }
