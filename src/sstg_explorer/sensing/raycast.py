"""Vectorized 2D ray-casting sensor used by the unknown-map protocol."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from sstg_explorer.map.occupancy_grid import OccupancyGrid


@dataclass(frozen=True)
class SensorConfig:
    """Idealized planar range sensor parameters.

    ``field_of_view_deg=360`` models a spinning planar LiDAR. Partial fields
    model a directional LiDAR/depth-camera slice. Rays stop at the first
    occupied cell, so free space behind a wall remains unknown.
    """

    field_of_view_deg: float = 360.0
    max_range: float = 12.0
    angular_resolution_deg: float = 0.25
    ray_step_factor: float = 0.5
    obstacle_threshold: int = 50

    def __post_init__(self):
        if not 0.0 < self.field_of_view_deg <= 360.0:
            raise ValueError("field_of_view_deg must be in (0, 360]")
        if self.max_range <= 0.0:
            raise ValueError("max_range must be positive")
        if self.angular_resolution_deg <= 0.0:
            raise ValueError("angular_resolution_deg must be positive")
        if not 0.0 < self.ray_step_factor <= 1.0:
            raise ValueError("ray_step_factor must be in (0, 1]")

    @property
    def key(self) -> str:
        return f"fov{int(round(self.field_of_view_deg))}_r{self.max_range:g}"


@dataclass
class RayObservation:
    """Cells visible in one scan and cells newly added to the belief map."""

    visible_mask: np.ndarray
    new_mask: np.ndarray
    visible_flat_indices: np.ndarray
    new_flat_indices: np.ndarray
    new_free_count: int
    new_occupied_count: int


class RaycastSensor:
    """Occlusion-aware ideal range sensor with vectorized ray marching."""

    def __init__(self, config: SensorConfig):
        self.config = config

    def _angles(self, heading_deg: float) -> np.ndarray:
        fov = self.config.field_of_view_deg
        if fov >= 360.0 - 1e-9:
            count = max(1, int(np.ceil(360.0 / self.config.angular_resolution_deg)))
            relative = np.arange(count, dtype=float) * (360.0 / count) - 180.0
        else:
            count = max(2, int(np.ceil(fov / self.config.angular_resolution_deg)) + 1)
            relative = np.linspace(-fov / 2.0, fov / 2.0, count)
        return np.deg2rad(heading_deg + relative)

    def visible_mask(
        self,
        grid: OccupancyGrid,
        position: Tuple[float, float],
        heading_deg: float,
    ) -> np.ndarray:
        """Return cells reached before or at the first known obstacle per ray."""
        angles = self._angles(heading_deg)
        step = grid.resolution * self.config.ray_step_factor
        distances = np.arange(0.0, self.config.max_range + step * 0.5, step)

        xs = position[0] + np.cos(angles)[:, None] * distances[None, :]
        ys = position[1] + np.sin(angles)[:, None] * distances[None, :]
        cols = np.floor((xs - grid.origin[0]) / grid.resolution).astype(np.int32)
        rows = np.floor((ys - grid.origin[1]) / grid.resolution).astype(np.int32)
        valid = (
            (rows >= 0) & (rows < grid.height) &
            (cols >= 0) & (cols < grid.width)
        )

        clipped_rows = np.clip(rows, 0, grid.height - 1)
        clipped_cols = np.clip(cols, 0, grid.width - 1)
        values = grid.data[clipped_rows, clipped_cols]
        stop = (~valid) | (values >= self.config.obstacle_threshold)
        has_stop = np.any(stop, axis=1)
        first_stop = np.where(has_stop, np.argmax(stop, axis=1), stop.shape[1] - 1)
        sample_index = np.arange(stop.shape[1])[None, :]
        reached = valid & (sample_index <= first_stop[:, None])

        flat = (
            rows[reached].astype(np.int64) * grid.width +
            cols[reached].astype(np.int64)
        )
        mask = np.zeros(grid.data.size, dtype=bool)
        if flat.size:
            mask[np.unique(flat)] = True
        return mask.reshape(grid.shape)

    def observe(
        self,
        truth: OccupancyGrid,
        belief: OccupancyGrid,
        position: Tuple[float, float],
        heading_deg: float,
    ) -> RayObservation:
        """Update ``belief`` in-place using one ground-truth sensor scan."""
        if truth.shape != belief.shape or truth.resolution != belief.resolution:
            raise ValueError("truth and belief grids must share shape and resolution")
        visible = self.visible_mask(truth, position, heading_deg)
        new = visible & belief.get_unknown_mask()
        belief.data[visible] = truth.data[visible]
        visible_flat = np.flatnonzero(visible)
        new_flat = np.flatnonzero(new)
        new_values = truth.data.ravel()[new_flat]
        return RayObservation(
            visible_mask=visible,
            new_mask=new,
            visible_flat_indices=visible_flat,
            new_flat_indices=new_flat,
            new_free_count=int(np.sum((new_values >= 0) & (new_values < self.config.obstacle_threshold))),
            new_occupied_count=int(np.sum(new_values >= self.config.obstacle_threshold)),
        )

    def predict_unknown_gain(
        self,
        belief: OccupancyGrid,
        position: Tuple[float, float],
        heading_deg: float,
    ) -> int:
        """Optimistic unknown-cell gain using only currently known obstacles."""
        visible = self.visible_mask(belief, position, heading_deg)
        return int(np.sum(visible & belief.get_unknown_mask()))
