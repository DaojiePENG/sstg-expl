"""Regression tests for the unknown-map range sensor."""
import numpy as np

from sstg_explorer.map import OccupancyGrid
from sstg_explorer.sensing import RaycastSensor, SensorConfig
from sstg_explorer.unknown import UnknownExplorerConfig, UnknownMapExplorer
from sstg_explorer.environments import create_environment


def _wall_grid():
    data = np.zeros((100, 100), dtype=np.int8)
    data[:, 50:52] = 100
    return OccupancyGrid(data, resolution=0.1, origin=(0.0, 0.0))


def test_raycast_obstacle_occludes_cells_behind_wall():
    truth = _wall_grid()
    sensor = RaycastSensor(SensorConfig(90.0, 8.0, angular_resolution_deg=0.5))
    visible = sensor.visible_mask(truth, (2.0, 5.0), 0.0)

    assert visible[50, 50]  # First wall surface is visible.
    assert not visible[50, 70]  # Cells behind it are occluded.


def test_partial_fov_respects_heading_and_updates_only_visible_cells():
    truth = _wall_grid()
    belief = OccupancyGrid(
        np.full_like(truth.data, -1), truth.resolution, truth.origin
    )
    sensor = RaycastSensor(SensorConfig(90.0, 3.0, angular_resolution_deg=0.5))
    observation = sensor.observe(truth, belief, (2.0, 5.0), 0.0)

    assert observation.new_free_count > 0
    assert belief.get_value(3.0, 5.0) == 0
    assert belief.get_value(1.0, 5.0) == -1  # Behind a forward-facing sensor.

    full = RaycastSensor(SensorConfig(360.0, 3.0, angular_resolution_deg=0.5))
    assert full.visible_mask(truth, (2.0, 5.0), 0.0)[50, 10]


def test_unknown_explorer_reaches_target_without_leaking_false_cells():
    environment = create_environment("multiple_rooms")
    truth = environment.get_occupancy_map()
    explorer = UnknownMapExplorer(UnknownExplorerConfig(
        strategy="sstg",
        sensor=SensorConfig(120.0, 8.0, angular_resolution_deg=0.5),
        target_coverage=0.80,
        max_decisions=25,
        seed=42,
    ))
    result = explorer.explore(truth, environment.get_start_pose())
    belief = result["belief_final"]
    known = belief >= 0

    assert result["success"]
    assert result["metadata"]["protocol"] == "unknown_static_grid_occlusion_aware"
    assert np.array_equal(belief[known], truth.data[known])
    assert any(step["observed_updates"] for step in result["steps"])

    replayed = np.full_like(belief, -1)
    flat = replayed.ravel()
    for step in result["steps"]:
        for index, value in step["observed_updates"]:
            flat[index] = value
        assert all("priority" in candidate for candidate in step["active_frontiers"])
        keys = [
            (
                tuple(np.round(candidate["target"], 6)),
                round(candidate["heading"] % 360.0, 4),
            )
            for candidate in step["generated_candidates"]
        ]
        assert len(keys) == len(set(keys))
    assert np.array_equal(replayed, belief)
