"""Regression tests for the unknown-map range sensor."""
import numpy as np

from sstg_explorer.map import OccupancyGrid
from sstg_explorer.sensing import RaycastSensor, SensorConfig
from sstg_explorer.unknown import UnknownExplorerConfig, UnknownMapExplorer
from sstg_explorer.environments import create_environment
from sstg_explorer.core.coverage_analyzer import CoverageAnalyzer


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
        coverage_objective="sensor",
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


def test_topological_coverage_matches_known_map_disk_proxy():
    environment = create_environment("empty", width=8.0, height=8.0)
    truth = environment.get_occupancy_map()
    positions = [(2.0, 2.0), (5.0, 5.0)]
    expected = CoverageAnalyzer(truth).compute_coverage_map(positions, 2.0)
    actual = UnknownMapExplorer.topological_coverage_map(truth, positions, 2.0)

    assert np.array_equal(actual, expected)


def test_final_unknown_sstg_uses_selected_spacing_utility():
    config = UnknownExplorerConfig()
    assert config.coverage_objective == "joint"
    assert config.spacing_weight == 0.30


def test_joint_objective_continues_after_long_range_sensor_saturates():
    environment = create_environment("empty", width=8.0, height=8.0)
    truth = environment.get_occupancy_map()
    explorer = UnknownMapExplorer(UnknownExplorerConfig(
        strategy="sstg",
        sensor=SensorConfig(360.0, 16.0, angular_resolution_deg=0.5),
        target_coverage=0.80,
        coverage_objective="joint",
        topological_radius=2.0,
        target_topological_coverage=0.80,
        max_decisions=30,
        seed=42,
    ))
    result = explorer.explore(truth, environment.get_start_pose())
    metadata = result["metadata"]

    assert result["success"]
    assert metadata["sensor_coverage_ratio"] >= 0.80
    assert metadata["topological_coverage_ratio"] >= 0.80
    assert metadata["coverage_ratio"] == metadata["topological_coverage_ratio"]
    assert len(result["nodes"]) > 1
    assert any(
        candidate.get("kind") == "coverage_gap"
        for step in result["steps"]
        for candidate in step.get("generated_candidates", [])
    )
    assert all(
        "predicted_topological_gain" in candidate
        for step in result["steps"]
        for candidate in step.get("generated_candidates", [])
    )
    executed = [
        tuple(step["selected_frontier"]["execution_key"])
        for step in result["steps"]
        if (step.get("selected_frontier") or {}).get("status") == "selected"
    ]
    assert len(executed) == len(set(executed))


def test_directional_actions_merge_into_spatial_topological_nodes():
    environment = create_environment("empty")
    explorer = UnknownMapExplorer(UnknownExplorerConfig(
        strategy="sstg",
        sensor=SensorConfig(90.0, 12.0, angular_resolution_deg=0.5),
        coverage_objective="joint",
        topological_radius=2.0,
        max_decisions=80,
        seed=42,
    ))
    result = explorer.explore(
        environment.get_occupancy_map(), environment.get_start_pose()
    )
    metadata = result["metadata"]
    nodes = {node["id"]: node for node in result["nodes"]}
    views_by_node = {}
    for view in result["oriented_views"]:
        node_id = view["topological_node_id"]
        assert node_id in nodes
        distance = np.linalg.norm(
            np.asarray(view["position"]) - np.asarray(nodes[node_id]["position"])
        )
        assert distance <= explorer.config.topological_merge_distance + 1e-9
        views_by_node.setdefault(node_id, []).append(view)

    assert result["success"]
    assert metadata["topological_node_count"] == len(result["nodes"])
    assert metadata["oriented_view_count"] == len(result["oriented_views"])
    assert len(result["oriented_views"]) > len(result["nodes"])
    assert metadata["in_place_rotations"] > 0
    assert any(len(views) > 1 for views in views_by_node.values())
