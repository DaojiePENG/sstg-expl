"""Regression tests for the installed package structure and final defaults."""

from sstg_explorer import ExplorerConfig, FrontierSelectionStrategy, SSTGExplorer
from sstg_explorer.benchmark import BenchmarkRunner
from sstg_explorer.environments import create_environment
from sstg_explorer.planning.astar import AStarPlanner
from scipy.ndimage import label
import numpy as np


def test_public_default_is_final_sstg_explorer():
    explorer = SSTGExplorer(config=ExplorerConfig(verbose=False))
    assert explorer.name == "SSTG-Explorer"
    assert explorer.config.d_theta == 30.0
    assert explorer.config.frontier_strategy is FrontierSelectionStrategy.ENHANCED_DISTANCE
    assert explorer.config.use_astar is True
    assert explorer.config.use_adaptive_sampling is False


def test_factory_and_environment_public_apis(tmp_path):
    runner = BenchmarkRunner(output_dir=str(tmp_path), num_runs=1, seed=42)
    explorer = runner.create_algorithm("sstg_explorer", verbose=False)
    environment = create_environment("empty", width=4.0, height=4.0)

    assert explorer.name == "SSTG-Explorer"
    assert environment.get_occupancy_map().data.ndim == 2


def test_decision_trace_contains_all_candidate_states():
    environment = create_environment("empty", width=5.0, height=5.0)
    explorer = SSTGExplorer(config=ExplorerConfig(target_coverage=0.50, verbose=False))
    result = explorer.explore(environment.get_occupancy_map(), environment.get_start_pose())

    initial = result["steps"][0]
    assert initial["event"] == "initialization"
    assert initial["generated_candidates"]
    assert {"explored_nodes", "new_frontiers", "active_frontiers"} <= initial.keys()
    assert all("status" in candidate for candidate in initial["generated_candidates"])
    accepted = next(step for step in result["steps"] if step["event"] == "node_accepted")
    assert accepted["path"]
    assert accepted["executed_paths"]
    assert accepted["executed_paths"][-1] == accepted["path"]


def test_hard_environments_are_valid_after_robot_inflation():
    dense = create_environment("dense_obstacles", seed=43)
    dense_grid = dense.get_occupancy_map()
    dense_planner = AStarPlanner(dense_grid, robot_radius=0.3, safety_margin=0.2)
    start = dense_grid.world_to_grid(*dense.get_start_pose()[:2])
    assert dense_planner.planning_grid.get_free_space_mask()[start]

    narrow = create_environment("narrow_passages")
    narrow_grid = narrow.get_occupancy_map()
    safe = AStarPlanner(
        narrow_grid, robot_radius=0.3, safety_margin=0.2
    ).planning_grid.get_free_space_mask()
    _, components = label(safe, structure=np.ones((3, 3), dtype=int))
    assert components == 1


def test_safety_metrics_cover_viewpoints_paths_and_boundaries():
    environment = create_environment("empty", width=5.0, height=5.0)
    grid = environment.get_occupancy_map()
    nodes = [(2.5, 2.5), (3.0, 2.5)]
    path = [[(2.5, 2.5), (3.0, 2.5)]]
    samples = BenchmarkRunner.sample_execution_paths(path, grid.resolution)
    metrics = BenchmarkRunner.compute_spatial_metrics(
        nodes, grid, path_positions=samples, required_clearance=0.5
    )
    assert metrics["avg_obstacle_distance"] > 0.5
    assert metrics["avg_boundary_distance"] > 0.5
    assert metrics["path_safe_fraction"] == 1.0
