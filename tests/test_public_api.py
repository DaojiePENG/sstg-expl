"""Regression tests for the installed package structure and final defaults."""

from sstg_explorer import ExplorerConfig, FrontierSelectionStrategy, SSTGExplorer
from sstg_explorer.benchmark import BenchmarkRunner
from sstg_explorer.environments import create_environment


def test_public_default_is_final_sstg_explorer():
    explorer = SSTGExplorer(config=ExplorerConfig(verbose=False))
    assert explorer.name == "SSTG-Explorer"
    assert explorer.config.d_theta == 30.0
    assert explorer.config.frontier_strategy is FrontierSelectionStrategy.ENHANCED_DISTANCE
    assert explorer.config.use_astar is True
    assert explorer.config.use_adaptive_sampling is True


def test_factory_and_environment_public_apis(tmp_path):
    runner = BenchmarkRunner(output_dir=str(tmp_path), num_runs=1, seed=42)
    explorer = runner.create_algorithm("sstg_explorer", verbose=False)
    environment = create_environment("empty", width=4.0, height=4.0)

    assert explorer.name == "SSTG-Explorer"
    assert environment.get_occupancy_map().data.ndim == 2
