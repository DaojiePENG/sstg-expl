"""
Benchmark runner for comparing exploration algorithms.

This module provides a framework for running multiple exploration algorithms
on standardized environments and collecting performance metrics for comparison.
"""

import time
import json
import os
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial.distance import pdist, squareform

from src.baseline import (
    BaseExplorer,
    UniformGridExplorer,
    RRTExplorer,
    FrontierExplorer,
    NextBestViewExplorer
)
from src.core.explorer import SSTGExplorer
from src.config import ExplorerConfig, FrontierSelectionStrategy


@dataclass
class BenchmarkResult:
    """
    Results from a single benchmark run.

    Attributes:
        algorithm: Algorithm name
        environment: Environment name
        run_id: Run identifier (for multiple runs)
        total_distance: Total travel distance (m)
        num_nodes: Number of sampled nodes
        coverage_ratio: Achieved coverage ratio [0,1]
        coverage_efficiency: Coverage per unit distance
        computation_time: Algorithm runtime (seconds)
        success: Whether exploration completed successfully
        additional_metrics: Algorithm-specific metrics
    """
    algorithm: str
    environment: str
    run_id: int
    total_distance: float
    num_nodes: int
    coverage_ratio: float
    coverage_efficiency: float
    computation_time: float
    success: bool
    additional_metrics: Dict = None

    def to_dict(self) -> Dict:
        """Convert to dictionary with proper type conversion for JSON."""
        data = asdict(self)

        # Convert numpy types to Python native types
        def convert_numpy_types(obj):
            """Recursively convert numpy types to Python types."""
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, (np.integer, np.int64, np.int32)):
                return int(obj)
            elif isinstance(obj, (np.floating, np.float64, np.float32)):
                return float(obj)
            elif isinstance(obj, (np.bool_, bool)):
                return bool(obj)
            elif isinstance(obj, dict):
                return {k: convert_numpy_types(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [convert_numpy_types(item) for item in obj]
            return obj

        return convert_numpy_types(data)


class BenchmarkRunner:
    """
    Benchmark runner for exploration algorithms.

    Manages execution of multiple algorithms on multiple environments,
    collecting standardized metrics for fair comparison.

    Args:
        output_dir: Directory to save results
        num_runs: Number of runs per algorithm-environment pair
        seed: Base random seed for reproducibility
    """

    def __init__(
        self,
        output_dir: str = 'outputs/benchmarks',
        num_runs: int = 5,
        seed: int = 42
    ):
        """Initialize benchmark runner."""
        self.output_dir = output_dir
        self.num_runs = num_runs
        self.seed = seed

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(f'{output_dir}/results', exist_ok=True)

        self.results = []

    @staticmethod
    def compute_spatial_metrics(
        node_positions: List,
        occupancy_grid_obj,
        r_view: float = 2.0
    ) -> Dict[str, float]:
        """
        Compute spatial quality metrics for exploration node placement.

        Metrics:
        1. avg_obstacle_distance: Mean distance from each node to nearest obstacle
        2. min_obstacle_distance: Minimum node-to-obstacle distance
        3. mean_nn_distance: Mean nearest-neighbor distance between nodes
        4. nn_distance_std: Std of nearest-neighbor distances
        5. dispersion_uniformity: Uniformity of node spacing (1 - CV)
           Higher = more uniform. 1.0 = perfectly uniform spacing.

        Args:
            node_positions: List of (x, y) tuples
            occupancy_grid_obj: OccupancyGrid object with grid data
            r_view: Sensor view radius (meters)

        Returns:
            Dict of spatial quality metrics
        """
        default_metrics = {
            'avg_obstacle_distance': 0.0,
            'min_obstacle_distance': 0.0,
            'mean_nn_distance': 0.0,
            'nn_distance_std': 0.0,
            'dispersion_uniformity': 0.0,
        }

        if len(node_positions) < 2:
            return default_metrics

        # --- 1. Distance to nearest obstacle ---
        # Get occupancy grid data
        if hasattr(occupancy_grid_obj, 'data'):
            grid_data = occupancy_grid_obj.data
            resolution = occupancy_grid_obj.resolution
            origin = occupancy_grid_obj.origin
        else:
            grid_data = occupancy_grid_obj
            resolution = 0.05
            origin = (0.0, 0.0)

        # Build obstacle mask and compute distance transform
        obstacle_mask = grid_data >= 50  # Occupied cells
        # distance_transform_edt gives distance in cells from each cell to nearest obstacle
        dist_field = distance_transform_edt(~obstacle_mask) * resolution  # Convert to meters

        # Query distance for each node
        obstacle_distances = []
        for pos in node_positions:
            if isinstance(pos, dict):
                pos = pos.get('position', pos)
            x, y = pos[0], pos[1]
            # World to grid
            j = int((x - origin[0]) / resolution)
            i = int((y - origin[1]) / resolution)
            # Clamp to grid bounds
            i = max(0, min(i, grid_data.shape[0] - 1))
            j = max(0, min(j, grid_data.shape[1] - 1))
            obstacle_distances.append(dist_field[i, j])

        obstacle_distances = np.array(obstacle_distances)

        # --- 2. Node-to-node nearest neighbor distances ---
        positions_array = []
        for pos in node_positions:
            if isinstance(pos, dict):
                pos = pos.get('position', pos)
            positions_array.append([pos[0], pos[1]])
        positions_array = np.array(positions_array)

        # Pairwise distance matrix
        if len(positions_array) >= 2:
            dist_matrix = squareform(pdist(positions_array))
            # Set diagonal to inf to find nearest neighbor
            np.fill_diagonal(dist_matrix, np.inf)
            nn_distances = np.min(dist_matrix, axis=1)
        else:
            nn_distances = np.array([0.0])

        # --- 3. Compute metrics ---
        avg_obstacle_dist = float(np.mean(obstacle_distances))
        min_obstacle_dist = float(np.min(obstacle_distances))

        mean_nn = float(np.mean(nn_distances))
        std_nn = float(np.std(nn_distances))

        # Dispersion uniformity: 1 - coefficient of variation
        # CV = std/mean; uniformity = 1 - CV (clamped to [0, 1])
        # Higher = more uniform spacing between nodes
        if mean_nn > 0:
            cv = std_nn / mean_nn
            dispersion_uniformity = max(0.0, min(1.0, 1.0 - cv))
        else:
            dispersion_uniformity = 0.0

        return {
            'avg_obstacle_distance': avg_obstacle_dist,
            'min_obstacle_distance': min_obstacle_dist,
            'mean_nn_distance': mean_nn,
            'nn_distance_std': std_nn,
            'dispersion_uniformity': dispersion_uniformity,
        }

    def create_algorithm(self, algorithm_name: str, **kwargs) -> BaseExplorer:
        """
        Create algorithm instance by name.

        Args:
            algorithm_name: Name of algorithm
            **kwargs: Algorithm-specific parameters

        Returns:
            Algorithm instance

        Raises:
            ValueError: If algorithm name not recognized
        """
        if algorithm_name == 'uniform_grid':
            return UniformGridExplorer(**kwargs)
        elif algorithm_name == 'rrt':
            return RRTExplorer(**kwargs)
        elif algorithm_name == 'frontier':
            return FrontierExplorer(**kwargs)
        elif algorithm_name == 'nbv':
            return NextBestViewExplorer(**kwargs)
        elif algorithm_name == 'sstg':
            # SSTG with baseline strategy
            config = ExplorerConfig(
                frontier_strategy=FrontierSelectionStrategy.BASELINE,
                **kwargs
            )
            explorer = SSTGExplorer(config=config)
            explorer.name = 'SSTG'
            return explorer
        elif algorithm_name == 'sstg_enhanced':
            # SSTG with enhanced distance strategy
            config = ExplorerConfig(
                frontier_strategy=FrontierSelectionStrategy.ENHANCED_DISTANCE,
                **kwargs
            )
            explorer = SSTGExplorer(config=config)
            explorer.name = 'SSTG (Enhanced)'
            return explorer
        elif algorithm_name == 'sstg_optimal':
            # SSTG with best configuration (from ablation study)
            config = ExplorerConfig(
                frontier_strategy=FrontierSelectionStrategy.ENHANCED_DISTANCE,
                use_astar=True,
                use_adaptive_sampling=True,
                **kwargs
            )
            explorer = SSTGExplorer(config=config)
            explorer.name = 'SSTG (Optimal)'
            return explorer
        else:
            raise ValueError(f"Unknown algorithm: {algorithm_name}")

    def run_single_experiment(
        self,
        algorithm: BaseExplorer,
        env,  # Environment object from simple_env
        run_id: int
    ) -> BenchmarkResult:
        """
        Run a single experiment.

        Args:
            algorithm: Algorithm instance
            env: Simulation environment
            run_id: Run identifier

        Returns:
            Benchmark result
        """
        # Reset algorithm
        if hasattr(algorithm, 'reset'):
            algorithm.reset()

        # Get environment data
        occupancy_grid_obj = env.get_occupancy_map()
        start_pose = env.get_start_pose()

        # SSTG Explorer needs OccupancyGrid object, others need numpy array
        if 'sstg' in algorithm.name.lower():
            occupancy_grid = occupancy_grid_obj
        else:
            # Convert to numpy array for baseline algorithms
            if hasattr(occupancy_grid_obj, 'data'):
                occupancy_grid = occupancy_grid_obj.data
            else:
                occupancy_grid = occupancy_grid_obj

        # Run exploration
        start_time = time.time()
        result = algorithm.explore(occupancy_grid, start_pose, visualizer=None)
        elapsed_time = time.time() - start_time

        # Extract metrics
        metadata = result.get('metadata', {})
        total_distance = metadata.get('total_distance', 0.0)
        coverage_ratio = metadata.get('coverage_ratio', 0.0)
        num_nodes = len(result.get('nodes', []))

        # Compute coverage efficiency
        coverage_efficiency = coverage_ratio / max(total_distance, 0.01)

        # Compute spatial quality metrics (obstacle distance + node dispersion)
        node_positions = [node['position'] for node in result.get('nodes', [])]
        spatial_metrics = self.compute_spatial_metrics(
            node_positions, occupancy_grid_obj, r_view=2.0
        )

        # Create benchmark result
        benchmark_result = BenchmarkResult(
            algorithm=algorithm.name,
            environment=env.name,
            run_id=run_id,
            total_distance=total_distance,
            num_nodes=num_nodes,
            coverage_ratio=coverage_ratio,
            coverage_efficiency=coverage_efficiency,
            computation_time=elapsed_time,
            success=result.get('success', False),
            additional_metrics={
                **{k: v for k, v in metadata.items()
                   if k not in ['total_distance', 'coverage_ratio', 'num_nodes',
                               'computation_time', 'success']},
                **spatial_metrics,
            }
        )

        return benchmark_result

    def run_benchmark(
        self,
        algorithms: List[str],
        environments: List,  # List of Environment objects
        algorithm_kwargs: Optional[Dict[str, Dict]] = None
    ):
        """
        Run full benchmark suite.

        Args:
            algorithms: List of algorithm names
            environments: List of environments
            algorithm_kwargs: Optional dict of algorithm-specific parameters
        """
        algorithm_kwargs = algorithm_kwargs or {}

        total_experiments = len(algorithms) * len(environments) * self.num_runs
        experiment_count = 0

        print("=" * 80)
        print("SSTG Explorer - Benchmark Suite")
        print("=" * 80)
        print(f"\nAlgorithms: {len(algorithms)}")
        for alg in algorithms:
            print(f"  - {alg}")
        print(f"\nEnvironments: {len(environments)}")
        for env in environments:
            print(f"  - {env.name}")
        print(f"\nRuns per configuration: {self.num_runs}")
        print(f"Total experiments: {total_experiments}")
        print("=" * 80)

        for env in environments:
            print(f"\n\n{'='*80}")
            print(f"Environment: {env.name.upper()}")
            print(f"{'='*80}")

            for alg_name in algorithms:
                print(f"\n  Algorithm: {alg_name}")
                print(f"  {'-'*76}")

                alg_results = []

                for run_id in range(self.num_runs):
                    experiment_count += 1
                    progress = (experiment_count / total_experiments) * 100

                    print(f"    Run {run_id + 1}/{self.num_runs}... ", end='', flush=True)

                    try:
                        # Create algorithm instance
                        kwargs = algorithm_kwargs.get(alg_name, {})
                        kwargs['r_view'] = 2.0  # Standard view radius
                        algorithm = self.create_algorithm(alg_name, **kwargs)

                        # Run experiment
                        result = self.run_single_experiment(algorithm, env, run_id)
                        alg_results.append(result)
                        self.results.append(result)

                        # Print result
                        print(f"[{progress:5.1f}%] "
                              f"Dist: {result.total_distance:6.2f}m, "
                              f"Nodes: {result.num_nodes:3d}, "
                              f"Cov: {result.coverage_ratio:.1%}, "
                              f"Time: {result.computation_time:5.2f}s")

                    except Exception as e:
                        print(f"FAILED - {str(e)}")
                        continue

                # Print summary for this algorithm
                if len(alg_results) > 0:
                    self._print_summary(alg_results)

        print(f"\n\n{'='*80}")
        print("Benchmark Complete!")
        print(f"{'='*80}")
        print(f"Total experiments: {len(self.results)}")

        # Save results
        self.save_results()

    def _print_summary(self, results: List[BenchmarkResult]):
        """Print summary statistics for a set of results."""
        if len(results) == 0:
            return

        distances = [r.total_distance for r in results]
        nodes = [r.num_nodes for r in results]
        coverages = [r.coverage_ratio for r in results]
        efficiencies = [r.coverage_efficiency for r in results]
        times = [r.computation_time for r in results]

        # Spatial quality metrics
        obs_dists = [r.additional_metrics.get('avg_obstacle_distance', 0) for r in results if r.additional_metrics]
        nn_dists = [r.additional_metrics.get('mean_nn_distance', 0) for r in results if r.additional_metrics]
        uniformities = [r.additional_metrics.get('dispersion_uniformity', 0) for r in results if r.additional_metrics]

        print(f"\n  Summary (n={len(results)}):")
        print(f"    Distance:    {np.mean(distances):7.2f} ± {np.std(distances):5.2f} m")
        print(f"    Nodes:       {np.mean(nodes):7.1f} ± {np.std(nodes):5.1f}")
        print(f"    Coverage:    {np.mean(coverages):7.1%} ± {np.std(coverages):.1%}")
        print(f"    Efficiency:  {np.mean(efficiencies):7.4f} ± {np.std(efficiencies):.4f}")
        print(f"    Time:        {np.mean(times):7.2f} ± {np.std(times):5.2f} s")
        if obs_dists:
            print(f"    Obs Dist:    {np.mean(obs_dists):7.3f} ± {np.std(obs_dists):5.3f} m")
        if nn_dists:
            print(f"    NN Dist:     {np.mean(nn_dists):7.3f} ± {np.std(nn_dists):5.3f} m")
        if uniformities:
            print(f"    Uniformity:  {np.mean(uniformities):7.3f} ± {np.std(uniformities):5.3f}")

    def save_results(self):
        """Save benchmark results to JSON file."""
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = f'{self.output_dir}/results/benchmark_{timestamp}.json'

        results_dict = {
            'metadata': {
                'num_experiments': len(self.results),
                'num_runs': self.num_runs,
                'seed': self.seed,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
            },
            'results': [r.to_dict() for r in self.results]
        }

        with open(filename, 'w') as f:
            json.dump(results_dict, f, indent=2)

        print(f"\nResults saved to: {filename}")

    def load_results(self, filename: str):
        """Load benchmark results from JSON file."""
        with open(filename, 'r') as f:
            data = json.load(f)

        self.results = [
            BenchmarkResult(**r) for r in data['results']
        ]

        print(f"Loaded {len(self.results)} results from {filename}")
