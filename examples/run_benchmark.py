"""
Example script to run benchmark experiments.

This script demonstrates how to use the benchmark framework to compare
SSTG Explorer with baseline exploration algorithms.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from simulation.benchmark import BenchmarkRunner
from simulation.simple_env import create_environment


def main():
    """Run benchmark experiments."""
    # Create benchmark runner
    runner = BenchmarkRunner(
        output_dir='outputs/benchmarks',
        num_runs=5,
        seed=42
    )

    # Define algorithms to test
    algorithms = [
        'uniform_grid',
        'rrt',
        'frontier',
        'nbv',
        'sstg',            # SSTG with baseline strategy
        'sstg_enhanced',   # SSTG with enhanced distance
        'sstg_optimal',    # SSTG with A* + adaptive sampling
    ]

    # Define test environments
    environments = [
        create_environment('empty', width=10.0, height=10.0),
        create_environment('obstacles', width=10.0, height=10.0,
                          num_obstacles=5, seed=42),
        create_environment('corridor', length=15.0, width=2.5),
    ]

    # Algorithm-specific parameters (optional)
    algorithm_kwargs = {
        'uniform_grid': {
            'grid_spacing': 2.0,
            'visit_order': 'nearest'
        },
        'rrt': {
            'max_iterations': 500,
            'step_size': 1.0
        },
        'frontier': {
            'target_coverage': 0.95,
            'max_iterations': 500
        },
        'nbv': {
            'n_candidates': 50,
            'target_coverage': 0.95,
            'max_iterations': 500
        },
        'sstg': {
            'd_theta': 45.0,
            'target_coverage': 0.95
        },
        'sstg_enhanced': {
            'd_theta': 45.0,
            'target_coverage': 0.95,
            'beta': 1.0
        },
        'sstg_optimal': {
            'd_theta': 45.0,
            'target_coverage': 0.95,
            'beta': 1.0
        }
    }

    # Run benchmark
    runner.run_benchmark(
        algorithms=algorithms,
        environments=environments,
        algorithm_kwargs=algorithm_kwargs
    )

    # Generate analysis report
    from simulation.benchmark_analysis import BenchmarkAnalyzer

    # Get the results file path
    results_files = sorted([
        f'outputs/benchmarks/results/{f}'
        for f in os.listdir('outputs/benchmarks/results')
        if f.startswith('benchmark_') and f.endswith('.json')
    ])

    if results_files:
        latest_results = results_files[-1]
        print(f"\nAnalyzing results from: {latest_results}")

        analyzer = BenchmarkAnalyzer(latest_results)
        analyzer.generate_full_report()

        print("\nBenchmark complete!")
        print("Check 'outputs/benchmarks/analysis/' for visualizations")
    else:
        print("\nNo results files found")


if __name__ == '__main__':
    main()
