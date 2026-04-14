"""
Strategy comparison script for ablation study.

Compares different frontier selection strategies on various environments.
"""
import sys
import os
import time
import json
from typing import Dict, List

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.core.explorer import SSTGExplorer
from src.config import ExplorerConfig, FrontierSelectionStrategy
from simulation.simple_env import create_environment
from src.utils.visualization import visualize_exploration


def run_single_experiment(
    env_type: str,
    strategy: FrontierSelectionStrategy,
    run_id: int = 0,
    r_view: float = 2.0,
    d_theta: float = 45.0,
    overlap: float = 0.25,
    seed: int = 42,
    verbose: bool = False
) -> Dict:
    """
    Run a single exploration experiment with specified strategy.

    Args:
        env_type: Environment type ('empty', 'obstacles', etc.).
        strategy: Frontier selection strategy to use.
        run_id: Run identifier for multiple trials.
        r_view: View radius in meters.
        d_theta: Angular interval in degrees.
        overlap: Overlap distance in meters.
        seed: Random seed for environment generation.
        verbose: Whether to print progress.

    Returns:
        Dictionary containing experiment results and metrics.
    """
    # Create environment
    if env_type == 'empty':
        env = create_environment('empty', width=10.0, height=10.0)
    elif env_type == 'sparse_obstacles':
        env = create_environment('obstacles', width=10.0, height=10.0,
                                num_obstacles=5, seed=seed + run_id)
    elif env_type == 'dense_obstacles':
        env = create_environment('obstacles', width=10.0, height=10.0,
                                num_obstacles=15, seed=seed + run_id)
    elif env_type == 'corridor':
        env = create_environment('corridor', length=15.0, width=2.5)
    elif env_type == 'multiple_rooms':
        env = create_environment('multiple_rooms', width=15.0, height=10.0)
    elif env_type == 'apartment':
        env = create_environment('apartment', width=12.0, height=12.0)
    else:
        raise ValueError(f"Unknown environment type: {env_type}")

    occupancy_grid = env.get_occupancy_map()
    start_pose = env.get_start_pose()

    # Create explorer with specified strategy
    config = ExplorerConfig(
        r_view=r_view,
        d_theta=d_theta,
        overlap=overlap,
        frontier_strategy=strategy,
        verbose=verbose
    )

    explorer = SSTGExplorer(config=config)

    # Run exploration
    start_time = time.time()
    result = explorer.explore(occupancy_grid, start_pose, visualizer=None)
    elapsed_time = time.time() - start_time

    # Compute additional metrics
    nodes = result['nodes']
    total_distance = result['metadata']['total_distance']
    coverage_ratio = result['metadata']['coverage_ratio']

    # Compute step distances and jump statistics
    step_distances = []
    jump_count = 0
    jump_threshold = 1.5 * r_view

    for i in range(1, len(nodes)):
        prev_pos = (nodes[i-1]['position'][0], nodes[i-1]['position'][1])
        curr_pos = (nodes[i]['position'][0], nodes[i]['position'][1])
        dist = ((curr_pos[0] - prev_pos[0])**2 + (curr_pos[1] - prev_pos[1])**2)**0.5
        step_distances.append(dist)

        if dist > jump_threshold:
            jump_count += 1

    import numpy as np
    step_std = np.std(step_distances) if len(step_distances) > 0 else 0.0
    step_mean = np.mean(step_distances) if len(step_distances) > 0 else 0.0

    # Coverage efficiency
    coverage_efficiency = coverage_ratio / max(total_distance, 0.01)

    # Compile results
    experiment_result = {
        'env_type': env_type,
        'strategy': strategy.value,
        'run_id': run_id,
        'parameters': {
            'r_view': r_view,
            'd_theta': d_theta,
            'overlap': overlap,
            'seed': seed + run_id
        },
        'metrics': {
            'total_distance': total_distance,
            'num_nodes': len(nodes),
            'coverage_ratio': coverage_ratio,
            'coverage_efficiency': coverage_efficiency,
            'step_mean': step_mean,
            'step_std': step_std,
            'jump_count': jump_count,
            'total_time': elapsed_time
        },
        'success': result['success']
    }

    return experiment_result


def run_ablation_study(
    strategies: List[FrontierSelectionStrategy] = None,
    environments: List[str] = None,
    num_runs: int = 5,
    output_dir: str = 'outputs/ablation_study'
):
    """
    Run complete ablation study comparing all strategies.

    Args:
        strategies: List of strategies to compare (default: all).
        environments: List of environments to test (default: all).
        num_runs: Number of runs per strategy-environment pair.
        output_dir: Directory to save results.
    """
    # Default configurations
    if strategies is None:
        strategies = [
            FrontierSelectionStrategy.BASELINE,
            FrontierSelectionStrategy.ENHANCED_DISTANCE,
            FrontierSelectionStrategy.DUAL_FACTOR,
            FrontierSelectionStrategy.CUMULATIVE_DISTANCE,
            FrontierSelectionStrategy.CLUSTER_PRIORITY,
            FrontierSelectionStrategy.HYBRID_ADAPTIVE
        ]

    if environments is None:
        environments = [
            'empty',
            'sparse_obstacles',
            'dense_obstacles',
            'corridor',
            'multiple_rooms',
            'apartment'
        ]

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(f'{output_dir}/results', exist_ok=True)

    print("="*80)
    print("SSTG Explorer - Ablation Study: Frontier Selection Strategies")
    print("="*80)
    print(f"\nStrategies to test: {len(strategies)}")
    for s in strategies:
        print(f"  - {s.value}")
    print(f"\nEnvironments: {len(environments)}")
    for e in environments:
        print(f"  - {e}")
    print(f"\nRuns per configuration: {num_runs}")
    print(f"Total experiments: {len(strategies) * len(environments) * num_runs}")
    print("="*80)

    # Run experiments
    all_results = []
    total_experiments = len(strategies) * len(environments) * num_runs
    experiment_count = 0

    for env_type in environments:
        print(f"\n\n{'='*80}")
        print(f"Environment: {env_type.upper()}")
        print(f"{'='*80}")

        for strategy in strategies:
            print(f"\n  Strategy: {strategy.value}")
            print(f"  {'-'*76}")

            strategy_results = []

            for run_id in range(num_runs):
                experiment_count += 1
                progress = (experiment_count / total_experiments) * 100

                print(f"    Run {run_id + 1}/{num_runs}... ", end='', flush=True)

                try:
                    result = run_single_experiment(
                        env_type=env_type,
                        strategy=strategy,
                        run_id=run_id,
                        verbose=False
                    )

                    strategy_results.append(result)
                    all_results.append(result)

                    # Print metrics
                    m = result['metrics']
                    print(f"[{progress:5.1f}%] "
                          f"Dist: {m['total_distance']:6.2f}m, "
                          f"Nodes: {m['num_nodes']:3d}, "
                          f"Cov: {m['coverage_ratio']:.1%}, "
                          f"Jumps: {m['jump_count']:2d}")

                except Exception as e:
                    print(f"FAILED - {str(e)}")
                    continue

            # Compute statistics for this strategy on this environment
            if len(strategy_results) > 0:
                import numpy as np

                metrics = ['total_distance', 'num_nodes', 'coverage_ratio',
                          'coverage_efficiency', 'step_std', 'jump_count']

                print(f"\n  Summary statistics:")
                for metric in metrics:
                    values = [r['metrics'][metric] for r in strategy_results]
                    mean_val = np.mean(values)
                    std_val = np.std(values)

                    if metric in ['coverage_ratio', 'coverage_efficiency']:
                        print(f"    {metric:25s}: {mean_val:7.3f} ± {std_val:.3f}")
                    else:
                        print(f"    {metric:25s}: {mean_val:7.2f} ± {std_val:.2f}")

    print(f"\n\n{'='*80}")
    print("Ablation Study Complete!")
    print(f"{'='*80}")
    print(f"Total experiments run: {len(all_results)}")

    # Save results to JSON
    results_file = f'{output_dir}/results/ablation_study_results.json'
    with open(results_file, 'w') as f:
        json.dump({
            'metadata': {
                'num_strategies': len(strategies),
                'num_environments': len(environments),
                'num_runs': num_runs,
                'total_experiments': len(all_results),
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
            },
            'results': all_results
        }, f, indent=2)

    print(f"\nResults saved to: {results_file}")

    # Generate summary comparison
    print(f"\n{'='*80}")
    print("SUMMARY COMPARISON (mean ± std)")
    print(f"{'='*80}")

    import numpy as np

    for env_type in environments:
        print(f"\n{env_type.upper()}:")
        print(f"{'Strategy':<25} {'Distance':>12} {'Nodes':>8} {'Coverage':>10} {'Jumps':>8}")
        print(f"{'-'*80}")

        for strategy in strategies:
            strategy_env_results = [
                r for r in all_results
                if r['env_type'] == env_type and r['strategy'] == strategy.value
            ]

            if len(strategy_env_results) == 0:
                continue

            dist_values = [r['metrics']['total_distance'] for r in strategy_env_results]
            node_values = [r['metrics']['num_nodes'] for r in strategy_env_results]
            cov_values = [r['metrics']['coverage_ratio'] for r in strategy_env_results]
            jump_values = [r['metrics']['jump_count'] for r in strategy_env_results]

            dist_mean, dist_std = np.mean(dist_values), np.std(dist_values)
            node_mean, node_std = np.mean(node_values), np.std(node_values)
            cov_mean, cov_std = np.mean(cov_values), np.std(cov_values)
            jump_mean, jump_std = np.mean(jump_values), np.std(jump_values)

            print(f"{strategy.value:<25} "
                  f"{dist_mean:6.1f}±{dist_std:4.1f} "
                  f"{node_mean:4.0f}±{node_std:3.0f} "
                  f"{cov_mean:5.1%}±{cov_std:4.1%} "
                  f"{jump_mean:4.0f}±{jump_std:3.0f}")

    print(f"\n{'='*80}\n")

    return all_results


def quick_comparison(env_type: str = 'sparse_obstacles'):
    """Quick comparison of all strategies on a single environment."""
    print(f"\n{'='*80}")
    print(f"Quick Strategy Comparison - Environment: {env_type}")
    print(f"{'='*80}\n")

    strategies = [
        FrontierSelectionStrategy.BASELINE,
        FrontierSelectionStrategy.ENHANCED_DISTANCE,
        FrontierSelectionStrategy.DUAL_FACTOR,
        FrontierSelectionStrategy.CUMULATIVE_DISTANCE,
        FrontierSelectionStrategy.CLUSTER_PRIORITY,
        FrontierSelectionStrategy.HYBRID_ADAPTIVE
    ]

    results = []

    for strategy in strategies:
        print(f"Testing {strategy.value}... ", end='', flush=True)

        result = run_single_experiment(
            env_type=env_type,
            strategy=strategy,
            run_id=0,
            verbose=False
        )

        results.append(result)

        m = result['metrics']
        print(f"Dist: {m['total_distance']:6.2f}m, "
              f"Nodes: {m['num_nodes']:3d}, "
              f"Cov: {m['coverage_ratio']:.1%}, "
              f"Jumps: {m['jump_count']:2d}")

    print(f"\n{'='*80}")
    print("Comparison Summary:")
    print(f"{'='*80}")
    print(f"{'Strategy':<25} {'Distance':>10} {'Nodes':>8} {'Coverage':>10} {'Jumps':>8}")
    print(f"{'-'*80}")

    for result in results:
        m = result['metrics']
        print(f"{result['strategy']:<25} "
              f"{m['total_distance']:8.2f}m "
              f"{m['num_nodes']:6d} "
              f"{m['coverage_ratio']:8.1%} "
              f"{m['jump_count']:6d}")

    print(f"{'='*80}\n")

    return results


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Strategy comparison for ablation study')
    parser.add_argument('--mode', type=str, default='quick',
                       choices=['quick', 'full'],
                       help='Comparison mode: quick or full ablation study')
    parser.add_argument('--env', type=str, default='sparse_obstacles',
                       help='Environment for quick mode')
    parser.add_argument('--runs', type=int, default=5,
                       help='Number of runs per configuration (full mode)')

    args = parser.parse_args()

    if args.mode == 'quick':
        quick_comparison(env_type=args.env)
    else:
        run_ablation_study(num_runs=args.runs)
