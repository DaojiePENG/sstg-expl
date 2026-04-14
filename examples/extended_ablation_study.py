"""
Extended ablation study: Frontier strategies + A* + Adaptive sampling.

Tests all combinations of:
- 6 frontier selection strategies
- A* path planning (on/off)
- Adaptive sampling (on/off)
"""
import sys
import os
import time
import json
from typing import Dict, List
import itertools

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.core.explorer import SSTGExplorer
from src.config import ExplorerConfig, FrontierSelectionStrategy
from simulation.simple_env import create_environment


def run_extended_experiment(
    env_type: str,
    strategy: FrontierSelectionStrategy,
    use_astar: bool,
    use_adaptive: bool,
    run_id: int = 0,
    seed: int = 42,
    verbose: bool = False
) -> Dict:
    """
    Run exploration with specific configuration.

    Args:
        env_type: Environment type.
        strategy: Frontier selection strategy.
        use_astar: Whether to use A* path planning.
        use_adaptive: Whether to use adaptive sampling.
        run_id: Run identifier.
        seed: Random seed.
        verbose: Whether to print progress.

    Returns:
        Results dictionary.
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
    else:
        raise ValueError(f"Unknown environment type: {env_type}")

    occupancy_grid = env.get_occupancy_map()
    start_pose = env.get_start_pose()

    # Create configuration
    config = ExplorerConfig(
        r_view=2.0,
        d_theta=45.0,
        overlap=0.25,
        frontier_strategy=strategy,
        use_astar=use_astar,
        use_adaptive_sampling=use_adaptive,
        verbose=verbose
    )

    # Create explorer
    explorer = SSTGExplorer(config=config)

    # Run exploration
    start_time = time.time()
    result = explorer.explore(occupancy_grid, start_pose, visualizer=None)
    elapsed_time = time.time() - start_time

    # Compute metrics
    nodes = result['nodes']
    total_distance = result['metadata']['total_distance']
    coverage_ratio = result['metadata']['coverage_ratio']

    # Compute step distances and jumps
    step_distances = []
    jump_count = 0
    jump_threshold = 1.5 * 2.0  # 1.5 * r_view

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

    # Compile results (convert numpy types to Python native types for JSON)
    experiment_result = {
        'env_type': env_type,
        'strategy': strategy.value,
        'use_astar': bool(use_astar),
        'use_adaptive': bool(use_adaptive),
        'run_id': int(run_id),
        'parameters': {
            'r_view': 2.0,
            'd_theta': 45.0,
            'overlap': 0.25,
            'seed': seed + run_id
        },
        'metrics': {
            'total_distance': float(total_distance),
            'num_nodes': int(len(nodes)),
            'coverage_ratio': float(coverage_ratio),
            'coverage_efficiency': float(coverage_efficiency),
            'step_mean': float(step_mean),
            'step_std': float(step_std),
            'jump_count': int(jump_count),
            'total_time': float(elapsed_time)
        },
        'success': bool(result['success'])
    }

    return experiment_result


def run_extended_ablation_study(
    environments: List[str] = None,
    strategies: List[FrontierSelectionStrategy] = None,
    test_astar: bool = True,
    test_adaptive: bool = True,
    num_runs: int = 3,
    output_dir: str = 'outputs/extended_ablation'
):
    """
    Run extended ablation study.

    Args:
        environments: List of environments to test.
        strategies: List of strategies to test.
        test_astar: Whether to test A* variants.
        test_adaptive: Whether to test adaptive sampling variants.
        num_runs: Number of runs per configuration.
        output_dir: Output directory.
    """
    # Default configurations
    if environments is None:
        environments = ['empty', 'sparse_obstacles', 'corridor']

    if strategies is None:
        strategies = [
            FrontierSelectionStrategy.BASELINE,
            FrontierSelectionStrategy.ENHANCED_DISTANCE,
            FrontierSelectionStrategy.HYBRID_ADAPTIVE
        ]

    # Configuration variants
    astar_options = [False, True] if test_astar else [False]
    adaptive_options = [False, True] if test_adaptive else [False]

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(f'{output_dir}/results', exist_ok=True)

    print("="*80)
    print("SSTG Explorer - Extended Ablation Study")
    print("="*80)
    print(f"\nEnvironments: {len(environments)}")
    for e in environments:
        print(f"  - {e}")
    print(f"\nStrategies: {len(strategies)}")
    for s in strategies:
        print(f"  - {s.value}")
    print(f"\nFeature variants:")
    print(f"  A* path planning: {astar_options}")
    print(f"  Adaptive sampling: {adaptive_options}")
    print(f"\nRuns per configuration: {num_runs}")

    # Calculate total experiments
    total_configs = (len(environments) * len(strategies) *
                    len(astar_options) * len(adaptive_options))
    total_experiments = total_configs * num_runs

    print(f"\nTotal configurations: {total_configs}")
    print(f"Total experiments: {total_experiments}")
    print("="*80)

    # Run experiments
    all_results = []
    experiment_count = 0

    for env_type in environments:
        print(f"\n\n{'='*80}")
        print(f"Environment: {env_type.upper()}")
        print(f"{'='*80}")

        for strategy in strategies:
            for use_astar in astar_options:
                for use_adaptive in adaptive_options:
                    # Configuration name
                    config_name = f"{strategy.value}"
                    if use_astar:
                        config_name += " + A*"
                    if use_adaptive:
                        config_name += " + Adaptive"

                    print(f"\n  Config: {config_name}")
                    print(f"  {'-'*76}")

                    config_results = []

                    for run_id in range(num_runs):
                        experiment_count += 1
                        progress = (experiment_count / total_experiments) * 100

                        print(f"    Run {run_id + 1}/{num_runs}... ", end='', flush=True)

                        try:
                            result = run_extended_experiment(
                                env_type=env_type,
                                strategy=strategy,
                                use_astar=use_astar,
                                use_adaptive=use_adaptive,
                                run_id=run_id,
                                verbose=False
                            )

                            config_results.append(result)
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

                    # Summary statistics for this configuration
                    if len(config_results) > 0:
                        import numpy as np
                        metrics = ['total_distance', 'num_nodes', 'coverage_ratio',
                                  'coverage_efficiency', 'jump_count']

                        print(f"\n  Summary:")
                        for metric in metrics:
                            values = [r['metrics'][metric] for r in config_results]
                            mean_val = np.mean(values)
                            std_val = np.std(values)
                            if metric in ['coverage_ratio', 'coverage_efficiency']:
                                print(f"    {metric:25s}: {mean_val:7.3f} ± {std_val:.3f}")
                            else:
                                print(f"    {metric:25s}: {mean_val:7.2f} ± {std_val:.2f}")

    print(f"\n\n{'='*80}")
    print("Extended Ablation Study Complete!")
    print(f"{'='*80}")
    print(f"Total experiments run: {len(all_results)}")

    # Save results
    results_file = f'{output_dir}/results/extended_ablation_results.json'
    with open(results_file, 'w') as f:
        json.dump({
            'metadata': {
                'num_environments': len(environments),
                'num_strategies': len(strategies),
                'astar_variants': len(astar_options),
                'adaptive_variants': len(adaptive_options),
                'num_runs': num_runs,
                'total_experiments': len(all_results),
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
            },
            'results': all_results
        }, f, indent=2)

    print(f"\nResults saved to: {results_file}")

    # Generate summary comparison
    print(f"\n{'='*80}")
    print("SUMMARY COMPARISON")
    print(f"{'='*80}")

    import numpy as np

    for env_type in environments:
        print(f"\n{env_type.upper()}:")
        print(f"{'Configuration':<40} {'Distance':>12} {'Nodes':>8} {'Coverage':>10} {'Jumps':>8}")
        print(f"{'-'*80}")

        for strategy in strategies:
            for use_astar in astar_options:
                for use_adaptive in adaptive_options:
                    # Filter results for this configuration
                    config_results = [
                        r for r in all_results
                        if (r['env_type'] == env_type and
                            r['strategy'] == strategy.value and
                            r['use_astar'] == use_astar and
                            r['use_adaptive'] == use_adaptive)
                    ]

                    if len(config_results) == 0:
                        continue

                    # Configuration name
                    config_name = strategy.value[:20]
                    if use_astar:
                        config_name += "+A*"
                    if use_adaptive:
                        config_name += "+Adp"

                    # Compute statistics
                    dist_values = [r['metrics']['total_distance'] for r in config_results]
                    node_values = [r['metrics']['num_nodes'] for r in config_results]
                    cov_values = [r['metrics']['coverage_ratio'] for r in config_results]
                    jump_values = [r['metrics']['jump_count'] for r in config_results]

                    dist_mean = np.mean(dist_values)
                    node_mean = np.mean(node_values)
                    cov_mean = np.mean(cov_values)
                    jump_mean = np.mean(jump_values)

                    print(f"{config_name:<40} "
                          f"{dist_mean:8.1f}m "
                          f"{node_mean:6.0f} "
                          f"{cov_mean:8.1%} "
                          f"{jump_mean:6.0f}")

    print(f"\n{'='*80}\n")

    return all_results


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Extended ablation study')
    parser.add_argument('--runs', type=int, default=3,
                       help='Number of runs per configuration')
    parser.add_argument('--quick', action='store_true',
                       help='Quick test (fewer configurations)')

    args = parser.parse_args()

    if args.quick:
        # Quick test: 2 strategies, 2 environments
        strategies = [
            FrontierSelectionStrategy.BASELINE,
            FrontierSelectionStrategy.ENHANCED_DISTANCE
        ]
        environments = ['empty', 'corridor']
        num_runs = 2
    else:
        # Full test
        strategies = [
            FrontierSelectionStrategy.BASELINE,
            FrontierSelectionStrategy.ENHANCED_DISTANCE,
            FrontierSelectionStrategy.DUAL_FACTOR,
            FrontierSelectionStrategy.HYBRID_ADAPTIVE
        ]
        environments = ['empty', 'sparse_obstacles', 'corridor']
        num_runs = args.runs

    run_extended_ablation_study(
        environments=environments,
        strategies=strategies,
        test_astar=True,
        test_adaptive=True,
        num_runs=num_runs
    )
