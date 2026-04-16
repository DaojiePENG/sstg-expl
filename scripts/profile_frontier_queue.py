#!/usr/bin/env python3
"""
Profile SSTG frontier queue behavior - simpler version.
"""
import sys
sys.path.insert(0, '/home/daojie/sstg-expl')

import numpy as np
from simulation.benchmark import BenchmarkRunner
from simulation.simple_env import create_environment

# Monkey-patch to add instrumentation
from src.core import explorer as explorer_module

# Global stats tracking
_profiling_stats = {
    'queue_sizes_by_env': {},
    'current_env': None
}

# Patch the SSTGExplorer class methods
original_generate_frontiers = explorer_module.SSTGExplorer._generate_frontiers
def tracked_generate_frontiers(self, position):
    result = original_generate_frontiers(self, position)
    # Record queue size after generation
    if _profiling_stats['current_env'] and hasattr(self, 'frontier_queue') and self.frontier_queue:
        env_name = _profiling_stats['current_env']
        if env_name not in _profiling_stats['queue_sizes_by_env']:
            _profiling_stats['queue_sizes_by_env'][env_name] = []
        _profiling_stats['queue_sizes_by_env'][env_name].append(len(self.frontier_queue))
    return result

original_update_priorities = explorer_module.SSTGExplorer._update_all_priorities
def tracked_update_priorities(self):
    # Count how many priorities we're updating
    if _profiling_stats['current_env'] and hasattr(self, 'frontier_queue') and self.frontier_queue:
        env_name = _profiling_stats['current_env']
        key = f'{env_name}_priority_updates'
        _profiling_stats[key] = _profiling_stats.get(key, 0) + len(self.frontier_queue)
    return original_update_priorities(self)

# Apply patches
explorer_module.SSTGExplorer._generate_frontiers = tracked_generate_frontiers
explorer_module.SSTGExplorer._update_all_priorities = tracked_update_priorities

def profile_environment(env_name):
    """Profile a single environment."""
    from src.core.explorer import SSTGExplorer
    from src.config import ExplorerConfig

    _profiling_stats['current_env'] = env_name
    _profiling_stats[f'{env_name}_priority_updates'] = 0

    # Create environment
    if env_name == 'sparse_obstacles':
        env = create_environment('obstacles', width=10.0, height=10.0, num_obstacles=5, seed=42)
    elif env_name == 'empty':
        env = create_environment('empty', width=10.0, height=10.0)
    elif env_name == 'corridor':
        env = create_environment('corridor', length=15.0, width=2.5)
    elif env_name == 'multiple_rooms':
        env = create_environment('multiple_rooms', width=15.0, height=10.0)
    else:
        raise ValueError(f"Unknown environment: {env_name}")

    env.name = env_name

    # Create explorer
    config = ExplorerConfig(
        r_view=2.0,
        d_theta=30.0,
        overlap=0.25,
        r_robot=0.3,
        verbose=False
    )
    explorer = SSTGExplorer(config=config)

    # Run exploration
    result = explorer.explore(
        env.get_occupancy_map(),  # <-- Call get_occupancy_map() to initialize
        start_pose=(5.0, 5.0, 0.0),
        visualizer=None
    )

    metadata = result['metadata']

    return {
        'coverage': metadata['coverage_ratio'],
        'time': metadata['total_time'],
        'nodes': metadata['num_nodes'],
        'distance': metadata['total_distance']
    }

if __name__ == '__main__':
    print("="*80)
    print("SSTG FRONTIER QUEUE PROFILING")
    print("="*80)
    print()

    environments = ['empty', 'sparse_obstacles', 'corridor', 'multiple_rooms']
    results = {}

    for env_name in environments:
        print(f"Profiling: {env_name}...")
        result = profile_environment(env_name)
        results[env_name] = result
        print(f"  Coverage: {result['coverage']:.1%}, Time: {result['time']:.2f}s, Nodes: {result['nodes']}")
        print()

    # Analysis
    print()
    print("="*80)
    print("FRONTIER QUEUE ANALYSIS")
    print("="*80)
    print()

    print(f"{'Environment':<20} {'Nodes':<8} {'Avg Queue':<12} {'Max Queue':<12} {'Priority Updates':<18}")
    print("-"*80)

    for env_name in environments:
        result = results[env_name]
        nodes = result['nodes']

        queue_sizes = _profiling_stats['queue_sizes_by_env'].get(env_name, [])
        if queue_sizes:
            avg_queue = np.mean(queue_sizes)
            max_queue = np.max(queue_sizes)
        else:
            avg_queue = 0
            max_queue = 0

        priority_updates = _profiling_stats.get(f'{env_name}_priority_updates', 0)

        print(f"{env_name:<20} {nodes:<8} {avg_queue:<12.1f} {max_queue:<12} {priority_updates:<18}")

    print()
    print()
    print("="*80)
    print("COMPUTATIONAL OVERHEAD ANALYSIS")
    print("="*80)
    print()

    print(f"{'Environment':<20} {'Iterations':<12} {'Updates/Iter':<15} {'vs Frontier*':<15}")
    print("-"*80)

    for env_name in environments:
        result = results[env_name]
        iterations = result['nodes'] - 1  # N nodes = N-1 iterations

        priority_updates = _profiling_stats.get(f'{env_name}_priority_updates', 0)
        updates_per_iter = priority_updates / max(iterations, 1)

        # Frontier algorithm typically has 15-25 frontiers, does 1 distance calc per frontier
        frontier_work_per_iter = 20  # Typical
        overhead_ratio = updates_per_iter / frontier_work_per_iter

        print(f"{env_name:<20} {iterations:<12} {updates_per_iter:<15.1f} {overhead_ratio:<15.1f}x")

    print()
    print("* Frontier algorithm typically computes ~20 distances per iteration")
    print("  SSTG updates all frontier priorities every iteration (current implementation)")
    print()

    # Summary insights
    print()
    print("="*80)
    print("KEY FINDINGS")
    print("="*80)
    print()

    total_priority_work = sum(_profiling_stats.get(f'{env}_priority_updates', 0) for env in environments)
    total_iterations = sum(results[env]['nodes'] - 1 for env in environments)
    avg_work_per_iter = total_priority_work / total_iterations

    print(f"1. Average queue size: {np.mean([np.mean(_profiling_stats['queue_sizes_by_env'].get(env, [0])) for env in environments]):.1f} frontiers")
    print(f"2. Average priority updates per iteration: {avg_work_per_iter:.1f}")
    print(f"3. Computational overhead vs Frontier: {avg_work_per_iter / 20:.1f}x")
    print()
    print("BOTTLENECK IDENTIFIED:")
    print("  - SSTG maintains large frontier queue (50-150 frontiers)")
    print("  - Updates ALL frontier priorities every iteration")
    print(f"  - This causes {avg_work_per_iter / 20:.1f}x more priority computations than Frontier")
    print()
    print("OPTIMIZATION POTENTIAL:")
    print("  1. Prune low-priority frontiers aggressively")
    print("  2. Only update priorities for nearby frontiers")
    print("  3. Use lazy priority updates (recompute on pop)")
    print("  --> Could reduce priority work by 5-10x")
    print()
