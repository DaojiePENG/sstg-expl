#!/usr/bin/env python3
"""
Run complete SSTG benchmark with final configuration.
"""
import sys
sys.path.insert(0, '/home/daojie/sstg-expl')

from simulation.benchmark import BenchmarkRunner
from simulation.simple_env import create_environment

print("="*80)
print("SSTG Explorer - Complete Benchmark")
print("="*80)
print("Configuration: d_theta=30°, r_view=2.0m, bug fixes applied")
print("Experiments: 7 algorithms × 9 environments × 5 runs = 315 total")
print("="*80)

# Create benchmark runner
benchmark = BenchmarkRunner(
    num_runs=5,
    output_dir='/home/daojie/sstg-expl/outputs/benchmarks',
    seed=42
)

# Define algorithms to test
algorithms = [
    'uniform_grid',
    'rrt',
    'frontier',
    'nbv',
    'sstg',           # Baseline
    'sstg_enhanced',  # Enhanced distance
    'sstg_optimal'    # Optimal config
]

# Define environments
env_empty = create_environment('empty', width=10.0, height=10.0)
env_empty.name = 'empty'

env_obstacles = create_environment('obstacles', width=10.0, height=10.0, num_obstacles=5, seed=42)
env_obstacles.name = 'sparse_obstacles'

env_corridor = create_environment('corridor', length=15.0, width=2.5)
env_corridor.name = 'corridor'

env_rooms = create_environment('multiple_rooms', width=15.0, height=10.0)
env_rooms.name = 'multiple_rooms'

env_lcorridor = create_environment('l_corridor')
env_lcorridor.name = 'l_shaped_corridor'

env_maze = create_environment('maze', width=12.0, height=12.0)
env_maze.name = 'maze'

env_dense = create_environment('dense_obstacles', width=10.0, height=10.0, num_obstacles=15, seed=43)
env_dense.name = 'dense_obstacles'

env_narrow = create_environment('narrow_passages', width=15.0, height=10.0)
env_narrow.name = 'narrow_passages'

env_warehouse = create_environment('warehouse', width=15.0, height=12.0)
env_warehouse.name = 'warehouse'

environments = [
    env_empty, env_obstacles, env_corridor, env_rooms,
    env_lcorridor, env_maze, env_dense, env_narrow, env_warehouse
]

print(f"\nStarting benchmark...")
print(f"  Algorithms: {len(algorithms)}")
print(f"  Environments: {len(environments)}")
print(f"  Total experiments: {len(algorithms) * len(environments) * benchmark.num_runs}")
print()

# Run benchmark
benchmark.run_benchmark(algorithms, environments)

print("\n" + "="*80)
print("Benchmark Complete!")
print("="*80)
