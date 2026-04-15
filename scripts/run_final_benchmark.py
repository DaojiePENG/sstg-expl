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
print("Experiments: 7 algorithms × 5 environments × 5 runs = 175 total")
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
env_empty = create_environment('empty')
env_empty.name = 'empty'

env_obstacles = create_environment('obstacles')
env_obstacles.name = 'sparse_obstacles'

env_corridor = create_environment('corridor')
env_corridor.name = 'corridor'

env_rooms = create_environment('multiple_rooms')
env_rooms.name = 'multiple_rooms'

environments = [env_empty, env_obstacles, env_corridor, env_rooms]

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
