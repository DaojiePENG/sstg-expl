"""
Quick test script to verify baseline algorithms work correctly.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.baseline import (
    UniformGridExplorer,
    RRTExplorer,
    FrontierExplorer,
    NextBestViewExplorer
)
from simulation.simple_env import create_environment


def test_algorithm(algorithm, env_name="empty"):
    """Test a single algorithm."""
    print(f"\nTesting {algorithm.name} Explorer...")

    # Create environment
    env = create_environment(env_name, width=10.0, height=10.0)
    occupancy_grid_obj = env.get_occupancy_map()
    # Convert to numpy array for baseline algorithms
    occupancy_grid = occupancy_grid_obj.data
    start_pose = env.get_start_pose()

    # Run exploration
    try:
        result = algorithm.explore(occupancy_grid, start_pose)

        if result['success']:
            metadata = result['metadata']
            print(f"  ✓ Success!")
            print(f"    Distance: {metadata['total_distance']:.2f}m")
            print(f"    Nodes: {metadata['num_nodes']}")
            print(f"    Coverage: {metadata['coverage_ratio']:.1%}")
            print(f"    Time: {metadata['computation_time']:.2f}s")
        else:
            print(f"  ✗ Failed")

        return result['success']

    except Exception as e:
        print(f"  ✗ Error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Test all baseline algorithms."""
    print("="*80)
    print("Baseline Algorithms - Quick Test")
    print("="*80)

    algorithms = [
        UniformGridExplorer(grid_spacing=2.0, r_view=2.0),
        RRTExplorer(max_iterations=100, step_size=1.0, r_view=2.0),
        FrontierExplorer(r_view=2.0, max_iterations=100),
        NextBestViewExplorer(r_view=2.0, n_candidates=20, max_iterations=100)
    ]

    results = []
    for alg in algorithms:
        success = test_algorithm(alg)
        results.append(success)

    print("\n" + "="*80)
    print(f"Test Results: {sum(results)}/{len(results)} algorithms passed")
    print("="*80)

    if all(results):
        print("\n✓ All baseline algorithms working correctly!")
    else:
        print("\n✗ Some algorithms failed - check errors above")


if __name__ == '__main__':
    main()
