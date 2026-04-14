"""
Basic exploration example using SSTG Explorer.

This script demonstrates how to use the SSTG Explorer algorithm
to explore different environments.
"""
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.core.explorer import SSTGExplorer
from src.config import ExplorerConfig, FrontierSelectionStrategy
from simulation.simple_env import create_environment
from src.utils.visualization import (
    visualize_exploration,
    visualize_coverage_map,
    plot_exploration_metrics
)
from src.utils.realtime_viz import RealtimeVisualizer
import os


def run_basic_exploration(
    env_type: str = 'empty',
    r_view: float = 2.0,
    d_theta: float = 45.0,
    overlap: float = 0.25,
    visualize: bool = True,
    save_results: bool = True,  # Changed default to True
    realtime_viz: bool = False,  # Enable real-time visualization
    save_animation: bool = False,  # Save exploration animation
    frontier_strategy: FrontierSelectionStrategy = FrontierSelectionStrategy.BASELINE
):
    """
    Run basic exploration on a specified environment.

    Args:
        env_type: Type of environment ('empty', 'obstacles', 'corridor',
                                       'multiple_rooms', 'apartment').
        r_view: View radius in meters.
        d_theta: Angular interval in degrees.
        overlap: Overlap distance in meters.
        visualize: Whether to visualize results.
        save_results: Whether to save results to files.
        realtime_viz: Whether to show real-time exploration visualization.
        save_animation: Whether to save exploration process as animated GIF.
        frontier_strategy: Frontier selection strategy to use.
    """
    print("="*60)
    print("SSTG Explorer - Basic Exploration Example")
    print("="*60)

    # Create output directories
    if save_results:
        os.makedirs('outputs/explorations', exist_ok=True)
        os.makedirs('outputs/coverage', exist_ok=True)
        os.makedirs('outputs/metrics', exist_ok=True)
        os.makedirs('outputs/animations', exist_ok=True)

    # Create environment
    print(f"\n1. Creating '{env_type}' environment...")
    if env_type == 'empty':
        env = create_environment('empty', width=10.0, height=10.0)
    elif env_type == 'obstacles':
        env = create_environment('obstacles', width=10.0, height=10.0,
                                num_obstacles=5, seed=42)
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

    print(f"   Environment size: {env.width}m x {env.height}m")
    print(f"   Resolution: {env.resolution}m")
    print(f"   Start pose: ({start_pose[0]:.2f}, {start_pose[1]:.2f}, {start_pose[2]:.1f}°)")

    # Create explorer
    print(f"\n2. Creating SSTG Explorer...")
    print(f"   r_view: {r_view}m")
    print(f"   d_theta: {d_theta}°")
    print(f"   overlap: {overlap}m")
    print(f"   frontier_strategy: {frontier_strategy.value}")

    explorer = SSTGExplorer(
        r_view=r_view,
        d_theta=d_theta,
        overlap=overlap,
        r_robot=0.3,
        frontier_strategy=frontier_strategy
    )

    # Run exploration
    print(f"\n3. Running exploration...")
    if realtime_viz:
        print("   Real-time visualization enabled - watch the exploration live!")
    if save_animation:
        print("   Animation recording enabled - will save to outputs/animations/")
    print("-" * 60)

    # Create real-time visualizer if requested
    visualizer = None
    if realtime_viz:
        animation_path = f'outputs/animations/animation_{env_type}.gif' if save_animation else None
        visualizer = RealtimeVisualizer(
            occupancy_grid=occupancy_grid,
            r_view=r_view,
            figsize=(14, 12),
            update_interval=0.3,  # Update every 0.3 seconds
            save_animation=save_animation,
            animation_path=animation_path
        )

    result = explorer.explore(occupancy_grid, start_pose, visualizer=visualizer)

    print("-" * 60)

    # Print results
    print(f"\n4. Exploration Results:")
    print(f"   Success: {result['success']}")
    print(f"   Number of nodes: {result['metadata']['num_nodes']}")
    print(f"   Coverage ratio: {result['metadata']['coverage_ratio']:.2%}")
    print(f"   Total distance: {result['metadata']['total_distance']:.2f}m")
    print(f"   Exploration time: {result['metadata']['total_time']:.2f}s")
    print(f"   Min node distance: {result['metadata']['min_node_distance']:.2f}m")
    print(f"   Mean node distance: {result['metadata']['mean_node_distance']:.2f}m")

    # Visualize
    if visualize:
        print(f"\n5. Generating visualizations...")
        if save_results:
            print(f"   Results will be saved to outputs/ with prefix: '{env_type}'")

        # Main exploration result
        visualize_exploration(
            occupancy_grid=occupancy_grid,
            explored_nodes=result['nodes'],
            r_view=r_view,
            show_coverage=True,
            show_connections=True,
            save_path=f'outputs/explorations/exploration_{env_type}.png' if save_results else None
        )

        # Coverage analysis
        visualize_coverage_map(
            occupancy_grid=occupancy_grid,
            explored_nodes=result['nodes'],
            r_view=r_view,
            save_path=f'outputs/coverage/coverage_{env_type}.png' if save_results else None
        )

        # Metrics
        plot_exploration_metrics(
            results_dict=result,
            save_path=f'outputs/metrics/metrics_{env_type}.png' if save_results else None
        )

        if save_results:
            print(f"\n   ✓ All visualizations saved to outputs/!")

    print(f"\n{'='*60}")
    print("Exploration complete!")
    print("="*60)

    return result


def compare_parameters():
    """Compare exploration with different parameter settings."""
    print("="*60)
    print("SSTG Explorer - Parameter Comparison")
    print("="*60)

    # Create environment
    env = create_environment('obstacles', width=10.0, height=10.0,
                            num_obstacles=5, seed=42)
    occupancy_grid = env.get_occupancy_map()
    start_pose = env.get_start_pose()

    # Different parameter sets
    param_sets = [
        {'name': 'Dense (r=1.5m, θ=45°)', 'r_view': 1.5, 'd_theta': 45.0, 'overlap': 0.25},
        {'name': 'Standard (r=2.0m, θ=45°)', 'r_view': 2.0, 'd_theta': 45.0, 'overlap': 0.25},
        {'name': 'Sparse (r=2.5m, θ=45°)', 'r_view': 2.5, 'd_theta': 45.0, 'overlap': 0.25},
        {'name': 'Fine angle (r=2.0m, θ=22.5°)', 'r_view': 2.0, 'd_theta': 22.5, 'overlap': 0.25},
    ]

    results = []

    for params in param_sets:
        print(f"\nTesting: {params['name']}")
        print("-" * 60)

        explorer = SSTGExplorer(
            r_view=params['r_view'],
            d_theta=params['d_theta'],
            overlap=params['overlap'],
            r_robot=0.3
        )

        result = explorer.explore(occupancy_grid, start_pose)
        results.append({
            'name': params['name'],
            'result': result
        })

        print(f"   Nodes: {result['metadata']['num_nodes']}")
        print(f"   Coverage: {result['metadata']['coverage_ratio']:.2%}")
        print(f"   Distance: {result['metadata']['total_distance']:.2f}m")
        print(f"   Time: {result['metadata']['total_time']:.2f}s")

    # Summary comparison
    print("\n" + "="*60)
    print("Comparison Summary:")
    print("="*60)
    print(f"{'Configuration':<30} {'Nodes':>8} {'Coverage':>10} {'Distance':>10} {'Time':>8}")
    print("-" * 60)

    for r in results:
        print(f"{r['name']:<30} "
              f"{r['result']['metadata']['num_nodes']:>8} "
              f"{r['result']['metadata']['coverage_ratio']:>9.1%} "
              f"{r['result']['metadata']['total_distance']:>9.2f}m "
              f"{r['result']['metadata']['total_time']:>7.2f}s")

    print("="*60)


if __name__ == '__main__':
    # Example 1: Run on different environments (with real-time visualization and animation)
    print("\n\n### Example 1: Empty Room (Real-time Visualization + Animation) ###\n")
    run_basic_exploration(env_type='empty', visualize=True, realtime_viz=True, save_animation=True)

    print("\n\n### Example 2: Room with Obstacles ###\n")
    run_basic_exploration(env_type='obstacles', visualize=True, realtime_viz=True, save_animation=True)

    # print("\n\n### Example 3: Corridor ###\n")
    # run_basic_exploration(env_type='corridor', visualize=True)

    # print("\n\n### Example 4: Multiple Rooms ###\n")
    # run_basic_exploration(env_type='multiple_rooms', visualize=True)

    # print("\n\n### Example 5: Complex Apartment ###\n")
    # run_basic_exploration(env_type='apartment', visualize=True)

    # Example 2: Parameter comparison
    # print("\n\n### Parameter Comparison ###\n")
    # compare_parameters()
