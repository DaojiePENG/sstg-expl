"""
Simple example demonstrating real-time visualization.

Run this to see the exploration process in real-time with all frontier states.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.core.explorer import SSTGExplorer
from simulation.simple_env import create_environment
from src.utils.realtime_viz import RealtimeVisualizer


def demo_realtime_visualization():
    """
    Demonstrate real-time visualization of exploration.

    Shows:
    - Current position (green star)
    - Explored nodes (red dots with numbers)
    - Active frontiers (yellow circles)
    - Blocked frontiers - obstacles (red X)
    - Blocked frontiers - too close (orange X)
    - Coverage circles (blue)
    - Real-time statistics in title
    """
    print("="*60)
    print("Real-time Visualization Demo")
    print("="*60)

    # Create environment
    print("\n1. Creating environment...")
    env = create_environment('obstacles', width=10.0, height=10.0,
                            num_obstacles=5, seed=42)
    occupancy_grid = env.get_occupancy_map()
    start_pose = env.get_start_pose()

    print(f"   Environment: 10m x 10m with 5 obstacles")
    print(f"   Start: ({start_pose[0]:.1f}, {start_pose[1]:.1f})")

    # Create real-time visualizer
    print("\n2. Creating real-time visualizer...")
    print("   Legend:")
    print("   ⭐ Green Star     = Current Position")
    print("   🔴 Red Dot+Number = Explored Node")
    print("   🟡 Yellow Circle  = Active Frontier (to explore)")
    print("   ❌ Red X          = Blocked by Obstacle")
    print("   🟠 Orange X       = Too Close to Explored Node")
    print("   🔵 Blue Circle    = Coverage Area")

    visualizer = RealtimeVisualizer(
        occupancy_grid=occupancy_grid,
        r_view=2.0,
        figsize=(14, 12),
        update_interval=0.5  # Update every 0.5 seconds
    )

    # Create explorer
    print("\n3. Running exploration...")
    print("   Watch the visualization window for real-time updates!")
    print("-" * 60)

    explorer = SSTGExplorer(
        r_view=2.0,
        d_theta=45.0,
        overlap=0.25,
        r_robot=0.3
    )

    # Run exploration with real-time visualization
    result = explorer.explore(occupancy_grid, start_pose, visualizer=visualizer)

    print("-" * 60)

    # Print results
    print(f"\n4. Exploration Complete!")
    print(f"   Nodes: {len(result['nodes'])}")
    print(f"   Coverage: {result['metadata']['coverage_ratio']:.1%}")
    print(f"   Distance: {result['metadata']['total_distance']:.2f}m")
    print(f"   Time: {result['metadata']['total_time']:.2f}s")

    # Save final state
    os.makedirs('outputs/explorations', exist_ok=True)
    output_path = 'outputs/explorations/realtime_demo.png'
    visualizer.save(output_path)
    print(f"\n   Visualization saved to: {output_path}")

    print("\n" + "="*60)
    print("Demo complete! Close the visualization window to exit.")
    print("="*60)

    # Keep window open until user closes it
    input("\nPress Enter to close and exit...")
    visualizer.close()


if __name__ == '__main__':
    demo_realtime_visualization()
