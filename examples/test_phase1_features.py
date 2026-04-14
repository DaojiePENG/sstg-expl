"""
Test script for Phase 1 advanced features:
- A* path planning
- Distance field
- Narrow passage detection
"""
import sys
import os
import time
import matplotlib.pyplot as plt
import numpy as np

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from simulation.simple_env import create_environment
from src.planning.astar import AStarPlanner
from src.map.distance_field import DistanceField
from src.core.narrow_passage import NarrowPassageDetector


def test_astar_planning():
    """Test A* path planning."""
    print("=" * 80)
    print("Test 1: A* Path Planning")
    print("=" * 80)

    # Create environment with obstacles
    env = create_environment('corridor', length=15.0, width=2.5)
    grid = env.get_occupancy_map()

    # Create A* planner
    planner = AStarPlanner(grid, robot_radius=0.3)

    # Plan some paths
    test_cases = [
        ((1.0, 1.25), (14.0, 1.25), "Straight path through corridor"),
        ((1.0, 0.5), (14.0, 2.0), "Diagonal path"),
        ((7.5, 0.5), (7.5, 2.0), "Cross corridor")
    ]

    print("\nPlanning paths:")
    for start, goal, description in test_cases:
        print(f"\n  {description}")
        print(f"    Start: {start}, Goal: {goal}")

        start_time = time.time()
        path = planner.plan(start, goal)
        elapsed = time.time() - start_time

        if path is not None:
            path_length = planner.get_path_length(path)
            print(f"    ✓ Path found: {len(path)} waypoints, "
                  f"{path_length:.2f}m length, {elapsed*1000:.1f}ms")
        else:
            print(f"    ✗ No path found ({elapsed*1000:.1f}ms)")

    # Visualize a path
    if test_cases:
        start, goal, _ = test_cases[0]
        path = planner.plan(start, goal)

        if path:
            fig, ax = plt.subplots(figsize=(12, 4))

            # Plot map
            extent = [grid.origin[0],
                     grid.origin[0] + grid.world_width,
                     grid.origin[1],
                     grid.origin[1] + grid.world_height]

            ax.imshow(grid.data, cmap='gray_r', origin='lower',
                     extent=extent, alpha=0.7)

            # Plot path
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            ax.plot(path_x, path_y, 'b-', linewidth=2, label='A* Path')
            ax.plot(path_x, path_y, 'bo', markersize=6)

            # Mark start and goal
            ax.plot(start[0], start[1], 'go', markersize=12, label='Start')
            ax.plot(goal[0], goal[1], 'ro', markersize=12, label='Goal')

            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('A* Path Planning Test')
            ax.legend()
            ax.axis('equal')
            ax.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig('outputs/phase1_astar_test.png', dpi=100)
            print(f"\n  Visualization saved to outputs/phase1_astar_test.png")
            plt.close()

    print("\n✓ A* path planning test complete!\n")


def test_distance_field():
    """Test distance field."""
    print("=" * 80)
    print("Test 2: Distance Field")
    print("=" * 80)

    # Create environment
    env = create_environment('obstacles', width=10.0, height=10.0, num_obstacles=5)
    grid = env.get_occupancy_map()

    # Create distance field
    print("\nComputing distance field...")
    start_time = time.time()
    distance_field = DistanceField(grid, obstacle_threshold=50)
    elapsed = time.time() - start_time
    print(f"  Distance field computed in {elapsed*1000:.1f}ms")

    # Get statistics
    stats = distance_field.get_statistics()
    print(f"\n  Statistics:")
    print(f"    Free cells: {stats['num_free_cells']}")
    print(f"    Obstacle cells: {stats['num_obstacle_cells']}")
    print(f"    Min distance: {stats['min_distance']:.3f}m")
    print(f"    Max distance: {stats['max_distance']:.3f}m")
    print(f"    Mean distance: {stats['mean_distance']:.3f}m")
    print(f"    Std distance: {stats['std_distance']:.3f}m")

    # Test distance queries
    print(f"\n  Testing distance queries:")
    test_points = [
        (5.0, 5.0, "Center"),
        (1.0, 1.0, "Corner"),
        (9.0, 9.0, "Far corner")
    ]

    for x, y, label in test_points:
        dist = distance_field.get_distance(x, y)
        is_safe = distance_field.is_collision_free(x, y, safety_distance=0.5)
        print(f"    {label} ({x:.1f}, {y:.1f}): "
              f"distance={dist:.3f}m, safe={is_safe}")

    # Visualize distance field
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    extent = [grid.origin[0],
             grid.origin[0] + grid.world_width,
             grid.origin[1],
             grid.origin[1] + grid.world_height]

    # Original map
    ax1.imshow(grid.data, cmap='gray_r', origin='lower',
              extent=extent, alpha=0.7)
    ax1.set_title('Original Occupancy Map')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.axis('equal')

    # Distance field
    vis_field = distance_field.visualize_distance_field(max_distance=3.0)
    im = ax2.imshow(vis_field, cmap='jet', origin='lower',
                    extent=extent, alpha=0.9)
    ax2.set_title('Distance Field (max 3.0m)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.axis('equal')

    plt.colorbar(im, ax=ax2, label='Distance to obstacle (normalized)')
    plt.tight_layout()
    plt.savefig('outputs/phase1_distance_field_test.png', dpi=100)
    print(f"\n  Visualization saved to outputs/phase1_distance_field_test.png")
    plt.close()

    print("\n✓ Distance field test complete!\n")

    return distance_field  # Return for use in next test


def test_narrow_passage_detection(distance_field=None):
    """Test narrow passage detection."""
    print("=" * 80)
    print("Test 3: Narrow Passage Detection")
    print("=" * 80)

    # Create corridor environment (has narrow passages)
    env = create_environment('corridor', length=15.0, width=2.5)
    grid = env.get_occupancy_map()

    # Create distance field if not provided
    if distance_field is None:
        print("\nComputing distance field...")
        distance_field = DistanceField(grid, obstacle_threshold=50)

    # Create narrow passage detector
    detector = NarrowPassageDetector(
        grid,
        distance_field,
        robot_radius=0.3,
        narrow_threshold=1.5,  # Consider passages < 1.5m as narrow
        base_d_theta=45.0,
        min_d_theta=15.0
    )

    # Test passage detection at various points
    print("\n  Testing passage detection:")
    test_points = [
        (7.5, 1.25, "Corridor center"),
        (7.5, 0.5, "Near wall"),
        (1.0, 1.25, "Entrance")
    ]

    for x, y, label in test_points:
        passage = detector.detect_passage((x, y))
        print(f"\n    {label} ({x:.1f}, {y:.1f}):")
        print(f"      Width: {passage.width:.3f}m")
        print(f"      Direction: {passage.direction:.1f}°")
        print(f"      Is narrow: {passage.is_narrow}")
        print(f"      Recommended d_theta: {passage.recommended_d_theta:.1f}°")

    # Find all narrow regions
    print("\n  Finding narrow regions...")
    narrow_positions = detector.find_narrow_regions(sample_spacing=0.5)
    print(f"    Found {len(narrow_positions)} narrow region sample points")

    # Get statistics
    stats = detector.get_statistics()
    print(f"\n  Passage statistics:")
    print(f"    Num narrow regions: {stats['num_narrow_regions']}")
    print(f"    Min width: {stats['min_width']:.3f}m")
    print(f"    Max width: {stats['max_width']:.3f}m")
    print(f"    Mean width: {stats['mean_width']:.3f}m")
    if stats['narrowest_position']:
        print(f"    Narrowest position: {stats['narrowest_position']}")

    # Visualize narrow regions
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))

    extent = [grid.origin[0],
             grid.origin[0] + grid.world_width,
             grid.origin[1],
             grid.origin[1] + grid.world_height]

    # Original map
    ax1.imshow(grid.data, cmap='gray_r', origin='lower',
              extent=extent, alpha=0.7)
    ax1.set_title('Occupancy Map')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.axis('equal')

    # Passage width map
    width_map = detector.get_passage_width_map()
    im2 = ax2.imshow(width_map, cmap='viridis', origin='lower',
                     extent=extent, alpha=0.9, vmin=0, vmax=3.0)
    ax2.set_title('Passage Width Map')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.axis('equal')
    plt.colorbar(im2, ax=ax2, label='Width (m)')

    # Narrow regions
    narrow_map = detector.visualize_narrow_regions()
    ax3.imshow(grid.data, cmap='gray_r', origin='lower',
              extent=extent, alpha=0.3)
    ax3.imshow(narrow_map, cmap='Reds', origin='lower',
              extent=extent, alpha=0.6)
    ax3.set_title(f'Narrow Regions (< {detector.narrow_threshold}m)')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.axis('equal')

    plt.tight_layout()
    plt.savefig('outputs/phase1_narrow_passage_test.png', dpi=100)
    print(f"\n  Visualization saved to outputs/phase1_narrow_passage_test.png")
    plt.close()

    print("\n✓ Narrow passage detection test complete!\n")


def main():
    """Run all Phase 1 feature tests."""
    print("\n" + "=" * 80)
    print("SSTG Explorer - Phase 1 Advanced Features Test Suite")
    print("=" * 80 + "\n")

    # Create output directory
    os.makedirs('outputs', exist_ok=True)

    # Run tests
    test_astar_planning()
    distance_field = test_distance_field()
    test_narrow_passage_detection(distance_field)

    print("=" * 80)
    print("All Phase 1 tests complete!")
    print("=" * 80)
    print("\nGenerated visualizations:")
    print("  - outputs/phase1_astar_test.png")
    print("  - outputs/phase1_distance_field_test.png")
    print("  - outputs/phase1_narrow_passage_test.png")
    print()


if __name__ == '__main__':
    main()
