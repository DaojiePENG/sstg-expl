"""
Simple unit tests for SSTG Explorer core components.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.core.frontier import Frontier, FrontierQueue
from src.utils.geometry import *
from src.map.occupancy_grid import OccupancyGrid
from simulation.simple_env import EmptyRoom


def test_frontier_queue():
    """Test FrontierQueue basic operations."""
    print("Testing FrontierQueue...")

    queue = FrontierQueue()

    # Test add and pop
    id1 = queue.add((0, 0), 45, (1, 1), priority=0.8)
    id2 = queue.add((0, 0), 90, (0, 2), priority=0.5)
    id3 = queue.add((1, 1), 0, (2, 1), priority=0.9)

    assert queue.size() == 3, "Queue size should be 3"

    # Pop should return highest priority (0.9)
    best = queue.pop()
    assert best.priority == 0.9, f"Expected priority 0.9, got {best.priority}"
    assert queue.size() == 2, "Queue size should be 2 after pop"

    # Update priority
    queue.update_priority(id1, 1.0)
    best = queue.peek()
    assert best.priority == 1.0, "Updated priority should be 1.0"

    # Remove
    queue.remove(id2)
    assert queue.size() == 1, "Queue size should be 1 after remove"

    print("✓ FrontierQueue tests passed!")


def test_geometry():
    """Test geometry utilities."""
    print("Testing geometry utilities...")

    # Test compute_target_point
    target = compute_target_point((0, 0), 1.0, 0)
    assert np.isclose(target[0], 1.0) and np.isclose(target[1], 0.0)

    target = compute_target_point((0, 0), 1.0, 90)
    assert np.isclose(target[0], 0.0) and np.isclose(target[1], 1.0)

    # Test euclidean_distance
    dist = euclidean_distance((0, 0), (3, 4))
    assert np.isclose(dist, 5.0)

    # Test find_longest_free_sector
    sector_status = [True, True, False, True, True, True, False, True]
    start, end, center = find_longest_free_sector(sector_status, 45.0)
    assert start >= 0 and end >= 0, "Should find free sector"

    print("✓ Geometry tests passed!")


def test_occupancy_grid():
    """Test OccupancyGrid."""
    print("Testing OccupancyGrid...")

    # Create empty grid
    grid = OccupancyGrid.create_empty(10.0, 10.0, 0.1)

    assert grid.width == 100, "Grid width should be 100 cells"
    assert grid.height == 100, "Grid height should be 100 cells"

    # Test coordinate conversion
    i, j = grid.world_to_grid(5.0, 5.0)
    x, y = grid.grid_to_world(i, j)
    assert np.isclose(x, 5.0, atol=0.1) and np.isclose(y, 5.0, atol=0.1)

    # Test occupancy
    assert grid.is_free(5.0, 5.0), "Center should be free"

    grid.set_value(5.0, 5.0, 100)
    assert grid.is_occupied(5.0, 5.0), "Center should now be occupied"

    print("✓ OccupancyGrid tests passed!")


def test_environment():
    """Test environment creation."""
    print("Testing environment creation...")

    env = EmptyRoom(width=10.0, height=10.0)
    occupancy_grid = env.get_occupancy_map()

    assert occupancy_grid.world_width >= 10.0
    assert occupancy_grid.world_height >= 10.0

    # Check that center is free and edges are occupied
    assert occupancy_grid.is_free(5.0, 5.0), "Center should be free"
    assert occupancy_grid.is_occupied(0.1, 0.1), "Corner should be occupied (wall)"

    print("✓ Environment tests passed!")


def run_all_tests():
    """Run all tests."""
    print("="*60)
    print("Running SSTG Explorer Tests")
    print("="*60)

    try:
        test_frontier_queue()
        test_geometry()
        test_occupancy_grid()
        test_environment()

        print("\n" + "="*60)
        print("✓ All tests passed!")
        print("="*60)
        return True

    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        return False
    except Exception as e:
        print(f"\n✗ Error during testing: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
