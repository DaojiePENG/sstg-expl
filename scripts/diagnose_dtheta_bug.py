"""
Diagnose why d_theta=30° causes frontiers to have near-zero priority.
"""
import sys
import os
sys.path.insert(0, '/home/daojie/sstg-expl')

import numpy as np
from simulation.simple_env import create_environment
from src.core.explorer import SSTGExplorer
from src.config import ExplorerConfig
from src.core.collision_checker import CollisionChecker, CollisionType
from src.utils.geometry import compute_target_point

print("="*70)
print("Diagnosing d_theta=30° frontier generation")
print("="*70)

# Create simple environment
env = create_environment('obstacles', width=10.0, height=10.0, num_obstacles=5, seed=42)
occupancy_grid = env.get_occupancy_map()
start_pose = env.get_start_pose()

# Test frontier generation with different d_theta
configs = [
    ('45°', 45.0),
    ('30°', 30.0)
]

for name, d_theta in configs:
    print(f"\n{'='*70}")
    print(f"Testing d_theta={name}")
    print(f"{'='*70}")

    # Create config
    config = ExplorerConfig(d_theta=d_theta)
    r_view = config.r_view
    d_repel = config.d_repel

    print(f"Parameters: r_view={r_view}m, d_repel={d_repel}m")
    print(f"Number of directions: {int(360/d_theta)}")

    # Create collision checker
    collision_checker = CollisionChecker(
        occupancy_grid,
        config.r_robot,
        config.d_safe
    )

    # Simulate first 3 nodes
    explored_nodes = [start_pose[:2]]

    for iteration in range(3):
        print(f"\n--- Iteration {iteration} ---")
        print(f"Explored nodes: {len(explored_nodes)}")
        print(f"Current position: ({explored_nodes[-1][0]:.2f}, {explored_nodes[-1][1]:.2f})")

        # Generate frontiers from current position
        position = explored_nodes[-1]
        angles = [i * d_theta for i in range(int(360/d_theta))]

        free_count = 0
        soft_count = 0
        hard_count = 0
        strengths = []

        for angle in angles:
            target = compute_target_point(position, r_view, angle)
            collision_type, strength = collision_checker.check_collision_type(
                target, explored_nodes, d_repel
            )

            if collision_type == CollisionType.FREE:
                free_count += 1
                strengths.append(strength)
            elif collision_type == CollisionType.SOFT_OBSTACLE:
                soft_count += 1
                strengths.append(strength)
            else:  # HARD_OBSTACLE
                hard_count += 1

        print(f"  FREE: {free_count}, SOFT_OBSTACLE: {soft_count}, HARD_OBSTACLE: {hard_count}")

        if strengths:
            print(f"  Strength range: [{min(strengths):.4f}, {max(strengths):.4f}], mean: {np.mean(strengths):.4f}")
            print(f"  Strengths < 0.5: {sum(1 for s in strengths if s < 0.5)}")
            print(f"  Strengths < 0.1: {sum(1 for s in strengths if s < 0.1)}")

        # Simulate moving to best frontier (simple heuristic: first FREE with high strength)
        best_target = None
        for angle in angles:
            target = compute_target_point(position, r_view, angle)
            collision_type, strength = collision_checker.check_collision_type(
                target, explored_nodes, d_repel
            )
            if collision_type == CollisionType.FREE and strength >= 0.8:
                best_target = target
                break

        if best_target is None:
            print("  ❌ No good frontiers available!")
            break
        else:
            explored_nodes.append(best_target)
            print(f"  ✅ Moved to: ({best_target[0]:.2f}, {best_target[1]:.2f})")

print("\n" + "="*70)
print("KEY INSIGHTS")
print("="*70)
print("""
The problem with d_theta=30° likely comes from:
1. More directions (12 vs 8) means more frontiers
2. As explored_nodes grow, more frontiers fall within d_repel (1.75m)
3. These frontiers get SOFT_OBSTACLE status with strength < 1.0
4. With alpha=2.0, distance decay is aggressive: priority = strength * 1/(1+(dist/r_view)^2)
5. For distant frontiers (dist > 3-4m), even with strength=1.0, priority becomes very small
6. Combined with strength < 1.0, priority → near-zero

Solution approaches:
A. Reduce d_repel (allow closer re-exploration)
B. Change how SOFT_OBSTACLE affects priority
C. Use different priority formula that doesn't decay so aggressively
""")
