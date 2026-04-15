"""
Visualize the improvement from parameter tuning on multiple_rooms environment.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from simulation.simple_env import create_environment
from src.core.explorer import SSTGExplorer
from src.config import ExplorerConfig
from src.utils.visualization import visualize_exploration

print("="*70)
print("Visualizing SSTG Improvement on Multiple Rooms")
print("="*70)

# Create environment
env = create_environment('multiple_rooms', width=15.0, height=10.0)
occupancy_grid = env.get_occupancy_map()
start_pose = env.get_start_pose()

# Test with improved parameters
print("\nRunning SSTG with improved parameters...")
config = ExplorerConfig()  # Uses improved defaults
explorer = SSTGExplorer(config=config)
result = explorer.explore(occupancy_grid, start_pose)

print(f"\nResults:")
print(f"  Coverage: {result['metadata']['coverage_ratio']:.1%}")
print(f"  Nodes: {result['metadata']['num_nodes']}")
print(f"  Distance: {result['metadata']['total_distance']:.2f}m")

# Generate visualization
title = (f"SSTG (Improved) - Multiple Rooms\n"
         f"Coverage: {result['metadata']['coverage_ratio']:.1%} | "
         f"Distance: {result['metadata']['total_distance']:.1f}m | "
         f"Nodes: {result['metadata']['num_nodes']}\n"
         f"d_theta=30°, alpha_early=1.5, threshold=0.005")

output_path = 'outputs/visualizations/sstg_multiple_rooms_improved.png'
os.makedirs(os.path.dirname(output_path), exist_ok=True)

visualize_exploration(
    occupancy_grid=occupancy_grid,
    explored_nodes=result['nodes'],
    r_view=config.r_view,
    show_coverage=True,
    show_connections=True,
    save_path=output_path,
    figsize=(12, 10),
    dpi=150,
    title=title
)

print(f"\nVisualization saved to: {output_path}")
print("\nComparison with before:")
print(f"  Before: 63.5% coverage, 14 nodes, 30.14m")
print(f"  After:  {result['metadata']['coverage_ratio']:.1%} coverage, {result['metadata']['num_nodes']} nodes, {result['metadata']['total_distance']:.2f}m")
print(f"  Improvement: +{(result['metadata']['coverage_ratio'] - 0.635)*100:.1f}% coverage, +{result['metadata']['num_nodes'] - 14} nodes")
