"""
Visualization utilities for SSTG Explorer.
"""
import numpy as np
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation

from src.map.occupancy_grid import OccupancyGrid


def visualize_exploration(
    occupancy_grid: OccupancyGrid,
    explored_nodes: List[dict],
    r_view: float,
    show_coverage: bool = True,
    show_connections: bool = False,
    save_path: Optional[str] = None,
    figsize: Tuple[int, int] = (12, 10),
    dpi: int = 100
):
    """
    Visualize exploration results.

    Args:
        occupancy_grid: Occupancy grid map.
        explored_nodes: List of explored node dictionaries.
        r_view: View radius in meters.
        show_coverage: Whether to show coverage circles.
        show_connections: Whether to show connections between sequential nodes.
        save_path: Path to save figure (None = display only).
        figsize: Figure size.
        dpi: Figure DPI.
    """
    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

    # Plot occupancy grid
    map_data = occupancy_grid.data
    extent = [
        occupancy_grid.origin[0],
        occupancy_grid.origin[0] + occupancy_grid.world_width,
        occupancy_grid.origin[1],
        occupancy_grid.origin[1] + occupancy_grid.world_height
    ]

    # Free space in white, obstacles in black
    ax.imshow(
        map_data,
        cmap='gray_r',
        origin='lower',
        extent=extent,
        alpha=0.7
    )

    # Extract positions
    positions = [node['position'] for node in explored_nodes]

    # Plot coverage circles
    if show_coverage:
        for pos in positions:
            circle = Circle(
                pos, r_view,
                color='lightblue',
                alpha=0.3,
                zorder=1
            )
            ax.add_patch(circle)

    # Plot connections
    if show_connections and len(positions) > 1:
        for i in range(len(positions) - 1):
            ax.plot(
                [positions[i][0], positions[i+1][0]],
                [positions[i][1], positions[i+1][1]],
                'b-', alpha=0.3, linewidth=1, zorder=2
            )

    # Plot nodes
    x_coords = [pos[0] for pos in positions]
    y_coords = [pos[1] for pos in positions]

    # Start node in green
    ax.plot(x_coords[0], y_coords[0], 'go', markersize=12,
            label='Start', zorder=4)

    # Other nodes in red
    if len(positions) > 1:
        ax.plot(x_coords[1:], y_coords[1:], 'ro', markersize=6,
                label='Explored Nodes', zorder=3)

    # End node with special marker
    ax.plot(x_coords[-1], y_coords[-1], 'rs', markersize=12,
            label='End', zorder=5)

    # Add node numbers as text annotations
    for i, (x, y) in enumerate(positions):
        # White background box for better readability
        bbox_props = dict(boxstyle='round,pad=0.3', facecolor='white',
                         edgecolor='black', alpha=0.8, linewidth=0.5)

        # Different colors for start, end, and intermediate nodes
        if i == 0:
            color = 'darkgreen'
            fontweight = 'bold'
        elif i == len(positions) - 1:
            color = 'darkred'
            fontweight = 'bold'
        else:
            color = 'darkred'
            fontweight = 'normal'

        ax.text(x, y, str(i),
                fontsize=8,
                color=color,
                fontweight=fontweight,
                ha='center',
                va='center',
                bbox=bbox_props,
                zorder=6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'SSTG Exploration Result ({len(positions)} nodes)')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=dpi, bbox_inches='tight')
        print(f"Figure saved to {save_path}")
        plt.show()  # Also display the figure
    else:
        plt.show()

    plt.close()


def visualize_coverage_map(
    occupancy_grid: OccupancyGrid,
    explored_nodes: List[dict],
    r_view: float,
    save_path: Optional[str] = None,
    figsize: Tuple[int, int] = (15, 5),
    dpi: int = 100
):
    """
    Visualize coverage analysis with multiple views.

    Args:
        occupancy_grid: Occupancy grid map.
        explored_nodes: List of explored nodes.
        r_view: View radius.
        save_path: Path to save figure.
        figsize: Figure size.
        dpi: Figure DPI.
    """
    from src.core.coverage_analyzer import CoverageAnalyzer

    # Create analyzer
    analyzer = CoverageAnalyzer(occupancy_grid)

    # Extract positions
    positions = [node['position'] for node in explored_nodes]

    # Compute coverage
    coverage_map = analyzer.compute_coverage_map(positions, r_view)
    free_space_mask = occupancy_grid.get_free_space_mask()
    coverage_ratio = analyzer.compute_coverage_ratio(positions, r_view)

    # Create figure
    fig, axes = plt.subplots(1, 3, figsize=figsize, dpi=dpi)

    extent = [
        occupancy_grid.origin[0],
        occupancy_grid.origin[0] + occupancy_grid.world_width,
        occupancy_grid.origin[1],
        occupancy_grid.origin[1] + occupancy_grid.world_height
    ]

    # Plot 1: Occupancy map with nodes
    ax = axes[0]
    ax.imshow(occupancy_grid.data, cmap='gray_r', origin='lower', extent=extent)
    x_coords = [pos[0] for pos in positions]
    y_coords = [pos[1] for pos in positions]
    ax.plot(x_coords, y_coords, 'ro', markersize=4)
    ax.set_title('Explored Nodes')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.axis('equal')

    # Plot 2: Coverage map
    ax = axes[1]
    ax.imshow(free_space_mask, cmap='gray', origin='lower',
              extent=extent, alpha=0.3)
    ax.imshow(coverage_map, cmap='Greens', origin='lower',
              extent=extent, alpha=0.7)
    ax.set_title(f'Coverage Map ({coverage_ratio:.1%})')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.axis('equal')

    # Plot 3: Uncovered regions
    ax = axes[2]
    uncovered = free_space_mask & (~coverage_map)
    ax.imshow(free_space_mask, cmap='gray', origin='lower',
              extent=extent, alpha=0.3)
    ax.imshow(uncovered, cmap='Reds', origin='lower',
              extent=extent, alpha=0.7)

    # Find and plot gaps
    gaps = analyzer.find_coverage_gaps(positions, r_view, min_gap_size=0.3)
    if gaps:
        gap_x = [g[0] for g in gaps]
        gap_y = [g[1] for g in gaps]
        ax.plot(gap_x, gap_y, 'bx', markersize=10, markeredgewidth=2)

    ax.set_title(f'Coverage Gaps ({len(gaps)} gaps)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.axis('equal')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=dpi, bbox_inches='tight')
        print(f"Coverage analysis saved to {save_path}")
        plt.show()  # Also display the figure
    else:
        plt.show()

    plt.close()


def plot_exploration_metrics(
    results_dict: dict,
    save_path: Optional[str] = None,
    figsize: Tuple[int, int] = (12, 8)
):
    """
    Plot exploration metrics.

    Args:
        results_dict: Dictionary with exploration results.
        save_path: Path to save figure.
        figsize: Figure size.
    """
    metadata = results_dict['metadata']

    fig, axes = plt.subplots(2, 2, figsize=figsize)

    # Metrics to display
    metrics = [
        ('Coverage Ratio', metadata['coverage_ratio'], '%'),
        ('Number of Nodes', metadata['num_nodes'], ''),
        ('Total Distance', metadata['total_distance'], 'm'),
        ('Exploration Time', metadata['total_time'], 's'),
        ('Min Node Distance', metadata['min_node_distance'], 'm'),
        ('Mean Node Distance', metadata['mean_node_distance'], 'm'),
        ('Coverage Uniformity', metadata['coverage_uniformity'], '')
    ]

    # Create text summary
    ax = axes[0, 0]
    ax.axis('off')
    text_lines = []
    for name, value, unit in metrics:
        if unit == '%':
            text_lines.append(f"{name}: {value:.1%}")
        else:
            text_lines.append(f"{name}: {value:.2f} {unit}")

    ax.text(0.1, 0.5, '\n'.join(text_lines),
            fontsize=12, verticalalignment='center',
            family='monospace')
    ax.set_title('Exploration Metrics', fontsize=14, fontweight='bold')

    # Bar chart: Key metrics
    ax = axes[0, 1]
    metrics_to_plot = [
        ('Coverage', metadata['coverage_ratio'] * 100),
        ('Nodes', metadata['num_nodes']),
    ]
    names = [m[0] for m in metrics_to_plot]
    values = [m[1] for m in metrics_to_plot]

    bars = ax.bar(names, values, color=['green', 'blue'])
    ax.set_ylabel('Value')
    ax.set_title('Key Metrics')
    ax.grid(True, alpha=0.3, axis='y')

    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}',
                ha='center', va='bottom')

    # Distance metrics
    ax = axes[1, 0]
    distance_metrics = [
        ('Total\nDistance', metadata['total_distance']),
        ('Min Node\nDistance', metadata['min_node_distance']),
        ('Mean Node\nDistance', metadata['mean_node_distance'])
    ]
    names = [m[0] for m in distance_metrics]
    values = [m[1] for m in distance_metrics]

    ax.bar(names, values, color=['orange', 'red', 'purple'])
    ax.set_ylabel('Distance (m)')
    ax.set_title('Distance Metrics')
    ax.grid(True, alpha=0.3, axis='y')

    # Parameters
    ax = axes[1, 1]
    ax.axis('off')
    param_text = [
        f"r_view: {metadata['r_view']:.2f} m",
        f"overlap: {metadata['overlap']:.2f} m",
        f"d_theta: {metadata['d_theta']:.1f}°"
    ]
    ax.text(0.1, 0.5, '\n'.join(param_text),
            fontsize=12, verticalalignment='center',
            family='monospace')
    ax.set_title('Parameters', fontsize=14, fontweight='bold')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=100, bbox_inches='tight')
        print(f"Metrics plot saved to {save_path}")
        plt.show()  # Also display the figure
    else:
        plt.show()

    plt.close()


def create_exploration_animation(
    occupancy_grid: OccupancyGrid,
    explored_nodes: List[dict],
    r_view: float,
    save_path: Optional[str] = None,
    fps: int = 5,
    figsize: Tuple[int, int] = (10, 10)
):
    """
    Create animation of exploration process.

    Args:
        occupancy_grid: Occupancy grid map.
        explored_nodes: List of explored nodes (in order).
        r_view: View radius.
        save_path: Path to save animation (MP4 or GIF).
        fps: Frames per second.
        figsize: Figure size.
    """
    fig, ax = plt.subplots(figsize=figsize)

    # Setup map
    extent = [
        occupancy_grid.origin[0],
        occupancy_grid.origin[0] + occupancy_grid.world_width,
        occupancy_grid.origin[1],
        occupancy_grid.origin[1] + occupancy_grid.world_height
    ]

    ax.imshow(occupancy_grid.data, cmap='gray_r', origin='lower',
              extent=extent, alpha=0.7)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('SSTG Exploration Animation')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    # Animation elements
    coverage_circles = []
    node_points, = ax.plot([], [], 'ro', markersize=6)
    path_line, = ax.plot([], [], 'b-', alpha=0.3, linewidth=1)
    current_node, = ax.plot([], [], 'g*', markersize=15)

    def init():
        node_points.set_data([], [])
        path_line.set_data([], [])
        current_node.set_data([], [])
        return [node_points, path_line, current_node]

    def animate(frame):
        # Clear old circles
        for circle in coverage_circles:
            circle.remove()
        coverage_circles.clear()

        # Get nodes up to current frame
        current_nodes = explored_nodes[:frame+1]
        positions = [node['position'] for node in current_nodes]

        # Draw coverage circles
        for pos in positions:
            circle = Circle(pos, r_view, color='lightblue',
                          alpha=0.2, zorder=1)
            ax.add_patch(circle)
            coverage_circles.append(circle)

        # Update node positions
        if positions:
            x_coords = [pos[0] for pos in positions]
            y_coords = [pos[1] for pos in positions]
            node_points.set_data(x_coords, y_coords)
            path_line.set_data(x_coords, y_coords)
            current_node.set_data([x_coords[-1]], [y_coords[-1]])

        ax.set_title(f'SSTG Exploration (Node {frame+1}/{len(explored_nodes)})')

        return [node_points, path_line, current_node] + coverage_circles

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=len(explored_nodes),
        interval=1000//fps, blit=True
    )

    if save_path:
        if save_path.endswith('.gif'):
            anim.save(save_path, writer='pillow', fps=fps)
        else:
            anim.save(save_path, writer='ffmpeg', fps=fps)
        print(f"Animation saved to {save_path}")
    else:
        plt.show()

    plt.close()
