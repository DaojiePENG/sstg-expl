"""
Visualization utilities for SSTG Explorer.
"""
import numpy as np
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
from collections import Counter
from matplotlib.lines import Line2D

from sstg_explorer.map.occupancy_grid import OccupancyGrid


def visualize_exploration_step(
    occupancy_grid: OccupancyGrid,
    step: dict,
    r_view: float,
    save_path: Optional[str] = None,
    figsize: Tuple[int, int] = (11, 7),
    dpi: int = 120,
    title: Optional[str] = None,
):
    """Render one complete SSTG decision state for analysis and publication.

    The view distinguishes the explored trajectory, current pose, all active
    frontiers, candidates newly generated in this decision, their rejection
    reasons, the selected target, and its collision-free A* path.
    """
    from sstg_explorer.core.coverage_analyzer import CoverageAnalyzer

    fig, (ax, info_ax) = plt.subplots(
        1, 2, figsize=figsize, dpi=dpi,
        gridspec_kw={'width_ratios': [4.2, 1.35]},
    )
    extent = [
        occupancy_grid.origin[0],
        occupancy_grid.origin[0] + occupancy_grid.world_width,
        occupancy_grid.origin[1],
        occupancy_grid.origin[1] + occupancy_grid.world_height,
    ]
    ax.imshow(occupancy_grid.data, cmap='gray_r', origin='lower', extent=extent, alpha=0.9)

    explored = [tuple(point) for point in step.get('explored_nodes', [])]
    if explored:
        coverage = CoverageAnalyzer(occupancy_grid).compute_coverage_map(explored, r_view)
        overlay = np.ma.masked_where(~coverage, coverage)
        ax.imshow(overlay, cmap='Blues', origin='lower', extent=extent,
                  alpha=0.18, vmin=0, vmax=1)
        ax.scatter([p[0] for p in explored], [p[1] for p in explored],
                   c='#1976d2', s=34, edgecolors='white', linewidths=0.6,
                   zorder=6, label='Explored viewpoints')

    executed_paths = step.get('executed_paths', [])
    for path_index, executed_path in enumerate(executed_paths):
        if len(executed_path) > 1:
            ax.plot(
                [p[0] for p in executed_path], [p[1] for p in executed_path],
                color='#1565c0', linewidth=2.2, alpha=0.82, zorder=4,
                label='Executed trajectory' if path_index == 0 else None,
            )

    active = step.get('active_frontiers', [])
    if active:
        points = [frontier['target'] for frontier in active]
        priorities = np.asarray([frontier.get('priority', 0.0) for frontier in active])
        sizes = 38 + 85 * priorities / max(float(np.max(priorities)), 1e-6)
        ax.scatter([p[0] for p in points], [p[1] for p in points],
                   c='#f9a825', marker='^', s=sizes, edgecolors='#5d4037',
                   linewidths=0.7, alpha=0.78, zorder=7,
                   label='Pending frontiers')

    style = {
        'added': ('#00c853', 'D'),
        'added_soft': ('#64dd17', 'D'),
        'recovery_added': ('#00b8d4', 'P'),
        'blocked_obstacle': ('#d50000', 'x'),
        'pruned_strength': ('#ff6d00', 'x'),
        'pruned_priority': ('#aa00ff', 'x'),
        'pruned_duplicate': ('#616161', 'x'),
        'pruned_nonpositive': ('#795548', 'x'),
        'recovery_unreachable': ('#212121', 'X'),
        'recovery_duplicate': ('#607d8b', 'x'),
    }
    candidates = step.get('generated_candidates', [])
    for candidate in candidates:
        target = candidate['target']
        origin = candidate.get('origin')
        status = candidate.get('status', 'generated')
        color, marker = style.get(status, ('#9e9e9e', '.'))
        if origin is not None:
            ax.plot([origin[0], target[0]], [origin[1], target[1]],
                    color=color, linewidth=0.55, alpha=0.28, zorder=2)
        ax.scatter([target[0]], [target[1]], c=color, marker=marker,
                   s=64 if 'added' in status else 46, linewidths=1.5,
                   zorder=9)

    selected = step.get('selected_frontier')
    if selected:
        target = selected['target']
        ax.scatter([target[0]], [target[1]], c='#e91e63', marker='*',
                   s=260, edgecolors='black', linewidths=0.9, zorder=12,
                   label='Selected frontier')

    path = step.get('path', [])
    if len(path) > 1:
        ax.plot([p[0] for p in path], [p[1] for p in path],
                color='#00acc1', linewidth=3.0, alpha=0.9, zorder=10,
                label='A* path')

    current = step.get('current_position')
    if current:
        ax.scatter([current[0]], [current[1]], c='#76ff03', marker='o',
                   s=72, edgecolors='#1b5e20', linewidths=1.2, zorder=13,
                   label='Current pose')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=0.18)
    ax.set_title(title or f"SSTG decision trace {step.get('trace_id', 0)}")
    handles, labels = ax.get_legend_handles_labels()
    handles.extend([
        Line2D([0], [0], marker='D', linestyle='None', markerfacecolor='#00c853',
               markeredgecolor='#00c853', label='New candidate'),
        Line2D([0], [0], marker='x', linestyle='None', color='#d50000',
               label='Rejected: obstacle'),
        Line2D([0], [0], marker='x', linestyle='None', color='#ff6d00',
               label='Pruned: low gain'),
        Line2D([0], [0], marker='x', linestyle='None', color='#aa00ff',
               label='Pruned: low priority'),
        Line2D([0], [0], marker='P', linestyle='None', color='#00b8d4',
               label='Recovery candidate'),
    ])
    labels.extend([
        'New candidate', 'Rejected: obstacle', 'Pruned: low gain',
        'Pruned: low priority', 'Recovery candidate',
    ])
    if handles:
        ax.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, -0.09),
                  ncol=3, fontsize=8, frameon=False)

    counts = Counter(candidate.get('status', 'generated') for candidate in candidates)
    selected_text = 'none'
    if selected:
        selected_text = (
            f"id={selected.get('frontier_id')}\n"
            f"type={selected.get('kind')}\n"
            f"priority={selected.get('priority', 0):.3f}"
        )
    lines = [
        'DECISION STATE',
        f"trace: {step.get('trace_id', 0)}",
        f"iteration: {step.get('iteration', 0)}",
        f"event: {step.get('event', '-')}",
        '',
        f"coverage: {step.get('coverage_before', 0):.1%}",
        f"       → {step.get('coverage_after', 0):.1%}",
        f"gain: {step.get('coverage_gain', 0):+.2%}",
        f"explored: {len(explored)}",
        f"pending: {step.get('queue_size', len(active))}",
        '',
        'SELECTED', selected_text,
        '', 'GENERATED',
    ]
    lines.extend(f"{key}: {value}" for key, value in sorted(counts.items()))
    info_ax.axis('off')
    info_ax.text(0.02, 0.98, '\n'.join(lines), va='top', ha='left',
                 family='monospace', fontsize=8.4,
                 bbox=dict(boxstyle='round,pad=0.7', facecolor='#fafafa',
                           edgecolor='#bdbdbd'))
    fig.subplots_adjust(bottom=0.16, wspace=0.05)
    if save_path:
        fig.savefig(save_path, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)


def visualize_exploration(
    occupancy_grid: OccupancyGrid,
    explored_nodes: List[dict],
    r_view: float,
    show_coverage: bool = True,
    show_connections: bool = False,
    execution_paths: Optional[List[List[Tuple[float, float]]]] = None,
    save_path: Optional[str] = None,
    figsize: Tuple[int, int] = (12, 10),
    dpi: int = 100,
    title: Optional[str] = None
):
    """
    Visualize exploration results.

    Args:
        occupancy_grid: Occupancy grid map.
        explored_nodes: List of explored node dictionaries.
        r_view: View radius in meters.
        show_coverage: Whether to show coverage circles.
        show_connections: Whether to show connections between sequential nodes.
        execution_paths: Actual collision-free path segments executed so far.
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
    if execution_paths:
        for path_index, path in enumerate(execution_paths):
            if len(path) > 1:
                ax.plot(
                    [point[0] for point in path], [point[1] for point in path],
                    color='#1565c0', alpha=0.8, linewidth=2.0, zorder=2,
                    label='Executed trajectory' if path_index == 0 else None,
                )

    # Optional straight viewpoint-order links are diagnostic only and are
    # deliberately distinct from the actual executed trajectory.
    if show_connections and len(positions) > 1:
        for i in range(len(positions) - 1):
            ax.plot(
                [positions[i][0], positions[i+1][0]],
                [positions[i][1], positions[i+1][1]],
                linestyle='--', color='#90a4ae', alpha=0.5,
                linewidth=1, zorder=2
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
    if title:
        ax.set_title(title)
    else:
        ax.set_title(f'SSTG Exploration Result ({len(positions)} nodes)')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=dpi, bbox_inches='tight')
        plt.close()
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
    from sstg_explorer.core.coverage_analyzer import CoverageAnalyzer

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
