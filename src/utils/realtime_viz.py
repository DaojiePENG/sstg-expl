"""
Real-time visualization for SSTG Explorer.

Provides interactive visualization of the exploration process,
showing frontiers, obstacles, and exploration progress in real-time.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation, PillowWriter
from typing import List, Tuple, Optional, Dict
import time
import os

from src.map.occupancy_grid import OccupancyGrid
from src.core.frontier import Frontier


class RealtimeVisualizer:
    """
    Real-time visualization of exploration process.

    Shows:
    - Current position (large green marker)
    - Explored nodes (small red circles)
    - Active frontiers (yellow circles)
    - Blocked frontiers - obstacles (red X)
    - Blocked frontiers - too close to explored (orange X)
    - Coverage circles (transparent blue)
    - Exploration path (blue line)
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        r_view: float,
        figsize: Tuple[int, int] = (12, 10),
        update_interval: float = 0.5,
        save_animation: bool = False,
        animation_path: Optional[str] = None
    ):
        """
        Initialize real-time visualizer.

        Args:
            occupancy_grid: Occupancy grid map.
            r_view: View radius in meters.
            figsize: Figure size.
            update_interval: Minimum time between updates (seconds).
            save_animation: Whether to save animation of exploration process.
            animation_path: Path to save animation (default: outputs/explorations/animation.gif).
        """
        self.grid = occupancy_grid
        self.r_view = r_view
        self.figsize = figsize
        self.update_interval = update_interval
        self.save_animation = save_animation
        self.animation_path = animation_path or 'outputs/animations/animation.gif'

        # Setup figure
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots(figsize=figsize)

        # Map extent
        self.extent = [
            occupancy_grid.origin[0],
            occupancy_grid.origin[0] + occupancy_grid.world_width,
            occupancy_grid.origin[1],
            occupancy_grid.origin[1] + occupancy_grid.world_height
        ]

        # Plot occupancy grid
        self.ax.imshow(
            occupancy_grid.data,
            cmap='gray_r',
            origin='lower',
            extent=self.extent,
            alpha=0.7
        )

        # Initialize plot elements
        self.explored_scatter = None
        self.current_marker = None
        self.path_line = None
        self.coverage_circles = []
        self.frontier_markers = {}

        # Data tracking
        self.explored_positions = []
        self.last_update_time = 0

        # Animation frame recording
        self.animation_frames = []
        self.frame_count = 0

        # Setup plot
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('SSTG Explorer - Real-time Exploration')
        self.ax.axis('equal')
        self.ax.grid(True, alpha=0.3)

        # Legend
        self._setup_legend()

        plt.tight_layout()
        plt.pause(0.1)

    def _setup_legend(self):
        """Setup legend for visualization."""
        from matplotlib.lines import Line2D

        legend_elements = [
            Line2D([0], [0], marker='*', color='w', label='Current Position',
                   markerfacecolor='lime', markersize=15, markeredgecolor='darkgreen'),
            Line2D([0], [0], marker='o', color='w', label='Explored Nodes',
                   markerfacecolor='red', markersize=8),
            Line2D([0], [0], marker='o', color='w', label='Active Frontiers',
                   markerfacecolor='yellow', markersize=8, markeredgecolor='orange'),
            Line2D([0], [0], marker='x', color='red', label='Blocked - Obstacle',
                   markersize=8, markeredgewidth=2),
            Line2D([0], [0], marker='x', color='orange', label='Blocked - Too Close',
                   markersize=8, markeredgewidth=2),
            Line2D([0], [0], color='lightblue', label='Coverage Circle',
                   linewidth=2, alpha=0.5)
        ]

        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=9)

    def update(
        self,
        current_position: Tuple[float, float],
        explored_nodes: List[Tuple[float, float]],
        active_frontiers: List[Frontier],
        blocked_obstacle: List[Tuple[float, float]],
        blocked_explored: List[Tuple[float, float]],
        iteration: int,
        coverage_ratio: float
    ):
        """
        Update visualization with current exploration state.

        Args:
            current_position: Current robot position (x, y).
            explored_nodes: List of all explored positions.
            active_frontiers: List of active frontiers.
            blocked_obstacle: Frontier targets blocked by obstacles.
            blocked_explored: Frontier targets too close to explored nodes.
            iteration: Current iteration number.
            coverage_ratio: Current coverage ratio.
        """
        # Throttle updates
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return
        self.last_update_time = current_time

        # Record frame data for animation
        if self.save_animation:
            frame_data = {
                'current_position': current_position,
                'explored_nodes': explored_nodes.copy(),
                'active_frontiers': [(f.target[0], f.target[1]) for f in active_frontiers],
                'blocked_obstacle': blocked_obstacle.copy(),
                'blocked_explored': blocked_explored.copy(),
                'iteration': iteration,
                'coverage_ratio': coverage_ratio
            }
            self.animation_frames.append(frame_data)
            self.frame_count += 1

        # Clear previous frontier markers
        for markers in self.frontier_markers.values():
            for marker in markers:
                marker.remove()
        self.frontier_markers = {}

        # Clear previous coverage circles
        for circle in self.coverage_circles:
            circle.remove()
        self.coverage_circles = []

        # Update explored nodes
        self.explored_positions = explored_nodes.copy()

        # Plot coverage circles for all explored nodes
        for pos in explored_nodes:
            circle = Circle(
                pos, self.r_view,
                color='lightblue',
                alpha=0.2,
                fill=True,
                zorder=1
            )
            self.ax.add_patch(circle)
            self.coverage_circles.append(circle)

        # Plot explored nodes
        if len(explored_nodes) > 0:
            ex = [p[0] for p in explored_nodes]
            ey = [p[1] for p in explored_nodes]

            if self.explored_scatter is not None:
                self.explored_scatter.remove()

            self.explored_scatter = self.ax.scatter(
                ex, ey,
                c='red',
                s=80,
                marker='o',
                edgecolors='darkred',
                linewidths=1.5,
                zorder=4,
                label='Explored Nodes'
            )

            # Add node numbers
            for i, (x, y) in enumerate(explored_nodes):
                bbox_props = dict(boxstyle='round,pad=0.3', facecolor='white',
                                 edgecolor='black', alpha=0.8, linewidth=0.5)
                self.ax.text(x, y, str(i),
                            fontsize=7,
                            color='darkred',
                            fontweight='bold' if i == 0 else 'normal',
                            ha='center',
                            va='center',
                            bbox=bbox_props,
                            zorder=6)

        # Plot path
        if len(explored_nodes) > 1:
            px = [p[0] for p in explored_nodes]
            py = [p[1] for p in explored_nodes]

            if self.path_line is not None:
                self.path_line.remove()

            self.path_line, = self.ax.plot(
                px, py,
                'b-',
                alpha=0.4,
                linewidth=2,
                zorder=2
            )

        # Plot current position
        if self.current_marker is not None:
            self.current_marker.remove()

        self.current_marker = self.ax.scatter(
            [current_position[0]], [current_position[1]],
            c='lime',
            s=400,
            marker='*',
            edgecolors='darkgreen',
            linewidths=2,
            zorder=10,
            label='Current'
        )

        # Plot active frontiers
        if active_frontiers:
            fx = [f.target[0] for f in active_frontiers]
            fy = [f.target[1] for f in active_frontiers]

            markers = self.ax.scatter(
                fx, fy,
                c='yellow',
                s=100,
                marker='o',
                edgecolors='orange',
                linewidths=1.5,
                alpha=0.7,
                zorder=5
            )
            self.frontier_markers['active'] = [markers]

        # Plot blocked frontiers - obstacles
        if blocked_obstacle:
            bx = [p[0] for p in blocked_obstacle]
            by = [p[1] for p in blocked_obstacle]

            markers = self.ax.scatter(
                bx, by,
                c='red',
                s=80,
                marker='x',
                linewidths=2,
                alpha=0.6,
                zorder=5
            )
            self.frontier_markers['blocked_obstacle'] = [markers]

        # Plot blocked frontiers - too close to explored
        if blocked_explored:
            bex = [p[0] for p in blocked_explored]
            bey = [p[1] for p in blocked_explored]

            markers = self.ax.scatter(
                bex, bey,
                c='orange',
                s=80,
                marker='x',
                linewidths=2,
                alpha=0.5,
                zorder=5
            )
            self.frontier_markers['blocked_explored'] = [markers]

        # Update title with stats
        self.ax.set_title(
            f'SSTG Explorer - Iteration {iteration} | '
            f'Nodes: {len(explored_nodes)} | '
            f'Coverage: {coverage_ratio:.1%} | '
            f'Frontiers: {len(active_frontiers)}'
        )

        # Refresh display
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def save(self, filepath: str):
        """
        Save current visualization to file.

        Args:
            filepath: Path to save figure.
        """
        self.fig.savefig(filepath, dpi=100, bbox_inches='tight')
        print(f"Real-time visualization saved to {filepath}")

    def close(self):
        """Close the visualization window."""
        plt.ioff()
        plt.close(self.fig)

    def finalize(self, filepath: Optional[str] = None):
        """
        Finalize visualization and optionally save.

        Args:
            filepath: Optional path to save final state.
        """
        self.ax.set_title(
            f'SSTG Explorer - Exploration Complete | '
            f'Total Nodes: {len(self.explored_positions)}'
        )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        if filepath:
            self.save(filepath)

        # Save animation if enabled
        if self.save_animation and len(self.animation_frames) > 0:
            print(f"\nGenerating animation from {len(self.animation_frames)} frames...")
            self._save_animation()

        # Keep window open
        plt.ioff()
        print("\nReal-time visualization complete. Close window to continue...")

    def _save_animation(self):
        """Generate and save animation from recorded frames."""
        # Create output directory
        os.makedirs(os.path.dirname(self.animation_path), exist_ok=True)

        # Create a new figure for animation
        anim_fig, anim_ax = plt.subplots(figsize=self.figsize)

        # Plot occupancy grid
        anim_ax.imshow(
            self.grid.data,
            cmap='gray_r',
            origin='lower',
            extent=self.extent,
            alpha=0.7
        )

        anim_ax.set_xlabel('X (m)')
        anim_ax.set_ylabel('Y (m)')
        anim_ax.axis('equal')
        anim_ax.grid(True, alpha=0.3)

        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='*', color='w', label='Current Position',
                   markerfacecolor='lime', markersize=15, markeredgecolor='darkgreen'),
            Line2D([0], [0], marker='o', color='w', label='Explored Nodes',
                   markerfacecolor='red', markersize=8),
            Line2D([0], [0], marker='o', color='w', label='Active Frontiers',
                   markerfacecolor='yellow', markersize=8, markeredgecolor='orange'),
            Line2D([0], [0], marker='x', color='red', label='Blocked - Obstacle',
                   markersize=8, markeredgewidth=2),
            Line2D([0], [0], marker='x', color='orange', label='Blocked - Too Close',
                   markersize=8, markeredgewidth=2),
            Line2D([0], [0], color='lightblue', label='Coverage Circle',
                   linewidth=2, alpha=0.5)
        ]
        anim_ax.legend(handles=legend_elements, loc='upper right', fontsize=9)

        # Animation update function
        def animate_frame(frame_idx):
            # Clear previous frame
            anim_ax.clear()

            # Replot grid
            anim_ax.imshow(
                self.grid.data,
                cmap='gray_r',
                origin='lower',
                extent=self.extent,
                alpha=0.7
            )

            frame = self.animation_frames[frame_idx]

            # Plot coverage circles
            for pos in frame['explored_nodes']:
                circle = Circle(
                    pos, self.r_view,
                    color='lightblue',
                    alpha=0.2,
                    fill=True,
                    zorder=1
                )
                anim_ax.add_patch(circle)

            # Plot explored nodes
            if len(frame['explored_nodes']) > 0:
                ex = [p[0] for p in frame['explored_nodes']]
                ey = [p[1] for p in frame['explored_nodes']]
                anim_ax.scatter(
                    ex, ey,
                    c='red',
                    s=80,
                    marker='o',
                    edgecolors='darkred',
                    linewidths=1.5,
                    zorder=4
                )

                # Add node numbers
                for i, (x, y) in enumerate(frame['explored_nodes']):
                    bbox_props = dict(boxstyle='round,pad=0.3', facecolor='white',
                                     edgecolor='black', alpha=0.8, linewidth=0.5)
                    anim_ax.text(x, y, str(i),
                                fontsize=7,
                                color='darkred',
                                fontweight='bold' if i == 0 else 'normal',
                                ha='center',
                                va='center',
                                bbox=bbox_props,
                                zorder=6)

            # Plot path
            if len(frame['explored_nodes']) > 1:
                px = [p[0] for p in frame['explored_nodes']]
                py = [p[1] for p in frame['explored_nodes']]
                anim_ax.plot(px, py, 'b-', alpha=0.4, linewidth=2, zorder=2)

            # Plot current position
            curr_pos = frame['current_position']
            anim_ax.scatter(
                [curr_pos[0]], [curr_pos[1]],
                c='lime',
                s=400,
                marker='*',
                edgecolors='darkgreen',
                linewidths=2,
                zorder=10
            )

            # Plot active frontiers
            if frame['active_frontiers']:
                fx = [p[0] for p in frame['active_frontiers']]
                fy = [p[1] for p in frame['active_frontiers']]
                anim_ax.scatter(
                    fx, fy,
                    c='yellow',
                    s=100,
                    marker='o',
                    edgecolors='orange',
                    linewidths=1.5,
                    alpha=0.7,
                    zorder=5
                )

            # Plot blocked frontiers - obstacles
            if frame['blocked_obstacle']:
                bx = [p[0] for p in frame['blocked_obstacle']]
                by = [p[1] for p in frame['blocked_obstacle']]
                anim_ax.scatter(
                    bx, by,
                    c='red',
                    s=80,
                    marker='x',
                    linewidths=2,
                    alpha=0.6,
                    zorder=5
                )

            # Plot blocked frontiers - too close
            if frame['blocked_explored']:
                bex = [p[0] for p in frame['blocked_explored']]
                bey = [p[1] for p in frame['blocked_explored']]
                anim_ax.scatter(
                    bex, bey,
                    c='orange',
                    s=80,
                    marker='x',
                    linewidths=2,
                    alpha=0.5,
                    zorder=5
                )

            # Update title
            anim_ax.set_title(
                f'SSTG Explorer - Iteration {frame["iteration"]} | '
                f'Nodes: {len(frame["explored_nodes"])} | '
                f'Coverage: {frame["coverage_ratio"]:.1%} | '
                f'Frontiers: {len(frame["active_frontiers"])}'
            )

            anim_ax.set_xlabel('X (m)')
            anim_ax.set_ylabel('Y (m)')
            anim_ax.axis('equal')
            anim_ax.grid(True, alpha=0.3)
            anim_ax.legend(handles=legend_elements, loc='upper right', fontsize=9)

        # Create animation
        anim = FuncAnimation(
            anim_fig,
            animate_frame,
            frames=len(self.animation_frames),
            interval=max(200, int(self.update_interval * 1000)),  # milliseconds
            repeat=True,
            blit=False
        )

        # Save as GIF
        writer = PillowWriter(fps=min(5, int(1.0 / self.update_interval)))
        anim.save(self.animation_path, writer=writer)

        plt.close(anim_fig)
        print(f"Animation saved to {self.animation_path}")
        print(f"  - {len(self.animation_frames)} frames")
        print(f"  - {writer.fps} fps")
        print(f"  - Duration: ~{len(self.animation_frames) / writer.fps:.1f} seconds")


class ExplorationLogger:
    """
    Logger for exploration process data.

    Tracks all exploration states for post-processing or replay.
    """

    def __init__(self):
        """Initialize exploration logger."""
        self.states = []
        self.start_time = time.time()

    def log_state(
        self,
        iteration: int,
        current_position: Tuple[float, float],
        explored_nodes: List[Tuple[float, float]],
        active_frontiers: List[Frontier],
        blocked_obstacle: List[Tuple[float, float]],
        blocked_explored: List[Tuple[float, float]],
        coverage_ratio: float
    ):
        """
        Log current exploration state.

        Args:
            iteration: Current iteration number.
            current_position: Current position.
            explored_nodes: All explored nodes.
            active_frontiers: Active frontiers.
            blocked_obstacle: Blocked by obstacles.
            blocked_explored: Blocked by explored nodes.
            coverage_ratio: Current coverage ratio.
        """
        state = {
            'iteration': iteration,
            'timestamp': time.time() - self.start_time,
            'current_position': current_position,
            'explored_nodes': explored_nodes.copy(),
            'num_active_frontiers': len(active_frontiers),
            'num_blocked_obstacle': len(blocked_obstacle),
            'num_blocked_explored': len(blocked_explored),
            'coverage_ratio': coverage_ratio
        }
        self.states.append(state)

    def get_states(self) -> List[Dict]:
        """Get all logged states."""
        return self.states

    def save_log(self, filepath: str):
        """
        Save exploration log to file.

        Args:
            filepath: Path to save log (JSON format).
        """
        import json

        with open(filepath, 'w') as f:
            json.dump(self.states, f, indent=2)

        print(f"Exploration log saved to {filepath}")
