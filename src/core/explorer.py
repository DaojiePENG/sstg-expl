"""
SSTG Explorer - Main exploration algorithm.
"""
import time
from typing import Tuple, List, Dict, Optional
import numpy as np

from src.config import ExplorerConfig
from src.map.occupancy_grid import OccupancyGrid
from src.core.frontier import FrontierQueue
from src.core.collision_checker import CollisionChecker, CollisionType
from src.core.coverage_analyzer import CoverageAnalyzer
from src.utils.geometry import (
    compute_target_point,
    euclidean_distance,
    find_longest_free_sector
)


class SSTGExplorer:
    """
    Spatial Semantic Topological Graph Explorer.

    Main exploration algorithm that maintains a global frontier queue
    and adaptively explores the environment.
    """

    def __init__(
        self,
        r_view: float = 2.0,
        d_theta: float = 45.0,
        overlap: float = 0.25,
        r_robot: float = 0.3,
        config: Optional[ExplorerConfig] = None
    ):
        """
        Initialize SSTG Explorer.

        Args:
            r_view: View radius in meters.
            d_theta: Angular interval in degrees.
            overlap: Overlap distance in meters.
            r_robot: Robot radius in meters.
            config: Optional configuration object (overrides individual params).
        """
        if config is not None:
            self.config = config
        else:
            self.config = ExplorerConfig(
                r_view=r_view,
                d_theta=d_theta,
                overlap=overlap,
                r_robot=r_robot
            )

        # Derived parameters
        self.d_repel = self.config.d_repel
        self.n_directions = self.config.n_directions

        # State variables (initialized in explore())
        self.explored_nodes: List[Tuple[float, float]] = []
        self.frontier_queue: Optional[FrontierQueue] = None
        self.collision_checker: Optional[CollisionChecker] = None
        self.coverage_analyzer: Optional[CoverageAnalyzer] = None
        self.current_pose: Optional[Tuple[float, float]] = None

        # Tracking for visualization
        self.blocked_obstacle_points: List[Tuple[float, float]] = []
        self.blocked_explored_points: List[Tuple[float, float]] = []

        # Statistics
        self.total_distance = 0.0
        self.iteration_count = 0
        self.start_time = 0.0

    def explore(
        self,
        occupancy_grid: OccupancyGrid,
        start_pose: Tuple[float, float, float],
        visualizer=None
    ) -> Dict:
        """
        Main exploration loop.

        Args:
            occupancy_grid: Occupancy grid map.
            start_pose: Starting pose (x, y, theta) in meters and degrees.
            visualizer: Optional RealtimeVisualizer for live visualization.

        Returns:
            Dictionary containing exploration results:
            {
                'nodes': List of explored nodes,
                'metadata': Dictionary of statistics,
                'success': Boolean indicating completion
            }
        """
        # Initialize
        self.start_time = time.time()
        self.explored_nodes = [(start_pose[0], start_pose[1])]
        self.current_pose = (start_pose[0], start_pose[1])
        self.total_distance = 0.0
        self.iteration_count = 0
        self.blocked_obstacle_points = []
        self.blocked_explored_points = []

        # Create helper objects
        self.frontier_queue = FrontierQueue()
        self.collision_checker = CollisionChecker(
            occupancy_grid,
            self.config.r_robot,
            self.config.d_safe,
            self.config.obstacle_threshold
        )
        self.coverage_analyzer = CoverageAnalyzer(occupancy_grid)

        # Generate initial frontiers
        if self.config.verbose:
            print(f"Starting exploration from {start_pose}")
            print(f"Parameters: r_view={self.config.r_view}m, "
                  f"d_theta={self.config.d_theta}°, "
                  f"overlap={self.config.overlap}m")

        self._generate_frontiers(self.current_pose)

        # Initial visualization
        if visualizer is not None:
            active_frontiers = self.frontier_queue.get_all_frontiers()
            coverage = self._get_current_coverage()
            visualizer.update(
                current_position=self.current_pose,
                explored_nodes=self.explored_nodes.copy(),
                active_frontiers=active_frontiers,
                blocked_obstacle=self.blocked_obstacle_points.copy(),
                blocked_explored=self.blocked_explored_points.copy(),
                iteration=0,
                coverage_ratio=coverage
            )

        # Main exploration loop
        while not self._should_terminate():
            # Get best frontier
            best_frontier = self.frontier_queue.pop()

            if best_frontier is None:
                if self.config.verbose:
                    print("No more frontiers available")
                break

            target = best_frontier.target

            # Check if still valid (not covered by recent explorations)
            if self._is_covered_by_explored(target):
                continue

            # Check path feasibility
            if not self.collision_checker.check_path(self.current_pose, target):
                continue

            # Navigate to target
            travel_dist = euclidean_distance(self.current_pose, target)
            self.total_distance += travel_dist
            self.current_pose = target
            self.explored_nodes.append(self.current_pose)
            self.iteration_count += 1

            # Generate new frontiers from current pose
            self._generate_frontiers(self.current_pose)

            # Update priorities of all existing frontiers
            self._update_all_priorities()

            # Update visualization
            if visualizer is not None:
                active_frontiers = self.frontier_queue.get_all_frontiers()
                coverage = self._get_current_coverage()
                visualizer.update(
                    current_position=self.current_pose,
                    explored_nodes=self.explored_nodes.copy(),
                    active_frontiers=active_frontiers,
                    blocked_obstacle=self.blocked_obstacle_points.copy(),
                    blocked_explored=self.blocked_explored_points.copy(),
                    iteration=self.iteration_count,
                    coverage_ratio=coverage
                )

            # Progress logging
            if self.config.verbose and self.iteration_count % 10 == 0:
                coverage = self._get_current_coverage()
                print(f"Iteration {self.iteration_count}: "
                      f"{len(self.explored_nodes)} nodes, "
                      f"coverage={coverage:.1%}, "
                      f"frontiers={self.frontier_queue.size()}")

        # Finalize and compute statistics
        elapsed_time = time.time() - self.start_time
        coverage_stats = self.coverage_analyzer.compute_statistics(
            self.explored_nodes,
            self.config.r_view,
            self.d_repel
        )

        if self.config.verbose:
            print(f"\nExploration complete!")
            print(f"Nodes: {len(self.explored_nodes)}")
            print(f"Coverage: {coverage_stats.coverage_ratio:.1%}")
            print(f"Distance: {self.total_distance:.2f}m")
            print(f"Time: {elapsed_time:.2f}s")

        # Finalize visualization
        if visualizer is not None:
            visualizer.finalize()

        # Return results
        return {
            'nodes': [
                {
                    'id': i,
                    'position': node,
                    'orientation': 0.0,  # Can be extended
                    'timestamp': i
                }
                for i, node in enumerate(self.explored_nodes)
            ],
            'metadata': {
                'r_view': self.config.r_view,
                'overlap': self.config.overlap,
                'd_theta': self.config.d_theta,
                'coverage_ratio': coverage_stats.coverage_ratio,
                'total_distance': self.total_distance,
                'total_time': elapsed_time,
                'num_nodes': len(self.explored_nodes),
                'min_node_distance': coverage_stats.min_node_distance,
                'mean_node_distance': coverage_stats.mean_node_distance,
                'coverage_uniformity': coverage_stats.coverage_uniformity
            },
            'success': coverage_stats.coverage_ratio >= self.config.target_coverage
        }

    def _generate_frontiers(self, position: Tuple[float, float]):
        """
        Generate frontiers for a given position.

        Args:
            position: Position to generate frontiers from (x, y).
        """
        # Clear blocked points tracking for this position's frontiers
        # Note: We accumulate blocked points across all positions
        temp_blocked_obstacle = []
        temp_blocked_explored = []

        # Generate candidate targets at each angle
        for angle_idx in range(self.n_directions):
            angle = angle_idx * self.config.d_theta

            # Compute target point
            target = compute_target_point(position, self.config.r_view, angle)

            # Check collision type
            collision_type, strength = self.collision_checker.check_collision_type(
                target, self.explored_nodes, self.d_repel
            )

            # Track blocked frontiers
            if collision_type == CollisionType.HARD_OBSTACLE:
                temp_blocked_obstacle.append(target)
                continue
            elif collision_type == CollisionType.SOFT_OBSTACLE:
                # This is a frontier but with reduced priority
                temp_blocked_explored.append(target)

            # Compute priority
            priority = self._compute_priority(
                position, target, strength
            )

            # Add to queue
            if priority > 0:
                self.frontier_queue.add(position, angle, target, priority)

        # Update accumulated blocked points (keep last N positions worth)
        max_tracked = 200  # Limit to avoid memory issues
        self.blocked_obstacle_points.extend(temp_blocked_obstacle)
        self.blocked_explored_points.extend(temp_blocked_explored)

        # Trim if too many
        if len(self.blocked_obstacle_points) > max_tracked:
            self.blocked_obstacle_points = self.blocked_obstacle_points[-max_tracked:]
        if len(self.blocked_explored_points) > max_tracked:
            self.blocked_explored_points = self.blocked_explored_points[-max_tracked:]

    def _compute_priority(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Compute priority for a frontier.

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value (higher = more important).
        """
        # Base score from exploration strength
        base_score = exploration_strength

        # Distance weight (favor nearby targets initially)
        distance = euclidean_distance(self.current_pose, target)
        coverage_ratio = self._get_current_coverage()
        alpha = self.config.get_alpha(coverage_ratio)

        distance_weight = 1.0 / (1.0 + (distance / self.config.r_view) ** alpha)

        # Combine factors
        priority = base_score * distance_weight

        return priority

    def _update_all_priorities(self):
        """Update priorities of all frontiers in the queue."""
        # Get all frontiers
        frontiers = self.frontier_queue.get_all_frontiers()

        for frontier in frontiers:
            # Recheck collision type (may have changed due to new explored nodes)
            collision_type, strength = self.collision_checker.check_collision_type(
                frontier.target, self.explored_nodes, self.d_repel
            )

            # Remove if now hitting hard obstacle
            if collision_type == CollisionType.HARD_OBSTACLE:
                self.frontier_queue.remove(frontier.frontier_id)
                continue

            # Recompute priority
            new_priority = self._compute_priority(
                frontier.position, frontier.target, strength
            )

            # Update priority
            self.frontier_queue.update_priority(frontier.frontier_id, new_priority)

    def _is_covered_by_explored(self, point: Tuple[float, float]) -> bool:
        """
        Check if a point is already covered by explored nodes.

        Args:
            point: Point to check (x, y).

        Returns:
            True if point is within d_repel of any explored node.
        """
        for explored in self.explored_nodes:
            if euclidean_distance(point, explored) < self.d_repel:
                return True
        return False

    def _get_current_coverage(self) -> float:
        """
        Get current coverage ratio.

        Returns:
            Coverage ratio [0, 1].
        """
        return self.coverage_analyzer.compute_coverage_ratio(
            self.explored_nodes,
            self.config.r_view
        )

    def _should_terminate(self) -> bool:
        """
        Check termination conditions.

        Returns:
            True if exploration should terminate.
        """
        # Max iterations reached
        if self.iteration_count >= self.config.max_iterations:
            if self.config.verbose:
                print("Max iterations reached")
            return True

        # No more frontiers
        if self.frontier_queue.is_empty():
            return True

        # Max priority too low
        max_priority = self.frontier_queue.max_priority()
        if max_priority is not None and max_priority < self.config.min_priority_threshold:
            if self.config.verbose:
                print(f"Max priority {max_priority:.3f} below threshold")
            return True

        # Coverage target reached and no high-priority frontiers remain
        coverage = self._get_current_coverage()
        if coverage >= self.config.target_coverage:
            if max_priority is not None and max_priority < 0.5:
                if self.config.verbose:
                    print(f"Coverage target reached: {coverage:.1%}")
                return True

        return False

    def get_explored_nodes(self) -> List[Tuple[float, float]]:
        """Get list of explored node positions."""
        return self.explored_nodes.copy()

    def get_statistics(self) -> Dict:
        """Get current exploration statistics."""
        if self.coverage_analyzer is None:
            return {}

        coverage_stats = self.coverage_analyzer.compute_statistics(
            self.explored_nodes,
            self.config.r_view,
            self.d_repel
        )

        return {
            'num_nodes': len(self.explored_nodes),
            'coverage_ratio': coverage_stats.coverage_ratio,
            'total_distance': self.total_distance,
            'iteration_count': self.iteration_count,
            'frontiers_remaining': self.frontier_queue.size() if self.frontier_queue else 0
        }

    def __repr__(self) -> str:
        return (
            f"SSTGExplorer(r_view={self.config.r_view}m, "
            f"d_theta={self.config.d_theta}°, "
            f"overlap={self.config.overlap}m)"
        )
