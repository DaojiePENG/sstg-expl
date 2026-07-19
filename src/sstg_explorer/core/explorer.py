"""
SSTG Explorer - Main exploration algorithm.
"""
import time
from typing import Tuple, List, Dict, Optional
import numpy as np
from scipy.ndimage import distance_transform_edt, label

from sstg_explorer.config import ExplorerConfig, FrontierSelectionStrategy
from sstg_explorer.map.occupancy_grid import OccupancyGrid
from sstg_explorer.core.frontier import FrontierQueue, Frontier
from sstg_explorer.core.collision_checker import CollisionChecker, CollisionType
from sstg_explorer.core.coverage_analyzer import CoverageAnalyzer
from sstg_explorer.utils.geometry import (
    compute_target_point,
    euclidean_distance,
    find_longest_free_sector
)

# Optional imports for advanced features
try:
    from sstg_explorer.planning.astar import AStarPlanner
    ASTAR_AVAILABLE = True
except ImportError:
    ASTAR_AVAILABLE = False

try:
    from sstg_explorer.map.distance_field import DistanceField
    from sstg_explorer.core.narrow_passage import NarrowPassageDetector
    ADAPTIVE_SAMPLING_AVAILABLE = True
except ImportError:
    ADAPTIVE_SAMPLING_AVAILABLE = False


class SSTGExplorer:
    """
    Spatial Semantic Topological Graph Explorer.

    Main exploration algorithm that maintains a global frontier queue
    and adaptively explores the environment.
    """

    def __init__(
        self,
        r_view: float = 2.0,
        d_theta: float = 30.0,
        overlap: float = 0.25,
        r_robot: float = 0.3,
        frontier_strategy: FrontierSelectionStrategy = FrontierSelectionStrategy.ENHANCED_DISTANCE,
        config: Optional[ExplorerConfig] = None
    ):
        """
        Initialize SSTG Explorer.

        Args:
            r_view: View radius in meters.
            d_theta: Angular interval in degrees.
            overlap: Overlap distance in meters.
            r_robot: Robot radius in meters.
            frontier_strategy: Frontier selection strategy for ablation study.
            config: Optional configuration object (overrides individual params).
        """
        if config is not None:
            self.config = config
        else:
            self.config = ExplorerConfig(
                r_view=r_view,
                d_theta=d_theta,
                overlap=overlap,
                r_robot=r_robot,
                frontier_strategy=frontier_strategy
            )

        self.name = "SSTG-Explorer"

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

        # Performance optimization: cached coverage value
        self.cached_coverage = 0.0  # Updated once per iteration instead of per priority calculation

        # Phase 1 Optimization: Initialize adaptive priority threshold
        self.adaptive_min_priority = self.config.min_priority_threshold

        # Paper-grade trace: every decision state is returned by explore().
        self.step_history: List[Dict] = []
        self.recovery_rounds = 0
        self.recovery_targets: List[Tuple[float, float]] = []
        self.last_generation_events: List[Dict] = []
        self.termination_reason = "not_started"
        self.geodesic_cost_map = None
        self.obstacle_clearance_map = None
        self.executed_paths: List[List[Tuple[float, float]]] = []

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
        self.step_history = []
        self.recovery_rounds = 0
        self.recovery_targets = []
        self.last_generation_events = []
        self.termination_reason = "running"
        self.geodesic_cost_map = None
        self.executed_paths = []
        free_mask = occupancy_grid.data < self.config.obstacle_threshold
        self.obstacle_clearance_map = (
            distance_transform_edt(free_mask) * occupancy_grid.resolution
        )

        # Create helper objects
        self.frontier_queue = FrontierQueue()
        self.collision_checker = CollisionChecker(
            occupancy_grid,
            self.config.r_robot,
            self.config.d_safe,
            self.config.obstacle_threshold
        )
        self.coverage_analyzer = CoverageAnalyzer(occupancy_grid)

        # Create A* planner if enabled
        self.astar_planner = None
        if self.config.use_astar:
            if not ASTAR_AVAILABLE:
                print("Warning: A* requested but not available. Using direct paths.")
            else:
                self.astar_planner = AStarPlanner(
                    occupancy_grid,
                    self.config.r_robot,
                    self.config.d_safe,
                    self.config.obstacle_threshold
                )
                if self.config.verbose:
                    print("  Using A* path planning")

        # Create distance field and narrow passage detector if adaptive sampling enabled
        self.narrow_passage_detector = None
        if self.config.use_adaptive_sampling:
            if not ADAPTIVE_SAMPLING_AVAILABLE:
                print("Warning: Adaptive sampling requested but not available. Using fixed d_theta.")
            else:
                # Compute distance field
                distance_field = DistanceField(
                    occupancy_grid,
                    self.config.obstacle_threshold
                )
                self.narrow_passage_detector = NarrowPassageDetector(
                    occupancy_grid,
                    distance_field,
                    self.config.r_robot,
                    self.config.narrow_threshold,
                    self.config.d_theta,
                    self.config.min_d_theta
                )
                if self.config.verbose:
                    print(f"  Using adaptive sampling (narrow < {self.config.narrow_threshold}m)")

        # Environment density is needed by priority scoring, including the first
        # generation round.
        self.environment_density = self._compute_environment_density(occupancy_grid)
        self._refresh_geodesic_costs()

        # Generate initial frontiers
        if self.config.verbose:
            print(f"Starting exploration from {start_pose}")
            print(f"Parameters: r_view={self.config.r_view}m, "
                  f"d_theta={self.config.d_theta}°, "
                  f"overlap={self.config.overlap}m")

        initial_events = self._generate_frontiers(self.current_pose)
        initial_coverage = self._get_current_coverage()
        self._record_step(
            selected_frontier=None,
            path=[],
            generated_candidates=initial_events,
            coverage_before=0.0,
            coverage_after=initial_coverage,
            event="initialization",
        )

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

        # Compute environment density for adaptive thresholds
        if self.config.verbose and self.config.adaptive_threshold:
            print(f"  Environment density: {self.environment_density:.1%} occupied")

        # Adjust min_priority_threshold based on density
        self.adaptive_min_priority = self.config.min_priority_threshold
        if self.config.adaptive_threshold:
            # In dense environments, lower the threshold to allow exploration
            density_thresh = getattr(self.config, 'density_threshold', 0.20)
            if self.environment_density > density_thresh:
                # Lower threshold progressively with density
                density_factor = 1.0 + (self.environment_density - density_thresh) / 0.10
                self.adaptive_min_priority = self.config.min_priority_threshold / density_factor
                if self.config.verbose:
                    print(f"  Adjusted priority threshold: {self.adaptive_min_priority:.4f} (from {self.config.min_priority_threshold:.4f})")
            else:
                self.adaptive_min_priority = self.config.min_priority_threshold

        # Adaptive d_theta based on environment complexity
        if self.config.adaptive_dtheta:
            complexity_thresh = getattr(self.config, 'complexity_threshold', 0.12)
            if self.environment_density < complexity_thresh:
                # Simple environment: use larger d_theta (fewer directions, more efficient)
                self.config.d_theta = self.config.dtheta_simple
                if self.config.verbose:
                    print(f"  Adaptive d_theta: {self.config.d_theta}° (simple environment, density={self.environment_density:.1%})")
            else:
                # Complex environment: use smaller d_theta (more directions, better coverage)
                self.config.d_theta = self.config.dtheta_complex
                if self.config.verbose:
                    print(f"  Adaptive d_theta: {self.config.d_theta}° (complex environment, density={self.environment_density:.1%})")
        elif self.config.verbose:
            print(f"  Using fixed d_theta: {self.config.d_theta}°")

        # Main exploration loop
        while not self._should_terminate():
            # Get best frontier
            best_frontier = self.frontier_queue.pop()

            if best_frontier is None:
                if self.config.verbose:
                    print("No more frontiers available")
                self.termination_reason = "frontier_queue_empty"
                break

            target = best_frontier.target

            # Check if still valid (not covered by recent explorations)
            if self._is_covered_by_explored(target):
                self._record_step(
                    selected_frontier=best_frontier,
                    path=[], generated_candidates=[],
                    coverage_before=self._get_current_coverage(),
                    coverage_after=self._get_current_coverage(),
                    event="selected_frontier_already_covered",
                )
                continue

            # Check path feasibility and plan path
            if self.astar_planner is not None:
                # Use A* path planning
                path = self.astar_planner.plan(
                    self.current_pose,
                    target,
                    max_iterations=max(
                        self.config.astar_max_iterations,
                        occupancy_grid.data.size,
                    )
                )
                if path is None:
                    # No path found
                    self._record_step(
                        selected_frontier=best_frontier,
                        path=[], generated_candidates=[],
                        coverage_before=self._get_current_coverage(),
                        coverage_after=self._get_current_coverage(),
                        event="selected_frontier_unreachable",
                    )
                    continue
                # Compute actual path length
                travel_dist = self.astar_planner.get_path_length(path)
            else:
                # Use direct line check
                if not self.collision_checker.check_path(self.current_pose, target):
                    self._record_step(
                        selected_frontier=best_frontier,
                        path=[], generated_candidates=[],
                        coverage_before=self._get_current_coverage(),
                        coverage_after=self._get_current_coverage(),
                        event="selected_frontier_path_collision",
                    )
                    continue
                travel_dist = euclidean_distance(self.current_pose, target)
                path = [self.current_pose, target]

            # Navigate to target
            coverage_before = self._get_current_coverage()
            self.total_distance += travel_dist
            self.current_pose = target
            self.explored_nodes.append(self.current_pose)
            self.executed_paths.append(path)
            self.iteration_count += 1
            self._refresh_geodesic_costs()

            # Performance optimization: Update KD-tree for fast collision checking
            self.collision_checker.update_explored_tree(self.explored_nodes)

            # Performance optimization: Update coverage cache once per iteration
            # This avoids recomputing coverage for every priority calculation
            self.cached_coverage = self._get_current_coverage()

            # Generate new frontiers from current pose
            generated_events = self._generate_frontiers(self.current_pose)

            # Update priorities of all existing frontiers
            self._update_all_priorities()

            self._record_step(
                selected_frontier=best_frontier,
                path=path,
                generated_candidates=generated_events,
                coverage_before=coverage_before,
                coverage_after=self.cached_coverage,
                event="node_accepted",
            )

            # Update visualization
            if visualizer is not None:
                active_frontiers = self.frontier_queue.get_all_frontiers()
                # Use cached coverage for visualization
                visualizer.update(
                    current_position=self.current_pose,
                    explored_nodes=self.explored_nodes.copy(),
                    active_frontiers=active_frontiers,
                    blocked_obstacle=self.blocked_obstacle_points.copy(),
                    blocked_explored=self.blocked_explored_points.copy(),
                    iteration=self.iteration_count,
                    coverage_ratio=self.cached_coverage
                )

            # Progress logging
            if self.config.verbose and self.iteration_count % 10 == 0:
                # Use cached coverage for logging
                print(f"Iteration {self.iteration_count}: "
                      f"{len(self.explored_nodes)} nodes, "
                      f"coverage={self.cached_coverage:.1%}, "
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
                'coverage_uniformity': coverage_stats.coverage_uniformity,
                'recovery_rounds': self.recovery_rounds,
                'termination_reason': self.termination_reason,
                'paths': self.executed_paths,
            },
            'steps': self.step_history,
            'success': coverage_stats.coverage_ratio >= self.config.target_coverage
        }

    def _generate_frontiers(self, position: Tuple[float, float]) -> List[Dict]:
        """
        Generate frontiers for a given position.

        Uses adaptive angular sampling if enabled, otherwise uses fixed d_theta.
        Phase 1 Optimization: Aggressive pruning during generation.

        Args:
            position: Position to generate frontiers from (x, y).
        """
        # Phase 1 Optimization: Prune covered frontiers periodically
        if (self.config.enable_aggressive_pruning and
            self.iteration_count % self.config.frontier_prune_interval == 0):
            self._prune_covered_frontiers()

        # Clear blocked points tracking for this position's frontiers
        # Note: We accumulate blocked points across all positions
        temp_blocked_obstacle = []
        temp_blocked_explored = []

        # Determine angles to use (adaptive or fixed)
        if self.narrow_passage_detector is not None:
            # Use adaptive sampling based on passage characteristics
            angles = self.narrow_passage_detector.get_adaptive_directions(
                position,
                base_d_theta=self.config.d_theta
            )
        else:
            # Use fixed angular sampling
            angles = [angle_idx * self.config.d_theta for angle_idx in range(self.n_directions)]

        # Phase 1 Optimization: Count how many frontiers we add/skip
        added_count = 0
        skipped_strength = 0
        skipped_distance = 0
        skipped_priority = 0
        events: List[Dict] = []

        # Generate candidate targets at each angle
        for angle in angles:
            # Compute target point
            target = compute_target_point(position, self.config.r_view, angle)
            event = {
                'target': target,
                'origin': position,
                'angle': float(angle),
                'kind': 'angular',
                'status': 'generated',
            }

            # Check collision type (FIXED: now passes r_view for correct coverage check)
            collision_type, strength = self.collision_checker.check_collision_type(
                target, self.explored_nodes, self.d_repel, self.config.r_view
            )
            event['collision_type'] = collision_type.name.lower()
            event['strength'] = float(strength)

            # Track blocked frontiers
            if collision_type == CollisionType.HARD_OBSTACLE:
                temp_blocked_obstacle.append(target)
                event['status'] = 'blocked_obstacle'
                events.append(event)
                continue
            elif collision_type == CollisionType.SOFT_OBSTACLE:
                # This is a frontier but with reduced priority
                temp_blocked_explored.append(target)

            # Phase 1 Optimization: Strength pruning
            if self.config.enable_aggressive_pruning:
                if strength < self.config.frontier_min_strength:
                    skipped_strength += 1
                    event['status'] = 'pruned_strength'
                    events.append(event)
                    continue

            # Compute priority
            priority = self._compute_priority(
                position, target, strength
            )
            event['priority'] = float(priority)
            event['obstacle_clearance'] = float(self._clearance_at(target))

            # Phase 1 Optimization: Priority pruning
            if self.config.enable_aggressive_pruning:
                min_acceptable_priority = self.adaptive_min_priority * self.config.frontier_priority_factor
                if priority < min_acceptable_priority:
                    skipped_priority += 1
                    event['status'] = 'pruned_priority'
                    events.append(event)
                    continue

            # Phase 1 Optimization: Distance pruning (check if too close to existing frontiers)
            if self.config.enable_aggressive_pruning:
                if self._too_close_to_existing_frontier(target, self.config.frontier_min_distance):
                    skipped_distance += 1
                    event['status'] = 'pruned_duplicate'
                    events.append(event)
                    continue

            # Add to queue
            if priority > 0:
                frontier_id = self.frontier_queue.add(position, angle, target, priority)
                event['frontier_id'] = frontier_id
                event['status'] = 'added_soft' if collision_type == CollisionType.SOFT_OBSTACLE else 'added'
                added_count += 1
            else:
                event['status'] = 'pruned_nonpositive'
            events.append(event)

        # Debug logging for pruning effectiveness
        if self.config.verbose and self.config.enable_aggressive_pruning and self.iteration_count % 5 == 0:
            total_skipped = skipped_strength + skipped_distance + skipped_priority
            if total_skipped > 0 or added_count > 0:
                print(f"  [Pruning] Added: {added_count}, Skipped: {total_skipped} "
                      f"(str:{skipped_strength}, dist:{skipped_distance}, pri:{skipped_priority})")

        # Update accumulated blocked points (keep last N positions worth)
        max_tracked = 200  # Limit to avoid memory issues
        self.blocked_obstacle_points.extend(temp_blocked_obstacle)
        self.blocked_explored_points.extend(temp_blocked_explored)

        # Trim if too many
        if len(self.blocked_obstacle_points) > max_tracked:
            self.blocked_obstacle_points = self.blocked_obstacle_points[-max_tracked:]
        if len(self.blocked_explored_points) > max_tracked:
            self.blocked_explored_points = self.blocked_explored_points[-max_tracked:]

        self.last_generation_events = events
        return events

    @staticmethod
    def _frontier_to_dict(frontier: Optional[Frontier]) -> Optional[Dict]:
        """Convert a frontier to a stable, JSON-serializable trace record."""
        if frontier is None:
            return None
        return {
            'frontier_id': int(frontier.frontier_id),
            'origin': frontier.position,
            'target': frontier.target,
            'angle': float(frontier.angle),
            'priority': float(frontier.priority),
            'kind': frontier.kind,
        }

    def _record_step(
        self,
        selected_frontier: Optional[Frontier],
        path: List[Tuple[float, float]],
        generated_candidates: List[Dict],
        coverage_before: float,
        coverage_after: float,
        event: str,
    ) -> None:
        """Capture the complete decision state used by paper visualizations."""
        active = sorted(
            self.frontier_queue.get_all_frontiers(),
            key=lambda frontier: frontier.priority,
            reverse=True,
        )
        self.step_history.append({
            'trace_id': len(self.step_history),
            'iteration': int(self.iteration_count),
            'event': event,
            'current_position': self.current_pose,
            'selected_frontier': self._frontier_to_dict(selected_frontier),
            'path': path,
            'executed_paths': [list(segment) for segment in self.executed_paths],
            'explored_nodes': list(self.explored_nodes),
            'generated_candidates': generated_candidates,
            'new_frontiers': [
                candidate for candidate in generated_candidates
                if candidate.get('status') in ('added', 'added_soft', 'recovery_added')
            ],
            'active_frontiers': [self._frontier_to_dict(frontier) for frontier in active],
            'coverage_before': float(coverage_before),
            'coverage_after': float(coverage_after),
            'coverage_gain': float(coverage_after - coverage_before),
            'queue_size': len(active),
            'recovery_round': int(self.recovery_rounds),
        })

    def _inject_recovery_frontiers(self) -> List[Dict]:
        """Seed reachable viewpoints in large uncovered free-space regions.

        Local angular expansion can be cut by doors or dense clutter. This
        recovery layer finds maxima of the uncovered-space distance transform,
        filters them through the inflated collision map, validates reachability
        with A*, and ranks them by marginal coverage, clearance, and path cost.
        """
        if not self.config.enable_global_recovery:
            return []
        if self.recovery_rounds >= self.config.recovery_max_rounds:
            return []
        coverage = self._get_current_coverage()
        target = min(self.config.target_coverage, self.config.recovery_min_coverage)
        if coverage >= target:
            return []
        if self.astar_planner is None:
            return []

        coverage_map = self.coverage_analyzer.compute_coverage_map(
            self.explored_nodes, self.config.r_view
        )
        free = self.coverage_analyzer.free_space_mask
        uncovered = free & (~coverage_map)
        safe = self.collision_checker.inflated_grid.get_free_space_mask(
            self.config.obstacle_threshold
        )

        # Ignore tiny speckles that cannot materially change coverage.
        components, count = label(uncovered)
        min_cells = max(
            1,
            int(self.config.recovery_min_gap_area / self.coverage_analyzer.grid.resolution ** 2),
        )
        meaningful = np.zeros_like(uncovered, dtype=bool)
        for component_id in range(1, count + 1):
            component = components == component_id
            if int(np.sum(component)) >= min_cells:
                meaningful |= component
        available = meaningful & safe
        if not np.any(available):
            return []

        grid = self.coverage_analyzer.grid
        resolution = grid.resolution
        gap_depth = distance_transform_edt(meaningful) * resolution
        obstacle_clearance = distance_transform_edt(safe) * resolution
        events: List[Dict] = []
        added = 0
        work = available.copy()
        suppress_cells = max(1, int(np.ceil(self.config.r_view / resolution)))

        for _ in range(self.config.recovery_max_candidates * 3):
            if added >= self.config.recovery_max_candidates or not np.any(work):
                break
            score_map = np.where(work, gap_depth, -1.0)
            row, col = np.unravel_index(int(np.argmax(score_map)), score_map.shape)
            point = grid.grid_to_world(int(row), int(col))
            event = {
                'target': point,
                'origin': self.current_pose,
                'angle': 0.0,
                'kind': 'global_recovery',
                'status': 'recovery_generated',
                'gap_depth': float(gap_depth[row, col]),
                'clearance': float(obstacle_clearance[row, col]),
            }

            # Suppress a neighborhood regardless of acceptance so candidates
            # describe distinct uncovered regions.
            r0, r1 = max(0, row - suppress_cells), min(work.shape[0], row + suppress_cells + 1)
            c0, c1 = max(0, col - suppress_cells), min(work.shape[1], col + suppress_cells + 1)
            yy, xx = np.ogrid[r0:r1, c0:c1]
            work[r0:r1, c0:c1][(yy - row) ** 2 + (xx - col) ** 2 <= suppress_cells ** 2] = False

            if any(euclidean_distance(point, old) < self.config.r_view for old in self.recovery_targets):
                event['status'] = 'recovery_duplicate'
                events.append(event)
                continue

            path = self.astar_planner.plan(
                self.current_pose, point,
                max_iterations=max(self.config.astar_max_iterations, grid.data.size),
            )
            if path is None:
                event['status'] = 'recovery_unreachable'
                events.append(event)
                continue

            path_cost = self.astar_planner.get_path_length(path)
            view_cells = max(1, int(np.pi * (self.config.r_view / resolution) ** 2))
            local_gain = int(np.sum(uncovered[r0:r1, c0:c1])) / view_cells
            normalized_cost = path_cost / max(self.config.r_view, 1e-6)
            priority = (
                self.config.recovery_gain_weight * min(local_gain, 1.0)
                + self.config.recovery_clearance_weight
                * min(obstacle_clearance[row, col] / self.config.r_view, 1.0)
                - self.config.recovery_cost_weight * min(normalized_cost / 10.0, 1.0)
            )
            priority = max(0.05, float(priority))
            frontier_id = self.frontier_queue.add(
                self.current_pose, 0.0, point, priority,
                kind='global_recovery',
            )
            event.update({
                'status': 'recovery_added',
                'frontier_id': frontier_id,
                'priority': priority,
                'estimated_gain': float(local_gain),
                'path_cost': float(path_cost),
                'path': path,
            })
            self.recovery_targets.append(point)
            events.append(event)
            added += 1

        if added:
            self.recovery_rounds += 1
            self._record_step(
                selected_frontier=None,
                path=[],
                generated_candidates=events,
                coverage_before=coverage,
                coverage_after=coverage,
                event='global_recovery',
            )
            if self.config.verbose:
                print(f"  [Recovery] Added {added} reachable gap candidates "
                      f"(round {self.recovery_rounds}, coverage={coverage:.1%})")
        return events

    def _compute_priority(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Compute priority for a frontier based on selected strategy.

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value (higher = more important).
        """
        strategy = self.config.frontier_strategy

        if strategy == FrontierSelectionStrategy.BASELINE:
            return self._priority_baseline(base_position, target, exploration_strength)
        elif strategy == FrontierSelectionStrategy.ENHANCED_DISTANCE:
            return self._priority_enhanced_distance(base_position, target, exploration_strength)
        elif strategy == FrontierSelectionStrategy.DUAL_FACTOR:
            return self._priority_dual_factor(base_position, target, exploration_strength)
        elif strategy == FrontierSelectionStrategy.CUMULATIVE_DISTANCE:
            return self._priority_cumulative_distance(base_position, target, exploration_strength)
        elif strategy == FrontierSelectionStrategy.CLUSTER_PRIORITY:
            return self._priority_cluster_priority(base_position, target, exploration_strength)
        elif strategy == FrontierSelectionStrategy.HYBRID_ADAPTIVE:
            return self._priority_hybrid_adaptive(base_position, target, exploration_strength)
        else:
            # Fallback to baseline
            return self._priority_baseline(base_position, target, exploration_strength)

    def _update_all_priorities(self):
        """
        Update priorities of all frontiers in the queue.

        Phase 1 Optimization: Localized updates - only update frontiers
        within influence radius of current position.
        """
        # Get all frontiers
        frontiers = self.frontier_queue.get_all_frontiers()

        # Phase 1 Optimization: Localized updates
        updated_count = 0
        skipped_count = 0

        for frontier in frontiers:
            # Localized update: only update if within influence radius
            if self.config.enable_localized_updates:
                dist_to_current = euclidean_distance(frontier.target, self.current_pose)
                if dist_to_current > self.config.priority_update_radius:
                    skipped_count += 1
                    continue

            # Recheck collision type (may have changed due to new explored nodes)
            # FIXED: now passes r_view for correct coverage check
            collision_type, strength = self.collision_checker.check_collision_type(
                frontier.target, self.explored_nodes, self.d_repel, self.config.r_view
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
            updated_count += 1

        # Debug logging for localized updates
        if self.config.verbose and self.config.enable_localized_updates and self.iteration_count % 5 == 0:
            total = updated_count + skipped_count
            if total > 0:
                print(f"  [Localized] Updated: {updated_count}/{total} frontiers "
                      f"(skipped {skipped_count} distant)")

    def _prune_covered_frontiers(self):
        """
        Phase 1 Optimization: Remove frontiers that are now covered by explored nodes.

        This is called periodically during exploration to clean up the queue.
        """
        to_remove = []
        for frontier in self.frontier_queue.get_all_frontiers():
            if self._is_covered_by_explored(frontier.target):
                to_remove.append(frontier.frontier_id)

        for fid in to_remove:
            self.frontier_queue.remove(fid)

        if self.config.verbose and len(to_remove) > 0:
            print(f"  [Pruning] Removed {len(to_remove)} covered frontiers, "
                  f"queue size: {len(self.frontier_queue)}")

    def _too_close_to_existing_frontier(self, target: Tuple[float, float], min_dist: float) -> bool:
        """
        Phase 1 Optimization: Check if target is too close to existing frontiers.

        This prevents adding redundant frontiers that are very close to each other.

        Args:
            target: Target position to check
            min_dist: Minimum allowed distance to existing frontiers

        Returns:
            True if too close to any existing frontier
        """
        for frontier in self.frontier_queue.get_all_frontiers():
            if euclidean_distance(target, frontier.target) < min_dist:
                return True
        return False

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

    def _compute_environment_density(self, occupancy_grid: OccupancyGrid) -> float:
        """
        Compute environment density (ratio of occupied space).

        Higher density means more obstacles, which makes exploration harder.

        Args:
            occupancy_grid: Occupancy grid map.

        Returns:
            Density ratio [0, 1] where 0 = completely free, 1 = completely occupied.
        """
        total_cells = occupancy_grid.data.size
        occupied_cells = np.sum(occupancy_grid.data >= self.config.obstacle_threshold)
        return occupied_cells / total_cells

    def _refresh_geodesic_costs(self) -> None:
        """Refresh one-to-all path cost once per accepted viewpoint."""
        if (self.config.use_geodesic_priority and
                getattr(self, 'astar_planner', None) is not None):
            self.geodesic_cost_map = self.astar_planner.compute_cost_map(self.current_pose)
        else:
            self.geodesic_cost_map = None

    def _distance_to_target(self, target: Tuple[float, float]) -> float:
        """Obstacle-aware travel estimate with Euclidean fallback."""
        if self.geodesic_cost_map is not None:
            row, col = self.coverage_analyzer.grid.world_to_grid(target[0], target[1])
            value = float(self.geodesic_cost_map[row, col])
            return value
        return euclidean_distance(self.current_pose, target)

    def _clearance_at(self, target: Tuple[float, float]) -> float:
        """Distance from a viewpoint to the closest raw obstacle/boundary."""
        if self.obstacle_clearance_map is None:
            return 0.0
        row, col = self.coverage_analyzer.grid.world_to_grid(target[0], target[1])
        return float(self.obstacle_clearance_map[row, col])

    def _should_terminate(self) -> bool:
        """
        Check termination conditions.

        Returns:
            True if exploration should terminate.
        """
        # Max iterations reached
        if self.iteration_count >= self.config.max_iterations:
            self.termination_reason = "max_iterations"
            if self.config.verbose:
                print("Max iterations reached")
            return True

        # No more frontiers
        if self.frontier_queue.is_empty():
            recovery_events = self._inject_recovery_frontiers()
            if any(event.get('status') == 'recovery_added' for event in recovery_events):
                return False
            self.termination_reason = "coverage_target" if (
                self._get_current_coverage() >= self.config.target_coverage
            ) else "frontier_queue_empty"
            return True

        # Get current coverage
        coverage = self._get_current_coverage()

        # Max priority too low (use adaptive threshold)
        max_priority = self.frontier_queue.max_priority()
        threshold = self.adaptive_min_priority if hasattr(self, 'adaptive_min_priority') else self.config.min_priority_threshold

        if max_priority is not None and max_priority < threshold:
            # IMPROVED: Check if priority is numerically valid (not near-zero due to numerical issues)
            # If max_priority is extremely small (< 1e-10), there might be numerical issues
            # or all frontiers are blocked by explored nodes
            if max_priority < 1e-10:
                # Check if we have valid unexplored frontiers by examining the queue
                # Count frontiers with non-zero priority (use entry_map to avoid REMOVED entries)
                valid_frontiers = sum(1 for f in self.frontier_queue._entry_map.values()
                                     if f != self.frontier_queue._REMOVED and abs(f.priority) > 1e-10)

                if valid_frontiers > 0 and coverage < 0.94:
                    # We have valid frontiers but priority calculation might have issues
                    # This can happen in multi-room environments where distance decay is too aggressive
                    if self.config.verbose:
                        print(f"Warning: Max priority near zero ({max_priority:.2e}) but {valid_frontiers} frontiers exist")
                        print(f"Coverage {coverage:.1%} < 94%, forcing re-evaluation")
                    # Return False to continue, but this indicates a potential issue
                    return False
                else:
                    if self.config.verbose:
                        print(f"Max priority {max_priority:.2e} near zero, no valid frontiers")
                    recovery_events = self._inject_recovery_frontiers()
                    if any(event.get('status') == 'recovery_added' for event in recovery_events):
                        return False
                    self.termination_reason = "priority_exhausted"
                    return True

            # In dense environments, be more lenient before giving up
            density_thresh = getattr(self.config, 'density_threshold', 0.20)
            if hasattr(self, 'environment_density') and self.environment_density > density_thresh:
                # Check if we still have reasonable coverage potential
                # If coverage is low and priority is not too bad, keep going
                if coverage < 0.85 and max_priority > threshold * 0.3:
                    return False

            # IMPROVED: For multi-room or complex environments, be more lenient
            # If coverage is far from target, don't give up too easily
            if coverage < 0.93:  # Increased from 0.80 to push closer to 95% target
                # Check if we have a reasonable number of unexplored frontiers
                queue_size = self.frontier_queue.size()
                if queue_size >= 3 and max_priority > threshold * 0.05:  # More lenient: 0.05 from 0.1
                    if self.config.verbose:
                        print(f"Coverage {coverage:.1%} < 93%, {queue_size} frontiers remain, continuing")
                    return False

            if self.config.verbose:
                print(f"Max priority {max_priority:.3f} below threshold {threshold:.3f}")
            recovery_events = self._inject_recovery_frontiers()
            if any(event.get('status') == 'recovery_added' for event in recovery_events):
                return False
            self.termination_reason = "priority_below_threshold"
            return True

        # Coverage target reached and no high-priority frontiers remain
        if coverage >= self.config.target_coverage:
            if max_priority is not None and max_priority < 0.5:
                if self.config.verbose:
                    print(f"Coverage target reached: {coverage:.1%}")
                self.termination_reason = "coverage_target"
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

    # =========================================================================
    # Frontier Selection Strategies (for Ablation Study)
    # =========================================================================

    def _priority_baseline(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Baseline strategy: Original implementation.

        Priority = S_explore(f) × W_dist(f)
        where W_dist(f) = 1 / (1 + (d_curr/r_view)^α)

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value.
        """
        # Base score from exploration strength
        base_score = exploration_strength

        # Distance weight (favor nearby targets)
        distance = self._distance_to_target(target)

        # Performance optimization: Use cached coverage instead of recomputing
        # This is computed once per iteration, not per priority calculation
        alpha = self.config.get_alpha(self.cached_coverage)

        distance_weight = 1.0 / (1.0 + (distance / self.config.r_view) ** alpha)

        # Combine factors
        priority = base_score * distance_weight

        return priority

    def _priority_enhanced_distance(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Strategy A: Enhanced distance weight with exponential decay.

        Priority = S_explore(f) × W_dist^enhanced(f) × density_bonus
        where W_dist^enhanced(f) = exp(-β × d_curr/r_view)

        In dense obstacle environments, applies a bonus to prevent premature termination.

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value.
        """
        base_score = exploration_strength

        # Enhanced distance weight with exponential decay
        distance = self._distance_to_target(target)
        beta = self.config.beta

        distance_weight = np.exp(-beta * distance / self.config.r_view)

        clearance_score = min(
            self._clearance_at(target) / max(self.config.r_view, 1e-6), 1.0
        )
        clearance_bonus = 1.0 + self.config.clearance_priority_weight * clearance_score
        priority = base_score * distance_weight * clearance_bonus

        # Apply density bonus in dense environments
        if hasattr(self, 'environment_density'):
            density_thresh = getattr(self.config, 'density_threshold', 0.20)
            if self.environment_density > density_thresh:
                # In dense environments, boost priority to encourage continued exploration
                # Bonus increases with density: up to 3x boost at 40% density
                density_excess = self.environment_density - density_thresh
                density_bonus = 1.0 + min(density_excess / 0.10, 2.0)  # Up to 3x
                priority *= density_bonus

        return priority

    def _priority_dual_factor(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Strategy B: Dual factor weighting (decoupled quality & distance).

        Priority = λ × S_explore(f) + (1-λ) × W_dist(f)
        where λ is adaptive based on coverage ratio

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value.
        """
        # Get adaptive lambda using cached coverage
        lambda_weight = self.config.get_lambda(self.cached_coverage)

        # Exploration quality component
        quality_score = exploration_strength

        # Distance component (using baseline formula)
        distance = self._distance_to_target(target)
        alpha = self.config.get_alpha(self.cached_coverage)
        distance_score = 1.0 / (1.0 + (distance / self.config.r_view) ** alpha)

        # Linear combination
        priority = lambda_weight * quality_score + (1.0 - lambda_weight) * distance_score

        return priority

    def _priority_cumulative_distance(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Strategy C: Cumulative distance penalty (global path efficiency).

        Priority = S_explore(f) × W_dist(f) × W_travel(f)
        where W_travel(f) = exp(-γ × (D_total + d_curr) / D_avg)

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value.
        """
        base_score = exploration_strength

        # Distance weight (baseline)
        distance = self._distance_to_target(target)
        coverage_ratio = self._get_current_coverage()
        alpha = self.config.get_alpha(coverage_ratio)
        distance_weight = 1.0 / (1.0 + (distance / self.config.r_view) ** alpha)

        # Travel cost penalty
        gamma = self.config.gamma

        # Compute average step distance (avoid division by zero)
        avg_step_distance = self.total_distance / max(1, len(self.explored_nodes) - 1)
        if avg_step_distance < 0.1:  # Minimum threshold
            avg_step_distance = self.config.r_view

        # Penalty based on cumulative + current distance
        cumulative_factor = (self.total_distance + distance) / avg_step_distance
        travel_weight = np.exp(-gamma * cumulative_factor)

        priority = base_score * distance_weight * travel_weight

        return priority

    def _priority_cluster_priority(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Strategy D: Local cluster priority (region-focused exploration).

        Priority = S_explore(f) × W_dist(f) × C_cluster(f)
        where C_cluster(f) = 1 + η × N_nearby / N_total

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value.
        """
        base_score = exploration_strength

        # Distance weight (baseline) using cached coverage
        distance = self._distance_to_target(target)
        alpha = self.config.get_alpha(self.cached_coverage)
        distance_weight = 1.0 / (1.0 + (distance / self.config.r_view) ** alpha)

        # Cluster factor: count nearby frontiers
        eta = self.config.eta
        r_cluster = self.config.r_cluster

        all_frontiers = self.frontier_queue.get_all_frontiers()
        n_total = len(all_frontiers)

        if n_total == 0:
            cluster_factor = 1.0
        else:
            # Count frontiers within cluster radius
            n_nearby = sum(
                1 for f in all_frontiers
                if euclidean_distance(target, f.target) < r_cluster and f.target != target
            )

            cluster_factor = 1.0 + eta * (n_nearby / n_total)

        priority = base_score * distance_weight * cluster_factor

        return priority

    def _priority_hybrid_adaptive(
        self,
        base_position: Tuple[float, float],
        target: Tuple[float, float],
        exploration_strength: float
    ) -> float:
        """
        Strategy E: Hybrid adaptive strategy (combines A + D).

        Priority = S_explore(f) × W_dist^enhanced(f) × [1 + ω(ρ) × C_cluster(f)]
        where ω adapts based on coverage ratio

        Args:
            base_position: Base position where frontier originates.
            target: Target position.
            exploration_strength: Strength from collision checker [0, 1].

        Returns:
            Priority value.
        """
        base_score = exploration_strength

        # Enhanced distance weight (exponential decay - from Strategy A)
        distance = self._distance_to_target(target)
        beta = self.config.beta
        distance_weight = np.exp(-beta * distance / self.config.r_view)

        # Adaptive cluster weight using cached coverage
        omega = self.config.get_omega(self.cached_coverage)

        # Cluster factor (from Strategy D)
        if omega > 0:  # Only compute if needed
            eta = self.config.eta
            r_cluster = self.config.r_cluster

            all_frontiers = self.frontier_queue.get_all_frontiers()
            n_total = len(all_frontiers)

            if n_total == 0:
                cluster_contribution = 0.0
            else:
                n_nearby = sum(
                    1 for f in all_frontiers
                    if euclidean_distance(target, f.target) < r_cluster and f.target != target
                )
                cluster_contribution = eta * (n_nearby / n_total)

            cluster_multiplier = 1.0 + omega * cluster_contribution
        else:
            cluster_multiplier = 1.0

        priority = base_score * distance_weight * cluster_multiplier

        return priority

    # =========================================================================

    def __repr__(self) -> str:
        return (
            f"SSTGExplorer(r_view={self.config.r_view}m, "
            f"d_theta={self.config.d_theta}°, "
            f"overlap={self.config.overlap}m)"
        )
