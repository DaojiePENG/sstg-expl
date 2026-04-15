"""
Configuration parameters for SSTG Explorer.
"""
from dataclasses import dataclass
from typing import Optional
from enum import Enum


class FrontierSelectionStrategy(Enum):
    """Frontier selection strategies for ablation study."""
    BASELINE = "baseline"
    ENHANCED_DISTANCE = "enhanced_distance"      # Strategy A
    DUAL_FACTOR = "dual_factor"                  # Strategy B
    CUMULATIVE_DISTANCE = "cumulative_distance"  # Strategy C
    CLUSTER_PRIORITY = "cluster_priority"        # Strategy D
    HYBRID_ADAPTIVE = "hybrid_adaptive"          # Strategy E


@dataclass
class ExplorerConfig:
    """Configuration for SSTG Explorer algorithm."""

    # Core exploration parameters
    r_view: float = 2.0          # View radius in meters
    d_theta: float = 30.0        # Angular interval in degrees (default, may be adapted)
    overlap: float = 0.25        # Overlap distance in meters
    r_robot: float = 0.3         # Robot radius in meters
    d_safe: float = 0.2          # Safety distance from obstacles in meters

    # Adaptive d_theta parameters
    adaptive_dtheta: bool = False  # Disable adaptive d_theta (fixed 30° performs best across all environments)
    dtheta_simple: float = 45.0    # d_theta for simple environments (fewer directions, better for open spaces)
    dtheta_complex: float = 30.0   # d_theta for complex environments (more directions, better for obstacles)
    complexity_threshold: float = 0.12  # Environment density threshold: <12% = simple, ≥12% = complex

    # Frontier selection strategy
    frontier_strategy: FrontierSelectionStrategy = FrontierSelectionStrategy.BASELINE

    # Priority calculation parameters (Baseline)
    alpha: float = 2.0           # Distance decay exponent (favor locality)
    adaptive_alpha: bool = True  # Whether to adapt alpha based on coverage
    alpha_early: float = 2.0     # Alpha for early exploration (< 30% coverage) - back to original
    alpha_mid: float = 1.0       # Alpha for mid exploration (30-70% coverage)
    alpha_late: float = 0.5      # Alpha for late exploration (> 70% coverage)

    # Strategy A: Enhanced Distance Weight
    beta: float = 1.0            # Distance penalty coefficient (exponential decay)

    # Strategy B: Dual Factor Weighting
    lambda_weight: float = 0.5   # Exploration quality weight [0, 1]
    adaptive_lambda: bool = True # Whether to adapt lambda based on coverage
    lambda_early: float = 0.3    # Lambda for early exploration (prioritize distance)
    lambda_mid: float = 0.5      # Lambda for mid exploration (balanced)
    lambda_late: float = 0.7     # Lambda for late exploration (prioritize quality)

    # Strategy C: Cumulative Distance Penalty
    gamma: float = 0.2           # Travel cost penalty coefficient

    # Strategy D: Local Cluster Priority
    eta: float = 1.0             # Cluster reward coefficient
    r_cluster: float = 4.0       # Cluster radius in meters (default: 2 * r_view)

    # Strategy E: Hybrid Adaptive (combines A + D)
    omega_early: float = 0.0     # Cluster weight for early exploration (< 20%)
    omega_mid: float = 0.5       # Cluster weight for mid exploration (20-80%)
    omega_late: float = 1.0      # Cluster weight for late exploration (> 80%)

    # Path planning options
    use_astar: bool = True       # Use A* path planning instead of straight lines (default True for robustness)
    astar_max_iterations: int = 10000  # Max iterations for A* search

    # Adaptive sampling options
    use_adaptive_sampling: bool = False  # Enable adaptive angular sampling
    narrow_threshold: float = 1.5        # Passage width threshold (meters)
    min_d_theta: float = 15.0            # Minimum angular interval for narrow passages

    # Termination conditions
    min_priority_threshold: float = 0.005  # Minimum priority to continue (lowered from 0.02 for multi-room environments)
    target_coverage: float = 0.95         # Target coverage ratio
    max_iterations: int = 10000           # Maximum number of exploration steps
    adaptive_threshold: bool = True       # Adapt threshold based on environment density
    density_threshold: float = 0.20       # Environment density threshold for adaptation (20%)

    # Map parameters
    map_resolution: float = 0.05  # Map resolution in meters/pixel
    obstacle_threshold: int = 50  # Occupancy threshold (0-100)

    # Path planning
    max_path_length: float = 10.0  # Maximum path length in meters
    path_planning_timeout: float = 5.0  # Path planning timeout in seconds

    # Debugging and visualization
    verbose: bool = True
    save_intermediate: bool = False
    visualize_steps: bool = False

    @property
    def d_repel(self) -> float:
        """Repulsion distance for explored nodes."""
        return self.r_view - self.overlap

    @property
    def n_directions(self) -> int:
        """Number of exploration directions."""
        return int(360 / self.d_theta)

    def get_alpha(self, coverage_ratio: float) -> float:
        """Get alpha parameter based on current coverage ratio."""
        if not self.adaptive_alpha:
            return self.alpha

        if coverage_ratio < 0.3:
            return self.alpha_early
        elif coverage_ratio < 0.7:
            return self.alpha_mid
        else:
            return self.alpha_late

    def get_lambda(self, coverage_ratio: float) -> float:
        """Get lambda parameter based on current coverage ratio (Strategy B)."""
        if not self.adaptive_lambda:
            return self.lambda_weight

        if coverage_ratio < 0.3:
            return self.lambda_early
        elif coverage_ratio < 0.7:
            return self.lambda_mid
        else:
            return self.lambda_late

    def get_omega(self, coverage_ratio: float) -> float:
        """Get omega (cluster weight) based on current coverage ratio (Strategy E)."""
        if coverage_ratio < 0.2:
            return self.omega_early
        elif coverage_ratio < 0.8:
            return self.omega_mid
        else:
            return self.omega_late


@dataclass
class SimulationConfig:
    """Configuration for simulation environments."""

    width: float = 10.0   # Environment width in meters
    height: float = 10.0  # Environment height in meters
    resolution: float = 0.05  # Map resolution in meters/pixel

    # Start pose
    start_x: float = 5.0
    start_y: float = 5.0
    start_theta: float = 0.0

    # Visualization
    figsize: tuple = (10, 10)
    dpi: int = 100


@dataclass
class BenchmarkConfig:
    """Configuration for benchmarking."""

    num_runs: int = 10  # Number of runs per algorithm per environment
    save_plots: bool = True
    save_data: bool = True
    output_dir: str = "benchmarks/results"

    # Metrics to compute
    compute_coverage: bool = True
    compute_node_spacing: bool = True
    compute_travel_distance: bool = True
    compute_time: bool = True
