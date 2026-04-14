"""
Configuration parameters for SSTG Explorer.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class ExplorerConfig:
    """Configuration for SSTG Explorer algorithm."""

    # Core exploration parameters
    r_view: float = 2.0          # View radius in meters
    d_theta: float = 45.0        # Angular interval in degrees
    overlap: float = 0.25        # Overlap distance in meters
    r_robot: float = 0.3         # Robot radius in meters
    d_safe: float = 0.2          # Safety distance from obstacles in meters

    # Priority calculation parameters
    alpha: float = 2.0           # Distance decay exponent (局部性控制)
    adaptive_alpha: bool = True  # Whether to adapt alpha based on coverage
    alpha_early: float = 2.0     # Alpha for early exploration (< 30% coverage)
    alpha_mid: float = 1.0       # Alpha for mid exploration (30-70% coverage)
    alpha_late: float = 0.5      # Alpha for late exploration (> 70% coverage)

    # Termination conditions
    min_priority_threshold: float = 0.1  # Minimum priority to continue
    target_coverage: float = 0.95         # Target coverage ratio
    max_iterations: int = 10000           # Maximum number of exploration steps

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
