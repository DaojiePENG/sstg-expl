"""
Coverage analysis for SSTG Explorer.
"""
import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass

from src.map.occupancy_grid import OccupancyGrid
from src.utils.geometry import euclidean_distance


@dataclass
class CoverageStatistics:
    """Statistics about exploration coverage."""
    coverage_ratio: float  # Covered area / free space area
    num_nodes: int  # Number of exploration nodes
    covered_area: float  # Total covered area in m²
    free_space_area: float  # Total free space area in m²
    min_node_distance: float  # Minimum distance between any two nodes
    mean_node_distance: float  # Mean distance between neighboring nodes
    coverage_uniformity: float  # Coefficient of variation of node distances


class CoverageAnalyzer:
    """
    Analyzer for exploration coverage.

    Computes coverage metrics and identifies gaps.
    """

    def __init__(self, occupancy_grid: OccupancyGrid):
        """
        Initialize coverage analyzer.

        Args:
            occupancy_grid: Occupancy grid map.
        """
        self.grid = occupancy_grid
        self.free_space_mask = occupancy_grid.get_free_space_mask()
        self.free_space_area = occupancy_grid.compute_free_space_area()

    def compute_coverage_map(
        self,
        explored_nodes: List[Tuple[float, float]],
        r_view: float
    ) -> np.ndarray:
        """
        Compute coverage map showing which areas are covered.

        Args:
            explored_nodes: List of explored positions (x, y).
            r_view: View radius in meters.

        Returns:
            Boolean array where True = covered by at least one node.
        """
        coverage_map = np.zeros_like(self.free_space_mask, dtype=bool)

        # For each explored node, mark covered cells
        for node in explored_nodes:
            i_center, j_center = self.grid.world_to_grid(node[0], node[1])

            # Radius in cells
            r_cells = int(np.ceil(r_view / self.grid.resolution))

            # Mark cells within radius
            for di in range(-r_cells, r_cells + 1):
                for dj in range(-r_cells, r_cells + 1):
                    i = i_center + di
                    j = j_center + dj

                    if not self.grid.is_valid_grid(i, j):
                        continue

                    # Check if cell is within circular radius
                    cell_pos = self.grid.grid_to_world(i, j)
                    if euclidean_distance(node, cell_pos) <= r_view:
                        coverage_map[i, j] = True

        return coverage_map

    def compute_coverage_ratio(
        self,
        explored_nodes: List[Tuple[float, float]],
        r_view: float
    ) -> float:
        """
        Compute coverage ratio.

        Args:
            explored_nodes: List of explored positions.
            r_view: View radius.

        Returns:
            Coverage ratio (covered free space / total free space).
        """
        if self.free_space_area == 0:
            return 0.0

        coverage_map = self.compute_coverage_map(explored_nodes, r_view)

        # Count covered free space cells
        covered_free = np.sum(coverage_map & self.free_space_mask)
        total_free = np.sum(self.free_space_mask)

        return covered_free / total_free if total_free > 0 else 0.0

    def compute_covered_area(
        self,
        explored_nodes: List[Tuple[float, float]],
        r_view: float
    ) -> float:
        """
        Compute total covered area in square meters.

        Args:
            explored_nodes: List of explored positions.
            r_view: View radius.

        Returns:
            Covered area in m².
        """
        coverage_map = self.compute_coverage_map(explored_nodes, r_view)
        covered_free = np.sum(coverage_map & self.free_space_mask)
        cell_area = self.grid.resolution ** 2
        return covered_free * cell_area

    def check_node_spacing(
        self,
        explored_nodes: List[Tuple[float, float]],
        min_distance: float
    ) -> Tuple[bool, List[Tuple[int, int, float]]]:
        """
        Check if all nodes satisfy minimum spacing constraint.

        Args:
            explored_nodes: List of explored positions.
            min_distance: Minimum required distance between nodes.

        Returns:
            Tuple of (all_valid, violations):
            - all_valid: True if all pairs satisfy constraint.
            - violations: List of (node_i, node_j, distance) for violations.
        """
        violations = []
        n = len(explored_nodes)

        for i in range(n):
            for j in range(i + 1, n):
                dist = euclidean_distance(explored_nodes[i], explored_nodes[j])
                if dist < min_distance:
                    violations.append((i, j, dist))

        return (len(violations) == 0, violations)

    def compute_node_distances(
        self,
        explored_nodes: List[Tuple[float, float]]
    ) -> Dict[str, float]:
        """
        Compute distance statistics between nodes.

        Args:
            explored_nodes: List of explored positions.

        Returns:
            Dictionary with 'min', 'max', 'mean', 'std' distances.
        """
        n = len(explored_nodes)

        if n < 2:
            return {'min': 0.0, 'max': 0.0, 'mean': 0.0, 'std': 0.0}

        distances = []
        for i in range(n):
            for j in range(i + 1, n):
                dist = euclidean_distance(explored_nodes[i], explored_nodes[j])
                distances.append(dist)

        distances = np.array(distances)

        return {
            'min': float(np.min(distances)),
            'max': float(np.max(distances)),
            'mean': float(np.mean(distances)),
            'std': float(np.std(distances))
        }

    def find_coverage_gaps(
        self,
        explored_nodes: List[Tuple[float, float]],
        r_view: float,
        min_gap_size: float = 0.5
    ) -> List[Tuple[float, float]]:
        """
        Find centers of uncovered gaps in free space.

        Args:
            explored_nodes: List of explored positions.
            r_view: View radius.
            min_gap_size: Minimum gap size in meters to report.

        Returns:
            List of gap center positions (x, y).
        """
        from scipy.ndimage import label, center_of_mass

        # Get coverage map
        coverage_map = self.compute_coverage_map(explored_nodes, r_view)

        # Find uncovered free space
        uncovered = self.free_space_mask & (~coverage_map)

        # Label connected components
        labeled, num_features = label(uncovered)

        # Find gaps larger than min_gap_size
        gap_centers = []
        min_cells = int((min_gap_size / self.grid.resolution) ** 2)

        for region_id in range(1, num_features + 1):
            region_mask = (labeled == region_id)
            region_size = np.sum(region_mask)

            if region_size >= min_cells:
                # Compute center of mass
                center_i, center_j = center_of_mass(region_mask)
                center_pos = self.grid.grid_to_world(int(center_i), int(center_j))
                gap_centers.append(center_pos)

        return gap_centers

    def compute_coverage_uniformity(
        self,
        explored_nodes: List[Tuple[float, float]]
    ) -> float:
        """
        Compute coverage uniformity (coefficient of variation of distances).

        Lower values indicate more uniform node distribution.

        Args:
            explored_nodes: List of explored positions.

        Returns:
            Coefficient of variation (std/mean) of pairwise distances.
        """
        dist_stats = self.compute_node_distances(explored_nodes)

        if dist_stats['mean'] == 0:
            return 0.0

        return dist_stats['std'] / dist_stats['mean']

    def compute_statistics(
        self,
        explored_nodes: List[Tuple[float, float]],
        r_view: float,
        min_node_distance: Optional[float] = None
    ) -> CoverageStatistics:
        """
        Compute comprehensive coverage statistics.

        Args:
            explored_nodes: List of explored positions.
            r_view: View radius.
            min_node_distance: Minimum required distance (defaults to r_view - overlap).

        Returns:
            CoverageStatistics object.
        """
        coverage_ratio = self.compute_coverage_ratio(explored_nodes, r_view)
        covered_area = self.compute_covered_area(explored_nodes, r_view)
        dist_stats = self.compute_node_distances(explored_nodes)
        uniformity = self.compute_coverage_uniformity(explored_nodes)

        return CoverageStatistics(
            coverage_ratio=coverage_ratio,
            num_nodes=len(explored_nodes),
            covered_area=covered_area,
            free_space_area=self.free_space_area,
            min_node_distance=dist_stats['min'],
            mean_node_distance=dist_stats['mean'],
            coverage_uniformity=uniformity
        )

    def visualize_coverage(
        self,
        explored_nodes: List[Tuple[float, float]],
        r_view: float,
        figsize: Tuple[int, int] = (12, 5)
    ):
        """
        Visualize coverage analysis (requires matplotlib).

        Args:
            explored_nodes: List of explored positions.
            r_view: View radius.
            figsize: Figure size.
        """
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import Circle
        except ImportError:
            print("Matplotlib not available for visualization")
            return

        coverage_map = self.compute_coverage_map(explored_nodes, r_view)
        gaps = self.find_coverage_gaps(explored_nodes, r_view)

        fig, axes = plt.subplots(1, 2, figsize=figsize)

        # Plot 1: Coverage map
        ax = axes[0]
        ax.imshow(self.free_space_mask, cmap='gray', origin='lower', alpha=0.3)
        ax.imshow(coverage_map, cmap='Greens', origin='lower', alpha=0.6)

        for node in explored_nodes:
            i, j = self.grid.world_to_grid(node[0], node[1])
            ax.plot(j, i, 'ro', markersize=3)

        ax.set_title('Coverage Map')
        ax.axis('equal')

        # Plot 2: Gaps
        ax = axes[1]
        ax.imshow(self.free_space_mask, cmap='gray', origin='lower', alpha=0.3)
        uncovered = self.free_space_mask & (~coverage_map)
        ax.imshow(uncovered, cmap='Reds', origin='lower', alpha=0.6)

        for gap in gaps:
            i, j = self.grid.world_to_grid(gap[0], gap[1])
            ax.plot(j, i, 'bx', markersize=10, markeredgewidth=2)

        ax.set_title('Coverage Gaps')
        ax.axis('equal')

        plt.tight_layout()
        plt.show()

    def __repr__(self) -> str:
        return (
            f"CoverageAnalyzer(grid={self.grid}, "
            f"free_space={self.free_space_area:.2f}m²)"
        )
