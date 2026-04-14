"""
Base explorer interface for all exploration algorithms.

This module defines the abstract base class that all exploration algorithms
must implement to ensure compatibility with the benchmark framework.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple
import numpy as np


class BaseExplorer(ABC):
    """
    Abstract base class for exploration algorithms.

    All exploration algorithms should inherit from this class and implement
    the explore() method with the same interface for fair comparison.

    Attributes:
        name: Algorithm name for identification
        config: Algorithm-specific configuration
    """

    def __init__(self, name: str, config: Optional[Dict] = None):
        """
        Initialize base explorer.

        Args:
            name: Algorithm name (e.g., "SSTG", "Uniform Grid", "RRT")
            config: Algorithm configuration dictionary
        """
        self.name = name
        self.config = config or {}
        self.nodes = []  # Explored nodes
        self.total_distance = 0.0
        self.coverage_ratio = 0.0

    @abstractmethod
    def explore(
        self,
        occupancy_grid: np.ndarray,
        start_pose: Tuple[float, float, float],
        visualizer: Optional[object] = None
    ) -> Dict:
        """
        Run exploration algorithm on given occupancy grid.

        This is the main interface method that all algorithms must implement.
        The method should explore the environment starting from start_pose
        and return exploration results in a standardized format.

        Args:
            occupancy_grid: 2D occupancy grid (0=free, 100=occupied)
            start_pose: Starting pose (x, y, theta) in world coordinates
            visualizer: Optional visualizer for real-time display

        Returns:
            Dictionary containing:
                - 'nodes': List of explored node dictionaries with keys:
                    - 'position': (x, y) in world coordinates
                    - 'theta': orientation
                    - 'coverage_area': area covered by this node
                - 'metadata': Dictionary with:
                    - 'total_distance': total travel distance
                    - 'coverage_ratio': fraction of free space covered
                    - 'num_nodes': number of sampled nodes
                    - 'computation_time': algorithm runtime
                    - 'success': whether exploration completed successfully
                - 'success': overall success flag
                - 'algorithm': algorithm name

        Raises:
            NotImplementedError: If not implemented by subclass
        """
        raise NotImplementedError("Subclass must implement explore() method")

    def reset(self):
        """Reset explorer state for a new run."""
        self.nodes = []
        self.total_distance = 0.0
        self.coverage_ratio = 0.0

    def get_metrics(self) -> Dict:
        """
        Get exploration metrics.

        Returns:
            Dictionary with standard metrics:
                - total_distance: cumulative travel distance
                - num_nodes: number of explored nodes
                - coverage_ratio: fraction of free space covered
                - coverage_efficiency: coverage per unit distance
        """
        return {
            'total_distance': self.total_distance,
            'num_nodes': len(self.nodes),
            'coverage_ratio': self.coverage_ratio,
            'coverage_efficiency': (
                self.coverage_ratio / max(self.total_distance, 0.01)
            )
        }

    def __str__(self) -> str:
        """String representation."""
        return f"{self.name} Explorer"

    def __repr__(self) -> str:
        """Detailed representation."""
        return (f"{self.__class__.__name__}(name='{self.name}', "
                f"config={self.config})")
