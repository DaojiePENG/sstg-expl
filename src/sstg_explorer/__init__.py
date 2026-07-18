"""Public API for SSTG-Explorer."""

from .config import ExplorerConfig, FrontierSelectionStrategy
from .core.explorer import SSTGExplorer
from .map.occupancy_grid import OccupancyGrid

__all__ = [
    "SSTGExplorer",
    "ExplorerConfig",
    "FrontierSelectionStrategy",
    "OccupancyGrid",
]

__version__ = "0.1.0"
