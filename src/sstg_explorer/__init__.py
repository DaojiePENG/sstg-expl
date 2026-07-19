"""Public API for SSTG-Explorer."""

from .config import ExplorerConfig, FrontierSelectionStrategy
from .core.explorer import SSTGExplorer
from .map.occupancy_grid import OccupancyGrid
from .sensing import SensorConfig
from .unknown import UnknownExplorerConfig, UnknownMapExplorer

__all__ = [
    "SSTGExplorer",
    "ExplorerConfig",
    "FrontierSelectionStrategy",
    "OccupancyGrid",
    "SensorConfig",
    "UnknownExplorerConfig",
    "UnknownMapExplorer",
]

__version__ = "0.1.0"
