"""
Frontier data structures for SSTG Explorer.

Frontiers represent potential exploration directions from a given position.
"""
import heapq
from dataclasses import dataclass, field
from typing import Tuple, Dict, List, Optional
import itertools


@dataclass(order=True)
class Frontier:
    """
    Represents a frontier (potential exploration direction).

    Attributes:
        priority: Priority value (higher = more important). Used for heap sorting.
        position: Base position (x, y) where this frontier originates.
        angle: Direction angle in degrees [0, 360).
        target: Target position (x, y) to explore.
        frontier_id: Unique identifier for this frontier.
    """

    priority: float = field(compare=True)
    position: Tuple[float, float] = field(compare=False)
    angle: float = field(compare=False)
    target: Tuple[float, float] = field(compare=False)
    frontier_id: int = field(default=0, compare=False)

    def __repr__(self) -> str:
        return (
            f"Frontier(id={self.frontier_id}, "
            f"pos=({self.position[0]:.2f}, {self.position[1]:.2f}), "
            f"angle={self.angle:.1f}°, "
            f"target=({self.target[0]:.2f}, {self.target[1]:.2f}), "
            f"priority={self.priority:.3f})"
        )


class FrontierQueue:
    """
    Priority queue for managing frontiers.

    Uses heapq for efficient priority-based operations. Supports:
    - Adding frontiers with priority
    - Popping highest-priority frontier
    - Updating priorities
    - Removing specific frontiers

    Note: Uses max-heap by negating priorities (heapq is min-heap by default).
    """

    def __init__(self):
        """Initialize an empty frontier queue."""
        self._heap: List[Frontier] = []
        self._entry_map: Dict[int, Frontier] = {}  # frontier_id -> Frontier
        self._counter = itertools.count()  # Unique ID generator
        self._REMOVED = "REMOVED"  # Marker for removed entries

    def add(self, position: Tuple[float, float], angle: float,
            target: Tuple[float, float], priority: float) -> int:
        """
        Add a new frontier to the queue.

        Args:
            position: Base position (x, y).
            angle: Direction angle in degrees.
            target: Target position (x, y).
            priority: Priority value (higher = more important).

        Returns:
            frontier_id: Unique identifier for the added frontier.
        """
        frontier_id = next(self._counter)

        # Negate priority for max-heap behavior
        frontier = Frontier(
            priority=-priority,  # Negate for max-heap
            position=position,
            angle=angle,
            target=target,
            frontier_id=frontier_id
        )

        heapq.heappush(self._heap, frontier)
        self._entry_map[frontier_id] = frontier

        return frontier_id

    def pop(self) -> Optional[Frontier]:
        """
        Remove and return the frontier with highest priority.

        Returns:
            Frontier with highest priority, or None if queue is empty.
        """
        while self._heap:
            frontier = heapq.heappop(self._heap)

            # Check if this entry was marked as removed
            if frontier.frontier_id in self._entry_map:
                del self._entry_map[frontier.frontier_id]

                # Restore original priority (un-negate)
                frontier.priority = -frontier.priority

                return frontier

        return None

    def peek(self) -> Optional[Frontier]:
        """
        Return the frontier with highest priority without removing it.

        Returns:
            Frontier with highest priority, or None if queue is empty.
        """
        while self._heap:
            frontier = self._heap[0]

            # Check if this entry was marked as removed
            if frontier.frontier_id not in self._entry_map:
                heapq.heappop(self._heap)
                continue

            # Return a copy with restored priority
            return Frontier(
                priority=-frontier.priority,
                position=frontier.position,
                angle=frontier.angle,
                target=frontier.target,
                frontier_id=frontier.frontier_id
            )

        return None

    def update_priority(self, frontier_id: int, new_priority: float) -> bool:
        """
        Update the priority of an existing frontier.

        Args:
            frontier_id: ID of the frontier to update.
            new_priority: New priority value (higher = more important).

        Returns:
            True if update was successful, False if frontier not found.
        """
        if frontier_id not in self._entry_map:
            return False

        # Remove old entry (lazy deletion)
        old_frontier = self._entry_map[frontier_id]
        del self._entry_map[frontier_id]

        # Add new entry with updated priority
        new_frontier = Frontier(
            priority=-new_priority,  # Negate for max-heap
            position=old_frontier.position,
            angle=old_frontier.angle,
            target=old_frontier.target,
            frontier_id=frontier_id
        )

        heapq.heappush(self._heap, new_frontier)
        self._entry_map[frontier_id] = new_frontier

        return True

    def remove(self, frontier_id: int) -> bool:
        """
        Remove a specific frontier from the queue.

        Args:
            frontier_id: ID of the frontier to remove.

        Returns:
            True if removal was successful, False if frontier not found.
        """
        if frontier_id not in self._entry_map:
            return False

        # Lazy deletion: just remove from entry map
        del self._entry_map[frontier_id]
        return True

    def get_all_frontiers(self) -> List[Frontier]:
        """
        Get all active frontiers in the queue.

        Returns:
            List of all frontiers (unsorted), with original priorities.
        """
        frontiers = []
        for frontier in self._entry_map.values():
            frontiers.append(Frontier(
                priority=-frontier.priority,  # Restore original priority
                position=frontier.position,
                angle=frontier.angle,
                target=frontier.target,
                frontier_id=frontier.frontier_id
            ))
        return frontiers

    def clear(self):
        """Remove all frontiers from the queue."""
        self._heap.clear()
        self._entry_map.clear()

    def is_empty(self) -> bool:
        """Check if the queue is empty."""
        return len(self._entry_map) == 0

    def size(self) -> int:
        """Get the number of frontiers in the queue."""
        return len(self._entry_map)

    def max_priority(self) -> Optional[float]:
        """
        Get the highest priority value in the queue.

        Returns:
            Highest priority value, or None if queue is empty.
        """
        frontier = self.peek()
        return frontier.priority if frontier else None

    def __len__(self) -> int:
        """Get the number of frontiers in the queue."""
        return self.size()

    def __repr__(self) -> str:
        return f"FrontierQueue(size={self.size()}, max_priority={self.max_priority()})"
