"""Pure validation and geometry helpers shared by baseline adapters."""
from __future__ import annotations

import math
from typing import Any, Iterable, Sequence


BUDGET_FIELDS = (
    "max_duration_s",
    "max_distance_m",
    "max_decisions",
    "goal_timeout_s",
)


def validate_budget(values: dict[str, Any]) -> dict[str, float | int]:
    """Normalize the fail-closed experiment budget used by every method."""
    missing = [field for field in BUDGET_FIELDS if field not in values]
    if missing:
        raise ValueError(f"experiment budget is missing: {', '.join(missing)}")
    normalized: dict[str, float | int] = {}
    for field in ("max_duration_s", "max_distance_m", "goal_timeout_s"):
        value = values[field]
        if isinstance(value, bool):
            raise ValueError(f"{field} must be a finite positive number")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{field} must be a finite positive number"
            ) from error
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{field} must be a finite positive number")
        normalized[field] = number
    decisions = values["max_decisions"]
    if isinstance(decisions, bool) or not isinstance(decisions, int):
        raise ValueError("max_decisions must be a positive integer")
    if decisions <= 0:
        raise ValueError("max_decisions must be a positive integer")
    normalized["max_decisions"] = decisions
    if normalized["goal_timeout_s"] > normalized["max_duration_s"]:
        raise ValueError("goal_timeout_s must not exceed max_duration_s")
    return normalized


def finite_pose(values: Sequence[Any], label: str = "pose") -> tuple[float, ...]:
    """Return a finite numeric pose tuple without silently accepting booleans."""
    result = []
    for index, value in enumerate(values):
        if isinstance(value, bool):
            raise ValueError(f"{label}[{index}] must be finite")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{label}[{index}] must be finite") from error
        if not math.isfinite(number):
            raise ValueError(f"{label}[{index}] must be finite")
        result.append(number)
    return tuple(result)


def path_length(points: Iterable[Sequence[Any]]) -> float:
    """Compute planar polyline length from finite x/y samples."""
    normalized = [finite_pose(point[:2], "path point") for point in points]
    return float(sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(normalized[:-1], normalized[1:])
    ))


def shortest_rotation_deg(start_yaw: float, end_yaw: float) -> float:
    """Absolute shortest planar rotation between two finite yaws."""
    start, end = finite_pose((start_yaw, end_yaw), "yaw")
    delta = math.atan2(math.sin(end - start), math.cos(end - start))
    return math.degrees(abs(delta))


def jsonable(value: Any) -> Any:
    """Convert trace values to strict JSON-compatible content."""
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    if hasattr(value, "item"):
        return jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value
