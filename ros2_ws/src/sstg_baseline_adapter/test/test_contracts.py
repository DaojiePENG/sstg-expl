import math

import pytest

from sstg_baseline_adapter.contracts import (
    jsonable,
    path_length,
    shortest_rotation_deg,
    validate_budget,
)


def test_validate_budget_matches_common_contract():
    assert validate_budget({
        "max_duration_s": 120,
        "max_distance_m": 20,
        "max_decisions": 4,
        "goal_timeout_s": 60,
    }) == {
        "max_duration_s": 120.0,
        "max_distance_m": 20.0,
        "max_decisions": 4,
        "goal_timeout_s": 60.0,
    }


@pytest.mark.parametrize(
    "field,value",
    [
        ("max_duration_s", 0),
        ("max_distance_m", math.inf),
        ("max_decisions", True),
        ("goal_timeout_s", -1),
    ],
)
def test_validate_budget_rejects_disabled_or_malformed_limits(field, value):
    budget = {
        "max_duration_s": 120.0,
        "max_distance_m": 20.0,
        "max_decisions": 4,
        "goal_timeout_s": 60.0,
    }
    budget[field] = value
    with pytest.raises(ValueError):
        validate_budget(budget)


def test_validate_budget_rejects_goal_timeout_beyond_session():
    with pytest.raises(ValueError, match="must not exceed"):
        validate_budget({
            "max_duration_s": 30.0,
            "max_distance_m": 20.0,
            "max_decisions": 4,
            "goal_timeout_s": 31.0,
        })


def test_path_and_rotation_helpers_are_planar_and_wrapped():
    assert path_length([(0, 0), (3, 4), (3, 8)]) == pytest.approx(9.0)
    assert shortest_rotation_deg(math.radians(170), math.radians(-170)) \
        == pytest.approx(20.0)


def test_jsonable_replaces_nonfinite_values_without_mutating_shape():
    assert jsonable({"x": [1.0, math.nan], "y": (math.inf,)}) == {
        "x": [1.0, None],
        "y": [None],
    }
