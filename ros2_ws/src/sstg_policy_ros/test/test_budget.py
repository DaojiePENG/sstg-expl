import pytest

from sstg_policy_ros.policy_node import _validated_policy_budget


VALID_BUDGET = {
    "max_duration_s": 900.0,
    "max_distance_m": 150.0,
    "max_decisions": 100,
    "goal_timeout_s": 180.0,
}


def test_policy_budget_accepts_finite_positive_limits():
    assert _validated_policy_budget(VALID_BUDGET) == VALID_BUDGET


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("max_duration_s", 0.0),
        ("max_distance_m", float("inf")),
        ("goal_timeout_s", -1.0),
        ("max_decisions", 1.5),
        ("max_decisions", True),
    ],
)
def test_policy_budget_rejects_disabled_or_malformed_limits(field, value):
    budget = {**VALID_BUDGET, field: value}
    with pytest.raises(ValueError):
        _validated_policy_budget(budget)


def test_policy_budget_requires_goal_timeout_within_session_duration():
    budget = {**VALID_BUDGET, "max_duration_s": 30.0}
    with pytest.raises(ValueError, match="must not exceed"):
        _validated_policy_budget(budget)
