"""Tests for descriptive ROS 2 system-simulation localization diagnostics."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.plot_system_sim_localization_diagnostics import (
    LocalizationDiagnosticError,
    TransformSample,
    collapse_transform_samples,
    largest_transform_correction,
    read_ate_samples,
)


def _write_metrics(path: Path, records: list[dict]) -> None:
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records), encoding="utf-8"
    )


def test_ate_reader_retains_available_cumulative_snapshots(tmp_path: Path) -> None:
    metrics = tmp_path / "evaluation_metrics.jsonl"
    _write_metrics(
        metrics,
        [
            {"event": "session_started"},
            {
                "event": "metrics_snapshot",
                "ros_time_ns": 1_500_000_000,
                "map_revision": 2,
                "payload": {
                    "ground_truth_motion": {
                        "ate_status": "available",
                        "ate_sample_count": 12,
                        "ate_mean_m": 0.1,
                        "ate_rmse_m": 0.2,
                        "ate_max_m": 0.4,
                    }
                },
            },
        ],
    )

    samples = read_ate_samples(metrics)

    assert len(samples) == 1
    assert samples[0].ros_time_s == 1.5
    assert samples[0].map_revision == 2
    assert samples[0].sample_count == 12
    assert samples[0].rmse_m == pytest.approx(0.2)


def test_ate_reader_rejects_time_reversal(tmp_path: Path) -> None:
    metrics = tmp_path / "evaluation_metrics.jsonl"
    records = []
    for time_ns in (2_000_000_000, 1_000_000_000):
        records.append(
            {
                "event": "metrics_snapshot",
                "ros_time_ns": time_ns,
                "map_revision": 1,
                "payload": {
                    "ground_truth_motion": {
                        "ate_status": "available",
                        "ate_sample_count": 1,
                        "ate_mean_m": 0.1,
                        "ate_rmse_m": 0.1,
                        "ate_max_m": 0.1,
                    }
                },
            }
        )
    _write_metrics(metrics, records)

    with pytest.raises(LocalizationDiagnosticError, match="not time ordered"):
        read_ate_samples(metrics)


def test_transform_collapse_and_largest_correction_are_descriptive() -> None:
    samples = (
        TransformSample(0.0, 0.0, 0.0, 0.0, 0.0),
        TransformSample(0.1, 0.1, 0.0, 0.0, 0.0),
        TransformSample(1.0, 0.9, 0.1, 0.0, 0.02),
        TransformSample(1.0, 0.92, -0.1, -2.0, -0.01),
        TransformSample(2.0, 1.9, -0.1, -2.0, -0.01),
    )

    collapsed = collapse_transform_samples(samples)
    correction = largest_transform_correction(collapsed)

    assert len(collapsed) == 4
    assert correction.header_time_s == 1.0
    assert correction.translation_m == pytest.approx((0.2**2 + 2.0**2) ** 0.5)
    assert correction.before_y_m == 0.0
    assert correction.after_y_m == -2.0


def test_transform_collapse_requires_evidence() -> None:
    with pytest.raises(LocalizationDiagnosticError, match="no samples"):
        collapse_transform_samples(())
