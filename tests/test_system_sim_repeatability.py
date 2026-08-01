"""Tests for the descriptive same-seed Gazebo repeatability audit."""
from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from scripts.analyze_system_sim_repeatability import (
    RepeatabilityError,
    analyze_repeatability,
    sha256_file,
)


def _write_jsonl(path: Path, records: list[dict]) -> None:
    path.write_text(
        "".join(json.dumps(record, separators=(",", ":")) + "\n" for record in records),
        encoding="utf-8",
    )


def _make_run(
    root: Path,
    *,
    label: str,
    first_key: list,
    known_free_cells: int,
    target: tuple[float, float],
    information: float,
    topological: float,
    target_recall: float,
    travel_m: float,
    ate_rmse_m: float,
    source_tree: str = "1" * 64,
) -> Path:
    schedule_dir = root / "experiments/system_sim/studies" / label
    schedule_dir.mkdir(parents=True)
    freeze = {
        "schema": "sstg_system_sim_schedule_freeze/v2",
        "seed_contract": {
            "seed_source": "replicate_seed",
            "valid_range_inclusive": [1, 2147483647],
            "launch_argument_columns": {
                "policy_seed": "replicate_seed",
                "simulation_seed": "replicate_seed",
            },
        },
        "experiment_budget": {
            "max_duration_s": 120.0,
            "max_distance_m": 20.0,
            "max_decisions": 2,
            "goal_timeout_s": 60.0,
        },
        "source": {
            "repository_commit": "a" * 40,
            "repository_dirty": False,
            "source_tree_sha256": source_tree,
        },
        "inputs": {
            "shared_stack": {"sha256": "2" * 64},
            "condition": {"sha256": "3" * 64},
            "world_registry": {"sha256": "4" * 64},
            "methods": [{"method": "sstg", "sha256": "5" * 64}],
            "worlds": [
                {
                    "world_id": "office",
                    "sha256": {"bundle": "6" * 64},
                }
            ],
        },
    }
    (schedule_dir / "schedule_freeze_manifest.yaml").write_text(
        yaml.safe_dump(freeze, sort_keys=False), encoding="utf-8"
    )

    run = root / "system_sim_outputs/runs" / label / "run"
    run.mkdir(parents=True)
    decision = {
        "event": "decision",
        "map_revision": 3,
        "ros_time_ns": 3_500_000_000,
        "payload": {
            "known_free_cells": known_free_cells,
            "selected_candidate": {"execution_key": first_key},
            "target_pose": [target[0], target[1], 0.0],
        },
    }
    finished = {
        "event": "session_finished",
        "payload": {"decisions_issued": 2},
    }
    _write_jsonl(run / "policy_trace.jsonl", [decision, finished])
    snapshot = {
        "event": "metrics_snapshot",
        "payload": {
            "reason": "policy_session_settled",
            "coverage_endpoints": {
                "c_i_information": information,
                "c_t_topological": topological,
                "joint_min": min(information, topological),
            },
            "targets": {"target_recall": target_recall},
            "ground_truth_motion": {
                "ground_truth_path_length_m": travel_m,
                "ate_rmse_m": ate_rmse_m,
            },
            "actions": {
                "navigation_goal_count": 2,
                "navigation_success_count": 2,
            },
            "safety": {"collision_count": 0},
        },
    }
    _write_jsonl(run / "evaluation_metrics.jsonl", [snapshot])
    (run / "launch.log").write_text(
        "[gazebo] Setting seed value: 7\n", encoding="utf-8"
    )
    artifact_files = {
        name: {"sha256": sha256_file(run / name)}
        for name in ("policy_trace.jsonl", "evaluation_metrics.jsonl", "launch.log")
    }
    manifest = {
        "schema": "sstg_system_sim_run_launch/v1",
        "study_id": label,
        "schedule_id": f"{label}__seed_7",
        "schedule_dir": schedule_dir.relative_to(root).as_posix(),
        "launch": {
            "arguments": {
                "policy_seed": "7",
                "simulation_seed": "7",
            }
        },
        "identity": {
            "world_id": "office",
            "start_id": "start_1",
            "method": "sstg",
            "condition": "nominal",
            "replicate_seed": "7",
        },
        "execution": {
            "status": "terminal_completed",
            "artifact_audit": {
                "valid": True,
                "files": artifact_files,
            },
        },
    }
    (run / "run_launch_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )
    return run


def _pair(root: Path) -> tuple[Path, Path]:
    first = _make_run(
        root,
        label="repeat_a",
        first_key=[0.1, 0.2, 45, "frontier"],
        known_free_cells=1400,
        target=(0.1, 0.2),
        information=0.2,
        topological=0.1,
        target_recall=0.25,
        travel_m=3.0,
        ate_rmse_m=0.01,
    )
    second = _make_run(
        root,
        label="repeat_b",
        first_key=[0.7, 0.1, 315, "frontier"],
        known_free_cells=1450,
        target=(0.7, 0.1),
        information=0.3,
        topological=0.15,
        target_recall=0.0,
        travel_m=4.0,
        ate_rmse_m=0.02,
    )
    return first, second


def test_repeatability_report_is_descriptive_hashed_and_non_overwriting(
    tmp_path: Path,
) -> None:
    first, second = _pair(tmp_path)
    output = tmp_path / "system_sim_outputs/reports/repeatability"

    result = analyze_repeatability(
        root=tmp_path,
        run_a=first,
        run_b=second,
        output_dir=output,
    )

    comparison = result["comparison"]
    assert comparison["seed_control_attested"] is True
    assert comparison["same_first_execution_key"] is False
    assert comparison["terminal_outcome_agreement"] is True
    assert comparison["tolerance_verdict"].startswith("not_applicable")
    assert (output / "repeatability_runs.csv").is_file()
    assert (output / "repeatability_deltas.csv").is_file()
    assert (output / "repeatability_figure.png").read_bytes().startswith(b"\x89PNG")
    manifest = json.loads((output / "analysis_manifest.json").read_text())
    assert len(manifest["outputs"]) == 4

    with pytest.raises(RepeatabilityError, match="refusing existing output"):
        analyze_repeatability(
            root=tmp_path,
            run_a=first,
            run_b=second,
            output_dir=output,
        )


def test_repeatability_rejects_artifact_tampering_before_output(tmp_path: Path) -> None:
    first, second = _pair(tmp_path)
    (second / "policy_trace.jsonl").write_text("tampered\n", encoding="utf-8")
    output = tmp_path / "system_sim_outputs/reports/tampered"

    with pytest.raises(RepeatabilityError, match="artifact hash mismatch"):
        analyze_repeatability(
            root=tmp_path,
            run_a=first,
            run_b=second,
            output_dir=output,
        )

    assert not output.exists()


def test_repeatability_rejects_unmatched_source_fingerprint(tmp_path: Path) -> None:
    first, second = _pair(tmp_path)
    freeze_path = (
        tmp_path
        / "experiments/system_sim/studies/repeat_b/schedule_freeze_manifest.yaml"
    )
    freeze = yaml.safe_load(freeze_path.read_text(encoding="utf-8"))
    freeze["source"]["source_tree_sha256"] = "9" * 64
    freeze_path.write_text(yaml.safe_dump(freeze, sort_keys=False), encoding="utf-8")
    output = tmp_path / "system_sim_outputs/reports/unmatched"

    with pytest.raises(RepeatabilityError, match="source_tree_sha256"):
        analyze_repeatability(
            root=tmp_path,
            run_a=first,
            run_b=second,
            output_dir=output,
        )

    assert not output.exists()
