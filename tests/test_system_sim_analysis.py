"""Fail-closed tests for system-simulation evidence analysis."""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from scripts.analyze_system_sim_experiments import (
    AnalysisError,
    analyze_study,
)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, sort_keys=True) + "\n", encoding="utf-8"
    )


def _write_jsonl(path: Path, values: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(value, sort_keys=True) + "\n" for value in values),
        encoding="utf-8",
    )


def _snapshot(
    *,
    information: float,
    topological: float,
    dual: bool,
    target_recall: float,
    travel: float,
    nodes: int,
    raw_nodes: int,
    duplicates: int,
    collisions: int,
    collision_free: bool | None,
    reason: str = "policy_session_finished",
    clearance: bool = True,
    ate: bool = True,
) -> dict:
    value = {
        "schema": "sstg_system_sim_evaluator_snapshot/v2",
        "reason": reason,
        "ros_time_ns": 123456789,
        "coverage_endpoints": {
            "c_i_information": information,
            "c_t_topological": topological,
            "joint_min": min(information, topological),
            "dual_threshold_success": dual,
        },
        "topological": {
            "node_audit": {
                "unique_node_count": nodes,
                "raw_node_observation_count": raw_nodes,
                "duplicate_node_observation_count": duplicates,
            }
        },
        "targets": {
            "target_recall": target_recall,
            "target_total_count": 4,
            "detected_target_count": round(target_recall * 4),
        },
        "ground_truth_motion": {
            "ground_truth_path_length_m": travel,
            "ground_truth_sample_count": 100,
            "ate_sample_count": 80 if ate else 0,
            "ate_mean_m": 0.08 if ate else None,
            "ate_rmse_m": 0.10 if ate else None,
            "ate_max_m": 0.25 if ate else None,
        },
        "actions": {
            "navigation_goal_count": 6,
            "execution_count": 5,
            "navigation_success_count": 4,
        },
        "safety": {
            "collision_count": collisions,
            "collision_free": collision_free,
            "contact_message_count": 50,
            "maximum_reported_penetration_depth_m": (
                0.01 if collisions else None
            ),
        },
    }
    if clearance:
        value["static_clearance"] = {
            "footprint_clearance_mean_m": 0.40,
            "footprint_clearance_min_m": 0.12,
            "footprint_clearance_p05_m": 0.20,
        }
    return value


def _schedule_rows() -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for seed in (1, 2, 3):
        for order, method in enumerate(("sstg", "frontier"), start=1):
            schedule_id = f"study__world__start__nominal__seed_{seed}__{method}"
            rows.append({
                "schema": "sstg_system_sim_run_schedule/v2",
                "study_id": "study",
                "schedule_id": schedule_id,
                "block_id": f"study__world__start__nominal__seed_{seed}",
                "block_index": str(seed),
                "order_position": str(order),
                "world_id": "world_01",
                "site_family": "office",
                "start_id": "start",
                "method": method,
                "condition": "nominal",
                "replicate_seed": str(seed),
                "run_output_dir": f"system_sim_outputs/runs/study/{schedule_id}",
                "formal_result_eligible": "false",
            })
    return rows


def _write_run(
    root: Path,
    row: dict[str, str],
    schedule_sha: str,
    *,
    status: str,
    snapshot: dict | None,
) -> Path:
    run_dir = root / row["run_output_dir"]
    run_dir.mkdir(parents=True)
    files: dict[str, dict[str, object]] = {}
    if status == "terminal_completed":
        _write_json(
            run_dir / "policy_manifest.json",
            {
                "schema": "sstg_system_sim_policy_manifest/v1",
                "truth_access": False,
            },
        )
        terminal = {"event": "session_finished", "payload": {}}
        _write_jsonl(run_dir / "policy_trace.jsonl", [terminal])
        _write_json(
            run_dir / "evaluation_manifest.json",
            {
                "schema": "sstg_system_sim_evaluator_manifest/v2",
                "truth_access": "evaluator_only",
            },
        )
        _write_jsonl(
            run_dir / "evaluation_observed_policy_trace.jsonl", [terminal]
        )
        assert snapshot is not None
        _write_jsonl(
            run_dir / "evaluation_metrics.jsonl",
            [
                {
                    "event": "policy_trace_ingested",
                    "payload": {"event": "session_finished"},
                },
                {"event": "metrics_snapshot", "payload": snapshot},
                {
                    "event": "metrics_snapshot",
                    "payload": {**snapshot, "reason": "map_update"},
                },
            ],
        )
        (run_dir / "launch.log").write_text("launch complete\n", encoding="utf-8")
        for path in run_dir.iterdir():
            files[path.name] = {
                "sha256": _sha(path),
                "size_bytes": path.stat().st_size,
            }
        audit_valid = True
    else:
        if snapshot is not None:
            _write_jsonl(
                run_dir / "evaluation_metrics.jsonl",
                [{"event": "metrics_snapshot", "payload": snapshot}],
            )
            path = run_dir / "evaluation_metrics.jsonl"
            files[path.name] = {
                "sha256": _sha(path),
                "size_bytes": path.stat().st_size,
            }
        (run_dir / "launch.log").write_text("timed out\n", encoding="utf-8")
        files["launch.log"] = {
            "sha256": _sha(run_dir / "launch.log"),
            "size_bytes": (run_dir / "launch.log").stat().st_size,
        }
        audit_valid = False
    manifest = {
        "schema": "sstg_system_sim_run_launch/v1",
        "study_id": "study",
        "schedule_id": row["schedule_id"],
        "schedule_sha256": schedule_sha,
        "execution": {
            "status": status,
            "artifact_audit": {
                "valid": audit_valid,
                "errors": [] if audit_valid else ["incomplete"],
                "files": files,
            },
        },
    }
    path = run_dir / "run_launch_manifest.yaml"
    path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return run_dir


def _fixture_project(tmp_path: Path) -> dict[str, object]:
    root = tmp_path / "project"
    study = root / "experiments/system_sim/studies/study"
    study.mkdir(parents=True)
    rows = _schedule_rows()
    schedule = study / "run_schedule.csv"
    with schedule.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    schedule_sha = _sha(schedule)
    freeze = {
        "schema": "sstg_system_sim_schedule_freeze/v2",
        "study_id": "study",
        "eligibility": {
            "evidence_tier": "development",
            "formal_result_eligible": False,
        },
        "outputs": {
            "run_schedule": "run_schedule.csv",
            "run_schedule_sha256": schedule_sha,
        },
    }
    (study / "schedule_freeze_manifest.yaml").write_text(
        yaml.safe_dump(freeze, sort_keys=False), encoding="utf-8"
    )
    indexed = {(row["method"], int(row["replicate_seed"])): row for row in rows}
    _write_run(
        root,
        indexed[("sstg", 1)],
        schedule_sha,
        status="terminal_completed",
        snapshot=_snapshot(
            information=0.9,
            topological=0.8,
            dual=True,
            target_recall=0.75,
            travel=10.0,
            nodes=5,
            raw_nodes=6,
            duplicates=1,
            collisions=0,
            collision_free=True,
            clearance=False,
        ),
    )
    _write_run(
        root,
        indexed[("sstg", 2)],
        schedule_sha,
        status="timeout",
        snapshot=_snapshot(
            information=0.5,
            topological=0.4,
            dual=False,
            target_recall=0.25,
            travel=6.0,
            nodes=3,
            raw_nodes=4,
            duplicates=1,
            collisions=1,
            collision_free=False,
            reason="periodic",
            ate=False,
        ),
    )
    for seed in (1, 2, 3):
        _write_run(
            root,
            indexed[("frontier", seed)],
            schedule_sha,
            status="terminal_completed",
            snapshot=_snapshot(
                information=0.6 + seed * 0.02,
                topological=0.55 + seed * 0.02,
                dual=seed == 3,
                target_recall=0.5,
                travel=12.0 + seed,
                nodes=7 + seed,
                raw_nodes=10,
                duplicates=2,
                collisions=0,
                collision_free=None if seed == 2 else True,
                clearance=seed != 1,
                ate=seed != 2,
            ),
        )
    return {"root": root, "study": study, "rows": rows}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def test_analysis_retains_all_runs_and_separates_success_constructs(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    study = project["study"]
    assert isinstance(root, Path) and isinstance(study, Path)
    output = root / "system_sim_outputs/reports/study/analysis"

    manifest = analyze_study(
        root=root,
        study_dir=study,
        output_dir=output,
        bootstrap_resamples=500,
        bootstrap_seed=17,
    )

    runs = _read_csv(output / "system_sim_runs.csv")
    aggregates = _read_csv(output / "system_sim_method_aggregate.csv")
    assert len(runs) == 6
    assert manifest["counts"]["scheduled_runs"] == 6
    assert manifest["counts"]["terminal_completed_runs"] == 4
    assert manifest["counts"]["not_executed_runs"] == 1
    unexecuted = next(row for row in runs if row["execution_status"] == "not_executed")
    assert unexecuted["task_completed"] == "false"
    assert unexecuted["information_coverage"] == ""
    timeout = next(row for row in runs if row["execution_status"] == "timeout")
    assert timeout["task_completed"] == "false"
    assert timeout["dual_threshold_success"] == "false"
    assert timeout["collision_free"] == "false"
    assert timeout["information_coverage"] == "0.5"
    sstg = next(row for row in aggregates if row["method"] == "sstg")
    assert float(sstg["task_completion_mean"]) == pytest.approx(1.0 / 3.0)
    assert float(sstg["dual_success_mean"]) == pytest.approx(0.5)
    assert float(sstg["collision_free_mean"]) == pytest.approx(0.5)
    assert sstg["task_completion_n_runs"] == "3"
    assert sstg["collision_free_n_runs"] == "2"
    assert sstg["minimum_clearance_m_n_runs"] == "1"
    table = (output / "system_sim_main_table.tex").read_text(encoding="utf-8")
    assert "Task completion" in table
    assert "dual-threshold success" in table
    assert "Target recall is a deterministic geometry proxy" in table


def test_analysis_is_deterministic_and_hashes_every_output(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    study = project["study"]
    assert isinstance(root, Path) and isinstance(study, Path)
    output_a = root / "outputs/a"
    output_b = root / "outputs/b"

    manifest_a = analyze_study(
        root=root,
        study_dir=study,
        output_dir=output_a,
        bootstrap_resamples=300,
        bootstrap_seed=91,
    )
    manifest_b = analyze_study(
        root=root,
        study_dir=study,
        output_dir=output_b,
        bootstrap_resamples=300,
        bootstrap_seed=91,
    )

    assert manifest_a == manifest_b
    for name, record in manifest_a["outputs"].items():
        assert _sha(output_a / name) == record["sha256"]
        assert (output_a / name).read_bytes() == (output_b / name).read_bytes()
    manifest_sha = _sha(output_a / "analysis_manifest.json")
    sidecar = (output_a / "analysis_manifest.sha256").read_text().split()[0]
    assert sidecar == manifest_sha


def test_schedule_hash_tampering_fails_before_writing_outputs(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    study = project["study"]
    assert isinstance(root, Path) and isinstance(study, Path)
    schedule = study / "run_schedule.csv"
    schedule.write_text(schedule.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    output = root / "outputs/tampered"

    with pytest.raises(AnalysisError, match="schedule hash disagrees"):
        analyze_study(root=root, study_dir=study, output_dir=output)
    assert not output.exists()


def test_completed_artifact_hash_mismatch_fails_closed(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    study = project["study"]
    rows = project["rows"]
    assert isinstance(root, Path) and isinstance(study, Path)
    assert isinstance(rows, list)
    row = next(
        row for row in rows
        if row["method"] == "sstg" and row["replicate_seed"] == "1"
    )
    trace = root / row["run_output_dir"] / "policy_trace.jsonl"
    trace.write_text(trace.read_text() + "{}\n", encoding="utf-8")
    output = root / "outputs/hash_failure"

    with pytest.raises(AnalysisError, match="hash_mismatch:policy_trace"):
        analyze_study(root=root, study_dir=study, output_dir=output)
    assert not output.exists()


def test_failed_run_with_untrusted_partial_snapshot_keeps_na_metrics(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    study = project["study"]
    rows = project["rows"]
    assert isinstance(root, Path) and isinstance(study, Path)
    assert isinstance(rows, list)
    row = next(
        row for row in rows
        if row["method"] == "sstg" and row["replicate_seed"] == "2"
    )
    metrics = root / row["run_output_dir"] / "evaluation_metrics.jsonl"
    metrics.write_text(metrics.read_text() + "{}\n", encoding="utf-8")
    output = root / "outputs/partial_hash_failure"

    analyze_study(root=root, study_dir=study, output_dir=output)

    run = next(
        item for item in _read_csv(output / "system_sim_runs.csv")
        if item["schedule_id"] == row["schedule_id"]
    )
    assert run["execution_status"] == "timeout"
    assert run["information_coverage"] == ""
    assert "hash_mismatch:evaluation_metrics.jsonl" in run["evidence_error"]


def test_completed_run_without_terminal_snapshot_fails_closed(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    study = project["study"]
    rows = project["rows"]
    assert isinstance(root, Path) and isinstance(study, Path)
    assert isinstance(rows, list)
    row = next(
        row for row in rows
        if row["method"] == "sstg" and row["replicate_seed"] == "1"
    )
    run_dir = root / row["run_output_dir"]
    metrics = run_dir / "evaluation_metrics.jsonl"
    records = [json.loads(line) for line in metrics.read_text().splitlines()]
    for record in records:
        if (
            record.get("event") == "metrics_snapshot"
            and record.get("payload", {}).get("reason")
            == "policy_session_finished"
        ):
            record["payload"]["reason"] = "periodic"
    _write_jsonl(metrics, records)
    manifest_path = run_dir / "run_launch_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["execution"]["artifact_audit"]["files"][
        "evaluation_metrics.jsonl"
    ]["sha256"] = _sha(metrics)
    manifest_path.write_text(yaml.safe_dump(manifest), encoding="utf-8")

    with pytest.raises(AnalysisError, match="lacks final evaluator snapshot"):
        analyze_study(
            root=root,
            study_dir=study,
            output_dir=root / "outputs/no_terminal",
        )


def test_nonempty_analysis_output_is_refused_by_default(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    study = project["study"]
    assert isinstance(root, Path) and isinstance(study, Path)
    output = root / "outputs/existing"
    output.mkdir(parents=True)
    (output / "keep.txt").write_text("user artifact\n", encoding="utf-8")

    with pytest.raises(AnalysisError, match="non-empty analysis output"):
        analyze_study(root=root, study_dir=study, output_dir=output)
    assert (output / "keep.txt").read_text() == "user artifact\n"
