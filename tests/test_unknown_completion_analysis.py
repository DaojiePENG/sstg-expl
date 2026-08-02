import csv
import hashlib
import json
from pathlib import Path

import yaml

from scripts import analyze_unknown_completion as analysis


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_run(root: Path, method: str, order: int) -> dict[str, str]:
    run = root / f"run_{method}"
    run.mkdir()
    policy = [
        {"event": "decision", "ros_time_ns": 5, "payload": {"decision_id": 1}},
        {"event": "execution", "ros_time_ns": 10, "payload": {"decision_id": 1, "succeeded": True}},
        {"event": "session_finished", "ros_time_ns": 20, "payload": {
            "termination_reason": "candidate_exhaustion",
            "native_termination_rule": f"no_{method}_candidate",
            "exhaustion_confirmation": 3,
            "exhaustion_confirmations_required": 3,
        }},
    ]
    metrics = []
    for time, distance, sensor, topology, reason in (
        (0, 0.0, .2, .1, "periodic"),
        (10, 4.0 + order, .96, .95, "policy_execution"),
        (20, 4.0 + order, .97, .96, "policy_session_settled"),
    ):
        metrics.append({"event": "metrics_snapshot", "ros_time_ns": time, "payload": {
            "reason": reason,
            "core_policy_endpoints": {
                "c_i_truth_sensor": sensor,
                "c_t_truth_endpoints": topology,
                "coverage_distance_auc_normalized": .7 + order / 100,
            },
            "core_policy": {"truth_topological": {"endpoint_audit": {
                "unique_endpoint_count": order,
                "raw_endpoint_observation_count": order,
                "redundant_endpoint_fraction": 0.0,
            }}},
            "ground_truth_motion": {"ground_truth_path_length_m": distance},
        }})
    policy_path = run / "policy_trace.jsonl"
    metrics_path = run / "evaluation_metrics.jsonl"
    policy_path.write_text("".join(json.dumps(item) + "\n" for item in policy))
    metrics_path.write_text("".join(json.dumps(item) + "\n" for item in metrics))
    manifest = {
        "schedule_id": f"schedule_{method}",
        "execution": {"status": "terminal_completed", "artifact_audit": {
            "valid": True,
            "files": {
                "policy_trace.jsonl": {"sha256": _sha256(policy_path)},
                "evaluation_metrics.jsonl": {"sha256": _sha256(metrics_path)},
            },
        }},
    }
    (run / "run_launch_manifest.yaml").write_text(yaml.safe_dump(manifest))
    return {
        "study_id": "focused_test", "schedule_id": f"schedule_{method}",
        "method": method, "order_position": str(order),
        "run_output_dir": str(run.relative_to(analysis.ROOT)),
    }


def test_focused_unknown_completion_report(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(analysis, "ROOT", tmp_path)
    schedule = tmp_path / "study"
    schedule.mkdir()
    rows = [
        _write_run(tmp_path, method, order)
        for order, method in enumerate(sorted(analysis.EXPECTED_METHODS), 1)
    ]
    with (schedule / "run_schedule.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    protocol = tmp_path / "protocol.yaml"
    protocol.write_text(yaml.safe_dump({
        "evaluator_contract": {
            "sensor_success_threshold": .95,
            "topological_success_threshold": .95,
        },
        "outputs": {"root": "system_sim_outputs/unknown_completion"},
    }))
    output = tmp_path / "report"

    result = analysis.analyze_study(schedule, protocol_path=protocol, output_dir=output)

    assert result["schema"] == analysis.REPORT_SCHEMA
    assert (output / "procedural_equivalent_comparison.png").stat().st_size > 1000
    summary = list(csv.DictReader((output / "summary.csv").open()))
    assert len(summary) == 5
    assert all(row["equivalent_95_95_reached"] == "true" for row in summary)
    assert all(row["native_exhaustion_confirmed"] == "true" for row in summary)
    assert all(row["redundant_endpoint_fraction_at_95_95"] == "0.0" for row in summary)
    conclusion = (output / "CONCLUSION.md").read_text()
    assert "核心算法比较" in conclusion
    assert "现实终止诊断" in conclusion


def test_report_rejects_post_audit_trace_mutation(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(analysis, "ROOT", tmp_path)
    row = _write_run(tmp_path, "sstg", 1)
    run = tmp_path / row["run_output_dir"]
    with (run / "policy_trace.jsonl").open("a") as stream:
        stream.write(json.dumps({"event": "tamper", "ros_time_ns": 21, "payload": {}}) + "\n")
    manifest = yaml.safe_load((run / "run_launch_manifest.yaml").read_text())

    try:
        analysis.analyze_run(row, sensor_threshold=.95, topological_threshold=.95)
    except analysis.AnalysisError as error:
        assert "changed after artifact audit" in str(error)
    else:
        raise AssertionError("post-audit mutation was accepted")
