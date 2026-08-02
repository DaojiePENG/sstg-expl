"""Tests for fail-closed four-world paired system-simulation analysis."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import pytest

from scripts.analyze_system_sim_paired import (
    EXTERNAL_METHOD,
    PairedAnalysisError,
    SSTG_METHOD,
    analyze_paired_runs,
)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _base_row(world_index: int, method: str, *, seed: int = 17) -> dict[str, str]:
    world = f"dev_world_{world_index:02d}"
    block = f"study__{world}__start__nominal__seed_{seed}"
    method_offset = 0.1 if method == SSTG_METHOD else 0.0
    return {
        "study_id": "study",
        "schedule_id": f"{block}__{method}",
        "block_id": block,
        "world_id": world,
        "site_family": f"family_{world_index}",
        "start_id": "start",
        "method": method,
        "condition": "nominal",
        "replicate_seed": str(seed),
        "run_output_dir": f"system_sim_outputs/runs/study/{block}__{method}",
        "formal_result_eligible": "false",
        "run_manifest_present": "true",
        "execution_status": "terminal_completed",
        "executed": "true",
        "task_completed": "true",
        "artifact_audit_valid": "true",
        "snapshot_present": "true",
        "snapshot_reason": "policy_session_settled",
        "evidence_error": "",
        "information_coverage": str(0.5 + method_offset + world_index * 0.01),
        "topological_coverage": str(0.3 + method_offset + world_index * 0.01),
        "target_recall_proxy": str(0.25 + method_offset),
        "ground_truth_travel_m": str(20.0 + world_index + method_offset),
        "mean_clearance_m": str(0.8 + method_offset),
        "minimum_clearance_m": str(0.2 + method_offset),
        "clearance_q05_m": str(0.3 + method_offset),
        "ate_mean_m": str(0.1 + method_offset),
        "ate_rmse_m": str(0.2 + method_offset),
        "ate_max_m": str(0.4 + method_offset),
        "collision_count": "1" if method == SSTG_METHOD else "0",
        "navigation_technical_failure_count": ("0" if method == SSTG_METHOD else "1"),
    }


def _fixture(
    tmp_path: Path, *, rows: list[dict[str, str]] | None = None
) -> tuple[Path, Path, list[dict[str, str]]]:
    root = tmp_path / "project"
    analysis = root / "system_sim_outputs/reports/study/analysis"
    analysis.mkdir(parents=True)
    if rows is None:
        rows = [
            _base_row(world_index, method)
            for world_index in range(1, 5)
            for method in (SSTG_METHOD, EXTERNAL_METHOD)
        ]
    source = analysis / "system_sim_runs.csv"
    with source.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    manifest = {
        "schema": "sstg_system_sim_analysis/v1",
        "study_id": "study",
        "evidence_source": "system_simulation",
        "formal_result_eligible": False,
        "counts": {"scheduled_runs": len(rows)},
        "outputs": {
            "system_sim_runs.csv": {
                "sha256": _sha(source),
                "bytes": source.stat().st_size,
            }
        },
    }
    (analysis / "analysis_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return root, source, rows


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def test_paired_analysis_retains_every_pair_and_hashes_outputs(
    tmp_path: Path,
) -> None:
    root, source, _ = _fixture(tmp_path)
    source_hash = _sha(source)
    output = root / "system_sim_outputs/reports/study/paired_analysis"

    manifest = analyze_paired_runs(root=root, input_csv=source, output_dir=output)

    assert _sha(source) == source_hash
    assert manifest["evidence_tier"] == "development_simulation"
    assert manifest["formal_result_eligible"] is False
    assert manifest["analysis_role"] == "descriptive_only"
    assert manifest["pair_key"] == [
        "world_id",
        "start_id",
        "condition",
        "replicate_seed",
    ]
    assert manifest["counts"] == {
        "input_runs": 8,
        "paired_runs": 4,
        "unique_worlds": 4,
    }
    pairs = _read_csv(output / "paired_run_deltas.csv")
    assert len(pairs) == 4
    assert all(
        row["delta_definition"] == "sstg_minus_frontier_mrtsp_dp_external"
        for row in pairs
    )
    assert all(
        float(row["delta_information_coverage"]) == pytest.approx(0.1) for row in pairs
    )
    assert all(row["delta_collision_count"] == "1" for row in pairs)
    assert all(row["delta_navigation_technical_failure_count"] == "-1" for row in pairs)

    report = json.loads((output / "paired_run_deltas.json").read_text())
    assert report["analysis_role"] == "descriptive_only"
    assert report["endpoint_policy"]["inference"].startswith("none")
    assert report["endpoint_policy"]["thresholds"].startswith("none")
    assert len(report["paired_runs"]) == 4
    assert report["descriptive_delta_summary"]["ate_rmse_m"]["n_pairs"] == 4
    assert (
        (output / "paired_endpoints.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    )
    for name, record in manifest["outputs"].items():
        path = output / name
        assert record == {"sha256": _sha(path), "bytes": path.stat().st_size}
    sidecar = (output / "paired_analysis_manifest.sha256").read_text().split()[0]
    assert sidecar == _sha(output / "paired_analysis_manifest.json")


def test_paired_analysis_is_deterministic(tmp_path: Path) -> None:
    root, source, _ = _fixture(tmp_path)
    output_a = root / "reports/a"
    output_b = root / "reports/b"

    manifest_a = analyze_paired_runs(root=root, input_csv=source, output_dir=output_a)
    manifest_b = analyze_paired_runs(root=root, input_csv=source, output_dir=output_b)

    assert manifest_a == manifest_b
    for name in manifest_a["outputs"]:
        assert (output_a / name).read_bytes() == (output_b / name).read_bytes()
    assert (output_a / "paired_analysis_manifest.json").read_bytes() == (
        output_b / "paired_analysis_manifest.json"
    ).read_bytes()


def test_paired_analysis_rejects_missing_method_without_writing(tmp_path: Path) -> None:
    rows = [
        _base_row(world_index, method)
        for world_index in range(1, 5)
        for method in (SSTG_METHOD, EXTERNAL_METHOD)
    ]
    rows.pop()
    root, source, _ = _fixture(tmp_path, rows=rows)
    output = root / "paired"

    with pytest.raises(PairedAnalysisError, match="incomplete frozen pair"):
        analyze_paired_runs(root=root, input_csv=source, output_dir=output)

    assert not output.exists()


def test_paired_analysis_rejects_duplicate_pair_member(tmp_path: Path) -> None:
    rows = [
        _base_row(world_index, method)
        for world_index in range(1, 5)
        for method in (SSTG_METHOD, EXTERNAL_METHOD)
    ]
    rows.append(dict(rows[0], schedule_id="duplicate"))
    root, source, _ = _fixture(tmp_path, rows=rows)

    with pytest.raises(PairedAnalysisError, match="duplicate sstg row"):
        analyze_paired_runs(root=root, input_csv=source, output_dir=root / "paired")


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("execution_status", "timeout", "incomplete pair member"),
        ("ate_rmse_m", "", "missing ate_rmse_m"),
    ],
)
def test_paired_analysis_rejects_incomplete_members_and_missing_endpoints(
    tmp_path: Path, field: str, value: str, message: str
) -> None:
    rows = [
        _base_row(world_index, method)
        for world_index in range(1, 5)
        for method in (SSTG_METHOD, EXTERNAL_METHOD)
    ]
    rows[0][field] = value
    root, source, _ = _fixture(tmp_path, rows=rows)

    with pytest.raises(PairedAnalysisError, match=message):
        analyze_paired_runs(root=root, input_csv=source, output_dir=root / "paired")


def test_paired_analysis_rejects_tampered_analyzer_output(tmp_path: Path) -> None:
    root, source, _ = _fixture(tmp_path)
    with source.open("a", encoding="utf-8") as stream:
        stream.write("\n")

    with pytest.raises(PairedAnalysisError, match="manifest hash/size"):
        analyze_paired_runs(root=root, input_csv=source, output_dir=root / "paired")


def test_paired_analysis_refuses_overwrite_and_analyzer_output_directory(
    tmp_path: Path,
) -> None:
    root, source, _ = _fixture(tmp_path)
    output = root / "paired"
    output.mkdir()

    with pytest.raises(PairedAnalysisError, match="refusing existing"):
        analyze_paired_runs(root=root, input_csv=source, output_dir=output)
    with pytest.raises(PairedAnalysisError, match="separate from the analyzer"):
        analyze_paired_runs(
            root=root,
            input_csv=source,
            output_dir=source.parent / "paired",
        )


def test_paired_analysis_requires_exactly_four_worlds(tmp_path: Path) -> None:
    rows = [
        _base_row(world_index, method)
        for world_index in range(1, 4)
        for method in (SSTG_METHOD, EXTERNAL_METHOD)
    ]
    root, source, _ = _fixture(tmp_path, rows=rows)

    with pytest.raises(PairedAnalysisError, match="exactly 4 worlds"):
        analyze_paired_runs(root=root, input_csv=source, output_dir=root / "paired")
