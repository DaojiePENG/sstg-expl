#!/usr/bin/env python3
"""Fail-fast audit of manuscript claims against the frozen benchmark release.

This checker intentionally covers the high-impact numerical claims and artifact
cardinalities used by the paper.  It does not replace peer review or assess whether
the experimental design is scientifically sufficient.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path


DEFAULT_RELEASE = Path("outputs/joint_benchmark_selected/20260719_223630")


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _row(rows: list[dict[str, str]], **keys: str) -> dict[str, str]:
    matches = [row for row in rows if all(row[key] == value for key, value in keys.items())]
    if len(matches) != 1:
        raise AssertionError(f"expected one row for {keys}, found {len(matches)}")
    return matches[0]


def _close(actual: str | float, expected: float, tolerance: float = 5e-9) -> None:
    value = float(actual)
    if abs(value - expected) > tolerance:
        raise AssertionError(f"{value} != {expected} (tol={tolerance})")


def _contains(text: str, fragment: str) -> None:
    if fragment not in text:
        raise AssertionError(f"manuscript is missing audited fragment: {fragment}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def audit(repo: Path, paper: Path, release: Path) -> dict[str, int | bool]:
    release = release if release.is_absolute() else repo / release
    root_tex = paper / "root.tex"
    manuscript = root_tex.read_text(encoding="utf-8")

    audit_report = json.loads((release / "audit_report.json").read_text(encoding="utf-8"))
    assert audit_report["passed"] is True
    assert audit_report["records"] == 810
    assert audit_report["file_counts"] == {
        "run_json": 810,
        "belief_npy": 810,
        "decision_csv": 810,
        "candidate_csv": 810,
        "trajectory_csv": 810,
        "oriented_views_csv": 810,
        "step_png": 6270,
        "gif": 270,
        "mp4": 270,
    }
    for key in (
        "missing_files",
        "belief_replay_mismatches",
        "truth_mismatches",
        "media_errors",
        "html_missing_references",
        "log_error_markers",
    ):
        assert audit_report[key] == [], key

    aggregate = _rows(release / "aggregate.csv")
    sstg = _row(aggregate, algorithm="SSTG-Explorer Joint")
    expected_sstg = {
        "experiments": 162.0,
        "sensor_coverage_mean": 0.9999006792013613,
        "topological_coverage_mean": 0.9613643566599842,
        "distance_mean": 63.99456155301911,
        "nodes_mean": 15.75925925925926,
        "oriented_views_mean": 17.296296296296298,
        "view_clearance_mean": 0.9499252348851958,
        "mean_nn_distance": 2.1525337335055297,
        "redundant_viewpoint_fraction": 0.0007407407407407407,
        "success_rate": 1.0,
    }
    for key, value in expected_sstg.items():
        _close(sstg[key], value)

    protocols = _rows(release / "three_protocol_comparison.csv")
    known = _row(protocols, protocol_case="known_map_topological", algorithm_key="sstg")
    sensor = _row(protocols, protocol_case="unknown_sensor_only", algorithm_key="sstg")
    joint = _row(protocols, protocol_case="unknown_joint_topological", algorithm_key="sstg")
    _close(known["topological_coverage_mean"], 0.985246985530747)
    _close(sensor["sensor_coverage_mean"], 0.9838390994538718)
    _close(sensor["topological_coverage_mean"], 0.33020359866274557)
    _close(joint["sensor_coverage_mean"], 0.9999006792013613)
    _close(joint["topological_coverage_mean"], 0.9613643566599842)

    effects = _rows(release / "pairwise_all_metrics.csv")
    redundancy_expected = {
        "ANS-Global Joint (adapted)": (-4.378470613226602, 1.798875018304773e-05),
        "Frontier Joint": (-29.5633664323599, 1.798875018304773e-05),
        "NBV Joint": (-14.95529453073516, 1.798875018304773e-05),
        "RRT Joint (adapted)": (-3.4120335359622835, 1.798875018304773e-05),
    }
    for baseline, (delta, upper_p) in redundancy_expected.items():
        row = _row(effects, metric="redundant_nodes_pp", baseline=baseline)
        _close(row["delta_sstg_minus_baseline"], delta)
        assert float(row["holm_p"]) <= upper_p + 1e-15

    hard = _rows(release / "hard_scene_analysis.csv")
    hard_expected = {
        "multiple_rooms": (0.9547114524536566, 90.86794596308185, 1.0),
        "dense_obstacles": (0.9615523465703971, 55.60310442006331, 1.0),
        "warehouse": (0.9560828797114203, 110.26672014744264, 1.0),
    }
    for environment, (coverage, distance, success) in hard_expected.items():
        row = _row(hard, algorithm="SSTG-Explorer Joint", environment=environment)
        _close(row["topological_coverage_mean"], coverage)
        _close(row["distance_mean_m"], distance)
        _close(row["success_rate"], success)

    closure = _row(
        _rows(release / "post_sensor_gap_closure_aggregate.csv"),
        algorithm="SSTG-Explorer Joint",
    )
    closure_expected = {
        "topology_when_sensor_reached_95_mean": 0.4830745939387145,
        "topological_gain_after_sensor_95_mean": 0.4782897627212699,
        "actions_after_sensor_95_mean": 10.75925925925926,
        "gap_actions_after_sensor_95_mean": 7.092592592592593,
        "zero_new_cell_actions_after_sensor_95_mean": 7.12962962962963,
    }
    for key, value in closure_expected.items():
        _close(closure[key], value)

    summary = _rows(release / "summary.csv")
    assert len(summary) == 270
    assert sum(int(row["runs"]) for row in summary) == 810

    process_root = release / "artifacts/fov360_r8/dense_obstacles"
    frame_total = 0
    for method in ("ans", "frontier", "nbv", "rrt", "sstg"):
        payload = json.loads((process_root / method / "run.json").read_text(encoding="utf-8"))
        steps = payload["steps"]
        pngs = sorted((process_root / method / "steps").glob("step_*.png"))
        assert len(pngs) == len(steps)
        frame_total += len(steps)
    assert frame_total == 123
    matched_process_panels = 5 * 6
    lifecycle_panels = 6
    assert (paper / "figures/algorithm_process_dense_comparison.pdf").is_file()
    assert (paper / "figures/sstg_candidate_lifecycle_dense.pdf").is_file()
    sstg_candidates = _rows(process_root / "sstg" / "candidates.csv")
    lifecycle_states = {row["status"] for row in sstg_candidates}
    assert {
        "active", "selected", "pruned_evaluation_budget",
        "pruned_executed", "pruned_gain",
    } <= lifecycle_states

    audited_fragments = (
        r"$98.38\%$ map coverage but only $33.02\%$ post-hoc 2~m coverage",
        r"$99.99/96.14\%$ sensor/topological coverage with 100\% success",
        r"$3.41$--$29.56$ percentage points",
        r"$5\times6\times9\times3=810$ runs",
        r"$48.31\%$ topologically",
        r"another $47.83$ points through 10.76 actions",
        "6,270 decision images, 270 GIFs, and 270 MP4s",
        "all 123 decision frames",
    )
    for fragment in audited_fragments:
        _contains(manuscript, fragment)

    bib_text = (paper / "reference.bib").read_text(encoding="utf-8")
    bib_keys = set(re.findall(r"@\w+\{([^,]+),", bib_text))
    cited: set[str] = set()
    for group in re.findall(r"\\cite\{([^}]+)\}", manuscript):
        cited.update(part.strip() for part in group.split(","))
    assert cited == bib_keys, (sorted(cited - bib_keys), sorted(bib_keys - cited))
    assert len(bib_keys) == 29

    trace = paper / "FIGURE_TABLE_TRACE.yaml"
    assert trace.is_file() and trace.stat().st_size > 0

    return {
        "passed": True,
        "run_records": audit_report["records"],
        "matrix_cells": len(summary),
        "process_frames": frame_total,
        "matched_process_panels": matched_process_panels,
        "sstg_lifecycle_panels": lifecycle_panels,
        "references": len(bib_keys),
        "manuscript_sha256_prefix": _sha256(root_tex)[:12],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--paper", type=Path, default=Path(__file__).resolve().parents[2] / "SSTGExplorerPaper")
    parser.add_argument("--release", type=Path, default=DEFAULT_RELEASE)
    args = parser.parse_args()
    result = audit(args.repo.resolve(), args.paper.resolve(), args.release)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
