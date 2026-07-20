#!/usr/bin/env python3
"""Assemble an audited paper matrix from frozen baselines and a selected SSTG run.

The script is intentionally provenance-preserving: it does not recompute or
rename numerical observations.  It reuses the four unchanged baseline
components from one audited paper matrix and replaces only the SSTG component
with a separately audited, configuration-matched selection run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

from run_unknown_benchmark import (
    MAIN_ALGORITHMS,
    ROOT,
    audit_output,
    html_report,
    jsonable,
    known_redundancy_supplement,
    summarize,
    three_protocol_comparison,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline_matrix", type=Path)
    parser.add_argument("selected_sstg", type=Path)
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "outputs" / "joint_benchmark_selected",
    )
    parser.add_argument(
        "--known-results", type=Path,
        default=ROOT / "outputs" / "benchmark_runs" / "latest" / "results.json",
    )
    parser.add_argument(
        "--sensor-results", type=Path,
        default=ROOT / "outputs" / "unknown_benchmark_runs" / "latest" / "results.json",
    )
    parser.add_argument(
        "--selection-report", type=Path, action="append", default=[],
        help="Variant-selection Markdown to embed; may be repeated.",
    )
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def record_key(record):
    return (
        record["sensor_key"], record["environment"],
        record["algorithm_key"], int(record["run_id"]),
    )


def validate_pairing(base, selected):
    base_manifest = base["manifest"]
    selected_manifest = selected["manifest"]
    required = (
        "coverage_objective", "environments", "sensors", "runs", "seed",
        "max_decisions",
    )
    mismatches = [
        key for key in required
        if base_manifest.get(key) != selected_manifest.get(key)
    ]
    base_common = base_manifest["common_explorer_config"]
    selected_common = selected_manifest["common_explorer_config"]
    for key in base_common:
        if key == "spacing_weight":
            continue
        if base_common.get(key) != selected_common.get(key):
            mismatches.append(f"common_explorer_config.{key}")
    if mismatches:
        raise ValueError(f"Component configuration mismatch: {mismatches}")

    selected_records = selected["results"]
    if not selected_records or {
        row["algorithm_key"] for row in selected_records
    } != {"sstg"}:
        raise ValueError("Selected component must contain only algorithm_key=sstg")
    expected = {
        (row["sensor_key"], row["environment"], int(row["run_id"]))
        for row in base["results"] if row["algorithm_key"] == "sstg"
    }
    actual = {
        (row["sensor_key"], row["environment"], int(row["run_id"]))
        for row in selected_records
    }
    if expected != actual:
        raise ValueError("Selected SSTG component does not match the 162 paired cells")


def link_tree(source: Path, target: Path):
    shutil.copytree(source, target, copy_function=os.link)


def main():
    args = parse_args()
    baseline_dir = args.baseline_matrix.resolve()
    selected_dir = args.selected_sstg.resolve()
    baseline_path = baseline_dir / "results.json"
    selected_path = selected_dir / "results.json"
    base = json.loads(baseline_path.read_text())
    selected = json.loads(selected_path.read_text())
    validate_pairing(base, selected)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = args.output.resolve()
    output = output_root / stamp
    output.mkdir(parents=True)
    link_tree(baseline_dir / "artifacts", output / "artifacts")
    for sensor in base["manifest"]["sensors"]:
        for environment in base["manifest"]["environments"]:
            target = output / "artifacts" / sensor / environment / "sstg"
            shutil.rmtree(target)
            link_tree(
                selected_dir / "artifacts" / sensor / environment / "sstg",
                target,
            )

    records = [
        row for row in base["results"] if row["algorithm_key"] != "sstg"
    ] + selected["results"]
    sensor_order = {key: index for index, key in enumerate(base["manifest"]["sensors"])}
    environment_order = {
        key: index for index, key in enumerate(base["manifest"]["environments"])
    }
    algorithm_order = {key: index for index, key in enumerate(MAIN_ALGORITHMS)}
    records.sort(key=lambda row: (
        sensor_order[row["sensor_key"]],
        environment_order[row["environment"]],
        algorithm_order[row["algorithm_key"]],
        int(row["run_id"]),
    ))

    selection_sources = []
    selection_sections = [
        "# Final SSTG-Explorer variant selection\n",
        "These are developmental paired comparisons used to freeze the final "
        "configuration; they are not independent confirmatory evidence.\n",
    ]
    for index, source in enumerate(args.selection_report, start=1):
        source = source.resolve()
        selection_sources.append({
            "path": str(source), "sha256": sha256(source),
        })
        selection_sections.extend([
            f"\n## Comparison {index}: {source.parent.name}\n",
            source.read_text(encoding="utf-8"),
        ])
    if args.selection_report:
        (output / "VARIANT_SELECTION.md").write_text(
            "\n".join(selection_sections), encoding="utf-8"
        )

    manifest = dict(selected["manifest"])
    manifest.update({
        "created": datetime.now().isoformat(),
        "command": sys.argv,
        "profile": "paper",
        "algorithms": MAIN_ALGORITHMS,
        "assembly": {
            "mode": "paired_component_reuse",
            "rationale": (
                "Four unchanged baseline components are reused from the audited "
                "paper matrix; the final SSTG component is replaced after a "
                "paired 54-cluster utility-selection experiment."
            ),
            "baseline_algorithms": ["frontier", "nbv", "rrt", "ans"],
            "selected_algorithm": "sstg",
            "baseline_records": 648,
            "selected_records": 162,
            "baseline_directory": str(baseline_dir),
            "selected_directory": str(selected_dir),
            "baseline_results_sha256": sha256(baseline_path),
            "selected_results_sha256": sha256(selected_path),
            "baseline_source_tree_sha256": base["manifest"].get("source_tree_sha256"),
            "selected_source_tree_sha256": selected["manifest"].get("source_tree_sha256"),
            "selection_reports": selection_sources,
        },
    })
    (output / "manifest.json").write_text(
        json.dumps(jsonable(manifest), indent=2), encoding="utf-8"
    )
    (output / "results.json").write_text(
        json.dumps({"manifest": manifest, "results": records}, indent=2),
        encoding="utf-8",
    )
    (output / "run.log").write_text(
        "ASSEMBLED AUDITED COMPONENTS\n"
        f"baseline={baseline_dir}\nselected_sstg={selected_dir}\n"
        "The assembly operation performed no numerical recomputation.\n",
        encoding="utf-8",
    )

    summarize(records, output)
    known_redundancy_supplement(args.known_results, output)
    three_protocol_comparison(
        records, args.known_results, args.sensor_results, output,
        manifest["common_explorer_config"]["topological_radius_m"],
    )
    html_report(records, output)
    audit = audit_output(
        records, output,
        SimpleNamespace(no_frames=False, media_runs="representative"),
    )
    latest = output_root / "latest"
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(output.name)
    print(json.dumps({"output": str(output), "audit": audit}, indent=2))


if __name__ == "__main__":
    main()
