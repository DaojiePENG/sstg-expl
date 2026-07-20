#!/usr/bin/env python3
"""Resume summary/HTML/audit after simulations completed but postprocessing failed."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from run_unknown_benchmark import (
    ROOT,
    audit_output,
    html_report,
    known_redundancy_supplement,
    summarize,
    three_protocol_comparison,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir", type=Path)
    parser.add_argument(
        "--known-results", type=Path,
        default=ROOT / "outputs" / "benchmark_runs" / "latest" / "results.json",
    )
    parser.add_argument(
        "--sensor-results", type=Path,
        default=ROOT / "outputs" / "unknown_benchmark_runs" / "latest" / "results.json",
    )
    parser.add_argument("--set-latest", action="store_true")
    return parser.parse_args()


def main():
    cli = parse_args()
    output = cli.results_dir.resolve()
    payload = json.loads((output / "results.json").read_text())
    manifest = payload["manifest"]
    records = payload["results"]
    command = manifest.get("command", [])
    audit_args = SimpleNamespace(
        no_frames="--no-frames" in command,
        media_runs=(
            command[command.index("--media-runs") + 1]
            if "--media-runs" in command else "representative"
        ),
    )
    summarize(records, output)
    known_redundancy_supplement(cli.known_results, output)
    if manifest.get("coverage_objective") == "joint" and manifest.get("profile") != "ablation":
        three_protocol_comparison(
            records, cli.known_results, cli.sensor_results, output,
            manifest["common_explorer_config"]["topological_radius_m"],
        )
    html_report(records, output)
    report = audit_output(records, output, audit_args)
    if cli.set_latest:
        latest = output.parent / "latest"
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(output.name)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
