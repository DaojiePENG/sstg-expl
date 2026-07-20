#!/usr/bin/env python3
"""Paired cluster comparison between two joint benchmark result files."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.stats import wilcoxon

from analyze_joint_benchmark import METRICS, rank_biserial


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path, help="results.json containing the reference algorithm")
    parser.add_argument("variant", type=Path, help="results.json containing one variant")
    parser.add_argument("--reference-prefix", default="SSTG-Explorer")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bootstrap", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--correction-factor", type=int, default=1,
        help=(
            "Conservative Bonferroni factor for a development family of "
            "full-matrix variants (two in the final selection study)."
        ),
    )
    return parser.parse_args()


def load(path):
    return json.loads(path.read_text())["results"]


def main():
    args = parse_args()
    reference_all = load(args.reference)
    reference = [
        row for row in reference_all
        if row["algorithm"].startswith(args.reference_prefix)
    ]
    variant = load(args.variant)
    if not reference or not variant:
        raise ValueError("reference or variant records are empty")
    clusters = sorted({
        (row["sensor_key"], row["environment"]) for row in reference
    })
    rng = np.random.default_rng(args.seed)
    samples = rng.integers(0, len(clusters), size=(args.bootstrap, len(clusters)))
    rows = []
    for metric, spec in METRICS.items():
        differences = []
        for cluster in clusters:
            ref = [
                spec["value"](row) for row in reference
                if (row["sensor_key"], row["environment"]) == cluster
            ]
            alt = [
                spec["value"](row) for row in variant
                if (row["sensor_key"], row["environment"]) == cluster
            ]
            differences.append(float(np.mean(ref) - np.mean(alt)))
        differences = np.asarray(differences)
        distribution = differences[samples].mean(axis=1)
        raw_p = float(
            wilcoxon(differences).pvalue
            if np.any(np.abs(differences) > 1e-12) else 1.0
        )
        rows.append({
            "metric": metric,
            "unit": spec["unit"],
            "better": spec["better"],
            "delta_reference_minus_variant": float(np.mean(differences)),
            "ci95_low": float(np.percentile(distribution, 2.5)),
            "ci95_high": float(np.percentile(distribution, 97.5)),
            "wilcoxon_p": raw_p,
            "multiplicity_p": min(raw_p * args.correction_factor, 1.0),
            "rank_biserial": rank_biserial(differences),
            "clusters": len(clusters),
        })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with args.output.with_suffix(".md").open("w") as stream:
        stream.write("# Joint variant selection\n\n")
        stream.write("Differences are reference minus variant over paired sensor–environment clusters. ")
        stream.write(
            f"The adjusted column uses a conservative Bonferroni factor of "
            f"{args.correction_factor} for the full-matrix development family.\n\n"
        )
        stream.write("| Metric | Delta | 95% CI | Wilcoxon p | Adjusted p | Rank-biserial |\n")
        stream.write("|---|---:|---:|---:|---:|---:|\n")
        for row in rows:
            stream.write(
                f"| {row['metric']} | {row['delta_reference_minus_variant']:.4f} {row['unit']} | "
                f"[{row['ci95_low']:.4f}, {row['ci95_high']:.4f}] | "
                f"{row['wilcoxon_p']:.6g} | {row['multiplicity_p']:.6g} | "
                f"{row['rank_biserial']:.3f} |\n"
            )
    print(args.output.with_suffix(".md"))


if __name__ == "__main__":
    main()
