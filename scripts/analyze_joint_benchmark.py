#!/usr/bin/env python3
"""Extended cluster statistics and post-sensor gap analysis for joint runs."""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.stats import wilcoxon


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = ROOT / "outputs" / "joint_benchmark_runs" / "latest"
TARGET_PREFIX = "SSTG-Explorer"
HARD_SCENES = ("multiple_rooms", "dense_obstacles", "warehouse")


METRICS = {
    "topological_coverage_pp": {
        "value": lambda row: 100.0 * row["topological_coverage_ratio"],
        "unit": "pp", "better": "higher",
    },
    "sensor_coverage_pp": {
        "value": lambda row: 100.0 * row["sensor_coverage_ratio"],
        "unit": "pp", "better": "higher",
    },
    "distance_m": {
        "value": lambda row: row["total_distance"],
        "unit": "m", "better": "lower",
    },
    "topological_nodes": {
        "value": lambda row: row["num_nodes"],
        "unit": "nodes", "better": "lower",
    },
    "oriented_actions": {
        "value": lambda row: row["num_oriented_views"],
        "unit": "actions", "better": "lower",
    },
    "clearance_m": {
        "value": lambda row: row["additional_metrics"]["avg_obstacle_distance"],
        "unit": "m", "better": "higher",
    },
    "redundant_nodes_pp": {
        "value": lambda row: 100.0 * row["additional_metrics"]["redundant_viewpoint_fraction"],
        "unit": "pp", "better": "lower",
    },
    "success_pp": {
        "value": lambda row: 100.0 * float(row["success"]),
        "unit": "pp", "better": "higher",
    },
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir", nargs="?", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--bootstrap", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def holm_adjust(rows, p_key="wilcoxon_p"):
    order = sorted(range(len(rows)), key=lambda index: rows[index][p_key])
    running = 0.0
    for rank, index in enumerate(order):
        adjusted = min(1.0, (len(rows) - rank) * rows[index][p_key])
        running = max(running, adjusted)
        rows[index]["holm_p"] = running


def rank_biserial(differences):
    nonzero = differences[np.abs(differences) > 1e-12]
    if not len(nonzero):
        return 0.0
    return float((np.sum(nonzero > 0) - np.sum(nonzero < 0)) / len(nonzero))


def cluster_statistics(records, bootstrap, seed):
    algorithms = sorted({row["algorithm"] for row in records})
    target = next(name for name in algorithms if name.startswith(TARGET_PREFIX))
    clusters = sorted({(row["sensor_key"], row["environment"]) for row in records})
    grouped = {}
    for algorithm in algorithms:
        for cluster in clusters:
            subset = [
                row for row in records
                if row["algorithm"] == algorithm and
                (row["sensor_key"], row["environment"]) == cluster
            ]
            grouped[(algorithm, cluster)] = {
                metric: float(np.mean([spec["value"](row) for row in subset]))
                for metric, spec in METRICS.items()
            }

    rng = np.random.default_rng(seed)
    samples = rng.integers(0, len(clusters), size=(bootstrap, len(clusters)))
    rows = []
    for metric, spec in METRICS.items():
        family = []
        for baseline in algorithms:
            if baseline == target:
                continue
            differences = np.asarray([
                grouped[(target, cluster)][metric] - grouped[(baseline, cluster)][metric]
                for cluster in clusters
            ], dtype=float)
            distribution = differences[samples].mean(axis=1)
            p_value = float(
                wilcoxon(differences).pvalue if np.any(np.abs(differences) > 1e-12)
                else 1.0
            )
            sensor_directions = []
            for sensor in sorted({cluster[0] for cluster in clusters}):
                indices = [i for i, cluster in enumerate(clusters) if cluster[0] == sensor]
                sensor_directions.append(float(np.mean(differences[indices])))
            environment_directions = []
            for environment in sorted({cluster[1] for cluster in clusters}):
                indices = [i for i, cluster in enumerate(clusters) if cluster[1] == environment]
                environment_directions.append(float(np.mean(differences[indices])))
            row = {
                "metric": metric,
                "unit": spec["unit"],
                "better": spec["better"],
                "baseline": baseline,
                "clusters": len(clusters),
                "delta_sstg_minus_baseline": float(np.mean(differences)),
                "ci95_low": float(np.percentile(distribution, 2.5)),
                "ci95_high": float(np.percentile(distribution, 97.5)),
                "wilcoxon_p": p_value,
                "rank_biserial": rank_biserial(differences),
                "sensor_groups_positive": int(np.sum(np.asarray(sensor_directions) > 0)),
                "sensor_groups_negative": int(np.sum(np.asarray(sensor_directions) < 0)),
                "environment_groups_positive": int(np.sum(np.asarray(environment_directions) > 0)),
                "environment_groups_negative": int(np.sum(np.asarray(environment_directions) < 0)),
            }
            family.append(row)
        holm_adjust(family)
        rows.extend(family)
    return rows, len(clusters)


def artifact_path(output, row):
    base = output / "artifacts" / row["sensor_key"] / row["environment"] / row["algorithm_key"]
    if int(row["run_id"]) == 0:
        return base / "run.json"
    return base / "runs" / f"run_{int(row['run_id']):03d}" / "run.json"


def post_sensor_analysis(records, output):
    per_run = []
    for row in records:
        payload = json.loads(artifact_path(output, row).read_text())
        steps = payload["steps"]
        first_sensor_index = next(
            (index for index, step in enumerate(steps)
             if step.get("sensor_coverage_after", 0.0) >= 0.95),
            len(steps) - 1,
        )
        first_step = steps[first_sensor_index]
        later = steps[first_sensor_index + 1:]
        accepted = [step for step in later if step.get("event") == "viewpoint_accepted"]
        per_run.append({
            "sensor_key": row["sensor_key"],
            "environment": row["environment"],
            "algorithm_key": row["algorithm_key"],
            "algorithm": row["algorithm"],
            "run_id": row["run_id"],
            "topology_when_sensor_reached_95": first_step.get("topological_coverage_after", 0.0),
            "terminal_topological_coverage": row["topological_coverage_ratio"],
            "topological_gain_after_sensor_95": (
                row["topological_coverage_ratio"] -
                first_step.get("topological_coverage_after", 0.0)
            ),
            "actions_after_sensor_95": len(accepted),
            "gap_actions_after_sensor_95": sum(
                (step.get("selected_frontier") or {}).get("kind") == "coverage_gap"
                for step in accepted
            ),
            "zero_new_cell_actions_after_sensor_95": sum(
                step.get("new_observed_count", 0) == 0 for step in accepted
            ),
            "distance_after_sensor_95_m": float(sum(
                step.get("translation_m", 0.0) for step in accepted
            )),
        })
    return per_run


def aggregate_rows(records, keys):
    groups = {}
    for row in records:
        key = tuple(row[name] for name in keys)
        groups.setdefault(key, []).append(row)
    output = []
    for key, subset in sorted(groups.items()):
        item = dict(zip(keys, key))
        numeric = [
            field for field, value in subset[0].items()
            if isinstance(value, (int, float, bool)) and field not in keys
        ]
        for field in numeric:
            item[f"{field}_mean"] = float(np.mean([row[field] for row in subset]))
        item["runs"] = len(subset)
        output.append(item)
    return output


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    output = args.results_dir.resolve()
    payload = json.loads((output / "results.json").read_text())
    records = payload["results"]
    statistics, cluster_count = cluster_statistics(
        records, args.bootstrap, args.seed
    )
    write_csv(output / "pairwise_all_metrics.csv", statistics)

    post_sensor = post_sensor_analysis(records, output)
    write_csv(output / "post_sensor_gap_closure.csv", post_sensor)
    post_sensor_aggregate = aggregate_rows(post_sensor, ["algorithm"])
    write_csv(output / "post_sensor_gap_closure_aggregate.csv", post_sensor_aggregate)

    hard = []
    for algorithm in sorted({row["algorithm"] for row in records}):
        for environment in HARD_SCENES:
            subset = [
                row for row in records
                if row["algorithm"] == algorithm and row["environment"] == environment
            ]
            hard.append({
                "algorithm": algorithm,
                "environment": environment,
                "runs": len(subset),
                "sensor_coverage_mean": float(np.mean([row["sensor_coverage_ratio"] for row in subset])),
                "topological_coverage_mean": float(np.mean([row["topological_coverage_ratio"] for row in subset])),
                "distance_mean_m": float(np.mean([row["total_distance"] for row in subset])),
                "nodes_mean": float(np.mean([row["num_nodes"] for row in subset])),
                "actions_mean": float(np.mean([row["num_oriented_views"] for row in subset])),
                "clearance_mean_m": float(np.mean([row["additional_metrics"]["avg_obstacle_distance"] for row in subset])),
                "redundant_nodes_mean": float(np.mean([row["additional_metrics"]["redundant_viewpoint_fraction"] for row in subset])),
                "success_rate": float(np.mean([row["success"] for row in subset])),
            })
    write_csv(output / "hard_scene_analysis.csv", hard)

    termination = Counter(
        (row["algorithm"], row["additional_metrics"].get("termination_reason", "unknown"))
        for row in records
    )
    termination_rows = [
        {"algorithm": key[0], "termination_reason": key[1], "runs": value}
        for key, value in sorted(termination.items())
    ]
    write_csv(output / "termination_reasons.csv", termination_rows)

    with (output / "statistical_analysis.md").open("w") as stream:
        stream.write("# Joint benchmark extended statistical analysis\n\n")
        stream.write(
            f"All differences are SSTG-Explorer minus baseline, based on "
            f"{cluster_count} "
            f"sensor–environment clusters, {args.bootstrap:,} cluster bootstrap "
            "resamples, paired Wilcoxon tests, and Holm correction separately "
            "within each metric family.\n\n"
        )
        stream.write("| Metric | Baseline | Delta | 95% CI | Holm p | Rank-biserial |\n")
        stream.write("|---|---|---:|---:|---:|---:|\n")
        for row in statistics:
            stream.write(
                f"| {row['metric']} | {row['baseline']} | "
                f"{row['delta_sstg_minus_baseline']:.4f} {row['unit']} | "
                f"[{row['ci95_low']:.4f}, {row['ci95_high']:.4f}] | "
                f"{row['holm_p']:.6g} | {row['rank_biserial']:.3f} |\n"
            )
        stream.write("\n## Actions after sensor coverage first reached 95%\n\n")
        stream.write("| Method | Topology at sensor target | Later topology gain | Later actions | Gap actions | Zero-new-cell actions | Later distance |\n")
        stream.write("|---|---:|---:|---:|---:|---:|---:|\n")
        for row in post_sensor_aggregate:
            stream.write(
                f"| {row['algorithm']} | "
                f"{row['topology_when_sensor_reached_95_mean']:.2%} | "
                f"{row['topological_gain_after_sensor_95_mean']:.2%} | "
                f"{row['actions_after_sensor_95_mean']:.2f} | "
                f"{row['gap_actions_after_sensor_95_mean']:.2f} | "
                f"{row['zero_new_cell_actions_after_sensor_95_mean']:.2f} | "
                f"{row['distance_after_sensor_95_m_mean']:.2f} m |\n"
            )
    summary = {
        "records": len(records),
        "clusters": cluster_count,
        "bootstrap_resamples": args.bootstrap,
        "statistics": statistics,
        "post_sensor_gap_closure": post_sensor_aggregate,
        "termination_reasons": termination_rows,
    }
    (output / "statistical_analysis.json").write_text(json.dumps(summary, indent=2))
    print(output / "statistical_analysis.md")


if __name__ == "__main__":
    main()
