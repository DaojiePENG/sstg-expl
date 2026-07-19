#!/usr/bin/env python3
"""Unknown-static-grid benchmark with occlusion-aware configurable sensing."""
from __future__ import annotations

import argparse
import contextlib
import csv
import json
import os
import random
import re
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLBACKEND", "Agg")

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.stats import wilcoxon

try:
    from scripts.run_benchmark import (
        ENVIRONMENTS, checkpoint_info, dependency_versions, git_info,
        jsonable, make_env, source_fingerprint,
    )
except ImportError:  # Direct ``python scripts/run_unknown_benchmark.py``.
    from run_benchmark import (
        ENVIRONMENTS, checkpoint_info, dependency_versions, git_info,
        jsonable, make_env, source_fingerprint,
    )

from sstg_explorer.benchmark import BenchmarkRunner
from sstg_explorer.sensing import SensorConfig
from sstg_explorer.unknown import UnknownExplorerConfig, UnknownMapExplorer
from sstg_explorer.visualization import reconstruct_beliefs, visualize_unknown_step


MAIN_ALGORITHMS = ["frontier", "nbv", "rrt", "ans", "sstg"]
ABLATION_ALGORITHMS = [
    "sstg", "sstg_single_centroid", "sstg_known_obstacle_only",
    "sstg_no_vantage", "sstg_with_spacing",
]
ALGORITHMS = MAIN_ALGORITHMS + [
    key for key in ABLATION_ALGORITHMS if key not in MAIN_ALGORITHMS
]
ABLATION_NAMES = {
    "sstg_single_centroid": "SSTG Unknown: single frontier centroid",
    "sstg_known_obstacle_only": "SSTG Unknown: known-obstacle-only safety",
    "sstg_no_vantage": "SSTG Unknown: no topological vantages",
    "sstg_with_spacing": "SSTG Unknown: with spacing utility",
}
SENSORS = {
    "fov360_r8": SensorConfig(360.0, 8.0, 0.25),
    "fov360_r12": SensorConfig(360.0, 12.0, 0.25),
    "fov360_r16": SensorConfig(360.0, 16.0, 0.25),
    "fov240_r12": SensorConfig(240.0, 12.0, 0.25),
    "fov120_r12": SensorConfig(120.0, 12.0, 0.25),
    "fov90_r12": SensorConfig(90.0, 12.0, 0.25),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile", choices=["smoke", "paper", "fov", "range", "ablation"],
        default="paper",
    )
    parser.add_argument("--runs", type=int)
    parser.add_argument("--algorithms", nargs="+", choices=ALGORITHMS)
    parser.add_argument("--environments", nargs="+", choices=list(ENVIRONMENTS))
    parser.add_argument("--sensors", nargs="+", choices=list(SENSORS))
    parser.add_argument(
        "--output", type=Path,
        default=None,
        help=(
            "Output root; defaults to outputs/unknown_ablation_runs for the "
            "ablation profile and outputs/unknown_benchmark_runs otherwise."
        ),
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-decisions", type=int, default=80)
    parser.add_argument("--no-frames", action="store_true")
    parser.add_argument(
        "--media-runs", choices=["all", "representative"],
        default="representative",
        help="Unknown paper matrix defaults to media for run 0; all runs retain traces.",
    )
    parser.add_argument(
        "--known-results", type=Path,
        default=ROOT / "outputs" / "benchmark_runs" / "latest" / "results.json",
        help="Existing known-map results used only for a redundancy supplement.",
    )
    return parser.parse_args()


def profile_scope(args):
    algorithms = args.algorithms or (
        ABLATION_ALGORITHMS if args.profile == "ablation" else MAIN_ALGORITHMS
    )
    if args.environments:
        environments = args.environments
    elif args.profile == "smoke":
        environments = ["multiple_rooms", "dense_obstacles"]
    elif args.profile == "range":
        environments = [
            "multiple_rooms", "maze", "dense_obstacles",
            "narrow_passages", "warehouse",
        ]
    elif args.profile == "ablation":
        environments = [
            "multiple_rooms", "dense_obstacles",
            "narrow_passages", "warehouse",
        ]
    else:
        environments = list(ENVIRONMENTS)
    if args.sensors:
        sensors = args.sensors
    elif args.profile == "smoke":
        sensors = ["fov360_r8", "fov90_r12"]
    elif args.profile == "fov":
        sensors = ["fov360_r12", "fov240_r12", "fov120_r12", "fov90_r12"]
    elif args.profile == "range":
        sensors = ["fov360_r8", "fov360_r12", "fov360_r16"]
    elif args.profile == "ablation":
        sensors = ["fov360_r12", "fov120_r12", "fov90_r12"]
    else:
        sensors = list(SENSORS)
    runs = args.runs or (1 if args.profile == "smoke" else 3)
    return algorithms, environments, sensors, runs


def create_algorithm(key, sensor, seed, max_decisions):
    checkpoint = str(ROOT / "models" / "checkpoints" / "ans_global_policy.pt")
    strategy = "sstg" if key in ABLATION_ALGORITHMS else key
    config = UnknownExplorerConfig(
        strategy=strategy,
        sensor=sensor,
        target_coverage=0.95,
        max_decisions=max_decisions,
        seed=seed,
        checkpoint=checkpoint if key == "ans" else None,
        multi_frontier=key != "sstg_single_centroid",
        require_known_footprint=key != "sstg_known_obstacle_only",
        use_topological_vantages=key != "sstg_no_vantage",
        spacing_weight=0.30 if key == "sstg_with_spacing" else 0.0,
    )
    algorithm = UnknownMapExplorer(config)
    if key in ABLATION_NAMES:
        algorithm.name = ABLATION_NAMES[key]
    return algorithm


def run_experiment(runner, algorithm_key, sensor_key, environment, run_id, seed, max_decisions):
    random.seed(seed)
    np.random.seed(seed)
    sensor = SENSORS[sensor_key]
    algorithm = create_algorithm(algorithm_key, sensor, seed, max_decisions)
    truth = environment.get_occupancy_map()
    started = time.perf_counter()
    result = algorithm.explore(truth, environment.get_start_pose())
    elapsed = time.perf_counter() - started
    nodes = result["nodes"]
    positions = [node["position"] for node in nodes]
    paths = result.get("paths", result["metadata"].get("paths", []))
    path_positions = runner.sample_execution_paths(paths, truth.resolution)
    spatial = runner.compute_spatial_metrics(
        positions, truth, r_view=2.0, path_positions=path_positions,
        required_clearance=0.5, redundancy_distance=1.0,
    )
    metadata = result["metadata"]
    trace_candidates = [
        candidate for step in result["steps"]
        for candidate in step.get("generated_candidates", [])
    ]
    record = {
        "protocol": "unknown_static_grid_occlusion_aware",
        "algorithm_key": algorithm_key,
        "algorithm": result["algorithm"],
        "sensor_key": sensor_key,
        "sensor_fov_deg": sensor.field_of_view_deg,
        "sensor_range_m": sensor.max_range,
        "environment": environment.name,
        "run_id": run_id,
        "seed": seed,
        "success": bool(result["success"]),
        "coverage_ratio": float(metadata["coverage_ratio"]),
        "known_ratio": float(metadata["known_ratio"]),
        "occupied_recall": float(metadata["occupied_recall"]),
        "total_distance": float(metadata["total_distance"]),
        "total_rotation_deg": float(metadata["total_rotation_deg"]),
        "num_nodes": len(nodes),
        "scan_count": int(metadata["scan_count"]),
        "in_place_rotations": int(metadata["in_place_rotations"]),
        "computation_time": elapsed,
        "coverage_efficiency": float(metadata["coverage_ratio"]) /
            max(float(metadata["total_distance"]), 0.01),
        "coverage_per_viewpoint": float(metadata["coverage_ratio"]) / max(len(nodes), 1),
        "viewpoints_per_95_coverage": len(nodes) * 0.95 /
            max(float(metadata["coverage_ratio"]), 1e-9),
        "additional_metrics": {
            **{key: value for key, value in metadata.items() if key != "paths"},
            **spatial,
            "num_decision_steps": len(result["steps"]),
            "num_generated_candidates": len(trace_candidates),
            "num_pruned_candidates": sum(
                candidate.get("status", "").startswith("pruned")
                for candidate in trace_candidates
            ),
        },
        "trajectory": nodes,
        "execution_paths": paths,
        "steps": result["steps"],
        "belief_final": result["belief_final"],
    }
    return record, truth


def create_media(record, truth, output):
    frames = output / "steps"
    frames.mkdir(parents=True, exist_ok=True)
    snapshots = reconstruct_beliefs(record["steps"], truth.shape)
    cumulative_paths = []
    for index, (step, belief) in enumerate(zip(record["steps"], snapshots)):
        if step.get("event") == "viewpoint_accepted":
            cumulative_paths.append(step.get("path", []))
        visualize_unknown_step(
            truth, belief, step, list(cumulative_paths),
            save_path=str(frames / f"step_{index:04d}.png"),
            title=(
                f"{record['algorithm']} · {record['environment']} · "
                f"{record['sensor_key']} · decision {index}/{len(record['steps']) - 1}"
            ),
        )
    paths = sorted(frames.glob("step_*.png"))
    opened = [Image.open(path).convert("RGB") for path in paths]
    width = max((image.width for image in opened), default=0)
    height = max((image.height for image in opened), default=0)
    width += width % 2
    height += height % 2
    images = []
    for item in opened:
        canvas = Image.new("RGB", (width, height), "white")
        canvas.paste(item, ((width - item.width) // 2, (height - item.height) // 2))
        images.append(np.asarray(canvas))
    if images:
        imageio.mimsave(output / "animation.gif", images, duration=0.42, loop=0)
        try:
            imageio.mimsave(output / "video.mp4", images, fps=3, macro_block_size=1)
        except Exception as exc:
            (output / "video_error.txt").write_text(str(exc))
        (output / "final.png").write_bytes(paths[-1].read_bytes())


def write_tabular_trace(record, output):
    """Export human-readable tables alongside the lossless JSON trace."""
    decisions, candidates, scans, path_waypoints = [], [], [], []
    for step in record["steps"]:
        selected = step.get("selected_frontier") or {}
        decisions.append({
            "trace_id": step.get("trace_id"),
            "iteration": step.get("iteration"),
            "event": step.get("event"),
            "pose_x": step.get("current_pose", [None, None, None])[0],
            "pose_y": step.get("current_pose", [None, None, None])[1],
            "pose_heading_deg": step.get("current_pose", [None, None, None])[2],
            "coverage_before": step.get("coverage_before"),
            "coverage_after": step.get("coverage_after"),
            "coverage_gain": step.get("coverage_gain"),
            "known_ratio": step.get("known_ratio"),
            "occupied_recall": step.get("occupied_recall"),
            "new_observed_cells": step.get("new_observed_count"),
            "visible_cells": step.get("visible_cell_count"),
            "translation_m": step.get("translation_m", 0.0),
            "rotation_deg": step.get("rotation_deg", 0.0),
            "generated_candidates": len(step.get("generated_candidates", [])),
            "active_candidates": len(step.get("active_frontiers", [])),
            "new_candidates": len(step.get("new_frontiers", [])),
            "selected_id": selected.get("frontier_id"),
            "selected_kind": selected.get("kind"),
            "selected_gain": selected.get("predicted_gain"),
            "selected_priority": selected.get("priority"),
        })
        selected_id = selected.get("frontier_id")
        new_ids = {
            item.get("frontier_id") for item in step.get("new_frontiers", [])
        }
        for candidate in step.get("generated_candidates", []):
            is_selected = candidate.get("frontier_id") == selected_id
            candidates.append({
                "trace_id": step.get("trace_id"),
                "iteration": step.get("iteration"),
                "frontier_id": candidate.get("frontier_id"),
                "status": selected.get("status") if is_selected else candidate.get("status"),
                "is_new": candidate.get("frontier_id") in new_ids,
                "is_selected": is_selected,
                "kind": candidate.get("kind"),
                "x": candidate.get("target", [None, None])[0],
                "y": candidate.get("target", [None, None])[1],
                "heading_deg": candidate.get("heading"),
                "optimistic_gain": candidate.get("optimistic_gain"),
                "predicted_gain": candidate.get("predicted_gain"),
                "geodesic_cost_m": candidate.get("path_cost"),
                "clearance_m": candidate.get("clearance"),
                "nearest_viewpoint_m": candidate.get("nearest_viewpoint_distance"),
                "priority": candidate.get("priority"),
                "cluster_size": candidate.get("cluster_size"),
            })
        for scan_id, pose in enumerate(step.get("scan_poses", [])):
            scans.append({
                "trace_id": step.get("trace_id"), "scan_id": scan_id,
                "x": pose[0], "y": pose[1], "heading_deg": pose[2],
            })
        for waypoint_id, point in enumerate(step.get("path", [])):
            path_waypoints.append({
                "trace_id": step.get("trace_id"),
                "waypoint_id": waypoint_id, "x": point[0], "y": point[1],
            })

    trajectory = [{
        "node_id": node.get("id"),
        "x": node["position"][0], "y": node["position"][1],
        "orientation_deg": node.get("orientation"),
        "decision_timestamp": node.get("timestamp"),
    } for node in record["trajectory"]]
    for filename, rows in (
        ("decisions.csv", decisions),
        ("candidates.csv", candidates),
        ("trajectory.csv", trajectory),
        ("scan_poses.csv", scans),
        ("path_waypoints.csv", path_waypoints),
    ):
        empty_fields = {
            "candidates.csv": [
                "trace_id", "iteration", "frontier_id", "status",
                "is_new", "is_selected", "kind", "x", "y",
                "heading_deg", "optimistic_gain", "predicted_gain",
                "geodesic_cost_m", "clearance_m", "nearest_viewpoint_m",
                "priority", "cluster_size",
            ],
            "path_waypoints.csv": ["trace_id", "waypoint_id", "x", "y"],
            "scan_poses.csv": ["trace_id", "scan_id", "x", "y", "heading_deg"],
        }
        fields = list(rows[0].keys()) if rows else empty_fields[filename]
        with (output / filename).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)


METRICS = [
    "coverage_ratio", "known_ratio", "occupied_recall", "total_distance",
    "num_nodes", "computation_time", "coverage_efficiency",
    "coverage_per_viewpoint", "viewpoints_per_95_coverage",
    "avg_obstacle_distance", "min_obstacle_distance",
    "avg_path_obstacle_distance", "min_path_obstacle_distance",
    "avg_boundary_distance", "node_safe_fraction", "path_safe_fraction",
    "mean_nn_distance", "median_nn_distance", "min_nn_distance",
    "nn_distance_std", "dispersion_uniformity",
    "redundant_viewpoint_fraction", "viewpoint_separation_ratio",
    "total_rotation_deg", "scan_count", "in_place_rotations",
]
NN_METRICS = {
    "mean_nn_distance", "median_nn_distance", "min_nn_distance",
    "nn_distance_std", "dispersion_uniformity", "viewpoint_separation_ratio",
}


def metric(record, name):
    return record.get(name, record.get("additional_metrics", {}).get(name, 0.0))


def metric_mean(records, name):
    values = [
        metric(record, name) for record in records
        if name not in NN_METRICS or
        metric(record, "nn_metric_defined") > 0.5
    ]
    return float(np.mean(values)) if values else 0.0


def summarize(records, output):
    groups = {}
    for record in records:
        key = (record["algorithm"], record["sensor_key"], record["environment"])
        groups.setdefault(key, []).append(record)
    rows = []
    for (algorithm, sensor, environment), subset in groups.items():
        row = {
            "algorithm": algorithm, "sensor": sensor,
            "environment": environment, "runs": len(subset),
        }
        for name in METRICS:
            values = np.asarray([
                metric(item, name) for item in subset
                if name not in NN_METRICS or
                metric(item, "nn_metric_defined") > 0.5
            ], dtype=float)
            if not len(values):
                values = np.asarray([0.0])
            row[name + "_mean"] = float(np.mean(values))
            row[name + "_std"] = float(np.std(values))
            row[name + "_ci95"] = float(1.96 * np.std(values) / np.sqrt(len(values)))
        row["success_rate"] = float(np.mean([item["success"] for item in subset]))
        rows.append(row)
    with (output / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    aggregates = []
    for algorithm in sorted({record["algorithm"] for record in records}):
        for sensor in sorted({record["sensor_key"] for record in records}):
            subset = [
                record for record in records
                if record["algorithm"] == algorithm and record["sensor_key"] == sensor
            ]
            if not subset:
                continue
            aggregates.append({
                "algorithm": algorithm, "sensor": sensor,
                "experiments": len(subset),
                "coverage_mean": float(np.mean([record["coverage_ratio"] for record in subset])),
                "coverage_std": float(np.std([record["coverage_ratio"] for record in subset])),
                "distance_mean": float(np.mean([record["total_distance"] for record in subset])),
                "nodes_mean": float(np.mean([record["num_nodes"] for record in subset])),
                "view_clearance_mean": float(np.mean([metric(record, "avg_obstacle_distance") for record in subset])),
                "mean_nn_distance": metric_mean(subset, "mean_nn_distance"),
                "redundant_viewpoint_fraction": float(np.mean([metric(record, "redundant_viewpoint_fraction") for record in subset])),
                "coverage_per_viewpoint": float(np.mean([record["coverage_per_viewpoint"] for record in subset])),
                "success_rate": float(np.mean([record["success"] for record in subset])),
                "time_mean": float(np.mean([record["computation_time"] for record in subset])),
            })
    with (output / "aggregate_by_sensor.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=aggregates[0].keys())
        writer.writeheader()
        writer.writerows(aggregates)

    macro = []
    for algorithm in sorted({record["algorithm"] for record in records}):
        subset = [record for record in records if record["algorithm"] == algorithm]
        macro.append({
            "algorithm": algorithm, "experiments": len(subset),
            "coverage_mean": float(np.mean([record["coverage_ratio"] for record in subset])),
            "coverage_std": float(np.std([record["coverage_ratio"] for record in subset])),
            "distance_mean": float(np.mean([record["total_distance"] for record in subset])),
            "nodes_mean": float(np.mean([record["num_nodes"] for record in subset])),
            "view_clearance_mean": float(np.mean([metric(record, "avg_obstacle_distance") for record in subset])),
            "mean_nn_distance": metric_mean(subset, "mean_nn_distance"),
            "redundant_viewpoint_fraction": float(np.mean([metric(record, "redundant_viewpoint_fraction") for record in subset])),
            "coverage_per_viewpoint": float(np.mean([record["coverage_per_viewpoint"] for record in subset])),
            "success_rate": float(np.mean([record["success"] for record in subset])),
            "time_mean": float(np.mean([record["computation_time"] for record in subset])),
        })
    with (output / "aggregate.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=macro[0].keys())
        writer.writeheader()
        writer.writerows(macro)

    with (output / "results_table.md").open("w") as stream:
        stream.write("| Algorithm | Coverage | Distance | Nodes | NN distance | Redundant views | Clearance | Success |\n")
        stream.write("|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for row in macro:
            stream.write(
                f"| {row['algorithm']} | {row['coverage_mean']*100:.2f}% "
                f"| {row['distance_mean']:.2f} m | {row['nodes_mean']:.2f} "
                f"| {row['mean_nn_distance']:.2f} m "
                f"| {row['redundant_viewpoint_fraction']:.1%} "
                f"| {row['view_clearance_mean']:.2f} m "
                f"| {row['success_rate']:.1%} |\n"
            )
    latex_escape = lambda value: (
        value.replace("_", "\\_").replace("%", "\\%")
    )
    with (output / "results_table.tex").open("w") as stream:
        stream.write("\\begin{tabular}{lrrrrrr}\\toprule\n")
        stream.write(
            r"Method & Cov. [\%] & Dist. [m] & Views & NN [m] & Red. [\%] & Succ. [\%] \\ \midrule"
            + "\n"
        )
        for row in macro:
            stream.write(
                f"{latex_escape(row['algorithm'])} & "
                f"{row['coverage_mean']*100:.2f} & {row['distance_mean']:.2f} & "
                f"{row['nodes_mean']:.2f} & {row['mean_nn_distance']:.2f} & "
                f"{row['redundant_viewpoint_fraction']*100:.1f} & "
                f"{row['success_rate']*100:.1f} " + r"\\" + "\n"
            )
        stream.write("\\bottomrule\\end{tabular}\n")

    _pairwise(records, output)
    _plots(aggregates, macro, output)
    return rows, aggregates, macro


def _pairwise(records, output):
    clusters = sorted({
        (record["sensor_key"], record["environment"]) for record in records
    })
    algorithms = sorted({record["algorithm"] for record in records})
    target = "SSTG-Explorer Unknown"
    fields = [
        "baseline", "coverage_delta_pp", "coverage_ci95_low", "coverage_ci95_high",
        "distance_delta_m", "distance_ci95_low", "distance_ci95_high",
        "nn_distance_delta_m", "redundancy_delta", "coverage_wilcoxon_p",
        "coverage_holm_p",
    ]
    rows = []
    if target in algorithms:
        means = {}
        for algorithm in algorithms:
            for cluster in clusters:
                subset = [
                    record for record in records
                    if record["algorithm"] == algorithm and
                    (record["sensor_key"], record["environment"]) == cluster
                ]
                means[(algorithm, cluster)] = {
                    "coverage": float(np.mean([record["coverage_ratio"] for record in subset])),
                    "distance": float(np.mean([record["total_distance"] for record in subset])),
                    "nn": metric_mean(subset, "mean_nn_distance"),
                    "nn_defined": any(
                        metric(record, "nn_metric_defined") > 0.5
                        for record in subset
                    ),
                    "redundancy": float(np.mean([metric(record, "redundant_viewpoint_fraction") for record in subset])),
                }
        rng = np.random.default_rng(42)
        for baseline in algorithms:
            if baseline == target:
                continue
            coverage = np.asarray([
                (means[(target, cluster)]["coverage"] - means[(baseline, cluster)]["coverage"]) * 100
                for cluster in clusters
            ])
            distance = np.asarray([
                means[(target, cluster)]["distance"] - means[(baseline, cluster)]["distance"]
                for cluster in clusters
            ])
            nn_delta = np.asarray([
                means[(target, cluster)]["nn"] - means[(baseline, cluster)]["nn"]
                for cluster in clusters
                if means[(target, cluster)]["nn_defined"] and
                means[(baseline, cluster)]["nn_defined"]
            ])
            redundancy = np.asarray([
                means[(target, cluster)]["redundancy"] - means[(baseline, cluster)]["redundancy"]
                for cluster in clusters
            ])
            samples = rng.integers(0, len(clusters), size=(10000, len(clusters)))
            coverage_boot = coverage[samples].mean(axis=1)
            distance_boot = distance[samples].mean(axis=1)
            rows.append({
                "baseline": baseline,
                "coverage_delta_pp": float(np.mean(coverage)),
                "coverage_ci95_low": float(np.percentile(coverage_boot, 2.5)),
                "coverage_ci95_high": float(np.percentile(coverage_boot, 97.5)),
                "distance_delta_m": float(np.mean(distance)),
                "distance_ci95_low": float(np.percentile(distance_boot, 2.5)),
                "distance_ci95_high": float(np.percentile(distance_boot, 97.5)),
                "nn_distance_delta_m": float(np.mean(nn_delta)) if len(nn_delta) else 0.0,
                "redundancy_delta": float(np.mean(redundancy)),
                "coverage_wilcoxon_p": float(
                    wilcoxon(coverage).pvalue if np.any(coverage) else 1.0
                ),
            })
        order = sorted(range(len(rows)), key=lambda index: rows[index]["coverage_wilcoxon_p"])
        running = 0.0
        for rank, index in enumerate(order):
            adjusted = min(1.0, (len(rows) - rank) * rows[index]["coverage_wilcoxon_p"])
            running = max(running, adjusted)
            rows[index]["coverage_holm_p"] = running
    with (output / "pairwise_vs_sstg_unknown.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plots(aggregates, macro, output):
    algorithms = sorted({row["algorithm"] for row in aggregates})
    sensors = list(SENSORS)
    matrix = np.full((len(algorithms), len(sensors)), np.nan)
    for i, algorithm in enumerate(algorithms):
        for j, sensor in enumerate(sensors):
            match = next((row for row in aggregates if row["algorithm"] == algorithm and row["sensor"] == sensor), None)
            if match:
                matrix[i, j] = match["coverage_mean"] * 100
    fig, ax = plt.subplots(figsize=(11, 5.5))
    image = ax.imshow(matrix, vmin=70, vmax=100, cmap="RdYlGn", aspect="auto")
    ax.set_xticks(range(len(sensors)), sensors, rotation=25, ha="right")
    ax.set_yticks(range(len(algorithms)), algorithms)
    for i in range(len(algorithms)):
        for j in range(len(sensors)):
            if np.isfinite(matrix[i, j]):
                ax.text(j, i, f"{matrix[i,j]:.1f}", ha="center", va="center", fontsize=8)
    fig.colorbar(image, ax=ax, label="Observed free-space coverage [%]")
    fig.tight_layout()
    fig.savefig(output / "sensor_coverage_heatmap.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for algorithm in algorithms:
        fov_rows = sorted(
            [row for row in aggregates if row["algorithm"] == algorithm and row["sensor"] in
             ("fov360_r12", "fov240_r12", "fov120_r12", "fov90_r12")],
            key=lambda row: SENSORS[row["sensor"]].field_of_view_deg,
        )
        if fov_rows:
            axes[0].plot(
                [SENSORS[row["sensor"]].field_of_view_deg for row in fov_rows],
                [row["coverage_mean"] * 100 for row in fov_rows], marker="o",
                label=algorithm,
            )
        range_rows = sorted(
            [row for row in aggregates if row["algorithm"] == algorithm and row["sensor"] in
             ("fov360_r8", "fov360_r12", "fov360_r16")],
            key=lambda row: SENSORS[row["sensor"]].max_range,
        )
        if range_rows:
            axes[1].plot(
                [SENSORS[row["sensor"]].max_range for row in range_rows],
                [row["coverage_mean"] * 100 for row in range_rows], marker="o",
                label=algorithm,
            )
    axes[0].set_xlabel("Horizontal FOV [deg]")
    axes[0].set_ylabel("Coverage [%]")
    axes[0].set_title("FOV sensitivity at 12 m")
    axes[1].set_xlabel("Maximum range [m]")
    axes[1].set_ylabel("Coverage [%]")
    axes[1].set_title("Range sensitivity at 360°")
    for ax in axes:
        ax.grid(alpha=.25)
    handles, labels = axes[1].get_legend_handles_labels()
    if handles:
        axes[1].legend(fontsize=7, loc="best")
    fig.tight_layout()
    fig.savefig(output / "fov_range_sensitivity.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for row in macro:
        ax.scatter(
            row["mean_nn_distance"], row["view_clearance_mean"],
            s=50 + 500 * row["coverage_mean"],
            label=row["algorithm"], alpha=.8,
        )
    ax.set_xlabel("Mean nearest-viewpoint distance [m] (higher = less spatial redundancy)")
    ax.set_ylabel("Mean obstacle clearance [m]")
    ax.set_title("Unknown-map safety–redundancy trade-off")
    ax.grid(alpha=.25)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(output / "safety_redundancy_tradeoff.png", dpi=180)
    plt.close(fig)


def known_redundancy_supplement(results_path: Path, output: Path):
    if not results_path.exists():
        return None
    payload = json.loads(results_path.read_text())
    records = payload["results"]
    rows = []
    for record in records:
        positions = [
            node.get("position", node) if isinstance(node, dict) else node
            for node in record["trajectory"]
        ]
        metrics = BenchmarkRunner.compute_spatial_metrics(
            positions,
            make_env(record["environment"]).get_occupancy_map(),
            r_view=2.0, redundancy_distance=1.0,
        )
        rows.append({
            "algorithm": record["algorithm"],
            "environment": record["environment"],
            "run_id": record["run_id"],
            "coverage": record["coverage_ratio"],
            "num_nodes": len(positions),
            "mean_nn_distance": metrics["mean_nn_distance"],
            "median_nn_distance": metrics["median_nn_distance"],
            "min_nn_distance": metrics["min_nn_distance"],
            "redundant_viewpoint_fraction": metrics["redundant_viewpoint_fraction"],
            "coverage_per_viewpoint": record["coverage_ratio"] / max(len(positions), 1),
        })
    with (output / "known_map_redundancy_supplement.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    aggregate = []
    for algorithm in sorted({row["algorithm"] for row in rows}):
        subset = [row for row in rows if row["algorithm"] == algorithm]
        aggregate.append({
            "algorithm": algorithm,
            "mean_nn_distance": float(np.mean([row["mean_nn_distance"] for row in subset])),
            "redundant_viewpoint_fraction": float(np.mean([row["redundant_viewpoint_fraction"] for row in subset])),
            "coverage_per_viewpoint": float(np.mean([row["coverage_per_viewpoint"] for row in subset])),
            "nodes_mean": float(np.mean([row["num_nodes"] for row in subset])),
        })
    with (output / "known_map_redundancy_table.md").open("w") as stream:
        stream.write("| Algorithm | Mean NN distance | Redundant views (<1 m) | Coverage/view | Nodes |\n")
        stream.write("|---|---:|---:|---:|---:|\n")
        for row in aggregate:
            stream.write(
                f"| {row['algorithm']} | {row['mean_nn_distance']:.3f} m "
                f"| {row['redundant_viewpoint_fraction']:.1%} "
                f"| {row['coverage_per_viewpoint']:.4f} | {row['nodes_mean']:.2f} |\n"
            )
    return aggregate


def html_report(records, output):
    cards = []
    for record in records:
        if record["run_id"] != 0:
            continue
        relative = (
            f"artifacts/{record['sensor_key']}/{record['environment']}/"
            f"{record['algorithm_key']}"
        )
        media = (output / relative / "animation.gif").exists()
        if media:
            media_html = (
                f'<a href="{relative}/video.mp4">'
                f'<img src="{relative}/animation.gif"></a>'
            )
        else:
            media_html = "<p>media disabled; numerical trace retained</p>"
        links = [f'<a href="{relative}/run.json">raw trace</a>']
        if (output / relative / "decisions.csv").exists():
            links.append(f'<a href="{relative}/decisions.csv">decisions CSV</a>')
        if (output / relative / "candidates.csv").exists():
            links.append(f'<a href="{relative}/candidates.csv">candidates CSV</a>')
        if (output / relative / "trajectory.csv").exists():
            links.append(f'<a href="{relative}/trajectory.csv">trajectory CSV</a>')
        if (output / relative / "steps").exists():
            links.insert(0, f'<a href="{relative}/steps/">steps</a>')
        if (output / relative / "video.mp4").exists():
            links.append(f'<a href="{relative}/video.mp4">MP4</a>')
        if (output / relative / "runs").exists():
            links.append(f'<a href="{relative}/runs/">other runs</a>')
        cards.append(
            f'<article><h3>{record["algorithm"]}</h3>'
            f'<p>{record["environment"]} · {record["sensor_key"]}</p>'
            f'{media_html}'
            f'<p>coverage {record["coverage_ratio"]:.1%} · '
            f'distance {record["total_distance"]:.1f} m · '
            f'nodes {record["num_nodes"]}</p>'
            f'<p>{" · ".join(links)}</p></article>'
        )
    html = f'''<!doctype html><meta charset="utf-8"><title>SSTG unknown-map benchmark</title>
<style>body{{font:15px system-ui;margin:2rem;background:#eef2f5}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:1rem}}article{{background:white;padding:1rem;border-radius:10px}}img{{width:100%}}.warn{{background:#fff3cd;padding:1rem;border-left:5px solid #f9a825}}</style>
<h1>Unknown Static Grid · Occlusion-Aware Exploration</h1>
<p class="warn">This protocol starts from an all-unknown occupancy belief. Algorithms never receive ground truth; only the simulator/evaluator does. Do not mix these values with the known-map benchmark.</p>
<p><a href="sensor_coverage_heatmap.png">sensor heatmap</a> · <a href="fov_range_sensitivity.png">FOV/range sensitivity</a> · <a href="safety_redundancy_tradeoff.png">safety/redundancy</a> · <a href="summary.csv">summary CSV</a> · <a href="aggregate.csv">macro CSV</a> · <a href="pairwise_vs_sstg_unknown.csv">statistics</a> · <a href="results_table.md">paper table</a> · <a href="results_table.tex">LaTeX</a> · <a href="known_map_redundancy_table.md">known-map redundancy supplement</a> · <a href="manifest.json">manifest</a> · <a href="audit_report.json">audit</a></p>
<main>{''.join(cards)}</main>'''
    (output / "index.html").write_text(html, encoding="utf-8")


def audit_output(records, output, args):
    """Verify numerical traces, media, belief replay and HTML references."""
    missing, replay_mismatches, truth_mismatches, media_errors = [], [], [], []
    file_counts = {
        "run_json": 0, "belief_npy": 0, "decision_csv": 0,
        "candidate_csv": 0, "trajectory_csv": 0,
        "step_png": 0, "gif": 0, "mp4": 0,
    }
    truth_cache = {}
    for record in records:
        base = (
            output / "artifacts" / record["sensor_key"] /
            record["environment"] / record["algorithm_key"]
        )
        artifact = (
            base if record["run_id"] == 0
            else base / "runs" / f"run_{record['run_id']:03d}"
        )
        required = [
            "run.json", "belief_final.npy", "decisions.csv",
            "candidates.csv", "trajectory.csv", "scan_poses.csv",
            "path_waypoints.csv",
        ]
        for name in required:
            path = artifact / name
            if not path.exists() or path.stat().st_size == 0:
                missing.append(str(path.relative_to(output)))
        if any(not (artifact / name).exists() for name in ("run.json", "belief_final.npy")):
            continue
        file_counts["run_json"] += 1
        file_counts["belief_npy"] += 1
        file_counts["decision_csv"] += int((artifact / "decisions.csv").exists())
        file_counts["candidate_csv"] += int((artifact / "candidates.csv").exists())
        file_counts["trajectory_csv"] += int((artifact / "trajectory.csv").exists())
        payload = json.loads((artifact / "run.json").read_text())
        final_belief = np.load(artifact / "belief_final.npy")
        replayed = np.full(final_belief.shape, -1, dtype=np.int8)
        flat = replayed.ravel()
        for step in payload["steps"]:
            for index, value in step.get("observed_updates", []):
                flat[int(index)] = int(value)
        if not np.array_equal(replayed, final_belief):
            replay_mismatches.append(str(artifact.relative_to(output)))
        environment = record["environment"]
        if environment not in truth_cache:
            truth_cache[environment] = make_env(environment).get_occupancy_map().data
        known = final_belief >= 0
        if not np.array_equal(final_belief[known], truth_cache[environment][known]):
            truth_mismatches.append(str(artifact.relative_to(output)))

        expects_media = (
            not args.no_frames and
            (args.media_runs == "all" or record["run_id"] == 0)
        )
        if expects_media:
            for name, key in (("animation.gif", "gif"), ("video.mp4", "mp4")):
                path = artifact / name
                if not path.exists() or path.stat().st_size == 0:
                    media_errors.append(str(path.relative_to(output)))
                else:
                    file_counts[key] += 1
            frames = list((artifact / "steps").glob("step_*.png"))
            file_counts["step_png"] += len(frames)
            if len(frames) != len(payload["steps"]):
                media_errors.append(
                    f"{artifact.relative_to(output)}: "
                    f"{len(frames)} frames for {len(payload['steps'])} steps"
                )

    html = (output / "index.html").read_text(encoding="utf-8")
    html_missing = []
    for reference in re.findall(r'(?:href|src)="([^"]+)"', html):
        if reference == "audit_report.json":
            continue  # Written after all other audit checks complete.
        if reference.startswith(("http://", "https://", "#")):
            continue
        if not (output / reference).exists():
            html_missing.append(reference)
    log_text = (output / "run.log").read_text(encoding="utf-8")
    log_errors = [
        marker for marker in ("Traceback", "FAILED", "video_error.txt")
        if marker in log_text
    ]
    expected_records = len(records)
    passed = not any((
        missing, replay_mismatches, truth_mismatches,
        media_errors, html_missing, log_errors,
    )) and file_counts["run_json"] == expected_records
    report = {
        "passed": passed,
        "records": expected_records,
        "file_counts": file_counts,
        "missing_files": missing,
        "belief_replay_mismatches": replay_mismatches,
        "truth_mismatches": truth_mismatches,
        "media_errors": media_errors,
        "html_missing_references": html_missing,
        "log_error_markers": log_errors,
    }
    (output / "audit_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    if not passed:
        raise RuntimeError(f"Benchmark audit failed; see {output / 'audit_report.json'}")
    return report


def main():
    args = parse_args()
    algorithms, environments, sensors, runs = profile_scope(args)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = args.output or (
        ROOT / "outputs" /
        ("unknown_ablation_runs" if args.profile == "ablation"
         else "unknown_benchmark_runs")
    )
    output = output_root / stamp
    output.mkdir(parents=True)
    manifest = {
        "created": datetime.now().isoformat(),
        "protocol": "unknown_static_grid_occlusion_aware",
        "command": sys.argv,
        "profile": args.profile,
        "algorithms": algorithms,
        "environments": environments,
        "environment_definitions": {
            key: {"generator": ENVIRONMENTS[key][0], "parameters": ENVIRONMENTS[key][1]}
            for key in environments
        },
        "sensors": {key: jsonable(SENSORS[key].__dict__) for key in sensors},
        "runs": runs,
        "seed": args.seed,
        "max_decisions": args.max_decisions,
        "common_explorer_config": {
            "target_coverage": 0.95,
            "robot_radius_m": 0.3,
            "preferred_clearance_m": 0.5,
            "target_spacing_m": 2.0,
            "scan_interval_m": 1.0,
            "min_gain_cells": 8,
            "max_frontier_candidates": 48,
            "random_candidates": 24,
            "exact_gain_budget": 18,
            "clearance_weight": 1.5,
            "spacing_weight": 0.0,
        },
        "ablation_definitions": {
            "sstg_single_centroid": "multi_frontier=False",
            "sstg_known_obstacle_only": "require_known_footprint=False",
            "sstg_no_vantage": "use_topological_vantages=False",
            "sstg_with_spacing": "spacing_weight=0.30",
        },
        "ground_truth_access": "sensor and evaluator only",
        "planning": (
            "robot-centre A* on the start-connected erosion of known-free "
            "cells by the 0.3 m robot footprint; 0.5 m is a reported "
            "preferred-clearance threshold"
        ),
        "continuous_sensing": True,
        "git": git_info(),
        "source_tree_sha256": source_fingerprint(),
        "dependencies": dependency_versions(),
        "learning_checkpoint": checkpoint_info(),
    }
    (output / "manifest.json").write_text(
        json.dumps(jsonable(manifest), indent=2), encoding="utf-8"
    )
    runner = BenchmarkRunner(output_dir=str(output), num_runs=runs, seed=args.seed)
    records = []
    with (output / "run.log").open("w", buffering=1) as log, \
            contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
        print(json.dumps(manifest, indent=2), flush=True)
        for sensor_key in sensors:
            for environment_name in environments:
                for algorithm_key in algorithms:
                    for run_id in range(runs):
                        print(
                            f"RUN {sensor_key}/{environment_name}/{algorithm_key}/{run_id}",
                            flush=True,
                        )
                        record, truth = run_experiment(
                            runner, algorithm_key, sensor_key,
                            make_env(environment_name), run_id,
                            args.seed + run_id, args.max_decisions,
                        )
                        base = (
                            output / "artifacts" / sensor_key / environment_name /
                            algorithm_key
                        )
                        artifact = base if run_id == 0 else base / "runs" / f"run_{run_id:03d}"
                        artifact.mkdir(parents=True, exist_ok=True)
                        (artifact / "run.json").write_text(
                            json.dumps(jsonable({
                                key: value for key, value in record.items()
                                if key != "belief_final"
                            }), indent=2),
                            encoding="utf-8",
                        )
                        np.save(artifact / "belief_final.npy", record["belief_final"])
                        write_tabular_trace(record, artifact)
                        for step in record["steps"]:
                            selected = step.get("selected_frontier") or {}
                            print(
                                "STEP "
                                f"{sensor_key}/{environment_name}/{algorithm_key}/"
                                f"{run_id}/{step.get('trace_id')} "
                                f"event={step.get('event')} "
                                f"coverage={step.get('coverage_after', 0):.6f} "
                                f"new_cells={step.get('new_observed_count', 0)} "
                                f"candidates={len(step.get('generated_candidates', []))} "
                                f"selected={selected.get('frontier_id')}",
                                flush=True,
                            )
                        make_media = args.media_runs == "all" or run_id == 0
                        if not args.no_frames and make_media:
                            create_media(record, truth, artifact)
                        records.append({
                            key: value for key, value in record.items()
                            if key not in (
                                "belief_final", "steps", "trajectory",
                                "execution_paths",
                            )
                        })
    (output / "results.json").write_text(
        json.dumps({"manifest": manifest, "results": jsonable(records)}, indent=2),
        encoding="utf-8",
    )
    summarize(records, output)
    known_redundancy_supplement(args.known_results, output)
    html_report(records, output)
    audit_output(records, output, args)
    latest = output_root / "latest"
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(output.name)
    print(output)


if __name__ == "__main__":
    main()
