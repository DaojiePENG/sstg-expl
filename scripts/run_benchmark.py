#!/usr/bin/env python3
"""Reproducible, self-contained SSTG-Explorer benchmark pipeline."""
from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import json
import os
import platform
import random
import subprocess
import sys
import time
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLBACKEND", "Agg")

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.stats import wilcoxon

from sstg_explorer.benchmark import BenchmarkRunner
from sstg_explorer.environments import create_environment
from sstg_explorer.visualization import visualize_exploration, visualize_exploration_step

MAIN_ALGORITHMS = ["uniform_grid", "rrt", "frontier", "nbv", "active_neural_slam", "sstg_explorer"]
ABLATION_ALGORITHMS = [
    "sstg_explorer", "sstg_euclidean", "sstg_no_recovery",
    "sstg_local_updates", "sstg_adaptive_sampling", "sstg_no_pruning",
    "sstg_no_clearance",
]
ALGORITHMS = MAIN_ALGORITHMS + [name for name in ABLATION_ALGORITHMS if name not in MAIN_ALGORITHMS]
ENVIRONMENTS = {
    "empty": ("empty", {"width": 10.0, "height": 10.0}),
    "sparse_obstacles": ("obstacles", {"width": 10.0, "height": 10.0, "num_obstacles": 5, "seed": 42}),
    "corridor": ("corridor", {"length": 15.0, "width": 2.5}),
    "multiple_rooms": ("multiple_rooms", {"width": 15.0, "height": 10.0}),
    "l_shaped_corridor": ("l_corridor", {}),
    "maze": ("maze", {"width": 12.0, "height": 12.0}),
    "dense_obstacles": ("dense_obstacles", {"width": 10.0, "height": 10.0, "num_obstacles": 15, "seed": 43}),
    "narrow_passages": ("narrow_passages", {"width": 15.0, "height": 10.0}),
    "warehouse": ("warehouse", {"width": 15.0, "height": 12.0}),
}
KWARGS = {
    "uniform_grid": {"grid_spacing": 2.0, "visit_order": "nearest"},
    "rrt": {"max_iterations": 5000, "step_size": 1.0},
    "frontier": {"target_coverage": 0.95, "max_iterations": 500, "frontier_min_size": 1},
    "nbv": {"n_candidates": 50, "target_coverage": 0.95, "max_iterations": 500},
    "active_neural_slam": {
        "checkpoint": str(ROOT / "models" / "checkpoints" / "ans_global_policy.pt"),
        "target_coverage": 0.95,
    },
    "sstg_explorer": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
    "sstg_euclidean": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
    "sstg_no_recovery": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
    "sstg_local_updates": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
    "sstg_adaptive_sampling": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
    "sstg_no_pruning": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
    "sstg_no_clearance": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
}
for _sstg_name in ABLATION_ALGORITHMS:
    KWARGS[_sstg_name]["clearance_priority_weight"] = (
        0.0 if _sstg_name == "sstg_no_clearance" else 2.0
    )


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", choices=["smoke", "full", "ablation"], default="full")
    p.add_argument("--runs", type=int, default=None)
    p.add_argument("--algorithms", nargs="+", choices=ALGORITHMS)
    p.add_argument("--environments", nargs="+", choices=list(ENVIRONMENTS))
    p.add_argument("--output", type=Path, default=ROOT / "outputs" / "benchmark_runs")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--no-frames", action="store_true", help="Skip per-step media (data is still saved).")
    p.add_argument("--media-runs", choices=["all", "representative"], default="all",
                   help="Generate media for every run (default) or only run 0.")
    return p.parse_args()


def jsonable(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, Path): return str(value)
    if isinstance(value, dict): return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [jsonable(v) for v in value]
    return value


def git_info():
    def run(*args):
        return subprocess.run(args, cwd=ROOT, text=True, capture_output=True).stdout.strip()
    return {"commit": run("git", "rev-parse", "HEAD"), "status": run("git", "status", "--short")}


def source_fingerprint():
    """Hash experiment-defining code/config files, including untracked edits."""
    digest = hashlib.sha256()
    roots = [ROOT / name for name in ("src", "scripts")]
    files = [ROOT / "setup.py", ROOT / "environment.yml", ROOT / "requirements.txt"]
    for directory in roots:
        files.extend(path for path in directory.rglob("*") if path.is_file() and "__pycache__" not in path.parts)
    for path in sorted(set(files)):
        digest.update(str(path.relative_to(ROOT)).encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def checkpoint_info():
    path = Path(KWARGS["active_neural_slam"]["checkpoint"])
    if not path.exists():
        return {"path": str(path), "exists": False}
    return {
        "path": str(path), "exists": True,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def dependency_versions():
    packages = [
        "numpy", "scipy", "scikit-image", "matplotlib", "pandas",
        "imageio", "imageio-ffmpeg", "pillow", "torch", "gdown",
    ]
    versions = {}
    for package in packages:
        try:
            versions[package] = version(package)
        except PackageNotFoundError:
            versions[package] = None
    return versions


def make_env(name):
    kind, kwargs = ENVIRONMENTS[name]
    env = create_environment(kind, **kwargs)
    env.name = name
    return env


def run_experiment(runner, algo_key, env, run_id, seed):
    random.seed(seed); np.random.seed(seed)
    algo = runner.create_algorithm(algo_key, r_view=2.0, **KWARGS[algo_key])
    grid_obj, start = env.get_occupancy_map(), env.get_start_pose()
    # Every method receives the same metric OccupancyGrid so adapters can use
    # the identical robot-inflation and A* execution protocol.
    grid = grid_obj
    started = time.perf_counter()
    result = algo.explore(grid, start, visualizer=None)
    elapsed = time.perf_counter() - started
    nodes = result.get("nodes", [])
    positions = [n.get("position", n) if isinstance(n, dict) else n for n in nodes]
    meta = result.get("metadata", {})
    steps = result.get("steps", [])
    candidate_events = [
        candidate for step in steps
        for candidate in step.get("generated_candidates", [])
    ]
    trace_metrics = {
        "num_decision_steps": len(steps),
        "num_generated_candidates": len(candidate_events),
        "num_rejected_candidates": sum(
            candidate.get("status") not in ("added", "added_soft", "recovery_added")
            for candidate in candidate_events
        ),
        "num_recovery_candidates": sum(
            candidate.get("kind") == "global_recovery"
            for candidate in candidate_events
        ),
    }
    paths = meta.get("paths") or [
        step.get("path", []) for step in steps if step.get("path")
    ]
    path_positions = runner.sample_execution_paths(paths, grid_obj.resolution)
    spatial = runner.compute_spatial_metrics(
        positions, grid_obj, path_positions=path_positions,
        required_clearance=0.5,
    )
    record = {
        "algorithm_key": algo_key, "algorithm": algo.name, "environment": env.name,
        "run_id": run_id, "seed": seed, "success": bool(result.get("success", False)),
        "coverage_ratio": float(meta.get("coverage_ratio", 0)),
        "total_distance": float(meta.get("total_distance", 0)), "num_nodes": len(nodes),
        "computation_time": elapsed,
        "coverage_efficiency": float(meta.get("coverage_ratio", 0)) / max(float(meta.get("total_distance", 0)), .01),
        "additional_metrics": {
            **{key: value for key, value in meta.items() if key != "paths"},
            **spatial, **trace_metrics,
        }, "trajectory": nodes,
        "steps": steps, "execution_paths": paths,
    }
    return record, grid_obj


def create_media(record, grid, out):
    nodes = record["trajectory"]
    execution_paths = record.get("execution_paths", [])
    decision_steps = record.get("steps", [])
    frames = out / "steps"; frames.mkdir(parents=True, exist_ok=True)
    title = f'{record["algorithm"]} · {record["environment"]}'
    if decision_steps:
        for i, step in enumerate(decision_steps):
            visualize_exploration_step(
                grid, step, r_view=2.0,
                save_path=str(frames / f"step_{i:04d}.png"),
                figsize=(11, 7), dpi=110,
                title=f"{title} · decision {i}/{len(decision_steps)-1}",
            )
    else:
        for i in range(1, len(nodes) + 1):
            visualize_exploration(
                grid, nodes[:i], r_view=2.0,
                execution_paths=execution_paths[:max(0, i - 1)],
                save_path=str(frames / f"step_{i:04d}.png"),
                figsize=(7, 7), dpi=100,
                title=f"{title} · step {i}/{len(nodes)}",
            )
    # ``bbox_inches='tight'`` can differ by a few pixels as labels change.
    # Pad every frame to a common canvas before encoding GIF/MP4.
    paths = sorted(frames.glob("*.png"))
    opened = [Image.open(p).convert("RGB") for p in paths]
    width = max((im.width for im in opened), default=0)
    height = max((im.height for im in opened), default=0)
    width += width % 2       # H.264 requires even dimensions.
    height += height % 2
    images = []
    for im in opened:
        canvas = Image.new("RGB", (width, height), "white")
        canvas.paste(im, ((width - im.width) // 2, (height - im.height) // 2))
        images.append(np.asarray(canvas))
    if images:
        imageio.mimsave(out / "animation.gif", images, duration=0.35, loop=0)
        try: imageio.mimsave(out / "video.mp4", images, fps=3, macro_block_size=1)
        except Exception as exc: (out / "video_error.txt").write_text(str(exc))
        (out / "final.png").write_bytes(paths[-1].read_bytes())


def summarize(records, out):
    metrics = [
        "coverage_ratio", "total_distance", "num_nodes", "computation_time", "coverage_efficiency",
        "avg_obstacle_distance", "min_obstacle_distance",
        "avg_boundary_distance", "min_boundary_distance", "node_safe_fraction",
        "avg_path_obstacle_distance", "min_path_obstacle_distance",
        "avg_path_boundary_distance", "min_path_boundary_distance", "path_safe_fraction",
        "mean_nn_distance", "dispersion_uniformity",
    ]
    def metric(record, name):
        return record.get(name, record.get("additional_metrics", {}).get(name, 0.0))
    groups = {}
    for r in records: groups.setdefault((r["algorithm"], r["environment"]), []).append(r)
    rows = []
    for (a, e), rs in groups.items():
        row = {"algorithm": a, "environment": e, "runs": len(rs)}
        for m in metrics:
            vals = [metric(x, m) for x in rs]; row[m + "_mean"] = float(np.mean(vals)); row[m + "_std"] = float(np.std(vals))
            row[m + "_ci95"] = float(1.96 * np.std(vals) / np.sqrt(max(len(vals), 1)))
        row["success_rate"] = float(np.mean([x["success"] for x in rs]))
        rows.append(row)
    with (out / "summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)

    aggregate = []
    for algorithm in sorted({record["algorithm"] for record in records}):
        subset = [record for record in records if record["algorithm"] == algorithm]
        aggregate.append({
            "algorithm": algorithm,
            "experiments": len(subset),
            "coverage_mean": float(np.mean([record["coverage_ratio"] for record in subset])),
            "coverage_std": float(np.std([record["coverage_ratio"] for record in subset])),
            "distance_mean": float(np.mean([record["total_distance"] for record in subset])),
            "distance_std": float(np.std([record["total_distance"] for record in subset])),
            "nodes_mean": float(np.mean([record["num_nodes"] for record in subset])),
            "time_mean": float(np.mean([record["computation_time"] for record in subset])),
            "node_clearance_mean": float(np.mean([metric(record, "avg_obstacle_distance") for record in subset])),
            "node_clearance_min_mean": float(np.mean([metric(record, "min_obstacle_distance") for record in subset])),
            "boundary_clearance_mean": float(np.mean([metric(record, "avg_boundary_distance") for record in subset])),
            "node_safe_fraction": float(np.mean([metric(record, "node_safe_fraction") for record in subset])),
            "path_clearance_mean": float(np.mean([metric(record, "avg_path_obstacle_distance") for record in subset])),
            "path_clearance_min_mean": float(np.mean([metric(record, "min_path_obstacle_distance") for record in subset])),
            "path_safe_fraction": float(np.mean([metric(record, "path_safe_fraction") for record in subset])),
            "success_rate": float(np.mean([record["success"] for record in subset])),
        })
    with (out / "aggregate.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=aggregate[0].keys())
        writer.writeheader(); writer.writerows(aggregate)

    # Cluster bootstrap over environments (not over individual nodes/candidates).
    environment_names = sorted({record["environment"] for record in records})
    algorithm_names = sorted({record["algorithm"] for record in records})
    means = {}
    for algorithm in algorithm_names:
        for environment in environment_names:
            subset = [record for record in records if record["algorithm"] == algorithm and record["environment"] == environment]
            means[(algorithm, environment)] = {
                "coverage": float(np.mean([record["coverage_ratio"] for record in subset])),
                "distance": float(np.mean([record["total_distance"] for record in subset])),
                "view_clearance": float(np.mean([
                    metric(record, "avg_obstacle_distance") for record in subset
                ])),
                "path_min_clearance": float(np.mean([
                    metric(record, "min_path_obstacle_distance") for record in subset
                ])),
            }
    rng = np.random.default_rng(42)
    pairwise = []
    if "SSTG-Explorer" in algorithm_names:
        for baseline in algorithm_names:
            if baseline == "SSTG-Explorer":
                continue
            coverage_delta = np.asarray([
                (means[("SSTG-Explorer", environment)]["coverage"] - means[(baseline, environment)]["coverage"]) * 100
                for environment in environment_names
            ])
            distance_delta = np.asarray([
                means[("SSTG-Explorer", environment)]["distance"] - means[(baseline, environment)]["distance"]
                for environment in environment_names
            ])
            view_clearance_delta = np.asarray([
                means[("SSTG-Explorer", environment)]["view_clearance"] - means[(baseline, environment)]["view_clearance"]
                for environment in environment_names
            ])
            path_min_clearance_delta = np.asarray([
                means[("SSTG-Explorer", environment)]["path_min_clearance"] - means[(baseline, environment)]["path_min_clearance"]
                for environment in environment_names
            ])
            samples = rng.integers(0, len(environment_names), size=(10000, len(environment_names)))
            coverage_boot = coverage_delta[samples].mean(axis=1)
            distance_boot = distance_delta[samples].mean(axis=1)
            view_clearance_boot = view_clearance_delta[samples].mean(axis=1)
            path_min_clearance_boot = path_min_clearance_delta[samples].mean(axis=1)
            pairwise.append({
                "baseline": baseline,
                "coverage_delta_pp": float(coverage_delta.mean()),
                "coverage_ci95_low": float(np.percentile(coverage_boot, 2.5)),
                "coverage_ci95_high": float(np.percentile(coverage_boot, 97.5)),
                "distance_delta_m": float(distance_delta.mean()),
                "distance_ci95_low": float(np.percentile(distance_boot, 2.5)),
                "distance_ci95_high": float(np.percentile(distance_boot, 97.5)),
                "view_clearance_delta_m": float(view_clearance_delta.mean()),
                "view_clearance_ci95_low": float(np.percentile(view_clearance_boot, 2.5)),
                "view_clearance_ci95_high": float(np.percentile(view_clearance_boot, 97.5)),
                "path_min_clearance_delta_m": float(path_min_clearance_delta.mean()),
                "path_min_clearance_ci95_low": float(np.percentile(path_min_clearance_boot, 2.5)),
                "path_min_clearance_ci95_high": float(np.percentile(path_min_clearance_boot, 97.5)),
                "coverage_environment_win_rate": float(np.mean(coverage_delta > 0)),
                "coverage_wilcoxon_p": float(
                    wilcoxon(coverage_delta).pvalue if np.any(coverage_delta) else 1.0
                ),
                "distance_wilcoxon_p": float(
                    wilcoxon(distance_delta).pvalue if np.any(distance_delta) else 1.0
                ),
                "view_clearance_wilcoxon_p": float(
                    wilcoxon(view_clearance_delta).pvalue if np.any(view_clearance_delta) else 1.0
                ),
                "path_min_clearance_wilcoxon_p": float(
                    wilcoxon(path_min_clearance_delta).pvalue if np.any(path_min_clearance_delta) else 1.0
                ),
            })

        def add_holm(rows, source, target):
            order = sorted(range(len(rows)), key=lambda index: rows[index][source])
            running = 0.0
            for rank, index in enumerate(order):
                adjusted = min(1.0, (len(rows) - rank) * rows[index][source])
                running = max(running, adjusted)
                rows[index][target] = running

        add_holm(pairwise, "coverage_wilcoxon_p", "coverage_holm_p")
        add_holm(pairwise, "distance_wilcoxon_p", "distance_holm_p")
        add_holm(pairwise, "view_clearance_wilcoxon_p", "view_clearance_holm_p")
        add_holm(pairwise, "path_min_clearance_wilcoxon_p", "path_min_clearance_holm_p")
    with (out / "pairwise_vs_sstg.csv").open("w", newline="") as stream:
        fields = [
            "baseline", "coverage_delta_pp", "coverage_ci95_low", "coverage_ci95_high",
            "distance_delta_m", "distance_ci95_low", "distance_ci95_high",
            "view_clearance_delta_m", "view_clearance_ci95_low", "view_clearance_ci95_high",
            "path_min_clearance_delta_m", "path_min_clearance_ci95_low", "path_min_clearance_ci95_high",
            "coverage_environment_win_rate", "coverage_wilcoxon_p",
            "coverage_holm_p", "distance_wilcoxon_p", "distance_holm_p",
            "view_clearance_wilcoxon_p", "view_clearance_holm_p",
            "path_min_clearance_wilcoxon_p", "path_min_clearance_holm_p",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(pairwise)
    algos, envs = sorted({r["algorithm"] for r in rows}), sorted({r["environment"] for r in rows})
    matrix = np.array([[next((r["coverage_ratio_mean"] * 100 for r in rows if r["algorithm"] == a and r["environment"] == e), np.nan) for e in envs] for a in algos])
    fig, ax = plt.subplots(figsize=(12, 5)); im = ax.imshow(matrix, vmin=0, vmax=100, cmap="RdYlGn", aspect="auto")
    ax.set_xticks(range(len(envs)), envs, rotation=35, ha="right"); ax.set_yticks(range(len(algos)), algos)
    for i in range(len(algos)):
        for j in range(len(envs)): ax.text(j, i, f"{matrix[i,j]:.1f}", ha="center", va="center", fontsize=8)
    fig.colorbar(im, ax=ax, label="Coverage (%)"); fig.tight_layout(); fig.savefig(out / "coverage_heatmap.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for item in aggregate:
        ax.scatter(item["distance_mean"], item["coverage_mean"] * 100, s=85)
        ax.annotate(item["algorithm"], (item["distance_mean"], item["coverage_mean"] * 100),
                    xytext=(5, 5), textcoords="offset points", fontsize=8)
    ax.set_xlabel("Mean travel distance [m]"); ax.set_ylabel("Mean coverage [%]")
    ax.set_title("Coverage–travel trade-off"); ax.grid(alpha=.25); fig.tight_layout()
    fig.savefig(out / "coverage_distance_tradeoff.png", dpi=180); plt.close(fig)

    # Safety is reported separately from coverage/travel to avoid hiding a
    # collision-prone method behind a high coverage number.
    labels = [item["algorithm"] for item in aggregate]
    x = np.arange(len(labels)); width = 0.34
    fig, ax = plt.subplots(figsize=(10.5, 5.5))
    ax.bar(x - width / 2, [item["node_clearance_mean"] for item in aggregate], width,
           label="Viewpoint clearance")
    ax.bar(x + width / 2, [item["path_clearance_mean"] for item in aggregate], width,
           label="Executed-path clearance")
    ax.axhline(0.5, color="crimson", linestyle="--", linewidth=1.2,
               label="Required robot + safety clearance (0.5 m)")
    ax.set_xticks(x, labels, rotation=25, ha="right")
    ax.set_ylabel("Mean obstacle clearance [m]")
    ax.set_title("Safety comparison under the common inflated-grid protocol")
    ax.grid(axis="y", alpha=.25); ax.legend(fontsize=8); fig.tight_layout()
    fig.savefig(out / "safety_comparison.png", dpi=180); plt.close(fig)

    with (out / "results_table.md").open("w") as stream:
        stream.write("| Algorithm | Coverage (%) | Distance (m) | Nodes | Time (s) | Success |\n")
        stream.write("|---|---:|---:|---:|---:|---:|\n")
        for item in aggregate:
            stream.write(
                f"| {item['algorithm']} | {item['coverage_mean']*100:.2f} ± {item['coverage_std']*100:.2f} "
                f"| {item['distance_mean']:.2f} ± {item['distance_std']:.2f} "
                f"| {item['nodes_mean']:.2f} | {item['time_mean']:.2f} | {item['success_rate']:.1%} |\n"
            )
    latex_escape = lambda value: value.replace("_", "\\_").replace("%", "\\%")
    with (out / "results_table.tex").open("w") as stream:
        stream.write("\\begin{tabular}{lrrrr}\\toprule\n")
        stream.write(r"Method & Coverage [\%] & Distance [m] & Nodes & Success [\%] \\ \midrule" + "\n")
        for item in aggregate:
            stream.write(
                f"{latex_escape(item['algorithm'])} & {item['coverage_mean']*100:.2f} & "
                f"{item['distance_mean']:.2f} & {item['nodes_mean']:.2f} & "
                f"{item['success_rate']*100:.1f} " + r"\\" + "\n"
            )
        stream.write("\\bottomrule\\end{tabular}\n")

    with (out / "safety_table.csv").open("w", newline="") as stream:
        fields = [
            "algorithm", "node_clearance_mean", "node_clearance_min_mean",
            "boundary_clearance_mean", "node_safe_fraction",
            "path_clearance_mean", "path_clearance_min_mean", "path_safe_fraction",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(aggregate)
    with (out / "safety_table.md").open("w") as stream:
        stream.write("| Algorithm | View avg/min clearance (m) | Boundary avg (m) | Safe views | Path avg/min clearance (m) | Safe path samples |\n")
        stream.write("|---|---:|---:|---:|---:|---:|\n")
        for item in aggregate:
            stream.write(
                f"| {item['algorithm']} | {item['node_clearance_mean']:.3f} / {item['node_clearance_min_mean']:.3f} "
                f"| {item['boundary_clearance_mean']:.3f} | {item['node_safe_fraction']:.1%} "
                f"| {item['path_clearance_mean']:.3f} / {item['path_clearance_min_mean']:.3f} "
                f"| {item['path_safe_fraction']:.1%} |\n"
            )
    with (out / "safety_table.tex").open("w") as stream:
        stream.write("\\begin{tabular}{lrrrrr}\\toprule\n")
        stream.write(r"Method & View clr. & Min clr. & Boundary & Safe views & Safe path \\ \midrule" + "\n")
        for item in aggregate:
            stream.write(
                f"{latex_escape(item['algorithm'])} & {item['node_clearance_mean']:.3f} & "
                f"{item['node_clearance_min_mean']:.3f} & {item['boundary_clearance_mean']:.3f} & "
                f"{item['node_safe_fraction']*100:.1f} & {item['path_safe_fraction']*100:.1f} " + r"\\" + "\n"
            )
        stream.write("\\bottomrule\\end{tabular}\n")
    return rows


def html_report(rows, records, out):
    cards = []
    for r in records:
        if r["run_id"] != 0: continue
        rel = f'artifacts/{r["environment"]}/{r["algorithm_key"]}'
        cards.append(f'<article><h3>{r["algorithm"]} · {r["environment"]}</h3><a href="{rel}/video.mp4"><img src="{rel}/animation.gif"></a><p>coverage {r["coverage_ratio"]:.1%} · distance {r["total_distance"]:.1f} m · nodes {r["num_nodes"]}</p><p><a href="{rel}/steps/">run 0 steps</a> · <a href="{rel}/run.json">raw data</a> · <a href="{rel}/video.mp4">MP4</a> · <a href="{rel}/runs/">runs 1–4</a></p></article>')
    body = "\n".join(cards)
    html = f'''<!doctype html><meta charset="utf-8"><title>SSTG-Explorer benchmark</title><style>body{{font:15px system-ui;margin:2rem;background:#f4f6f8}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:1rem}}article{{background:white;padding:1rem;border-radius:10px}}img{{width:100%}}table{{background:white}}code{{background:#eee}}</style><h1>SSTG-Explorer benchmark</h1><p>Self-contained results under a common 0.5 m inflated-grid A* protocol. Blue polylines are actual executed paths, not straight links between viewpoints. Click an animation for MP4; every trajectory step and raw record is retained.</p><p><a href="coverage_heatmap.png">coverage heatmap</a> · <a href="coverage_distance_tradeoff.png">coverage–distance</a> · <a href="safety_comparison.png">safety comparison</a> · <a href="safety_table.md">safety table</a> · <a href="summary.csv">per-environment CSV</a> · <a href="aggregate.csv">aggregate CSV</a> · <a href="pairwise_vs_sstg.csv">cluster-bootstrap comparison</a> · <a href="results_table.md">paper table</a> · <a href="results.json">all data</a> · <a href="manifest.json">manifest</a></p><main>{body}</main>'''
    (out / "index.html").write_text(html, encoding="utf-8")


def main():
    args = parse_args(); stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = args.output / stamp; out.mkdir(parents=True)
    if args.algorithms:
        algorithms = args.algorithms
    elif args.profile == "full":
        algorithms = MAIN_ALGORITHMS
    elif args.profile == "ablation":
        algorithms = ABLATION_ALGORITHMS
    else:
        algorithms = ["frontier", "sstg_explorer"]
    env_names = args.environments or (
        list(ENVIRONMENTS) if args.profile in ("full", "ablation")
        else ["empty", "maze"]
    )
    runs = args.runs or (5 if args.profile in ("full", "ablation") else 1)
    manifest = {"created": datetime.now().isoformat(), "command": sys.argv, "profile": args.profile, "runs": runs,
                "seed": args.seed, "algorithms": algorithms, "environments": env_names,
                "python": platform.python_version(), "platform": platform.platform(),
                "dependencies": dependency_versions(),
                "hardware": {
                    "machine": platform.machine(), "processor": platform.processor(),
                    "logical_cpu_count": os.cpu_count(),
                    "conda_environment": os.environ.get("CONDA_DEFAULT_ENV"),
                },
                "git": git_info(), "source_tree_sha256": source_fingerprint(),
                "learning_checkpoint": checkpoint_info(), "parameters": KWARGS,
                "safety_protocol": {
                    "map_resolution_m": 0.05, "robot_radius_m": 0.3,
                    "safety_margin_m": 0.2, "required_clearance_m": 0.5,
                    "execution": "common inflated-grid A*",
                }}
    (out / "manifest.json").write_text(json.dumps(jsonable(manifest), indent=2), encoding="utf-8")
    runner = BenchmarkRunner(output_dir=str(out), num_runs=runs, seed=args.seed); records = []
    with (out / "run.log").open("w", buffering=1) as log, contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
        print(json.dumps(manifest, indent=2))
        for env_name in env_names:
            for algo in algorithms:
                for run_id in range(runs):
                    print(f"RUN {env_name}/{algo}/{run_id}", flush=True)
                    record, grid = run_experiment(runner, algo, make_env(env_name), run_id, args.seed + run_id)
                    records.append(record)
                    base = out / "artifacts" / env_name / algo
                    artifact = base if run_id == 0 else base / "runs" / f"run_{run_id:03d}"
                    artifact.mkdir(parents=True, exist_ok=True)
                    (artifact / "run.json").write_text(json.dumps(jsonable(record), indent=2), encoding="utf-8")
                    make_media = args.media_runs == "all" or run_id == 0
                    if not args.no_frames and make_media: create_media(record, grid, artifact)
    (out / "results.json").write_text(json.dumps({"manifest": manifest, "results": jsonable(records)}, indent=2), encoding="utf-8")
    rows = summarize(records, out); html_report(rows, records, out)
    latest = args.output / "latest"
    if latest.is_symlink() or latest.exists(): latest.unlink()
    latest.symlink_to(out.name)
    print(out)


if __name__ == "__main__": main()
