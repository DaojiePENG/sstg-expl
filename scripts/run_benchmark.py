#!/usr/bin/env python3
"""Reproducible, self-contained SSTG-Explorer benchmark pipeline."""
from __future__ import annotations

import argparse
import contextlib
import csv
import json
import os
import platform
import random
import subprocess
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

from sstg_explorer.benchmark import BenchmarkRunner
from sstg_explorer.environments import create_environment
from sstg_explorer.visualization import visualize_exploration

ALGORITHMS = ["uniform_grid", "rrt", "frontier", "nbv", "sstg_explorer"]
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
    "rrt": {"max_iterations": 500, "step_size": 1.0},
    "frontier": {"target_coverage": 0.95, "max_iterations": 500},
    "nbv": {"n_candidates": 50, "target_coverage": 0.95, "max_iterations": 500},
    "sstg_explorer": {"d_theta": 30.0, "target_coverage": 0.95, "beta": 1.0},
}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", choices=["smoke", "full"], default="full")
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


def make_env(name):
    kind, kwargs = ENVIRONMENTS[name]
    env = create_environment(kind, **kwargs)
    env.name = name
    return env


def run_experiment(runner, algo_key, env, run_id, seed):
    random.seed(seed); np.random.seed(seed)
    algo = runner.create_algorithm(algo_key, r_view=2.0, **KWARGS[algo_key])
    grid_obj, start = env.get_occupancy_map(), env.get_start_pose()
    grid = grid_obj if "sstg" in algo.name.lower() else grid_obj.data
    started = time.perf_counter()
    result = algo.explore(grid, start, visualizer=None)
    elapsed = time.perf_counter() - started
    nodes = result.get("nodes", [])
    positions = [n.get("position", n) if isinstance(n, dict) else n for n in nodes]
    meta = result.get("metadata", {})
    spatial = runner.compute_spatial_metrics(positions, grid_obj)
    record = {
        "algorithm_key": algo_key, "algorithm": algo.name, "environment": env.name,
        "run_id": run_id, "seed": seed, "success": bool(result.get("success", False)),
        "coverage_ratio": float(meta.get("coverage_ratio", 0)),
        "total_distance": float(meta.get("total_distance", 0)), "num_nodes": len(nodes),
        "computation_time": elapsed,
        "coverage_efficiency": float(meta.get("coverage_ratio", 0)) / max(float(meta.get("total_distance", 0)), .01),
        "additional_metrics": {**meta, **spatial}, "trajectory": nodes,
    }
    return record, grid_obj


def create_media(record, grid, out):
    nodes = record["trajectory"]
    frames = out / "steps"; frames.mkdir(parents=True, exist_ok=True)
    title = f'{record["algorithm"]} · {record["environment"]}'
    for i in range(1, len(nodes) + 1):
        visualize_exploration(grid, nodes[:i], r_view=2.0, save_path=str(frames / f"step_{i:04d}.png"),
                              figsize=(7, 7), dpi=100, title=f"{title} · step {i}/{len(nodes)}")
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
        (out / "final.png").write_bytes((frames / f"step_{len(nodes):04d}.png").read_bytes())


def summarize(records, out):
    metrics = ["coverage_ratio", "total_distance", "num_nodes", "computation_time", "coverage_efficiency",
               "avg_obstacle_distance", "min_obstacle_distance", "mean_nn_distance", "dispersion_uniformity"]
    def metric(record, name):
        return record.get(name, record.get("additional_metrics", {}).get(name, 0.0))
    groups = {}
    for r in records: groups.setdefault((r["algorithm"], r["environment"]), []).append(r)
    rows = []
    for (a, e), rs in groups.items():
        row = {"algorithm": a, "environment": e, "runs": len(rs)}
        for m in metrics:
            vals = [metric(x, m) for x in rs]; row[m + "_mean"] = float(np.mean(vals)); row[m + "_std"] = float(np.std(vals))
        row["success_rate"] = float(np.mean([x["success"] for x in rs]))
        rows.append(row)
    with (out / "summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    algos, envs = sorted({r["algorithm"] for r in rows}), sorted({r["environment"] for r in rows})
    matrix = np.array([[next((r["coverage_ratio_mean"] * 100 for r in rows if r["algorithm"] == a and r["environment"] == e), np.nan) for e in envs] for a in algos])
    fig, ax = plt.subplots(figsize=(12, 5)); im = ax.imshow(matrix, vmin=0, vmax=100, cmap="RdYlGn", aspect="auto")
    ax.set_xticks(range(len(envs)), envs, rotation=35, ha="right"); ax.set_yticks(range(len(algos)), algos)
    for i in range(len(algos)):
        for j in range(len(envs)): ax.text(j, i, f"{matrix[i,j]:.1f}", ha="center", va="center", fontsize=8)
    fig.colorbar(im, ax=ax, label="Coverage (%)"); fig.tight_layout(); fig.savefig(out / "coverage_heatmap.png", dpi=180); plt.close(fig)
    return rows


def html_report(rows, records, out):
    cards = []
    for r in records:
        if r["run_id"] != 0: continue
        rel = f'artifacts/{r["environment"]}/{r["algorithm_key"]}'
        cards.append(f'<article><h3>{r["algorithm"]} · {r["environment"]}</h3><a href="{rel}/video.mp4"><img src="{rel}/animation.gif"></a><p>coverage {r["coverage_ratio"]:.1%} · distance {r["total_distance"]:.1f} m · nodes {r["num_nodes"]}</p><p><a href="{rel}/steps/">run 0 steps</a> · <a href="{rel}/run.json">raw data</a> · <a href="{rel}/video.mp4">MP4</a> · <a href="{rel}/runs/">runs 1–4</a></p></article>')
    body = "\n".join(cards)
    html = f'''<!doctype html><meta charset="utf-8"><title>SSTG-Explorer benchmark</title><style>body{{font:15px system-ui;margin:2rem;background:#f4f6f8}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:1rem}}article{{background:white;padding:1rem;border-radius:10px}}img{{width:100%}}table{{background:white}}code{{background:#eee}}</style><h1>SSTG-Explorer benchmark</h1><p>Self-contained results. Click an animation for MP4; every trajectory step and raw record is retained.</p><p><a href="coverage_heatmap.png">coverage heatmap</a> · <a href="summary.csv">summary CSV</a> · <a href="results.json">all data</a> · <a href="manifest.json">manifest</a></p><main>{body}</main>'''
    (out / "index.html").write_text(html, encoding="utf-8")


def main():
    args = parse_args(); stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = args.output / stamp; out.mkdir(parents=True)
    algorithms = args.algorithms or (ALGORITHMS if args.profile == "full" else ["frontier", "sstg_explorer"])
    env_names = args.environments or (list(ENVIRONMENTS) if args.profile == "full" else ["empty", "maze"])
    runs = args.runs or (5 if args.profile == "full" else 1)
    manifest = {"created": datetime.now().isoformat(), "command": sys.argv, "profile": args.profile, "runs": runs,
                "seed": args.seed, "algorithms": algorithms, "environments": env_names,
                "python": platform.python_version(), "platform": platform.platform(), "git": git_info(), "parameters": KWARGS}
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
