"""Decision-trace visualization for online unknown-map exploration."""
from __future__ import annotations

from collections import Counter
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Wedge
import numpy as np

from sstg_explorer.map import OccupancyGrid


def apply_observed_updates(belief: np.ndarray, updates: List[List[int]]) -> None:
    """Apply compact ``[flat_index, value]`` trace updates in-place."""
    flat = belief.ravel()
    for index, value in updates:
        flat[int(index)] = int(value)


def reconstruct_beliefs(steps: List[dict], shape: Tuple[int, int]) -> List[np.ndarray]:
    """Rebuild every belief snapshot from an auditable update stream."""
    belief = np.full(shape, -1, dtype=np.int8)
    snapshots = []
    for step in steps:
        apply_observed_updates(belief, step.get("observed_updates", []))
        snapshots.append(belief.copy())
    return snapshots


def _plot_paths(ax, paths, label=True):
    for index, path in enumerate(paths):
        if len(path) > 1:
            ax.plot(
                [point[0] for point in path],
                [point[1] for point in path],
                color="#1565c0", linewidth=2.1, alpha=0.82, zorder=5,
                label="Executed trajectory" if label and index == 0 else None,
            )


def visualize_unknown_step(
    truth: OccupancyGrid,
    belief_data: np.ndarray,
    step: dict,
    execution_paths: List,
    save_path: Optional[str] = None,
    figsize: Tuple[int, int] = (15, 6),
    dpi: int = 110,
    title: Optional[str] = None,
):
    """Plot policy-visible belief, evaluation-only truth and full trace state."""
    fig, (belief_ax, truth_ax, info_ax) = plt.subplots(
        1, 3, figsize=figsize, dpi=dpi,
        gridspec_kw={"width_ratios": [3.7, 3.7, 1.55]},
    )
    extent = [
        truth.origin[0], truth.origin[0] + truth.world_width,
        truth.origin[1], truth.origin[1] + truth.world_height,
    ]
    display = np.zeros_like(belief_data, dtype=np.int8)
    display[(belief_data >= 0) & (belief_data < 50)] = 1
    display[belief_data >= 50] = 2
    belief_ax.imshow(
        display, origin="lower", extent=extent,
        cmap=ListedColormap(["#78909c", "#fafafa", "#17212b"]),
        vmin=0, vmax=2,
    )
    belief_ax.set_title("Policy-visible belief (unknown / free / occupied)")

    truth_ax.imshow(
        truth.data, origin="lower", extent=extent,
        cmap="gray_r", vmin=0, vmax=100,
    )
    known = belief_data >= 0
    known_overlay = np.ma.masked_where(~known, known)
    truth_ax.imshow(
        known_overlay, origin="lower", extent=extent,
        cmap="Blues", alpha=0.20, vmin=0, vmax=1,
    )
    unknown_free = truth.get_free_space_mask() & (~known)
    unknown_overlay = np.ma.masked_where(~unknown_free, unknown_free)
    truth_ax.imshow(
        unknown_overlay, origin="lower", extent=extent,
        cmap="Reds", alpha=0.20, vmin=0, vmax=1,
    )
    truth_ax.set_title("Evaluation-only truth (red = unseen free space)")

    _plot_paths(belief_ax, execution_paths)
    _plot_paths(truth_ax, execution_paths, label=False)
    explored = step.get("explored_nodes", [])
    if explored:
        points = [node["position"] for node in explored]
        for ax in (belief_ax, truth_ax):
            ax.scatter(
                [point[0] for point in points], [point[1] for point in points],
                c="#1976d2", s=30, edgecolors="white", linewidths=0.5,
                zorder=7,
            )

    active = step.get("active_frontiers", [])
    if active:
        targets = [candidate["target"] for candidate in active]
        priorities = np.asarray([
            candidate.get("priority", 0.0) for candidate in active
        ], dtype=float)
        if np.ptp(priorities) > 1e-9:
            sizes = 38.0 + 52.0 * (
                (priorities - priorities.min()) / np.ptp(priorities)
            )
        else:
            sizes = np.full(len(active), 58.0)
        belief_ax.scatter(
            [point[0] for point in targets], [point[1] for point in targets],
            c="#f9a825", marker="^", s=sizes, edgecolors="#5d4037",
            linewidths=0.6, alpha=0.8, zorder=9, label="Pending candidates",
        )
        headings = np.deg2rad([
            candidate.get("heading", 0.0) for candidate in active
        ])
        belief_ax.quiver(
            [point[0] for point in targets], [point[1] for point in targets],
            np.cos(headings), np.sin(headings),
            angles="xy", scale_units="xy", scale=2.2, width=0.003,
            color="#6d4c41", alpha=0.72, zorder=9,
        )
    new = step.get("new_frontiers", [])
    if new:
        targets = [candidate["target"] for candidate in new]
        belief_ax.scatter(
            [point[0] for point in targets], [point[1] for point in targets],
            c="#00c853", marker="D", s=50, zorder=10,
            label="New candidates",
        )
    rejected_styles = {
        "pruned_gain": ("#e65100", "x"),
        "pruned_evaluation_budget": ("#757575", "x"),
        "unreachable": ("#d50000", "X"),
    }
    generated = step.get("generated_candidates", [])
    for candidate in generated:
        style = rejected_styles.get(candidate.get("status"))
        if style:
            belief_ax.scatter(
                [candidate["target"][0]], [candidate["target"][1]],
                c=style[0], marker=style[1], s=34, linewidths=1.2,
                alpha=0.75, zorder=8,
            )

    selected = step.get("selected_frontier")
    if selected:
        target = selected["target"]
        unreachable = selected.get("status") == "unreachable"
        belief_ax.scatter(
            [target[0]], [target[1]],
            c="#d50000" if unreachable else "#e91e63",
            marker="X" if unreachable else "*", s=240,
            edgecolors="black", linewidths=0.8, zorder=12,
            label="Selected unreachable" if unreachable else "Selected viewpoint",
        )
        selected_heading = np.deg2rad(selected.get("heading", 0.0))
        belief_ax.arrow(
            target[0], target[1],
            0.65 * np.cos(selected_heading),
            0.65 * np.sin(selected_heading),
            width=0.025, head_width=0.15, color="#ad1457", zorder=13,
        )
    path = step.get("path", [])
    if len(path) > 1:
        belief_ax.plot(
            [point[0] for point in path], [point[1] for point in path],
            color="#00acc1", linewidth=3.0, zorder=11,
            label="Current A* path",
        )

    pose = step.get("current_pose")
    sensor = step.get("sensor", {})
    if pose:
        x, y, heading = pose
        fov = float(sensor.get("fov_deg", 360.0))
        radius = float(sensor.get("range_m", 0.0))
        if fov >= 359.999:
            patch = Circle((x, y), radius, fill=False, linestyle="--",
                           edgecolor="#00acc1", linewidth=1.2, alpha=0.75)
        else:
            patch = Wedge(
                (x, y), radius, heading - fov / 2.0, heading + fov / 2.0,
                fill=False, linestyle="--", edgecolor="#00acc1",
                linewidth=1.2, alpha=0.75,
            )
        belief_ax.add_patch(patch)
        length = min(0.8, max(radius * 0.12, 0.3))
        belief_ax.arrow(
            x, y, length * np.cos(np.deg2rad(heading)),
            length * np.sin(np.deg2rad(heading)),
            width=0.03, head_width=0.18, color="#76ff03", zorder=13,
        )
        belief_ax.scatter([x], [y], c="#76ff03", s=65,
                          edgecolors="#1b5e20", zorder=14)

    for ax in (belief_ax, truth_ax):
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.16)
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
    if title:
        fig.suptitle(title, fontsize=13)

    counts = Counter(
        candidate.get("status", "generated") for candidate in generated
    )
    selected_text = "none"
    if selected:
        selected_text = (
            f"id={selected.get('frontier_id')}\n"
            f"kind={selected.get('kind')}\n"
            f"gain={selected.get('predicted_gain', 0)}\n"
            f"priority={selected.get('priority', 0):.3f}"
        )
    ranked = sorted(
        active, key=lambda candidate: candidate.get("priority", -np.inf),
        reverse=True,
    )[:3]
    pending_text = [
        (
            f"#{candidate.get('frontier_id')} "
            f"{candidate.get('kind', '-')[:9]} "
            f"G={candidate.get('predicted_gain', 0)} "
            f"P={candidate.get('priority', 0):.2f} "
            f"d={candidate.get('path_cost', 0):.1f}m"
        )
        for candidate in ranked
    ]
    lines = [
        "UNKNOWN-MAP STATE",
        f"trace: {step.get('trace_id', 0)}",
        f"event: {step.get('event', '-')}",
        f"FOV/range: {sensor.get('fov_deg', '-')}° / {sensor.get('range_m', '-')}m",
        "",
        f"free coverage: {step.get('coverage_before', 0):.1%}",
        f"            -> {step.get('coverage_after', 0):.1%}",
        f"gain: {step.get('coverage_gain', 0):+.2%}",
        f"known map: {step.get('known_ratio', 0):.1%}",
        f"occupied recall: {step.get('occupied_recall', 0):.1%}",
        f"new cells: {step.get('new_observed_count', 0)}",
        f"scan poses: {len(step.get('scan_poses', []))}",
        "", "SELECTED", selected_text,
        "", f"TOP PENDING ({len(active)})", *pending_text,
        "", "CANDIDATE STATES",
    ]
    lines.extend(f"{key}: {value}" for key, value in sorted(counts.items()))
    info_ax.axis("off")
    info_ax.text(
        0.02, 0.98, "\n".join(lines), va="top", ha="left",
        family="monospace", fontsize=8.3,
        bbox=dict(boxstyle="round,pad=0.7", facecolor="#fafafa",
                  edgecolor="#b0bec5"),
    )
    handles = [
        Line2D([0], [0], color="#1565c0", lw=2, label="Executed trajectory"),
        Line2D([0], [0], marker="^", color="none", markerfacecolor="#f9a825",
               markeredgecolor="#5d4037", label="Pending"),
        Line2D([0], [0], marker="D", color="none", markerfacecolor="#00c853",
               label="New"),
        Line2D([0], [0], marker="x", color="#e65100", linestyle="None",
               label="Rejected: low gain"),
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#e91e63",
               markeredgecolor="black", markersize=12, label="Selected"),
    ]
    belief_ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.10),
                     ncol=3, fontsize=8, frameon=False)
    fig.subplots_adjust(bottom=0.17, wspace=0.12)
    if save_path:
        fig.savefig(save_path, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)
