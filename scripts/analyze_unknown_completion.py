#!/usr/bin/env python3
"""Create the focused ROS unknown-completion comparison report.

The primary endpoint is evaluator-only: the first sample at which both sensor
and topological coverage reach their configured thresholds.  Policy-native
candidate exhaustion and resource fail-safes are reported separately.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_PATH = ROOT / "experiments/system_sim/configs/unknown_completion.yaml"
REPORT_SCHEMA = "sstg_unknown_completion_analysis/v1"
EXPECTED_METHODS = {"frontier", "nbv", "rrt_adapted", "ans_adapted", "sstg"}
METHOD_LABELS = {
    "frontier": "Frontier",
    "nbv": "NBV",
    "rrt_adapted": "RRT",
    "ans_adapted": "ANS",
    "sstg": "SSTG",
}
FAIL_SAFE_REASONS = {"action_budget", "distance_budget", "time_budget"}


class AnalysisError(ValueError):
    """Raised when the comparison cannot be supported by frozen evidence."""


@dataclass(frozen=True)
class CoverageSample:
    ros_time_ns: int
    distance_m: float
    sensor: float
    topological: float
    joint: float
    auc: float
    unique_endpoints: int | None
    raw_endpoint_observations: int | None
    redundant_endpoint_fraction: float | None


@dataclass(frozen=True)
class MethodResult:
    method: str
    method_label: str
    schedule_id: str
    run_output_dir: str
    equivalent_95_95_reached: bool
    distance_to_95_95_m: float | None
    ros_time_to_95_95_s: float | None
    decisions_to_95_95: int | None
    executions_to_95_95: int | None
    sensor_at_95_95: float | None
    topological_at_95_95: float | None
    joint_at_95_95: float | None
    coverage_distance_auc_at_95_95: float | None
    unique_endpoints_at_95_95: int | None
    raw_endpoint_observations_at_95_95: int | None
    redundant_endpoint_fraction_at_95_95: float | None
    terminal_reason: str
    native_termination_rule: str
    native_exhaustion_confirmed: bool
    exhaustion_confirmation: int
    exhaustion_confirmations_required: int
    full_distance_m: float
    full_decisions: int
    full_executions: int
    full_navigation_successes: int
    full_navigation_failures: int
    terminal_sensor_coverage: float
    terminal_topological_coverage: float
    terminal_joint_coverage: float
    terminal_coverage_distance_auc: float


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise AnalysisError(f"missing evidence: {path}")
    content = path.read_text(encoding="utf-8")
    if content and not content.endswith("\n"):
        raise AnalysisError(f"JSONL is not newline-terminated: {path}")
    records: list[dict[str, Any]] = []
    previous_time = -1
    for line_number, line in enumerate(content.splitlines(), 1):
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise AnalysisError(f"invalid JSON at {path}:{line_number}") from error
        if not isinstance(record, dict):
            raise AnalysisError(f"non-object JSON at {path}:{line_number}")
        ros_time_ns = int(record.get("ros_time_ns", -1))
        if ros_time_ns < previous_time:
            raise AnalysisError(f"unordered ROS timestamps in {path}")
        previous_time = ros_time_ns
        records.append(record)
    if not records:
        raise AnalysisError(f"empty evidence: {path}")
    return records


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise AnalysisError(f"{label} is not numeric")
    result = float(value)
    if not math.isfinite(result):
        raise AnalysisError(f"{label} is not finite")
    return result


def _audit_input(run_dir: Path, manifest: Mapping[str, Any], name: str) -> Path:
    execution = manifest.get("execution")
    if not isinstance(execution, Mapping):
        raise AnalysisError(f"{run_dir}: missing execution manifest")
    if execution.get("status") != "terminal_completed":
        raise AnalysisError(f"{run_dir}: execution is not terminal_completed")
    audit = execution.get("artifact_audit")
    if not isinstance(audit, Mapping) or audit.get("valid") is not True:
        raise AnalysisError(f"{run_dir}: artifact audit is not valid")
    files = audit.get("files")
    record = files.get(name) if isinstance(files, Mapping) else None
    if not isinstance(record, Mapping) or not isinstance(record.get("sha256"), str):
        raise AnalysisError(f"{run_dir}: audit lacks {name}")
    path = run_dir / name
    if _sha256(path) != record["sha256"]:
        raise AnalysisError(f"{run_dir}: {name} changed after artifact audit")
    return path


def _coverage_samples(records: Sequence[Mapping[str, Any]]) -> list[CoverageSample]:
    samples: list[CoverageSample] = []
    for record in records:
        if record.get("event") != "metrics_snapshot":
            continue
        payload = record.get("payload")
        if not isinstance(payload, Mapping):
            continue
        endpoints = payload.get("core_policy_endpoints")
        motion = payload.get("ground_truth_motion")
        if not isinstance(endpoints, Mapping) or not isinstance(motion, Mapping):
            continue
        required = (
            endpoints.get("c_i_truth_sensor"),
            endpoints.get("c_t_truth_endpoints"),
            endpoints.get("coverage_distance_auc_normalized"),
            motion.get("ground_truth_path_length_m"),
        )
        if any(value is None for value in required):
            continue
        sensor = _number(required[0], "sensor coverage")
        topological = _number(required[1], "topological coverage")
        auc = _number(required[2], "coverage-distance AUC")
        distance = _number(required[3], "ground-truth distance")
        if not (0.0 <= sensor <= 1.0 and 0.0 <= topological <= 1.0):
            raise AnalysisError("coverage lies outside [0, 1]")
        if not (0.0 <= auc <= 1.0) or distance < 0.0:
            raise AnalysisError("AUC or distance lies outside its valid range")
        if samples and distance + 1e-6 < samples[-1].distance_m:
            raise AnalysisError("ground-truth distance decreases")
        core_policy = payload.get("core_policy")
        truth_topological = (
            core_policy.get("truth_topological")
            if isinstance(core_policy, Mapping) else None
        )
        endpoint_audit = (
            truth_topological.get("endpoint_audit")
            if isinstance(truth_topological, Mapping) else None
        )
        unique_endpoints = None
        raw_endpoint_observations = None
        redundant_endpoint_fraction = None
        if isinstance(endpoint_audit, Mapping):
            if endpoint_audit.get("unique_endpoint_count") is not None:
                unique_endpoints = int(endpoint_audit["unique_endpoint_count"])
            if endpoint_audit.get("raw_endpoint_observation_count") is not None:
                raw_endpoint_observations = int(
                    endpoint_audit["raw_endpoint_observation_count"]
                )
            if endpoint_audit.get("redundant_endpoint_fraction") is not None:
                redundant_endpoint_fraction = _number(
                    endpoint_audit["redundant_endpoint_fraction"],
                    "redundant endpoint fraction",
                )
                if not 0.0 <= redundant_endpoint_fraction <= 1.0:
                    raise AnalysisError("redundant endpoint fraction lies outside [0, 1]")
        samples.append(CoverageSample(
            ros_time_ns=int(record["ros_time_ns"]),
            distance_m=distance,
            sensor=sensor,
            topological=topological,
            joint=min(sensor, topological),
            auc=auc,
            unique_endpoints=unique_endpoints,
            raw_endpoint_observations=raw_endpoint_observations,
            redundant_endpoint_fraction=redundant_endpoint_fraction,
        ))
    if not samples:
        raise AnalysisError("no evaluator core-policy coverage samples")
    return samples


def _settled_record(records: Sequence[Mapping[str, Any]]) -> Mapping[str, Any]:
    settled = [
        record for record in records
        if record.get("event") == "metrics_snapshot"
        and isinstance(record.get("payload"), Mapping)
        and record["payload"].get("reason") == "policy_session_settled"
    ]
    if len(settled) != 1:
        raise AnalysisError("expected exactly one policy_session_settled snapshot")
    return settled[0]


def analyze_run(
    row: Mapping[str, str], *, sensor_threshold: float, topological_threshold: float
) -> tuple[MethodResult, list[CoverageSample], dict[str, str]]:
    run_dir = (ROOT / row["run_output_dir"]).resolve()
    manifest_path = run_dir / "run_launch_manifest.yaml"
    if not manifest_path.is_file():
        raise AnalysisError(f"missing run manifest: {manifest_path}")
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, Mapping):
        raise AnalysisError(f"invalid run manifest: {manifest_path}")
    if manifest.get("schedule_id") != row["schedule_id"]:
        raise AnalysisError(f"schedule identity drift: {run_dir}")
    policy_path = _audit_input(run_dir, manifest, "policy_trace.jsonl")
    metrics_path = _audit_input(run_dir, manifest, "evaluation_metrics.jsonl")
    policy_records = _read_jsonl(policy_path)
    metric_records = _read_jsonl(metrics_path)
    samples = _coverage_samples(metric_records)
    settled = _settled_record(metric_records)
    settled_time = int(settled["ros_time_ns"])
    final_candidates = [sample for sample in samples if sample.ros_time_ns <= settled_time]
    if not final_candidates:
        raise AnalysisError(f"{run_dir}: no coverage at settled snapshot")
    final = final_candidates[-1]
    crossing = next((
        sample for sample in samples
        if sample.sensor >= sensor_threshold
        and sample.topological >= topological_threshold
    ), None)
    decisions = [record for record in policy_records if record.get("event") == "decision"]
    executions = [record for record in policy_records if record.get("event") == "execution"]
    finished = [record for record in policy_records if record.get("event") == "session_finished"]
    if len(finished) != 1 or not isinstance(finished[0].get("payload"), Mapping):
        raise AnalysisError(f"{run_dir}: expected one session_finished event")
    summary = finished[0]["payload"]
    confirmation = int(summary.get("exhaustion_confirmation", 0))
    required = int(summary.get("exhaustion_confirmations_required", 0))
    terminal_reason = str(summary.get("termination_reason", "unknown"))
    crossing_time = None if crossing is None else crossing.ros_time_ns
    full_successes = sum(bool(item.get("payload", {}).get("succeeded")) for item in executions)
    result = MethodResult(
        method=row["method"],
        method_label=METHOD_LABELS.get(row["method"], row["method"]),
        schedule_id=row["schedule_id"],
        run_output_dir=row["run_output_dir"],
        equivalent_95_95_reached=crossing is not None,
        distance_to_95_95_m=None if crossing is None else crossing.distance_m,
        ros_time_to_95_95_s=None if crossing is None else crossing.ros_time_ns / 1e9,
        decisions_to_95_95=None if crossing_time is None else sum(
            int(item.get("ros_time_ns", -1)) <= crossing_time for item in decisions
        ),
        executions_to_95_95=None if crossing_time is None else sum(
            int(item.get("ros_time_ns", -1)) <= crossing_time for item in executions
        ),
        sensor_at_95_95=None if crossing is None else crossing.sensor,
        topological_at_95_95=None if crossing is None else crossing.topological,
        joint_at_95_95=None if crossing is None else crossing.joint,
        coverage_distance_auc_at_95_95=None if crossing is None else crossing.auc,
        unique_endpoints_at_95_95=(
            None if crossing is None else crossing.unique_endpoints
        ),
        raw_endpoint_observations_at_95_95=(
            None if crossing is None else crossing.raw_endpoint_observations
        ),
        redundant_endpoint_fraction_at_95_95=(
            None if crossing is None else crossing.redundant_endpoint_fraction
        ),
        terminal_reason=terminal_reason,
        native_termination_rule=str(summary.get("native_termination_rule", "unknown")),
        native_exhaustion_confirmed=(
            terminal_reason == "candidate_exhaustion" and required > 0
            and confirmation >= required
        ),
        exhaustion_confirmation=confirmation,
        exhaustion_confirmations_required=required,
        full_distance_m=final.distance_m,
        full_decisions=len(decisions),
        full_executions=len(executions),
        full_navigation_successes=full_successes,
        full_navigation_failures=len(executions) - full_successes,
        terminal_sensor_coverage=final.sensor,
        terminal_topological_coverage=final.topological,
        terminal_joint_coverage=final.joint,
        terminal_coverage_distance_auc=final.auc,
    )
    return result, samples, {
        "run_launch_manifest.yaml": _sha256(manifest_path),
        "policy_trace.jsonl": _sha256(policy_path),
        "evaluation_metrics.jsonl": _sha256(metrics_path),
    }


def _csv_value(value: Any) -> Any:
    if isinstance(value, bool):
        return str(value).lower()
    if value is None:
        return ""
    return value


def _write_summary(path: Path, results: Sequence[MethodResult]) -> None:
    fields = list(asdict(results[0]))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({key: _csv_value(value) for key, value in asdict(result).items()})


def _plot_comparison(
    path: Path,
    results: Sequence[MethodResult],
    traces: Mapping[str, Sequence[CoverageSample]],
    threshold: float,
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {
        "frontier": "#4C78A8", "nbv": "#F58518", "rrt_adapted": "#54A24B",
        "ans_adapted": "#B279A2", "sstg": "#E45756",
    }
    ordered = sorted(results, key=lambda item: (
        item.distance_to_95_95_m is None,
        math.inf if item.distance_to_95_95_m is None else item.distance_to_95_95_m,
    ))
    fig, (curve_axis, distance_axis, action_axis) = plt.subplots(
        1, 3, figsize=(17, 5.2)
    )
    for result in results:
        samples = traces[result.method]
        if result.distance_to_95_95_m is not None:
            samples = [item for item in samples if item.distance_m <= result.distance_to_95_95_m + 1e-6]
        distances: list[float] = []
        joints: list[float] = []
        for sample in samples:
            if distances and math.isclose(distances[-1], sample.distance_m, abs_tol=1e-9):
                joints[-1] = sample.joint
            else:
                distances.append(sample.distance_m)
                joints.append(sample.joint)
        curve_axis.plot(distances, joints, color=colors[result.method], linewidth=2.0,
                        label=result.method_label)
        if result.distance_to_95_95_m is not None:
            curve_axis.scatter([result.distance_to_95_95_m], [result.joint_at_95_95],
                               color=colors[result.method], s=35, zorder=3)
    curve_axis.axhline(threshold, color="#444444", linestyle="--", linewidth=1.2)
    curve_axis.set(xlabel="Ground-truth travel distance (m)", ylabel="Joint coverage min(Ci, Ct)",
                   title="Primary curves truncated at first joint 95/95 crossing", ylim=(0.0, 1.02))
    curve_axis.grid(alpha=0.25)
    curve_axis.legend(loc="lower right")

    values = [item.distance_to_95_95_m if item.distance_to_95_95_m is not None else 0.0 for item in ordered]
    bars = distance_axis.bar([item.method_label for item in ordered], values,
                             color=[colors[item.method] for item in ordered])
    for bar, result in zip(bars, ordered):
        text = "not reached" if result.distance_to_95_95_m is None else f"{result.distance_to_95_95_m:.1f} m\nAUC {result.coverage_distance_auc_at_95_95:.3f}"
        distance_axis.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(values + [1.0]) * .015,
            text, ha="center", va="bottom", fontsize=9,
        )
    distance_axis.set(
        xlabel="Method", ylabel="Distance to first joint 95/95 (m)",
        title="Travel efficiency (lower is better)",
    )
    distance_axis.grid(axis="y", alpha=0.25)
    distance_axis.set_ylim(0.0, max(values + [1.0]) * 1.18)

    action_order = sorted(results, key=lambda item: (
        item.executions_to_95_95 is None,
        math.inf if item.executions_to_95_95 is None else item.executions_to_95_95,
    ))
    action_values = [item.executions_to_95_95 or 0 for item in action_order]
    action_bars = action_axis.bar(
        [item.method_label for item in action_order], action_values,
        color=[colors[item.method] for item in action_order],
    )
    for bar, result in zip(action_bars, action_order):
        if result.executions_to_95_95 is None:
            label = "not reached"
        else:
            redundancy = result.redundant_endpoint_fraction_at_95_95
            label = f"{result.executions_to_95_95} actions"
            if redundancy is not None:
                label += f"\n{100.0 * redundancy:.1f}% redundant"
        action_axis.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(action_values + [1]) * .015,
            label, ha="center", va="bottom", fontsize=9,
        )
    action_axis.set(
        xlabel="Method", ylabel="Executions to first joint 95/95",
        title="Oriented-action efficiency (lower is better)",
    )
    action_axis.grid(axis="y", alpha=0.25)
    action_axis.set_ylim(0.0, max(action_values + [1]) * 1.18)
    fig.suptitle("ROS2/Gazebo unknown-completion | development scene, one seed")
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _write_conclusion(path: Path, results: Sequence[MethodResult]) -> None:
    reached = [item for item in results if item.equivalent_95_95_reached]
    distance_rank = sorted(reached, key=lambda item: item.distance_to_95_95_m or math.inf)
    auc_rank = sorted(reached, key=lambda item: item.coverage_distance_auc_at_95_95 or -math.inf, reverse=True)
    action_rank = sorted(
        reached,
        key=lambda item: math.inf if item.executions_to_95_95 is None
        else item.executions_to_95_95,
    )
    sstg = next(item for item in results if item.method == "sstg")
    native = [item for item in results if item.native_exhaustion_confirmed]
    lines = [
        "# ROS2/Gazebo unknown-completion 集中结论",
        "",
        "> 证据等级：单个 development 房间、单个起点、单个 seed 的工程筛查；不能当作论文统计结论。策略只读取 SLAM belief，95/95 真值只由离线 evaluator 判定。",
        "",
    ]
    if distance_rank:
        best = distance_rank[0]
        lines.append(
            f"核心结论：{len(reached)}/{len(results)} 个方法达到统一 95/95 端点；"
            f"最短距离是 {best.method_label}（{best.distance_to_95_95_m:.2f} m）。"
        )
    else:
        lines.append("核心结论：本轮没有方法达到统一 95/95 端点。")
    if sstg.equivalent_95_95_reached:
        distance_position = distance_rank.index(sstg) + 1
        auc_position = auc_rank.index(sstg) + 1
        action_position = action_rank.index(sstg) + 1
        lines.append(
            f"SSTG 在阈值距离上为第 {distance_position}/{len(reached)}（{sstg.distance_to_95_95_m:.2f} m），"
            f"到该端点的覆盖—距离 AUC 为第 {auc_position}/{len(reached)}（{sstg.coverage_distance_auc_at_95_95:.3f}），"
            f"定向执行次数为第 {action_position}/{len(reached)}（{sstg.executions_to_95_95} 次）。"
        )
    lines.extend([
        "",
        "## 统一端点（核心算法比较）",
        "",
        "| 方法 | 达到95/95 | 距离/m | AUC@端点 | 决策/执行 | 唯一端点 | 冗余端点 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ])
    for item in distance_rank + [item for item in results if item not in reached]:
        lines.append(
            f"| {item.method_label} | {'是' if item.equivalent_95_95_reached else '否'} | "
            f"{'' if item.distance_to_95_95_m is None else f'{item.distance_to_95_95_m:.2f}'} | "
            f"{'' if item.coverage_distance_auc_at_95_95 is None else f'{item.coverage_distance_auc_at_95_95:.3f}'} | "
            f"{'' if item.decisions_to_95_95 is None else f'{item.decisions_to_95_95}/{item.executions_to_95_95}'} | "
            f"{'' if item.unique_endpoints_at_95_95 is None else item.unique_endpoints_at_95_95} | "
            f"{'' if item.redundant_endpoint_fraction_at_95_95 is None else f'{100.0 * item.redundant_endpoint_fraction_at_95_95:.1f}%'} |"
        )
    lines.extend([
        "",
        "## 现实终止诊断（不参与核心排序）",
        "",
        f"原生候选耗尽经 3 个新地图版本确认的方法：{', '.join(item.method_label for item in native) if native else '无'}。",
        "",
        "| 方法 | 最终终止 | 原生确认 | 全程距离/m | 最终Ci/Ct | 成功/失败导航 |",
        "|---|---|---:|---:|---:|---:|",
    ])
    for item in results:
        lines.append(
            f"| {item.method_label} | {item.terminal_reason} | "
            f"{item.exhaustion_confirmation}/{item.exhaustion_confirmations_required} | "
            f"{item.full_distance_m:.2f} | {item.terminal_sensor_coverage:.3f}/{item.terminal_topological_coverage:.3f} | "
            f"{item.full_navigation_successes}/{item.full_navigation_failures} |"
        )
    lines.extend([
        "",
        "安全、碰撞和 ATE 不用于上述核心排名；它们只应作为 ROS/真机可执行性的次级诊断。",
        "",
        "可视化：[统一对比图](procedural_equivalent_comparison.png)；逐点编号图与视频位于各 run 的 `media/unknown_completion/`。",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def analyze_study(
    schedule_dir: Path | str,
    *,
    protocol_path: Path | str = PROTOCOL_PATH,
    output_dir: Path | str | None = None,
) -> dict[str, Any]:
    schedule_dir = Path(schedule_dir).resolve()
    protocol_path = Path(protocol_path).resolve()
    protocol = yaml.safe_load(protocol_path.read_text(encoding="utf-8"))
    evaluator = protocol.get("evaluator_contract", {})
    sensor_threshold = _number(evaluator.get("sensor_success_threshold"), "sensor threshold")
    topological_threshold = _number(evaluator.get("topological_success_threshold"), "topological threshold")
    if not (0.0 < sensor_threshold <= 1.0 and 0.0 < topological_threshold <= 1.0):
        raise AnalysisError("coverage thresholds must lie in (0, 1]")
    with (schedule_dir / "run_schedule.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise AnalysisError("schedule is empty")
    methods = {row.get("method") for row in rows}
    if methods != EXPECTED_METHODS or len(rows) != len(EXPECTED_METHODS):
        raise AnalysisError(f"schedule must contain exactly {sorted(EXPECTED_METHODS)}")
    study_ids = {row.get("study_id") for row in rows}
    if len(study_ids) != 1:
        raise AnalysisError("schedule contains multiple study IDs")
    study_id = str(next(iter(study_ids)))
    if output_dir is None:
        root = (ROOT / str(protocol["outputs"]["root"])).resolve()
        output = root / "reports" / study_id
    else:
        output = Path(output_dir).resolve()
    if os.path.lexists(output):
        raise AnalysisError(f"refusing to overwrite report directory: {output}")
    results: list[MethodResult] = []
    traces: dict[str, list[CoverageSample]] = {}
    inputs: dict[str, Any] = {}
    for row in sorted(rows, key=lambda item: int(item["order_position"])):
        result, samples, hashes = analyze_run(
            row,
            sensor_threshold=sensor_threshold,
            topological_threshold=topological_threshold,
        )
        results.append(result)
        traces[result.method] = samples
        inputs[result.schedule_id] = hashes
    output.mkdir(parents=True)
    summary_path = output / "summary.csv"
    conclusion_path = output / "CONCLUSION.md"
    plot_path = output / "procedural_equivalent_comparison.png"
    _write_summary(summary_path, results)
    _write_conclusion(conclusion_path, results)
    _plot_comparison(plot_path, results, traces, min(sensor_threshold, topological_threshold))
    manifest = {
        "schema": REPORT_SCHEMA,
        "study_id": study_id,
        "evidence_tier": "development_single_scene_single_start_single_seed",
        "policy_truth_access": False,
        "primary_endpoint": "first_joint_threshold_crossing",
        "thresholds": {"sensor": sensor_threshold, "topological": topological_threshold},
        "protocol": {"path": str(protocol_path), "sha256": _sha256(protocol_path)},
        "inputs": inputs,
        "outputs": {
            path.name: {"sha256": _sha256(path), "bytes": path.stat().st_size}
            for path in (summary_path, conclusion_path, plot_path)
        },
    }
    manifest_path = output / "analysis_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {**manifest, "output_dir": str(output)}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("schedule_dir", type=Path)
    parser.add_argument("--protocol", type=Path, default=PROTOCOL_PATH)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = analyze_study(
            args.schedule_dir, protocol_path=args.protocol, output_dir=args.output_dir
        )
    except (AnalysisError, KeyError, OSError, TypeError, yaml.YAMLError) as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
