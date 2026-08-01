"""Integrity tests for the Gazebo system-simulation schedule freezer."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from scripts.generate_system_sim_schedule import (
    ScheduleError,
    freeze_schedule,
    inverse_spawn_transform,
)
from scripts.run_system_sim_schedule import (
    RunPlan,
    RunnerError,
    execute_run,
    load_run_plan,
    reserve_run_output,
    supervise_process,
    validate_completed_artifacts,
)


METHODS = ("sstg", "frontier", "nbv", "rrt_adapted")
EXPERIMENT_BUDGET = {
    "max_duration_s": 900.0,
    "max_distance_m": 150.0,
    "max_decisions": 100,
    "goal_timeout_s": 180.0,
}


def _write_yaml(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _fixture_project(tmp_path: Path) -> dict[str, object]:
    root = tmp_path / "project"
    source = root / "src" / "sstg_explorer"
    source.mkdir(parents=True)
    (source / "policy.py").write_text("POLICY_VERSION = 1\n", encoding="utf-8")

    bundle = (
        root
        / "ros2_ws/src/sstg_gazebo/worlds/development/office/dev_office_01"
    )
    bundle.mkdir(parents=True)
    (bundle / "world.sdf").write_text(
        "<sdf version='1.10'><world name='office'/></sdf>\n", encoding="utf-8"
    )
    _write_yaml(
        bundle / "evaluation/truth_map.yaml",
        {"image": "truth_map.pgm", "resolution": 0.05, "origin": [-5.0, -5.0, 0.0]},
    )
    _write_yaml(
        bundle / "metadata.yaml",
        {
            "schema": "sstg_system_sim_world/v1",
            "world_id": "dev_office_01",
            "backend": "gazebo_harmonic",
            "split": "development",
            "site_family": "multi_room_office",
            "formal_result_eligible": False,
        },
    )
    _write_yaml(
        bundle / "starts.yaml",
        {
            "schema": "sstg_system_sim_starts/v1",
            "world_id": "dev_office_01",
            "starts": [
                {"start_id": "start_a", "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0},
                {"start_id": "start_b", "x_m": 1.0, "y_m": 2.0, "yaw_deg": 90.0},
            ],
        },
    )
    _write_yaml(
        bundle / "targets.yaml",
        {
            "schema": "sstg_system_sim_targets/v1",
            "world_id": "dev_office_01",
            "targets": [{"target_id": "target_01", "x_m": 2.0, "y_m": 1.0}],
        },
    )

    experiment = root / "experiments/system_sim"
    registry_path = experiment / "registries/worlds.yaml"
    _write_yaml(
        registry_path,
        {
            "schema": "sstg_system_sim_world_registry/v1",
            "worlds": [
                {
                    "world_id": "dev_office_01",
                    "backend": "gazebo_harmonic",
                    "split": "development",
                    "site_family": "multi_room_office",
                    "bundle": bundle.relative_to(root).as_posix(),
                    "formal_result_eligible": False,
                }
            ],
        },
    )
    shared_path = experiment / "configs/shared_stack.yaml"
    _write_yaml(
        shared_path,
        {
            "schema": "sstg_system_sim_shared_stack/v1",
            "backend": "gazebo_harmonic",
            "ros_distribution": "jazzy",
            "experiment_budget": EXPERIMENT_BUDGET,
            "freeze_status": "development",
        },
    )
    condition_path = experiment / "configs/conditions/nominal.yaml"
    _write_yaml(
        condition_path,
        {
            "schema": "sstg_system_sim_condition/v1",
            "condition": "nominal",
            "lidar": {"range_noise_stddev_m": 0.0},
            "status": "development",
        },
    )
    method_paths: list[Path] = []
    for method in METHODS:
        path = experiment / f"configs/methods/{method}.yaml"
        _write_yaml(
            path,
            {
                "schema": "sstg_system_sim_method/v1",
                "method": method,
                "strategy": method,
                "coverage_objective": "joint",
                "comparison_role": "internal_algorithmic_ablation",
                "formal_method_eligible": False,
                "status": "development_adapter",
            },
        )
        method_paths.append(path)
    return {
        "root": root,
        "bundle": bundle,
        "registry": registry_path,
        "shared": shared_path,
        "condition": condition_path,
        "methods": method_paths,
        "source_paths": [root / "src", root / "ros2_ws/src"],
    }


def _freeze(
    project: dict[str, object],
    output_name: str,
    *,
    randomization_seed: int = 711,
    method_paths: list[Path] | None = None,
    force: bool = False,
    evidence_tier: str = "development",
    budget_overrides: dict[str, float | int] | None = None,
) -> tuple[dict, Path]:
    root = project["root"]
    assert isinstance(root, Path)
    output = root / f"experiments/system_sim/studies/{output_name}"
    manifest = freeze_schedule(
        root=root,
        study_id="gazebo_dev_test",
        output_dir=output,
        world_registry_path=project["registry"],
        shared_stack_path=project["shared"],
        method_paths=method_paths or project["methods"],
        condition_path=project["condition"],
        world_ids=["dev_office_01"],
        replicate_seeds=[202, 101],
        randomization_seed=randomization_seed,
        evidence_tier=evidence_tier,
        start_policy="all",
        budget_overrides=budget_overrides,
        source_paths=project["source_paths"],
        force=force,
    )
    return manifest, output


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _method_orders(rows: list[dict[str, str]]) -> dict[str, tuple[str, ...]]:
    blocks: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        blocks.setdefault(row["block_id"], []).append(row)
    return {
        block_id: tuple(
            row["method"]
            for row in sorted(block_rows, key=lambda item: int(item["order_position"]))
        )
        for block_id, block_rows in blocks.items()
    }


def test_schedule_is_matched_block_deterministic_and_hashed(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    manifest_a, output_a = _freeze(project, "freeze_a")
    manifest_b, output_b = _freeze(
        project, "freeze_b", method_paths=list(reversed(project["methods"]))
    )

    schedule_a = output_a / "run_schedule.csv"
    schedule_b = output_b / "run_schedule.csv"
    assert schedule_a.read_bytes() == schedule_b.read_bytes()
    assert manifest_a == manifest_b
    rows = _rows(schedule_a)
    assert len(rows) == 2 * 2 * len(METHODS)
    assert manifest_a["design"]["block_count"] == 4
    assert manifest_a["design"]["scheduled_run_count"] == len(rows)

    for block_rows in {
        block_id: [row for row in rows if row["block_id"] == block_id]
        for block_id in {row["block_id"] for row in rows}
    }.values():
        assert {row["method"] for row in block_rows} == set(METHODS)
        assert sorted(int(row["order_position"]) for row in block_rows) == [1, 2, 3, 4]
        for matched_field in (
            "world_id",
            "start_id",
            "condition",
            "replicate_seed",
            "world_bundle_sha256",
            "condition_config_sha256",
            "shared_stack_sha256",
            "source_tree_sha256",
        ):
            assert len({row[matched_field] for row in block_rows}) == 1

    expected_schedule_hash = hashlib.sha256(schedule_a.read_bytes()).hexdigest()
    assert manifest_a["outputs"]["run_schedule_sha256"] == expected_schedule_hash
    assert manifest_a["execution"] == {
        "simulator_invoked": False,
        "status": "not_started",
    }
    assert manifest_a["inputs"]["worlds"][0]["world_name"] == "office"
    assert manifest_a["launch"]["argument_columns"]["world_name"] == "world_name"
    assert manifest_a["experiment_budget"] == EXPERIMENT_BUDGET
    assert manifest_a["budget_provenance"] == {
        "source": "shared_stack",
        "development_overrides": {},
    }
    assert manifest_a["inputs"]["shared_stack"][
        "experiment_budget"
    ] == EXPERIMENT_BUDGET
    for field, value in EXPERIMENT_BUDGET.items():
        assert manifest_a["launch"]["argument_columns"][field] == field
        assert all(float(row[field]) == pytest.approx(float(value)) for row in rows)
    assert manifest_a["design"]["run_output_root"] == (
        "system_sim_outputs/runs/gazebo_dev_test"
    )
    assert len({row["run_output_dir"] for row in rows}) == len(rows)
    assert all(
        row["run_output_dir"].startswith(
            "system_sim_outputs/runs/gazebo_dev_test/"
        )
        for row in rows
    )
    assert all(row["world_name"] == "office" for row in rows)


def test_inverse_spawn_transform_and_90_degree_schedule_regression(
    tmp_path: Path,
) -> None:
    inverse = inverse_spawn_transform(1.0, 2.0, 90.0)
    assert inverse["spawn_yaw_rad"] == pytest.approx(math.pi / 2.0)
    assert inverse["truth_to_map_x_m"] == pytest.approx(-2.0)
    assert inverse["truth_to_map_y_m"] == pytest.approx(1.0)
    assert inverse["truth_to_map_yaw_rad"] == pytest.approx(-math.pi / 2.0)

    project = _fixture_project(tmp_path)
    manifest, output = _freeze(project, "yaw_regression")
    rows = [row for row in _rows(output / "run_schedule.csv") if row["start_id"] == "start_b"]
    assert rows
    for row in rows:
        assert float(row["start_x_m"]) == pytest.approx(1.0)
        assert float(row["start_y_m"]) == pytest.approx(2.0)
        assert float(row["start_yaw_rad"]) == pytest.approx(math.pi / 2.0)
        assert float(row["truth_to_map_x_m"]) == pytest.approx(-2.0)
        assert float(row["truth_to_map_y_m"]) == pytest.approx(1.0)
        assert float(row["truth_to_map_yaw_rad"]) == pytest.approx(-math.pi / 2.0)
        assert row["truth_registration_id"] == (
            "dev_office_01:start_b:inverse_spawn_pose"
        )
    assert manifest["inputs"]["worlds"][0]["starts"][1][
        "truth_to_map_x_m"
    ] == pytest.approx(-2.0)


def test_randomization_seed_changes_only_deterministic_method_order(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, output_a = _freeze(project, "seed_a", randomization_seed=100)
    _, output_b = _freeze(project, "seed_b", randomization_seed=101)
    rows_a = _rows(output_a / "run_schedule.csv")
    rows_b = _rows(output_b / "run_schedule.csv")

    assert _method_orders(rows_a) != _method_orders(rows_b)
    assert {
        (row["world_id"], row["start_id"], row["replicate_seed"], row["method"])
        for row in rows_a
    } == {
        (row["world_id"], row["start_id"], row["replicate_seed"], row["method"])
        for row in rows_b
    }


def test_world_and_method_edits_change_recorded_hashes(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    manifest_a, output_a = _freeze(project, "before")
    rows_a = _rows(output_a / "run_schedule.csv")

    bundle = project["bundle"]
    assert isinstance(bundle, Path)
    (bundle / "world.sdf").write_text(
        "<sdf version='1.10'><world name='changed'/></sdf>\n", encoding="utf-8"
    )
    sstg_path = project["methods"][0]
    config = yaml.safe_load(sstg_path.read_text(encoding="utf-8"))
    config["gain_weight"] = 0.75
    _write_yaml(sstg_path, config)
    manifest_b, output_b = _freeze(project, "after")
    rows_b = _rows(output_b / "run_schedule.csv")

    assert rows_a[0]["world_sdf_sha256"] != rows_b[0]["world_sdf_sha256"]
    assert rows_a[0]["world_bundle_sha256"] != rows_b[0]["world_bundle_sha256"]
    assert rows_a[0]["source_tree_sha256"] != rows_b[0]["source_tree_sha256"]
    sstg_a = next(row for row in rows_a if row["method"] == "sstg")
    sstg_b = next(row for row in rows_b if row["method"] == "sstg")
    assert sstg_a["method_config_sha256"] != sstg_b["method_config_sha256"]
    assert sstg_a["run_config_sha256"] != sstg_b["run_config_sha256"]
    assert (
        manifest_a["inputs"]["study_config_sha256"]
        != manifest_b["inputs"]["study_config_sha256"]
    )


def test_development_is_explicit_and_formal_freeze_is_refused(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    manifest, output = _freeze(project, "development")
    assert manifest["eligibility"]["evidence_tier"] == "development"
    assert manifest["eligibility"]["formal_result_eligible"] is False
    assert "evidence_tier_is_development" in manifest["eligibility"]["reasons"]
    assert any(
        reason.startswith("method_not_formal_eligible:")
        for reason in manifest["eligibility"]["reasons"]
    )
    assert all(row["formal_result_eligible"] == "false" for row in _rows(
        output / "run_schedule.csv"
    ))

    with pytest.raises(ScheduleError, match="formal schedule refused"):
        _freeze(project, "formal", evidence_tier="formal")
    assert not (
        project["root"]
        / "experiments/system_sim/studies/formal/run_schedule.csv"
    ).exists()


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("max_duration_s", None, "is missing"),
        ("max_distance_m", 0.0, "must be positive"),
        ("max_decisions", True, "positive integer"),
        ("goal_timeout_s", 901.0, "must not exceed"),
    ],
)
def test_shared_stack_budget_is_required_and_fail_closed(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    project = _fixture_project(tmp_path)
    shared_path = project["shared"]
    assert isinstance(shared_path, Path)
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    if value is None:
        del shared["experiment_budget"][field]
    else:
        shared["experiment_budget"][field] = value
    _write_yaml(shared_path, shared)

    with pytest.raises(ScheduleError, match=message):
        _freeze(project, "invalid_budget")


def test_development_budget_override_is_explicit_and_formal_override_is_refused(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    overrides = {
        "max_duration_s": 30.0,
        "max_distance_m": 5.0,
        "max_decisions": 1,
        "goal_timeout_s": 10.0,
    }
    manifest, output = _freeze(
        project, "development_smoke", budget_overrides=overrides
    )
    assert manifest["experiment_budget"] == overrides
    assert manifest["budget_provenance"] == {
        "source": "development_override",
        "development_overrides": overrides,
    }
    rows = _rows(output / "run_schedule.csv")
    for field, value in overrides.items():
        assert all(float(row[field]) == pytest.approx(float(value)) for row in rows)

    with pytest.raises(ScheduleError, match="formal schedules cannot override"):
        _freeze(
            project,
            "formal_override",
            evidence_tier="formal",
            budget_overrides={"max_decisions": 1},
        )


def test_clean_frozen_test_inputs_can_create_a_formal_schedule(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    assert isinstance(root, Path)

    registry_path = project["registry"]
    registry = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    registry["worlds"][0]["split"] = "test"
    registry["worlds"][0]["formal_result_eligible"] = True
    _write_yaml(registry_path, registry)
    bundle = project["bundle"]
    metadata = yaml.safe_load((bundle / "metadata.yaml").read_text(encoding="utf-8"))
    metadata["split"] = "test"
    metadata["formal_result_eligible"] = True
    _write_yaml(bundle / "metadata.yaml", metadata)
    shared_path = project["shared"]
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    shared["freeze_status"] = "frozen"
    _write_yaml(shared_path, shared)
    condition_path = project["condition"]
    condition = yaml.safe_load(condition_path.read_text(encoding="utf-8"))
    condition["status"] = "frozen"
    _write_yaml(condition_path, condition)
    for method_path in project["methods"]:
        method = yaml.safe_load(method_path.read_text(encoding="utf-8"))
        method["status"] = "frozen"
        method["formal_method_eligible"] = True
        _write_yaml(method_path, method)

    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(["git", "-C", str(root), "add", "."], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(root),
            "-c",
            "user.name=Schedule Test",
            "-c",
            "user.email=schedule@example.invalid",
            "commit",
            "-qm",
            "freeze fixtures",
        ],
        check=True,
    )

    manifest, output = _freeze(project, "formal_ready", evidence_tier="formal")
    assert manifest["eligibility"] == {
        "evidence_tier": "formal",
        "formal_result_eligible": True,
        "reasons": [],
    }
    assert manifest["source"]["repository_dirty"] is False
    assert manifest["source"]["repository_commit"]
    rows = _rows(output / "run_schedule.csv")
    assert all(row["formal_result_eligible"] == "true" for row in rows)
    assert all(row["eligibility_reasons"] == "" for row in rows)


def test_placeholder_condition_and_overwrite_are_guarded(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    _, output = _freeze(project, "frozen")
    with pytest.raises(ScheduleError, match="refusing to overwrite"):
        _freeze(project, "frozen")

    condition_path = project["condition"]
    assert isinstance(condition_path, Path)
    condition = yaml.safe_load(condition_path.read_text(encoding="utf-8"))
    condition["lidar"]["range_noise_stddev_m"] = "TBD_FROM_PILOT"
    condition["status"] = "placeholder_not_runnable"
    _write_yaml(condition_path, condition)
    with pytest.raises(ScheduleError, match="not runnable or contains TBD"):
        _freeze(project, "placeholder")
    assert not (output.parent / "placeholder/run_schedule.csv").exists()


def test_preexisting_run_output_is_refused_even_when_empty(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    _, output = _freeze(project, "initial")
    row = _rows(output / "run_schedule.csv")[0]
    root = project["root"]
    assert isinstance(root, Path)
    reserved = root / row["run_output_dir"]
    reserved.mkdir(parents=True)

    with pytest.raises(ScheduleError, match="pre-existing run output"):
        _freeze(project, "collision", force=True)
    assert not (output.parent / "collision/run_schedule.csv").exists()


def test_missing_start_yaw_is_rejected(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    bundle = project["bundle"]
    assert isinstance(bundle, Path)
    starts_path = bundle / "starts.yaml"
    starts = yaml.safe_load(starts_path.read_text(encoding="utf-8"))
    del starts["starts"][0]["yaw_deg"]
    _write_yaml(starts_path, starts)

    with pytest.raises(ScheduleError, match="yaw_deg must be a finite number"):
        _freeze(project, "missing_yaw")


def test_run_plan_carries_frozen_world_pose_truth_and_output_args(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner")
    row = next(
        row
        for row in _rows(schedule_dir / "run_schedule.csv")
        if row["start_id"] == "start_b"
    )
    root = project["root"]
    assert isinstance(root, Path)

    plan = load_run_plan(
        root=root,
        schedule_dir=schedule_dir,
        schedule_id=row["schedule_id"],
    )

    assert plan.launch_arguments["world_name"] == "office"
    assert float(plan.launch_arguments["start_yaw"]) == pytest.approx(math.pi / 2.0)
    assert float(plan.launch_arguments["truth_to_map_x_m"]) == pytest.approx(-2.0)
    assert float(plan.launch_arguments["truth_to_map_y_m"]) == pytest.approx(1.0)
    assert float(plan.launch_arguments["truth_to_map_yaw_rad"]) == pytest.approx(
        -math.pi / 2.0
    )
    assert plan.experiment_budget == EXPERIMENT_BUDGET
    for field, value in EXPERIMENT_BUDGET.items():
        assert float(plan.launch_arguments[field]) == pytest.approx(float(value))
        assert f"{field}:={plan.launch_arguments[field]}" in plan.command
    assert plan.output_dir == (root / row["run_output_dir"]).resolve()
    assert f"world_name:=office" in plan.command
    assert not plan.output_dir.exists()


def test_run_reservation_records_manifest_and_cannot_be_reused(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_reserve")
    row = _rows(schedule_dir / "run_schedule.csv")[0]
    root = project["root"]
    assert isinstance(root, Path)
    plan = load_run_plan(
        root=root,
        schedule_dir=schedule_dir,
        schedule_id=row["schedule_id"],
    )

    manifest_path = reserve_run_output(plan)
    run_manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    assert run_manifest["execution"]["status"] == "reserved"
    assert run_manifest["identity"]["world_name"] == "office"
    assert run_manifest["launch"]["arguments"]["world_name"] == "office"
    assert run_manifest["experiment_budget"] == EXPERIMENT_BUDGET

    with pytest.raises(RunnerError, match="existing run output"):
        reserve_run_output(plan)
    with pytest.raises(RunnerError, match="existing run output"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )


def test_runner_rejects_schedule_modified_after_freeze(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_hash")
    schedule_path = schedule_dir / "run_schedule.csv"
    row = _rows(schedule_path)[0]
    schedule_path.write_text(
        schedule_path.read_text(encoding="utf-8") + "\n", encoding="utf-8"
    )
    root = project["root"]
    assert isinstance(root, Path)

    with pytest.raises(RunnerError, match="hash disagrees"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )


def test_runner_rejects_budget_manifest_or_launch_contract_drift(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_budget_drift")
    row = _rows(schedule_dir / "run_schedule.csv")[0]
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    root = project["root"]
    assert isinstance(root, Path)

    manifest["experiment_budget"]["max_decisions"] = 99
    _write_yaml(manifest_path, manifest)
    with pytest.raises(RunnerError, match="budget provenance disagrees"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )

    manifest["experiment_budget"]["max_decisions"] = 100
    del manifest["launch"]["argument_columns"]["goal_timeout_s"]
    _write_yaml(manifest_path, manifest)
    with pytest.raises(RunnerError, match="must pass goal_timeout_s"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )


def _dummy_run_plan(tmp_path: Path, command: tuple[str, ...]) -> RunPlan:
    root = tmp_path.resolve()
    return RunPlan(
        root=root,
        schedule_dir=root / "schedule",
        schedule_id="dummy_run_001",
        study_id="dummy_study",
        schedule_sha256="a" * 64,
        output_dir=root / "run",
        launch_package="dummy_package",
        launch_file="dummy.launch.py",
        launch_arguments={"world_name": "dummy_world"},
        experiment_budget=EXPERIMENT_BUDGET,
        command=command,
        schedule_row={
            "world_id": "dummy_world_id",
            "world_name": "dummy_world",
            "start_id": "start_a",
            "method": "sstg",
            "condition": "nominal",
            "replicate_seed": "101",
        },
    )


def _artifact_writer_program() -> str:
    return "\n".join(
        (
            "import json, pathlib, sys, time",
            "output = pathlib.Path(sys.argv[1])",
            "def write_json(name, value):",
            "    (output / name).write_text(json.dumps(value) + '\\n')",
            "def write_jsonl(name, values):",
            "    text = ''.join(json.dumps(value) + '\\n' for value in values)",
            "    (output / name).write_text(text)",
            "write_json('policy_manifest.json', {",
            "    'schema': 'sstg_system_sim_policy_manifest/v1',",
            "    'truth_access': False,",
            "    'parameters': {",
            "        'max_duration_s': 900.0,",
            "        'max_distance_m': 150.0,",
            "        'max_decisions': 100,",
            "        'goal_timeout_s': 180.0}})",
            "write_json('evaluation_manifest.json', {",
            "    'schema': 'sstg_system_sim_evaluator_manifest/v2',",
            "    'truth_access': 'evaluator_only'})",
            "terminal = {'event': 'session_finished', 'payload': {}}",
            "write_jsonl('evaluation_observed_policy_trace.jsonl', [terminal])",
            "write_jsonl('evaluation_metrics.jsonl', [",
            "    {'event': 'policy_trace_ingested',",
            "     'payload': {'event': 'session_finished'}},",
            "    {'event': 'metrics_snapshot',",
            "     'payload': {'reason': 'policy_session_finished'}}])",
            "write_jsonl('policy_trace.jsonl', [",
            "    {'event': 'session_started', 'payload': {}}, terminal])",
            "print('terminal artifacts written', flush=True)",
            "if len(sys.argv) > 2 and sys.argv[2] == 'exit':",
            "    raise SystemExit(0)",
            "while True:",
            "    time.sleep(1)",
        )
    )


def test_supervisor_terminates_group_and_audits_terminal_artifacts(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "run"
    command = (
        sys.executable,
        "-c",
        _artifact_writer_program(),
        str(output_dir),
    )
    plan = _dummy_run_plan(tmp_path, command)

    result = execute_run(
        plan,
        wall_timeout_s=5.0,
        evaluator_flush_s=0.05,
        poll_interval_s=0.02,
        sigint_grace_s=0.5,
        term_grace_s=0.2,
    )

    assert result.status == "terminal_completed"
    assert result.exit_code == 0
    assert result.artifact_audit["valid"] is True
    assert "SIGINT" in result.shutdown_signals
    assert "launch.log" in result.artifact_audit["files"]
    assert all(
        len(record["sha256"]) == 64
        for record in result.artifact_audit["files"].values()
    )
    manifest = yaml.safe_load(
        (output_dir / "run_launch_manifest.yaml").read_text(encoding="utf-8")
    )
    assert manifest["execution"]["status"] == "terminal_completed"
    assert manifest["execution"]["artifact_audit"]["valid"] is True


def test_zero_returncode_before_terminal_event_is_early_exit(tmp_path: Path) -> None:
    plan = _dummy_run_plan(
        tmp_path,
        (sys.executable, "-c", "raise SystemExit(0)"),
    )

    result = execute_run(
        plan,
        wall_timeout_s=2.0,
        evaluator_flush_s=0.0,
        poll_interval_s=0.02,
        sigint_grace_s=0.1,
        term_grace_s=0.1,
    )

    assert result.status == "early_exit"
    assert result.exit_code == 3
    assert result.process_returncode == 0
    assert result.artifact_audit["valid"] is False


def test_wall_timeout_stops_process_group(tmp_path: Path) -> None:
    plan = _dummy_run_plan(
        tmp_path,
        (sys.executable, "-c", "import time; time.sleep(60)"),
    )

    result = execute_run(
        plan,
        wall_timeout_s=0.1,
        evaluator_flush_s=0.0,
        poll_interval_s=0.02,
        sigint_grace_s=0.5,
        term_grace_s=0.1,
    )

    assert result.status == "timeout"
    assert result.exit_code == 124
    assert "SIGINT" in result.shutdown_signals


def test_process_group_shutdown_escalates_to_kill_when_signals_are_ignored(
    tmp_path: Path,
) -> None:
    ignore_signals = (
        "import signal, time; "
        "signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "time.sleep(60)"
    )
    plan = _dummy_run_plan(tmp_path, (sys.executable, "-c", ignore_signals))

    result = execute_run(
        plan,
        wall_timeout_s=0.15,
        evaluator_flush_s=0.0,
        poll_interval_s=0.02,
        sigint_grace_s=0.05,
        term_grace_s=0.05,
    )

    assert result.status == "timeout"
    assert result.shutdown_signals == ("SIGINT", "SIGTERM", "SIGKILL")


def test_manual_interrupt_has_distinct_supervisor_status(tmp_path: Path) -> None:
    class FakeProcess:
        pid = 12345

        @staticmethod
        def poll():
            return None

    def interrupt(_seconds: float) -> None:
        raise KeyboardInterrupt

    def shutdown(_process: object, **_kwargs: object) -> tuple[str, ...]:
        return ("SIGINT",)

    outcome = supervise_process(
        FakeProcess(),
        trace_path=tmp_path / "missing.jsonl",
        wall_timeout_s=10.0,
        poll_interval_s=0.01,
        sleeper=interrupt,
        shutdown=shutdown,
    )

    assert outcome.status == "manual_interrupt"
    assert outcome.shutdown_signals == ("SIGINT",)


def test_artifact_audit_rejects_missing_evaluator_final_snapshot(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
    )
    metrics_path = output / "evaluation_metrics.jsonl"
    metrics = [json.loads(line) for line in metrics_path.read_text().splitlines()]
    metrics_path.write_text(json.dumps(metrics[0]) + "\n", encoding="utf-8")

    audit = validate_completed_artifacts(output)

    assert audit["valid"] is False
    assert any("final policy_session_finished snapshot" in error for error in audit["errors"])


def test_artifact_audit_rejects_runtime_budget_drift(tmp_path: Path) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
    )
    manifest_path = output / "policy_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["parameters"]["max_decisions"] = 99
    manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")

    audit = validate_completed_artifacts(
        output, expected_experiment_budget=EXPERIMENT_BUDGET
    )

    assert audit["valid"] is False
    assert any(
        "runtime experiment budget disagrees" in error
        for error in audit["errors"]
    )
