"""Integrity tests for the Gazebo system-simulation schedule freezer."""
from __future__ import annotations

import csv
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest
import yaml

import scripts.run_system_sim_schedule as system_sim_runner
from scripts.generate_system_sim_schedule import (
    CORE_BAG_REQUIRED_TOPICS,
    CORE_BAG_TOPIC_TYPES,
    CORE_BAG_TOPICS,
    ROS_GZ_BRIDGE_CONTRACT,
    ROS_MIDDLEWARE_CONTRACT,
    ScheduleError,
    freeze_schedule,
    inverse_spawn_transform,
    sha256_tree,
    validate_ros_gz_bridge_contract,
    validate_ros_middleware_contract,
)
from scripts.run_system_sim_schedule import (
    RunPlan,
    RunnerError,
    SUPERVISOR_SHUTDOWN_BEGIN,
    SUPERVISOR_SHUTDOWN_END,
    execute_run,
    load_run_plan,
    reserve_run_output,
    shutdown_process_group,
    supervise_process,
    validate_completed_artifacts,
    verify_ros_gz_bridge_runtime,
    verify_ros_middleware_runtime,
)


METHODS = ("sstg", "frontier", "nbv", "rrt_adapted")
EXPERIMENT_BUDGET = {
    "max_duration_s": 900.0,
    "max_distance_m": 150.0,
    "max_decisions": 100,
    "goal_timeout_s": 180.0,
}
RECORDING_CONTRACT = {
    "enabled": True,
    "backend": "rosbag2",
    "storage_id": "mcap",
    "storage_preset_profile": "zstd_fast",
    "output": "bags/core",
    "topics": list(CORE_BAG_TOPICS),
    "topic_types": dict(CORE_BAG_TOPIC_TYPES),
    "required_nonempty_topics": list(CORE_BAG_REQUIRED_TOPICS),
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
            "ros_gz_bridge": dict(ROS_GZ_BRIDGE_CONTRACT),
            "ros_middleware": validate_ros_middleware_contract(
                ROS_MIDDLEWARE_CONTRACT
            ),
            "physics": {
                "seed_source": "replicate_seed",
                "seed_valid_range_inclusive": [1, 0x7FFFFFFF],
            },
            "recording": RECORDING_CONTRACT,
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
                "runtime_adapter": "sstg_policy",
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
    assert manifest_a["launch"]["argument_columns"]["simulation_seed"] == (
        "replicate_seed"
    )
    assert manifest_a["seed_contract"] == {
        "seed_source": "replicate_seed",
        "valid_range_inclusive": [1, 0x7FFFFFFF],
        "launch_argument_columns": {
            "policy_seed": "replicate_seed",
            "simulation_seed": "replicate_seed",
        },
    }
    assert manifest_a["inputs"]["shared_stack"]["seed_contract"] == (
        manifest_a["seed_contract"]
    )
    assert manifest_a["ros_gz_bridge_contract"] == ROS_GZ_BRIDGE_CONTRACT
    assert manifest_a["inputs"]["shared_stack"][
        "ros_gz_bridge_contract"
    ] == ROS_GZ_BRIDGE_CONTRACT
    assert manifest_a["ros_middleware_contract"] == ROS_MIDDLEWARE_CONTRACT
    assert manifest_a["inputs"]["shared_stack"][
        "ros_middleware_contract"
    ] == ROS_MIDDLEWARE_CONTRACT
    assert manifest_a["recording_contract"] == RECORDING_CONTRACT
    assert manifest_a["inputs"]["shared_stack"]["recording_contract"] == (
        manifest_a["recording_contract"]
    )
    assert manifest_a["launch"]["fixed_arguments"]["record_bag"] == "true"
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


def test_external_runtime_adapter_is_frozen_into_schedule_and_launch(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    external = (
        project["root"]
        / "experiments/system_sim/configs/methods/frontier_external.yaml"
    )
    _write_yaml(external, {
        "schema": "sstg_system_sim_method/v1",
        "method": "frontier_mrtsp_dp_external",
        "strategy": "frontier_mrtsp_dp_external",
        "runtime_adapter": "frontier_mrtsp_dp_external",
        "coverage_objective": "joint",
        "formal_method_eligible": False,
        "status": "development_adapter_e2e_pending",
    })

    manifest, output = _freeze(
        project, "external_adapter", method_paths=[external]
    )
    row = _rows(output / "run_schedule.csv")[0]
    assert row["runtime_adapter"] == "frontier_mrtsp_dp_external"
    assert manifest["inputs"]["methods"] == [{
        "method": "frontier_mrtsp_dp_external",
        "runtime_adapter": "frontier_mrtsp_dp_external",
        "path": (
            "experiments/system_sim/configs/methods/frontier_external.yaml"
        ),
        "sha256": row["method_config_sha256"],
        "status": "development_adapter_e2e_pending",
    }]
    assert manifest["launch"]["argument_columns"]["runtime_adapter"] == (
        "runtime_adapter"
    )


def test_schedule_rejects_unknown_runtime_adapter(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    method = project["methods"][0]
    config = yaml.safe_load(method.read_text(encoding="utf-8"))
    config["runtime_adapter"] = "unregistered_adapter"
    _write_yaml(method, config)

    with pytest.raises(ScheduleError, match="unsupported runtime_adapter"):
        _freeze(project, "unknown_runtime", method_paths=[method])


def test_runner_rejects_runtime_adapter_launch_column_drift_before_reservation(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runtime_launch_drift")
    row = _rows(schedule_dir / "run_schedule.csv")[0]
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["launch"]["argument_columns"]["runtime_adapter"] = "strategy"
    _write_yaml(manifest_path, manifest)
    root = project["root"]

    with pytest.raises(RunnerError, match="must pass runtime_adapter"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )
    assert not (root / row["run_output_dir"]).exists()


def test_schedule_requires_runtime_adapter_field(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    method = project["methods"][0]
    config = yaml.safe_load(method.read_text(encoding="utf-8"))
    config.pop("runtime_adapter")
    _write_yaml(method, config)

    with pytest.raises(ScheduleError, match="invalid runtime_adapter"):
        _freeze(project, "missing_runtime", method_paths=[method])


def test_external_adapter_rejects_alias_method_identity(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    method = project["methods"][0]
    config = yaml.safe_load(method.read_text(encoding="utf-8"))
    config["runtime_adapter"] = "frontier_mrtsp_dp_external"
    _write_yaml(method, config)

    with pytest.raises(ScheduleError, match="requires method ID"):
        _freeze(project, "external_alias", method_paths=[method])


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


@pytest.mark.parametrize(
    ("physics", "message"),
    [
        (None, "physics must be a mapping"),
        (
            {
                "seed_source": "default_random_device",
                "seed_valid_range_inclusive": [1, 0x7FFFFFFF],
            },
            "seed_source must be replicate_seed",
        ),
        (
            {
                "seed_source": "replicate_seed",
                "seed_valid_range_inclusive": [0, 0xFFFFFFFF],
            },
            "seed_valid_range_inclusive must be",
        ),
    ],
)
def test_shared_stack_seed_contract_is_required_and_fail_closed(
    tmp_path: Path, physics: object, message: str
) -> None:
    project = _fixture_project(tmp_path)
    shared_path = project["shared"]
    assert isinstance(shared_path, Path)
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    shared["physics"] = physics
    _write_yaml(shared_path, shared)

    with pytest.raises(ScheduleError, match=message):
        _freeze(project, "invalid_seed_contract")


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("required_version", "1.0.22", "required_version must be '1.0.23'"),
        ("required_fix_commit", "0" * 40, "required_fix_commit must be"),
        ("system_apt_eligible", True, "system_apt_eligible must be False"),
    ],
)
def test_shared_stack_bridge_contract_is_required_and_fail_closed(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    project = _fixture_project(tmp_path)
    shared_path = project["shared"]
    assert isinstance(shared_path, Path)
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    shared["ros_gz_bridge"][field] = value
    _write_yaml(shared_path, shared)

    with pytest.raises(ScheduleError, match=message):
        _freeze(project, "invalid_bridge_contract")


def test_shared_stack_bridge_contract_rejects_missing_or_unknown_fields(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    shared_path = project["shared"]
    assert isinstance(shared_path, Path)
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    del shared["ros_gz_bridge"]["source_tag"]
    _write_yaml(shared_path, shared)
    with pytest.raises(ScheduleError, match="is missing: source_tag"):
        _freeze(project, "missing_bridge_contract_field")

    shared["ros_gz_bridge"]["source_tag"] = "1.0.23"
    shared["ros_gz_bridge"]["local_patch"] = True
    _write_yaml(shared_path, shared)
    with pytest.raises(ScheduleError, match="has unknown fields: local_patch"):
        _freeze(project, "extra_bridge_contract_field")


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("implementation", "rmw_cyclonedds_cpp", "implementation must be"),
        ("required_version", "8.4.3", "required_version must be '8.4.4'"),
        ("custom_underlays_eligible", True, "must be False"),
    ],
)
def test_shared_stack_middleware_contract_is_fail_closed(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    project = _fixture_project(tmp_path)
    shared_path = project["shared"]
    assert isinstance(shared_path, Path)
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    shared["ros_middleware"][field] = value
    _write_yaml(shared_path, shared)

    with pytest.raises(ScheduleError, match=message):
        _freeze(project, "invalid_middleware_contract")


def test_shared_stack_middleware_contract_rejects_missing_or_unknown_fields(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    shared_path = project["shared"]
    assert isinstance(shared_path, Path)
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    del shared["ros_middleware"]["required_linked_dependencies"]
    _write_yaml(shared_path, shared)
    with pytest.raises(
        ScheduleError, match="is missing: required_linked_dependencies"
    ):
        _freeze(project, "missing_middleware_contract_field")

    shared["ros_middleware"] = validate_ros_middleware_contract(
        ROS_MIDDLEWARE_CONTRACT
    )
    shared["ros_middleware"]["host_defaults_allowed"] = True
    _write_yaml(shared_path, shared)
    with pytest.raises(
        ScheduleError, match="has unknown fields: host_defaults_allowed"
    ):
        _freeze(project, "extra_middleware_contract_field")


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("enabled", False, "enabled must be True"),
        ("storage_id", "sqlite3", "storage_id must be 'mcap'"),
        ("topics", ["/map"], "topics must match"),
        (
            "topic_types",
            {"/map": "nav_msgs/msg/OccupancyGrid"},
            "topic_types must match",
        ),
        (
            "required_nonempty_topics",
            ["/map"],
            "required_nonempty_topics must match",
        ),
    ],
)
def test_shared_stack_recording_contract_is_required_and_fail_closed(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    project = _fixture_project(tmp_path)
    shared_path = project["shared"]
    assert isinstance(shared_path, Path)
    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    shared["recording"][field] = value
    _write_yaml(shared_path, shared)

    with pytest.raises(ScheduleError, match=message):
        _freeze(project, "invalid_recording_contract")


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
    assert plan.launch_arguments["record_bag"] == "true"
    assert plan.launch_arguments["simulation_seed"] == row["replicate_seed"]
    assert float(plan.launch_arguments["start_yaw"]) == pytest.approx(math.pi / 2.0)
    assert float(plan.launch_arguments["truth_to_map_x_m"]) == pytest.approx(-2.0)
    assert float(plan.launch_arguments["truth_to_map_y_m"]) == pytest.approx(1.0)
    assert float(plan.launch_arguments["truth_to_map_yaw_rad"]) == pytest.approx(
        -math.pi / 2.0
    )
    assert plan.experiment_budget == EXPERIMENT_BUDGET
    assert plan.recording_contract is not None
    assert plan.recording_contract["topics"] == list(CORE_BAG_TOPICS)
    assert plan.ros_gz_bridge_contract == ROS_GZ_BRIDGE_CONTRACT
    assert plan.ros_middleware_contract == ROS_MIDDLEWARE_CONTRACT
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
    assert run_manifest["recording_contract"] == plan.recording_contract
    assert run_manifest["ros_gz_bridge_contract"] == ROS_GZ_BRIDGE_CONTRACT
    assert run_manifest["ros_middleware_contract"] == ROS_MIDDLEWARE_CONTRACT

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


def test_runner_rejects_seed_manifest_or_launch_contract_drift(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_seed_drift")
    row = _rows(schedule_dir / "run_schedule.csv")[0]
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    root = project["root"]
    assert isinstance(root, Path)

    manifest["launch"]["argument_columns"]["simulation_seed"] = "max_decisions"
    _write_yaml(manifest_path, manifest)
    with pytest.raises(RunnerError, match="simulation_seed from replicate_seed"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )

    manifest["launch"]["argument_columns"]["simulation_seed"] = "replicate_seed"
    manifest["seed_contract"]["valid_range_inclusive"] = [0, 0x7FFFFFFF]
    _write_yaml(manifest_path, manifest)
    with pytest.raises(RunnerError, match="seed contract is unsupported"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )


def test_runner_rejects_noncanonical_or_out_of_range_row_seed(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_bad_row_seed")
    schedule_path = schedule_dir / "run_schedule.csv"
    rows = _rows(schedule_path)
    rows[0]["replicate_seed"] = "0"
    with schedule_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"]["run_schedule_sha256"] = hashlib.sha256(
        schedule_path.read_bytes()
    ).hexdigest()
    _write_yaml(manifest_path, manifest)
    root = project["root"]
    assert isinstance(root, Path)

    with pytest.raises(RunnerError, match="positive signed 32-bit"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=rows[0]["schedule_id"],
        )


def test_runner_rejects_recording_manifest_or_launch_contract_drift(
    tmp_path: Path,
) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_recording_drift")
    row = _rows(schedule_dir / "run_schedule.csv")[0]
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    root = project["root"]
    assert isinstance(root, Path)

    manifest["launch"]["fixed_arguments"]["record_bag"] = "false"
    _write_yaml(manifest_path, manifest)
    with pytest.raises(RunnerError, match="must fix record_bag=true"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )

    manifest["launch"]["fixed_arguments"]["record_bag"] = "true"
    manifest["recording_contract"]["storage_id"] = "sqlite3"
    _write_yaml(manifest_path, manifest)
    with pytest.raises(RunnerError, match="storage_id must be 'mcap'"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )


def test_runner_rejects_bridge_contract_manifest_drift(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_bridge_drift")
    row = _rows(schedule_dir / "run_schedule.csv")[0]
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["ros_gz_bridge_contract"]["required_version"] = "1.0.22"
    _write_yaml(manifest_path, manifest)
    root = project["root"]
    assert isinstance(root, Path)

    with pytest.raises(RunnerError, match="required_version must be '1.0.23'"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )


def test_runner_rejects_middleware_contract_manifest_drift(tmp_path: Path) -> None:
    project = _fixture_project(tmp_path)
    _, schedule_dir = _freeze(project, "runner_middleware_drift")
    row = _rows(schedule_dir / "run_schedule.csv")[0]
    manifest_path = schedule_dir / "schedule_freeze_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["ros_middleware_contract"]["implementation"] = (
        "rmw_cyclonedds_cpp"
    )
    _write_yaml(manifest_path, manifest)
    root = project["root"]
    assert isinstance(root, Path)

    with pytest.raises(RunnerError, match="implementation must be"):
        load_run_plan(
            root=root,
            schedule_dir=schedule_dir,
            schedule_id=row["schedule_id"],
        )


def test_source_tree_hash_excludes_nested_git_metadata(tmp_path: Path) -> None:
    root = tmp_path / "project"
    source = root / "ros2_ws/src/third_party/example"
    source.mkdir(parents=True)
    (source / "bridge.cpp").write_text("int bridge = 1;\n", encoding="utf-8")
    metadata = source / ".git"
    metadata.mkdir()
    (metadata / "HEAD").write_text("ref: refs/heads/main\n", encoding="utf-8")

    initial = sha256_tree(root, [source])
    (metadata / "HEAD").write_text("ref: refs/heads/other\n", encoding="utf-8")
    assert sha256_tree(root, [source]) == initial
    (source / "bridge.cpp").write_text("int bridge = 2;\n", encoding="utf-8")
    assert sha256_tree(root, [source]) != initial


def _configure_clean_middleware_environment(
    monkeypatch: pytest.MonkeyPatch, root: Path
) -> None:
    contract = ROS_MIDDLEWARE_CONTRACT
    monkeypatch.setenv("RMW_IMPLEMENTATION", contract["implementation"])
    for name, value in contract["required_environment"].items():
        monkeypatch.setenv(name, value)
    for name in contract["forbidden_environment"]:
        monkeypatch.delenv(name, raising=False)
    for name in list(os.environ):
        if any(
            name.startswith(prefix)
            for prefix in contract["forbidden_environment_prefixes"]
        ):
            monkeypatch.delenv(name, raising=False)
    for name in contract["prefix_path_environment"]:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv(
        "PATH", "/usr/bin:/bin:/usr/sbin:/sbin:/opt/ros/jazzy/bin"
    )
    workspace_install = root / "ros2_ws/install"
    monkeypatch.setenv(
        "AMENT_PREFIX_PATH",
        os.pathsep.join((str(workspace_install / "ros_gz_bridge"), "/opt/ros/jazzy")),
    )
    monkeypatch.setenv("COLCON_PREFIX_PATH", str(workspace_install))
    monkeypatch.setenv(
        "CMAKE_PREFIX_PATH",
        os.pathsep.join((str(workspace_install / "ros_gz_bridge"), "/opt/ros/jazzy")),
    )
    monkeypatch.setenv(
        "LD_LIBRARY_PATH",
        os.pathsep.join(
            (
                str(workspace_install / "ros_gz_bridge/lib"),
                "/opt/ros/jazzy/lib",
            )
        ),
    )
    monkeypatch.setenv(
        "PYTHONPATH",
        os.pathsep.join(
            (
                str(root / "ros2_ws/build/sstg_policy_ros"),
                "/opt/ros/jazzy/lib/python3.12/site-packages",
            )
        ),
    )


def _fake_middleware_run(
    command: list[str],
    *,
    wrong_dependency: bool = False,
    wrong_apt_version: bool = False,
    wrong_dependency_apt_version: bool = False,
    **_kwargs: object,
) -> subprocess.CompletedProcess[str]:
    arguments = list(command)
    if arguments == ["ros2", "pkg", "prefix", "rmw_fastrtps_cpp"]:
        output = "/opt/ros/jazzy\n"
    elif arguments[:3] == ["dpkg-query", "-W", "-f=${Version}\\n"]:
        versions = {
            "ros-jazzy-rmw-fastrtps-cpp": "8.4.4-1noble.20260615.124621",
            "ros-jazzy-rmw-fastrtps-shared-cpp": (
                "8.4.4-1noble.20260615.124045"
            ),
            "ros-jazzy-fastrtps": "2.14.6-1noble.20260303.233638",
            "ros-jazzy-fastcdr": "2.2.7-1noble.20260225.051855",
        }
        package = arguments[3]
        if package not in versions:
            raise AssertionError(f"unexpected apt package: {package}")
        if wrong_apt_version and package == "ros-jazzy-rmw-fastrtps-cpp":
            output = "8.4.3-1noble.invalid\n"
        elif wrong_dependency_apt_version and package == "ros-jazzy-fastrtps":
            output = "2.14.5-1noble.invalid\n"
        else:
            output = versions[package] + "\n"
    elif arguments == ["ldd", "/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so"]:
        dependency_root = "/tmp/custom" if wrong_dependency else "/opt/ros/jazzy/lib"
        output = "\n".join(
            (
                "librmw_fastrtps_shared_cpp.so => "
                f"{dependency_root}/librmw_fastrtps_shared_cpp.so (0x01)",
                f"libfastrtps.so.2.14 => {dependency_root}/libfastrtps.so.2.14 (0x02)",
                f"libfastcdr.so.2 => {dependency_root}/libfastcdr.so.2 (0x03)",
            )
        )
    else:
        raise AssertionError(f"unexpected middleware command: {arguments}")
    return subprocess.CompletedProcess(arguments, 0, output, "")


def test_middleware_runtime_attestation_is_clean_and_pinned(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = (tmp_path / "project").resolve()
    _configure_clean_middleware_environment(monkeypatch, root)
    monkeypatch.setattr(system_sim_runner.subprocess, "run", _fake_middleware_run)

    attestation = verify_ros_middleware_runtime(
        root, validate_ros_middleware_contract(ROS_MIDDLEWARE_CONTRACT)
    )

    assert attestation["implementation"] == "rmw_fastrtps_cpp"
    assert attestation["version"] == "8.4.4"
    assert attestation["apt_version"] == "8.4.4-1noble.20260615.124621"
    assert len(attestation["library"]["sha256"]) == 64
    assert set(attestation["linked_dependencies"]) == {
        "rmw_fastrtps_shared_cpp",
        "fastrtps",
        "fastcdr",
    }
    assert all(
        len(dependency["sha256"]) == 64
        for dependency in attestation["linked_dependencies"].values()
    )
    for label, contract in ROS_MIDDLEWARE_CONTRACT[
        "required_linked_dependencies"
    ].items():
        dependency = attestation["linked_dependencies"][label]
        assert dependency["sha256"] == contract["required_sha256"]
        assert dependency["apt_version"] == contract["apt_version"]
    assert attestation["environment"]["forbidden_variables_set"] == []


@pytest.mark.parametrize(
    ("failure_mode", "message"),
    [
        ("implementation", "RMW_IMPLEMENTATION must be"),
        ("required_discovery", "required ROS environment does not match"),
        ("discovery_range", "required ROS environment does not match"),
        ("default_xml", "required ROS environment does not match"),
        ("publication_mode", "custom middleware environment is forbidden"),
        ("transport", "custom middleware environment is forbidden"),
        ("profile", "custom middleware environment is forbidden"),
        ("localhost_legacy", "custom middleware environment is forbidden"),
        ("underlay", "undeclared underlay paths"),
        ("toolchain_path", "undeclared underlay paths"),
        ("empty_path", "empty segment"),
        ("build_library", "undeclared underlay paths"),
        ("linkage", "resolves outside"),
        ("apt_version", "version must be"),
        ("dependency_apt_version", "ros-jazzy-fastrtps version must be"),
        ("dependency_hash", "dependency fastrtps hash must be"),
    ],
)
def test_middleware_runtime_attestation_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_mode: str,
    message: str,
) -> None:
    root = (tmp_path / "project").resolve()
    _configure_clean_middleware_environment(monkeypatch, root)
    if failure_mode == "implementation":
        monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    elif failure_mode == "required_discovery":
        monkeypatch.setenv("ROS_DOMAIN_ID", "77")
    elif failure_mode == "discovery_range":
        monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    elif failure_mode == "default_xml":
        monkeypatch.setenv("SKIP_DEFAULT_XML", "0")
    elif failure_mode == "publication_mode":
        monkeypatch.setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")
    elif failure_mode == "transport":
        monkeypatch.setenv("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    elif failure_mode == "profile":
        monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/tmp/custom.xml")
    elif failure_mode == "localhost_legacy":
        monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    elif failure_mode == "underlay":
        monkeypatch.setenv("AMENT_PREFIX_PATH", "/tmp/custom_underlay")
    elif failure_mode == "toolchain_path":
        monkeypatch.setenv("PATH", "/tmp/conda/bin:/usr/bin")
    elif failure_mode == "empty_path":
        monkeypatch.setenv("LD_LIBRARY_PATH", ":/opt/ros/jazzy/lib")
    elif failure_mode == "build_library":
        monkeypatch.setenv(
            "LD_LIBRARY_PATH", str(root / "ros2_ws/build/custom/lib")
        )
    real_sha256_file = system_sim_runner.sha256_file
    if failure_mode == "dependency_hash":
        monkeypatch.setattr(
            system_sim_runner,
            "sha256_file",
            lambda path: (
                "0" * 64
                if Path(path).resolve().name == "libfastrtps.so.2.14.6"
                else real_sha256_file(Path(path))
            ),
        )
    monkeypatch.setattr(
        system_sim_runner.subprocess,
        "run",
        lambda command, **_kwargs: _fake_middleware_run(
            command,
            wrong_dependency=failure_mode == "linkage",
            wrong_apt_version=failure_mode == "apt_version",
            wrong_dependency_apt_version=(
                failure_mode == "dependency_apt_version"
            ),
        ),
    )

    with pytest.raises(RunnerError, match=message):
        verify_ros_middleware_runtime(root, ROS_MIDDLEWARE_CONTRACT)


def test_bridge_runtime_attestation_checks_overlay_source_and_linkage(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = (tmp_path / "project").resolve()
    contract = validate_ros_gz_bridge_contract(ROS_GZ_BRIDGE_CONTRACT)
    prefix = root / contract["required_prefix"]
    checkout = root / contract["source_checkout"]
    executable = prefix / "lib/ros_gz_bridge/parameter_bridge"
    library = prefix / "lib/libros_gz_bridge.so"
    package_xml = prefix / "share/ros_gz_bridge/package.xml"
    checkout.mkdir(parents=True)
    executable.parent.mkdir(parents=True)
    library.parent.mkdir(parents=True, exist_ok=True)
    package_xml.parent.mkdir(parents=True)
    executable.write_bytes(b"official parameter bridge\n")
    library.write_bytes(b"official bridge library\n")
    package_xml.write_text(
        "<package><name>ros_gz_bridge</name><version>1.0.23</version></package>\n",
        encoding="utf-8",
    )

    def fake_run(
        command: list[str], **_kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        arguments = list(command)
        if arguments == ["ros2", "pkg", "prefix", "ros_gz_bridge"]:
            output = f"{prefix}\n"
        elif arguments[:3] == ["git", "-C", str(checkout)]:
            action = arguments[3:]
            if action == ["rev-parse", "HEAD"]:
                output = f"{contract['source_commit']}\n"
            elif action == ["rev-list", "-n", "1", contract["source_tag"]]:
                output = f"{contract['source_commit']}\n"
            elif action == ["status", "--porcelain", "--untracked-files=all"]:
                output = ""
            elif action == [
                "merge-base",
                "--is-ancestor",
                contract["required_fix_commit"],
                "HEAD",
            ]:
                output = ""
            else:
                raise AssertionError(f"unexpected git command: {arguments}")
        elif arguments == ["ldd", str(executable)]:
            output = f"libros_gz_bridge.so => {library} (0x01)\n"
        else:
            raise AssertionError(f"unexpected runtime command: {arguments}")
        return subprocess.CompletedProcess(arguments, 0, output, "")

    monkeypatch.setattr(system_sim_runner.subprocess, "run", fake_run)

    attestation = verify_ros_gz_bridge_runtime(root, contract)

    assert attestation["version"] == "1.0.23"
    assert attestation["source_commit"] == contract["source_commit"]
    assert attestation["required_fix_ancestor"] is True
    assert attestation["parameter_bridge"]["sha256"] == hashlib.sha256(
        executable.read_bytes()
    ).hexdigest()
    assert attestation["library"]["sha256"] == hashlib.sha256(
        library.read_bytes()
    ).hexdigest()


@pytest.mark.parametrize(
    ("failure_mode", "message"),
    [
        ("prefix", "must resolve to"),
        ("version", "version must be 1.0.23"),
        ("dirty", "source checkout has local changes"),
        ("linkage", "expected overlay"),
    ],
)
def test_bridge_runtime_attestation_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_mode: str,
    message: str,
) -> None:
    root = (tmp_path / "project").resolve()
    contract = validate_ros_gz_bridge_contract(ROS_GZ_BRIDGE_CONTRACT)
    prefix = root / contract["required_prefix"]
    checkout = root / contract["source_checkout"]
    executable = prefix / "lib/ros_gz_bridge/parameter_bridge"
    library = prefix / "lib/libros_gz_bridge.so"
    wrong_library = root / "wrong/libros_gz_bridge.so"
    package_xml = prefix / "share/ros_gz_bridge/package.xml"
    checkout.mkdir(parents=True)
    executable.parent.mkdir(parents=True)
    library.parent.mkdir(parents=True, exist_ok=True)
    wrong_library.parent.mkdir(parents=True)
    package_xml.parent.mkdir(parents=True)
    executable.write_bytes(b"parameter bridge\n")
    library.write_bytes(b"required library\n")
    wrong_library.write_bytes(b"wrong library\n")
    observed_version = "1.0.22" if failure_mode == "version" else "1.0.23"
    package_xml.write_text(
        "<package><name>ros_gz_bridge</name>"
        f"<version>{observed_version}</version></package>\n",
        encoding="utf-8",
    )

    def fake_run(
        command: list[str], **_kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        arguments = list(command)
        if arguments == ["ros2", "pkg", "prefix", "ros_gz_bridge"]:
            resolved_prefix = (
                Path("/opt/ros/jazzy")
                if failure_mode == "prefix"
                else prefix
            )
            output = f"{resolved_prefix}\n"
        elif arguments[:3] == ["git", "-C", str(checkout)]:
            action = arguments[3:]
            if action in (
                ["rev-parse", "HEAD"],
                ["rev-list", "-n", "1", contract["source_tag"]],
            ):
                output = f"{contract['source_commit']}\n"
            elif action == ["status", "--porcelain", "--untracked-files=all"]:
                output = (
                    " M ros_gz_bridge/src/convert/sensor_msgs.cpp\n"
                    if failure_mode == "dirty"
                    else ""
                )
            elif action == [
                "merge-base",
                "--is-ancestor",
                contract["required_fix_commit"],
                "HEAD",
            ]:
                output = ""
            else:
                raise AssertionError(f"unexpected git command: {arguments}")
        elif arguments == ["ldd", str(executable)]:
            linked = wrong_library if failure_mode == "linkage" else library
            output = f"libros_gz_bridge.so => {linked} (0x01)\n"
        else:
            raise AssertionError(f"unexpected runtime command: {arguments}")
        return subprocess.CompletedProcess(arguments, 0, output, "")

    monkeypatch.setattr(system_sim_runner.subprocess, "run", fake_run)

    with pytest.raises(RunnerError, match=message):
        verify_ros_gz_bridge_runtime(root, contract)


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
        recording_contract=None,
        command=command,
        schedule_row={
            "world_id": "dummy_world_id",
            "world_name": "dummy_world",
            "start_id": "start_a",
            "method": "sstg",
            "runtime_adapter": "sstg_policy",
            "condition": "nominal",
            "replicate_seed": "101",
        },
    )


def test_middleware_runtime_gate_runs_before_output_reservation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    plan = replace(
        _dummy_run_plan(tmp_path, (sys.executable, "-c", "raise SystemExit(0)")),
        ros_middleware_contract=validate_ros_middleware_contract(
            ROS_MIDDLEWARE_CONTRACT
        ),
    )

    def reject_runtime(_root: Path, _contract: object) -> dict[str, object]:
        raise RunnerError("ROS environment contains undeclared underlay paths")

    monkeypatch.setattr(
        system_sim_runner,
        "verify_ros_middleware_runtime",
        reject_runtime,
    )

    with pytest.raises(RunnerError, match="undeclared underlay paths"):
        execute_run(plan, wall_timeout_s=1.0)
    assert not plan.output_dir.exists()


def test_bridge_runtime_gate_runs_before_output_reservation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    plan = replace(
        _dummy_run_plan(tmp_path, (sys.executable, "-c", "raise SystemExit(0)")),
        ros_gz_bridge_contract=dict(ROS_GZ_BRIDGE_CONTRACT),
    )

    def reject_runtime(_root: Path, _contract: object) -> dict[str, object]:
        raise RunnerError("ros_gz_bridge must resolve to the source overlay")

    monkeypatch.setattr(
        system_sim_runner,
        "verify_ros_gz_bridge_runtime",
        reject_runtime,
    )

    with pytest.raises(RunnerError, match="must resolve to the source overlay"):
        execute_run(plan, wall_timeout_s=1.0)
    assert not plan.output_dir.exists()


def test_run_manifest_preserves_bridge_runtime_attestation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    plan = replace(
        _dummy_run_plan(tmp_path, (sys.executable, "-c", "raise SystemExit(0)")),
        ros_gz_bridge_contract=dict(ROS_GZ_BRIDGE_CONTRACT),
        ros_middleware_contract=validate_ros_middleware_contract(
            ROS_MIDDLEWARE_CONTRACT
        ),
    )
    attestation = {
        "package": "ros_gz_bridge",
        "version": "1.0.23",
        "parameter_bridge": {"sha256": "a" * 64},
        "library": {"sha256": "b" * 64},
    }
    middleware_attestation = {
        "implementation": "rmw_fastrtps_cpp",
        "version": "8.4.4",
        "library": {"sha256": "c" * 64},
    }
    monkeypatch.setattr(
        system_sim_runner,
        "verify_ros_gz_bridge_runtime",
        lambda _root, _contract: attestation,
    )
    monkeypatch.setattr(
        system_sim_runner,
        "verify_ros_middleware_runtime",
        lambda _root, _contract: middleware_attestation,
    )

    result = execute_run(
        plan,
        wall_timeout_s=1.0,
        poll_interval_s=0.02,
        sigint_grace_s=0.1,
        term_grace_s=0.1,
    )

    assert result.status == "early_exit"
    manifest = yaml.safe_load((
        plan.output_dir / "run_launch_manifest.yaml"
    ).read_text(encoding="utf-8"))
    assert manifest["ros_gz_bridge_contract"] == ROS_GZ_BRIDGE_CONTRACT
    assert manifest["execution"]["ros_gz_bridge_runtime"] == attestation
    assert manifest["ros_middleware_contract"] == ROS_MIDDLEWARE_CONTRACT
    assert (
        manifest["execution"]["ros_middleware_runtime"]
        == middleware_attestation
    )


def _artifact_writer_program() -> str:
    return "\n".join(
        (
            "import json, pathlib, signal, sys, time",
            "output = pathlib.Path(sys.argv[1])",
            "def stop(*_args):",
            "    raise SystemExit(0)",
            "signal.signal(signal.SIGINT, stop)",
            "def write_json(name, value):",
            "    (output / name).write_text(json.dumps(value) + '\\n')",
            "def write_jsonl(name, values):",
            "    text = ''.join(json.dumps(value) + '\\n' for value in values)",
            "    (output / name).write_text(text)",
            "print('[INFO] [policy_node-21]: process started with pid [21]', flush=True)",
            "write_json('policy_manifest.json', {",
            "    'schema': 'sstg_system_sim_policy_manifest/v1',",
            "    'truth_access': False,",
            "    'runtime_adapter': 'sstg_policy',",
            "    'parameters': {",
            "        'max_duration_s': 900.0,",
            "        'max_distance_m': 150.0,",
            "        'max_decisions': 100,",
            "        'goal_timeout_s': 180.0}})",
            "write_json('evaluation_manifest.json', {",
            "    'schema': 'sstg_system_sim_evaluator_manifest/v2',",
            "    'truth_access': 'evaluator_only',",
            "    'parameters': {'use_sim_time': True}})",
            "started = {'event': 'session_started', 'payload': {}}",
            "terminal = {'event': 'session_finished', 'payload': {}}",
            "write_jsonl('evaluation_observed_policy_trace.jsonl', [started, terminal])",
            "write_jsonl('evaluation_metrics.jsonl', [",
            "    {'event': 'policy_trace_ingested',",
            "     'payload': {'event': 'session_finished'}},",
            "    {'event': 'metrics_snapshot',",
            "     'payload': {'reason': 'policy_session_finished'}},",
            "    {'event': 'metrics_snapshot',",
            "     'payload': {",
            "         'reason': 'policy_session_settled',",
            "         'diagnostics': {",
            "             'ate_pending_sample_count': 0,",
            "             'ate_settlement_pending': False},",
            "         'ground_truth_motion': {",
            "             'ate_pending_sample_count': 0}}}])",
            "write_jsonl('policy_trace.jsonl', [started, terminal])",
            "print('terminal artifacts written', flush=True)",
            "if len(sys.argv) > 2 and sys.argv[2] == 'exit':",
            "    raise SystemExit(0)",
            "while True:",
            "    time.sleep(1)",
        )
    )


def _write_core_bag(
    output: Path,
    *,
    empty_topic: str | None = None,
    wrong_type_topic: str | None = None,
) -> None:
    rosbag2_py = pytest.importorskip("rosbag2_py")
    bag = output / "bags" / "core"
    bag.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(bag),
            storage_id="mcap",
            storage_preset_profile="zstd_fast",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="",
            output_serialization_format="",
        ),
    )
    timestamp_ns = 1_000_000
    for index, topic in enumerate(CORE_BAG_TOPICS):
        topic_type = CORE_BAG_TOPIC_TYPES[topic]
        if topic == wrong_type_topic:
            topic_type = "std_msgs/msg/String"
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=index,
                name=topic,
                type=topic_type,
                serialization_format="cdr",
            )
        )
        if topic == empty_topic:
            continue
        for _ in range(2):
            writer.write(topic, b"\x00\x01\x00\x00", timestamp_ns)
            timestamp_ns += 1_000_000
    writer.close()
    del writer
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    information = metadata["rosbag2_bagfile_information"]
    # Jazzy's direct SequentialWriter doubles this per-file test count; the
    # ros2 bag record CLI used by system runs writes the correct aggregate.
    information["files"][0]["message_count"] = information["message_count"]
    _write_yaml(metadata_path, metadata)


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
    assert manifest["execution"]["sigint_grace_s"] == 0.5
    assert manifest["execution"]["term_grace_s"] == 0.2
    launch_log = (output_dir / "launch.log").read_text(encoding="utf-8")
    assert SUPERVISOR_SHUTDOWN_BEGIN in launch_log
    assert f"{SUPERVISOR_SHUTDOWN_END}SIGINT" in launch_log


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


def test_orderly_shutdown_sends_sigint_only_to_launch_leader(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeProcess:
        pid = 12345
        returncode = None

        def poll(self):
            return self.returncode

        @staticmethod
        def wait(timeout: float):
            del timeout
            return 0

    group_states = iter((True, False))
    leader_signals: list[tuple[int, int]] = []
    group_signals: list[tuple[int, int]] = []
    monkeypatch.setattr(
        system_sim_runner,
        "_process_group_exists",
        lambda _process_group_id: next(group_states),
    )
    monkeypatch.setattr(
        system_sim_runner.os,
        "kill",
        lambda process_id, sent_signal: leader_signals.append(
            (process_id, sent_signal)
        ),
    )
    monkeypatch.setattr(
        system_sim_runner.os,
        "killpg",
        lambda process_group_id, sent_signal: group_signals.append(
            (process_group_id, sent_signal)
        ),
    )

    sent = shutdown_process_group(FakeProcess(), sigint_grace_s=0.0)

    assert sent == ("SIGINT",)
    assert leader_signals == [(12345, signal.SIGINT)]
    assert group_signals == []


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


def test_supervisor_waits_for_settled_evaluator_snapshot(tmp_path: Path) -> None:
    trace_path = tmp_path / "policy_trace.jsonl"
    metrics_path = tmp_path / "evaluation_metrics.jsonl"
    trace_path.write_text(
        json.dumps({"event": "session_finished"}) + "\n", encoding="utf-8"
    )
    metrics_path.write_text(
        json.dumps(
            {
                "event": "metrics_snapshot",
                "payload": {"reason": "policy_session_finished"},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    class FakeProcess:
        @staticmethod
        def poll():
            return None

    elapsed = [0.0]

    def clock() -> float:
        return elapsed[0]

    def sleeper(seconds: float) -> None:
        elapsed[0] += seconds
        if elapsed[0] >= 0.2:
            with metrics_path.open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(
                        {
                            "event": "metrics_snapshot",
                            "payload": {"reason": "policy_session_settled"},
                        }
                    )
                    + "\n"
                )

    outcome = supervise_process(
        FakeProcess(),
        trace_path=trace_path,
        metrics_path=metrics_path,
        wall_timeout_s=5.0,
        evaluator_flush_s=1.0,
        poll_interval_s=0.1,
        clock=clock,
        sleeper=sleeper,
        shutdown=lambda *_args, **_kwargs: ("SIGINT",),
    )

    assert outcome.status == "terminal_observed"
    assert 0.2 <= outcome.wall_elapsed_s < 1.0


def test_artifact_audit_rejects_missing_evaluator_settled_snapshot(
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
    assert any("policy_session_settled snapshot" in error for error in audit["errors"])


def test_artifact_audit_rejects_evaluator_trace_loss(tmp_path: Path) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
    )
    observed_path = output / "evaluation_observed_policy_trace.jsonl"
    observed = [
        json.loads(line) for line in observed_path.read_text().splitlines()
    ]
    observed_path.write_text(
        json.dumps(observed[-1]) + "\n", encoding="utf-8"
    )

    audit = validate_completed_artifacts(output)

    assert audit["valid"] is False
    assert any("records disagree" in error for error in audit["errors"])


def test_artifact_audit_rejects_nonempty_settled_ate_queue(
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
    settled = metrics[-1]["payload"]
    settled["diagnostics"]["ate_pending_sample_count"] = 1
    settled["diagnostics"]["ate_settlement_pending"] = True
    settled["ground_truth_motion"]["ate_pending_sample_count"] = 1
    metrics_path.write_text(
        "".join(json.dumps(record) + "\n" for record in metrics),
        encoding="utf-8",
    )

    audit = validate_completed_artifacts(output)

    assert audit["valid"] is False
    assert "evaluation_metrics.jsonl: settled ATE queue is not empty" in audit[
        "errors"
    ]
    assert "evaluation_metrics.jsonl: ATE settlement remains pending" in audit[
        "errors"
    ]


@pytest.mark.parametrize(
    "field", ("trace_rejection_count", "topology_trace_rejection_count")
)
def test_artifact_audit_rejects_settled_trace_rejections(
    tmp_path: Path, field: str,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
    )
    metrics_path = output / "evaluation_metrics.jsonl"
    metrics = [json.loads(line) for line in metrics_path.read_text().splitlines()]
    metrics[-1]["payload"]["diagnostics"][field] = 1
    metrics_path.write_text(
        "".join(json.dumps(record) + "\n" for record in metrics),
        encoding="utf-8",
    )

    audit = validate_completed_artifacts(output)

    assert audit["valid"] is False
    assert any(field in error for error in audit["errors"])


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


def test_artifact_audit_rejects_runtime_adapter_drift(tmp_path: Path) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
    )

    audit = validate_completed_artifacts(
        output, expected_runtime_adapter="frontier_mrtsp_dp_external"
    )

    assert audit["valid"] is False
    assert any("runtime_adapter disagrees" in error for error in audit["errors"])


def test_artifact_audit_rejects_evaluator_without_simulation_clock(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
    )
    manifest_path = output / "evaluation_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["parameters"]["use_sim_time"] = False
    manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")

    audit = validate_completed_artifacts(output)

    assert audit["valid"] is False
    assert (
        "evaluation_manifest.json: use_sim_time must be true"
        in audit["errors"]
    )


def test_artifact_audit_hashes_core_mcap_and_requires_key_topics(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        subprocess.run(
            [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
            check=True,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
    _write_core_bag(output)

    audit = validate_completed_artifacts(
        output, expected_recording_contract=RECORDING_CONTRACT
    )

    assert audit["valid"] is True, audit["errors"]
    assert audit["completion_checks"]["core_bag_complete"] is True
    assert audit["core_bag"]["message_count"] > 0
    assert "bags/core/metadata.yaml" in audit["files"]
    assert "bags/core/core_0.mcap" in audit["files"]

    metadata = output / "bags/core/metadata.yaml"
    value = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    records = value["rosbag2_bagfile_information"]["topics_with_message_count"]
    next(
        record
        for record in records
        if record["topic_metadata"]["name"] == "/scan"
    )["message_count"] = 0
    _write_yaml(metadata, value)
    missing = validate_completed_artifacts(
        output, expected_recording_contract=RECORDING_CONTRACT
    )
    assert missing["valid"] is False
    assert any(
        "required topic is empty or absent: /scan" in error
        for error in missing["errors"]
    )


def test_artifact_audit_reads_mcap_to_eof_and_checks_types_and_counts(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        subprocess.run(
            [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
            check=True,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
    _write_core_bag(output, wrong_type_topic="/scan")

    wrong_type = validate_completed_artifacts(
        output, expected_recording_contract=RECORDING_CONTRACT
    )
    assert wrong_type["valid"] is False
    assert any(
        "topic type disagrees with recording contract: /scan" in error
        for error in wrong_type["errors"]
    )

    metadata_path = output / "bags/core/metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    metadata["rosbag2_bagfile_information"]["message_count"] += 1
    _write_yaml(metadata_path, metadata)
    count_drift = validate_completed_artifacts(
        output, expected_recording_contract=RECORDING_CONTRACT
    )
    assert count_drift["valid"] is False
    assert "core bag topic counts do not sum to message_count" in count_drift["errors"]


def test_artifact_audit_rejects_truncated_mcap(tmp_path: Path) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    _write_core_bag(output)
    metadata = yaml.safe_load(
        (output / "bags/core/metadata.yaml").read_text(encoding="utf-8")
    )
    relative = metadata["rosbag2_bagfile_information"]["relative_file_paths"][0]
    mcap_path = output / "bags/core" / relative
    content = mcap_path.read_bytes()
    mcap_path.write_bytes(content[:-8])

    audit = validate_completed_artifacts(
        output, expected_recording_contract=RECORDING_CONTRACT
    )

    assert audit["valid"] is False
    assert any("MCAP framing is invalid" in error for error in audit["errors"])


def test_artifact_audit_rejects_core_bag_outside_run_output(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (output / "bags").symlink_to(outside, target_is_directory=True)
    _write_core_bag(output)

    audit = validate_completed_artifacts(
        output, expected_recording_contract=RECORDING_CONTRACT
    )

    assert audit["valid"] is False
    assert "core bag directory escapes the run output" in audit["errors"]


def test_artifact_audit_rejects_child_crash_but_allows_coordinated_sigint(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        subprocess.run(
            [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
            check=True,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        launch_log.write(
            "[ERROR] [gazebo-1]: process has died "
            "[pid 10, exit code -2, cmd gz]\n"
        )

    premature = validate_completed_artifacts(output)
    assert premature["valid"] is False
    assert any("gazebo-1 exit code -2" in error for error in premature["errors"])

    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        launch_log.write(f"{SUPERVISOR_SHUTDOWN_BEGIN}\n")
        launch_log.write(
            "[ERROR] [gazebo-1]: process has died "
            "[pid 10, exit code -2, cmd gz]\n"
        )
        launch_log.write(
            "[ERROR] [planner_server-2]: process has died "
            "[pid 12, exit code -15, cmd planner_server]\n"
        )
        launch_log.write(
            "[ERROR] [stuck_process-3]: process has died "
            "[pid 13, exit code -9, cmd stuck_process]\n"
        )
        launch_log.write(
            f"{SUPERVISOR_SHUTDOWN_END}SIGINT,SIGTERM,SIGKILL\n"
        )

    clean = validate_completed_artifacts(output)
    assert clean["valid"] is True
    assert clean["completion_checks"]["launch_log_clean"] is True

    with (output / "launch.log").open("a", encoding="utf-8") as launch_log:
        launch_log.write("[parameter_bridge-8] corrupted double-linked list\n")
        launch_log.write(
            "[ERROR] [parameter_bridge-8]: process has died "
            "[pid 11, exit code -6, cmd parameter_bridge]\n"
        )
    crashed = validate_completed_artifacts(output)
    assert crashed["valid"] is False
    assert crashed["completion_checks"]["launch_log_clean"] is False
    assert any(
        "parameter_bridge-8 exit code -6" in error
        for error in crashed["errors"]
    )


def test_artifact_audit_rejects_required_process_clean_exit_before_shutdown(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
    )
    (output / "launch.log").write_text(
        "[INFO] [policy_node-21]: process has finished cleanly [pid 21]\n"
        f"{SUPERVISOR_SHUTDOWN_BEGIN}\n"
        f"{SUPERVISOR_SHUTDOWN_END}SIGINT\n",
        encoding="utf-8",
    )

    audit = validate_completed_artifacts(output)

    assert audit["valid"] is False
    assert any(
        "required process exited before coordinated shutdown: policy_node-21"
        in error
        for error in audit["errors"]
    )


def test_artifact_audit_requires_selected_adapter_processes(
    tmp_path: Path,
) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    manifest_path = output / "policy_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["runtime_adapter"] = "frontier_mrtsp_dp_external"
    manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")
    (output / "launch.log").write_text(
        "[INFO] [frontier_explorer-3]: process started with pid [30]\n",
        encoding="utf-8",
    )

    audit = validate_completed_artifacts(
        output, expected_runtime_adapter="frontier_mrtsp_dp_external"
    )

    assert audit["valid"] is False
    assert (
        "launch.log: required adapter process did not start: "
        "frontier_action_adapter-"
    ) in audit["errors"]


def test_external_process_gate_is_adapter_specific(tmp_path: Path) -> None:
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "[INFO] [frontier_explorer-3]: process started with pid [30]\n"
        "[INFO] [frontier_action_adapter-4]: process started with pid [40]\n"
        "[INFO] [policy_node-5]: process has finished cleanly [pid 50]\n",
        encoding="utf-8",
    )

    assert system_sim_runner._launch_log_runtime_errors(
        launch_log, "frontier_mrtsp_dp_external"
    ) == []

    launch_log.write_text(
        launch_log.read_text(encoding="utf-8")
        + "[INFO] [frontier_explorer-3]: process has finished cleanly [pid 30]\n",
        encoding="utf-8",
    )
    errors = system_sim_runner._launch_log_runtime_errors(
        launch_log, "frontier_mrtsp_dp_external"
    )
    assert errors == [
        "required process exited before coordinated shutdown: "
        "frontier_explorer-3"
    ]


def test_artifact_audit_rejects_unknown_expected_adapter(tmp_path: Path) -> None:
    output = tmp_path / "artifacts"
    output.mkdir()
    subprocess.run(
        [sys.executable, "-c", _artifact_writer_program(), str(output), "exit"],
        check=True,
        stdout=subprocess.DEVNULL,
    )

    audit = validate_completed_artifacts(
        output, expected_runtime_adapter="unknown_adapter"
    )

    assert audit["valid"] is False
    assert any(
        "unsupported expected runtime_adapter" in error
        for error in audit["errors"]
    )


@pytest.mark.parametrize(
    "seed", [-1, 0, 0x80000000, 0xFFFFFFFF, True, 1.0]
)
def test_replicate_seed_must_fit_gazebo_positive_signed_32_bit_range(
    tmp_path: Path, seed: object,
) -> None:
    project = _fixture_project(tmp_path)
    root = project["root"]
    assert isinstance(root, Path)

    with pytest.raises(ScheduleError, match="positive signed 32-bit"):
        freeze_schedule(
            root=root,
            study_id="invalid_seed",
            output_dir=root / "experiments/system_sim/studies/invalid_seed",
            world_registry_path=project["registry"],
            shared_stack_path=project["shared"],
            method_paths=project["methods"],
            condition_path=project["condition"],
            world_ids=["dev_office_01"],
            replicate_seeds=[seed],
            randomization_seed=1,
            source_paths=project["source_paths"],
        )
