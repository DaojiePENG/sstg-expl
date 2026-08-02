#!/usr/bin/env python3
"""Static and host-runtime preflight for the ROS 2 system simulation."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from xml.etree import ElementTree

import yaml

try:
    from scripts.generate_system_sim_schedule import (
        ScheduleError,
        validate_ros_gz_bridge_contract,
        validate_ros_middleware_contract,
    )
    from scripts.run_system_sim_schedule import (
        RunnerError,
        verify_ros_gz_bridge_runtime,
        verify_ros_middleware_runtime,
    )
except ModuleNotFoundError as error:
    if error.name != "scripts":
        raise
    from generate_system_sim_schedule import (  # type: ignore[no-redef]
        ScheduleError,
        validate_ros_gz_bridge_contract,
        validate_ros_middleware_contract,
    )
    from run_system_sim_schedule import (  # type: ignore[no-redef]
        RunnerError,
        verify_ros_gz_bridge_runtime,
        verify_ros_middleware_runtime,
    )


GAZEBO_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws/src/sstg_gazebo"
)
if str(GAZEBO_SOURCE) not in sys.path:
    sys.path.insert(0, str(GAZEBO_SOURCE))

from sstg_gazebo.instrumented_tb3 import (  # noqa: E402
    InstrumentationError,
    prepare_instrumented_tb3_sdf,
)


ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGES = (
    "nav2_bringup",
    "nav2_minimal_tb3_sim",
    "ros_gz_bridge",
    "ros_gz_interfaces",
    "ros_gz_sim",
    "slam_toolbox",
    "sstg_explorer_core",
    "sstg_gazebo",
    "sstg_nav_bringup",
    "sstg_policy_ros",
    "sstg_system_eval",
)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def registered_worlds():
    registry = yaml.safe_load((
        ROOT / "experiments/system_sim/registries/worlds.yaml"
    ).read_text(encoding="utf-8"))
    return registry["worlds"]


def command_output(command):
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"ok": False, "detail": str(error)}
    detail = (completed.stdout or completed.stderr).strip()
    return {"ok": completed.returncode == 0, "detail": detail}


def static_checks():
    errors = []
    parsed = {"yaml": 0, "xml": 0, "python": 0}
    vendor_root = ROOT / "ros2_ws/src/third_party"
    roots = [
        ROOT / "experiments" / "system_sim",
        ROOT / "ros2_ws" / "src",
    ]
    for base in roots:
        if not base.exists():
            errors.append(f"missing directory: {base.relative_to(ROOT)}")
            continue
        for path in base.rglob("*.yaml"):
            if path.is_relative_to(vendor_root):
                continue
            try:
                yaml.safe_load(path.read_text(encoding="utf-8"))
                parsed["yaml"] += 1
            except Exception as error:
                errors.append(f"YAML {path.relative_to(ROOT)}: {error}")
        for pattern in ("*.xml", "*.sdf"):
            for path in base.rglob(pattern):
                if path.is_relative_to(vendor_root):
                    continue
                try:
                    ElementTree.parse(path)
                    parsed["xml"] += 1
                except Exception as error:
                    errors.append(f"XML {path.relative_to(ROOT)}: {error}")
        for path in base.rglob("*.py"):
            if path.is_relative_to(vendor_root):
                continue
            try:
                compile(path.read_text(encoding="utf-8"), str(path), "exec")
                parsed["python"] += 1
            except Exception as error:
                errors.append(f"Python {path.relative_to(ROOT)}: {error}")

    topic_access = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/topic_access.yaml"
    ).read_text(encoding="utf-8"))
    policy_subscriptions = topic_access["policy"]["subscriptions"]
    if any(name.startswith("/evaluation/") for name in policy_subscriptions):
        errors.append("policy subscription allowlist includes evaluator truth")

    world_entries = registered_worlds()
    if len({entry["world_id"] for entry in world_entries}) != len(world_entries):
        errors.append("world registry contains duplicate IDs")
    if len({entry["site_family"] for entry in world_entries}) != len(world_entries):
        errors.append("development world registry repeats a site family")
    for entry in world_entries:
        bundle = ROOT / entry["bundle"]
        world_path = bundle / "world.sdf"
        try:
            world = ElementTree.parse(world_path).getroot().find("world")
            metadata = yaml.safe_load((bundle / "metadata.yaml").read_text())
            manifest = yaml.safe_load((
                bundle / "evaluation/truth_map_manifest.yaml"
            ).read_text())
        except Exception as error:
            errors.append(f"world bundle {entry['world_id']}: {error}")
            continue
        if world is None or world.get("name") != entry["world_id"]:
            errors.append(f"world SDF ID mismatch: {entry['world_id']}")
        if metadata.get("world_id") != entry["world_id"]:
            errors.append(f"world metadata ID mismatch: {entry['world_id']}")
        if metadata.get("site_family") != entry["site_family"]:
            errors.append(f"world site-family mismatch: {entry['world_id']}")
        if world.find(".//plugin[@name='gz::sim::systems::Sensors']") is None:
            errors.append(f"world has no Sensors system: {entry['world_id']}")
        for relative, expected in manifest.get("sha256", {}).items():
            artifact = (
                bundle / "evaluation" / relative
                if relative.startswith("truth_map")
                else bundle / relative
            )
            if not artifact.is_file() or sha256(artifact) != expected:
                errors.append(
                    f"truth artifact hash mismatch: {entry['world_id']}:{relative}"
                )
    stack = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    try:
        validate_ros_gz_bridge_contract(stack.get("ros_gz_bridge"))
    except ScheduleError as error:
        errors.append(str(error))
    try:
        validate_ros_middleware_contract(stack.get("ros_middleware"))
    except ScheduleError as error:
        errors.append(str(error))
    robot = stack["robot"]
    if robot.get("upstream_package") != "nav2_minimal_tb3_sim":
        errors.append("default robot is not the released Nav2 TurtleBot3")
    launch_source = (
        ROOT / "ros2_ws/src/sstg_gazebo/launch/sim.launch.py"
    ).read_text(encoding="utf-8")
    if '"launch", "spawn_tb3.launch.py"' not in launch_source:
        errors.append("simulation launch does not reuse upstream TB3 spawn")
    if "prepare_instrumented_tb3_sdf(" not in launch_source:
        errors.append("simulation launch does not instrument upstream TB3")
    if "ExecuteProcess" in launch_source:
        errors.append("simulation launch still has asynchronous xacro execution")
    overlay_path = (
        ROOT / "ros2_ws/src/sstg_gazebo/models/"
        "tb3_evaluation_overlay/model.sdf"
    )
    if overlay_path.exists() or "DetachableJoint" in launch_source:
        errors.append("detachable TB3 evaluation overlay still exists")

    nav2 = yaml.safe_load((
        ROOT / "ros2_ws/src/sstg_nav_bringup/config/nav2_params.yaml"
    ).read_text(encoding="utf-8"))
    controller = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    if (
        controller["max_linear_vel"] > robot["max_linear_velocity_mps"]
        or controller["max_angular_vel"] > robot["max_angular_velocity_radps"]
    ):
        errors.append("Nav2 controller exceeds robot velocity limits")
    return {"ok": not errors, "parsed": parsed, "errors": errors}


def runtime_checks():
    checks = {
        "python_executable": {
            "ok": Path(sys.executable).resolve() == Path("/usr/bin/python3").resolve(),
            "detail": sys.executable,
        },
        "ros2_executable": {
            "ok": shutil.which("ros2") is not None,
            "detail": shutil.which("ros2") or "not found",
        },
        "gz_executable": {
            "ok": shutil.which("gz") is not None,
            "detail": shutil.which("gz") or "not found",
        },
        "xacro_executable": {
            "ok": shutil.which("xacro") is not None,
            "detail": shutil.which("xacro") or "not found",
        },
    }
    for package in ROS_PACKAGES:
        checks[f"ros_package:{package}"] = command_output(
            ["ros2", "pkg", "prefix", package]
        ) if checks["ros2_executable"]["ok"] else {
            "ok": False,
            "detail": "ros2 executable unavailable",
        }
    stack = yaml.safe_load((
        ROOT / "experiments/system_sim/configs/shared_stack.yaml"
    ).read_text(encoding="utf-8"))
    try:
        middleware_contract = validate_ros_middleware_contract(
            stack.get("ros_middleware")
        )
        middleware_attestation = verify_ros_middleware_runtime(
            ROOT, middleware_contract
        )
        checks["ros_middleware_runtime"] = {
            "ok": True,
            "detail": middleware_attestation,
        }
    except (ScheduleError, RunnerError) as error:
        checks["ros_middleware_runtime"] = {
            "ok": False,
            "detail": str(error),
        }
    try:
        bridge_contract = validate_ros_gz_bridge_contract(
            stack.get("ros_gz_bridge")
        )
        bridge_attestation = verify_ros_gz_bridge_runtime(ROOT, bridge_contract)
        checks["ros_gz_bridge_runtime"] = {
            "ok": True,
            "detail": bridge_attestation,
        }
    except (ScheduleError, RunnerError) as error:
        checks["ros_gz_bridge_runtime"] = {
            "ok": False,
            "detail": str(error),
        }
    if checks["gz_executable"]["ok"]:
        sdf_files = {
            f"world:{entry['world_id']}": ROOT / entry["bundle"] / "world.sdf"
            for entry in registered_worlds()
        }
        for label, path in sdf_files.items():
            checks[f"gz_sdf:{label}"] = command_output(
                ["gz", "sdf", "-k", str(path)]
            )
    if (
        checks["xacro_executable"]["ok"]
        and checks.get("ros_package:nav2_minimal_tb3_sim", {}).get("ok")
    ):
        prefix = Path(
            checks["ros_package:nav2_minimal_tb3_sim"]["detail"]
        )
        upstream_xacro = (
            prefix / "share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro"
        )
        try:
            with tempfile.TemporaryDirectory(
                prefix="sstg_preflight_tb3_"
            ) as directory:
                prepared = prepare_instrumented_tb3_sdf(
                    upstream_xacro,
                    output_directory=directory,
                    xacro_executable=checks["xacro_executable"]["detail"],
                )
                report = prepared.instrumentation
                checks["instrumented_tb3"] = {
                    "ok": (
                        report.upstream_structure_preserved
                        and report.contact_sensor_count == report.collision_count
                    ),
                    "detail": {
                        "contacts": report.contact_sensor_count,
                        "collisions": report.collision_count,
                        "imu_joint_backported": report.imu_joint_backported,
                        "source_xacro_sha256": (
                            prepared.source_xacro_sha256
                        ),
                        "upstream_sha256": report.upstream_sha256,
                        "derivative_sha256": report.derivative_sha256,
                    },
                }
                if checks["gz_executable"]["ok"]:
                    checks["gz_sdf:instrumented_tb3"] = command_output(
                        ["gz", "sdf", "-k", str(prepared.output_path)]
                    )
        except (InstrumentationError, OSError) as error:
            checks["instrumented_tb3"] = {
                "ok": False,
                "detail": str(error),
            }
    try:
        import skimage
        checks["python3:skimage"] = {
            "ok": True,
            "detail": getattr(skimage, "__version__", "installed"),
        }
    except ImportError as error:
        checks["python3:skimage"] = {"ok": False, "detail": str(error)}
    return {
        "ok": all(item["ok"] for item in checks.values()),
        "checks": checks,
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--require-runtime",
        action="store_true",
        help="fail if Gazebo/Nav2/SLAM runtime dependencies are missing",
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    report = {
        "schema": "sstg_system_sim_preflight/v1",
        "static": static_checks(),
        "runtime": runtime_checks(),
    }
    report["status"] = (
        "ready" if report["static"]["ok"] and report["runtime"]["ok"]
        else "runtime_pending" if report["static"]["ok"]
        else "static_failure"
    )
    encoded = json.dumps(report, indent=2, sort_keys=True)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    if not report["static"]["ok"]:
        return 1
    if args.require_runtime and not report["runtime"]["ok"]:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
