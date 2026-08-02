#!/usr/bin/env python3
"""Index and register the standard offline-rendered system-sim media bundle.

This helper deliberately registers only artifacts produced from the immutable
core MCAP.  Gazebo and RViz screen captures remain absent and visible in the
resulting development manifest rather than being inferred from offline plots.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Sequence

import yaml

try:
    from scripts.register_system_sim_media import (
        INDEX_SCHEMA,
        MediaError,
        build_media_manifest,
        write_media_manifest,
    )
except ModuleNotFoundError as error:
    if error.name != "scripts":
        raise
    from register_system_sim_media import (  # type: ignore[no-redef]
        INDEX_SCHEMA,
        MediaError,
        build_media_manifest,
        write_media_manifest,
    )


RUN_SCHEMA = "sstg_system_sim_run_launch/v1"
STANDARD_CAPTURES = (
    {
        "path": "raw/sensor_sanity.png",
        "role": "sensor_sanity",
        "source": "core_mcap_offline_render",
        "description": "Final 360-beam LaserScan rendered from the core MCAP.",
    },
    {
        "path": "raw/final_state.png",
        "role": "final_state",
        "source": "core_mcap_offline_render",
        "description": (
            "Final SLAM map and registered ground-truth path rendered from "
            "the core MCAP."
        ),
    },
    {
        "path": "raw/task_camera_depth.mp4",
        "role": "key_interval_video",
        "source": "core_mcap_depth_render",
        "description": (
            "Full task-camera depth stream rendered with the frozen metric scale."
        ),
    },
)


def _mapping(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink():
        raise MediaError(f"{label} must not be a symlink: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MediaError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise MediaError(f"{label} must be a YAML mapping: {path}")
    return value


def _captured_at_utc(value: str | None) -> str:
    if value is None:
        return datetime.now(timezone.utc).isoformat(timespec="seconds").replace(
            "+00:00", "Z"
        )
    text = value.strip()
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as error:
        raise MediaError("captured_at_utc must be an ISO-8601 timestamp") from error
    if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
        raise MediaError("captured_at_utc must identify UTC")
    return parsed.astimezone(timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def register_rendered_media(
    run_dir: Path | str,
    *,
    evidence_tier: str,
    captured_at_utc: str | None = None,
) -> tuple[Path, Path, dict[str, Any]]:
    """Register the three standard offline derivatives without overwriting."""
    supplied = Path(run_dir).expanduser()
    if supplied.is_symlink():
        raise MediaError(f"run directory must not be a symlink: {supplied}")
    run = supplied.resolve()
    if not run.is_dir():
        raise MediaError(f"run directory does not exist: {run}")

    launch_path = run / "run_launch_manifest.yaml"
    launch = _mapping(launch_path, "run launch manifest")
    if launch.get("schema") != RUN_SCHEMA:
        raise MediaError("unsupported run launch manifest schema")
    execution = launch.get("execution")
    if not isinstance(execution, dict):
        raise MediaError("run launch manifest lacks execution metadata")
    if execution.get("status") != "terminal_completed":
        raise MediaError("offline media requires a terminal-completed run")
    audit = execution.get("artifact_audit")
    if not isinstance(audit, dict) or audit.get("valid") is not True:
        raise MediaError("offline media requires a valid artifact audit")

    media_dir = run / "media"
    index_path = media_dir / "capture_index.yaml"
    manifest_path = media_dir / "media_manifest.json"
    checksum_path = media_dir / "media_manifest.sha256"
    for output in (index_path, manifest_path, checksum_path):
        if os.path.lexists(output):
            raise MediaError(f"refusing to overwrite existing media metadata: {output}")

    stamp = _captured_at_utc(captured_at_utc)
    captures = []
    for template in STANDARD_CAPTURES:
        relative = str(template["path"])
        artifact = media_dir / relative
        if artifact.is_symlink() or not artifact.is_file() or artifact.stat().st_size <= 0:
            raise MediaError(f"required rendered artifact is missing or unsafe: {relative}")
        captures.append({**template, "captured_at_utc": stamp})
    index = {"schema": INDEX_SCHEMA, "captures": captures}

    media_dir.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".capture_index.", suffix=".yaml", dir=media_dir
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(index, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        manifest = build_media_manifest(
            run,
            evidence_tier=evidence_tier,
            index_path=temporary,
        )
        manifest["capture_index"] = index_path.relative_to(media_dir).as_posix()
        os.replace(temporary, index_path)
        output, checksum = write_media_manifest(run, manifest)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return output, checksum, manifest


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--evidence-tier",
        choices=("development", "formal"),
        required=True,
    )
    parser.add_argument("--captured-at-utc")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        output, checksum, manifest = register_rendered_media(
            args.run_dir,
            evidence_tier=args.evidence_tier,
            captured_at_utc=args.captured_at_utc,
        )
    except MediaError as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    print(json.dumps({
        "media_manifest": str(output),
        "checksum": str(checksum),
        "capture_count": manifest["capture_count"],
        "complete_minimum_set": manifest["complete_minimum_set"],
        "missing_minimum_roles": manifest["missing_minimum_roles"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
