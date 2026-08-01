#!/usr/bin/env python3
"""Validate and hash the visual evidence attached to one system-sim run."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping, Sequence

import yaml


SCHEMA = "sstg_system_sim_media_manifest/v1"
INDEX_SCHEMA = "sstg_system_sim_media_index/v1"
SUPPORTED_SUFFIXES = {
    ".jpeg", ".jpg", ".mkv", ".mp4", ".pdf", ".png", ".svg", ".webm"
}
VIDEO_SUFFIXES = {".mkv", ".mp4", ".webm"}
MINIMUM_ROLES = {
    "gazebo_overview",
    "rviz_navigation",
    "sensor_sanity",
    "final_state",
    "key_interval_video",
}


class MediaError(ValueError):
    """Raised when a visual-evidence bundle is ambiguous or unsafe."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mapping(path: Path, label: str) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MediaError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise MediaError(f"{label} must be a YAML mapping: {path}")
    return value


def _inside_regular_file(media_dir: Path, relative: Any) -> Path:
    text = str(relative).strip()
    if not text:
        raise MediaError("capture path must be non-empty")
    candidate = media_dir / text
    if candidate.is_symlink():
        raise MediaError(f"capture must not be a symlink: {text}")
    resolved = candidate.resolve()
    try:
        resolved.relative_to(media_dir.resolve())
    except ValueError as error:
        raise MediaError(f"capture escapes media directory: {text}") from error
    if not resolved.is_file() or resolved.stat().st_size <= 0:
        raise MediaError(f"capture is missing, empty, or not regular: {text}")
    if resolved.suffix.casefold() not in SUPPORTED_SUFFIXES:
        raise MediaError(f"unsupported capture suffix: {text}")
    return resolved


def build_media_manifest(
    run_dir: Path,
    *,
    evidence_tier: str,
    index_path: Path | None = None,
) -> dict[str, Any]:
    run_dir = run_dir.expanduser().resolve()
    if evidence_tier not in {"development", "formal"}:
        raise MediaError("evidence_tier must be development or formal")
    launch_path = run_dir / "run_launch_manifest.yaml"
    launch = _mapping(launch_path, "run launch manifest")
    if launch.get("schema") != "sstg_system_sim_run_launch/v1":
        raise MediaError("unsupported run launch manifest schema")
    schedule_id = str(launch.get("schedule_id", "")).strip()
    study_id = str(launch.get("study_id", "")).strip()
    if not schedule_id or not study_id:
        raise MediaError("run launch manifest lacks study_id or schedule_id")

    media_dir = run_dir / "media"
    index_path = index_path or media_dir / "capture_index.yaml"
    index_path = index_path.expanduser().resolve()
    try:
        index_relative = index_path.relative_to(media_dir.resolve())
    except ValueError as error:
        raise MediaError(
            f"media capture index must be inside the run media directory: {index_path}"
        ) from error
    if index_path.is_symlink():
        raise MediaError("media capture index must not be a symlink")
    index = _mapping(index_path, "media capture index")
    if index.get("schema") != INDEX_SCHEMA:
        raise MediaError("unsupported media capture index schema")
    captures = index.get("captures")
    if not isinstance(captures, list) or not captures:
        raise MediaError("media capture index must contain captures")

    records: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    roles: set[str] = set()
    for number, raw in enumerate(captures, start=1):
        if not isinstance(raw, Mapping):
            raise MediaError(f"capture {number} must be a mapping")
        relative = str(raw.get("path", "")).strip()
        role = str(raw.get("role", "")).strip()
        source = str(raw.get("source", "")).strip()
        captured_at = str(raw.get("captured_at_utc", "")).strip()
        if not role or not source or not captured_at:
            raise MediaError(
                f"capture {number} requires role, source and captured_at_utc"
            )
        if relative in seen_paths:
            raise MediaError(f"duplicate capture path: {relative}")
        path = _inside_regular_file(media_dir, relative)
        seen_paths.add(relative)
        roles.add(role)
        records.append({
            "path": relative,
            "role": role,
            "source": source,
            "captured_at_utc": captured_at,
            "description": str(raw.get("description", "")).strip(),
            "media_type": (
                "video" if path.suffix.casefold() in VIDEO_SUFFIXES else "image"
            ),
            "bytes": path.stat().st_size,
            "sha256": _sha256(path),
        })

    missing = sorted(MINIMUM_ROLES - roles)
    return {
        "schema": SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat().replace(
            "+00:00", "Z"
        ),
        "evidence_source": "system_simulation",
        "evidence_tier": evidence_tier,
        "study_id": study_id,
        "schedule_id": schedule_id,
        "identity": launch.get("identity", {}),
        "capture_index": index_relative.as_posix(),
        "capture_count": len(records),
        "minimum_role_set": sorted(MINIMUM_ROLES),
        "missing_minimum_roles": missing,
        "complete_minimum_set": not missing,
        "development_media_not_formal_evidence": evidence_tier == "development",
        "captures": sorted(records, key=lambda item: (item["role"], item["path"])),
    }


def write_media_manifest(run_dir: Path, manifest: Mapping[str, Any]) -> tuple[Path, Path]:
    media_dir = run_dir.expanduser().resolve() / "media"
    output = media_dir / "media_manifest.json"
    checksum = media_dir / "media_manifest.sha256"
    if output.exists() or checksum.exists():
        raise MediaError("refusing to overwrite an existing media manifest")
    encoded = (json.dumps(dict(manifest), indent=2, sort_keys=True) + "\n").encode(
        "utf-8"
    )
    media_dir.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".media_manifest.", suffix=".tmp", dir=media_dir
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, output)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    checksum.write_text(f"{_sha256(output)}  {output.name}\n", encoding="utf-8")
    return output, checksum


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--evidence-tier",
        choices=("development", "formal"),
        required=True,
    )
    parser.add_argument("--index", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        manifest = build_media_manifest(
            args.run_dir,
            evidence_tier=args.evidence_tier,
            index_path=args.index,
        )
        output, checksum = write_media_manifest(args.run_dir, manifest)
    except MediaError as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    print(json.dumps({
        "media_manifest": str(output),
        "checksum": str(checksum),
        "capture_count": manifest["capture_count"],
        "complete_minimum_set": manifest["complete_minimum_set"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
