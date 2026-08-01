from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from scripts.register_system_sim_media import (
    INDEX_SCHEMA,
    MediaError,
    build_media_manifest,
    write_media_manifest,
)


def _run(tmp_path: Path) -> Path:
    run = tmp_path / "run"
    (run / "media" / "raw").mkdir(parents=True)
    (run / "run_launch_manifest.yaml").write_text(
        yaml.safe_dump({
            "schema": "sstg_system_sim_run_launch/v1",
            "study_id": "dev_smoke",
            "schedule_id": "run_001",
            "identity": {"world_id": "dev_office_01", "method": "sstg"},
        }),
        encoding="utf-8",
    )
    return run


def _capture(role: str, path: str) -> dict[str, str]:
    return {
        "path": path,
        "role": role,
        "source": "test_capture",
        "captured_at_utc": "2026-08-02T00:00:00Z",
        "description": role,
    }


def test_complete_media_bundle_is_hashed_and_fail_closed(tmp_path: Path):
    run = _run(tmp_path)
    captures = [
        _capture("gazebo_overview", "raw/gazebo.png"),
        _capture("rviz_navigation", "raw/rviz.png"),
        _capture("sensor_sanity", "raw/sensor.png"),
        _capture("final_state", "raw/final.png"),
        _capture("key_interval_video", "raw/interval.mp4"),
    ]
    for number, item in enumerate(captures, start=1):
        (run / "media" / item["path"]).write_bytes(f"capture-{number}".encode())
    (run / "media" / "capture_index.yaml").write_text(
        yaml.safe_dump({"schema": INDEX_SCHEMA, "captures": captures}),
        encoding="utf-8",
    )

    manifest = build_media_manifest(run, evidence_tier="development")
    assert manifest["complete_minimum_set"] is True
    assert manifest["missing_minimum_roles"] == []
    assert manifest["capture_count"] == 5
    assert manifest["development_media_not_formal_evidence"] is True
    assert {item["media_type"] for item in manifest["captures"]} == {
        "image", "video"
    }

    output, checksum = write_media_manifest(run, manifest)
    decoded = json.loads(output.read_text(encoding="utf-8"))
    expected = hashlib.sha256(output.read_bytes()).hexdigest()
    assert decoded["schedule_id"] == "run_001"
    assert checksum.read_text(encoding="utf-8") == (
        f"{expected}  media_manifest.json\n"
    )
    with pytest.raises(MediaError, match="overwrite"):
        write_media_manifest(run, manifest)


def test_partial_failed_run_media_remains_explicit(tmp_path: Path):
    run = _run(tmp_path)
    image = run / "media" / "raw" / "failure.png"
    image.write_bytes(b"failure state")
    (run / "media" / "capture_index.yaml").write_text(
        yaml.safe_dump({
            "schema": INDEX_SCHEMA,
            "captures": [_capture("final_state", "raw/failure.png")],
        }),
        encoding="utf-8",
    )
    manifest = build_media_manifest(run, evidence_tier="development")
    assert manifest["complete_minimum_set"] is False
    assert "key_interval_video" in manifest["missing_minimum_roles"]


def test_media_paths_cannot_escape_or_use_symlinks(tmp_path: Path):
    run = _run(tmp_path)
    outside = tmp_path / "outside.png"
    outside.write_bytes(b"outside")
    index = run / "media" / "capture_index.yaml"
    index.write_text(
        yaml.safe_dump({
            "schema": INDEX_SCHEMA,
            "captures": [_capture("final_state", "../../outside.png")],
        }),
        encoding="utf-8",
    )
    with pytest.raises(MediaError, match="escapes"):
        build_media_manifest(run, evidence_tier="development")

    link = run / "media" / "raw" / "link.png"
    link.symlink_to(outside)
    index.write_text(
        yaml.safe_dump({
            "schema": INDEX_SCHEMA,
            "captures": [_capture("final_state", "raw/link.png")],
        }),
        encoding="utf-8",
    )
    with pytest.raises(MediaError, match="symlink"):
        build_media_manifest(run, evidence_tier="development")


def test_capture_index_must_be_inside_media_directory(tmp_path: Path):
    run = _run(tmp_path)
    outside_index = tmp_path / "capture_index.yaml"
    outside_index.write_text(
        yaml.safe_dump({
            "schema": INDEX_SCHEMA,
            "captures": [_capture("final_state", "raw/failure.png")],
        }),
        encoding="utf-8",
    )
    with pytest.raises(MediaError, match="inside the run media directory"):
        build_media_manifest(
            run,
            evidence_tier="development",
            index_path=outside_index,
        )
