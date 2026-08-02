from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from scripts.register_system_sim_media import MediaError
from scripts.register_system_sim_rendered_media import register_rendered_media


def _run(tmp_path: Path, *, status: str = "terminal_completed", valid: bool = True) -> Path:
    run = tmp_path / "run"
    raw = run / "media" / "raw"
    raw.mkdir(parents=True)
    (run / "run_launch_manifest.yaml").write_text(
        yaml.safe_dump({
            "schema": "sstg_system_sim_run_launch/v1",
            "study_id": "development_media",
            "schedule_id": "run_001",
            "identity": {"world_id": "dev_office_01", "method": "sstg"},
            "execution": {
                "status": status,
                "artifact_audit": {"valid": valid},
            },
        }),
        encoding="utf-8",
    )
    for name in ("sensor_sanity.png", "final_state.png", "task_camera_depth.mp4"):
        (raw / name).write_bytes(f"rendered-{name}".encode())
    return run


def test_standard_rendered_bundle_is_indexed_and_hashed(tmp_path: Path):
    run = _run(tmp_path)
    output, checksum, manifest = register_rendered_media(
        run,
        evidence_tier="development",
        captured_at_utc="2026-08-02T03:00:00Z",
    )

    assert output.is_file()
    assert checksum.is_file()
    assert manifest["capture_count"] == 3
    assert manifest["complete_minimum_set"] is False
    assert manifest["missing_minimum_roles"] == [
        "gazebo_overview",
        "rviz_navigation",
    ]
    index = yaml.safe_load((run / "media" / "capture_index.yaml").read_text())
    assert {item["source"] for item in index["captures"]} == {
        "core_mcap_offline_render",
        "core_mcap_depth_render",
    }
    assert {item["captured_at_utc"] for item in index["captures"]} == {
        "2026-08-02T03:00:00Z"
    }
    decoded = json.loads(output.read_text(encoding="utf-8"))
    assert decoded["capture_index"] == "capture_index.yaml"


@pytest.mark.parametrize(
    ("status", "valid", "message"),
    [
        ("running", True, "terminal-completed"),
        ("terminal_completed", False, "valid artifact audit"),
    ],
)
def test_incomplete_or_invalid_run_is_rejected(
    tmp_path: Path,
    status: str,
    valid: bool,
    message: str,
):
    run = _run(tmp_path, status=status, valid=valid)
    with pytest.raises(MediaError, match=message):
        register_rendered_media(run, evidence_tier="development")


def test_missing_capture_and_metadata_overwrite_are_rejected(tmp_path: Path):
    run = _run(tmp_path)
    (run / "media" / "raw" / "task_camera_depth.mp4").unlink()
    with pytest.raises(MediaError, match="required rendered artifact"):
        register_rendered_media(run, evidence_tier="development")

    (run / "media" / "raw" / "task_camera_depth.mp4").write_bytes(b"video")
    (run / "media" / "capture_index.yaml").write_text("existing\n")
    with pytest.raises(MediaError, match="overwrite"):
        register_rendered_media(run, evidence_tier="development")


def test_capture_timestamp_must_be_utc(tmp_path: Path):
    run = _run(tmp_path)
    with pytest.raises(MediaError, match="identify UTC"):
        register_rendered_media(
            run,
            evidence_tier="development",
            captured_at_utc="2026-08-02T11:00:00+08:00",
        )
