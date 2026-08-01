from __future__ import annotations

from io import BytesIO
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

from scripts.render_system_sim_bag_media import (
    BagEvidence,
    DEVELOPMENT_LABEL,
    FINAL_STATE_NAME,
    RenderError,
    ScanSnapshot,
    SENSOR_SANITY_NAME,
    grid_from_message,
    load_run_context,
    media_output_paths,
    publish_media_pngs,
    render_final_state_png,
    render_sensor_sanity_png,
    transform_truth_points,
)


def _run(tmp_path: Path) -> Path:
    run = tmp_path / "run"
    bag = run / "bags" / "core"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("storage_identifier: mcap\n", encoding="utf-8")
    (bag / "core_0.mcap").write_bytes(b"test-mcap")
    (run / "run_launch_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "schema": "sstg_system_sim_run_launch/v1",
                "study_id": "dev_study",
                "schedule_id": "run_001",
                "launch": {
                    "arguments": {
                        "truth_registration_id": "test:inverse_spawn",
                        "truth_to_map_x_m": "2.0",
                        "truth_to_map_y_m": "3.0",
                        "truth_to_map_yaw_rad": str(np.pi / 2.0),
                    }
                },
                "identity": {
                    "world_id": "dev_world",
                    "method": "sstg",
                    "condition": "nominal",
                    "replicate_seed": "7",
                },
                "execution": {"status": "timeout"},
            }
        ),
        encoding="utf-8",
    )
    return run


def _quaternion(yaw: float = 0.0) -> SimpleNamespace:
    return SimpleNamespace(
        x=0.0,
        y=0.0,
        z=float(np.sin(yaw / 2.0)),
        w=float(np.cos(yaw / 2.0)),
    )


def _grid_message() -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            width=3,
            height=2,
            resolution=0.5,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=-0.5),
                orientation=_quaternion(),
            ),
        ),
        data=[-1, 0, 100, 0, 0, 100],
    )


def _evidence(include_scan: bool = False) -> BagEvidence:
    scan = None
    counts = {"/map": 2, "/evaluation/ground_truth_odom": 3}
    if include_scan:
        scan = ScanSnapshot(
            angle_min_rad=-np.pi / 2.0,
            angle_increment_rad=np.pi / 4.0,
            range_min_m=0.1,
            range_max_m=4.0,
            frame_id="base_scan",
            bag_timestamp_ns=30,
            ranges_m=np.asarray([1.0, 2.0, np.inf, 3.0, 1.5]),
        )
        counts["/scan"] = 5
    return BagEvidence(
        final_map=grid_from_message(_grid_message(), 20),
        truth_points=np.asarray([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]),
        truth_frame_id="world",
        final_scan=scan,
        topic_message_counts=counts,
    )


def test_context_and_truth_registration_are_manifest_anchored(tmp_path: Path):
    run = _run(tmp_path)

    context = load_run_context(run)
    transformed = transform_truth_points(
        np.asarray([[0.0, 0.0], [1.0, 0.0]]), context.registration
    )

    assert context.bag_dir == (run / "bags" / "core").resolve()
    assert context.registration.registration_id == "test:inverse_spawn"
    np.testing.assert_allclose(transformed, [[2.0, 3.0], [2.0, 4.0]])


def test_context_rejects_bag_escape_and_symlink(tmp_path: Path):
    run = _run(tmp_path)
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "metadata.yaml").write_text("storage_identifier: mcap\n")
    (outside / "outside.mcap").write_bytes(b"outside")

    with pytest.raises(RenderError, match="escapes run directory"):
        load_run_context(run, outside)

    linked = run / "linked_bag"
    linked.symlink_to(outside, target_is_directory=True)
    with pytest.raises(RenderError, match="symlink"):
        load_run_context(run, Path("linked_bag"))


def test_context_rejects_missing_truth_registration(tmp_path: Path):
    run = _run(tmp_path)
    manifest = yaml.safe_load(
        (run / "run_launch_manifest.yaml").read_text(encoding="utf-8")
    )
    del manifest["launch"]["arguments"]["truth_registration_id"]
    (run / "run_launch_manifest.yaml").write_text(
        yaml.safe_dump(manifest), encoding="utf-8"
    )

    with pytest.raises(RenderError, match="truth_registration_id"):
        load_run_context(run)


def test_grid_payload_and_geometry_are_strictly_validated():
    grid = grid_from_message(_grid_message(), 123)
    assert grid.data.shape == (2, 3)
    assert grid.frame_id == "map"
    assert grid.bag_timestamp_ns == 123

    malformed = _grid_message()
    malformed.data = [0]
    with pytest.raises(RenderError, match="payload size"):
        grid_from_message(malformed, 123)


def test_final_and_scan_renderers_embed_development_evidence_label(tmp_path: Path):
    context = load_run_context(_run(tmp_path))
    evidence = _evidence(include_scan=True)

    for encoded in (
        render_final_state_png(context, evidence),
        render_sensor_sanity_png(context, evidence),
    ):
        image = Image.open(BytesIO(encoded))
        assert image.width >= 1000
        assert image.height >= 1000
        assert image.info["Title"] == DEVELOPMENT_LABEL
        assert DEVELOPMENT_LABEL in image.info["Description"]


def test_media_publish_is_atomic_and_refuses_overwrite(tmp_path: Path):
    run = _run(tmp_path)
    encoded = render_final_state_png(load_run_context(run), _evidence())

    outputs = publish_media_pngs(
        run,
        {FINAL_STATE_NAME: encoded},
        include_sensor_sanity=False,
    )
    output = outputs[FINAL_STATE_NAME]
    original = output.read_bytes()
    assert output == run / "media" / "raw" / FINAL_STATE_NAME

    with pytest.raises(RenderError, match="refusing to overwrite"):
        publish_media_pngs(
            run,
            {FINAL_STATE_NAME: encoded},
            include_sensor_sanity=False,
        )
    assert output.read_bytes() == original


def test_requested_output_set_is_preflighted_before_any_write(tmp_path: Path):
    run = _run(tmp_path)
    raw = run / "media" / "raw"
    raw.mkdir(parents=True)
    existing_sensor = raw / SENSOR_SANITY_NAME
    existing_sensor.write_bytes(b"existing evidence")
    payload = b"\x89PNG\r\n\x1a\nrendered"

    with pytest.raises(RenderError, match="refusing to overwrite"):
        publish_media_pngs(
            run,
            {
                FINAL_STATE_NAME: payload,
                SENSOR_SANITY_NAME: payload,
            },
            include_sensor_sanity=True,
        )
    assert not (raw / FINAL_STATE_NAME).exists()
    assert existing_sensor.read_bytes() == b"existing evidence"


def test_media_output_rejects_symlinked_parent(tmp_path: Path):
    run = _run(tmp_path)
    outside = tmp_path / "outside_media"
    outside.mkdir()
    (run / "media").symlink_to(outside, target_is_directory=True)

    with pytest.raises(RenderError, match="symlink"):
        media_output_paths(run, include_sensor_sanity=False)
