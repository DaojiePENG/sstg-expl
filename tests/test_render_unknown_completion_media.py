from pathlib import Path

import numpy as np
import pytest

from scripts.render_system_sim_bag_media import GridSnapshot, RenderError
from scripts.render_unknown_completion_media import (
    CoverageSample,
    coverage_at,
    extract_endpoints,
    read_jsonl_events,
    remaining_truth_masks,
)
from sstg_system_eval.metrics import TruthGrid


def test_remaining_truth_masks_distinguish_unknown_and_false_occupied():
    truth = TruthGrid(
        free=np.ones((2, 3), dtype=bool),
        occupied=np.zeros((2, 3), dtype=bool),
        resolution=1.0,
        origin=(0.0, 0.0),
    )
    belief = GridSnapshot(
        width=3,
        height=2,
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        origin_yaw_rad=0.0,
        frame_id="map",
        bag_timestamp_ns=1,
        data=np.asarray([[-1, 0, 100], [0, -1, 0]]),
    )

    unknown, false_occupied = remaining_truth_masks(truth, belief)

    assert np.array_equal(unknown, [[True, False, False], [False, True, False]])
    assert np.array_equal(false_occupied, [[False, False, True], [False, False, False]])


def test_trace_parser_and_endpoint_numbering_are_strict(tmp_path: Path):
    path = tmp_path / "policy_trace.jsonl"
    path.write_text(
        "\n".join([
            '{"event":"session_started","ros_time_ns":1,"payload":{"nodes":[{"position":[0,0],"orientation":0}]}}',
            '{"event":"execution","ros_time_ns":3,"payload":{"decision_id":1,"translation_m":1.25,"commanded_pose":[1,0,0],"reached_pose":[0.9,0.1,5],"succeeded":true,"reason":"ok","topological_node_created":true}}',
        ]) + "\n",
        encoding="utf-8",
    )
    endpoints = extract_endpoints(read_jsonl_events(path, label="test trace"))

    assert [item.decision_id for item in endpoints] == [0, 1]
    assert endpoints[-1].cumulative_distance_m == pytest.approx(1.25)
    assert endpoints[-1].reached_pose == (0.9, 0.1, 5.0)


def test_coverage_sampling_uses_first_snapshot_at_or_after_endpoint():
    samples = [
        CoverageSample(10, 0.0, 0.2, 0.1),
        CoverageSample(20, 1.0, 0.4, 0.3),
    ]

    assert coverage_at(samples, 11) == samples[1]
    assert coverage_at(samples, 99) == samples[1]


def test_jsonl_parser_rejects_nonmonotonic_time(tmp_path: Path):
    path = tmp_path / "bad.jsonl"
    path.write_text(
        '{"event":"a","ros_time_ns":2,"payload":{}}\n'
        '{"event":"b","ros_time_ns":1,"payload":{}}\n',
        encoding="utf-8",
    )
    with pytest.raises(RenderError, match="not ordered"):
        read_jsonl_events(path, label="bad trace")
