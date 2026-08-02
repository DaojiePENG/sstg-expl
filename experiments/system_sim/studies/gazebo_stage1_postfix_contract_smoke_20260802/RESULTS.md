# Post-fix cancellation and action-evidence smoke

This is a development contract smoke, not a performance comparison.  It tests
the prospectively fixed SSTG distance-budget trace, adapter-specific ROS action
topic gates and continuous localization reporting on one matched 4 m block.
Raw runs are isolated under
`system_sim_outputs/runs/gazebo_stage1_postfix_contract_smoke_20260802/` and
derived tables under the matching `system_sim_outputs/reports/` tree; neither
uses the legacy benchmark `outputs/` directory.

## Frozen design and completion

The block used `dev_office_01`, `start_southwest`, nominal sensing, replicate
seed 271 and randomized method-order seed 1033.  The external adapter ran
first and SSTG second.  Both methods received 120 s duration, 4 m trace-distance,
100-decision fail-safe and 60 s per-goal caps.  The 4 m distance cap bound both
runs.

| Method | Status | Wall time (s) | MCAP messages | MCAP duration (s) | MCAP bytes | Audit |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| External MRTSP-DP frontier | `terminal_completed` | 42.080 | 24,140 | 23.828 | 21,248,084 | valid |
| SSTG | `terminal_completed` | 53.757 | 34,535 | 35.820 | 36,223,803 | valid |

Both launch and supervisor return codes were zero, the evaluator settled, and
rosbag2 was independently read to EOF.  Each run has zero artifact errors and
zero trace, topology, map, ground-truth, contact, world-statistics or ATE-TF
rejections.  Gazebo required the runner's documented SIGINT-to-SIGTERM shutdown
escalation in both runs; no policy or evaluator process failed.

## Action-evidence and causal cancellation gates

The external runtime contract required the shared and proxy action
feedback/status topics.  Their nonzero message counts were 1,447, 6, 1,447 and
8 respectively.  Its two raw Nav2 terminals were `CANCELED`: one confirmed
upstream policy transition and one final adapter distance-budget cancellation.
The evaluator reports one policy transition, one adapter cancel, zero
non-cancel failures and zero technical failures.

The SSTG runtime contract required the shared action topics, which contain
2,539 feedback and 7 status messages.  Its trace contains two successful Nav2
executions followed by the binding-distance cancellation:

```text
reason=nav2_status_5:distance_budget
nav2_status=5
cancel_origin=adapter_session_termination
termination_reason=distance_budget
```

The final settled evaluator therefore reports three goals, two successes, one
cancellation, one adapter cancellation, zero non-cancel failures and zero
technical failures.  This closes the old paired-office trace-classification
mismatch without modifying that immutable run.

## Descriptive metrics

| Metric | External | SSTG |
| --- | ---: | ---: |
| Trace distance (m) | 4.018065 | 4.067738 |
| Ground-truth travel (m) | 4.007951 | 4.134275 |
| Information coverage, C-I | 0.275271 | 0.271013 |
| Topological / joint coverage, C-T | 0.068426 | 0.106702 |
| Target-recall geometry proxy | 0 | 0 |
| Collisions | 0 | 0 |
| Minimum footprint clearance (m) | 0.491037 | 0.497755 |
| ATE mean (m) | 0.006706 | 0.006112 |
| ATE RMSE (m) | 0.009717 | 0.007279 |
| ATE maximum (m) | 0.024623 | 0.013173 |

The updated analyzer includes ATE mean, RMSE and maximum in the method
aggregate while retaining all scheduled rows.  The frozen localization
contract has no threshold, does not exclude either run and does not use ATE to
adjust coverage.  These intentionally short trajectories calibrate contracts
only and support no method ranking.

## Visual evidence

Both media manifests and their SHA-256 sidecars verify.  The visually checked
offline renders show coherent short paths and 360/360 valid final LiDAR
returns.

| Method | Artifact | Size / duration | SHA-256 |
| --- | --- | --- | --- |
| External | `final_state.png` | 133,223 bytes | `27612db182e0b6e7680440507ffc925ec59f54a88330d3bd64281d84bc720ff4` |
| External | `sensor_sanity.png` | 101,490 bytes | `dd49641e6e1c2a086a2621b9616832c66f601527376f08cb8c313237dd945b6d` |
| External | `task_camera_depth.mp4` | 120 frames, 24.0 s, 113,250 bytes | `9aa2a433be5138098f9fcdb53d85230477ec36c53c81050f3fe825003a654f9c` |
| SSTG | `final_state.png` | 128,136 bytes | `21b3d2ae8bf6c307b5c579743964353d09c1a0cdd5a5e2ab00398fed8a1ec9a2` |
| SSTG | `sensor_sanity.png` | 98,376 bytes | `e4a3dc1ec9932887b89adaafdbac2e075b6734e3cb94804022f783ced9b76127` |
| SSTG | `task_camera_depth.mp4` | 180 frames, 36.0 s, 198,493 bytes | `13198a82df8f8b50a0fb43ffe16ecdf95f59503dbaa87da79f30288b9b33daca` |

The run was headless, so the media manifests correctly retain Gazebo-overview
and RViz-navigation as missing roles.  Those roles remain assigned to a
separate GUI showcase rather than being inferred from offline MCAP renders.

## Gate decision

The SSTG causal distance-budget trace, external and SSTG adapter-specific
action-topic gates, analyzer localization aggregates and media workflow now
pass together in Gazebo.  The next development gate is the already planned
four-family, multi-seed matched batch; formal eligibility remains closed.
