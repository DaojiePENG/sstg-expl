# External frontier distance-budget calibration

This is development simulation evidence, not a formal comparison result. It is
the first Gazebo--SLAM--Nav2 run of the pinned, unmodified
`frontier_exploration_ros2` v1.6.1 MRTSP-DP explorer using both a deliberately
nonbinding action cap and the prospectively frozen
`policy_transition_node_v1` topology contract. Raw artifacts are kept under
`system_sim_outputs/runs/gazebo_stage1_frontier_distance_smoke_20260802/` and
derived tables under
`system_sim_outputs/reports/gazebo_stage1_frontier_distance_smoke_20260802/`.
Neither path is the legacy benchmark `outputs/` tree.

## Artifact and budget gate

The single `dev_office_01` run (seed 239) finished with runner status
`terminal_completed`, return code 0, and no artifact-audit errors. The policy
terminated for `distance_budget` after 35.037 m of trace-recomputed odometry
travel, at 23 decisions. Thus the frozen 35 m distance budget bound the run,
while the 240 s duration and 100-decision fail-safe caps did not.

The audit additionally verified:

- one `session_started` and one `session_finished`, plus a settled evaluator
  snapshot;
- exact parsed equality of all 95 policy-trace records and all 95
  evaluator-observed records;
- zero policy-trace, topology-trace, map, ground-truth, contact, world-stats,
  or ATE-TF rejections;
- a Zstd-compressed MCAP read to EOF with 118,432 messages over 131.980 s and
  all required nonempty topics;
- 144,217,549 bytes of MCAP data with its SHA-256 identity recorded in the run
  manifest;
- 149.576 s wall time and a mean observed simulation real-time factor of
  0.9991.

## Settled development metrics

| Metric | Value |
| --- | ---: |
| Information coverage, C-I | 0.859488 |
| Topological / joint coverage, C-T | 0.526384 |
| Target recall geometry proxy | 0.75 (3 / 4) |
| Ground-truth travel | 35.378 m |
| Trace-reported odom travel | 35.037 m |
| Collision count | 0 |
| Mean static footprint clearance | 0.663 m |
| Minimum static footprint clearance | 0.237 m |
| Static footprint-clearance fifth percentile | 0.330 m |
| ATE RMSE | 0.0398 m |
| ATE maximum | 0.0980 m |

The dual 0.95 coverage threshold was not met. This single-world, single-seed
run calibrates measurement and budget semantics only; it cannot establish a
method ranking.

## Cancellation and topology audit

All 23 downstream Nav2 executions retained their raw `CANCELED` status. The
causal classification separates those statuses into 22 expected upstream
policy transitions and one adapter cancellation at the binding distance
budget. There were zero non-cancel failures and zero technical failures. Raw
navigation success therefore remains 0/23 without being misreported as 23
algorithmic failures.

Each of the 22 upstream cancellations froze one causal robot pose and was
confirmed by `downstream_cancel_accepted`. The adapter emitted 22 unique
`topological_node` events, all of which passed evaluator recomputation. Together
with the initial pose they produced 23 unique nodes and C-T 0.526384. All 22
candidate poses were at least 0.446 m from their nearest retained node, above
the frozen 0.25 m merge threshold; no event was merged. The final
distance-budget cancellation was correctly excluded from topology credit.

This validates the explicit upstream-cancel branch of
`policy_transition_node_v1` end to end. Native goal replacement is covered by
real ROS graph/runtime tests, including late-result pose freezing, but was not
triggered by this Gazebo trajectory and remains an explicit branch-coverage
item before the baseline is promoted.

## Evidence gaps found by audit

The recording contract listed four ROS action hidden topics, but rosbag2 did
not record them because the runner did not request hidden-topic inclusion:

- `/navigate_to_pose/_action/feedback`;
- `/navigate_to_pose/_action/status`;
- `/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/feedback`;
- `/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/status`.

They were not members of the frozen required-nonempty set, so their absence
does not invalidate this run or any metric above. The recording command and
declared contract must nevertheless be aligned before a formal batch.

Likewise, `launch_log_clean` means that no audited child crashed or exited
early; it does not mean that the log contains no recoverable runtime error.
This run contains expected behavior-tree halt messages during cancellation,
one controller progress failure and two planner failures followed by Nav2
recovery, plus two adapter future-cleanup exceptions during coordinated
shutdown. None became an action-level technical failure, but the shutdown
exceptions should be removed before paired batch execution.

## Visual evidence

The run has three hashed artifacts under `media/raw/`:

- `final_state.png`: final SLAM map and registered ground-truth path, 163,642
  bytes, SHA-256 `30b3b1122a25a7220173f899eafe9291843078c0b63446df44899e36a1d86b40`;
- `sensor_sanity.png`: final LaserScan with 360/360 valid returns, 96,811 bytes,
  SHA-256 `50c701b7394b57525a59468406a8e4370bbb3115ddb82a247c046d6fcdd1c129`;
- `task_camera_depth.mp4`: 660 H.264 frames at 640 x 560 and 5 fps, covering
  132.0 s with a fixed 0.05--5.0 m scale, 810,487 bytes, SHA-256
  `3750105e8ad4ac102482d05a07f11ec2f79f89fb73465d175be989972f686936`.

The media checksum verifies. The manifest intentionally reports Gazebo-overview
and RViz-navigation captures as missing because this pass was headless; those
roles require a separate GUI showcase capture and are not inferred from the
offline bag.

## Gate decision

Distance-led matched budgets, cancel-aware outcome reporting, and the observed
causal topology path are ready for paired development runs. The next gate is a
paired SSTG/external run followed by multiple room families and matched seeds.
Formal eligibility still requires sensor-parameter freeze, Gazebo coverage of
the native-replacement branch, hidden action-topic contract alignment, clean
adapter shutdown, and the planned multi-scene validation.
