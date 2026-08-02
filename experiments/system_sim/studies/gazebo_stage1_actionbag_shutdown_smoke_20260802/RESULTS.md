# Hidden-action recording and shutdown smoke

This is a development artifact-contract smoke, not a performance result. It
tests two issues found after the distance-budget calibration: rosbag2 omission
of declared ROS action hidden topics and unconsumed Jazzy executor exceptions
during adapter shutdown. Raw output is isolated under
`system_sim_outputs/runs/gazebo_stage1_actionbag_shutdown_smoke_20260802/`;
derived tables are under the matching `system_sim_outputs/reports/` directory.

## Gate result

The `dev_office_01`, seed-251 run completed in 38.205 s wall time with return
code 0 and an artifact-valid, settled evaluator snapshot. The deliberately
small two-decision cap bound the smoke after 2.779 m of trace odometry travel.
The 18,964,596-byte MCAP was independently read to EOF with 20,915 messages
over 21.016 s.

All four declared ROS action hidden topics are now present and nonempty:

| Topic | Messages |
| --- | ---: |
| `/navigate_to_pose/_action/feedback` | 1,094 |
| `/navigate_to_pose/_action/status` | 6 |
| `/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/feedback` | 1,094 |
| `/baseline/frontier_mrtsp_dp/navigate_to_pose/_action/status` | 8 |

The launch log contains neither the rosbag2 hidden-topic warning nor
`Destroyable`, `exception was never retrieved`, or the adapter's unexpected
shutdown-callback marker. The simulator itself still required the runner's
documented SIGINT-to-SIGTERM escalation; this is separate from adapter cleanup
and remained inside the audited shutdown window.

## Trace and diagnostic checks

The policy and evaluator each retained the same 12 parsed trace events: two
decisions, two confirmed upstream cancellations, two executions, two causal
topology nodes, one session start/finish pair, and the two action-budget events.
Both raw Nav2 terminals were `CANCELED`, both were normal policy transitions,
and technical failures were zero. The settled evaluator reported zero trace,
topology, map, ground-truth, contact, world-statistics, or ATE-TF rejections.

The short smoke ended at C-I 0.229659 and C-T 0.111632, with 2.794 m
ground-truth travel, zero collisions, 0.491 m minimum static footprint
clearance, and 0.00655 m ATE RMSE. These values are reported only to prove that
the common evaluator settled; the intentionally binding two-action cap makes
them unsuitable for method comparison.

## Visual evidence

The checksum-registered `media/raw/` bundle contains:

- `final_state.png`, 132,781 bytes, SHA-256
  `9a1bd0f32f2bf958408582731759cd58a29c4dcabb0af910759c57c05076d5cc`;
- `sensor_sanity.png`, 101,283 bytes, with 360/360 valid LaserScan returns,
  SHA-256
  `d161c62e14c0476758f3ede5e947bebc7e4fed2f7cf86739264bfa612ef4fb0e`;
- `task_camera_depth.mp4`, 106 H.264 frames at 640 x 560 and 5 fps over
  21.2 s, 90,559 bytes, SHA-256
  `d5d56d4ab311a1176e86fd92fbfc47b1175c9995a0af635ab34c9e4ef5d06534`.

Gazebo-overview and RViz-navigation roles remain absent because this was a
headless contract smoke. A separate GUI showcase must supply those views.

## Decision

The hidden-action recording and adapter shutdown-cleanup blockers are closed.
The external baseline remains development-only for the independent remaining
gates: Gazebo native-replacement branch coverage, matched sensor-parameter
freeze, and paired multi-scene/multi-seed validation.
