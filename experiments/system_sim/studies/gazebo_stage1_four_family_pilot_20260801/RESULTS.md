# Four-family system-simulation pilot results

This is development evidence, not a formal paper result. The frozen protocol
uses one furnished world from each of four site families, SSTG, the nominal
condition, replicate seed 211, and a ten-decision budget. Every run artifact is
kept below
`system_sim_outputs/runs/gazebo_stage1_four_family_pilot_20260801/`; aggregate
outputs are below
`system_sim_outputs/reports/gazebo_stage1_four_family_pilot_20260801/analysis/`.
Neither location is the legacy benchmark `outputs/` tree.

## Completion and recording gate

| World | Site family | Runner status | Wall time (s) | MCAP messages | MCAP duration (s) | MCAP bytes | Artifact audit |
| --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| `dev_office_01` | multi-room office | `terminal_completed` | 200.347 | 163,835 | 183.016 | 210,971,254 | valid |
| `dev_lab_01` | dense laboratory | `terminal_completed` | 233.979 | 193,403 | 216.036 | 236,776,714 | valid |
| `dev_warehouse_01` | warehouse aisles | `terminal_completed` | 220.359 | 181,720 | 202.984 | 217,107,389 | valid |
| `dev_corridor_01` | corridor with alcoves | `terminal_completed` | 242.882 | 201,412 | 224.944 | 254,315,347 | valid |

All four supervisors and launch processes returned zero. Each run recorded all
required core topics, ten navigation executions, an artifact-valid final
`policy_session_finished` evaluator snapshot, and a readable Zstd-compressed
MCAP. No launch log contains heap-corruption, double-linked-list, SIGABRT,
double-free or segmentation-fault evidence from the ROS-Gazebo bridges.

The Gazebo server required the ROS launch escalation from SIGINT to SIGTERM
after five seconds in all four runs. This is the same stable server-only
shutdown path accepted by the frozen runner; the remaining processes shut down
cleanly and the four independent artifact audits passed.

## Per-world metrics

| World | Information coverage | Topological / joint coverage | Target proxy | Travel (m) | Navigation successes | Collisions | Minimum clearance (m) | ATE RMSE (m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `dev_office_01` | 0.849045 | 0.541817 | 0.75 | 30.142 | 10 / 10 | 0 | 0.120607 | 0.068071 |
| `dev_lab_01` | 0.897059 | 0.571955 | 0.75 | 44.944 | 10 / 10 | 0 | 0.343669 | 0.043353 |
| `dev_warehouse_01` | 0.948371 | 0.580143 | 1.00 | 36.527 | 10 / 10 | 0 | 0.018165 | 0.109736 |
| `dev_corridor_01` | 0.962913 | 0.542011 | 0.25 | 48.301 | 10 / 10 | 0 | 0.106080 | 0.072032 |
| Mean | 0.914347 | 0.558982 | 0.6875 | 39.978 | 10 / 10 | 0 | 0.147130 | 0.073298 |

All runs were collision-free and runner-complete. They did not meet the frozen
dual coverage threshold because topological coverage remained below threshold;
runner completion, collision freedom and evaluator dual-threshold success are
therefore reported separately. The warehouse minimum-clearance value also
shows that a zero collision count must not be interpreted as generous safety
margin.

The aggregate analysis clusters by replicate seed. This pilot has only one
replicate seed, so its bootstrap intervals collapse to the corresponding point
estimates. These values validate the data path and expose scene-dependent
behavior, but they are not inferential estimates and must not be used as the
formal comparison table.

## Logged calibration findings

- The laboratory, warehouse and corridor logs each dropped one expired ATE
  sample after a 36 ms future-TF lookup. Thousands of other ATE samples remain
  in each final estimate, and no metric is missing, but the time-alignment path
  should be calibrated before the formal schedule is frozen.
- SLAM Toolbox requested 50 Ceres threads while the installed Ceres threading
  model supports at most 24. The warning occurred twice in the laboratory and
  56 times in the corridor. The pilot remains frozen as executed; the thread
  setting should be bounded in a separate calibration change before later
  studies.
- The four site families differ materially in path length, clearance, target
  proxy and localization error. This supports retaining family-stratified
  results rather than relying only on one pooled mean.

## Visual evidence

Each run has three registered and hashed media artifacts under its
`media/raw/` directory:

- `final_state.png`: final SLAM map and registered ground-truth path;
- `sensor_sanity.png`: final 360-beam LaserScan, with 360/360 valid returns in
  every scene;
- `task_camera_depth.mp4`: the full 32FC1 depth stream rendered as H.264 at
  640 x 560 and 5 frames/s with a fixed 0.05--5.0 m scale.

The four videos contain 916, 1,081, 1,015 and 1,125 frames for office,
laboratory, warehouse and corridor respectively. The media manifests label
them as development simulation evidence. Because the runs were headless, the
manifests honestly report the Gazebo-overview and RViz-navigation captures as
missing. Those two roles must be captured in a GUI-enabled showcase or formal
media pass rather than being inferred from offline renders.
