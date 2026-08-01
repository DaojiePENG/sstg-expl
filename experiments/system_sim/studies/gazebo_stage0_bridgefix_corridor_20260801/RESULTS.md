# Bridge-fixed corridor regression results

This is development evidence, not a formal paper result.  The frozen protocol
uses `dev_corridor_01`, `start_west`, SSTG, the nominal condition, replicate
seeds 113 and 127, and a two-decision budget.  Every run artifact is kept below
`system_sim_outputs/runs/gazebo_stage0_bridgefix_corridor_20260801/`; aggregate
outputs are below
`system_sim_outputs/reports/gazebo_stage0_bridgefix_corridor_20260801/analysis/`.

## Completion and shutdown gate

| Seed | Runner status | Wall time (s) | MCAP messages | MCAP duration (s) | Artifact audit |
| ---: | --- | ---: | ---: | ---: | --- |
| 113 | `terminal_completed` | 50.277 | 29,155 | 32.680 | valid |
| 127 | `terminal_completed` | 41.390 | 21,354 | 23.988 | valid |

Both runner processes returned zero.  In both shutdowns, all three
`parameter_bridge` processes, including the separate CameraInfo bridge, exited
cleanly.  Neither log contains heap-corruption, double-linked-list, SIGABRT or
segmentation-fault evidence.  The Gazebo server required the ROS launch
escalation from SIGINT to SIGTERM after five seconds in both runs; the frozen
runner classifies this stable server-only shutdown path as clean, and both core
bags passed independent reader verification.

The two manifests record identical runtime binaries:

- `parameter_bridge`: `2d2ece6fc345263f78dc1dc00c3710666039c9c07ef9cc62de2cf29a8ea47561`
- `libros_gz_bridge.so`: `29dbebe4c8633408178b29242283039f0ed201cf63a43759b0a9d69606a3b2c4`
- `librmw_fastrtps_cpp.so`: `046375a1ef195094abb57c832b275c385261052cfed3ee044cce70e25a42cef3`
- Fast DDS: `8d39de86a55a92e1be92640a22e6322f099227930d4fc98bd821b6effd7a3eaa`
- Fast CDR: `0eeb1f3d1859db07e7551be9df814053a8a4805c9feb9cb990363e43ac45cd69`

## Short-budget metrics

| Seed | Information coverage | Topological / joint coverage | Travel (m) | Collisions | Minimum clearance (m) | ATE RMSE (m) |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 113 | 0.604660 | 0.146303 | 4.975621 | 0 | 0.714804 | 0.012549 |
| 127 | 0.472174 | 0.124871 | 3.256091 | 0 | 0.869905 | 0.009812 |
| Mean | 0.538417 | 0.135587 | 4.115856 | 0 | 0.792355 | 0.011181 |

The dual-threshold result is intentionally false in both runs because this
shutdown regression allows only two policy decisions; it is not an exploration
performance experiment.  Collision-free execution, nonzero mapped coverage and
centimetre-scale ATE confirm that the quasi-real sensor/evaluation pipeline was
active rather than merely exercising process startup.

## Visual evidence

Each run contains three registered, hashed media artifacts under `media/raw/`:

- `final_state.png`: final SLAM map plus registered ground-truth path;
- `sensor_sanity.png`: the final 360-beam LaserScan;
- `task_camera_depth.mp4`: the full 32FC1 task-camera stream rendered as H.264
  with a fixed 0.05--5.0 m scale.

The media are explicitly labelled development simulation evidence.  Because
the run was headless, the media manifest honestly reports the Gazebo-overview
and RViz-screen-capture roles as missing; these will be collected in a later
showcase/formal-media run rather than relabelling offline renders.
