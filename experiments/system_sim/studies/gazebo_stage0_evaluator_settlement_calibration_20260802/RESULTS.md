# Evaluator settlement calibration results

This is development calibration evidence, not a formal exploration result. The
frozen protocol uses `dev_office_01`, `start_southwest`, SSTG, the nominal
condition, replicate seed 227, and a one-decision budget. Run artifacts are
under
`system_sim_outputs/runs/gazebo_stage0_evaluator_settlement_calibration_20260802/`;
derived tables are under
`system_sim_outputs/reports/gazebo_stage0_evaluator_settlement_calibration_20260802/`.
Neither path uses the legacy benchmark `outputs/` tree.

## Completion gate

The runner returned zero with status `terminal_completed` after 28.845 s. The
launch process returned zero after coordinated `SIGINT` / Gazebo-server
`SIGTERM` shutdown. The artifact audit reported no errors and passed all of
the following gates:

- policy and evaluator both observed `session_finished`;
- the evaluator emitted both `policy_session_finished` and
  `policy_session_settled` snapshots;
- the settled ATE queue was empty;
- the launch log had no disallowed runtime failure;
- the Zstd MCAP was read to EOF and all required topics were non-empty.

The MCAP contains 14,114 messages over 15.992 s and occupies 11,238,028
bytes. It includes 444 ground-truth odometry messages, 80 LaserScans, 80 task
camera frames, 16 maps, and all frozen policy/evaluator evidence topics.

## Terminal settlement result

| Snapshot | Ground-truth samples | ATE pairs | Pending | Dropped | Pairing fraction |
| --- | ---: | ---: | ---: | ---: | ---: |
| `policy_session_finished` | 153 | 148 | 5 | 0 | 0.967320 |
| `policy_session_settled` | 153 | 153 | 0 | 0 | 1.000000 |

The settled snapshot was emitted exactly 2.0 s of simulation time after the
policy terminal timestamp, matching the frozen `ate_tf_expiration_s` window.
This confirms that selecting the immediate terminal snapshot would silently
omit valid delayed exact-time TF pairs, while the new runner/analyzer path
retains all samples in this calibration.

Before the frozen run, a separate full-stack sentinel probe placed
`known_free_threshold=37` and `use_sim_time=false` in an evaluator-specific
YAML file. The live node reported threshold 37 and simulation time true,
proving that the dedicated evaluator file crossed the top-level launch while
the explicit clock override took precedence. A runtime request to set
`use_sim_time=false` was rejected, and the value remained true. This diagnostic
is stored separately under `system_sim_outputs/calibration/` and is not counted
as a study replicate.

## Short-budget metrics

| Metric | Value |
| --- | ---: |
| Information coverage | 0.125676 |
| Topological / joint coverage | 0.065453 |
| Target proxy recall | 0.000000 |
| Ground-truth travel | 0.525518 m |
| Navigation success | 1 / 1 |
| Collisions | 0 |
| Minimum footprint clearance | 1.031085 m |
| ATE mean / RMSE / max | 0.000929 / 0.001157 / 0.001907 m |

Low coverage and zero target recall are expected under the one-decision
calibration budget and are not claims about exploration performance. The
purpose of this run is timestamp-pair settlement, supervision, bag integrity,
and media-pipeline validation.

## Visual evidence

Three captures are registered and hashed under the run's `media/` directory:

- `raw/final_state.png`: final map and registered ground-truth path;
- `raw/sensor_sanity.png`: final LaserScan with 360/360 valid returns;
- `raw/task_camera_depth.mp4`: 80 H.264 frames, 640 x 560 at 5 fps, covering
  the full 16.0 s depth stream with a fixed 0.05--5.0 m scale.

The run was headless, so the media manifest truthfully marks Gazebo overview
and RViz navigation captures as missing. Those roles remain for a later
GUI-enabled showcase pass; the three offline artifacts are development media,
not formal or real-robot evidence.
