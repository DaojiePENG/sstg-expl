# SSTG system evaluator

`sstg_system_eval` is a separate ROS 2 process that owns all access to the
Gazebo ground-truth map. It never publishes goals, commands, belief maps or
other inputs consumed by the exploration policy.

Inputs:

- `/map` (`nav_msgs/OccupancyGrid`), used as the SLAM belief snapshot
- `/tf` and `/tf_static`, sampled as the odometric `odom -> base_footprint` path
- `/policy/decision_trace` (`std_msgs/String` JSON), used for action outcomes
- `/evaluation/ground_truth_odom` (`nav_msgs/Odometry`), used only for physical
  travel, timestamp-paired planar ATE and the target-visibility proxy
- `/evaluation/contacts` (`ros_gz_interfaces/Contacts`), used only for safety
- `/evaluation/world_stats` (`ros_gz_interfaces/WorldStatistics`), used only
  for simulator-clock and real-time-factor diagnostics
- evaluator-local `truth_map.yaml`, PGM image and `targets.yaml`

Outputs are restricted to the evaluator namespace:

- `/evaluation/metrics`: strict JSON snapshots
- `/evaluation/status`: transient-local strict JSON node state
- `evaluation_metrics.jsonl`: append-only events and metric snapshots
- `evaluation_observed_policy_trace.jsonl`: accepted trace records as observed
- `evaluation_manifest.json`: truth hash, parameters and topic boundary

Information coverage C-I (`geometric_coverage`) is
`truth-free cells classified known-free by /map / all truth-free cells`.
Topological coverage C-T is the fraction of registered truth-free cells inside
the union of `topological_radius_m` disks centered on actual nodes. Only the
initial node in `session_started` and successful `execution` records with
`topological_node_created=true` enter C-T. Raw and spatially deduplicated node
counts are both retained. Joint coverage is `min(C-I, C-T)` and success requires
both frozen thresholds (0.95 by default). The node also reports TF-sampled path
length, action counts and success rate, decision time, trace-reported path
length, a recomputed trace-polyline length, and their disagreement.

The primary travel metric is the physical XY polyline accumulated from
`/evaluation/ground_truth_odom` between accepted `session_started` and
`session_finished` policy events; pre-session spawn/settling motion is excluded.
The TF-sampled `odom -> base_footprint` length spans evaluator lifetime and is
retained only as an estimated-odometry diagnostic; it must not replace physical
travel in result tables. Planar ATE transforms every in-session Gazebo/world
truth sample with the same explicit `T_map_truth` used for coverage, then pairs
it by stamp with an exact-time `map -> base_footprint` TF lookup after a short
buffering delay. The snapshot exposes accepted, waiting, expired-pair and
pairing-fraction fields; a run with no ATE pairs is not a localization-accuracy
result.

Static truth clearance is sampled at the same in-session ground-truth poses.
`raw_static_obstacle_distance_{min,p05,mean}_m` is robot-center distance to the
nearest occupied/unknown truth-cell square or map exterior. The primary
`footprint_clearance_{min,p05,mean}_m` subtracts the frozen conservative
clearance-radius parameter (`robot_clearance_radius_m`, currently `0.24 m` for
the upstream TB3 circular footprint plus frozen costmap padding), then clamps
at zero. Outside-map samples are counted, included as zero
in both series, and reported as a fraction; this is deliberately
fail-conservative. These are pose-sample-weighted static 2-D truth-map
clearances, not dynamic-object or 3-D clearances.

World statistics report latest/elapsed simulation and Gazebo real time, latest
and mean reported RTF, mean RTF recomputed from consecutive deltas, paused
state/fraction, stepping state, iteration span, model count, step size, stalls
while unpaused, and non-monotonic clock/iteration counts. A non-monotonic input
marks the diagnostic `degraded_nonmonotonic_clock` rather than silently
producing a clean timing claim.

Coverage requires an explicit static transform `T_map_truth`. The default
launch values `(x=6.5 m, y=4.5 m, yaw=0)` are the inverse spawn transform for
the bundled `dev_office_01/start_southwest` run, whose wheel odometry starts at
the robot's spawn pose as local zero. Any other world or start must pass its own
`truth_registration_id` and `truth_to_map_*` values; all four fields are saved
in the evaluator manifest. For a spawn pose `(t, yaw)`, use rotation `-yaw` and
translation `-R(-yaw)t` (not simply `-t`). This fixed registration is a
development gate; it assumes SLAM keeps the same initial map geometry.

Collision count is the debounced onset count of attributed robot/non-ground
collision-name pairs, not the number of raw contact points or 50 Hz messages.
Configured floor/ground contacts, robot self-contact, and contacts whose entity
names cannot be attributed to the robot are excluded and separately counted.
This is intentionally conservative: name filtering must be checked against a
live Gazebo message before freezing a world, abnormal chassis-floor contact is
also excluded, and a bridge gap longer than `collision_event_separation_s` can
split a sustained episode. `collision_free` remains `null` until at least one
contacts message establishes that the input is alive, and also remains `null`
when a contact cannot be attributed from entity names or contact timestamps are
non-monotonic. A non-null value applies only within the configured contact
sensor's collision scope.

Coverage is limited to collision geometries selected by the Gazebo contact
sensors. The evaluator-only overlay instruments the upstream TurtleBot3's
conservative footprint without modifying its released dynamics; the evaluator
retains a pair across empty interleaved sensor messages and ends it only after
`collision_event_separation_s`. Entity naming, topic aggregation and floor
filtering still require live-message validation before the safety gate freezes.

Primary target recall is a shared evaluator-only deterministic geometry proxy,
not an image-detector result. For every ground-truth pose it applies the frozen
camera translation, height, yaw/pitch, horizontal/vertical FOV and range;
requires the camera to view the registered front surface; and ray-tests LOS in
the 2-D truth occupancy grid. FOV/range/facing use the target center rather than
its projected panel area. It records detected IDs and each first-seen ROS
time / elapsed time from the accepted `session_started` policy trace. Target
evaluation stops at `session_finished`; pre-session simulator poses cannot
earn recall. The 2-D LOS is conservative (no obstacle height or
transparency), and it omits blur, illumination, dwell time and recognition
confidence. These limitations and all frozen camera fields are emitted in each
snapshot. The policy never subscribes to these truth inputs or evaluator
outputs.

`targets_yaml` is explicit in the evaluator launch. If a higher-level launch
leaves it empty, the evaluator resolves `targets.yaml` beside the world bundle
containing `evaluation/truth_map.yaml`; the resolved absolute path and SHA-256
are always recorded in the manifest.

`allow_existing_output` defaults to `false`.  The scheduler may atomically
reserve the shared run directory and place `run_launch_manifest.yaml`, media,
or bag subdirectories there before ROS starts.  Evaluator startup rejects any
pre-existing evaluator-owned manifest or JSONL file and exclusively creates
its manifest, preventing metric/trace append contamination.  Explicit `true`
is development-only reuse; formal schedules must retain the fail-closed
default.

Run the evaluator alongside the simulation stack:

```bash
ros2 launch sstg_system_eval evaluator.launch.py \
  output_dir:=system_sim_outputs/runs/development/dev_office_01/run_001
```

For formal runs, enforce the filesystem and ROS-domain allowlists specified in
`experiments/system_sim/configs/topic_access.yaml`; this package-level boundary
is an auditable development safeguard, not a security sandbox.
