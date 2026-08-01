# ROS 2 system-level simulation

This tree contains simulator-neutral experiment definitions.  Gazebo is the
first backend; a future Isaac backend must use its own backend label and may not
be pooled into the Gazebo main table without a fully paired design.

## Directory contract

```text
ros2_ws/src/                         ROS packages, worlds and runtime nodes
experiments/system_sim/configs/      shared stack, methods and perturbations
experiments/system_sim/registries/   versioned world and asset identities
experiments/system_sim/studies/      frozen schedules and hashes
system_sim_outputs/runs/<study_id>/  raw bags, media and per-run logs (Git-ignored)
system_sim_outputs/reports/<study_id>/ derived tables and figures (Git-ignored)
system_sim_outputs/preflight/        host/runtime readiness records (Git-ignored)
```

World truth and target registries are evaluator-only.  The policy node subscribes
to `/map` and TF, sends goals through Nav2, and publishes `/policy/*` traces; it
does not subscribe to `/evaluation/*`.

The default robot is not project-authored.  It is Nav2's Apache-2.0
`nav2_minimal_tb3_sim` TurtleBot3 Waffle, including its released SDF/URDF,
meshes, dynamics, LiDAR, depth camera, bridge and spawn launch.  The exact apt
version and upstream release/patch commits are frozen in `configs/shared_stack.yaml` and
`ros2_ws/src/sstg_gazebo/UPSTREAM.md`.  Repository-owned simulation code is
limited to experiment worlds and narrowly allowlisted evaluator
instrumentation.  See `OPEN_SOURCE_REUSE.md` and
`registries/upstreams.yaml` for the reuse and provenance gate.

Executed runs also reserve `media/` for checkable Gazebo/RViz/sensor captures,
short videos, and a hash manifest.  Raw bags are retained so figures and video
frames can be regenerated; development media is labeled separately from formal
evidence.

Scheduled runs also start the upstream `ros2 bag record` CLI automatically.
The frozen `zstd_fast` MCAP profile is written to `bags/core`; its topic counts,
duration, metadata, MCAP files and SHA-256 hashes are part of terminal artifact
validation.  The audit opens the bag with upstream `rosbag2_py`, reads every
record to EOF, and cross-checks message counts, per-file counts, framing and
frozen ROS topic types.  A missing `/map`, `/scan`, ground-truth path, policy
trace, world clock/stats, or task-camera stream fails the run instead of
producing a partial figure later.

## Host dependencies

Use the ROS system Python, not the project's Python 3.10 Conda environment:

```bash
sudo apt update
sudo apt install \
  python3-skimage \
  python3-vcstool \
  python3-rosdep \
  python3-colcon-common-extensions \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-minimal-tb3-sim \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro \
  ros-jazzy-robot-localization \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-image-view \
  ffmpeg \
  imagemagick \
  qtbase5-dev
```

The repository owner must run this once because unattended `sudo` is not
available to the development agent.

The apt repository currently provides `ros_gz_bridge` 1.0.22.  Simulation
runs require the upstream 1.0.23 Jazzy source overlay because it contains the
official sensor-message bounds fix.  Import the exact commits before building:

```bash
cd /home/daojie/SSTG_Explorer/sstg-expl
mkdir -p ros2_ws/src/third_party
vcs import ros2_ws/src/third_party < ros2_ws/third_party.repos
```

## Build and development launch

```bash
cd /home/daojie/SSTG_Explorer/sstg-expl
cd ros2_ws
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
unset RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
/usr/bin/colcon build --symlink-install \
  --packages-select ros_gz_bridge \
  --allow-overriding ros_gz_bridge \
  --cmake-clean-cache \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DProtobuf_PROTOC_EXECUTABLE=/usr/bin/protoc
/usr/bin/colcon build --symlink-install --cmake-clean-cache \
  --packages-select \
  sstg_explorer_core sstg_gazebo sstg_nav_bringup \
  sstg_policy_ros sstg_system_eval \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML
cd ..
/usr/bin/python3 scripts/preflight_system_sim.py --require-runtime
ros2 launch sstg_nav_bringup system_sim.launch.py \
  policy_seed:=101 simulation_seed:=101 \
  max_duration_s:=300 max_distance_m:=60 \
  max_decisions:=30 goal_timeout_s:=90
```

Keep the system `protoc` ahead of Conda while compiling this overlay.  A Conda
protobuf compiler paired with ROS's system protobuf libraries is an unsupported
mixed toolchain and fails during linking.

Run the upstream bridge tests from the workspace directory:

```bash
cd /home/daojie/SSTG_Explorer/sstg-expl/ros2_ws
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
unset RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/usr/bin/colcon test --packages-select ros_gz_bridge \
  --event-handlers console_direct+
/usr/bin/colcon test-result --test-result-base build/ros_gz_bridge --verbose
```

The clean `/opt/ros/jazzy` plus Fast DDS build passed all 18 upstream CTest
groups (`383 tests, 0 errors, 0 failures`), including all six sensor bounds
regressions.  An earlier host-environment diagnostic with a custom CycloneDDS
underlay failed the unrelated GID filtering test; the registry keeps that
historical result separate from the clean verification.

Headless smoke run:

```bash
ros2 launch sstg_nav_bringup system_sim.launch.py \
  headless:=true rviz:=false \
  policy_seed:=101 simulation_seed:=101 \
  max_duration_s:=300 max_distance_m:=60 \
  max_decisions:=30 goal_timeout_s:=90 \
  output_dir:=system_sim_outputs/runs/gazebo_dev_v0/manual_smoke
```

The first launch is an engineering run, not a paper result.  Formal runs begin
only after the test worlds, schedules, parameters and hashes are frozen.

## Freeze and launch a scheduled run

The schedule freezer reads each registered `starts.yaml` pose, converts
`yaw_deg` to the Gazebo spawn yaw, and records the evaluator registration as
`T_map_truth = inverse(T_world_spawn)`.  It also extracts the SDF `world_name`
and assigns every row a distinct directory below
`system_sim_outputs/runs/<study_id>/`.

```bash
/usr/bin/python3 scripts/generate_system_sim_schedule.py \
  --study-id gazebo_dev_v0 \
  --method sstg \
  --condition nominal \
  --replicate-seed 101 \
  --randomization-seed 711 \
  --start-policy all
```

The four positive policy limits in `configs/shared_stack.yaml` are part of the
matched-stack independent-variable contract.  They are copied into the CSV,
freeze manifest, launch command, run manifest and policy manifest, with hashes
checked at every boundary.  Development schedules may explicitly override
them with `--max-duration-s`, `--max-distance-m`, `--max-decisions` and
`--goal-timeout-s`; formal schedules reject every such override and always use
the frozen shared-stack values.

The replicate seed is also copied to both `policy_seed` and
`simulation_seed`.  Gazebo is launched with that seed, so the simulator's
LiDAR/IMU noise stream and the policy's stochastic choices are paired within a
matched block.  Valid replicate seeds are `1..2147483647`; zero does not freeze
Gazebo's random stream.  Direct development launches must set both arguments
explicitly; the schedule freezer and runner cross-check this shared-stack
contract before reserving an output path, then record both values.
This controls the declared stochastic inputs; asynchronous ROS scheduling means
it is not a claim of bitwise-identical trajectories, so repeat runs remain part
of the development gate.

ROS middleware is fixed to the Jazzy apt `rmw_fastrtps_cpp` 8.4.4 build.
Execution requires the explicit `RMW_IMPLEMENTATION` value, rejects custom DDS
profile variables, and audits all ROS prefix-path environment variables against
only this workspace's install/build artifacts and `/opt/ros/jazzy`.  The build
root is needed by Python packages produced with `colcon --symlink-install`.  The
gate also hashes the selected RMW library and confirms its shared Fast DDS
dependencies resolve below the official ROS prefix.  This prevents a real-robot
or Conda underlay from changing simulator scheduling as an undeclared control
variable.

The `ros_gz_bridge` source-overlay contract follows the same fail-closed path.
Freezing copies its exact version, official tag and commit, required fix,
checkout and workspace prefix into the schedule manifest.  Planning checks all
three frozen copies without requiring a live ROS graph.  Execution and runtime
preflight additionally require the workspace prefix, package version, clean
source HEAD, fix ancestry, `parameter_bridge` binary and linked overlay library
before any run directory is reserved.  The final run manifest records SHA-256
hashes for both executable objects; resolving only the system apt 1.0.22 package
is a hard failure.

Inspect one row's exact launch plan without starting ROS or writing files:

```bash
/usr/bin/python3 scripts/run_system_sim_schedule.py \
  --schedule-dir experiments/system_sim/studies/gazebo_dev_v0 \
  --schedule-id '<schedule_id from run_schedule.csv>'
```

Add `--execute` only after checking the printed command.  Execution atomically
reserves the row's output directory and writes `run_launch_manifest.yaml`
before invoking ROS.  The runner captures all launch output in `launch.log`,
waits for `session_finished` in `policy_trace.jsonl`, gives the evaluator two
seconds to flush its terminal snapshot, then sends `SIGINT` only to the
`ros2 launch` leader so it can stop each child once.  Residual group members
receive `SIGTERM` and then `SIGKILL` only if necessary.  Set the hard process
limit with `--wall-timeout-s` (default 1200 seconds).  The default lifecycle
grace is 15 seconds and can be audited or changed with `--sigint-grace-s` and
`--term-grace-s`; both effective values are written to the run manifest.

An existing path is refused even when empty; choose a new study/run ID instead
of reusing it, because the runtime JSONL writers append.  A zero ROS launch
return code is not sufficient for completion.  `terminal_completed` is written
only when both manifests, all policy/evaluator JSONL evidence, evaluator
ingestion of `session_finished`, the final evaluator snapshot, and their
SHA-256 hashes pass the runner audit.  The finalized core MCAP and its required
non-empty topics must pass the same audit.  The audit also rejects fatal runtime
markers and any child-process crash in `launch.log`; an expected `-2` signal
exit is accepted only inside the runner's ordered shutdown markers.  The same
rule covers recorded SIGTERM/SIGKILL escalation, while required long-running
nodes exiting before that marker invalidate the run.  Other terminal statuses
include `timeout`, `early_exit`, `manual_interrupt`, and
`artifact_validation_failed`; `shutdown_failed` is used if the launch leader
survives the full signal escalation.

## Analyze a frozen study

```bash
/usr/bin/python3 scripts/analyze_system_sim_experiments.py \
  experiments/system_sim/studies/gazebo_dev_v0
```

The analyzer starts from every frozen schedule row, so failed and unexecuted
runs remain visible.  It verifies completed-run artifact hashes and the final
evaluator snapshot before producing:

```text
system_sim_outputs/reports/<study_id>/analysis/system_sim_runs.csv
system_sim_outputs/reports/<study_id>/analysis/system_sim_method_aggregate.csv
system_sim_outputs/reports/<study_id>/analysis/system_sim_main_table.tex
system_sim_outputs/reports/<study_id>/analysis/analysis_manifest.json
system_sim_outputs/reports/<study_id>/analysis/analysis_manifest.sha256
```

Method intervals use a fixed-seed bootstrap over replicate-seed means, with
worlds and starts kept inside each seed cluster.  Task completion, evaluator
dual-threshold success, and collision-free status are separate outcomes.
Missing clearance, ATE, contact, target-proxy, or other evaluator fields remain
NA and are counted in the analysis manifest; they are never replaced with
zeros or inferred from another metric.

For a descriptive same-seed development audit, compare two terminal-completed
runs directly:

```bash
/usr/bin/python3 scripts/analyze_system_sim_repeatability.py \
  system_sim_outputs/runs/<study_a>/<schedule_a> \
  system_sim_outputs/runs/<study_b>/<schedule_b> \
  --output-dir system_sim_outputs/reports/<repeatability_report_id>
```

The repeatability analyzer verifies source/config fingerprints, declared
artifact hashes and both Gazebo/policy seed attestations before producing run
and delta CSVs, JSON and a PNG comparison.  It is descriptive and deliberately
does not derive a pass/fail tolerance from the observed pair.

## Regenerate and register visual evidence

The offline renderer reads the final `/map`, evaluator-only ground-truth path,
and optionally `/scan` directly from a run's core MCAP.  It refuses path
escape, symlinks and overwrite, and visibly labels every output as development
simulation evidence:

```bash
/usr/bin/python3 scripts/render_system_sim_bag_media.py \
  system_sim_outputs/runs/<study_id>/<schedule_id> \
  --sensor-sanity

/usr/bin/python3 scripts/render_system_sim_depth_video.py \
  system_sim_outputs/runs/<study_id>/<schedule_id>

/usr/bin/python3 scripts/register_system_sim_media.py \
  system_sim_outputs/runs/<study_id>/<schedule_id> \
  --evidence-tier development
```

The depth-video renderer consumes the upstream TurtleBot3 `32FC1` task camera
stream and writes `media/raw/task_camera_depth.mp4`.  It uses a fixed
0.05--5.0 m color scale, preserves simulation-time playback at 5 Hz, labels
the output as development simulation, and verifies H.264 dimensions, duration,
frame count and SHA-256 before atomically publishing the file.

The registrar hashes the captures and requires Gazebo, RViz, sensor-sanity,
final-state and key-interval-video roles before marking the media bundle
complete.  Raw MCAP remains the source of truth for derived figures.

The upstream depth image is bridged with `ros_gz_image`.  Camera intrinsics are
frozen from the upstream SDF/evaluator geometry rather than bridged at runtime:
the Harmonic `parameter_bridge` CameraInfo converter showed intermittent heap
corruption during otherwise orderly shutdown and is not consumed by the current
LiDAR policy or evaluator.
