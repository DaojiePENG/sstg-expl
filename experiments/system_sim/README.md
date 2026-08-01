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

## Build and development launch

```bash
cd /home/daojie/SSTG_Explorer/sstg-expl
source /opt/ros/jazzy/setup.bash
cd ros2_ws
/usr/bin/colcon build --symlink-install --cmake-clean-cache \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
cd ..
/usr/bin/python3 scripts/preflight_system_sim.py --require-runtime
ros2 launch sstg_nav_bringup system_sim.launch.py \
  max_duration_s:=300 max_distance_m:=60 \
  max_decisions:=30 goal_timeout_s:=90
```

Headless smoke run:

```bash
ros2 launch sstg_nav_bringup system_sim.launch.py \
  headless:=true rviz:=false \
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
seconds to flush its terminal snapshot, then stops the complete launch process
group with escalating `SIGINT`, `SIGTERM`, and `SIGKILL` if necessary.  Set the
hard process limit with `--wall-timeout-s` (default 1200 seconds).

An existing path is refused even when empty; choose a new study/run ID instead
of reusing it, because the runtime JSONL writers append.  A zero ROS launch
return code is not sufficient for completion.  `terminal_completed` is written
only when both manifests, all policy/evaluator JSONL evidence, evaluator
ingestion of `session_finished`, the final evaluator snapshot, and their
SHA-256 hashes pass the runner audit.  Other terminal statuses include
`timeout`, `early_exit`, `manual_interrupt`, and
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

## Regenerate and register visual evidence

The offline renderer reads the final `/map`, evaluator-only ground-truth path,
and optionally `/scan` directly from a run's core MCAP.  It refuses path
escape, symlinks and overwrite, and visibly labels every output as development
simulation evidence:

```bash
/usr/bin/python3 scripts/render_system_sim_bag_media.py \
  system_sim_outputs/runs/<study_id>/<schedule_id> \
  --sensor-sanity

/usr/bin/python3 scripts/register_system_sim_media.py \
  system_sim_outputs/runs/<study_id>/<schedule_id> \
  --evidence-tier development
```

The registrar hashes the captures and requires Gazebo, RViz, sensor-sanity,
final-state and key-interval-video roles before marking the media bundle
complete.  Raw MCAP remains the source of truth for derived figures.
