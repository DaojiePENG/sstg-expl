# Upstream simulation assets

The default robot is the ROS Navigation project's minimal TurtleBot3 Waffle
simulation.  SSTG does not vendor or reimplement the robot description.

- Package: `nav2_minimal_tb3_sim`
- Apt version: `1.0.1-1noble.20260616.074421`
- Repository: `https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation`
- Release tag / commit: `1.0.1` /
  `6b64127f0e0d677ecdaa458bce57b89119cb08ee`
- Released SDF xacro SHA-256:
  `133ebbe76997b98e43dbe03aea5a77dd6bd4117a3100343d5857b63cd3128a83`
- Audited upstream IMU-fix commit:
  `b9d523ad7ea0e98174627fdefeb4b1ae9b515063`
- License: Apache-2.0
- Robot SDF: `share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro`
- Robot URDF: `share/nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf`
- Spawn launch: `share/nav2_minimal_tb3_sim/launch/spawn_tb3.launch.py`

The repository-owned Gazebo package supplies experiment worlds,
evaluator-only bridges, composition launch, and a narrow runtime instrumenter.
Immediately before launch, the instrumenter synchronously renders the
installed `gz_waffle.sdf.xacro` and writes a content-addressed derivative under
`/tmp/sstg_gazebo/instrumented_models/`.  Nav2's released
`spawn_tb3.launch.py` then spawns that completed file.  There is no detachable
overlay model and no asynchronous render/spawn race.

The derivative allowlist contains exactly three additions:

1. One contact sensor for each collision already present on an upstream link.
   Each sensor refers to the existing collision by name; no collision element
   or geometry is changed.
2. Gazebo's `OdometryPublisher` system on the existing robot model, publishing
   only evaluator ground truth.
3. If absent, the fixed `imu_joint` from upstream Jazzy commit
   `b9d523ad7ea0e98174627fdefeb4b1ae9b515063` (parent `base_link`, child
   `imu_link`, pose `0.0 0 0.068 0 0 0`).  Apt 1.0.1 predates that fix.

The instrumenter removes those allowed additions in memory and verifies that
the complete upstream XML structure is recovered before materializing the
derivative.  Thus the released DiffDrive configuration, original sensors,
mesh URIs and scales, inertials, joints, and collision geometry remain
unchanged.  In particular, only the missing `imu_joint` is backported; later
upstream collision-size changes are intentionally not copied into the apt
1.0.1 model.

Nav2 parameters are copied from `ros-jazzy-nav2-bringup` 1.3.12 and changed
only where the experiment needs a larger local window, LiDAR range, explicit
TurtleBot3 geometry, or conservative controller limits.  SLAM uses the
released `slam_toolbox` launch and node.
