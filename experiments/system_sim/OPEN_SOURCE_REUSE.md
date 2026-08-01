# Open-source reuse boundary

The system simulation intentionally owns only the SSTG policy adapter,
experiment scheduler, evaluator, controlled scene definitions, and thin
integration code.  Robot dynamics and sensors, ROS--Gazebo transport, SLAM,
navigation, and public comparison methods should come from traceable upstream
projects whenever a compatible implementation exists.

Exact versions, commits, hashes, licenses, audit state, and local-change scope
are recorded in `registries/upstreams.yaml`.  A version label or README alone
is not a sufficient source pin.  Source dependencies use an immutable commit
in `ros2_ws/third_party.repos` and are imported outside the tracked project
source tree:

```bash
mkdir -p ros2_ws/src/third_party
vcs import ros2_ws/src/third_party < ros2_ws/third_party.repos
```

The current robot is Nav2's released TurtleBot3 Waffle.  The runtime derivative
adds evaluator-only instrumentation and an exact upstream IMU-joint fix; it
does not replace or alter the released drive, inertials, collision geometry,
LiDAR, depth camera, meshes, primary bridge, or spawn launch.

ROS--Gazebo message conversion uses the unmodified official Jazzy
`ros_gz_bridge` 1.0.23 source overlay pinned in `third_party.repos`.  The system
apt candidate remains 1.0.22 and is intentionally ineligible: the 1.0.23 tag
contains the official Jazzy sensor-message bounds fix at commit
`4c6cb80bb30fc0871bbd5ec95761272ce49a150d`, covering CameraInfo, LaserScan,
and JointState converters.  The locally observed shutdown heap corruption was
on a CameraInfo bridge path; applying the full official fix avoids maintaining
a speculative local patch.  Gazebo launch, image bridging, interfaces, robot
bridge configuration, and spawn composition remain upstream code; no local
converter patch is carried.

The existing `frontier`, `nbv`, and `rrt_adapted` strategy switches in the SSTG
codebase are internal algorithmic ablations.  They are useful for development
diagnosis but are not independent public baselines and are barred from formal
baseline claims.  The first audited external candidate is
`frontier_mrtsp_dp_external_v1_6_1`.  It must keep that specific identity: it
is WFD plus decision-map optimization, costmap filtering, MRTSP, bounded-horizon
DP, and preemption, not classic nearest-frontier.

That external package is admitted only to development after a common action
proxy / trace adapter exists.  The adapter must provide the same lifecycle,
budget, Nav2 result, endpoint-node, and terminal trace contract as SSTG while
remaining unable to read `/evaluation/*`.  Its upstream
`exploration_complete` event means frontier exhaustion, not coverage success;
the independent evaluator still decides coverage, target recall, collision,
and joint success.  Until local build and Gazebo--Nav2--SLAM end-to-end gates
pass, it remains ineligible for a confirmatory table.

Experiment-specific room topology and collision truth may be generated locally
because they are controlled independent variables.  Decorative meshes and
semantic furnishings should be taken from license-audited Gazebo Fuel or other
open assets, pinned by owner/version/hash, while the primitive collision truth
remains stable.  Asset-rich and geometry-controlled suites must be reported as
separate strata rather than silently mixed.
