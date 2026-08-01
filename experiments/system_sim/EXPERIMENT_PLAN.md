# Gazebo embodied-simulation execution plan

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: plan + run
- Origin Date: 2026-08-01
- Verification Status: PARTIALLY VERIFIED
- Version Label: system_sim_plan_v1
- Current Backend: Gazebo Harmonic with ROS 2 Jazzy

## Objective and evidence boundary

The system simulation tests whether SSTG's belief-only joint coverage behavior
survives a complete ROS 2 autonomy stack with simulated dynamics, LiDAR, online
SLAM, Nav2 execution, latency and failures.  It is system-level simulation
evidence, not real-robot evidence.  Physical experiments remain a separate
sim-to-real validation stage.

The policy independent variable is the global exploration method.  Robot
geometry, sensor profile, SLAM, Nav2, controller, budgets, world, start, target
layout and matched noise seeds are controls.  Primary outcomes remain joint
success, geometric task-target recall and redundant-node fraction;
ground-truth travel, time, actions, collision events, clearance, localization
ATE, failures and decision latency are secondary outcomes.  Target recall is
an evaluator-only camera proxy (range, horizontal/vertical FOV, surface
incidence and truth-map line of sight), not a learned detector accuracy claim.

## Stages and mandatory gates

| Stage | Scope | Evidence gate |
|---|---|---|
| 0 Bring-up | Development calibration/office world | TF tree, `/scan`, `/odom`, `/map`, Nav2 action and policy trace all live; robot moves without truth access |
| 1 Fairness | 4 families × 1 dev layout × (SSTG + 3 internal ablations) × 2 seeds = 32 development runs | Diagnose shared-stack fairness only; internal switches are not public baselines |
| 2 Public-baseline pilot | SSTG + each adapter-ready pinned upstream method on development layouts | Freeze adapters, budgets, primary public baseline and failure taxonomy; no pilot run enters formal statistics |
| 3 Freeze | 12 untouched test layouts and schedules | World/config/source hashes and preregistration complete before opening test results |
| 4 Nominal | 12 test layouts × M adapter-verified methods × 5 seeds | Complete paired blocks, retained failures, layout/family-stratified inference; M is frozen only after external E2E gates |
| 5 Sensor-only | 4 × 3 × 2 methods × 3 seeds = 72 runs | Quantify information/topology construct mismatch separately |
| 6 Robustness | 4 challenge layouts × 2 methods × 3 perturbations × 5 seeds = 120 runs | LiDAR, odometry and latency/dropout conditions remain separate from nominal |
| 7 Reproduction | Approximately 10% of frozen runs | Deterministic structure and stochastic metrics within preregistered tolerance |
| 8 Physical transfer | 2 sites × 2 methods × 5 blocks plus sensor-only | Exploratory real-robot evidence; never pooled with simulation |

Formal Gazebo analysis has 12 independently frozen layouts; seeds are
within-layout repeats and must not be described as independent environments.

## Current stage

Stage 0 is active.  The belief-only incremental core and ROS adapter tests are
implemented.  Four development-only scene families now have frozen static
bundles: multi-room office, dense laboratory, warehouse aisles and corridor
alcoves.  The default robot and its LiDAR, depth camera, IMU, differential
drive, bridge and spawn launch are reused from Nav2's released
`nav2_minimal_tb3_sim` TurtleBot3 Waffle package; this repository owns only
the SSTG adapter, experiment worlds, allowlisted evaluator instrumentation and
protocol.  A five-minute office development smoke has verified Gazebo dynamics,
LiDAR and odometry bridging, a growing SLAM map, policy-driven robot motion and
13 completed Nav2 goals.  It ended at the development runner's wall timeout,
so the run itself is not task-complete.  A standalone post-fix probe verified
the scoped 320x240 depth-camera bridge at 5 Hz.  The clean evaluator shutdown
path and frozen-budget terminal event still require a scheduled runtime repeat.

## Stage-0 success criteria

1. `gz sim` loads the development office world and robot without SDF/plugin errors.
2. ROS receives monotonic `/clock`, 5 Hz `/scan`, `/odom`, `odom -> base_footprint`
   and camera messages.
3. SLAM Toolbox publishes a growing `/map` and `map -> odom`.
4. Nav2 accepts at least one policy-selected goal and returns success.
5. `policy_trace.jsonl` contains generated, active, selected and executed states.
6. Ground-truth travel exceeds 1 m; no collision or manual pose teleport is used.
7. No `/evaluation/*`, world-state or ground-truth artifact is visible to the policy.
8. A second same-seed smoke execution produces the same first policy target.
9. Estimated odometry ATE, ground-truth path length and target-proxy recall are
   present and finite in the evaluator artifact.

The next gate is mandatory: no multi-run pilot begins before all nine checks
are recorded in a smoke-run manifest.

## Reuse and baseline identity gate

The upstream registry and integration rules are frozen in
`registries/upstreams.yaml` and `OPEN_SOURCE_REUSE.md`.  The repository's
`frontier`, `nbv`, and `rrt_adapted` switches are development ablations, not
independent public implementations, and their method configs explicitly bar
formal-baseline use.  A formal schedule fails closed when any method lacks
`formal_method_eligible: true`.

The first public candidate is the pinned Apache-2.0
`frontier_exploration_ros2` v1.6.1 commit, recorded under the unambiguous ID
`frontier_mrtsp_dp_external_v1_6_1`.  Source and upstream Jazzy CI have been
audited, but it remains ineligible until a common Nav2 action/trace/budget
adapter and local Gazebo--SLAM--Nav2 end-to-end test pass.  Its frontier-
exhaustion event is only a termination reason; the evaluator, not the method,
determines coverage and joint success.

## Visual evidence contract

Every executed smoke or pilot run reserves a `media/` directory alongside raw
metrics.  At minimum it will contain a Gazebo overview, an RViz map/TF/path
view, a sensor sanity image, a final-state view, and a short key-interval video.
Each file is listed with SHA-256, capture time, run ID, evidence tier, and
whether it is a raw capture or derived visualization.  Development captures
are watermarked or captioned as development evidence and cannot be substituted
for frozen formal figures.  ROS bags remain the source for regenerating plots
and selected video frames.
