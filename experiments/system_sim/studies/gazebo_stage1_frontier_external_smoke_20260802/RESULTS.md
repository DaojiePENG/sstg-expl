# External frontier baseline system-simulation smoke

This is development evidence, not a formal comparison result. It is the first
end-to-end Gazebo--SLAM--Nav2 run of the pinned, unmodified
`frontier_exploration_ros2` v1.6.1 MRTSP-DP explorer through the common action,
trace and budget adapter. Raw artifacts are kept only below
`system_sim_outputs/runs/gazebo_stage1_frontier_external_smoke_20260802/` and
derived tables below
`system_sim_outputs/reports/gazebo_stage1_frontier_external_smoke_20260802/`.
Neither path is the legacy benchmark `outputs/` tree.

## Artifact gate

The single office run finished with runner status `terminal_completed` in
149.42 s wall time. Both required external processes, `frontier_explorer` and
`frontier_action_adapter`, remained alive until coordinated shutdown. The
artifact audit reported no errors and verified:

- one `session_started` and one `session_finished`, with termination reason
  `action_budget`;
- ten decisions and ten terminal executions with no rejected trace records;
- an artifact-valid, settled evaluator snapshot;
- a Zstd-compressed MCAP read to EOF with 118,376 messages over 132.072 s and
  all required nonempty topics;
- 143,807,811 bytes of MCAP data and exact SHA-256 identities in the run
  manifest.

## Development metrics

| Metric | Value |
| --- | ---: |
| Information coverage | 0.865420 |
| Topological / joint coverage | 0.056578 |
| Target recall proxy | 0.75 (3 / 4) |
| Ground-truth travel | 32.385 m |
| Trace-reported odom travel | 32.292 m |
| Collision count | 0 |
| Minimum static footprint clearance | 0.218 m |
| ATE RMSE | 0.0221 m |

The dual 0.95 coverage threshold was not met. This single-seed smoke validates
the data path only and must not be interpreted as a performance ranking.

## Preemption and budget finding

All ten downstream actions ended with Nav2 status CANCELED and
`cancel_origin=upstream_cancel_request`; none was aborted or rejected by Nav2.
The upstream implementation explicitly treats CANCELED as expected during its
revealed-frontier, visible-gain and close-enough preemption paths. Its robot
therefore mapped substantial space and detected three targets despite a raw
navigation-success count of zero.

This prospectively changes the comparison gate: a raw NavigateToPose dispatch
cap cannot be the primary matched budget against policies that select only
after each terminal action. Later comparisons will keep a generous shared
action cap as a fail-safe while duration and distance are the binding matched
budgets. The evaluator will also report upstream cancellations separately from
non-cancel navigation failures. This finding was made before any formal result
was opened.

## Topological measurement finding

The reported C-T value, 0.056578, contains only the initial node because the
first adapter contract required Nav2 SUCCEEDED before admitting a terminal
pose. That rule is lifecycle-biased for this upstream: all ten action terminals
were normal policy-requested cancellations. As a transparent offline
diagnostic only, spatially merging those ten observed terminal poses at 0.25 m
would retain ten nodes including the start and yield C-T 0.431455. This does not
replace or mutate the artifact-valid result above.

Later development runs prospectively use `policy_transition_node_v1`. It
freezes a pose and simulation timestamp when the explorer requests cancellation
or accepts a replacement goal, commits a separate hashed trace event only after
the transition is confirmed, and excludes adapter budget/timeout, rejection,
transport failure and unconfirmed cancellation. Raw CANCELED/ABORTED/SUCCEEDED
counts remain separate. This construct was fixed before opening formal results.

## Visual evidence

The run contains three registered and hashed artifacts under `media/raw/`:

- `final_state.png`: final SLAM map and registered ground-truth path;
- `sensor_sanity.png`: final LaserScan with 360/360 valid returns;
- `task_camera_depth.mp4`: 661 H.264 frames at 640 x 560 and 5 fps, covering
  132.2 s with a fixed 0.05--5.0 m scale.

The media manifest correctly reports the Gazebo-overview and RViz-navigation
roles as missing because this scheduled pass was headless. Those views require
a separate GUI showcase capture and are not inferred from offline data.
