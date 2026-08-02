# Paired office matched-distance pilot results

This is a development-only end-to-end diagnostic, not a formal paper result.
It contains one development world, one start, one nominal condition and one
replicate seed. It therefore must not be used to rank SSTG against the external
frontier method. Raw artifacts are isolated under
`system_sim_outputs/runs/gazebo_stage1_paired_office_pilot_20260802/`; derived
tables and the paired visualization are under the matching
`system_sim_outputs/reports/` directory. Neither path is the legacy benchmark
`outputs/` tree.

## Audit status and matched design

Verification status is `ANALYZED`: the frozen schedule, manifests, raw traces,
MCAP streams, evaluator snapshots, derived tables and media were independently
cross-checked, but the simulation was not rerun. The schedule itself records
`evidence_tier: development` and `formal_result_eligible: false`; in particular,
`dev_office_01` is a development world and neither method is frozen for formal
comparison.

The paired block held the world (`dev_office_01`), start
(`start_southwest` at -6.5 m, -4.5 m, 0 rad), nominal condition, simulation and
replicate seed (263), shared SLAM--Nav2--sensor stack, and evaluator fixed. The
world-bundle, condition and shared-stack hashes are identical across the two
schedule rows. Only the method-specific configuration and resulting run
configuration differ. Method order was randomized with seed 1021 and executed
as SSTG first, then the pinned external method.

Both methods received the same effective caps: 240 s duration, 35 m trace
odometry distance, 100 decisions as a fail-safe, and 90 s per navigation goal.
The distance cap bound both runs at 35.036552 m for SSTG and 35.004212 m for the
external method; neither duration nor decision cap bound. Ground-truth travel
was correspondingly close at 35.151818 m and 35.445418 m. Thus this is a valid
matched-distance development pair, although matching alone does not make one
seed an inferential comparison.

## Completion and artifact gate

| Method | Runner status | Wall time (s) | MCAP messages | MCAP duration (s) | MCAP bytes | Artifact audit |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| SSTG | `terminal_completed` | 212.887 | 192,052 | 195.296 | 238,802,954 | valid |
| External MRTSP-DP frontier | `terminal_completed` | 154.671 | 148,244 | 137.008 | 153,456,155 | valid |

Both launch and supervisor return codes were zero. Each MCAP was read to EOF,
its recomputed SHA-256 matched the run and analysis manifests, and the parsed
policy trace exactly matched the evaluator-observed trace (24 records for SSTG
and 118 for the external method). Both final settled snapshots report zero
trace, topology, map, ground-truth, contact, world-statistics and ATE-TF
rejections. Runtime-applicable hidden Nav2 action evidence is nonempty: SSTG has
17,110 feedback and 21 status messages on the shared action, while the external
run has 12,521/86 shared and 12,519/115 proxy feedback/status messages.
The subsequent repository commit `b030d6c` prospectively makes those two shared
topics mandatory for `sstg_policy` and all four topics mandatory for the
external runtime adapter. That stronger gate does not retroactively change this
completed run's frozen artifact contract.

The Gazebo server followed the runner's documented SIGINT-to-SIGTERM escalation
in both runs; all completion and artifact gates nevertheless passed. This is a
coordinated server shutdown condition, not an observed policy or evaluator
crash.

## Settled development metrics

| Metric | SSTG | External MRTSP-DP frontier |
| --- | ---: | ---: |
| Trace-recomputed odometry travel (m) | 35.036552 | 35.004212 |
| Ground-truth travel (m) | 35.151818 | 35.445418 |
| Information coverage, C-I | 0.718215 | 0.874429 |
| Topological / joint coverage, C-T | 0.493934 | 0.534302 |
| Dual 0.95-threshold success | no | no |
| Target-recall geometry proxy | 0.25 (1 / 4) | 0.75 (3 / 4) |
| Unique topological nodes | 10 | 22 |
| Redundant-node fraction | 0 | 0 |
| Navigation executions | 10 | 29 |
| Collision count | 0 | 0 |
| Mean footprint clearance (m) | 0.663027 | 0.696102 |
| Minimum footprint clearance (m) | 0.046809 | 0.232990 |
| Footprint-clearance fifth percentile (m) | 0.140789 | 0.393164 |
| ATE mean (m) | 1.182033 | 0.029871 |
| ATE RMSE (m) | 1.733847 | 0.036766 |
| ATE maximum (m) | 2.606673 | 0.075054 |

ATE used all 5,251 SSTG and 3,640 external in-session ground-truth samples, with
a pairing fraction of 1.0 and no dropped, expired or pending sample. SSTG's mean
observed decision time was 161.415 ms over ten decisions. The external upstream
component did not expose internal decision timing, so all 29 values remain
explicitly unavailable rather than being imputed. The target value is the
frozen deterministic geometry proxy, not an image-detector score, and collision
freedom is limited to the configured Gazebo contact-sensor scope.

These are the two observed development runs. The aggregate file has only one
replicate seed, so its bootstrap intervals collapse to the point estimates; no
effect estimate, uncertainty claim or method ranking is supported.

## SSTG localization discontinuity

The SSTG ATE is not an evaluator bookkeeping artifact. Direct deserialization
of the raw `/tf` stream found a 2.134426 m consecutive translation change in
`map->odom` at approximately 107.8 s: the transform changed from
`(x=0.033243, y=0.075308, yaw=-0.006980)` to
`(x=-0.090004, y=-2.055557, yaw=-0.019971)`. Further 0.225138 m and 0.088903 m
corrections occurred at approximately 108.2 s and 108.6 s. The evaluator's map
update at 108.184 s simultaneously raised maximum ATE from 0.079046 m to
2.259261 m; it reached 2.544871 m at 109.188 s and 2.603336 m at 110.192 s,
before the final 2.606673 m maximum and 1.733847 m RMSE.

For comparison only as a diagnostic, the largest consecutive `map->odom`
translation update in the external run was 0.103443 m at 74.2 s. The derived
`analysis/localization_diagnostic.json` freezes both maximum-jump records and
the final ATE summaries (2,755 bytes, SHA-256
`70f99c8db25e794ac0249518725929d58d1d201a3e5bcb57ed418eb6baef4d07`).
Its visually checked four-panel plot,
`analysis/localization_diagnostic.png`, shows cumulative ATE and both recorded
`map->odom` translation components (2520 x 1620, 270,484 bytes, SHA-256
`1d2c7a2b09a8eebc89c67334c8fd199e19b45a827944c5adfe7fd05700fb5aea`).
Both derivatives are registered by
`analysis/localization_diagnostic_manifest.json` against the two raw MCAP and
evaluator-metric hashes. Visual inspection of the registered final-state
render additionally shows displaced and duplicated structure in the SSTG map,
consistent with the raw transform jump, while the external render remains
geometrically coherent.

Both methods used the same frozen shared SLAM, Nav2 and sensor stack, but their
policies generated different trajectories. The localization collapse is
therefore part of SSTG's realized end-to-end system outcome in this assigned
run: the associated ATE and mapping/coverage consequences must be retained and
must not be post-hoc removed or independently realigned after seeing the
result. Conversely, one pair cannot establish that SSTG systematically causes
such collapses. Multi-scene, multi-seed repetitions and a prospectively defined
localization-quality sensitivity analysis are required to assess recurrence.

## Cancellation and topology semantics

SSTG's raw trace contains nine successful executions and one final Nav2
`CANCELED` terminal caused by the binding distance budget. Its causal sequence
is unambiguous: `budget_cancel_requested(reason=distance_budget)` at 192.500 s,
the `nav2_status_5` execution terminal at 192.524 s, and
`budget_reached(reason=distance_budget)` at 192.524 s. Because the completed
run's older SSTG trace encoded only the bare reason `nav2_status_5`, the settled
evaluator and derived CSV misclassify this one terminal as one non-cancel
failure and one technical failure, with zero adapter cancellations. That is a
known trace-semantics mismatch, not a navigation failure. The causally correct
development reading is nine successes, one distance-budget cancellation and
zero technical failures for this terminal. The immutable raw run is retained;
the trace reason was corrected prospectively in repository commit `6319759`,
which emits the shared structured cancellation contract. This completed
artifact and its old derived CSV remain unchanged.

The external run has one raw `SUCCEEDED` terminal and 28 raw `CANCELED`
terminals. Of the cancellations, 27 are confirmed upstream cancel requests used
for normal policy transitions and one is the adapter's session termination at
the binding distance budget; every downstream cancel response was accepted.
The evaluator consequently records 27 policy transitions, one adapter cancel,
zero non-cancel failures and zero technical failures. These 28 raw cancellations
must not be described as 28 algorithmic failures.

The pinned, unmodified upstream component is
`frontier_exploration_ros2` v1.6.1 at commit
`b0fad500e5c81ad3154f0469ca283b2702a3f90c`, with algorithm identity
`wfd_decision_map_mrtsp_bounded_horizon_dp_with_preemption`. Its trace contains
28 causal topology events: 27 triggered by confirmed upstream policy
transitions and one by navigation success. Twenty-one created nodes and seven
merged into an existing node; together with the initial pose, this yields 22
unique nodes. The final distance-budget cancellation receives no topology
credit.

## Visual evidence

Both raw media manifests and their checksum files verify. The registered
artifacts are:

| Method | Artifact | Size / duration | SHA-256 |
| --- | --- | --- | --- |
| SSTG | `final_state.png` | 160,983 bytes | `5e2c04a1aeaad93ababcf0cc18ea48ceea048a721b161d019fb7932a81c8ff73` |
| SSTG | `sensor_sanity.png` | 97,010 bytes | `97704f22eafffb1a1689c5e6ab34ffaf4aeb551c81a432714a7c452645dde942` |
| SSTG | `task_camera_depth.mp4` | 977 frames, 195.4 s, 1,123,597 bytes | `4aa4d5b986f5f0db49ed6b164aef99874797c7e264c39c6cc4575a262e8e2bc2` |
| External | `final_state.png` | 159,859 bytes | `3aa9e545431327f9a08dba63670abd7704fbf7725c2cfb06335ff559406263c8` |
| External | `sensor_sanity.png` | 99,794 bytes | `1f4e931afe26c6d7da6d17a0e07a013440c114ec5351efb8a313853e33263389` |
| External | `task_camera_depth.mp4` | 686 frames, 137.2 s, 839,641 bytes | `7dccebccf6a64798a6b210aff8a139ea21122f41acedf83d17d2d4426bff8d46` |

Both videos are H.264, 640 x 560, at 5 frames/s. The derived side-by-side figure
is
`system_sim_outputs/reports/gazebo_stage1_paired_office_pilot_20260802/analysis/paired_final_state.png`
(1968 x 984, 974,141 bytes, SHA-256
`499ed60cadc22a241af76af8772dc7ad5cd834b4f3f11b572bb26318e8b48158`).
It was visually checked, but it is a convenience derivative and is not yet
listed in `analysis_manifest.json`; the two source `final_state.png` files are
the checksum-registered evidence.

The headless media manifests correctly mark Gazebo-overview and RViz-navigation
roles as missing. A separate GUI showcase pass must provide those views rather
than treating the offline final-state renders as substitutes.

## Gate decision

This pilot validates the matched-distance execution, paired artifact path,
cancel-aware external baseline accounting and visual-evidence workflow. It also
exposed the need to preserve the distance-budget cause in SSTG execution traces;
that correction and the adapter-specific action-evidence gate are now closed
prospectively by `6319759` and `b030d6c`. Before a larger batch, the remaining
mandatory design step is to predefine how localization quality and sensitivity
will be reported without excluding runs after observing their outcomes. No
ranking claim is made. The next evidentiary step is paired multi-family,
multi-seed development validation, followed by the separately planned GUI
capture and only then a frozen formal schedule.
