# Stage-2 Gazebo simulation result

## Conclusion

The corridor localization failure is fixed for this development regression,
and the original SSTG policy shows the intended core exploration advantage.
The evidence supports a narrow claim: SSTG acquires information more
efficiently with fewer, non-redundant policy endpoints. It does not dominate
every secondary metric, and the exploratory Gazebo-specific score tuning is
not an improvement.

This is development simulation evidence, not a formal multi-world result.

## 1. Corridor localization regression

Study:
`gazebo_stage2_corridor_localization_regression_v2_20260802`.

All four scheduled runs (SSTG and independent external frontier, seeds 277 and
283) finished with valid artifacts. The old symmetric-corridor failures had
2.1--2.8 m `map -> odom` corrections. After tightening the shared
slam_toolbox loop-closure profile, the largest adjacent corrections were:

| Method | Seed 277 | Seed 283 | ATE RMSE range |
|---|---:|---:|---:|
| SSTG | 0.069 m | 0.076 m | 0.027--0.059 m |
| External frontier | 0.070 m | 0.093 m | 0.041--0.044 m |

No localization-based run exclusion or post-hoc pass threshold was used.

At the same 35 m travel budget, SSTG and the external frontier reached nearly
the same final ideal truth-sensor coverage (97.3% versus 98.3% mean), while
SSTG had higher normalized coverage-distance AUC (0.832 versus 0.816), higher
truth topological coverage (52.0% versus 39.3%), and used 10 unique endpoints
per run instead of 15. SSTG endpoint redundancy was 0%; the external method's
mean raw endpoint redundancy was 44.9%.

Localization figure:
`system_sim_outputs/reports/gazebo_stage2_corridor_localization_final_v2_20260802/localization_diagnostic.png`.

## 2. Office core-method screen

Study: `gazebo_stage2_core_method_screen_v2_20260802` (one frozen development
seed, nominal condition). Five methods reached the 20 m distance budget. The
internal simplified frontier hit the 300 s time cap at 7.96 m and is retained
as an end-to-end outcome, but is not fixed-distance comparable.

| Method | Travel | Truth sensor coverage | Coverage-distance AUC | Truth topological coverage | Unique endpoints | Endpoint redundancy |
|---|---:|---:|---:|---:|---:|---:|
| **SSTG (original)** | 20.12 m | **80.5%** | **0.481** | 35.5% | **7** | **0%** |
| NBV | 20.18 m | 77.4% | 0.476 | 36.0% | 8 | 0% |
| External frontier | 20.22 m | 78.4% | 0.463 | **41.3%** | 11 | 8.3% |
| Adapted RRT | 20.07 m | 75.3% | 0.463 | 35.1% | 8 | 0% |
| SSTG Gazebo tuning | 20.26 m | 76.2% | 0.453 | 40.9% | 8 | 0% |
| Internal frontier* | 7.96 m | 20.8% | 0.190 | 12.8% | 9 | 0% |

The original SSTG ranks first for final information coverage, coverage per
distance summarized by AUC, and endpoint count. The external frontier ranks
first for topological coverage. Therefore the supported result is information
efficiency and decision simplicity, not universal dominance.

The Gazebo-specific tuning (`clearance_weight=0.5`,
`travel_cost_weight=0.8`) traded away too much information efficiency. Keep
the original SSTG weights for subsequent experiments; retain the tuned run as
a negative ablation.

Summary figures:

- `system_sim_outputs/reports/gazebo_stage2_core_method_screen_v2_20260802/figures/core_method_screen.png`
- `system_sim_outputs/reports/gazebo_stage2_core_method_screen_v2_20260802/figures/coverage_vs_distance.png`
- `system_sim_outputs/reports/gazebo_stage2_core_method_screen_v2_20260802/figures/final_state_montage.png`

Each of the six run directories contains an auditable final-state image,
LiDAR sanity image, H.264 depth-camera video, media manifest, and immutable
core MCAP bag below:
`system_sim_outputs/runs/gazebo_stage2_core_method_screen_v2_20260802/`.

## 3. Fairness interpretation

The primary comparison lane uses evaluator-only ideal 2-D truth-ray coverage,
coverage-distance AUC, and truth-frame policy endpoints. The policy never
receives truth-map or ground-truth pose feedback. Every method uses the same
Gazebo world, LiDAR, slam_toolbox, Nav2 stack, start, seed, and budgets.

ATE is only timestamp-paired planar localization error; it contains no
collision or safety term. Collision, static clearance, ATE, and Nav2 technical
failures are retained as separate system diagnostics and are excluded from
the core score. All six office runs recorded zero attributed collisions, so a
safety advantage cannot explain SSTG's core ranking in this screen.

## Next decision

Do not add more methods to this screen. Use original SSTG, NBV, independent
external frontier, and adapted RRT in a later multi-seed/multi-room study.
Keep the tightened shared SLAM profile, and add a small real-robot validation
set only after the simulation protocol is frozen.
