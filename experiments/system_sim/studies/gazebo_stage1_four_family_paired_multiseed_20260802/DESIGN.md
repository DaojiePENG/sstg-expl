# Four-family paired multi-seed development design

This is a development validation study, not a formal or real-robot result.
It contains four existing development scene families, the first registered
start in each world, two matched replicate seeds and the two adapter-verified
methods: SSTG and the pinned external MRTSP-DP frontier implementation.

## Frozen scope

- Worlds: `dev_corridor_01`, `dev_lab_01`, `dev_office_01`,
  `dev_warehouse_01`.
- Replicate seeds: 277 and 283.
- Condition: nominal.
- Matching keys: world, start, condition and replicate seed.
- Budget: 240 s, 35 m trace distance, 100 decisions and 90 s per goal.
- Raw-output root:
  `system_sim_outputs/runs/gazebo_stage1_four_family_paired_multiseed_20260802/`.
- Analysis population: all 16 scheduled rows; no localization-based exclusion.

## Restricted method-order randomization

The initial candidate randomization seed was 1061.  Before any run was
executed or output directory reserved, its deterministic SHA-256 allocation
was inspected and found to place the external method first in six of eight
blocks.  To control long-batch order drift, the prospective restricted rule is:

1. enumerate integer seeds starting at 1061;
2. apply the freezer's unchanged `sha256-key-sort/v1` method ordering to all
   eight frozen block IDs;
3. select the first seed assigning each method to order position 1 exactly
   four times.

The selected seed is 1064.  This rule used schedule allocation only and was
completed before observing any experimental outcome.  Method order remains
independently hash-derived within each block; no run is reordered after
execution.

## Interpretation boundary

The study checks recurrence, artifact completeness, localization behavior and
directional stability across development scenes.  Two seeds are insufficient
for a formal ranking.  All failures and missing outcomes remain visible, and
ATE is a continuous secondary outcome rather than an exclusion rule or an
adjustment covariate.
