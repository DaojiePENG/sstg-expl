# Four-family paired multi-seed development results

This is development simulation evidence, not a formal paper or real-robot
result. The study contains four development worlds, one registered start per
world, two replicate seeds and two methods: SSTG and the pinned external
MRTSP-DP frontier implementation. Its 16 runs form eight complete matched
pairs. Two seeds and one world per family are insufficient for population
inference or a method ranking.

Raw artifacts remain isolated under
`system_sim_outputs/runs/gazebo_stage1_four_family_paired_multiseed_20260802/`.
Generic analysis, strict paired analysis, localization diagnostics and media
checks are under the matching `system_sim_outputs/reports/` directory. Neither
location is the legacy benchmark `outputs/` tree.

## Audit status and frozen interpretation contract

Verification status is `ANALYZED`: the frozen schedule, all 16 run manifests,
policy and evaluator traces, settled snapshots, generic analysis, paired
analysis and media manifests were cross-checked without rerunning or modifying
the simulation. The schedule was generated from clean repository commit
`aebf272f93de1fdaabf23eedc1c181b76269ec5f`, and records both
`evidence_tier: development` and `formal_result_eligible: false`.

The matching key is world, start, nominal condition and replicate seed. The
four worlds are `dev_corridor_01`, `dev_lab_01`, `dev_office_01` and
`dev_warehouse_01`; seeds are 277 and 283. Each method was prospectively placed
first in four blocks by the documented restricted randomization with seed
1064. Both rows in every block share the world, start, seed, condition,
SLAM--Nav2--sensor stack, evaluator and effective caps of 240 s, 35 m trace
distance, 100 decisions and 90 s per navigation goal.

The paired contract is deliberately strict:

- every delta is `SSTG - frontier_mrtsp_dp_external` within the frozen block;
- all eight pairs and all scheduled localization outcomes are retained;
- no run is excluded for localization quality or any other observed outcome;
- no missing value is imputed; the paired analysis would reject the whole
  analysis rather than silently drop an incomplete pair;
- no significance test, confidence interval or post-hoc pass/fail threshold is
  introduced; and
- ATE is a continuous secondary system outcome, not an adjustment covariate.

All eight pairs are complete and all paired endpoints are present. The generic
method aggregate mechanically includes seed-cluster bootstrap intervals, but
only two seed clusters contribute. Those intervals are not used here as
inferential confidence statements; the strict paired result reports the eight
observed deltas using means, medians, ranges and directions only.

## Completion and artifact gate

| Method | Scheduled | Terminal / artifact-valid | Distance cap | Time cap | Other session end | Dual 0.95 success | Collisions | Technical failures |
| --- | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: |
| SSTG | 8 | 8 / 8 | 7 | 0 | 1 candidate exhaustion | 0 / 8 | 0 | 1 |
| External MRTSP-DP frontier | 8 | 8 / 8 | 6 | 2 | 0 | 0 / 8 | 0 | 0 |

All 16 run and supervisor return codes are zero, all completion checks pass,
and every final `policy_session_settled` snapshot has zero trace, topology,
map, ground-truth, contact, world-statistics and ATE-TF rejection counts. The
16 readable MCAPs contain 3,063,029 messages and 3,241,572,505 bytes in total.
All runtime-applicable shared and proxy hidden action topics are nonempty. The
1,485 parsed policy records exactly equal their evaluator-observed copies.

All runs followed the documented Gazebo-server SIGINT-to-SIGTERM shutdown
path. The run manifests still distinguish task completion from evaluator
dual-threshold success, collision freedom and action-level technical failure.
Consequently, the one technical failure below is retained even though its run
is artifact-valid and `terminal_completed`.

## Strict paired endpoint summary

The two method columns are observed means across the same eight frozen pairs.
`+ / 0 / -` counts the signs of the eight deltas. A positive delta means only
that SSTG's numerical value is larger: it is favorable for coverage or
clearance, unfavorable for ATE, collision and failure counts, and neutral as a
quality direction for travel.

| Endpoint | SSTG mean | External mean | Mean delta | Median delta | Delta range | `+ / 0 / -` pairs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Information coverage, C-I | 0.906345 | 0.873303 | +0.033041 | +0.006666 | [-0.037557, +0.139481] | 5 / 0 / 3 |
| Topological / joint coverage, C-T | 0.507317 | 0.506674 | +0.000643 | -0.019659 | [-0.077330, +0.144515] | 4 / 0 / 4 |
| Target-recall geometry proxy | 0.656250 | 0.593750 | +0.062500 | 0 | [-0.250000, +0.500000] | 3 / 2 / 3 |
| Ground-truth travel (m) | 35.005330 | 33.629062 | +1.376267 | -0.264285 | [-0.546205, +13.202145] | 1 / 0 / 7 |
| Mean footprint clearance (m) | 0.698407 | 0.635085 | +0.063323 | +0.016195 | [-0.054430, +0.297406] | 4 / 0 / 4 |
| Minimum footprint clearance (m) | 0.151171 | 0.193239 | -0.042068 | -0.031212 | [-0.199650, +0.171166] | 2 / 0 / 6 |
| Clearance fifth percentile (m) | 0.258711 | 0.296745 | -0.038035 | -0.044609 | [-0.252720, +0.233449] | 3 / 0 / 5 |
| ATE mean (m) | 0.121337 | 0.292244 | -0.170907 | +0.005694 | [-1.159260, +0.070854] | 5 / 0 / 3 |
| ATE RMSE (m) | 0.206808 | 0.407819 | -0.201011 | +0.005009 | [-1.446163, +0.080102] | 5 / 0 / 3 |
| ATE maximum (m) | 0.421357 | 0.627904 | -0.206547 | +0.007576 | [-1.896451, +0.137900] | 5 / 0 / 3 |
| Collision count | 0 | 0 | 0 | 0 | [0, 0] | 0 / 8 / 0 |
| Technical failures per run | 0.125 | 0 | +0.125 | 0 | [0, 1] | 1 / 7 / 0 |

This table does not show a stable method direction. Information coverage is
higher for SSTG in five of eight observed pairs, but topological coverage splits
four versus four and target recall splits three versus three with two ties.
Minimum and fifth-percentile clearance are lower for SSTG in six and five
pairs, respectively. Both methods are collision-free in the configured Gazebo
contact-sensor scope, but neither run meets the dual 0.95 coverage threshold.
These observations describe this frozen development batch only.

## Budget realization and action semantics

The caps are matched, but the realized stopping mechanism is not identical in
all corridor runs. Thirteen runs bind at 35 m trace distance. Both external
corridor runs bind at the 240 s time budget instead, and SSTG corridor seed 277
ends by candidate exhaustion after 33.266 m trace distance. In seven of eight
pairs SSTG ground-truth travel is 0.119--0.546 m lower. In corridor seed 283,
however, SSTG travels 35.327 m while the time-bound external run travels only
22.125 m, producing the +13.202 m delta that reverses the mean travel delta to
+1.376 m even though its median is -0.264 m. Travel is therefore a budget
realization diagnostic, not evidence that one method explores more efficiently.

Across SSTG, 83 executions partition into 75 successes, seven correctly
attributed distance-budget cancellations and one non-cancel technical failure.
The technical failure occurs in corridor seed 277: decision 9 terminates as
Nav2 `ABORTED` (`nav2_status_6`). The policy later records another success and
then candidate exhaustion, so no causal claim is made that the abort alone
ended the session. The failure remains visible as one of the eight paired
technical-failure deltas.

The external method has 316 raw `CANCELED` terminals: 308 are confirmed
upstream preemption/cancel requests used for normal policy transitions and
eight are adapter session terminations at the binding time or distance budget.
It has zero non-cancel and zero technical failures. Thus 316 raw cancellations
must not be described as 316 algorithmic failures. Its causal transition
contract yields 194 unique nodes across the eight runs, versus 83 SSTG nodes;
node count alone is not a quality ranking, while the common C-T evaluator is
reported above.

## Corridor localization outcomes

The prospectively frozen localization contract has no numerical threshold.
Nevertheless, the raw `map->odom` diagnostics expose three corridor outcomes
that are numerically separated from the rest of this batch and explain why the
ATE means and medians point in different directions.

| Seed | SSTG largest adjacent correction | External largest adjacent correction | SSTG ATE RMSE | External ATE RMSE | Paired RMSE delta | Realized endpoint |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 277 | 2.766 m at 164.4 s | 2.591 m at 157.6 s | 1.302702 m | 1.563370 m | -0.260668 m | SSTG candidate exhaustion with one technical failure; external time budget |
| 283 | 0.105 m at 108.2 s | 2.134 m at 90.2 s | 0.056981 m | 1.503143 m | -1.446163 m | SSTG distance budget; external time budget |

In corridor seed 277, both methods undergo large SLAM corrections and finish
with maximum ATE of 2.745958 m (SSTG) and 2.619889 m (external). In corridor
seed 283, the external method alone undergoes the large correction and finishes
at 1.990445 m maximum ATE, while SSTG finishes at 0.093994 m. Across the other
13 method-runs, the largest adjacent correction is at most 0.175526 m and ATE
RMSE is at most 0.100382 m. This contrast is descriptive; it does not create a
pass/fail threshold.

All three corridor localization outcomes remain in every aggregate and paired
delta. As a result, mean paired ATE RMSE is -0.201011 m, while the median is
+0.005009 m and SSTG actually has the larger RMSE in five of eight pairs. The
negative mean is dominated by the two very large external corridor values
versus one very large SSTG corridor value. It must not be phrased as a stable
SSTG localization advantage. These are end-to-end shared-SLAM system outcomes
under different realized trajectories; none is post-hoc realigned, removed or
used as an adjustment covariate.

The paired corridor diagnostic JSON/PNG bundles are checksum-registered by
their local manifests. For seed 277 their hashes are
`3ffa02f7f6838ebccb2b5fc1a64705215038e4436fa98ac553fe183dcd57929b`
and `6038237e2e56b7b3d6f6d2eae1278fc0f1e1f29f1714d70d9765d0a537f1a3d6`;
for seed 283 they are
`94b6291e18610daced9371892c582919f1ba55ac7e9c33f349d9aa5d49ff0a7a`
and `f79c80a45cbbd4e5e93130e249772bfef9fb1243d4ce5b11cded3a056ed53e04`.

## Visual evidence

The checksum-registered paired endpoint plot is
`paired_analysis/paired_endpoints.png` (539,713 bytes, SHA-256
`b8cb65bdf3ed165df565ece4dd33a89937f44fbd86362cebbf8f4f864a766d29`).
It shows every one of the eight paired lines and was visually checked; it does
not hide the corridor travel, ATE or technical-failure outcomes. The underlying
paired CSV and JSON hashes are
`b5df7fe5e0a631c6a755eb79640bdba237f6ef9cfab03b6f417bea7f2545c693`
and `11bfacdbbb8388baa17ea667cad4c82a8fc5cc1fadbe94d2ba6792d23ab1b290`.

Each of the 16 raw run media manifests contains and hashes a final-state image,
a 360-beam sensor-sanity image and a full task-camera depth video. All 48 raw
media hashes and sizes were independently verified. The four-family depth
contact sheet under `media_checks/` was visually checked and has SHA-256
`534d8460bddab9c53e67900e7e8ebe944baa1d7297713fc15df758cbcd0c040d`.
Because this batch is headless, all media manifests honestly retain
Gazebo-overview and RViz-navigation as missing roles; a separate GUI showcase
must supply them.

## Gate decision

This study validates the 16-run artifact path, runtime-adapter-specific hidden
action recording, strict paired analysis and per-pair localization diagnostics.
It also demonstrates why family/seed-stratified lines and medians must accompany
means: corridor localization and stopping outcomes dominate several averages.

No method is ranked. The evidence remains development-only with two seeds,
four individual development worlds and no inferential test. The next gates are
to investigate the retained corridor SLAM corrections and SSTG Nav2 abort,
collect additional prospectively scheduled seeds and scenes, complete the GUI
media pass, and only then freeze a separate test-split formal study.
