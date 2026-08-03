# Next-stage unknown-completion results

This is a development-only ROS2/Gazebo screen for SSTG-first inspection. The
four paired cells are identified by
`world_id/start_id/condition/replicate_seed`; recovery schedules fill a failed
parent cell but are not counted as additional scenes. The evaluator-only
endpoint is the first `Ci >= 0.95` and `Ct >= 0.95` crossing. Policies never
receive the evaluator truth and do not stop on this endpoint.

## Result snapshot

All 4 paired cells have one valid terminal run for each method (20 valid
terminal rows). Values below are descriptive means over those four cells.

| method | first 95/95 | distance to 95/95 (m) | executions to 95/95 | coverage-distance AUC | full distance (m) | nav failures |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| SSTG | 4/4 | 133.28 | 24.00 | 0.919 | 153.34 | 0.50 |
| Frontier | 4/4 | 81.25 | 31.75 | 0.786 | 90.52 | 0.00 |
| NBV | 3/4 | 93.30 | 31.00 | 0.871 | 170.98 | 1.75 |
| RRT | 4/4 | 122.06 | 26.50 | 0.903 | 230.09 | 2.00 |
| ANS | 4/4 | 112.78 | 24.50 | 0.891 | 153.13 | 0.50 |

SSTG is the primary result: it reached the evaluator threshold in all four
cells, used the fewest mean executions (24.0; ANS 24.5), and had the highest
mean coverage-distance AUC in this screen (0.919). Frontier travelled less
distance overall, but its AUC and decision efficiency were lower; therefore
the data do not support a blanket claim that SSTG minimizes distance. The
sample is too small and development-only for formal ranking, significance
testing, or generalization claims.

## Retained anomalies

Eight rows remain invalid/incomplete in `reports/run_audit.csv`: six original
parent rows (ANS missing the clean-interpreter PyTorch dependency, an
interrupted corridor RRT, a lab SSTG SLAM crash, and office launch/bridge
failures), one corridor ANS recovery overlap anomaly, and one intentionally
unrun lab RRT recovery row. They were not converted into successes. ANS
recovery rows use the preinstalled Anaconda PyTorch 2.13.0 via a temporary
workspace-only path hook; the hook was removed after the runs and no core code
was changed.

## Media and diagnostics

Every valid row has a separate output directory under
`system_sim_outputs/unknown_completion/next_stage/...`, with numbered final
state/decision frames, coverage evolution, decision-sequence MP4, sensor
sanity image, depth MP4, and a checksum manifest. SSTG also has localization
diagnostic figures (`reports/localization_main/` and
`reports/localization_lab/`). The media registration correctly reports the
two unavailable live-capture roles (`gazebo_overview`, `rviz_navigation`);
the recorded bag-derived artifacts are present.

The machine-readable tables and figure are in the sibling `reports/`
directory: `paired_blocks.csv`, `aggregate_by_method.csv`,
`run_audit.csv`, and `first95_efficiency.png`.
