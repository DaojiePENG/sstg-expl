# Four-world paired system-simulation analysis

`scripts/analyze_system_sim_paired.py` converts the run-level CSV emitted by
`analyze_system_sim_experiments.py` into a separate, descriptive paired report.
It is intentionally restricted to development simulation evidence and must not
be used as a confirmatory test.

## Contract

- Pair key: `world_id/start_id/condition/replicate_seed`.
- Methods: exactly one `sstg` and one `frontier_mrtsp_dp_external` row per key.
- Delta: `SSTG - frontier_mrtsp_dp_external`.
- Population: every CSV row; no run is filtered, excluded, or imputed.
- Input: exactly four worlds, one study, non-formal evidence, and only complete
  artifact-valid `terminal_completed` rows with a final
  `policy_session_settled` snapshot.
- Endpoints: information/topological coverage, target-recall proxy,
  ground-truth travel, mean/minimum/5th-percentile clearance, ATE
  mean/RMSE/maximum, collision count, and navigation technical-failure count.
- Interpretation: descriptive only. The tool performs no significance test,
  confidence interval, localization exclusion, or pass/fail threshold.

Missing endpoints, incomplete or duplicate pair members, an unexpected method,
an input hash disagreement, or a world count other than four aborts the entire
analysis. The tool verifies `system_sim_runs.csv` against its sibling
`analysis_manifest.json` before pairing.

## Run

Write paired artifacts beside, rather than inside, the analyzer's `analysis/`
directory:

```bash
/usr/bin/python3 scripts/analyze_system_sim_paired.py \
  system_sim_outputs/reports/<study_id>/analysis/system_sim_runs.csv \
  --output-dir system_sim_outputs/reports/<study_id>/paired_analysis
```

The output directory must not already exist. This fail-closed rule prevents an
old report from being partly overwritten.

The command creates:

- `paired_run_deltas.csv`: one row per frozen pair, with both method values and
  the run-level delta for every endpoint;
- `paired_run_deltas.json`: the same run-level evidence plus non-inferential
  mean/median/minimum/maximum delta summaries;
- `paired_endpoints.png`: a paired-line inspection figure colored by world;
- `paired_analysis_manifest.json` and its SHA-256 sidecar: input, tool, and
  output provenance.

These derived files belong under the simulation-specific
`system_sim_outputs/` tree, never the legacy benchmark `outputs/` directory.
