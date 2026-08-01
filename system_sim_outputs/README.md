# System-simulation outputs

All generated ROS 2/Gazebo experiment artifacts live under this dedicated
top-level directory. They are intentionally separate from the repository's
existing benchmark `outputs/` tree and are ignored by Git except for this
contract file.

```text
system_sim_outputs/
  preflight/                  host and runtime readiness records
  runs/<study_id>/<run_id>/   bags, logs, metrics, traces and captured media
  reports/<study_id>/         derived tables, figures and analysis manifests
```

Frozen schedules and configuration hashes remain version controlled under
`experiments/system_sim/studies/<study_id>/`. Do not put real-robot data in
this directory or describe Gazebo runs as real-robot evidence.
