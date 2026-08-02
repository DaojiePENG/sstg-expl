# Gazebo and RViz development showcase v2

This is the display-only replacement for
`gazebo_stage1_gui_showcase_20260802`.  In the first study, the quantitative
runner passed preflight and the simulation reached its natural terminal event,
but an X11 screenshot error terminated the shell that owned the runner before
it could write the final audit.  That raw attempt is retained as invalid and is
not repaired, analyzed, registered, or silently reused.

## Frozen scope

- World: development office `dev_office_01`, first registered start.
- Method: SSTG, nominal condition, new seed 311.
- Budget: 180 s, 25 m trace distance, 100 decisions and 90 s per goal.
- Quantitative eligibility: false; display/evidence role only.
- Raw-output root:
  `system_sim_outputs/runs/gazebo_stage1_gui_showcase_v2_20260802/`.
- Report root:
  `system_sim_outputs/reports/gazebo_stage1_gui_showcase_v2_20260802/`.

The schedule SHA-256 is
`bd7d82db42f76561addf496eab444408a3258c0e05f6d33600699e313517508d`.
It was frozen before simulator invocation.  It is separate from both the
16-run quantitative population and the failed v1 showcase attempt.

## Failure-isolated capture contract

The scheduled runner remains the sole process that owns and supervises ROS
launch.  Gazebo GUI, RViz and capture commands execute independently; their
failure cannot terminate or orphan the runner.  Both clients are read-only and
attach to the runner's Gazebo partition and ROS domain.  Capture uses the full
1600 x 900 nested X11 display because direct window import was unreliable in
the v1 Xephyr session.

The required registered roles remain actual `gazebo_overview`, actual
`rviz_navigation`, offline `sensor_sanity`, offline `final_state`, and an
unedited Gazebo `key_interval_video`.  Missing GUI evidence remains missing;
offline plots are never relabeled as screenshots.
