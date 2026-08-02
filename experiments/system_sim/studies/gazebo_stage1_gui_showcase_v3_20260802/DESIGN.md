# Gazebo and RViz development showcase v3

This display-only study replaces the unusable GUI evidence from
`gazebo_stage1_gui_showcase_v2_20260802`.  The v2 scheduled run itself
completed and passed its artifact audit, but the separately attached Gazebo
client showed only a loading cursor and RViz was connected after the ROS graph
had stopped.  Those captures are retained as failed attempts and must not be
registered or relabeled as valid screenshots.

## Frozen scope

- World: development office `dev_office_01`, first registered start.
- Method: SSTG, nominal condition, fresh seed 313.
- Budget: 240 s, 35 m trace distance, 100 decisions and 90 s per goal.
- Quantitative eligibility: false; display/evidence role only.
- Raw-output root:
  `system_sim_outputs/runs/gazebo_stage1_gui_showcase_v3_20260802/`.
- Report root:
  `system_sim_outputs/reports/gazebo_stage1_gui_showcase_v3_20260802/`.

The schedule SHA-256 is
`3b0eeb4183bd52b0d4c1fa0ce8bf761eade2b11d8315606d0e797576a9bca5b1`.
It was frozen before simulator invocation and is disjoint from the 16-run
quantitative population and from both earlier showcase studies.

## Failure-isolated capture contract

The scheduled runner is the sole supervisor of ROS launch.  The nested X11
display, Gazebo GUI, RViz, and capture processes run independently and cannot
terminate or orphan the scheduled runner.

After the server starts, the capture process reads the live Gazebo process
environment from `/proc/<pid>/environ` and uses the observed `GZ_PARTITION`
and ROS domain rather than assuming them.  It captures the complete 1600 x 900
nested display.  A frame is accepted only after visual inspection confirms
that Gazebo contains the furnished office and robot, and that RViz contains
live map/navigation data.  Blank, loading, or disconnected frames remain
failed attempts and are not registered.

Required registered roles are actual `gazebo_overview`, actual
`rviz_navigation`, offline `sensor_sanity`, offline `final_state`, and an
unedited Gazebo `key_interval_video`.  Offline plots are never relabeled as
GUI screenshots.
