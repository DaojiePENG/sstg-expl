# Gazebo and RViz development showcase design

This single run exists only to capture checkable Gazebo, RViz and screen-video
evidence from the already validated ROS 2 system-simulation stack.  It is not a
method comparison, is not part of the four-family quantitative population, and
must not be merged into its tables or effect summaries.

## Frozen scope

- World: development office `dev_office_01`, first registered start.
- Method: SSTG, nominal condition, seed 307.
- Budget: 180 s, 25 m trace distance, 100 decisions and 90 s per goal.
- Quantitative eligibility: false; display/evidence role only.
- Raw-output root:
  `system_sim_outputs/runs/gazebo_stage1_gui_showcase_20260802/`.
- Report root:
  `system_sim_outputs/reports/gazebo_stage1_gui_showcase_20260802/`.

The schedule SHA-256 is
`9f390c7008dcf417cc9afc4c177f6eb89e66da66a98959ed3009d846d2eb0921`.
It was generated from clean runtime source at repository commit
`ca33afb6510de019a176333f3710ce59eb01d494` before the simulator was invoked.

## Capture contract

The scheduled runner remains headless so its launch and artifact contract is
unchanged.  A Gazebo GUI client attaches to the same isolated Gazebo partition,
and RViz attaches to the same isolated ROS domain.  The two clients do not
publish navigation commands or change the policy.  Required media roles are:

1. `gazebo_overview`: robot and furnished office visible in Gazebo;
2. `rviz_navigation`: map, TF, LiDAR, plan and policy markers visible in RViz;
3. `sensor_sanity`: offline final LaserScan render from the core MCAP;
4. `final_state`: offline final map and registered truth-path render;
5. `key_interval_video`: unedited Gazebo GUI interval.

All captures remain development-simulation evidence.  If either GUI cannot be
rendered, the role remains missing; an offline plot must not be relabeled as a
GUI capture.
