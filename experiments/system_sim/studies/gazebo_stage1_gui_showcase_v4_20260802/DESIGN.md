# Gazebo and RViz development showcase v4

This display-only study is the final media-completion pass.  The v3 scheduled
run and its MCAP are valid and produced inspected Gazebo and RViz stills plus
an RViz replay video, but its Gazebo client was attached too late to record the
prospectively required dynamic Gazebo interval.  V3 remains a partial media
attempt and is not silently relabeled or substituted into this run.

## Frozen scope

- World: development office `dev_office_01`, first registered start.
- Method: SSTG, nominal condition, fresh seed 317.
- Budget: 180 s, 25 m trace distance, 100 decisions and 90 s per goal.
- Quantitative eligibility: false; display/evidence role only.
- Raw-output root:
  `system_sim_outputs/runs/gazebo_stage1_gui_showcase_v4_20260802/`.
- Report root:
  `system_sim_outputs/reports/gazebo_stage1_gui_showcase_v4_20260802/`.

The schedule SHA-256 is
`51936d08662d293c51351a3f8751fe7ba02db29d411e5869773ce7c62c3911ad`.
It was frozen before simulator invocation and is disjoint from the 16-run
quantitative population and all earlier showcase attempts.

## Fixed capture sequence

The scheduled runner remains the sole supervisor of ROS launch.  Read-only GUI
clients run in separate process sessions on a 1600 x 900 nested X11 display.
The live Gazebo process environment is checked before attachment.  The Gazebo
8 client uses the same observed partition and resource path, explicitly avoids
the Xephyr-incompatible `LIBGL_ALWAYS_SOFTWARE` override, and must expose a
real child window before capture.

The capture sequence is fixed as follows: save the furnished-world overview;
move the Gazebo camera to `turtlebot3_waffle`, enable camera follow, and record
one unedited 30 s interval; stop only the GUI client; launch RViz on the live
ROS domain and save a frame containing map/navigation data.  Every still and a
video frame must pass visual inspection.  Blank, loading, disconnected or
static placeholder images are failures and are never registered.

After the scheduled run passes its artifact audit, `sensor_sanity` and
`final_state` are rendered from its own MCAP.  The five minimum registered
roles are `gazebo_overview`, `rviz_navigation`, `sensor_sanity`, `final_state`
and `key_interval_video`.
