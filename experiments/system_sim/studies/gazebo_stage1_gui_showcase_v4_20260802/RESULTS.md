# Gazebo and RViz development showcase v4 results

This study completed the visual-evidence pass for the ROS 2/Gazebo development
experiments.  It is display-only development evidence and is not a member of
the 16-run paired quantitative population, a formal test-split result, or
real-robot evidence.

Raw artifacts are isolated under
`system_sim_outputs/runs/gazebo_stage1_gui_showcase_v4_20260802/`; visual
checks are under the matching `system_sim_outputs/reports/` directory.  No
artifact is written to the legacy benchmark `outputs/` tree.

## Run audit

The frozen seed-317 office run reached `terminal_completed` in 172.146 s with
runner and launch return codes both zero.  Its artifact audit is valid and all
seven completion checks pass, including policy/evaluator settlement, clean
launch shutdown, and readable core bag.  The MCAP contains 150,639 messages
over 153.632 s; its 175,842,818-byte data file has SHA-256
`0d4c87201f37b3f625b58bb74c2bc5b3ccd6af41a044b44bb0b590f013dffa73`.

For provenance only, the final settled snapshot reports C-I 0.840246, C-T
0.489109, target-recall geometry proxy 1.0, ground-truth travel 25.427 m, ATE
RMSE 0.130 m, minimum footprint clearance 0.221 m, and zero collisions.  Nine
navigation goals succeed and the tenth is correctly attributed to the binding
25 m distance budget, with zero technical failures.  These values are not
added to or compared with the frozen quantitative batch.

## Registered visual evidence

The development media manifest is complete: all five minimum roles are
present, plus a separate robot-follow still.  Its SHA-256 is
`b7d62657dd239397d874451720a49f2bbf8ab3a756be051ea5636b670160bd4a`,
and its checksum file verifies successfully.

| Role | Source | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| Gazebo overview | live Gazebo 8 nested-X11 capture | 101,793 | `e6aa49c4e23855d94474606cb8137f0430af45f4fc600117bae315b32ded212b` |
| Gazebo robot follow | live Gazebo 8 nested-X11 capture | 99,006 | `05255011953658f785b1fdd49318fd86f6ba76356859f913ef14f7d5f0140163` |
| RViz navigation | live ROS-domain capture | 111,751 | `7de1109f005bf4f2e5fd2f01a553839186a1c79de792f182bdb2e9f1bc9e7619` |
| Sensor sanity | this run's core-MCAP offline render | 96,330 | `47964b4c67bcabf69c257362aed5df902cf25b5fed2f881c4deacd737afddcac` |
| Final state | this run's core-MCAP offline render | 154,149 | `4c276e0f76f163c6bf9900bdd405b99924cb998fe4e1f40304fa8261f75a770c` |
| 30 s key interval | live Gazebo 8 nested-X11 capture | 1,198,335 | `fb169912601d1232d014d15daf81cc03b0e8059694a04fef251e94f839be06b0` |

The video is H.264, 1000 x 846, 10 fps, 300 frames and exactly 30.0 s.  Frames
at 1, 15 and 29 s differ over nearly the full image, and the robot is visible
in the inspected middle frame.  The camera-follow service tracks the robot but
passes behind office walls in parts of the interval; this occlusion is retained
and disclosed rather than edited out.  The overview and RViz stills are the
preferred figure sources.  The RViz frame visibly contains the occupancy map,
LiDAR returns, robot pose, global plan, policy markers and active Nav2 feedback.
The final scan has 360/360 valid returns.

The four-panel inspection sheet under `media_checks/showcase_contact_sheet.png`
is 1542 x 1232, 1,258,572 bytes, and has SHA-256
`341da4410a40791d4149072f62303bd40b912a3c726d33712ae7ba37965918e5`.
It is a convenience copy only; the registered raw files and their manifest are
the auditable sources.

## Retained capture history

Earlier attempts remain visible instead of being repaired post hoc:

- v1 first failed middleware preflight without invoking the simulator; its
  second attempt reached a natural session end, but the screenshot shell also
  owned the runner and left an unfinalized `status: running` manifest.  That
  raw attempt is invalid and was never analyzed or registered.
- v2 fixed process ownership and produced a valid scheduled run, but the
  Gazebo capture contained only a loading cursor and RViz connected after the
  graph stopped.  Both frames were rejected by visual inspection.
- v3 confirmed the real Gazebo partition and produced a valid scheduled run,
  Gazebo still, and RViz MCAP replay, while exposing that forcing
  `LIBGL_ALWAYS_SOFTWARE=1` under Xephyr crashes OGRE2 and that the dynamic
  Gazebo recording began too late.  It remains a partial media attempt.
- v4 removed only that incompatible rendering override, kept the runner and
  clients failure-isolated, and completed the prospectively frozen five-role
  media set.

No earlier screenshot or video was relabeled as v4 evidence, and no failed run
was deleted or silently reused.
