# Lab SSTG recovery row

Development-only recovery of the lab SSTG paired cell after the parent run
ended with a recorded SLAM/artifact-validation failure. The failed parent
directory is retained; this fresh serial run uses the same registered world,
start, nominal condition, seed 331, budgets, ROS/Gazebo stack and unknown-
completion evaluator. It is joined to the parent by
`world_id/start_id/condition/replicate_seed/method`.

Evidence tier: development. The recovery does not alter core policy code or
the paper, and it does not turn the original failure into a success silently.
