# Recovery rows for the next-stage all-method screen

This is a development-only recovery study for rows reserved by the first
screen before the ANS optional dependency was visible to the clean ROS
interpreter, plus the interrupted corridor RRT row. It uses the same worlds,
registered starts, nominal condition, seed 331, budgets, ROS/Gazebo stack and
unknown-completion evaluator as the parent screen. It does not overwrite the
failed directories and is joined to the parent screen only by
`world_id/start_id/condition/replicate_seed/method`.

The ANS rows use a temporary `ros2_ws/build/ans_torch_shim/sitecustomize.py`
path hook to expose the already-installed `/home/daojie/anaconda3` PyTorch
2.13.0 wheel to `/usr/bin/python3`. The hook is an environment-only recovery
aid; no source or paper files are changed. Its path and package version must
remain visible in the run manifest and final report.

Evidence tier: development. No formal ranking, confidence interval or
generalization claim is permitted. The original startup failures remain
retained as anomalies.
