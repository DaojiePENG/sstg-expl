# SSTG 失败邻域记忆 post-study 验证

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: validate
- Origin Date: 2026-08-02
- Verification Status: ANALYZED
- Version Label: sstg_failure_memory_probe_v1
- Source: `gazebo_unknown_completion_sstg_failure_memory_probe_20260802`
- Overall Confidence: CAUTION

本轮是 v15 之后的单方法工程验证，不是新的五方法比较，也不更新 v15 的核心排名。场景、起点和 seed 与 v15 SSTG 相同，但 ROS/Gazebo/SLAM 异步执行不是逐位确定的重复实验。

## 1. 一句话结论

加入五方法共享的 0.80 m 失败目标邻域记忆后，SSTG 本轮以 38/38 次导航成功、0 碰撞和原生候选耗尽完整结束；但本轮没有产生 Nav2 失败，因此只能证明改动没有造成系统级回归，不能把“0 失败”因果归于邻域抑制。

## 2. 核心端点与现实终止

| 指标 | 本轮结果 |
|---|---:|
| 首次 evaluator-only 95/95 距离 | 145.49 m |
| 到 95/95 的决策 / 执行 | 24 / 24 |
| AUC@95/95 | 0.868 |
| 原生终止确认 | 3 / 3 |
| 全程决策 / 执行 | 41 / 38 |
| 导航成功 / 失败 | 38 / 0 |
| 全程真值距离 | 281.61 m |
| 最终 Ci / Ct | 1.000 / 0.998 |
| 碰撞 / 非地面接触 | 0 / 0 |

v15 SSTG 的对应值是 123.94 m、26 次执行、AUC 0.877，以及全程 36/3 次成功/失败。本轮在动作数、距离和失败数上同时发生变化，说明单次异步重复存在明显波动；不能据此声称性能提升或下降，更不能替换五方法冻结结果。

## 3. 失败邻域机制是否被实际触发

- 本轮 38 个 Nav2 结果全部为成功，`failure_neighborhood_recorded=true` 为 0 次，`pruned_navigation_failure_neighborhood` 也为 0 个候选。
- 因此，Gazebo 本轮验证的是“启用该参数后可无失败完成”，不是“发生失败后邻域剪枝被触发”。
- 触发语义由确定性回归测试覆盖：Nav2 abort 或非 adapter-owned cancel 会记录失败邻域；预算取消、目标拒绝和通信错误不会污染空间记忆。
- 该适配对 Frontier、NBV、RRT、ANS、SSTG 使用同一个参数和代码路径，只读取 Nav2 结果，不读取真值覆盖率或 ATE。

## 4. 跳变、安全与审计

运行状态为 `terminal_completed`，supervisor 返回 0，artifact audit 有效，core MCAP 的 967,603 条消息已由 reader 验证。最终 ATE RMSE 为 0.030 m、最大 ATE 为 0.126 m；最大相邻 `map -> odom` 平移修正为 0.235 m。它没有伴随导航失败，但这里只作描述性诊断，不设置事后通过阈值。

真值接触 evaluator 报告 0 次碰撞；静态 footprint clearance 最小值为 0.182 m。最终 `/scan` 为 360/360 个有效返回。媒体均由冻结 core MCAP 离线生成，缺少实时 Gazebo overview 和 RViz 截屏，因此不冒充真机或正式证据。

## 5. 可检查证据

- [编号终态图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_failure_memory_probe_20260802/gazebo_unknown_completion_sstg_failure_memory_probe_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/unknown_completion/numbered_final_state.png)
- [覆盖演化图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_failure_memory_probe_20260802/gazebo_unknown_completion_sstg_failure_memory_probe_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/unknown_completion/coverage_evolution.png)
- [逐点编号决策视频](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_failure_memory_probe_20260802/gazebo_unknown_completion_sstg_failure_memory_probe_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/unknown_completion/decision_sequence.mp4)
- [980.2 秒深度相机视频](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_failure_memory_probe_20260802/gazebo_unknown_completion_sstg_failure_memory_probe_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/raw/task_camera_depth.mp4)
- [最终雷达检查图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_failure_memory_probe_20260802/gazebo_unknown_completion_sstg_failure_memory_probe_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/raw/sensor_sanity.png)
- [定位诊断图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_failure_memory_probe_20260802/localization_diagnostics/localization_diagnostic.png)
- [运行审计清单](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_failure_memory_probe_20260802/gazebo_unknown_completion_sstg_failure_memory_probe_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/run_launch_manifest.yaml)

## 6. 验证边界

- 代码提交：`ad8874d`（共享失败目标空间邻域抑制）。
- 测试：仓库 261/261 通过；`sstg_policy_ros` 28/28 通过；ROS build 和 clean FastDDS runtime preflight 均通过。
- 单房间、单起点、单 seed，且是看过 v15 后的 post-study probe；不能用于显著性、效应量或五方法排名。
- Reproducibility verdict：CANNOT_VERIFY。需要预注册的多房间、多起点、多 seed 配对重复。
