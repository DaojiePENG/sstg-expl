# SSTG 原生终止与失败恢复 Gazebo 验证

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: validate
- Origin Date: 2026-08-02
- Verification Status: ANALYZED
- Version Label: sstg_native_stop_v3_validation_v1
- Source: `gazebo_unknown_completion_sstg_native_stop_v3_20260802`
- Overall Confidence: CAUTION

本轮只验证 SSTG 的 ROS2/Gazebo 工程改进，不是新的五方法排名。场景、起点和 seed 与冻结 v15 的 SSTG 相同，但代码、参数和 ROS/Gazebo 异步轨迹已经改变，因此两轮差值只能作为 post-study 工程诊断，不能当作配对效应量。

## 1. 一句话结论

SSTG v3 以 **26 次执行、144.85 m、AUC 0.896** 首次达到 evaluator-only 的严格 95/95，并依靠 belief-only 的前沿—拓扑收敛条件在 160.94 m 自主结束；相比冻结 v15，95/95 之后的无效扫尾由 142.06 m 缩短到 **16.09 m**，同时发生的 1 次 Nav2 abort 被空间失败记忆隔离，后续 14 次执行全部成功，全程 0 碰撞。

这轮支持“少动作、高覆盖过程效率、可恢复且不会无限扫尾”的 SSTG 论点；它不支持“最短路径”或“导航失败数优于所有基线”的论点。

## 2. 核心 unknown-completion 端点

核心端点仍是 evaluator 首次观测到 `Ci >= 0.95` 且 `Ct >= 0.95`。隐藏真值只进入 evaluator 和离线图，不反馈给 SSTG，也不触发机器人停止。

| 指标 | SSTG v3 |
|---|---:|
| 首次严格 95/95 距离 | 144.85 m |
| 到 95/95 的决策 / 执行 | 26 / 26 |
| 到 95/95 的成功 / 失败 | 25 / 1 |
| Ci / Ct | 0.999 / 0.958 |
| 覆盖—距离 AUC@95/95 | **0.896** |
| 唯一端点 / 冗余率 | 25 / 0.000 |

冻结 v15 的公平五方法 screen 中，SSTG 同样以 26 次执行与 RRT 并列最少，AUC 0.877 为五方法最高；但其 123.94 m 距离只排第 4。v3 保留了动作效率与覆盖过程效率这一主证据线，但核心距离没有改善，不能把 native-stop 改动解释成最短路优化。

## 3. 现实可用的原生终止

SSTG 不读取全局真实覆盖率。原生停止规则为：

1. 在线 belief map 中不存在可达且仍有信息增益的前沿；
2. belief 拓扑覆盖达到 `0.95 + resolution / topological_radius = 0.975`；
3. 上述状态在 3 个新的 SLAM map revision 中连续成立。

本轮在 map revision 598、599、601 完成 1/3、2/3、3/3 确认，触发器为 `sstg_frontier_topology_convergence`。终止时 belief 拓扑覆盖为 0.977，evaluator 的 Ci/Ct 为 0.999/0.976；全程 32 次决策、29 次执行、28 成功/1 失败、160.94 m。核心端点之后只增加 16.09 m，说明机器人既没有依赖隐藏真值停机，也没有在封闭空间无限搜索。

## 4. 失败恢复与安全证据

第 15 个目标 `[1.978, 9.542]` 被 Nav2 以 status 6 终止。机器人实际已到达 `[1.937, 9.523]`，位置误差约 0.045 m，失败来自终端朝向而不是碰撞。该目标被写入五方法共享的 0.80 m 空间失败记忆；后续候选中共有 18 个邻近点被剪枝，之后 14 次执行全部成功，没有形成重复撞墙或局部卡死。

共享 ROS 执行层的端点最小 belief clearance 已从 0.40 m 提高到 0.60 m；它不读取真值 clearance、ATE 或碰撞标签。真值 evaluator 观测到：

- 碰撞 0，非地面有效接触 0；
- 最小静态 footprint clearance 0.208 m；
- ATE RMSE 0.045 m，最大 ATE 0.090 m；
- 最大相邻 `map -> odom` 平移修正 0.170 m；
- 最终 `/scan` 为 360/360 个有效返回。

因此本轮能支持 SSTG 的“系统可执行性与失败恢复”论点。要声称比较安全性优势，仍需在同一当前栈上对五方法做预注册的多场景、多 seed 配对试验。

## 5. 与冻结 v15 SSTG 的变化

| 指标 | 冻结 v15 SSTG | native-stop v3 | 解释边界 |
|---|---:|---:|---|
| 95/95 距离 | **123.94 m** | 144.85 m | v3 未改善最短路 |
| 95/95 执行数 | 26 | 26 | 少动作特征保留 |
| AUC@95/95 | 0.877 | **0.896** | 单轮描述性提升 |
| 全程距离 | 266.01 m | **160.94 m** | 现实终止明显更紧凑 |
| 95/95 后扫尾 | 142.06 m | **16.09 m** | 主要工程改进 |
| 成功 / 失败 | 36 / 3 | 28 / 1 | 不同异步轨迹，非因果效应 |
| 最终 Ci / Ct | 1.000 / 0.999 | 0.999 / 0.976 | v3 在满足目标后及时结束 |
| 碰撞 | 0 | 0 | 两轮均无碰撞 |

v15 保持为当前唯一冻结的公平五方法结果；本轮不回写其排名。论文主张应把 SSTG 的重点放在动作数、覆盖—距离 AUC、有效自主终止和故障恢复，而不是挑选对 SSTG 有利的不同运行来与旧基线混比。

## 6. 两次未采用的开发试验

- v1 直接用 belief 0.95 终止，在 25 次执行、115.33 m 时结束，但 evaluator Ct 只有 0.948，未达到严格 95/95。该规则会受 belief 离散化偏差影响，已拒绝。
- v2 加入 0.975 margin，但仍使用 0.40 m 端点 clearance；机器人在边界目标附近发生 16 次连续/重复失败，最终 Ct 0.901。它还暴露出“所有候选被失败记忆剪光”曾被误报为普通候选耗尽，已改为 fail-closed 的 `navigation_failure_exhaustion`。该运行已拒绝。

两轮失败试验只保留在本节的拒绝记录，其重复 study/output 目录已删除，避免被误当作有效结果。

## 7. 可检查证据

- [编号终态图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_native_stop_v3_20260802/gazebo_unknown_completion_sstg_native_stop_v3_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/unknown_completion/numbered_final_state.png)
- [覆盖演化图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_native_stop_v3_20260802/gazebo_unknown_completion_sstg_native_stop_v3_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/unknown_completion/coverage_evolution.png)
- [逐点编号决策视频](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_native_stop_v3_20260802/gazebo_unknown_completion_sstg_native_stop_v3_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/unknown_completion/decision_sequence.mp4)
- [603.8 秒深度相机视频](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_native_stop_v3_20260802/gazebo_unknown_completion_sstg_native_stop_v3_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/raw/task_camera_depth.mp4)
- [最终雷达检查图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_native_stop_v3_20260802/gazebo_unknown_completion_sstg_native_stop_v3_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/media/raw/sensor_sanity.png)
- [定位诊断图](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_native_stop_v3_20260802/localization_diagnostics/localization_diagnostic.png)
- [运行审计清单](../../../../system_sim_outputs/unknown_completion/validation/gazebo_unknown_completion_sstg_native_stop_v3_20260802/gazebo_unknown_completion_sstg_native_stop_v3_20260802__dev_office_01__start_southwest__nominal__seed_311__order_01__sstg/run_launch_manifest.yaml)
- [冻结五方法比较](../gazebo_unknown_completion_office_v15_20260802/RESULTS.md)

所有图片和视频均从冻结 core MCAP 离线生成并登记为 development evidence。媒体清单明确缺少实时 Gazebo overview 与 RViz 截屏，因此不冒充真机或正式论文证据。

## 8. 验证与推断边界

- 运行状态为 `terminal_completed`，进程和 supervisor 返回 0，artifact audit 有效；core bag 含 594,311 条消息。
- 关键哈希：`evaluation_metrics.jsonl` 为 `7d35302c...`，`policy_trace.jsonl` 为 `261cd756...`，run launch manifest 为 `cace0b22...`。
- 回归验证：仓库 263/263、`sstg_policy_ros` 28/28 通过；ROS build 与 clean FastDDS runtime preflight 通过。
- 单房间、单起点、单 seed，且是在查看 v15 后形成的 post-study 验证；不能用于 p 值、置信区间、效应量或总体排名。
- Reproducibility verdict：CANNOT_VERIFY。TRO 统计结论仍需预注册的多房间、多起点、多 seed 配对重复，基线只使用与纯算法阶段一一对应的公平 ROS 适配。
