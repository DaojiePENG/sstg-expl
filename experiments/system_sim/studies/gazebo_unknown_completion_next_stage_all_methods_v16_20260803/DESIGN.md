# SSTG 主线四场景全方法 unknown-completion screen

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run/plan
- Origin Date: 2026-08-03
- Verification Status: UNVERIFIED
- Version Label: next_stage_v16_design_v1
- Overall Confidence: CAUTION

## 1. 目的

本轮只推进 ROS2/Gazebo 类真机 unknown-completion 证据，不修改 `src/`、`ros2_ws/src/`、论文或已有 benchmark 结果。SSTG 是主观察对象；Frontier、NBV、RRT、ANS 只作为与纯算法阶段一一对应的同栈公平对照。

## 2. 冻结设计

- 场景：`dev_corridor_01`、`dev_lab_01`、`dev_office_01`、`dev_warehouse_01`。
- 起点：每个场景的第一个注册起点。
- 条件：`nominal`。
- replicate seed：331。
- 匹配键：`world_id/start_id/condition/replicate_seed`。
- 方法：Frontier、NBV、RRT、ANS、SSTG，共 20 个 run、4 个配对 block。
- 方法顺序：`sha256-key-sort/v1`，randomization seed 20260803；顺序在运行前冻结。
- 预算：1800 s、300 m、80 decisions、单目标 180 s。
- 输出根：`system_sim_outputs/unknown_completion/next_stage/gazebo_unknown_completion_next_stage_all_methods_v16_20260803/`。

## 3. 主要端点

首个 evaluator-only `Ci >= 0.95 && Ct >= 0.95` 是算法比较端点；真值不反馈给策略，也不触发停止。每个方法自己的 belief-only 终止、失败恢复和 fail-safe 仅作现实执行诊断。核心优先报告：到 95/95 的距离、执行次数、覆盖—距离 AUC、唯一端点数和终止后的扫尾距离。

碰撞、静态 clearance、ATE、Nav2 technical failure 不进入核心算法排序，只保留为安全和可执行性次级结果。任何预算命中或 `navigation_failure_exhaustion` 都标为不完整/失败，不当作成功。

## 4. 输出与监控

每个 run 必须有 `run_launch_manifest.yaml`、policy/evaluator trace、settled metrics 和 core MCAP。SSTG run 额外离线生成编号终态图、覆盖演化图、决策序列视频、雷达检查图、深度视频和定位诊断；基线保留相同媒体合同的最终状态与传感器证据。

运行期间只监控已声明的 run 输出、launch log 和进程状态：每 30 s 检查进程存活、核心文件增长和资源异常；硬超时按 runner 合同处理，其他异常只记录并停止自动重试。

## 5. 推断边界

这是四个 development world、单起点、单 seed 的工程筛查，不是 test split，也不产生 p 值、置信区间或总体排名。若某个场景发生 SLAM 跳变、导航连续失败或预算绑定，保留该样本并在总结中分层报告，不事后删除。
