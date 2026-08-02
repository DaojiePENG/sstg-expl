# ROS2/Gazebo unknown-completion v15 结果

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: validate
- Origin Date: 2026-08-02
- Verification Status: ANALYZED
- Version Label: unknown_completion_v15_validation_v1
- Source: `gazebo_unknown_completion_office_v15_20260802`
- Overall Confidence: CAUTION

这里只能得出单个 development 房间、单个起点、单个 seed 的工程结论，不能当作论文统计结论。五个策略只读取在线 SLAM belief map、估计位姿和 Nav2 结果；隐藏真值只进入 evaluator 与离线绘图。

## 1. 一句话结论

SSTG 的 ROS2 复现保留了纯算法阶段的“少动作、高覆盖过程效率”优势，但没有获得最短路，也还没有获得最佳导航可靠性：它以 26 次执行达到统一 95/95，与 RRT 并列最少，覆盖—距离 AUC 0.877 为五方法最高；但距离 123.94 m 仅排第 4，且全程有 3 次相邻边缘目标失败。NBV 以 92.23 m 最短，Frontier 与 NBV 全程无导航失败。

## 2. 统一 95/95 核心端点

主端点是 evaluator 首次观测到 `Ci >= 0.95` 且 `Ct >= 0.95` 的样本。该端点从不反馈给策略，也不负责终止机器人。

| 方法 | 达到 95/95 | 距离 / m | 执行次数 | 覆盖—距离 AUC | 唯一端点观测 |
|---|---:|---:|---:|---:|---:|
| NBV | 是 | 92.23 | 37 | 0.818 | 35 |
| Frontier | 是 | 97.48 | 38 | 0.743 | 39 |
| RRT | 是 | 113.37 | 26 | 0.858 | 26 |
| SSTG | 是 | 123.94 | 26 | **0.877** | **24** |
| ANS | 是 | 128.65 | 29 | 0.861 | 27 |

解释：SSTG 不是靠更短的单步移动获胜，而是用更少的定向视点维持更高的覆盖—距离曲线。五方法在该 ROS 场景的端点冗余率都为 0，因此纯算法阶段 SSTG 的低冗余优势在这个单场景里没有区分度。

## 3. 各方法自己的现实终止

五方法均以自己的 belief 候选耗尽结束，并要求连续 3 个新的 SLAM map revision 都没有有效候选。真值覆盖率不参与终止。

| 方法 | 原生终止确认 | 全程距离 / m | 决策 / 执行 | 导航成功 / 失败 | 最终 Ci / Ct | 碰撞 |
|---|---:|---:|---:|---:|---:|---:|
| Frontier | 3/3 | 116.87 | 46 / 43 | 43 / 0 | 0.998 / 0.999 | 0 |
| SSTG | 3/3 | 266.01 | 42 / 39 | 36 / 3 | 1.000 / 0.999 | 0 |
| RRT | 3/3 | 224.44 | 41 / 38 | 37 / 1 | 0.999 / 1.000 | 0 |
| NBV | 3/3 | 148.24 | 52 / 47 | 47 / 0 | 0.995 / 1.000 | 0 |
| ANS | 3/3 | 177.91 | 38 / 35 | 32 / 3 | 1.000 / 0.999 | 0 |

现实终止证明了机器人不会依赖未知的全局探索率，也不会无限搜索；但 SSTG 从统一端点 123.94 m 扫尾到 266.01 m，说明它的 native 小增益候选耗尽条件仍过于保守。该扫尾距离不应拿来替代核心 95/95 排名。

## 4. 失败与定位跳变诊断

所有方法的真值接触计数均为 0。SSTG 和 ANS 各有 3 次、RRT 有 1 次 Nav2 失败；失败目标集中在 belief clearance 恰好为 0.40 m 的边界视点，换到其他区域后均恢复，没有演化成机器人卡死或连续全局失败。下一步应在五方法共享执行层加入“失败目标空间邻域屏蔽”，不能给 SSTG 单独加真值或 ATE 安全过滤。

Post-study follow-up：共享的 0.80 m 失败目标邻域记忆已在 `ad8874d` 实现，纯算法默认仍为 0，ROS/真机适配层五方法统一启用。它没有回写或重算本页冻结结果；独立验证见 [SSTG failure-memory probe](../gazebo_unknown_completion_sstg_failure_memory_probe_20260802/RESULTS.md)。

定位只作描述性诊断，不参与核心排名。五方法最终 ATE RMSE 为 0.041--0.090 m，最大 ATE 为 0.105--0.200 m，最大相邻 `map -> odom` 平移修正为 0.171--0.215 m。当前轨迹中未观察到此前那种破坏导航连续性的灾难性跳变，但没有事后设置通过阈值。

## 5. 与纯算法 unknown-completion 的对应关系

- 对应：五方法仍是 Frontier、NBV、RRT、ANS、SSTG；输入仍是未知 belief；统一比较端点仍是 evaluator-only 95/95；SSTG 的优势仍主要体现为动作数和覆盖过程效率，而不是最短距离。
- 不对应：Gazebo 中加入了 SLAM、Nav2、激光噪声、机器人动力学和目标可执行性，因此出现纯算法网格路径没有的 Nav2 失败与扫尾开销。
- 当前判断：核心优势得到“部分复现”，不是“全面胜出”。需要多房间、多起点、多 seed 后才能判断优势是否稳定。

## 6. 开源复用与真机替换边界

仿真复用了上游 TurtleBot3 模型、Gazebo Harmonic、SLAM Toolbox、Nav2 Regulated Pure Pursuit、Nav2 行为树以及已有 ANS/PyTorch 环境；没有自行重写底盘、SLAM 或局部控制器。核心 screen 只保留 Nav2 RPP 的单个局部碰撞投影门，外部 Collision Monitor 不参与核心算法排序。

真机入口为 [`unknown_completion_robot.launch.py`](../../../../ros2_ws/src/sstg_nav_bringup/launch/unknown_completion_robot.launch.py) 和 [`unknown_completion_robot_interface.yaml`](../../configs/unknown_completion_robot_interface.yaml)。替换 `/scan`、`/imu`、`/odom`、`/joint_states`、TF 与 `/cmd_vel`，并让 SLAM/Nav2 提供 `/map` 和 `NavigateToPose`，策略代码不需要改。真实底盘接入时必须同步更新 Nav2 footprint 与策略 `robot_radius_m`。

## 7. 可检查证据

- [集中结论](../../../../system_sim_outputs/unknown_completion/reports/gazebo_unknown_completion_office_v15_20260802/CONCLUSION.md)
- [汇总 CSV](../../../../system_sim_outputs/unknown_completion/reports/gazebo_unknown_completion_office_v15_20260802/summary.csv)
- [统一对比图](../../../../system_sim_outputs/unknown_completion/reports/gazebo_unknown_completion_office_v15_20260802/procedural_equivalent_comparison.png)
- [定位诊断图](../../../../system_sim_outputs/unknown_completion/reports/gazebo_unknown_completion_office_v15_20260802/localization_diagnostics/localization_diagnostic.png)
- [SSTG 编号终态图](../../../../system_sim_outputs/unknown_completion/runs/gazebo_unknown_completion_office_v15_20260802/gazebo_unknown_completion_office_v15_20260802__dev_office_01__start_southwest__nominal__seed_311__order_02__sstg/media/unknown_completion/numbered_final_state.png)
- [SSTG 覆盖演化图](../../../../system_sim_outputs/unknown_completion/runs/gazebo_unknown_completion_office_v15_20260802/gazebo_unknown_completion_office_v15_20260802__dev_office_01__start_southwest__nominal__seed_311__order_02__sstg/media/unknown_completion/coverage_evolution.png)
- [SSTG 编号决策视频](../../../../system_sim_outputs/unknown_completion/runs/gazebo_unknown_completion_office_v15_20260802/gazebo_unknown_completion_office_v15_20260802__dev_office_01__start_southwest__nominal__seed_311__order_02__sstg/media/unknown_completion/decision_sequence.mp4)
- [SSTG 原始深度视频](../../../../system_sim_outputs/unknown_completion/runs/gazebo_unknown_completion_office_v15_20260802/gazebo_unknown_completion_office_v15_20260802__dev_office_01__start_southwest__nominal__seed_311__order_02__sstg/media/raw/task_camera_depth.mp4)
- [SSTG 激光检查图](../../../../system_sim_outputs/unknown_completion/runs/gazebo_unknown_completion_office_v15_20260802/gazebo_unknown_completion_office_v15_20260802__dev_office_01__start_southwest__nominal__seed_311__order_02__sstg/media/raw/sensor_sanity.png)

其余四个方法具有相同的 `media/unknown_completion/` 文件结构。SSTG 的离线媒体登记明确缺少实时 Gazebo overview 与 RViz 截屏，因此这些文件是 core MCAP 派生的 development evidence，不冒充真机或正式论文证据。

## 8. 验证与推断边界

- 五份 run 均为 `terminal_completed`，artifact audit 有效，policy/evaluator trace 与 core MCAP 哈希已登记。
- 统计内容扫描：没有 p 值、置信区间或效应量；所有排名都是描述性单样本结果。
- Fallacy scan：11/11 已检查。Simpson、生态、Berkson、collider、base-rate、均值回归、幸存者偏差、相关因果、反向因果在当前描述性单场景比较中不适用；look-elsewhere 与 garden-of-forking-paths 为 CAUTION，因为控制参数在同一 development 场景上迭代过。
- ANS 首次启动因隔离 PyTorch 前缀未加入环境而在产生任何策略决策前失败；无效目录已删除，补跑使用同一冻结 schedule/seed。该事件不属于性能样本，但在工程日志中已透明记录。
- Reproducibility verdict：CANNOT_VERIFY。ROS/Gazebo 异步执行属于环境敏感的随机实验，当前没有独立 seed/重复 run；下一阶段至少需要多房间、多起点、多 seed 配对重复。
