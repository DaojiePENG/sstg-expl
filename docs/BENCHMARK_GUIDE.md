# Benchmark 使用、基线出处与结果解释

## 1. 环境准备

```bash
conda env create -f environment.yml
conda activate sstg-explorer
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

完整 profile 包含公开预训练的学习基线。首次运行：

```bash
python scripts/setup_learning_baselines.py --install-dependencies
```

该脚本安装 PyTorch/gdown，下载 Active Neural SLAM 作者发布的 global-policy checkpoint 到忽略提交的 `models/checkpoints/`，并校验：

```text
sha256 616fd1485e1f0ba9673db08340d586c050f001f171890d966809c0b9f0320314
```

## 2. 一条命令的完整流程

```bash
# 两方法、两环境的链路检查
python scripts/run_benchmark.py --profile smoke

# 六方法 × 九环境 × 五次
python scripts/run_benchmark.py --profile full

# Full SSTG 与五个模块消融
python scripts/run_benchmark.py --profile ablation --no-frames
```

限定范围：

```bash
python scripts/run_benchmark.py --runs 3 \
  --algorithms frontier active_neural_slam sstg_explorer \
  --environments maze warehouse
```

默认每次 run 都保存逐决策媒体。磁盘紧张时使用 `--media-runs representative`；只做数值调试用 `--no-frames`。正式投稿结果不得使用 `--no-frames`，因为逐步 trace 是方法可审计性证据。

## 3. 对比方法与论文出处

| CLI 名称 | 网页名 | 本仓库实现/适配 | 主要出处 | 公平性说明 |
|---|---|---|---|---|
| `uniform_grid` | Uniform Grid | 固定格心、障碍处投影到最近安全栅格、最近邻访问 | Choset, 2001；Galceran and Carreras, 2013 | 已知地图 coverage baseline |
| `rrt` | RRT | 安全栅格随机树采样；新增覆盖节点由共同 A* 执行 | LaValle, 1998；Umari and Mukhopadhyay, 2017 | 报告机器人实际执行距离；不是原 ROS multi-RRT 完整复现 |
| `frontier` | Frontier | 栅格 frontier 聚类，按可达测地距离选择 | Yamauchi, 1997 | 同地图、同起点、同 A*；最小 cluster 为 1 cell，避免窄门漏检 |
| `nbv` | NBV | 信息增益减去 A* 旅行代价 | Connolly, 1985 | 同安全候选、每步 50 候选、同 A* |
| `active_neural_slam` | ANS-Global (adapted) | 发布 checkpoint 的 global policy + 共同 A* | Chaplot et al., ICLR 2020 | 只比较 learned global goal；不含 RGB mapper/local policy |
| `sstg_explorer` | SSTG-Explorer | 本文完整方法 | 本项目 | 被评估方法 |

BibTeX 在 `docs/REFERENCES.bib`。代码相关的学习方法还有 DRL-Graph（Chen et al., IROS 2020）和 Exploring Exploration（Ramakrishnan et al., ECCV 2020），但它们分别需要 belief-state/GTSAM 或 Habitat RGB 任务，不能无说明地并入已知栅格主表。

## 4. 环境协议

九环境均为 0.05 m 栅格，默认 `r_view=2.0 m`、`r_robot=0.3 m`、`d_safe=0.2 m`、目标覆盖率 95%。所有方法在同一按 0.5 m 膨胀的安全栅格上执行 A*；距离不再使用可能穿墙的视点直线和，也不把 RRT 树边总长当机器人实际路程。

SSTG 正式参数为 `d_theta=30°`、`beta=1.0`、`clearance_priority_weight=2.0`；最后一项只影响可行候选排序，不能替代所有方法共同遵守的 0.5 m 硬安全约束。完整参数逐项写入 `manifest.json`。

| 环境 | 主要结构 | 作用 |
|---|---|---|
| empty | 开放房间 | 基本均匀覆盖与路径冗余 |
| sparse_obstacles | 少量随机矩形 | 一般避障 |
| corridor | 长走廊 | 单主方向 |
| multiple_rooms | 三房间与门洞 | 跨墙测地排序和 A* 搜索预算 |
| l_shaped_corridor | L 形通道 | 转角传播 |
| maze | 多墙迷宫 | 长程可达性 |
| dense_obstacles | 15 个密集障碍 | 起点安全、局部候选存活 |
| narrow_passages | 1.2 m 门洞与多分区 | 膨胀连通性、固定/自适应角采样消融 |
| warehouse | 货架结构 | 重复走廊和全局选择 |

dense 起点按障碍矩形到起点的真实最近距离保护，不能只按障碍中心保护。narrow 的门洞与横墙交点错开，保证在统一机器人膨胀模型下仍连通。环境生成错误不能包装成算法失败。

## 5. 输出目录

```text
outputs/benchmark_runs/<timestamp>/
├── manifest.json                    命令、参数、Git、实验源码 hash、依赖/硬件、checkpoint
├── run.log                          完整运行日志
├── results.json                     所有数值、trajectory 和 decision steps
├── summary.csv                      方法–环境 mean/std/95% CI
├── aggregate.csv                    跨环境汇总
├── pairwise_vs_sstg.csv             以环境为 cluster 的 bootstrap 差值/CI
├── results_table.{md,tex}           论文表格草稿
├── coverage_heatmap.png
├── coverage_distance_tradeoff.png
├── safety_comparison.png
├── safety_table.{csv,md,tex}
├── index.html
└── artifacts/<env>/<algorithm>/
    ├── run.json
    ├── steps/step_XXXX.png
    ├── final.png
    ├── animation.gif
    ├── video.mp4
    └── runs/run_XXX/                 其余 seeds 的同套产物
```

`outputs/benchmark_runs/latest` 指向最近结果。通过 HTTP 查看：

```bash
python -m http.server 8000 --directory outputs/benchmark_runs/latest
```

访问 `http://127.0.0.1:8000/`。

## 6. SSTG 单步 trace 字段

每个 `steps[i]` 是一次完整决策状态：

- `event`：initialization、node_accepted、unreachable、global_recovery 等；
- `explored_nodes`：截至该决策的全部已接受视点；
- `generated_candidates`：本步每个候选，而不只是成功候选；
- `new_frontiers`：本步插入队列的候选；
- `active_frontiers`：刷新优先级后的待探索队列；
- `selected_frontier`：本步选择及其 ID、类型、优先级；
- `path`：实际 A* 折线路径；
- `executed_paths`：截至该 step 的全部实际 A* 轨迹段，而非视点直线连线；
- `coverage_before/after/gain`：本步边际覆盖；
- `recovery_round` 和 `queue_size`。

候选状态包括：

| 状态 | 含义 | 图中标记 |
|---|---|---|
| `added` / `added_soft` | 新加入 global queue | 绿色菱形 |
| `blocked_obstacle` | 机器人膨胀后碰撞 | 红色 × |
| `pruned_strength` | 新颖性不足 | 橙色 × |
| `pruned_priority` | 效用低于阈值 | 紫色 × |
| `pruned_duplicate` | 与 pending target 重复 | 灰色 × |
| `recovery_added` | 全局缺口候选 | 青色 P |
| `recovery_unreachable` | 缺口候选不可达 | 黑色 X |

逐步图还显示蓝色已探索轨迹、黄色 pending frontiers、粉色 selected frontier、绿色 current pose 和青色 A* path。它不再只是覆盖圆动画。

## 7. 指标与统计

- `coverage_ratio`：圆盘覆盖 proxy，越高越好；不是遮挡感知视觉覆盖。
- `total_distance`：A* 折线路径累计，而非节点欧氏直线和。
- `coverage_efficiency`：coverage/distance。
- `avg/min_obstacle_distance`：节点到障碍物的安全距离。
- `avg/min_boundary_distance`：节点到地图矩形边界的距离。
- `node_safe_fraction`：满足 (D_O\ge r_{robot}+d_{safe}=0.5\,m) 的视点比例。
- `avg/min_path_obstacle_distance`、`path_safe_fraction`：以 0.05 m 间隔采样全部实际路径得到的安全性。
- `mean_nn_distance`、`dispersion_uniformity`：节点间距与均匀性。
- `success_rate`：达到目标覆盖的 run 比例。
- `num_generated/rejected/recovery_candidates`：只用于解释 SSTG 决策，不作跨方法独立样本检验。

每个方法–环境报告 mean ± std 和 95% CI。随机方法用 `base_seed + run_id`；确定性方法出现零方差是预期现象。`pairwise_vs_sstg.csv` 先对每个环境内的 seeds 求均值，再以九个环境为 cluster 做 10,000 次 bootstrap，并对环境级差值做配对 Wilcoxon；五个 baseline 的 coverage、distance、平均视点净空和最小路径净空检验分别进行 Holm 校正。候选、视点或路径采样点不被伪装成独立样本。

## 8. 结果核验清单

1. `results.json` 条数等于方法数 × 环境数 × runs。
2. run 0 及所有 `runs/run_XXX` 均有非空 JSON、GIF、MP4；图和视频包含实际 A* 轨迹，不用视点直线代替。
3. 每个 SSTG trace 至少包含 initialization 和 accepted/rejected decision。
4. `run.log` 无 Traceback/FAILED。
5. HTML 的全部 `src`/`href` 存在。
6. `manifest.json` 的 Git dirty 状态在最终论文实验中应为空。
7. checkpoint hash 与本指南一致。
8. 报告所有环境，包括失败环境；不挑选获胜子集。

## 9. 本仓库已冻结结果

- 主实验：`outputs/benchmark_runs/20260719_043528/`，270/270 records，6,622/6,622 step PNG，270 个非空 GIF、270 个非空 MP4，零媒体错误，HTML 334 个相对引用全部存在。
- 受控消融：`outputs/ablation_runs/20260719_043544/`，315/315 records；按设计使用 `--no-frames`，避免为参数变体复制约 1 GB 媒体。
- 两个 manifest 的 experiment-source SHA-256 都是 `efa1828c62bac7ca0f33849449e1363a3ff14bd0079a5e9668764b8ade7e8642`。
- SSTG 主结果为 coverage `98.52 ± 1.27%`、实际路径 `48.13 ± 23.49 m`、平均视点净空 `1.160 m`、100% success。
- SSTG 相对 Frontier 的 coverage cluster 差为 `+1.61 pp [0.90, 2.45]`，Holm `p=0.0234`；路径差 `+2.85 m [-2.80, 8.92]` 不显著。
- 去掉净空效用使视点净空下降 `0.140 m [0.074, 0.213]`，Holm `p=0.0469`；关闭 recovery 后 success 降至 88.9%。

早期目录（包括 `20260719_025630`、`040409` 和 `040955`）使用过旧直线路径、adaptive 候选或未最终选定的协议，仅用于开发审计，不可与上述最终表拼接。
