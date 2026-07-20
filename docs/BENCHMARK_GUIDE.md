# SSTG-Explorer Benchmark 指南

本文档是已知地图、旧 sensor-only unknown 和新 joint unknown 三套 benchmark 的统一说明。安装、最小 API、主结果和网页启动方式见根目录 [README](../README.md)；本文只展开协议、公平性、字段、统计和结果核验。

## 1. 协议与信息边界

| 名称 | Map input | Policy 可读信息 | 终止 | Coverage |
|---|---|---|---|---|
| `known_static_disk` | 完整 truth | 完整静态 occupancy | 2 m disk target | ground-truth free cells 内的 disk proxy |
| `unknown_static_grid_occlusion_aware` | hidden truth | all-unknown 起步的三值 belief | sensor target | truth-free 中已正确观测为 free 的比例 |
| `unknown_static_grid_joint_topological_coverage` | hidden truth | 同上 | sensor 与 topology 都达到 target | 两类 coverage 分列，joint 为二者最小值 |

Unknown 协议的硬 invariant：

1. ground truth 只由 `RaycastSensor` 与 evaluator 持有；
2. 每束 ray 在第一个 occupied cell 处停止，墙体可见、墙后保持 unknown；
3. candidate、gain、known-free reachable set、utility、ANS global adapter 和 A* 都只读取 \(B_t\)；
4. A* 只经过完整 0.3 m footprint 已观测为 free 的中心栅格；
5. 机器人沿实际 A* 折线每 1 m 观测，并在目标 heading 补一帧；
6. truth overlay 仅出现在明确标为 evaluation-only 的右侧图。

Joint 协议额外从 belief 生成：

\[
\widehat{\mathcal U}_t^T=
(\mathfrak R_t\cap\{B_t=0\})\setminus
\bigcup_{v_i\in V_t}\mathcal D(p_i,2\text{ m}).
\]

它不读取“尚未被 sensor 发现的 free space”。因此在 sensor saturation 后只能补当前已知安全区域；若仍有未知区域，frontier 和 gap 候选共同存在。

## 2. 方法与公平性

### 2.1 Joint 主表

| CLI | 报告名 | 候选/选择语义 | 文献边界 |
|---|---|---|---|
| `frontier` | Frontier Joint | connected frontier representatives + common gap adapter；偏近目标 | Yamauchi 1997；不是任意厂商 stack |
| `nbv` | NBV Joint | frontier/random/gap candidates；joint gain–cost | Connolly 1985 的 NBV 原则 |
| `rrt` | RRT Joint (adapted) | random reachable samples + gap adapter；随机 gain/cost | Umari and Mukhopadhyay 2017 的 sampling 思路；非完整 multi-RRT 系统 |
| `ans` | ANS-Global Joint (adapted) | 发布的 learned global policy + common belief/gap/planner | Chaplot et al. 2020；不含 RGB mapper 和 learned local controller |
| `sstg` | SSTG-Explorer Joint | multi-frontier、topological vantages、gap FPS、rotations；joint utility | 本项目最终方法 |

所有方法共享：环境、起点、truth sensor、belief update、机器人尺寸、known-free planner、scan interval、2 m disk、0.25 m node merge、95% 双阈值、80 decisions 和 seeds。比较的是公共在线决策任务下的 policy adapters，不能写成对完整 ANS/RRT 系统的复现或全面 SOTA 排名。

### 2.2 已知地图参考

Known profile 还含 `uniform_grid`；它依赖全图格点，因此不进入 unknown 主表。已知版本的 ANS/RRT 同样明确标记 `adapted`。已知 coverage 与 unknown sensor coverage 不能混表；它只证明 disk 结构规划、clearance、recovery 和 pruning 模块的受控作用。

基线出处与 BibTeX 见 [REFERENCES.bib](REFERENCES.bib)。

## 3. 命令

```bash
conda activate sstg-explorer

# 快速验证
python scripts/run_unknown_benchmark.py \
  --profile smoke --coverage-objective joint

# 正式 joint 主表：810 runs
python scripts/run_unknown_benchmark.py \
  --profile paper --coverage-objective joint

# joint hard-scene ablation：180 runs
python scripts/run_unknown_benchmark.py \
  --profile ablation --coverage-objective joint --no-frames

# 统计扩展
python scripts/analyze_joint_benchmark.py \
  outputs/joint_benchmark_runs/latest

# 已知地图参考
python scripts/run_benchmark.py --profile full
python scripts/run_benchmark.py --profile ablation --no-frames
```

自定义示例：

```bash
python scripts/run_unknown_benchmark.py \
  --coverage-objective joint \
  --algorithms frontier rrt sstg \
  --environments multiple_rooms dense_obstacles warehouse \
  --sensors fov360_r12 fov90_r12 \
  --runs 3 --topological-radius 2.0
```

## 4. 每个 run 的输出

| 文件 | 内容 |
|---|---|
| `run.json` | metadata、全部 steps、节点、朝向动作、实际 trajectory、A* paths |
| `belief_final.npy` | 终止三值 belief |
| `decisions.csv` | 每步两类 coverage before/after/gain、动作、节点创建、路径与候选统计 |
| `candidates.csv` | 每个候选的 ID/type/pose/gains/cost/clearance/NN/priority/state |
| `trajectory.csv` | 拓扑节点/决策位置序列 |
| `oriented_views.csv` | 每个 heading action 及其 `topological_node_id` |
| `path_waypoints.csv` | 所有实际 A* 折线点 |
| `scan_poses.csv` | 连续路径扫描位姿 |
| `steps/*.png` | policy belief、truth audit、所有候选与轨迹的逐步图 |
| `animation.gif`, `video.mp4` | run 0 的完整过程 |

### 4.1 Candidate 关键字段

- `frontier_id`：稳定 ID；同一候选跨步骤保持不变；
- `kind`：`frontier`、`topological`、`coverage_gap`、`rotation`、`sampled`；
- `optimistic_gain` / `predicted_gain`：无射线预筛与 belief ray gain；
- `predicted_topological_gain`：2 m disk 与 policy-visible gap 的交集栅格数；
- `normalized_information_gain` / `normalized_topological_gain` / `normalized_task_gain`；
- `path_cost`：known-free geodesic cost；
- `clearance`：belief known-obstacle distance；
- `nearest_viewpoint_distance`：与历史空间节点最近距离；
- `priority`：策略最终分数；
- `status`：`active`、`selected`、`pruned_gain`、`pruned_evaluation_budget`、`pruned_executed` 等；
- `is_new`：本轮首次生成；
- `execution_key`：位置、朝向和类型签名，已执行签名不会重复选中。

### 4.2 Step 关键字段

- `sensor/topological/joint_coverage_before/after`；
- `coverage_before/after`：joint profile 中主显示为 topology；
- `selected_frontier`、`generated_candidates`、`active_frontiers`；
- `explored_nodes` 与 `oriented_views`：空间节点和动作分开；
- `topological_node_created`；
- `observed_updates`：`[flat_index, value]`，可精确重放 belief；
- `path`、`translation_m`、`rotation_deg`、`scan_poses`；
- `visible_cell_count`、`new_observed_count`。

## 5. 指标定义

### 5.1 Coverage

\[
C_t^I=\frac{|\{c:M^\star(c)=0\land B_t(c)=0\}|}{|\mathcal F^\star|},
\]

\[
C_t^T=\frac{|\{c\in\mathcal F^\star:\min_i\|x_c-p_i\|\le r_v\}|}
{|\mathcal F^\star|},\qquad C_t^J=\min(C_t^I,C_t^T).
\]

Joint success 要求 \(C_t^I\ge0.95\land C_t^T\ge0.95\)。由于最后一个离散动作会 overshoot，终点 coverage 必须和 success、distance、nodes/actions 联合解释。

### 5.2 空间节点与朝向动作

动作 \(a_k=(p_k,\theta_k)\) 与最近节点距离不超过 0.25 m 时，不创建新节点。报告：

- `num_nodes` / `topological_node_count`：空间拓扑规模；
- `num_oriented_views` / `oriented_view_count`：感知动作数；
- `in_place_rotations` 和 `total_rotation_deg`：窄 FOV 代价；
- `mean/median/min_nn_distance`：空间节点最近邻间距；
- `redundant_viewpoint_fraction`：新空间节点与任一历史节点小于 1 m 的比例；不再把同点 rotation 当成空间冗余。

### 5.3 安全、路程和效率

- `total_distance`：实际 A* 折线路程，不是目标直线或 RRT tree edge 总和；
- `avg/min_obstacle_distance`：空间节点到 truth obstacle 的事后净空；
- `avg/min_path_obstacle_distance`：全部路径采样净空；
- `node/path_safe_fraction`：满足 0.5 m preferred-clearance threshold 的比例；
- 0.3 m footprint 是硬碰撞约束；0.5 m 只是偏好/报告阈值，不能写成机器人几何尺寸；
- `coverage_per_viewpoint` 在 joint profile 是 topology coverage / spatial nodes。

## 6. 统计协议

每个方法有 162 runs：6 sensors × 9 environments × 3 seeds。先在每个 sensor–environment cluster 内平均 seeds，再以 54 clusters 为配对单位：

1. SSTG minus baseline 的宏平均 effect；
2. 10,000 次 cluster bootstrap 95% CI；
3. paired Wilcoxon；
4. 每个指标族内对四个 baselines 做 Holm correction；
5. paired rank-biserial effect size；
6. 同时检查六个 sensor groups 与九个 environment groups 的方向一致性。

八个指标族：sensor/topological coverage、distance、topological nodes、oriented actions、clearance、redundant nodes、success。候选、steps、nodes 和路径采样点都不是独立样本。

完整表在 `pairwise_all_metrics.csv`。注意两类不一致：

- SSTG vs NBV 的 terminal topology bootstrap CI 不跨 0，但 Holm-corrected Wilcoxon `p=0.147`；不能写显著；
- SSTG vs NBV/RRT 的 clearance CI 略低于 0，但 Holm `p=0.165`；不能写显著较差。

## 7. 三协议结果如何解释

| Case | SSTG sensor | SSTG 2 m topology | Distance | Nodes/actions | Success |
|---|---:|---:|---:|---:|---:|
| known-map disk | N/A | 98.52% | 48.13 m | 20.56 / 20.56 | 100% |
| unknown sensor-only | 98.38% | 33.02% post hoc | 15.73 m | 4.57 / 4.57 | 100% sensor target |
| unknown joint | 99.99% | 96.14% | 63.99 m | 15.76 / 17.30 | 100% dual target |

Sensor-only 的“100% success”只表示达到旧 sensor target，不表示拓扑任务成功。三者差异揭示 construct mismatch，不做跨协议 p-value。

在新 joint run 中，SSTG 第一次达到 95% sensor coverage 时，平均 topology 只有 48.31%；之后再增 47.83 pp，执行 10.76 个动作，其中 7.09 个明确为 `coverage_gap`，7.13 个动作没有发现新 sensor cells。这些 `new_cells=0` 动作是拓扑补全，不是停滞。

### 7.1 最终配置为何是 multi-frontier + spacing

180-run hard-scene 消融先比较 Full、No spacing、Single centroid、No vantages 和 Known-obstacle-only footprint。后者只有 75% success；其余均 100%，但 12-cluster Holm 检验没有模块差异显著。由于 single-centroid 在 hard set 的点估计更好，又补跑了 162-run 全矩阵：multi-frontier 的 topology 高 0.361 pp（95% CI [0.108, 0.634]，三个开发变体校正 `p=0.0487`），success/冗余相同，但多 0.46 nodes / 0.70 actions。Spacing 相对 `w_s=0` 保持 coverage/success/clearance，少 2.00 m travel 和 0.33 actions。因此最终配置按 coverage–safety–redundancy 优先级保留 multi-frontier 与 `spacing_weight=0.30`；全部选择表在 `VARIANT_SELECTION.md`，并明确标为 development evidence。

## 8. 审计与可复现性

`audit_report.json` 强制检查：

- 每条结果的 JSON/NPY/CSV 是否存在；
- `observed_updates` 能否从全 unknown 精确重放终止 belief；
- 每个 known cell 是否与 truth 一致；
- run 0 的 frame 数是否等于 steps 数；
- GIF/MP4 是否存在且非空；
- HTML 本地引用是否存在；
- log 是否含 traceback、FAILED 或 video error。

最终 joint 发布集 `joint_benchmark_selected/20260719_223630`：810 records、6,270 step PNG、270 GIF、270 MP4，全部通过。最终 SSTG component 与配置选择 run 的 162/162 normalized result records 精确一致；冻结基线另有 20-run 同 seed 完整 run JSON 精确复现。

研究有效性报告 `VALIDATION_REPORT.md` 完成 11/11 fallacy scan，整体为 `CAUTION`，主要因为 2 m construct、仿真外部有效性、三 seeds、跨指标探索性和开发阶段未预注册，而不是 artifact 内部不一致。

## 9. 论文允许与禁止的主张

可以写：

- 长量程 sensor coverage 不是短距观测拓扑 coverage 的有效替代；
- SSTG 在 joint 主表中 162/162 成功，并显著降低相对所有四个基线的空间节点冗余；
- SSTG 节点/动作少于 ANS、Frontier、NBV，净空高于 ANS、Frontier；
- dense、multiple rooms、warehouse 均在所有六传感器下达到双阈值；
- 上述收益以更长路程为代价，RRT adapter 的节点更少。

禁止写：

- “所有指标全面最优”或 “shortest path”；
- 2 m disk 等价于真实语义可见性/识别准确率；
- 仿真 known-free A* 是安全认证；
- adapted ANS/RRT 是原完整系统复现；
- 不显著的 coverage/clearance difference 是显著；
- 把 known、sensor-only、joint coverage 混合做 p-value；
- 把 candidates、steps 或 pixels 当作独立实验单位。
