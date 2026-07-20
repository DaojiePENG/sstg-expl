# SSTG-Explorer

SSTG-Explorer（Spatial Semantic Topological Graph Explorer）面向未知室内环境，在线生成安全、稀疏、可供后续语义感知或巡检任务使用的观测拓扑。它解决的核心问题不是“雷达是否已经把地图看全”，而是：

> 长量程传感器已经完成占据建图时，机器人是否仍以规定的任务半径（正式实验为 2 m）覆盖了整个自由空间，并留下非冗余的空间观测节点？

本仓库提供算法库、Conda 环境、已知/未知地图基线、逐步候选 trace、完整 benchmark、统计分析、图片/视频和可浏览网页。当前实现是静态二维占据栅格仿真；“SSTG”中的 semantic 表示这些空间节点将来可承载语义证据，当前版本不声称已完成语义识别或真实机器人验证。

## 1. 三种实验语义

| 协议 | 策略可读信息 | 覆盖定义 | 用途 |
|---|---|---|---|
| `known_static_disk` | 完整静态地图 | 观测节点 2 m 圆盘覆盖 | 全知结构参考与模块消融 |
| `unknown_static_grid_occlusion_aware` | 在线三值 belief | 遮挡射线已正确观测的 free cells | 旧 sensor-only 诊断协议 |
| `unknown_static_grid_joint_topological_coverage` | 在线三值 belief | 同时满足 sensor coverage 与 2 m topology coverage | 论文主协议 |

三者不能混作同一随机变量做显著性检验。已知地图是全知参考；旧 unknown 结果用于证明“sensor 看全不等于拓扑看全”；joint unknown 是新算法与基线的公平主任务。

正式结果说明这个区别非常大：旧 sensor-only SSTG 的 sensor coverage 为 98.38%，但把其探索点按 2 m 重新评价后，topological coverage 只有 33.02%。新 joint SSTG 在线继续生成 coverage-gap candidates，最终达到 99.99% sensor coverage、96.14% topological coverage 和 100% success。

## 2. 安装与测试

需要 Conda、Git、FFmpeg 和 Linux/macOS。仓库提供 Python 3.10 环境：

```bash
conda env create -f environment.yml
conda activate sstg-explorer
python -m pip install -e .
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

本机 ROS Jazzy 会把 Python 3.12 的 pytest 插件注入项目的 Python 3.10 环境，因此测试命令显式关闭第三方插件自动加载。当前回归结果为 `18 passed`。

学习基线首次使用前运行：

```bash
python scripts/setup_learning_baselines.py --install-dependencies
```

该脚本安装 CPU PyTorch、下载 Active Neural SLAM global-policy checkpoint 并校验 SHA-256。权重不提交到 Git；正式 manifest 记录其来源和哈希。

## 3. 最小使用示例

### 3.1 在线 joint SSTG-Explorer（默认/推荐）

```python
from sstg_explorer import SensorConfig, UnknownExplorerConfig, UnknownMapExplorer
from sstg_explorer.environments import create_environment

environment = create_environment("multiple_rooms")
explorer = UnknownMapExplorer(UnknownExplorerConfig(
    strategy="sstg",
    coverage_objective="joint",
    sensor=SensorConfig(
        field_of_view_deg=120,
        max_range=12,
        angular_resolution_deg=0.25,
    ),
    topological_radius=2.0,
    target_coverage=0.95,              # sensor coverage target
    target_topological_coverage=0.95,
))

result = explorer.explore(
    environment.get_occupancy_map(),  # 内部仅由 sensor/evaluator 持有
    environment.get_start_pose(),
)

print(result["success"])
print(result["metadata"]["sensor_coverage_ratio"])
print(result["metadata"]["topological_coverage_ratio"])
print(result["metadata"]["topological_node_count"])
print(result["metadata"]["oriented_view_count"])
print(result["nodes"])                 # 空间拓扑节点
print(result["oriented_views"])        # 带朝向的感知动作
print(result["steps"][0])              # 完整候选和 belief update
```

传入的 occupancy grid 是仿真 hidden truth。策略候选生成、gain、reachability、utility 和 A* 只能读取逐步积累的 `belief`；truth 只进入 ray sensor 和 evaluator。

`UnknownExplorerConfig()` 默认即为最终 `coverage_objective="joint"`、`spacing_weight=0.30`。如需复现冻结的旧诊断协议，必须显式设置 `coverage_objective="sensor"`；它不代表最终 SSTG-Explorer。

### 3.2 已知地图结构规划

```python
from sstg_explorer import SSTGExplorer
from sstg_explorer.environments import create_environment

environment = create_environment("maze")
result = SSTGExplorer().explore(
    environment.get_occupancy_map(),
    environment.get_start_pose(),
)
print(result["metadata"])
```

默认 `SSTGExplorer()` 是已知地图最终版本；历史实验变体只在 benchmark profile 中暴露。

## 4. Joint SSTG-Explorer 算法

令 hidden truth 为 \(M^\star\)，策略 belief 为 \(B_t\in\{-1,0,100\}\)。算法同时报告：

\[
C_t^I=\frac{|\{c:M^\star(c)=0\land B_t(c)=0\}|}{|\{c:M^\star(c)=0\}|},
\]

\[
C_t^T=\frac{|\{c:M^\star(c)=0,\ \min_{v_i\in V_t}\|x_c-p_i\|\le r_v\}|}
{|\{c:M^\star(c)=0\}|},\qquad C_t^J=\min(C_t^I,C_t^T).
\]

正式任务要求 \(C_t^I\ge0.95\) 且 \(C_t^T\ge0.95\)，其中 \(r_v=2\) m。2 m disk 是与已知地图 benchmark 一致的任务距离 proxy，不等于真实相机可见性或语义识别置信度。

每轮包含五步：

1. 以 0.3 m 机器人 footprint 腐蚀 `known_free`，只在当前起点四连通安全区规划；unknown cell 从不当作 free。
2. 从 frontier band、reachable-space FPS vantages 和已知自由空间拓扑缺口生成候选。即使 sensor 已经没有 unknown gain，coverage-gap candidates 仍继续生成。
3. 对候选计算信息增益 (G_t^I)、2 m 边际拓扑增益 (G_t^T)、known-free 测地代价和障碍净空。
4. SSTG 使用与代码一致的联合效用：

   \[
   \bar g_t=0.4\bar G_t^I+0.6\bar G_t^T,\qquad
   U_t(f)=\frac{1.20\bar g_t(f)}{1+0.60\,d_G(p_t,f)/r_v}
   +0.30\bar c_t(f)+0.30\bar s_t(f),
   \]

   其中 \(\bar s_t\) 是候选到已有空间节点的截断归一化距离。全矩阵配对选择实验表明该项在不损失 coverage、success 或 clearance 的情况下减少路程、节点和动作，因此进入最终配置。

5. 执行 known-free A*，沿实际路径每 1 m 扫描并在最终朝向补一帧；更新 belief、节点、朝向动作和两类 coverage。

同一位置附近的不同朝向不是多个空间点：若动作与已有节点距离不超过 0.25 m，它关联到该节点但仍保留为 oriented action。这样可以分别评价拓扑冗余和窄 FOV 的旋转代价。

完整公式、伪代码、复杂度和论文表述边界见 [论文写作参考](docs/PAPER_WRITING_REFERENCE.md)。

## 5. 如何理解逐步可视化

Joint 图同时给出 policy belief、evaluation-only truth、实际 A* 轨迹、每个 2 m 节点圆盘和全部候选生命周期。

- 蓝色节点/折线：已接受空间节点与实际执行轨迹；
- 橙色区域：当前 belief 中已知自由但尚未被 2 m 节点覆盖；
- 右侧 truth 红/橙/紫/蓝：neither / sensor-only / topology-only / both；
- 橙色三角：pending candidates；
- 绿色菱形：本轮新生成候选；
- 蓝色空心方块：topology gap candidates；
- 星形：selected candidate；
- 各色叉号：gain、预算、重复或已执行剪枝；
- 右栏：sensor/topology/joint coverage、候选 ID/type、两类 gain、priority、cost、clearance 和状态计数。

图不是数据的替代品。每个 run 同时保存 `run.json`、`decisions.csv`、`candidates.csv`、`trajectory.csv`、`path_waypoints.csv`、`scan_poses.csv`、`oriented_views.csv` 和 `belief_final.npy`。

## 6. 运行 benchmark

先验证链路：

```bash
python scripts/run_unknown_benchmark.py \
  --profile smoke --coverage-objective joint
```

论文主矩阵：

```bash
# 6 sensors × 9 environments × 5 methods × 3 seeds = 810 runs
python scripts/run_unknown_benchmark.py \
  --profile paper --coverage-objective joint
```

联合困难场景消融：

```bash
# 5 SSTG variants × 4 hard scenes × 3 sensors × 3 seeds = 180 runs
python scripts/run_unknown_benchmark.py \
  --profile ablation --coverage-objective joint --no-frames
```

扩展统计与 sensor 饱和后 gap 分析：

```bash
python scripts/analyze_joint_benchmark.py \
  outputs/joint_benchmark_runs/latest
```

已知地图参考：

```bash
python scripts/run_benchmark.py --profile smoke
python scripts/run_benchmark.py --profile full
python scripts/run_benchmark.py --profile ablation --no-frames
```

自定义方法、场景、传感器和 seeds 可用 `--algorithms`、`--environments`、`--sensors`、`--runs`。调试才使用 `--no-frames`；正式 profile 的三个 seeds 都保留完整数值 trace，run 0 额外编码全部 PNG/GIF/MP4。

## 7. 输出、审计与网页

主结果位于：

```text
outputs/joint_benchmark_runs/<timestamp>/
├── manifest.json                         命令、参数、依赖、Git、源码/checkpoint hash
├── results.json                          810 条结果
├── summary.csv / aggregate.csv           分组与宏平均
├── pairwise_all_metrics.csv              8 个指标族的 CI/Wilcoxon/Holm/effect size
├── post_sensor_gap_closure*.csv           sensor 达标后拓扑补全分析
├── hard_scene_analysis.csv               dense/rooms/warehouse
├── three_protocol_comparison.{csv,md}     三种覆盖语义
├── statistical_analysis.{json,md}
├── VALIDATION_REPORT.md                   11/11 统计谬误扫描与复现结论
├── audit_report.json
├── index.html
└── artifacts/<sensor>/<env>/<method>/
    ├── run.json / belief_final.npy
    ├── decisions.csv / candidates.csv
    ├── trajectory.csv / path_waypoints.csv / scan_poses.csv
    ├── oriented_views.csv
    ├── steps/step_XXXX.png / final.png
    └── animation.gif / video.mp4
```

最终发布集 `outputs/joint_benchmark_selected/20260719_223630/` 由 648 个未受 utility 选择影响的冻结基线和 162 个最终 SSTG runs 组成；manifest 保存两部分的路径、结果 SHA-256、源码 SHA-256 与选择报告。自动审计为：

- 810/810 run JSON、belief 和六类 CSV 完整；
- 6,270 张 step PNG、270 GIF、270 MP4；
- belief replay mismatch = 0；known-cell/truth mismatch = 0；
- media error、HTML missing reference、log error marker 均为 0；
- 最终 SSTG 的 162-run 重跑删除算法标签和 wall-clock 字段后与选择实验 162/162 精确匹配；冻结基线另有 20/20 完整 normalized run JSON 精确复现。

打开网页：

```bash
python -m http.server 8002 \
  --directory outputs/joint_benchmark_selected/latest
```

浏览器访问 `http://127.0.0.1:8002/`。

## 8. 正式主结果

| 方法 | Sensor | Topology 2 m | Distance | Nodes | Actions | NN | Redundant nodes | Clearance | Success |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ANS-Global Joint (adapted) | 99.98% | 96.42% | 63.20 m | 19.00 | 21.41 | 1.96 m | 4.5% | 0.72 m | 98.1% |
| Frontier Joint | 99.96% | **97.04%** | 54.08 m | 32.02 | 39.46 | 1.27 m | 29.6% | 0.79 m | 96.3% |
| NBV Joint | 99.78% | 94.28% | **46.78 m** | 17.46 | 20.29 | 1.73 m | 15.0% | **0.99 m** | 90.7% |
| RRT Joint (adapted) | 99.99% | 96.19% | 57.64 m | **14.81** | **16.66** | 2.07 m | 3.5% | **0.99 m** | **100%** |
| **SSTG-Explorer Joint** | **99.99%** | 96.14% | 63.99 m | 15.76 | 17.30 | **2.15 m** | **0.1%** | 0.95 m | **100%** |

以 54 个 sensor–environment 为 cluster：

- SSTG 相对 ANS/Frontier/NBV/RRT 的空间节点冗余分别低 4.38/29.56/14.96/3.41 pp，所有 95% CI 不跨 0，所有 Holm `p < 1.8e-5`；
- SSTG 节点和动作显著少于 ANS、Frontier、NBV，但比 RRT 多 0.94 nodes / 0.64 actions；
- SSTG 净空显著高于 ANS 和 Frontier；相对 NBV/RRT 的小幅净空劣势经 Holm 后不显著；
- SSTG 和 RRT 均 100% success；SSTG 相对 NBV 高 9.26 pp（Holm `p=0.0266`）；
- SSTG 的明确代价是 travel：相对 Frontier/NBV/RRT 长 9.91/17.22/6.36 m，均显著。

因此最终算法定位是“可靠完成双覆盖、空间节点极少重复、净空较高的观测拓扑”，不是 shortest-path 算法，也不是所有指标全面第一。

最终配置选择也保留完整负结果。12-cluster hard set 中 single-centroid 的点估计一度更好，因此又补跑了 162-run 全矩阵；multi-frontier 最终以 +0.361 pp topology（95% CI [0.108, 0.634]，三个开发变体保守校正 `p=0.0487`）胜出，但多 0.46 nodes / 0.70 actions。Spacing 项相对 `w_s=0` 保持 coverage/success/clearance，并减少 2.00 m travel 与 0.33 actions。选择过程见最终网页的 `VARIANT_SELECTION.md`，这些属于透明的 development evidence，不冒充独立验证。

困难场景全部六传感器下，SSTG 在 `multiple_rooms` / `dense_obstacles` / `warehouse` 达到 95.47% / 96.16% / 95.61% topological coverage，54/54 成功；节点冗余为 0% / 0% / 0.67%。warehouse 仍有 110.27 m 的平均路程，是当前主要短板。

## 9. 项目结构

```text
src/sstg_explorer/
  core/                 known-map SSTG、frontier queue、coverage
  unknown/              belief-only joint explorer 与 adapters
  sensing/              FOV/range/first-obstacle ray casting
  planning/             A* 和 one-to-all geodesic map
  baselines/            Frontier/NBV/RRT/ANS/Uniform adapters
  benchmark/            公共执行和指标
  environments/         九类可复现二维场景
  visualization/        belief/truth/candidate/coverage 图
scripts/
  run_unknown_benchmark.py
  analyze_joint_benchmark.py
  compare_joint_variants.py
  assemble_joint_benchmark.py
  finalize_unknown_benchmark.py
  run_benchmark.py
  setup_learning_baselines.py
docs/
  BENCHMARK_GUIDE.md
  PAPER_STRUCTURE.md
  PAPER_WRITING_REFERENCE.md
  REFERENCES.bib
tests/
environment.yml
```

## 10. 论文与投稿材料

- [Benchmark 协议、字段、公平性和结果解释](docs/BENCHMARK_GUIDE.md)
- [11 页扩展主稿、RA-L 压缩路线与 T-RO 扩展门槛](docs/PAPER_STRUCTURE.md)
- [完整公式、算法、统计表述和写作禁区](docs/PAPER_WRITING_REFERENCE.md)
- [基线与相关工作 BibTeX](docs/REFERENCES.bib)
- [11 页英文扩展稿 PDF](../SSTGExplorerPaper/root.pdf)
- [从冻结数据生成全部论文图表的脚本](../SSTGExplorerPaper/figures/generate_paper_figures.py)
- [五算法 × 六阶段的 30-panel 同场景过程对照](../SSTGExplorerPaper/figures/algorithm_process_dense_comparison.pdf)
- [SSTG 候选生成、拒绝、选择与 gap closure 六阶段专图](../SSTGExplorerPaper/figures/sstg_candidate_lifecycle_dense.pdf)
- [同一 dense 场景五算法全部 123 个决策帧的 14 页联系表](../SSTGExplorerPaper/figures/algorithm_process_dense_all_steps.pdf)
- [Round-2 学术审稿与剩余风险](../SSTGExplorerPaper/REVIEW_ROUND2.md)
- [最终引用、数据、图表与原创性完整性报告](../SSTGExplorerPaper/FINAL_INTEGRITY_REPORT.md)
- [图表逐项来源与主张追踪](../SSTGExplorerPaper/FIGURE_TABLE_TRACE.yaml)
- [实验、负结果、限制和主张来源声明](../SSTGExplorerPaper/EXPERIMENT_PROVENANCE.yaml)
- [论文高影响数值防漂移检查脚本](scripts/verify_paper_claims.py)

论文仓库位于同级 `../SSTGExplorerPaper/`。当前正文用一张 17 列总表统一三协议与全部方法，另含 cluster effect、困难场景和消融三张表，以及真实三阶段动机图、图义完整解释的三协议 trade-off、五算法各 6 阶段共 30 个过程画面、SSTG 候选生成/拒绝/选择 6 阶段专图和完整 270-cell atlas。trace、网页和视频仍只作为可复现性证据，不作为科研创新点。

## 11. 局限与发布前事项

- 2 m disk 只表示任务距离 proxy；没有模拟可见性、入射角、分辨率或语义置信度；
- 静态 2-D、理想射线、完美位姿，不等同于 SLAM/真实机器人安全性；
- ANS 和 RRT 为公共 belief/sensor/planner 接口下的 adapters，不是原完整系统复现；
- 每个配置 3 seeds，参数开发未预注册；正式验证报告整体置信度为 `CAUTION`；
- 当前仓库没有明确 LICENSE。公开发布代码、checkpoint adapter 或投稿 artifact 前必须补充许可证并复核第三方许可。
