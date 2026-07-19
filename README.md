# SSTG-Explorer

SSTG-Explorer（Spatial Semantic Topological Graph Explorer）是在二维占据栅格上生成视觉语义观测节点序列的可追踪探索算法。它不仅输出最终路径，还记录每次决策中所有候选点、拒绝原因、待探索 frontier、所选目标、A* 路径、覆盖增量和全局恢复过程。

本仓库面向两类读者：

- 使用者可以从零创建环境、调用算法并浏览逐步结果；
- 论文作者可以复现六方法、九环境、多 seed 的 benchmark，并直接获得原始 JSON、逐步图、视频、统计表、LaTeX 表格和网页。

> 任务边界：当前实验给定静态二维占据栅格，覆盖率使用圆盘视野 proxy。它不是完整在线 SLAM，也尚未模拟真实相机遮挡和语义识别误差。论文材料明确保留这些限制。

## 1. 安装

需要 Conda、Git 和 Python 3.10。所有命令在仓库根目录执行。

```bash
conda env create -f environment.yml
conda activate sstg-explorer
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

更新已有环境：

```bash
conda env update -n sstg-explorer -f environment.yml --prune
python -m pip install -e .
```

完整 benchmark 包含公开预训练学习基线，首次使用额外执行：

```bash
python scripts/setup_learning_baselines.py --install-dependencies
```

这会安装 CPU PyTorch、下载 Active Neural SLAM global-policy checkpoint，并校验 SHA-256；权重不会提交到 Git。

## 2. 最小使用示例

```python
from sstg_explorer import SSTGExplorer
from sstg_explorer.environments import create_environment

env = create_environment("maze", width=12.0, height=12.0)
explorer = SSTGExplorer()
result = explorer.explore(
    env.get_occupancy_map(),
    env.get_start_pose(),
)

print(result["success"])
print(result["metadata"])
print(result["nodes"])
print(result["steps"][0])  # 完整候选与决策状态
```

`start_pose` 是 `(x, y, theta_deg)`。返回值：

- `nodes`：已接受观测节点；
- `steps`：逐决策 trace，包括失败选择和恢复事件；
- `metadata`：覆盖、A* 路径距离、全部实际 `paths`、节点数、时间、恢复轮次和终止原因；
- `success`：是否达到目标覆盖率。

默认 `SSTGExplorer()` 就是最终方法，不需要选择历史 `optimal` 变体。参数集中在 `src/sstg_explorer/config.py`。

## 3. 算法概览

### 3.1 安全角向候选

在当前节点周围以 `r_view` 为半径、固定 `d_theta=30°` 产生候选。候选依次经过机器人外形膨胀碰撞、已覆盖区域强度、最低优先级和队列重复检查。正式消融显示固定 30° 在覆盖、路程、净空和运行时间上均不差于窄通道 15° 增密，因此 adaptive sampling 只保留为消融，不属于最终 SSTG-Explorer。

### 3.2 可追踪全局队列

所有存活候选进入全局 max-priority queue。每次 priority 更新都会让旧 heap entry 失效，避免 stale priority 造成跨房间来回跳转。每个候选保留稳定 ID、来源、分数和状态。

### 3.3 测地、安全优先级与 A*

算法从当前节点计算 one-to-all 安全栅格测地 cost map。候选联合障碍感知距离、探索强度和障碍净空排序，而不是可能穿墙的欧氏距离；选中后再用按 `r_robot+d_safe=0.5 m` 膨胀的 A* 输出实际折线路径。A* 搜索预算至少覆盖整张栅格，避免把预算耗尽误判成不可达。

### 3.4 覆盖缺口恢复

若局部队列耗尽但覆盖仍不足，算法在未覆盖安全自由空间上计算距离变换，选择大缺口的局部极大值，经 A* 可达性验证后重新播种队列。最终同 commit 消融中，关闭该机制会使 `sparse_obstacles` 停在 95% 门槛以下；不要把它限定描述为某一个窄通道特例。

完整公式、伪代码、复杂度和消融设计见 [论文公式参考](docs/PAPER_WRITING_REFERENCE.md)。

## 4. 如何理解一张逐步图

每张 `step_XXXX.png` 同时给出：

- 蓝色圆点：已探索观测点；蓝色连续折线：累计实际 A* 执行轨迹；
- 黄色三角：当前待探索 global frontiers，大小反映 priority；
- 绿色菱形：本步新加入候选；
- 红色 ×：机器人膨胀后碰撞；
- 橙色 ×：探索强度不足；
- 紫色 ×：优先级不足；
- 灰色 ×：与队列候选重复；
- 粉色星形：本步 selected frontier；
- 绿色实心点：当前机器人位置；
- 青色折线：实际 A* 路径；
- 青色 P：全局覆盖缺口恢复候选。

右侧面板列出 event、coverage before/after/gain、explored/pending 数量、选择 ID/类型/priority，以及各候选状态计数。逐步图对应的完整数值保存在同目录 `run.json` 和全局 `results.json`。

## 5. Benchmark 方法与依据

正式 profile 比较：

| CLI 名称 | 方法 | 依据 |
|---|---|---|
| `uniform_grid` | 均匀覆盖网格 | coverage path planning 文献 |
| `rrt` | RRT 视点采样 | LaValle；multi-RRT exploration |
| `frontier` | 栅格 frontier | Yamauchi 1997 |
| `nbv` | next-best-view 信息增益 | Connolly 1985 |
| `active_neural_slam` | ANS-Global (adapted) | Chaplot et al., ICLR 2020 公开 checkpoint |
| `sstg_explorer` | 完整 SSTG-Explorer | 本项目 |

ANS 只适配发布的 learned global policy；RGB Neural-SLAM mapper 和 learned local controller 不在当前二维协议中。因此所有结果始终标为 `ANS-Global (adapted)`，不能当作完整 ANS 复现。

六种方法都在相同的 0.5 m 膨胀栅格上执行 A*，统一报告实际路径长度。Uniform 的障碍格点投影到最近安全格，Frontier 按可达测地距离选择，NBV 的旅行代价来自 cost map，RRT 的新增覆盖节点也由机器人实际访问；不会再让 baseline 用跨墙直线或树边总长获得虚假优势。

逐基线出处、公平性边界与 BibTeX 见 [Benchmark 指南](docs/BENCHMARK_GUIDE.md) 和 [REFERENCES.bib](docs/REFERENCES.bib)。

## 6. 运行实验

先验证数值和媒体链路：

```bash
python scripts/run_benchmark.py --profile smoke
```

论文级完整实验：

```bash
python scripts/run_benchmark.py --profile full
```

论文消融实验：

```bash
python scripts/run_benchmark.py --profile ablation --no-frames
```

自定义范围：

```bash
python scripts/run_benchmark.py --runs 3 \
  --algorithms frontier active_neural_slam sstg_explorer \
  --environments multiple_rooms dense_obstacles narrow_passages
```

调试时可用 `--no-frames`；正式实验默认保存每个 run 的所有决策帧、GIF 和 MP4。

## 7. 输出与网页

结果写入 `outputs/benchmark_runs/<timestamp>/`，`latest` 指向最近结果。核心文件：

```text
manifest.json                     命令、参数、Git、环境和 checkpoint
run.log                           逐实验日志
results.json                      全部 trajectory、decision trace 和指标
summary.csv                       逐环境 mean/std/95% CI
aggregate.csv                     跨环境汇总
results_table.md / .tex           论文表格草稿
coverage_heatmap.png
coverage_distance_tradeoff.png
safety_comparison.png / safety_table.{csv,md,tex}
artifacts/<env>/<algo>/steps/     每一个决策状态截图
artifacts/<env>/<algo>/video.mp4
index.html
```

打开网页：

```bash
python -m http.server 8000 --directory outputs/benchmark_runs/latest
```

浏览器访问 `http://127.0.0.1:8000/`。完整目录和核验规则见 [Benchmark 指南](docs/BENCHMARK_GUIDE.md)。

## 8. 指标解释

- `coverage_ratio`：自由栅格被视野圆覆盖的比例；
- `total_distance`：实际 A* 折线路径累计；
- `coverage_efficiency`：coverage/distance；
- `avg/min_obstacle_distance`：节点安全裕量；
- `avg/min_boundary_distance`：节点到地图边界净空；
- `node_safe_fraction`：满足 0.5 m 机器人+安全裕量的视点比例；
- `avg/min_path_obstacle_distance`、`path_safe_fraction`：对全部实际 A* 轨迹采样的安全指标；
- `mean_nn_distance`、`dispersion_uniformity`：节点最近邻间距与规则性；
- `success_rate`：达到目标覆盖的 run 比例；
- `num_generated/rejected/recovery_candidates`：决策过程诊断量。

论文应报告所有环境、mean ± std/CI、失败数和硬件信息。不能从不同 commit 拼结果，也不能只展示 SSTG 获胜环境。

## 9. 当前正式结果（2026-07-19）

最终目录 `outputs/benchmark_runs/20260719_043528/` 包含 6 方法 × 9 环境 × 5 runs，共 270 条记录；受控消融在 `outputs/ablation_runs/20260719_043544/`，共 315 条。跨环境/run 汇总：

| 方法 | Coverage | Distance | Nodes | Success |
|---|---:|---:|---:|---:|
| SSTG-Explorer | **98.52 ± 1.27%** | 48.13 ± 23.49 m | 20.56 | 100% |
| Frontier | 96.92 ± 1.69% | **45.28 ± 20.16 m** | 19.44 | 100% |
| ANS-Global (adapted) | 96.48 ± 0.97% | 79.49 ± 46.43 m | 19.00 | 100% |
| NBV | 96.11 ± 0.94% | 89.96 ± 50.59 m | **12.24** | 100% |
| RRT | 96.32 ± 1.57% | 214.56 ± 150.92 m | 51.24 | 100% |
| Uniform Grid | 96.93 ± 1.65% | 60.23 ± 29.99 m | 25.22 | 100% |

以环境为 cluster 的 10,000 次 bootstrap 与环境级 Wilcoxon/Holm 显示：SSTG 相对 Frontier 覆盖高 `+1.61 pp [0.90, 2.45]`（Holm `p=0.0234`），路径差 `+2.85 m [-2.80, 8.92]` 不显著。相对 ANS/NBV/RRT/Uniform，覆盖优势分别为 `+2.05/+2.42/+2.20/+1.60 pp`，Holm 校正后均显著；路径分别短 `31.36/41.83/166.43/12.10 m`。

旧短板已经重新验证：

- `multiple_rooms`：97.11%，66.19 m，平均视点净空 1.411 m；
- `dense_obstacles`：99.59%，48.38 m，六方法中覆盖、路程与视点净空均为最佳点估计；
- `narrow_passages`：98.00%，75.73 m，覆盖与视点净空均为最佳点估计。

SSTG 平均视点净空 1.160 m，仅低于路径代价极高的 RRT 1.188 m；它相对 Frontier 的净空增量为 `+0.097 m [0.032, 0.157]`（Holm `p=0.0469`），相对 ANS/NBV/Uniform 也显著更高。平均的“每个 run 最小路径净空”0.597 m 为最高点估计，所有方法安全视点比例均为 100%。消融中去掉净空效用会使视点净空下降 `0.140 m [0.074, 0.213]`（Holm `p=0.0469`）且覆盖下降 1.56 pp；关闭 recovery 使 success 降至 88.9%。

最终 fixed-30° 与 adaptive-15° 的差异未达统计显著；选择 fixed 是因为它的 coverage/distance/clearance/runtime 宏平均点估计同时不差。结论应定位为“显著更高覆盖、可比于 Frontier 的路程、较安全且可审计的语义视点”，而不是“所有指标全面最优”。

## 10. 项目结构

```text
src/sstg_explorer/
  core/                 SSTG 核心、frontier queue、碰撞与覆盖
  planning/             A* 与 one-to-all geodesic cost
  baselines/            Uniform/RRT/Frontier/NBV/ANS adapter
  benchmark/            统一执行和指标
  environments/         九类可复现二维环境
  visualization/        最终图与完整 decision-trace 图
scripts/
  run_benchmark.py             唯一 benchmark 入口
  setup_learning_baselines.py  学习权重/依赖安装
docs/
  BENCHMARK_GUIDE.md
  PAPER_STRUCTURE.md
  PAPER_WRITING_REFERENCE.md
  REFERENCES.bib
tests/                  单元与公共 API 回归测试
environment.yml         核心 Conda 环境
```

## 11. 测试与复现检查

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
python examples/basic_exploration.py
python scripts/run_benchmark.py --profile smoke --no-frames
```

常见问题：

- 无桌面环境可以运行，绘图使用 Matplotlib `Agg`。
- MP4 需要 `imageio-ffmpeg`；错误写入 `video_error.txt`。
- 完整 trace 占用大量磁盘，调试才使用 `--media-runs representative`。
- checkpoint 校验失败时重新运行 setup 脚本，不要使用未知权重。
- Pytest 若加载系统 ROS 插件，使用上述 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

## 12. RAL 写作材料

- [文章结构、两张 Mermaid 图与可投稿 SVG 架构图](docs/PAPER_STRUCTURE.md)
- [公式、伪代码、复杂度、消融和写作禁区](docs/PAPER_WRITING_REFERENCE.md)
- [Benchmark、逐步字段和基线公平性](docs/BENCHMARK_GUIDE.md)
- [BibTeX 文献库](docs/REFERENCES.bib)

当前仓库没有明确许可证文件。公开代码或提交 multimedia/code artifact 前必须补充许可证，并核对第三方 ANS checkpoint/代码的 MIT 许可和引用要求。
