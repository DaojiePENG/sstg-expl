# SSTG-Explorer

SSTG-Explorer（Spatial Semantic Topological Graph Explorer）是在已知二维占据栅格上，为视觉语义采集生成安全、近似均匀且具有覆盖性的机器人观测节点序列的探索算法。本仓库包含算法、四种基线、九类仿真环境，以及从实验运行到逐步截图、视频和网页报告的一站式可复现流程。

> 当前研究结论：消融实验中平均覆盖率最高的 SSTG 变体（历史名 `SSTG (Optimal)`）现正式命名为 **SSTG-Explorer**。它采用 30° 基础角采样、增强距离优先级、A* 路径检查和窄通道自适应采样。本仓库 2026-07-19 重新运行的 225 次实验中其环境宏平均覆盖率为 81.82%，低于 Frontier 的 97.22%，但总路径更短（31.84 m 对 41.12 m），并保持面向语义观测的安全、规则节点布局。仓库不宣称在所有覆盖率指标上领先；详见 `docs/PAPER_WRITING_REFERENCE.md`。

## 1. 安装

需要 Linux/macOS、Conda 和 Git。所有命令在仓库根目录执行。

```bash
conda env create -f environment.yml
conda activate sstg-explorer
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

若环境已存在：

```bash
conda env update -n sstg-explorer -f environment.yml --prune
```

## 2. 最小使用示例

```python
from sstg_explorer import SSTGExplorer
from sstg_explorer.environments import create_environment

env = create_environment("maze", width=12.0, height=12.0)
env.name = "maze"
explorer = SSTGExplorer()
result = explorer.explore(env.get_occupancy_map(), env.get_start_pose())
print(result["metadata"])
```

核心 API 是 `SSTGExplorer.explore(occupancy_grid, start_pose)`。直接调用 `SSTGExplorer()` 即使用最终方案（增强距离策略、A* 与窄通道自适应采样）。`start_pose` 为 `(x, y, theta_deg)`；结果包含 `nodes`、`metadata` 和 `success`。每个节点至少含二维 `position`。地图分辨率、机器人半径、视野半径等参数见 `src/sstg_explorer/config.py`。

## 3. SSTG-Explorer 算法

算法维护一个全局候选 frontier 队列：

1. 从当前观测节点沿离散方向生成候选点，步长由视野半径和重叠约束决定。
2. 拒绝与障碍、机器人安全外形或既有观测覆盖冲突的候选点。
3. 按新颖性、距离和局部结构计算优先级，并从全局队列选择下一个目标。
4. 使用 A* 检查目标可达性；窄通道中增加角采样密度。
5. 更新覆盖率和队列，达到目标覆盖率、队列耗尽或迭代上限后终止。

最终配置通过 `runner.create_algorithm("sstg_explorer")` 创建。`sstg_optimal` 是兼容旧实验文件的别名，不应用于新论文或图表。`sstg`、`sstg_enhanced` 仅用于消融研究。

关键默认值：`r_view=2.0 m`、`d_theta=30°`、`overlap=0.25 m`、`r_robot=0.3 m`、目标覆盖率 `0.95`。实际传感器标定时应重新设置视野和重叠，而不是直接复制仿真值。

## 4. 运行完整 benchmark

先做约数分钟以内的链路检查：

```bash
python scripts/run_benchmark.py --profile smoke
```

再运行论文级实验（5 算法 × 9 环境 × 5 次）：

```bash
python scripts/run_benchmark.py --profile full
```

结果写入 `outputs/benchmark_runs/<timestamp>/`，`outputs/benchmark_runs/latest` 指向最近一次结果。打开报告：

```bash
python -m http.server 8000 --directory outputs/benchmark_runs/latest
# 浏览器访问 http://localhost:8000/
```

每次运行保存参数、版本、随机种子、终端日志、完整轨迹、每一步 PNG、最终 PNG、GIF、MP4 和原始 JSON；网页默认展示每个算法–环境组合的 run 0。完整参数和结果解释见 [docs/BENCHMARK_GUIDE.md](docs/BENCHMARK_GUIDE.md)。

## 5. 复现实验与解释指标

- `coverage_ratio`：视野圆覆盖的可通行区域比例，越高越好。
- `total_distance`：按输出节点序列累计的运动距离，越低越好。
- `coverage_efficiency`：覆盖率/距离，越高越好。
- `avg/min_obstacle_distance`：观测点到最近障碍物的距离，越高通常越安全。
- `mean_nn_distance` 与 `dispersion_uniformity`：采样间距及均匀性。
- `computation_time`：本机墙钟时间，不应跨硬件直接比较。

所有随机方法按 `base_seed + run_id` 固定种子。论文中应报告均值、标准差、失败数和硬件信息，并保留 `manifest.json`。

## 6. 项目结构

```text
src/sstg_explorer/      唯一可安装 Python 包
  core/                 SSTG-Explorer 核心算法
  map/                  占据栅格和距离场
  planning/             A* 路径规划
  baselines/            四种对比算法
  benchmark/            benchmark 执行与分析
  environments/         九类仿真环境
  visualization/        静态图、逐步动画和实时可视化
scripts/run_benchmark.py 唯一正式 benchmark 入口
examples/basic_exploration.py 最小示例
tests/                  自动化测试
docs/                   benchmark、论文架构和 RAL 写作参考
outputs/benchmark_runs/ 可复现实验产物（按时间戳）
environment.yml         Conda 锁定入口
```

## 7. 测试与常见问题

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
python scripts/run_benchmark.py --profile smoke --no-frames
```

- 无图形桌面也可运行；脚本使用 Matplotlib `Agg` 后端。
- MP4 失败时检查 `imageio-ffmpeg`，错误会写入对应 `video_error.txt`。
- 结果很大是正常的；逐步帧占主要空间。调试时使用 `--no-frames`。
- 从外部目录调用脚本也受支持，路径均由脚本位置解析，不含机器绝对路径。

## 8. 论文材料与引用

- [论文架构](docs/PAPER_STRUCTURE.md)
- [RAL 写作参考](docs/PAPER_WRITING_REFERENCE.md)
- [Benchmark 使用与结果说明](docs/BENCHMARK_GUIDE.md)

```bibtex
@software{sstg_explorer_2026,
  title  = {SSTG-Explorer: Spatial Semantic Topological Graph Explorer},
  author = {Peng, Daojie},
  year   = {2026},
  url    = {https://github.com/DaojiePENG/sstg-expl}
}
```

当前仓库未包含明确许可证文件；在公开发布或投稿附带代码前请补充合适的 `LICENSE`。
