# Benchmark 使用与结果说明

## 一条命令的完整流程

`scripts/run_benchmark.py` 是唯一正式入口，负责环境构造、重复实验、原始数据保存、逐步可视化、动画、视频、统计和网页。先运行 smoke 验证依赖，再运行 full：

```bash
conda activate sstg-explorer
python scripts/run_benchmark.py --profile smoke
python scripts/run_benchmark.py --profile full
```

可限定范围或覆盖重复次数：

```bash
python scripts/run_benchmark.py --runs 3 \
  --algorithms frontier sstg_explorer \
  --environments maze warehouse
```

`--no-frames` 只跳过媒体，所有数值和轨迹仍保存。完整实验默认 225 次（5 算法 × 9 环境 × 5 次），默认每次运行都生成媒体。磁盘紧张时可使用 `--media-runs representative`，仅为每个组合的 run 0 生成媒体。

## 输出目录

```text
outputs/benchmark_runs/<timestamp>/
├── manifest.json            命令、参数、环境、Git commit/dirty 状态
├── run.log                  每一步运行日志和异常
├── results.json             全部实验及完整 trajectory
├── summary.csv              每个组合的均值和标准差
├── coverage_heatmap.png     覆盖率总览
├── index.html               自包含网页入口
└── artifacts/<env>/<algo>/
    ├── run.json             run 0 原始记录
    ├── steps/step_XXXX.png  每个新增观测节点后的截图
    ├── final.png            最终状态
    ├── animation.gif        浏览器内动画
    ├── video.mp4            run 0 最终视频
    └── runs/run_XXX/        其余重复运行的同套原始数据与媒体
```

`outputs/benchmark_runs/latest` 是最近结果的相对符号链接。建议通过 HTTP 打开，因为浏览器可能限制 `file://` 视频：

```bash
python -m http.server 8000 --directory outputs/benchmark_runs/latest
```

访问 `http://localhost:8000/`。远程服务器可用 SSH 端口转发：`ssh -L 8000:localhost:8000 <host>`。

## 公平性与复现

所有算法使用同一地图、起点、`r_view=2.0 m` 和 run seed。随机种子是 `42 + run_id`。环境构造参数和算法参数都写入 manifest。运行前保持 Git clean；若不干净，manifest 会完整记录差异文件。墙钟时间只在同一机器、空闲负载下比较。

结果核验顺序：先看 `run.log` 是否跑完；再检查 `results.json` 的记录数应为算法数 × 环境数 × runs；然后看 `summary.csv`；最后从热力图定位异常组合并在网页逐帧查看。低覆盖但 `success=true` 时应同时检查终止原因和 frontier 是否耗尽，不能把布尔 success 当作覆盖达标。

## 论文报告建议

主表报告 coverage、distance、nodes、time 的 mean ± std；安全/采样质量表报告 obstacle distance 与 uniformity。不要只挑选 SSTG 获胜环境。当前历史数据表明覆盖率不是 SSTG-Explorer 的全面优势，论文应将贡献定位为面向语义采样的安全、均匀节点布置，并明确密障碍和多房间失败案例。
