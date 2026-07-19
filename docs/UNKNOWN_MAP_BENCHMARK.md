# 未知静态二维栅格：遮挡感知传感器与在线探索 Benchmark

本协议与已知地图 benchmark 并列，不能混表。环境 ground truth 只由仿真传感器、碰撞审计和最终评价访问；所有算法在时刻 (t) 只能读取累计 belief map (B_t\in\{-1,0,100\}^{H\times W})，其中 `-1` 表示未知。

## 1. 传感器矩阵

正式 `paper` profile 使用六个正交敏感性配置：

| Key | 水平 FOV | 最大量程 | 目的 |
|---|---:|---:|---|
| `fov360_r8` | 360° | 8 m | 较短量程旋转式 LiDAR |
| `fov360_r12` | 360° | 12 m | 主 LiDAR 协议 |
| `fov360_r16` | 360° | 16 m | 长量程敏感性 |
| `fov240_r12` | 240° | 12 m | 宽视角定向传感器 |
| `fov120_r12` | 120° | 12 m | 广角相机/固态 LiDAR proxy |
| `fov90_r12` | 90° | 12 m | 前向深度相机 proxy |

角分辨率统一为 0.25°，射线步长为 0.5 个栅格。每条射线遇到第一个 occupied cell 即停止；障碍表面可见，墙后保持未知。机器人沿实际 A* 路径每 1 m 扫描一次，并在目标姿态再次扫描，因此不是“到离散点后瞬时画圆”。

参数不是对某一硬件的精确复刻，而是覆盖真实传感器量级的 controlled sweep：Hokuyo UST-10/20LX 官方规格给出 270°、10/20 m、0.25°；SLAMTEC RPLIDAR S2 为 360°，不同反射率下覆盖约 8–30 m；Luxonis OAK-D Pro 的双目相机 HFOV 约 80°、理想深度范围约 0.8–12 m。参见 [Hokuyo 产品规格](https://www.hokuyo-aut.jp/search/single.php?serial=167)、[RPLIDAR S2 官方规格](https://www.slamtec.com/en/s2)、[OAK-D Pro 官方文档](https://docs.luxonis.com/hardware/products/OAK-D%20Pro)。90°/120° 与 240° 是相机式、广角式和宽视角式的论文敏感性点，不应写成上述设备的额定值。

## 2. 未知地图状态与安全执行

1. 初始 belief 全未知，只在 start pose 执行第一帧观测。
2. 策略候选、信息增益、frontier、clearance 和 geodesic cost 只从 belief 计算。
3. A* 只允许机器人完整 footprint 已观测为 free 的中心栅格；实现等价于以 `r_robot=0.3 m` 腐蚀 known-free mask。
4. `0.5 m` 是 viewpoint/path 安全率与 clearance utility 的偏好阈值，不是把 0.3 m 机器人错误扩成 0.5 m 的硬尺寸。未知 cell 不可穿越，robot footprint 也不可压到 unknown 上。
5. ground truth 只用于 ray casting、结果 coverage 和事后安全审计。

cost map 的 reachable set 使用四连通 start component，实际 A* 使用禁止斜穿墙角的八连通搜索；因而被标记为 reachable 的 candidate 一定具有合法执行路径。每条实际路径首点保留真实 current pose，不以栅格中心替代。

frontier center 位于与 unknown 相距至多 `r_robot + 2 cells = 0.4 m` 的 footprint-safe band；`target_spacing=2 m` 只用于 SSTG 的 farthest-point 离散采样和 spacing utility，不用来人为加厚经典 Frontier 的边界。

地图画布尺寸对算法已知，栅格内容未知。动态扩张地图、定位漂移、传感器噪声和动态障碍不在本协议内。

## 3. 在线对比方法

| CLI | 网页名 | 未知协议适配 | 论文依据 |
|---|---|---|---|
| `frontier` | Frontier-Unknown | known-free/unknown 边界聚类，选择最近可达 frontier | Yamauchi 1997；Holz et al. 2011 |
| `nbv` | NBV-Unknown | 对 frontier 与已知安全采样点计算遮挡感知未知增益–路径代价 | Connolly 1985；Bircher et al. 2016 |
| `rrt` | RRT-Unknown (adapted) | 在当前可达 known-free 区域随机采样，按增益/代价随机化选择；不是完整 multi-RRT ROS 系统 | LaValle 1998；Umari and Mukhopadhyay 2017 |
| `ans` | ANS-Global Unknown (adapted) | 公开 pretrained global policy 只读取 belief occupancy/explored channels，共同 A* 执行 | Chaplot et al., ICLR 2020 |
| `sstg` | SSTG-Explorer Unknown | 多代表 frontier、确定性拓扑视点、有向转向、测地代价、净空、视点间距与未知增益联合排序 | 本文；测地执行依据 Hart et al. 1968 |

Uniform Grid 需要预先知道全地图格点布局，不满足 unknown-input invariant，因此不放入未知主表；它仍保留在已知地图主表。所有 `(adapted)` 名称均表示公共二维协议适配，不能冒充原论文完整传感器/局部控制栈。

全部 BibTeX 在 [REFERENCES.bib](REFERENCES.bib)。ANS 的作者项目页明确提供 PyTorch 实现和预训练模型；DRL-Graph/LSP/Habitat 类方法因 belief graph、定位不确定性、输入模态或训练分布不一致，只列 related work，不用未经作者 checkpoint 验证的简化器冒充学习基线。

## 4. 评价指标

- `coverage_ratio`：ground-truth free cells 中已经被正确观测为 free 的比例。
- `known_ratio`：整个地图中不再是 unknown 的比例。
- `occupied_recall`：ground-truth occupied cells 被观测到的比例。
- `total_distance`、`total_rotation_deg`、`scan_count` 和 `in_place_rotations`。
- 视点/路径障碍净空、边界净空和安全比例，与已知协议相同。
- `mean/median/min_nn_distance`：每个视点到最近其他视点的距离，越大表示空间上更离散。
- `redundant_viewpoint_fraction`：除起点外，与任一历史视点距离小于 1 m 的决策视点比例；原地换朝向会计入空间冗余，同时由 `in_place_rotations` 单列解释。
- `coverage_per_viewpoint`：最终 free coverage / oriented viewpoint 数量。
- `dispersion_uniformity`：最近邻距离变异系数导出的均匀性；必须与平均间距共同解释。

“平均间距大”不能单独证明方法好：如果 coverage 失败，少量远隔点也能产生很大的间距。因此论文应联合报告 coverage、节点数、平均最近邻距离、冗余率和 coverage/view。

只有一个 oriented viewpoint 的 run 不存在 nearest neighbor；raw metric 以 `nn_metric_defined=0` 标记，sensor/macro NN 汇总会排除该 run，而不会把未定义间距伪装成 0 m。冗余率仍定义为 0，因为没有回访决策。

## 5. 运行

```bash
conda activate sstg-explorer

# 2 sensors × 2 hard scenes × 5 methods × 1 run
python scripts/run_unknown_benchmark.py --profile smoke

# 6 sensors × 9 scenes × 5 methods × 3 seeds = 810 runs
python scripts/run_unknown_benchmark.py --profile paper

# 单独跑角度或量程敏感性
python scripts/run_unknown_benchmark.py --profile fov --no-frames
python scripts/run_unknown_benchmark.py --profile range --no-frames

# 5 SSTG variants × 4 hard scenes × 3 sensor settings × 3 seeds = 180 runs
python scripts/run_unknown_benchmark.py --profile ablation --no-frames
```

`paper` 默认对每个 sensor–environment–algorithm 的 run 0 生成完整逐步 PNG/GIF/MP4，其余 seeds 保存同样完整的数值 trace、belief updates 和 `belief_final.npy`。需要所有 seeds 的媒体时加 `--media-runs all`。

`ablation` 比较 Full、single frontier centroid、known-obstacle-only safety、no topological vantages 与 no spacing utility。前两项直接复现 dense/warehouse 的已定位失败机制；后两项检验 SSTG 的空间离散性贡献。消融默认不生成媒体，上述命令显式使用 `--no-frames`，但所有数值 trace 仍保留。

主实验默认写入 `outputs/unknown_benchmark_runs/`，消融默认写入独立的 `outputs/unknown_ablation_runs/`，各自维护 `latest`，不会互相覆盖网页入口。

## 6. 输出与图像解释

```text
outputs/unknown_benchmark_runs/<timestamp>/
├── manifest.json
├── results.json                         轻量全局索引
├── summary.csv / aggregate*.csv
├── results_table.{md,tex}
├── pairwise_vs_sstg_unknown.csv
├── audit_report.json                   数量、重放、truth、媒体和 HTML 审计
├── sensor_coverage_heatmap.png
├── fov_range_sensitivity.png
├── safety_redundancy_tradeoff.png
├── known_map_redundancy_{supplement.csv,table.md}
├── index.html
└── artifacts/<sensor>/<environment>/<algorithm>/
    ├── run.json                         完整候选与 belief 更新流
    ├── belief_final.npy
    ├── decisions.csv / candidates.csv
    ├── trajectory.csv / path_waypoints.csv / scan_poses.csv
    ├── steps/step_XXXX.png
    ├── final.png
    ├── animation.gif
    └── video.mp4
```

逐步图左侧是算法真正可见的 belief；右侧 ground truth 只用于评价，红色表示尚未观测的真实 free space。蓝线为实际 A* 轨迹，虚线圆/扇形为当前传感器量程和 FOV。每个黄色 pending 点带朝向箭头，大小反映 priority；绿色为新候选，橙/灰叉为低增益或预算剪枝，粉色星形与箭头为 selected viewpoint。右侧文字给出 top-3 pending 的 ID、kind、gain、priority 和 geodesic distance；全部候选数值在 `run.json`。

`observed_updates` 使用 `[flat_index, occupancy_value]` 增量流保存。任意 step 的 belief 都能从全未知数组顺序重放得到，不需要在 JSON 中重复保存整幅地图。

runner 结束前会逐 run 重放 `observed_updates` 并与 `belief_final.npy` 比较，再核验所有已知 belief cell 与 truth 一致、媒体帧数等于 decision step 数、HTML 引用存在且日志无错误；任一项失败会写出 `audit_report.json` 并以非零状态退出。

完整 observation、belief update、footprint-safe set、SSTG utility、冗余率公式与在线伪代码见 [RAL 公式参考](PAPER_WRITING_REFERENCE.md#11-未知地图在线-sstg-explorer)。
