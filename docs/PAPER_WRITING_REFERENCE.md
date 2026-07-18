# IEEE RAL 写作参考与数据口径

## 目前可以说什么

正式结果目录 `outputs/benchmark_runs/20260719_012029/` 含 5 算法、9 环境、每组合 5 次，共 225 条记录。SSTG-Explorer 的环境宏平均覆盖率为 81.82%，平均路径为 31.84 m；Frontier 对应为 97.22% 和 41.12 m。因此不能写 “SSTG-Explorer outperforms all baselines in coverage”。可写成：SSTG-Explorer 在走廊达到 100% 覆盖，在迷宫和窄通道超过 95%，并以较短的宏平均路径生成规则观测节点；但其多房间（34.17%）和密障碍（31.26%）覆盖明显退化，跨环境覆盖鲁棒性落后于 Frontier。

空间指标以同一正式结果的 `results.json` 为准。安全距离优势可讨论，但均匀性差异很小时，应报告统计检验/置信区间后再声称显著性。

## 摘要模板（用新实验替换方括号）

Spatial-semantic topological mapping requires viewpoints that jointly provide free-space coverage, safe clearance, and regular visual overlap. We present SSTG-Explorer, a global-candidate exploration method that combines distance-aware frontier prioritization, reachability validation, and passage-adaptive angular sampling. Across nine simulated occupancy-grid environments and four baselines, SSTG-Explorer achieves [coverage] while maintaining [clearance] and [uniformity]. Results expose a trade-off between viewpoint quality and coverage robustness, particularly in [failure environments]. Code, seeds, trajectories, step-wise renderings, and videos are released for reproducibility.

## 实验统计检查表

- 明确实验单位是一次 algorithm–environment–seed 运行；不要把节点当独立样本。
- 报告 mean ± std，同时给出 95% bootstrap CI 或配对非参数检验。
- 多算法、多环境比较应做多重比较校正（如 Holm）。
- 明确失败运行是否进入均值；推荐全部保留，并单独报告 success rate。
- 超参数选择与最终评估应分离；若使用相同九环境调参和测试，称为 benchmark evaluation 而不是泛化验证。
- 运行时间注明 CPU、内存、OS、Python、单/多线程及是否包含可视化。
- 图中误差条和表格小数位保持一致。

## RAL 稿件执行清单

- 从 IEEE Robotics and Automation Letters 官方作者页面下载投稿时的最新模板与页数规则；规则会变化，不在仓库硬编码。
- 摘要避免引用、缩写堆砌和没有量化的 “significantly”。
- 每条贡献都在方法或实验中有对应证据。
- 主文展示失败案例，补充材料提供全部网页/视频。
- 图中文字在双栏缩放后仍可读，优先矢量 PDF。
- 不把仿真中的 `success` 布尔值等同于真实任务成功。
- 投稿前核验匿名/非匿名要求、补充视频格式、代码链接可用性和数据许可。

## 结果生成与引用来源

正式数字只从新运行目录的 `results.json` 和 `summary.csv` 生成，网页只用于检查轨迹，不作为统计源。每次��表��记录 `manifest.json` 中的 Git commit 和时间戳。旧根目录报告仅是开发历史，整理后不再作为论文依据。
