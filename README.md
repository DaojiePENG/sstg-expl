# SSTG-Explorer: 空间语义拓扑图探索算法

**Spatial Semantic Topological Graph Explorer** - 面向视觉语义采集的机器人探索算法

## 项目简介

SSTG-Explorer 是一个专门为空间语义拓扑图构建设计的探索算法。与传统的 SLAM 探索算法（如 RRT-based Explorer）不同，本算法针对视觉语义信息采集进行了优化，确保：

- ✅ **高覆盖密度**: 生成足够密集的采样点，满足视觉图像采集需求
- ✅ **全区域覆盖**: 覆盖所有可探索区域（目标 ≥95%）
- ✅ **避免冗余**: 节点间距保持在合理范围（r_view - overlap）
- ✅ **视野优化**: 探索点分布考虑机器人视野范围

## 核心特性

### 算法创新
- **全局候选点管理**: 维护所有历史位置的未探索方向，避免局部陷阱
- **自适应探索策略**: 根据覆盖率动态调整局部/全局探索权重
- **软障碍机制**: 已探索区域作为"软障碍"，后期可查漏补缺
- **方向聚类选择**: 智能选择最大连续无障碍方向的中心点

### 实现特点
- 🐍 纯 Python 实现，易于集成和扩展
- 📊 完整的 Benchmark 框架，支持多算法对比
- 🎨 丰富的可视化工具
- 🧪 全面的单元测试和集成测试
- 🔌 支持与导航系统灵活集成

## 快速开始

### 安装

```bash
# 克隆仓库
git clone <repository_url>
cd sstg-expl

# 安装依赖
pip install -r requirements.txt

# 或者安装为包
pip install -e .
```

### 基础使用

```python
from src.core.explorer import SSTGExplorer
from src.map.occupancy_grid import OccupancyGrid
from simulation.simple_env import EmptyRoom

# 创建环境
env = EmptyRoom(width=10.0, height=10.0)
occupancy_map = env.get_occupancy_map()

# 创建探索器
explorer = SSTGExplorer(
    r_view=2.0,      # 视野半径 2m
    d_theta=45.0,    # 角度间隔 45°
    overlap=0.25,    # 重叠距离 0.25m
    r_robot=0.3      # 机器人半径 0.3m
)

# 执行探索
result = explorer.explore(occupancy_map, start_pose=(5.0, 5.0, 0.0))

# 查看结果
print(f"探索节点数: {len(result['nodes'])}")
print(f"覆盖率: {result['metadata']['coverage_ratio']:.2%}")
print(f"总距离: {result['metadata']['total_distance']:.2f}m")
```

### 可视化

```python
from src.utils.visualization import visualize_exploration

# 可视化探索结果
visualize_exploration(
    occupancy_map=occupancy_map,
    explored_nodes=result['nodes'],
    r_view=2.0,
    save_path="exploration_result.png"
)
```

## 项目结构

```
sstg-expl/
├── src/                        # 源代码
│   ├── core/                   # 核心算法
│   │   ├── explorer.py         # 主探索器
│   │   ├── frontier.py         # Frontier 数据结构
│   │   ├── collision_checker.py # 碰撞检测
│   │   └── coverage_analyzer.py # 覆盖度分析
│   ├── map/                    # 地图模块
│   │   └── occupancy_grid.py   # 占据栅格地图
│   ├── planning/               # 路径规划
│   │   ├── local_planner.py    # 局部规划
│   │   └── astar.py            # A* 算法
│   ├── utils/                  # 工具模块
│   │   ├── geometry.py         # 几何计算
│   │   └── visualization.py    # 可视化
│   └── config.py               # 配置参数
├── simulation/                 # 仿真环境
│   ├── simple_env.py           # 简单环境
│   ├── apartment_env.py        # 公寓环境
│   └── benchmark.py            # Benchmark 框架
├── examples/                   # 示例脚本
│   ├── basic_exploration.py    # 基础示例
│   └── visualize_exploration.py # 可视化示例
├── tests/                      # 单元测试
├── benchmarks/                 # Benchmark 数据
│   ├── environments/           # 测试环境配置
│   └── results/                # 实验结果
└── docs/                       # 文档
```

## 算法原理

### 核心思想

SSTG-Explorer 基于全局 Frontier 管理的探索策略：

1. **Frontier 生成**: 在每个探索点的 r_view 半径上，按 d_theta 角度间隔生成候选方向
2. **碰撞检测**: 检测候选点是否与障碍物或已探索区域冲突
3. **优先级计算**: 基于距离、覆盖度、新颖性计算 frontier 优先级
4. **自适应探索**: 根据覆盖率动态调整局部/全局探索权重
5. **终止判断**: 当所有 frontiers 耗尽或覆盖率达标时终止

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `r_view` | 2.0 | 视野半径（米），决定采样密度 |
| `d_theta` | 45.0 | 角度间隔（度），决定方向数量 |
| `overlap` | 0.25 | 重叠距离（米），避免过度重叠 |
| `r_robot` | 0.3 | 机器人半径（米），碰撞检测用 |
| `alpha` | 2.0 | 距离衰减指数，控制局部性 |

**参数选择建议**:
- **r_view**: 根据相机视野和语义识别需求确定（通常 1.5-3.0m）
- **d_theta**: 8方向（45°）或 16方向（22.5°），更密集则计算量更大
- **overlap**: 0.2-0.3 倍 r_view，平衡覆盖与效率
- **alpha**: 前期用 2.0（局部优先），后期自动降低（全局补充）

## Benchmark

本项目提供完整的 benchmark 框架，对比以下算法：

| 算法 | 特点 | 适用场景 |
|------|------|---------|
| **SSTG-Explorer** | 全局管理、自适应、高覆盖 | 语义地图构建 |
| **RRT Explorer** | 随机采样、稀疏、边界偏好 | 传统 SLAM |
| **Frontier-based** | 基于占据栅格 frontier | 未知环境探索 |
| **Uniform Grid** | 均匀网格采样 | Baseline |
| **Next-Best-View** | 信息增益最大化 | 主动 SLAM |

### 评估指标

- **覆盖率**: 探索圆覆盖面积 / 自由空间面积
- **节点数**: 探索节点总数
- **节点密度**: 节点数 / 覆盖面积
- **移动距离**: 机器人总移动距离
- **探索时间**: 总探索时长（模拟）
- **节点间距**: 验证 overlap 约束

### 运行 Benchmark

```bash
python simulation/benchmark.py --env all --algorithms all
```

结果保存在 `benchmarks/results/` 目录。

## 与导航系统集成

### 方式 1: 文件输出

```python
# 探索并保存结果
result = explorer.explore(occupancy_map, start_pose)
import json
with open('exploration_nodes.json', 'w') as f:
    json.dump(result, f)

# 导航系统读取
with open('exploration_nodes.json', 'r') as f:
    nodes = json.load(f)
# 进行语义标注...
```

### 方式 2: API 调用

```python
from sstg_expl import SSTGExplorer

# 在导航系统中直接调用
explorer = SSTGExplorer(r_view=2.0, d_theta=45, overlap=0.25)
nodes = explorer.explore(map_data, start_pose)

for node in nodes:
    image = capture_image(node['position'])
    semantic_info = vlm.analyze(image)
    graph.add_node(node, semantic_info)
```

### 方式 3: ROS2 集成（可选）

```python
# 订阅地图，发布探索目标
# 详见 docs/ros_integration.md
```

## 测试

```bash
# 运行所有测试
pytest tests/

# 运行特定测试
pytest tests/test_explorer.py

# 生成覆盖率报告
pytest --cov=src tests/
```

## 贡献指南

欢迎贡献！请遵循以下规范：

- 代码风格: Black formatter + Pylint
- 类型提示: 使用 Type hints (Python 3.8+)
- 文档字符串: Google style
- 提交前运行测试: `pytest tests/`

## 许可证

MIT License

## 引用

如果本项目对你的研究有帮助，欢迎引用：

```bibtex
@software{sstg_explorer,
  title = {SSTG-Explorer: Spatial Semantic Topological Graph Explorer},
  author = {SSTG Team},
  year = {2026},
  url = {<repository_url>}
}
```

## 联系方式

- Issues: <repository_url>/issues
- Email: <your_email>

## 相关项目

- 空间语义拓扑图导航系统 (SSTG-Nav)
- 视觉语言模型语义识别 (VLM-Semantic)

---

**Status**: 🚧 Active Development | **Version**: 0.1.0 | **Last Updated**: 2026-04-15
