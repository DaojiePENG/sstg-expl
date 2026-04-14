# SSTG-Explorer 快速开始指南

## 安装依赖

```bash
cd /home/daojie/sstg-expl
pip install -r requirements.txt
```

## 快速运行示例

### 1. 运行测试

```bash
python tests/test_basic.py
```

### 2. 运行基础探索示例

```bash
python examples/basic_exploration.py
```

这将在空房间和带障碍物的房间中运行探索，并显示可视化结果。

### 3. 快速测试（命令行）

```python
from src.core.explorer import SSTGExplorer
from simulation.simple_env import EmptyRoom

# 创建环境
env = EmptyRoom(width=10.0, height=10.0)
grid = env.get_occupancy_map()
start = env.get_start_pose()

# 运行探索
explorer = SSTGExplorer(r_view=2.0, d_theta=45.0, overlap=0.25)
result = explorer.explore(grid, start)

# 查看结果
print(f"覆盖率: {result['metadata']['coverage_ratio']:.1%}")
print(f"节点数: {len(result['nodes'])}")
```

## 自定义参数

```python
from src.config import ExplorerConfig

config = ExplorerConfig(
    r_view=2.0,          # 视野半径（米）
    d_theta=45.0,        # 角度间隔（度）
    overlap=0.25,        # 重叠距离（米）
    r_robot=0.3,         # 机器人半径（米）
    target_coverage=0.95, # 目标覆盖率
    adaptive_alpha=True  # 自适应探索策略
)

explorer = SSTGExplorer(config=config)
```

## 可用环境类型

- `'empty'`: 空房间
- `'obstacles'`: 带随机障碍物的房间
- `'corridor'`: 狭长走廊
- `'multiple_rooms'`: 多房间环境
- `'apartment'`: 复杂公寓布局

```python
from simulation.simple_env import create_environment

env = create_environment('apartment', width=12.0, height=12.0)
```

## 可视化

```python
from src.utils.visualization import visualize_exploration

visualize_exploration(
    occupancy_grid=grid,
    explored_nodes=result['nodes'],
    r_view=2.0,
    show_coverage=True,
    save_path='exploration_result.png'
)
```

## 输出格式

探索结果包含：

```python
{
    'nodes': [
        {'id': 0, 'position': (x, y), 'orientation': theta, 'timestamp': t},
        ...
    ],
    'metadata': {
        'r_view': 2.0,
        'overlap': 0.25,
        'd_theta': 45.0,
        'coverage_ratio': 0.96,
        'total_distance': 125.3,
        'total_time': 89.2,
        'num_nodes': 50,
        'min_node_distance': 1.75,
        'mean_node_distance': 3.2,
        'coverage_uniformity': 0.15
    },
    'success': True
}
```

## 与导航系统集成

### 方式1: JSON 文件

```python
import json

# 保存探索结果
with open('exploration_nodes.json', 'w') as f:
    json.dump(result, f, indent=2)

# 导航系统读取
with open('exploration_nodes.json', 'r') as f:
    nodes = json.load(f)
```

### 方式2: 直接API调用

```python
# 在导航系统中
from sstg_expl.src.core.explorer import SSTGExplorer

explorer = SSTGExplorer(r_view=2.0, d_theta=45.0, overlap=0.25)
result = explorer.explore(map_data, start_pose)

for node in result['nodes']:
    position = node['position']
    # 采集图像并进行语义识别
    image = capture_image(position)
    semantic_info = vlm.analyze(image)
    # 添加到空间语义拓扑图
    graph.add_node(node, semantic_info)
```

## 项目结构

```
sstg-expl/
├── src/                    # 源代码
│   ├── core/              # 核心算法
│   ├── map/               # 地图模块
│   ├── utils/             # 工具函数
│   └── config.py          # 配置
├── simulation/            # 仿真环境
├── examples/              # 示例脚本
├── tests/                 # 测试
└── benchmarks/            # Benchmark（待实现）
```

## 下一步

1. 查看 `SSTG-Expl-Plan.md` 了解详细的算法设计
2. 查看 `README.md` 了解项目全貌
3. 运行 `examples/basic_exploration.py` 查看更多示例
4. 实现 benchmark 框架进行算法对比（待实现）

## 常见参数调优建议

| 场景 | r_view | d_theta | overlap |
|------|--------|---------|---------|
| 大范围快速探索 | 2.5-3.0m | 45° | 0.3-0.5m |
| 标准密度采集 | 2.0m | 45° | 0.25m |
| 高密度语义采集 | 1.5m | 22.5° | 0.2m |
| 狭窄通道 | 1.5m | 22.5° | 0.2m |

## 问题排查

### 覆盖率不达标
- 减小 `r_view`
- 减小 `d_theta`（增加方向数）
- 减小 `overlap`

### 节点过多（冗余）
- 增大 `r_view`
- 增大 `overlap`
- 增大 `d_theta`

### 探索时间过长
- 增大 `r_view`
- 调整 `target_coverage`（如从0.95改为0.90）
- 增大 `min_priority_threshold`

---

**当前版本**: 0.1.0 (基础功能完成)  
**待实现**: Benchmark框架、ROS2集成、多机器人协同
