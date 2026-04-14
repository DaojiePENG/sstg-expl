# 可视化功能更新说明

## ✨ 新增功能

### 1. 节点编号显示
现在所有的探索节点都会显示其探索次序编号（0, 1, 2, ...）：
- **起始节点（0号）**: 深绿色粗体，更醒目
- **中间节点**: 深红色标注
- **终止节点**: 深红色粗体
- 所有编号都有白色背景框，确保可读性

### 2. 同时显示和保存
所有可视化函数现在支持在保存图片的同时也显示在屏幕上：
- `visualize_exploration()`: 探索结果主视图
- `visualize_coverage_map()`: 覆盖度分析（3视图）
- `plot_exploration_metrics()`: 统计指标图

## 🚀 使用方法

### 方式1: 使用示例脚本（推荐）

```bash
cd /home/daojie/sstg-expl
python examples/basic_exploration.py
```

**默认行为**（已修改）：
- ✅ 显示可视化窗口
- ✅ 保存图片文件（自动）
- ✅ 节点编号标注

**生成的文件**：
- `exploration_empty.png` - 探索结果（带编号）
- `coverage_empty.png` - 覆盖度分析
- `metrics_empty.png` - 统计指标

### 方式2: 直接调用API

```python
from src.core.explorer import SSTGExplorer
from simulation.simple_env import EmptyRoom
from src.utils.visualization import visualize_exploration

# 探索
env = EmptyRoom(width=10.0, height=10.0)
grid = env.get_occupancy_map()
start = env.get_start_pose()

explorer = SSTGExplorer(r_view=2.0, d_theta=45.0, overlap=0.25)
result = explorer.explore(grid, start)

# 可视化（同时显示和保存，带节点编号）
visualize_exploration(
    occupancy_grid=grid,
    explored_nodes=result['nodes'],
    r_view=2.0,
    show_coverage=True,        # 显示覆盖圆
    show_connections=True,     # 显示路径连线
    save_path='my_result.png'  # 保存路径
)
# 会同时显示窗口并保存到 my_result.png
```

### 方式3: 仅显示不保存

```python
visualize_exploration(
    occupancy_grid=grid,
    explored_nodes=result['nodes'],
    r_view=2.0,
    save_path=None  # 不保存，仅显示
)
```

## 📊 可视化说明

### 主探索结果图 (exploration_*.png)

包含以下元素：
1. **地图背景**: 灰色=自由空间，黑色=障碍物
2. **覆盖圆**: 浅蓝色透明圆（可选，show_coverage=True）
3. **路径连线**: 蓝色虚线连接探索轨迹（可选，show_connections=True）
4. **节点标记**:
   - 绿色圆圈：起始点
   - 红色圆点：探索节点
   - 红色方块：结束点
5. **🆕 节点编号**: 白色背景框内的数字标注

**图例解读**：
- 数字0（绿色粗体）: 探索起点
- 数字1-N（红色）: 探索次序
- 数字N（红色粗体）: 探索终点

### 覆盖度分析图 (coverage_*.png)

三个子图：
1. **左图**: 探索节点分布
2. **中图**: 覆盖区域（绿色=已覆盖）
3. **右图**: 覆盖空隙（红色=未覆盖，蓝色X=空隙中心）

### 统计指标图 (metrics_*.png)

四个子图：
1. **左上**: 文本指标摘要
2. **右上**: 关键指标柱状图
3. **左下**: 距离指标对比
4. **右下**: 参数配置

## 🎨 自定义选项

### 调整图片大小和质量

```python
visualize_exploration(
    occupancy_grid=grid,
    explored_nodes=result['nodes'],
    r_view=2.0,
    figsize=(16, 12),  # 更大的图片
    dpi=150,           # 更高的分辨率
    save_path='high_quality.png'
)
```

### 关闭覆盖圆和连线

```python
visualize_exploration(
    occupancy_grid=grid,
    explored_nodes=result['nodes'],
    r_view=2.0,
    show_coverage=False,    # 不显示覆盖圆
    show_connections=False, # 不显示连线
    save_path='simple_view.png'
)
```

## 📝 示例输出

运行 `python examples/basic_exploration.py` 后：

```
============================================================
SSTG Explorer - Basic Exploration Example
============================================================

1. Creating 'empty' environment...
   Environment size: 10.0m x 10.0m
   Resolution: 0.05m
   Start pose: (5.00, 5.00, 0.0°)

2. Creating SSTG Explorer...
   r_view: 2.0m
   d_theta: 45.0°
   overlap: 0.25m

3. Running exploration...
------------------------------------------------------------
Starting exploration from (5.0, 5.0, 0.0)
Parameters: r_view=2.0m, d_theta=45.0°, overlap=0.25m
Iteration 10: 11 nodes, coverage=99.1%, frontiers=34
Coverage target reached: 99.1%

Exploration complete!
Nodes: 11
Coverage: 99.1%
Distance: 39.27m
Time: 12.31s
------------------------------------------------------------

4. Exploration Results:
   Success: True
   Number of nodes: 11
   Coverage ratio: 99.1%
   Total distance: 39.27m
   Exploration time: 12.31s
   Min node distance: 1.75m
   Mean node distance: 3.21m

5. Generating visualizations...
   Results will be saved with prefix: 'empty'
Figure saved to exploration_empty.png
[显示窗口]
Coverage analysis saved to coverage_empty.png
[显示窗口]
Metrics plot saved to metrics_empty.png
[显示窗口]

   ✓ All visualizations saved!

============================================================
Exploration complete!
============================================================
```

## 🔧 故障排查

### 问题1: 窗口不显示
在远程服务器或无显示环境下，可能看不到窗口，但文件仍会正常保存。

**解决方案**: 使用非交互式后端
```python
import matplotlib
matplotlib.use('Agg')  # 在导入pyplot之前
import matplotlib.pyplot as plt
```

### 问题2: 节点编号重叠
节点过于密集时编号可能重叠。

**解决方案**: 增大视野半径或减小图片尺寸
```python
# 方法1: 增大 r_view
explorer = SSTGExplorer(r_view=2.5, d_theta=45, overlap=0.25)

# 方法2: 增大图片尺寸
visualize_exploration(..., figsize=(16, 14), dpi=150)
```

### 问题3: 保存路径错误
如果指定的目录不存在会报错。

**解决方案**: 使用相对路径或确保目录存在
```python
import os
os.makedirs('results', exist_ok=True)
visualize_exploration(..., save_path='results/my_exploration.png')
```

## 📚 更多示例

查看以下文件了解更多用法：
- `examples/basic_exploration.py` - 完整示例
- `QUICKSTART.md` - 快速开始指南
- `README.md` - 项目文档

## 🎯 与导航系统集成

生成的编号图片可以用于：
1. **文档记录**: 清晰展示探索顺序
2. **调试分析**: 识别探索策略问题
3. **结果演示**: 向团队展示算法性能
4. **论文配图**: 用于学术论文的图表

---

**更新日期**: 2026-04-15  
**功能状态**: ✅ 已完成并测试
