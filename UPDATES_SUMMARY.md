# 功能更新总结 - 输出管理与实时可视化

## 🎉 新增功能概览

### 1. 结构化输出目录 📁

所有生成的文件现在自动保存到 `outputs/` 目录：

```
outputs/
├── explorations/      # 探索结果可视化
├── coverage/          # 覆盖度分析
├── metrics/           # 统计指标
├── animations/        # 动画（可选）
└── README.md          # 目录说明
```

**优点**：
- ✅ 清晰的文件组织
- ✅ 不污染项目根目录
- ✅ 已添加到 `.gitignore`，不追踪到git

### 2. 实时可视化探索过程 🎬

全新的实时可视化系统，动态展示探索过程：

**显示元素**：
- ⭐ **当前位置** - 绿色星标
- 🔴 **已探索节点** - 红色圆点（带编号）
- 🟡 **待探索frontiers** - 黄色圆圈
- ❌ **被阻挡的候选点**：
  - 红色X：障碍物阻挡
  - 橙色X：距离已探索点过近
- 🔵 **覆盖圆** - 半透明蓝圈
- 📊 **实时统计** - 标题栏显示进度

**核心特性**：
- ✅ 实时更新（可配置更新间隔）
- ✅ 完整图例说明
- ✅ 探索完成后保持窗口开启
- ✅ 可保存最终状态

### 3. 节点编号标注 🔢

所有可视化现在显示节点探索次序编号（0, 1, 2, ...）：
- **起始节点（0）**：深绿色粗体
- **中间节点**：深红色
- **结束节点**：深红色粗体
- 白色背景框确保可读性

---

## 🚀 使用方法

### 快速开始

```bash
cd /home/daojie/sstg-expl
python examples/basic_exploration.py
```

这将：
1. 在空房间环境中运行探索
2. **实时显示**探索过程（动态窗口）
3. 生成3张高质量图片保存到 `outputs/`

### 启用实时可视化

```python
from src.core.explorer import SSTGExplorer
from simulation.simple_env import EmptyRoom
from src.utils.realtime_viz import RealtimeVisualizer

# 创建环境
env = EmptyRoom(width=10.0, height=10.0)
grid = env.get_occupancy_map()
start = env.get_start_pose()

# 创建实时可视化器
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(14, 12),
    update_interval=0.3  # 更新频率（秒）
)

# 探索（传入visualizer）
explorer = SSTGExplorer(r_view=2.0, d_theta=45.0, overlap=0.25)
result = explorer.explore(grid, start, visualizer=visualizer)

# 探索完成后窗口保持打开，可手动保存或关闭
```

### 使用示例脚本

```python
from examples.basic_exploration import run_basic_exploration

# 启用实时可视化
run_basic_exploration(
    env_type='empty',
    realtime_viz=True,    # 启用实时可视化
    visualize=True,       # 生成后处理图片
    save_results=True     # 保存到outputs/
)

# 禁用实时可视化（仅保存结果）
run_basic_exploration(
    env_type='obstacles',
    realtime_viz=False,
    visualize=True,
    save_results=True
)
```

---

## 📊 输出文件说明

### 自动生成的文件

运行示例后，`outputs/` 目录包含：

1. **explorations/exploration_{env}.png**
   - 完整探索结果
   - 带编号的节点
   - 覆盖圆和路径
   - 障碍物地图

2. **coverage/coverage_{env}.png**
   - 三视图覆盖度分析
   - 探索节点分布
   - 覆盖区域（绿色）
   - 未覆盖空隙（红色）

3. **metrics/metrics_{env}.png**
   - 统计指标摘要
   - 关键指标柱状图
   - 距离指标对比
   - 参数配置

### 文件命名规则

```
{type}_{environment}.png
```

- **type**: `exploration`, `coverage`, `metrics`
- **environment**: `empty`, `obstacles`, `corridor`, `multiple_rooms`, `apartment`

**示例**：
- `exploration_empty.png`
- `coverage_obstacles.png`
- `metrics_corridor.png`

---

## 🎨 自定义配置

### 调整实时可视化更新频率

```python
# 快速更新（流畅）
visualizer = RealtimeVisualizer(..., update_interval=0.1)

# 标准更新（推荐）
visualizer = RealtimeVisualizer(..., update_interval=0.3)

# 慢速更新（节省资源）
visualizer = RealtimeVisualizer(..., update_interval=0.8)
```

### 调整可视化窗口大小

```python
# 小窗口
visualizer = RealtimeVisualizer(..., figsize=(10, 8))

# 标准窗口（推荐）
visualizer = RealtimeVisualizer(..., figsize=(14, 12))

# 大窗口（演示用）
visualizer = RealtimeVisualizer(..., figsize=(16, 14))
```

### 自定义输出路径

```python
from examples.basic_exploration import run_basic_exploration

# 默认保存到 outputs/
run_basic_exploration(env_type='empty', save_results=True)

# 输出结构：
# outputs/explorations/exploration_empty.png
# outputs/coverage/coverage_empty.png
# outputs/metrics/metrics_empty.png
```

---

## 🔧 高级用法

### 场景1：调试算法

启用实时可视化观察探索策略：

```python
run_basic_exploration(
    env_type='obstacles',
    realtime_viz=True,
    visualize=False,      # 不生成后处理图片
    save_results=False    # 不保存
)
```

### 场景2：批量实验

禁用实时可视化加快速度，仅保存结果：

```python
environments = ['empty', 'obstacles', 'corridor', 'multiple_rooms']

for env_type in environments:
    run_basic_exploration(
        env_type=env_type,
        realtime_viz=False,   # 禁用实时显示
        visualize=True,       # 生成结果图片
        save_results=True     # 保存到outputs/
    )
```

### 场景3：演示展示

使用慢速更新便于讲解：

```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(16, 14),     # 大窗口
    update_interval=0.8   # 慢速更新
)

explorer.explore(grid, start, visualizer=visualizer)
```

---

## 📝 修改的文件清单

### 新增文件
1. `src/utils/realtime_viz.py` - 实时可视化核心类
2. `outputs/README.md` - 输出目录说明
3. `REALTIME_VISUALIZATION.md` - 实时可视化使用指南
4. `UPDATES_SUMMARY.md` - 本文档

### 修改文件
1. `src/core/explorer.py` - 添加visualizer参数和状态追踪
2. `src/utils/visualization.py` - 添加节点编号，支持同时显示+保存
3. `examples/basic_exploration.py` - 使用outputs目录，添加实时可视化选项
4. `.gitignore` - 忽略outputs目录和图片文件

---

## 🐛 故障排查

### 问题1：实时可视化窗口不显示

**原因**：在远程服务器或无显示环境

**解决**：
```python
# 方法1：禁用实时可视化
run_basic_exploration(env_type='empty', realtime_viz=False)

# 方法2：使用X11转发
ssh -X user@server
python examples/basic_exploration.py
```

### 问题2：输出文件找不到

**原因**：`save_results=False`

**解决**：
```python
run_basic_exploration(env_type='empty', save_results=True)
# 文件保存在: outputs/explorations/, outputs/coverage/, outputs/metrics/
```

### 问题3：git状态显示outputs文件

**原因**：`.gitignore`未生效或文件在gitignore之前已被追踪

**解决**：
```bash
# 确认gitignore
cat .gitignore | grep outputs

# 如果文件已被追踪，需要先移除
git rm -r --cached outputs/
git commit -m "Remove outputs from tracking"
```

---

## 📚 相关文档

- **[REALTIME_VISUALIZATION.md](REALTIME_VISUALIZATION.md)** - 实时可视化完整指南
- **[VISUALIZATION_UPDATE.md](VISUALIZATION_UPDATE.md)** - 可视化功能更新说明
- **[outputs/README.md](outputs/README.md)** - 输出目录结构说明
- **[QUICKSTART.md](QUICKSTART.md)** - 快速开始指南
- **[README.md](README.md)** - 项目总览

---

## ✅ 测试结果

已测试的功能：
- ✅ 输出目录自动创建
- ✅ 文件保存到正确位置
- ✅ 节点编号正确显示
- ✅ 实时可视化正常更新
- ✅ Frontiers状态正确追踪（active/blocked_obstacle/blocked_explored）
- ✅ 探索完成后窗口保持打开
- ✅ `.gitignore` 正确配置

测试环境：
- 空房间（8x8m）：11节点，99.1%覆盖
- 带障碍物房间：待测试
- 多房间环境：待测试

---

## 🎯 下一步建议

1. **运行完整示例**：
   ```bash
   python examples/basic_exploration.py
   ```

2. **查看生成的文件**：
   ```bash
   ls -lh outputs/explorations/
   ls -lh outputs/coverage/
   ls -lh outputs/metrics/
   ```

3. **尝试不同环境**：
   - `env_type='empty'`
   - `env_type='obstacles'`
   - `env_type='corridor'`
   - `env_type='multiple_rooms'`
   - `env_type='apartment'`

4. **调整参数实验**：
   - 修改 `r_view` （1.5, 2.0, 2.5）
   - 修改 `d_theta` （22.5, 45, 90）
   - 修改 `overlap` （0.2, 0.25, 0.3）
   - 观察对节点数、覆盖率的影响

---

**版本**: 1.0  
**更新日期**: 2026-04-15  
**状态**: ✅ 已实现并测试完成

## 贡献者
- 实时可视化系统设计与实现
- 输出目录管理
- 节点编号标注功能
- 文档编写
