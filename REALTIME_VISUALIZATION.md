# 实时可视化功能使用指南

## 🎯 功能概述

SSTG-Explorer 现在支持实时可视化探索过程，可以动态展示：

- ⭐ **当前位置**（大型绿色星标）
- 🔴 **已探索节点**（红色圆点+编号）
- 🟡 **待探索frontier**（黄色圆圈）
- ❌ **被障碍物阻挡的候选点**（红色X）
- 🟠 **距离已探索节点过近的候选点**（橙色X）
- 🔵 **覆盖圆**（半透明蓝色圆）
- 📈 **实时统计**（标题栏显示迭代次数、节点数、覆盖率、frontiers数量）
- 🎬 **动画保存**（可将整个探索过程保存为GIF动画）

## 🚀 快速开始

### 方式1: 使用示例脚本（推荐）

#### 基本用法（仅实时显示）
```bash
cd /home/daojie/sstg-expl
python examples/basic_exploration.py
```

#### 保存动画
```python
from examples.basic_exploration import run_basic_exploration

# 启用实时可视化 + 动画保存
run_basic_exploration(
    env_type='empty',
    realtime_viz=True,      # 实时显示窗口
    save_animation=True     # 保存为GIF动画
)
```

默认会在空房间环境中启用实时可视化，并将动画保存到 `outputs/animations/animation_empty.gif`。

### 方式2: 代码中启用

#### 基本实时可视化（不保存动画）
```python
from src.core.explorer import SSTGExplorer
from simulation.simple_env import EmptyRoom
from src.utils.realtime_viz import RealtimeVisualizer

# 创建环境
env = EmptyRoom(width=10.0, height=10.0)
grid = env.get_occupancy_map()
start = env.get_start_pose()

# 创建实时可视化器（不保存动画）
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(14, 12),
    update_interval=0.3  # 更新间隔（秒）
)

# 创建探索器并探索
explorer = SSTGExplorer(r_view=2.0, d_theta=45.0, overlap=0.25)
result = explorer.explore(grid, start, visualizer=visualizer)
```

#### 实时可视化 + 保存动画
```python
# 创建实时可视化器（启用动画保存）
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(14, 12),
    update_interval=0.3,
    save_animation=True,  # 启用动画保存
    animation_path='outputs/animations/my_exploration.gif'  # 可选：自定义路径
)

# 探索（会自动保存动画）
result = explorer.explore(grid, start, visualizer=visualizer)
```

## 📊 可视化元素说明

### 图例 (Legend)

- **Current Position** (⭐): 当前机器人位置 - 绿色星标
- **Explored Nodes** (🔴): 已探索的位置 - 红色圆点
- **Active Frontiers** (🟡): 待探索的候选点 - 黄色圆圈
- **Blocked - Obstacle** (❌红): 被障碍物阻挡无法前往
- **Blocked - Too Close** (❌橙): 距离已探索点太近（软障碍）
- **Coverage Circle** (🔵): 视野覆盖范围 - 半透明蓝圈

### 标题栏信息

```
SSTG Explorer - Iteration 15 | Nodes: 16 | Coverage: 87.3% | Frontiers: 42
```

- **Iteration**: 当前迭代次数
- **Nodes**: 已探索节点总数
- **Coverage**: 当前覆盖率
- **Frontiers**: 待探索候选点数量

### 节点编号

每个已探索节点旁边显示其探索次序编号（0, 1, 2, ...），帮助理解探索策略。

## 🎨 自定义选项

### 调整更新频率

```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    update_interval=0.1  # 更快更新（0.1秒）
)
```

- `update_interval=0.1`: 快速更新（流畅但可能影响性能）
- `update_interval=0.5`: 标准更新（推荐）
- `update_interval=1.0`: 慢速更新（节省资源）

**注意**: `update_interval` 同时影响实时显示和动画帧率。较小的值会产生更流畅的动画但生成更多帧。

### 调整图片大小

```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(16, 14),  # 更大的窗口
    update_interval=0.3
)
```

### 动画参数优化

#### 生成高质量动画（慢但细腻）
```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(16, 14),       # 大尺寸
    update_interval=0.2,    # 高帧率
    save_animation=True
)
```

#### 生成快速预览动画（快但较粗糙）
```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(10, 8),        # 小尺寸
    update_interval=0.5,    # 低帧率
    save_animation=True
)
```

## 💾 保存实时可视化结果

### 保存最终状态图片

实时可视化窗口会在探索完成后自动保持打开状态，允许你：

1. **查看最终状态**: 所有信息一览无余
2. **手动保存**: 使用matplotlib工具栏保存图片
3. **关闭继续**: 关闭窗口后程序继续执行

### 🎬 保存探索过程动画（NEW！）

现在支持将整个探索过程保存为GIF动画！

#### 方法1: 使用示例脚本
```python
from examples.basic_exploration import run_basic_exploration

run_basic_exploration(
    env_type='empty',
    realtime_viz=True,      # 实时显示
    save_animation=True     # 保存动画
)
```

#### 方法2: 代码中指定
```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    save_animation=True,  # 启用动画保存
    animation_path='outputs/explorations/custom_name.gif'  # 可选：自定义路径
)
```

#### 动画特性
- **格式**: GIF（易于分享和嵌入文档）
- **帧率**: 自动根据 `update_interval` 计算（通常2-5 FPS）
- **内容**: 包含所有可视化元素（节点、frontiers、覆盖圆等）
- **循环播放**: 默认循环播放，便于观察
- **文件大小**: 取决于探索复杂度，通常1-10MB

#### 输出示例
```
Generating animation from 45 frames...
Animation saved to outputs/animations/animation_empty.gif
  - 45 frames
  - 3 fps
  - Duration: ~15.0 seconds
```

#### 动画应用场景
- 📊 **论文配图**: 展示算法工作流程
- 🎓 **教学演示**: 帮助理解探索策略
- 🐛 **调试分析**: 回放查看问题出现时刻
- 💼 **项目汇报**: 直观展示算法效果

## 🔧 高级用法

### 禁用实时可视化（仅后处理可视化）

```python
# 示例脚本中
run_basic_exploration(env_type='empty', 
                     visualize=True,      # 后处理可视化
                     realtime_viz=False)  # 禁用实时

# 直接调用
result = explorer.explore(grid, start, visualizer=None)
```

### 同时启用多种可视化模式

```python
run_basic_exploration(
    env_type='empty',
    visualize=True,         # 后处理可视化（保存到outputs/）
    realtime_viz=True,      # 实时可视化（窗口）
    save_animation=True     # 探索过程动画（GIF）
)
```

这会生成：
1. 实时显示探索过程（动态窗口）
2. 探索完成后生成3张高质量静态图片（exploration, coverage, metrics）
3. 探索过程的GIF动画（animation_*.gif）

### 仅保存动画（不显示实时窗口）

**注意**: 动画保存功能依赖于实时可视化器来记录帧数据，因此必须启用 `realtime_viz=True`。

```python
# 必须同时启用实时可视化和动画保存
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    update_interval=0.3,
    save_animation=True
)

# 探索（窗口会显示，但可以最小化）
result = explorer.explore(grid, start, visualizer=visualizer)

# 完成后关闭窗口即可，动画已保存
visualizer.close()
```

### 记录探索过程日志

```python
from src.utils.realtime_viz import ExplorationLogger

# 创建日志记录器
logger = ExplorationLogger()

# 在探索循环中记录（需要修改Explorer类集成）
# logger.log_state(...)

# 保存日志
logger.save_log('outputs/exploration_log.json')
```

## 📝 使用场景

### 场景1: 调试算法

启用实时可视化，观察探索策略是否符合预期：

```python
run_basic_exploration(env_type='obstacles', realtime_viz=True)
```

**观察点**：
- Frontiers是否分布合理？
- 探索是否优先局部还是全局？
- 是否有死角或未覆盖区域？

### 场景2: 演示展示

向团队或导师展示算法工作原理：

```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(16, 12),    # 大窗口便于演示
    update_interval=0.8   # 慢一点便于讲解
)
```

### 场景3: 论文配图

#### 静态图片
探索完成后保存高质量截图：

1. 运行探索（实时可视化）
2. 探索完成后窗口保持打开
3. 使用matplotlib工具栏保存为高分辨率图片
4. 或手动调用 `visualizer.save('paper_figure.png')`

#### 动画演示
保存探索过程动画用于论文或报告：

```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(16, 12),       # 高分辨率
    update_interval=0.3,    # 适中帧率
    save_animation=True,
    animation_path='outputs/animations/algorithm_demo.gif'
)
```

**提示**: 
- GIF可直接嵌入Markdown文档
- 对于论文PDF，考虑转换为视频格式（MP4）
- 可使用工具如ffmpeg转换：
  ```bash
  ffmpeg -i animation.gif -movflags faststart -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" output.mp4
  ```

### 场景4: 批量测试

批量测试时禁用实时可视化加快速度：

```python
environments = ['empty', 'obstacles', 'corridor']
for env_type in environments:
    run_basic_exploration(
        env_type=env_type,
        visualize=True,         # 保存静态图结果
        realtime_viz=False,     # 不显示窗口
        save_animation=False    # 不生成动画（节省时间）
    )
```

**注意**: 动画生成会增加一定的处理时间，批量测试时建议禁用以提高效率。

## 🖼️ 输出文件管理

### 自动保存到outputs目录

所有输出文件自动保存到结构化目录：

```
outputs/
├── explorations/
│   ├── exploration_empty.png       # 带编号的探索结果（静态）
│   └── exploration_obstacles.png
├── animations/                      # 🎬 动画文件夹（NEW！）
│   ├── animation_empty.gif         # 探索过程动画
│   └── animation_obstacles.gif
├── coverage/
│   ├── coverage_empty.png          # 覆盖度分析
│   └── coverage_obstacles.png
└── metrics/
    ├── metrics_empty.png           # 统计指标
    └── metrics_obstacles.png
```

### 文件大小估算

- **静态图片** (.png): ~200-500 KB
- **动画文件** (.gif): ~1-10 MB（取决于复杂度和帧数）
  - 简单环境（20-30帧）: 1-3 MB
  - 复杂环境（50-100帧）: 5-10 MB

### Git管理

- ✅ `outputs/` 目录已添加到 `.gitignore`
- ✅ 不会污染git仓库
- ✅ 每次运行生成新结果，自动覆盖

## 🐛 故障排查

### 问题1: 窗口无法显示

**原因**: 在远程服务器或无显示环境

**解决方案**: 使用X11转发或禁用实时可视化

```bash
# 使用X11转发（SSH）
ssh -X user@server
python examples/basic_exploration.py

# 或禁用实时可视化
run_basic_exploration(env_type='empty', realtime_viz=False)
```

**注意**: 动画保存需要matplotlib后端支持，即使在无显示环境中也可能成功（使用Agg后端）。

### 问题2: 动画生成失败

**错误信息**: `ModuleNotFoundError: No module named 'PIL'`

**解决方案**: 安装Pillow库
```bash
pip install Pillow
```

**错误信息**: `ValueError: Cannot save animation: no frames available`

**原因**: `save_animation=True` 但 `realtime_viz=False`

**解决方案**: 必须同时启用实时可视化
```python
run_basic_exploration(
    env_type='empty',
    realtime_viz=True,      # 必须启用
    save_animation=True
)
```

### 问题3: 动画文件过大

**原因**: 探索过程太长或帧率太高

**解决方案**: 调整参数减小文件大小
```python
visualizer = RealtimeVisualizer(
    occupancy_grid=grid,
    r_view=2.0,
    figsize=(10, 8),        # 减小图片尺寸
    update_interval=0.5,    # 增大间隔（降低帧率）
    save_animation=True
)
```

或使用视频压缩工具：
```bash
# 使用gifsicle压缩GIF
gifsicle -O3 --colors 128 animation_original.gif -o animation_compressed.gif
```

### 问题4: 更新太快/太慢

**解决方案**: 调整`update_interval`参数

```python
# 太快 -> 增大interval
visualizer = RealtimeVisualizer(..., update_interval=0.5)

# 太慢 -> 减小interval  
visualizer = RealtimeVisualizer(..., update_interval=0.2)
```

### 问题5: 窗口卡顿

**原因**: 候选点太多或地图太大

**解决方案**: 
1. 增大 `update_interval`
2. 减小 `figsize`
3. 减少 `d_theta`（减少方向数）

### 问题6: 内存占用高

**原因**: 追踪了过多blocked points或动画帧

**解决方案**: 
- Explorer类中已限制最多追踪200个blocked点
- 动画帧存储已优化，每帧只保存必要数据

## 📚 示例代码

### 完整示例：对比不同参数（带动画）

```python
from src.core.explorer import SSTGExplorer
from simulation.simple_env import create_environment
from src.utils.realtime_viz import RealtimeVisualizer

# 创建环境
env = create_environment('obstacles', width=10.0, height=10.0, num_obstacles=5)
grid = env.get_occupancy_map()
start = env.get_start_pose()

# 参数集合
param_sets = [
    {'name': 'Dense', 'r_view': 1.5, 'd_theta': 45.0},
    {'name': 'Standard', 'r_view': 2.0, 'd_theta': 45.0},
    {'name': 'Sparse', 'r_view': 2.5, 'd_theta': 45.0},
]

for params in param_sets:
    print(f"\nTesting: {params['name']}")
    
    # 创建可视化器（带动画保存）
    visualizer = RealtimeVisualizer(
        occupancy_grid=grid,
        r_view=params['r_view'],
        update_interval=0.5,
        save_animation=True,
        animation_path=f"outputs/animations/comparison_{params['name']}.gif"
    )
    
    # 探索
    explorer = SSTGExplorer(
        r_view=params['r_view'],
        d_theta=params['d_theta'],
        overlap=0.25
    )
    
    result = explorer.explore(grid, start, visualizer=visualizer)
    
    print(f"Nodes: {len(result['nodes'])}, "
          f"Coverage: {result['metadata']['coverage_ratio']:.1%}")
    
    input("Press Enter to continue to next configuration...")
    visualizer.close()

print("\nAll animations saved to outputs/animations/")
```

### 示例：生成演示动画

```python
"""生成算法演示动画，适合教学或展示"""
from examples.basic_exploration import run_basic_exploration

# 配置：慢速、高质量
result = run_basic_exploration(
    env_type='obstacles',
    r_view=2.0,
    d_theta=45.0,
    visualize=True,
    realtime_viz=True,
    save_animation=True
)

print(f"\n演示动画已保存！")
print(f"可用于：")
print(f"  - 嵌入到README.md展示")
print(f"  - 添加到演示文稿")
print(f"  - 分享给团队成员")
```

## 🔗 相关文档

- [VISUALIZATION_UPDATE.md](VISUALIZATION_UPDATE.md) - 可视化功能更新说明
- [QUICKSTART.md](QUICKSTART.md) - 快速开始指南  
- [README.md](README.md) - 项目总览
- [outputs/README.md](outputs/README.md) - 输出目录说明

---

**版本**: 2.0  
**更新日期**: 2026-04-15  
**状态**: ✅ 已实现并测试  
**新增功能**: 🎬 探索过程动画保存（GIF格式）
