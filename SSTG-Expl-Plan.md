# 空间语义拓扑图探索算法 (SSTG-Explorer) 实现计划

## Context (背景)

用户正在开发基于空间语义拓扑图+视觉语言模型的导航系统。现有的 RRT Explorer 存在问题：
1. 探索点过于稀疏，无法满足视觉图像采集的覆盖度需求
2. 倾向于在边界取点，影响机器人视野

需要设计一套专门面向空间语义拓扑图构建的探索算法，确保：
- 足够密集的采样点用于视觉语义信息采集
- 全覆盖可探索区域
- 避免重复采样（节点间距 ≥ r_view - overlap）

## 核心思路分析与完善

### 用户原始思路回顾
1. 从地图原点开始探索
2. 固定视野半径 r_view（如2m），角度间隔 d_theta（如45度）
3. 无障碍时默认选择0度方向
4. 有障碍时记录边界点，对可选方向聚类，选择最大无障碍角度范围的中心方向
5. 已探索点视为"软障碍"（r_view - overlap 范围内）
6. 全局考虑所有历史位置的未探索方向
7. 优先当前位置探索（前期），后期全局查漏补缺
8. 终止条件：所有历史位置方向都指向障碍物或已探索区域
9. 最终保证全覆盖且无过密节点

### 思路完善与补充

#### 1. 碰撞检测机制细化
**问题**：如何判断"导航点碰到障碍物"？
**解决方案**：
- 使用栅格地图（占据栅格地图或距离场）进行碰撞检测
- 对候选点进行两层检测：
  - 点检测：候选点本身是否在障碍物内
  - 路径检测：从当前位置到候选点的路径是否可达（防止穿墙）
- 考虑机器人半径 r_robot，安全距离 d_safe

#### 2. 方向聚类与选择策略优化
**问题**：如何精确定义"没有障碍物角度最大的方向"和"中心点"？
**解决方案**：
- 将 360 度划分为 n = 360/d_theta 个扇区
- 标记每个扇区为：障碍物(O)、已探索(E)、自由(F)
- 寻找最长连续自由扇区序列
- 选择该序列的角度中点作为探索方向
- 示例：8方向(0,45,90,135,180,225,270,315)
  - 0:O, 45:F, 90:F, 135:O, 180:F, 225:F, 270:F, 315:F
  - 最长连续自由: [180,225,270,315] (4个)
  - 选择中点: (180+315)/2 = 247.5° ≈ 225° 或 270°

#### 3. "软障碍"（已探索点）处理
**问题**：overlap 参数如何影响判断？何时视为"接触"？
**解决方案**：
- 定义排斥距离: d_repel = r_view - overlap
- 候选点距离任意已探索点 < d_repel 时，视为受阻
- 但与硬障碍物区分：
  - 硬障碍物：完全禁止
  - 软障碍（已探索）：可通过但降低优先级（后期可选）
- 引入"探索强度"概念：
  - 未探索方向：强度 = 1.0
  - 受软障碍影响方向：强度 = distance / d_repel（0-1之间）

#### 4. 全局候选点管理
**问题**：如何高效管理"所有历史位置的未探索方向"？
**解决方案**：
- 维护数据结构：`FrontierQueue`
  - 每个 frontier: (position, angle, priority)
- 每探索一个新点：
  - 移除：该位置所有方向的 frontiers
  - 添加：新位置的 n 个方向 frontiers
  - 更新：所有受影响的老 frontiers 的 priority
- Priority 计算：
  ```
  priority = base_score * distance_weight * novelty_weight
  base_score = 1.0 (自由) 或 探索强度 (受软障碍影响)
  distance_weight = 1 / (1 + distance_to_current)^alpha  # alpha=1~2
  novelty_weight = 1.0 (完全未访问) 或 衰减值
  ```

#### 5. 局部优先与全局平衡
**问题**：如何量化"优先当前位置"？
**解决方案**：
- 使用距离衰减因子：distance_weight = 1 / (1 + d/r_view)^alpha
- alpha 参数控制局部性：
  - alpha = 0: 完全全局（不考虑距离）
  - alpha = 1: 线性衰减
  - alpha = 2: 二次衰减（强局部优先）
- 探索阶段自适应：
  - 前期（覆盖率 < 30%）：alpha = 2（强局部）
  - 中期（30%-70%）：alpha = 1（平衡）
  - 后期（> 70%）：alpha = 0.5（偏全局，查漏补缺）

#### 6. 终止条件与覆盖验证
**问题**：如何判断"所有方向都指向障碍物或已探索区域"？
**解决方案**：
- 终止条件：FrontierQueue 为空 OR 所有剩余 frontiers 优先级 < threshold
- 覆盖验证（后处理）：
  - 生成覆盖图：所有探索点的 r_view 圆覆盖的区域
  - 与可探索区域（自由空间）求交
  - 计算覆盖率 = 覆盖面积 / 自由空间面积
  - 目标：≥ 95%
- 节点间距验证：
  - 检查任意两节点距离 ≥ r_view - overlap
  - 若不满足，进行后处理剔除或调整

#### 7. 路径规划与导航
**问题**：如何从当前点导航到选定的探索点？
**解决方案**：
- 短距离（< 2*r_view）：直线导航（已做碰撞检测）
- 长距离：调用 A* 或 RRT 等路径规划器
- 导航失败处理：
  - 标记该 frontier 为不可达
  - 从队列中移除
  - 选择下一个候选点

#### 8. 边界情况处理
**补充问题**：
- 起始点选择：如何确保起始点合法？
  - 检查起始点不在障碍物内
  - 若地图未知，先进行局部探索建立初始地图
- 狭窄通道：d_theta 可能导致错过狭窄通道
  - 引入自适应角度细化机制
  - 当检测到障碍物与自由空间切换频繁时，局部增加采样密度
- 多房间环境：可能陷入局部房间
  - 使用拓扑分析识别未探索连通区域
  - 强制尝试跨房间探索

## 算法伪代码

```
Algorithm: SSTG-Explorer

Input: 
  - map: 占据栅格地图 (初始可为空)
  - start_pose: 起始位姿
  - r_view: 视野半径
  - d_theta: 角度间隔
  - overlap: 重叠距离
  - r_robot: 机器人半径

Output:
  - explored_nodes: 探索位置列表 [(x, y, theta), ...]

Initialize:
  1. explored_nodes = [start_pose]
  2. frontier_queue = PriorityQueue()
  3. obstacle_points = []
  4. d_repel = r_view - overlap
  
  # 添加起始点的所有方向到 frontier
  for angle in range(0, 360, d_theta):
    candidate = compute_point(start_pose, r_view, angle)
    if is_free(candidate):
      frontier_queue.add(Frontier(start_pose, angle, priority=1.0))

Main Loop:
  while not frontier_queue.empty():
    # 1. 选择最高优先级 frontier
    best_frontier = frontier_queue.pop()
    target_pose = compute_point(best_frontier.position, r_view, best_frontier.angle)
    
    # 2. 验证仍然有效（未被后续探索覆盖）
    if is_covered_by_explored(target_pose, explored_nodes, d_repel):
      continue
    
    # 3. 路径规划与导航
    path = plan_path(current_pose, target_pose)
    if path is None:
      mark_unreachable(best_frontier)
      continue
    
    navigate(path)
    current_pose = target_pose
    
    # 4. 记录探索点
    explored_nodes.append(current_pose)
    
    # 5. 更新地图（如果使用 SLAM）
    update_map(current_pose)
    
    # 6. 生成新的 frontiers
    for angle in range(0, 360, d_theta):
      candidate = compute_point(current_pose, r_view, angle)
      
      # 碰撞检测
      collision_type = check_collision(current_pose, candidate, explored_nodes, obstacle_points)
      
      if collision_type == FREE:
        priority = compute_priority(current_pose, candidate, angle, explored_nodes, current_pose)
        frontier_queue.add(Frontier(current_pose, angle, priority))
      elif collision_type == HARD_OBSTACLE:
        obstacle_points.append(candidate)
    
    # 7. 更新所有旧 frontiers 的优先级
    update_all_priorities(frontier_queue, current_pose, explored_nodes)
    
    # 8. 检查终止条件
    if coverage_ratio() > 0.95 and frontier_queue.max_priority() < threshold:
      break

Post-processing:
  1. 验证覆盖率
  2. 验证节点间距
  3. 可选：剔除冗余节点
  4. 保存结果

return explored_nodes
```

## 实现架构设计

### 目录结构
```
sstg-expl/
├── src/
│   ├── __init__.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── explorer.py           # 主探索算法类 SSTGExplorer
│   │   ├── frontier.py            # Frontier 数据结构和队列管理
│   │   ├── collision_checker.py   # 碰撞检测
│   │   └── coverage_analyzer.py   # 覆盖度分析
│   ├── map/
│   │   ├── __init__.py
│   │   ├── occupancy_grid.py     # 占据栅格地图
│   │   └── distance_field.py     # 距离场（可选，加速碰撞检测）
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── local_planner.py      # 局部路径规划
│   │   └── astar.py              # A* 实现（可选）
│   ├── utils/
│   │   ├── __init__.py
│   │   ├── geometry.py           # 几何计算工具
│   │   └── visualization.py      # 可视化工具
│   └── config.py                 # 配置参数
├── tests/
│   ├── __init__.py
│   ├── test_explorer.py
│   ├── test_frontier.py
│   ├── test_collision.py
│   └── test_coverage.py
├── simulation/
│   ├── __init__.py
│   ├── simple_env.py             # 简单几何环境
│   ├── apartment_env.py          # 公寓环境
│   └── benchmark.py              # benchmark 框架
├── examples/
│   ├── basic_exploration.py      # 基础示例
│   ├── compare_with_rrt.py       # 与 RRT 对比
│   └── visualize_exploration.py  # 可视化探索过程
├── benchmarks/
│   ├── environments/             # 标准测试环境
│   │   ├── empty_room.yaml
│   │   ├── corridor.yaml
│   │   ├── multiple_rooms.yaml
│   │   └── complex_apartment.yaml
│   └── results/                  # benchmark 结果
├── requirements.txt
├── setup.py
├── README.md
└── SSTG-Expl-Plan.md            # 本文档
```

### 核心类设计

#### 1. SSTGExplorer (explorer.py)
```python
class SSTGExplorer:
    def __init__(self, r_view, d_theta, overlap, r_robot):
        self.r_view = r_view
        self.d_theta = d_theta
        self.overlap = overlap
        self.r_robot = r_robot
        self.d_repel = r_view - overlap
        
        self.explored_nodes = []
        self.frontier_queue = FrontierQueue()
        self.collision_checker = CollisionChecker()
        self.coverage_analyzer = CoverageAnalyzer()
    
    def explore(self, map, start_pose):
        """主探索循环"""
        pass
    
    def generate_frontiers(self, pose):
        """为给定位姿生成候选 frontiers"""
        pass
    
    def compute_priority(self, frontier, current_pose):
        """计算 frontier 优先级"""
        pass
    
    def update_priorities(self, current_pose):
        """更新所有 frontiers 优先级"""
        pass
```

#### 2. Frontier & FrontierQueue (frontier.py)
```python
@dataclass
class Frontier:
    position: Tuple[float, float]  # 基准位置
    angle: float                    # 方向角
    priority: float                 # 优先级
    target: Tuple[float, float]     # 目标点 (预计算)
    
class FrontierQueue:
    """使用 heapq 实现的优先队列"""
    def __init__(self):
        self.heap = []
        self.entry_map = {}  # 用于快速查找和更新
    
    def add(self, frontier):
        pass
    
    def pop(self):
        pass
    
    def update_priority(self, frontier_id, new_priority):
        pass
    
    def remove(self, frontier_id):
        pass
```

#### 3. CollisionChecker (collision_checker.py)
```python
class CollisionChecker:
    def __init__(self, r_robot, d_safe):
        self.r_robot = r_robot
        self.d_safe = d_safe
    
    def check_point(self, point, map):
        """检查点是否在障碍物内"""
        pass
    
    def check_path(self, start, end, map):
        """检查路径是否无碰撞"""
        pass
    
    def check_against_explored(self, point, explored_nodes, d_repel):
        """检查是否过近已探索点（软障碍）"""
        pass
```

#### 4. CoverageAnalyzer (coverage_analyzer.py)
```python
class CoverageAnalyzer:
    def compute_coverage(self, explored_nodes, r_view, free_space_map):
        """计算覆盖率"""
        pass
    
    def check_node_spacing(self, explored_nodes, min_distance):
        """验证节点间距"""
        pass
    
    def find_coverage_gaps(self, explored_nodes, r_view, free_space_map):
        """找出未覆盖区域"""
        pass
```

## 仿真环境设计

### 基础环境接口
```python
class Environment:
    def get_occupancy_map(self) -> np.ndarray:
        """返回占据栅格地图"""
        pass
    
    def get_free_space_mask(self) -> np.ndarray:
        """返回自由空间掩码"""
        pass
    
    def get_start_pose(self) -> Tuple[float, float, float]:
        """返回起始位姿"""
        pass
    
    def visualize(self, explored_nodes=None):
        """可视化环境和探索结果"""
        pass
```

### 测试环境类型
1. **Empty Room**: 简单矩形房间，测试基础算法
2. **Corridor**: 狭长走廊，测试方向选择
3. **Multiple Rooms**: 多房间连通，测试全局探索
4. **Complex Apartment**: 复杂公寓布局，综合测试
5. **Office**: 办公室环境（桌椅等障碍物）
6. **Warehouse**: 仓库环境（货架阵列）

### Benchmark 对比算法

#### 1. **Baseline: Uniform Grid Sampling**
- 在自由空间均匀网格采样
- 最简单，用于对比覆盖度下限

#### 2. **RRT-based Explorer** (用户提到的现有方法)
- 基于 RRT 的探索
- 重现用户描述的"稀疏+边界偏好"问题

#### 3. **Frontier-based Exploration** (经典方法)
- 基于 occupancy grid frontiers
- 代表主流 SLAM 探索方法

#### 4. **Next-Best-View (NBV)**
- 基于信息增益的下一最佳视角
- 常用于主动 SLAM

#### 5. **Our SSTG-Explorer**
- 本项目实现的算法

### Benchmark 指标

#### 主要指标
1. **Coverage Ratio**: 覆盖率（探索圆覆盖面积 / 自由空间面积）
2. **Number of Nodes**: 探索节点数量
3. **Node Density**: 节点密度（节点数 / 覆盖面积）
4. **Exploration Time**: 探索总时长（模拟）
5. **Travel Distance**: 机器人总移动距离

#### 次要指标
6. **Min Node Distance**: 最小节点间距（验证 overlap 约束）
7. **Coverage Uniformity**: 覆盖均匀度（方差）
8. **Boundary Coverage**: 边界覆盖质量
9. **Computational Time**: 算法计算时间

#### 可视化输出
- 探索轨迹动画
- 覆盖热力图
- 节点分布图
- 对比雷达图

## 实现步骤

### Phase 1: 核心数据结构 (Week 1)
1. 实现 `Frontier` 和 `FrontierQueue`
2. 实现 `OccupancyGrid` 地图类
3. 实现 `CollisionChecker` 基础碰撞检测
4. 单元测试

### Phase 2: 探索算法核心 (Week 1-2)
1. 实现 `SSTGExplorer` 主循环
2. 实现 frontier 生成逻辑
3. 实现优先级计算（固定 alpha，先不自适应）
4. 实现简单直线路径规划
5. 集成测试（简单环境）

### Phase 3: 覆盖度分析 (Week 2)
1. 实现 `CoverageAnalyzer`
2. 实现覆盖率计算
3. 实现节点间距验证
4. 实现覆盖空隙检测

### Phase 4: 高级特性 (Week 2-3)
1. 自适应 alpha（根据覆盖率调整）
2. 软障碍处理（已探索点排斥）
3. A* 路径规划（替代直线）
4. 狭窄通道检测与自适应细化

### Phase 5: 仿真环境 (Week 3)
1. 实现 `Environment` 基类
2. 实现 6 种测试环境
3. 环境可视化
4. 配置文件加载

### Phase 6: Benchmark 框架 (Week 3-4)
1. 实现 baseline 算法（Uniform Grid, RRT）
2. 集成经典算法（Frontier-based, NBV）
3. 实现 Benchmark 运行器
4. 实现指标计算和对比可视化
5. 批量实验和结果分析

### Phase 7: 文档与示例 (Week 4)
1. 编写 README
2. 编写 API 文档
3. 创建示例脚本
4. 结果报告

## 技术栈

### Python 依赖
```
# 核心
numpy>=1.20.0
scipy>=1.7.0

# 地图和路径规划
scikit-image>=0.18.0

# 可视化
matplotlib>=3.3.0
seaborn>=0.11.0

# 数据处理
pandas>=1.3.0

# 配置
pyyaml>=5.4.0

# 测试
pytest>=6.2.0
pytest-cov>=2.12.0

# 可选：ROS 集成
# rclpy (如果需要与 ROS2 集成)
```

### 代码规范
- Type hints (Python 3.8+)
- Docstrings (Google style)
- Black formatter
- Pylint/Flake8

## 与导航系统的集成接口

### 输出格式
```python
# 探索结果
exploration_result = {
    'nodes': [
        {
            'id': 0,
            'position': (x, y),
            'orientation': theta,
            'timestamp': t,
            'neighbors': [1, 2, 3],  # 临近节点 ID
        },
        ...
    ],
    'metadata': {
        'r_view': 2.0,
        'overlap': 0.25,
        'd_theta': 45,
        'coverage_ratio': 0.96,
        'total_distance': 125.3,
        'total_time': 89.2,
    }
}

# 保存为 JSON 或 pickle
```

### 集成方式
1. **独立运行模式**: 
   - 输入：地图文件（YAML/PNG）
   - 输出：探索节点文件（JSON/pickle）
   - 导航系统读取文件进行语义标注

2. **API 模式**:
   ```python
   from sstg_expl import SSTGExplorer
   
   explorer = SSTGExplorer(r_view=2.0, d_theta=45, overlap=0.25)
   nodes = explorer.explore(map_data, start_pose)
   
   # 导航系统处理
   for node in nodes:
       image = capture_image(node.position, node.orientation)
       semantic_info = vlm.analyze(image)
       graph.add_node(node, semantic_info)
   ```

3. **ROS2 集成模式**（可选）:
   - 订阅: `/map` (occupancy grid)
   - 发布: `/exploration/goal` (下一个探索目标)
   - 服务: `/exploration/start`, `/exploration/status`

## 验证计划

### 单元测试
- 每个核心类的功能测试
- 边界情况测试
- 性能测试（大地图）

### 集成测试
- 端到端探索流程
- 不同参数组合
- 各种环境类型

### 可视化验证
- 实时探索过程可视化
- 覆盖度热力图
- 与 baseline 对比可视化

### Benchmark 验证
- 在标准环境上运行所有算法
- 统计指标对比
- 生成对比报告

## 潜在改进方向

1. **多机器人协同探索**: 扩展为多机器人场景
2. **动态环境**: 处理动态障碍物
3. **不确定性处理**: 考虑定位和地图不确定性
4. **语义引导**: 根据语义信息调整探索策略
5. **学习优化**: 使用 RL 学习最优 alpha 等参数

## 风险与挑战

1. **计算复杂度**: 全局管理所有 frontiers 可能在大环境中很慢
   - 缓解：空间索引（KD-tree）、优先级阈值剪枝
   
2. **参数敏感性**: r_view, overlap, d_theta 等参数影响大
   - 缓解：提供参数选择指南、自动参数调优

3. **复杂环境**: 多层、立体环境扩展困难
   - 缓解：先聚焦 2D，3D 作为未来工作

4. **实际部署**: 仿真与实际机器人差距
   - 缓解：充分测试、预留传感器噪声处理接口

## 时间估算

- **Phase 1-2 (核心算法)**: 1-2 周
- **Phase 3-4 (完善功能)**: 1 周  
- **Phase 5-6 (仿真和 benchmark)**: 1-2 周
- **Phase 7 (文档)**: 3-5 天

**总计**: 约 4-5 周全职开发时间

---

## 立即开始的实现优先级

基于上述规划，立即开始实现的优先顺序：

1. ✅ 项目结构搭建和基础配置
2. ✅ 核心数据结构 (`Frontier`, `FrontierQueue`)
3. ✅ 占据栅格地图 (`OccupancyGrid`)
4. ✅ 碰撞检测 (`CollisionChecker`)
5. ✅ 探索器核心逻辑 (`SSTGExplorer`)
6. ✅ 简单仿真环境
7. ✅ 基础可视化
8. ⏳ 完整 benchmark 框架
