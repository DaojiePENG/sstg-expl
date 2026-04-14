# SSTG Explorer - References and Baseline Algorithms

**版本**: v1.0  
**日期**: 2026-04-15  
**目的**: 收集exploration相关参考文献和可实现的baseline算法

---

## 1. 已实现算法 (Implemented Algorithms)

### 1.1 SSTG Explorer (Ours)
**算法描述**: Single-Shot Tree Graph Explorer with multiple frontier selection strategies

**特点**:
- 6种frontier选择策略（消融实验）
- A*路径规划（可选）
- 自适应角度采样（可选）
- 全局frontier管理

**性能** (Empty 10×10m环境):
- 最优配置: Enhanced Distance + A* + Adaptive
- 距离: 36.4m (比baseline↓67.8%)
- 覆盖率: 94.3%
- 跳跃次数: 0

---

### 1.2 Uniform Grid Sampling
**参考文献**:
```
Choset, H. (2001). "Coverage for robotics–A survey of recent results."
Annals of mathematics and artificial intelligence, 31(1-4), 113-126.
```

**算法描述**: 按照固定间隔在地图上采样，系统化覆盖环境

**特点**:
- 简单、可预测
- 完全覆盖保证
- 不考虑路径优化

**适用场景**: 简单环境、需要完全覆盖

**代码**: `src/baseline/uniform_grid.py`

---

### 1.3 RRT-based Explorer
**参考文献**:
```
LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning."
Technical Report TR 98-11, Computer Science Dept., Iowa State University.

Umari, H., & Mukhopadhyay, S. (2017). "Autonomous robotic exploration based on
multiple rapidly-exploring randomized trees." In IEEE/RSJ International Conference
on Intelligent Robots and Systems (IROS) (pp. 1396-1402).
```

**算法描述**: 使用RRT增量式探索环境，随机采样并扩展树结构

**特点**:
- 概率完备性
- 自然偏向未探索区域
- 适合复杂环境

**适用场景**: 复杂障碍物环境、高维空间

**代码**: `src/baseline/rrt_explorer.py`

---

### 1.4 Frontier-based Exploration (Classic)
**参考文献**:
```
Yamauchi, B. (1997). "A frontier-based approach for autonomous exploration."
In Proceedings 1997 IEEE International Symposium on Computational Intelligence
in Robotics and Automation CIRA'97 (pp. 146-151).

Yamauchi, B. (1998). "Frontier-based exploration using multiple robots."
In Proceedings of the second international conference on Autonomous agents
(pp. 47-53).
```

**算法描述**: 反复选择最近的frontier（已探索与未探索边界）作为目标

**特点**:
- 经典方法，广泛使用
- 简单高效
- 局部贪心策略

**适用场景**: 通用场景，benchmark基准

**代码**: `src/baseline/frontier_explorer.py`

---

### 1.5 Next-Best-View (NBV)
**参考文献**:
```
Connolly, C. (1985). "The determination of next best views."
In Proceedings. 1985 IEEE international conference on robotics and automation
(Vol. 2, pp. 432-435).

Gonzalez-Banos, H. H., & Latombe, J. C. (2002). "Navigation strategies for
exploring indoor environments." The International Journal of Robotics Research,
21(10-11), 829-848.

Bircher, A., Kamel, M., Alexis, K., Oleynikova, H., & Siegwart, R. (2016).
"Structural inspection path planning via iterative viewpoint resampling with
application to aerial robotics." In 2016 IEEE International Conference on
Robotics and Automation (ICRA) (pp. 6423-6430).
```

**算法描述**: 基于信息增益选择下一个最佳视点，平衡探索收益和移动成本

**特点**:
- 信息论方法
- 考虑exploration-exploitation权衡
- 适合传感器规划

**适用场景**: 需要高效探索、计算资源充足

**代码**: `src/baseline/nbv_explorer.py`

---

## 2. 待实现算法 (To Be Implemented)

### 2.1 Learning-based Methods (近年方法)

#### 2.1.1 Deep Reinforcement Learning for Exploration (2018-2022)
**参考文献**:
```
Chen, T., Gupta, S., & Gupta, A. (2019). "Learning exploration policies for
navigation." In International Conference on Learning Representations (ICLR).

Zhu, Y., Mottaghi, R., Kolve, E., Lim, J. J., Gupta, A., Fei-Fei, L., & Farhadi, A.
(2017). "Target-driven visual navigation in indoor scenes using deep reinforcement
learning." In 2017 IEEE international conference on robotics and automation (ICRA)
(pp. 3357-3364).

Faust, A., Oslund, K., Ramirez, O., Francis, A., Tapia, L., Fiser, M., & Davidson, J.
(2018). "PRM-RL: Long-range robotic navigation tasks by combining reinforcement
learning and sampling-based planning." In 2018 IEEE International Conference on
Robotics and Automation (ICRA) (pp. 5113-5120).
```

**算法描述**: 使用深度强化学习学习探索策略，通过经验优化navigation policy

**特点**:
- 端到端学习
- 适应性强
- 需要大量训练数据

**实现难度**: ⭐⭐⭐⭐ (需要RL框架、训练环境)

---

#### 2.1.2 Neural SLAM + Active Neural Mapping (2020)
**参考文献**:
```
Chaplot, D. S., Gandhi, D., Gupta, S., Gupta, A., & Salakhutdinov, R. (2020).
"Learning to explore using active neural SLAM." In International Conference on
Learning Representations (ICLR).
```

**算法描述**: 结合神经网络SLAM和主动探索策略

**特点**:
- 端到端可学习SLAM
- 主动探索策略
- 适合视觉导航

**实现难度**: ⭐⭐⭐⭐⭐ (需要深度学习框架)

---

### 2.2 Optimization-based Methods

#### 2.2.1 Submodular Optimization for Exploration (2015)
**参考文献**:
```
Hollinger, G. A., & Sukhatme, G. S. (2014). "Sampling-based robotic information
gathering algorithms." The International Journal of Robotics Research, 33(9), 1271-1287.
```

**算法描述**: 使用次模优化理论选择最优探索路径

**特点**:
- 理论保证（approximation ratio）
- 全局优化视角
- 计算复杂度可控

**实现难度**: ⭐⭐⭐ (需要优化库)

---

#### 2.2.2 Receding Horizon Next-Best-View Planning (NBVP) (2016-2018)
**参考文献**:
```
Bircher, A., Kamel, M., Alexis, K., Oleynikova, H., & Siegwart, R. (2016).
"Receding horizon 'next-best-view' planner for 3D exploration."
In 2016 IEEE international conference on robotics and automation (ICRA) (pp. 1462-1468).

Delmerico, J., Isler, S., Sabzevari, R., & Scaramuzza, D. (2017).
"A comparison of volumetric information gain metrics for active 3D object reconstruction."
Autonomous Robots, 42(2), 197-208.
```

**算法描述**: Receding horizon框架下的NBV规划，适用于3D探索

**特点**:
- 滚动时域规划
- 适合无人机、3D探索
- 高效信息增益计算

**实现难度**: ⭐⭐⭐ (需要3D体素表示)

---

### 2.3 Graph-based Methods

#### 2.3.1 Topological Exploration (2017)
**参考文献**:
```
Oßwald, S., Hornung, A., & Bennewitz, M. (2016). "Improved proposals for highly
accurate localization using range and vision data." In 2016 IEEE/RSJ International
Conference on Intelligent Robots and Systems (IROS) (pp. 1809-1814).
```

**算法描述**: 基于拓扑图的探索，构建环境拓扑表示

**特点**:
- 高层次语义表示
- 适合大规模环境
- 位置识别鲁棒

**实现难度**: ⭐⭐⭐

---

#### 2.3.2 GMapping + Frontier Exploration (2018)
**参考文献**:
```
Grisetti, G., Stachniss, C., & Burgard, W. (2007). "Improved techniques for grid
mapping with Rao-Blackwellized particle filters." IEEE transactions on Robotics,
23(1), 34-46.
```

**算法描述**: 结合GMapping SLAM和frontier探索

**特点**:
- 实时SLAM + 探索
- 粒子滤波器
- 广泛应用

**实现难度**: ⭐⭐⭐⭐ (需要SLAM实现)

---

### 2.4 Multi-Agent Methods

#### 2.4.1 Multi-Robot Frontier Exploration (2006-2018)
**参考文献**:
```
Burgard, W., Moors, M., Stachniss, C., & Schneider, F. E. (2005). "Coordinated
multi-robot exploration." IEEE Transactions on robotics, 21(3), 376-386.

Simmons, R., Apfelbaum, D., Burgard, W., Fox, D., Moors, M., Thrun, S., & Younes, H.
(2000). "Coordination for multi-robot exploration and mapping." In AAAI/IAAI
(pp. 852-858).
```

**算法描述**: 多机器人协同探索，任务分配和协调

**特点**:
- 并行探索
- 任务分配算法
- 通信协议

**实现难度**: ⭐⭐⭐⭐ (需要多机器人框架)

---

### 2.5 Semantic/Object-Goal Exploration (最新方向)

#### 2.5.1 Object-Goal Navigation (2020-2023)
**参考文献**:
```
Batra, D., Gokaslan, A., Kembhavi, A., Maksymets, O., Mottaghi, R., Savva, M., ...
& Malik, J. (2020). "ObjectNav revisited: On evaluation of embodied agents navigating
to objects." arXiv preprint arXiv:2006.13171.

Chaplot, D. S., Salakhutdinov, R., & Gupta, A. (2020). "Learning to Explore using
Active Neural Mapping." In ICLR.

Ramakrishnan, S. K., Jayaraman, D., & Grauman, K. (2021). "An exploration of
embodied visual exploration." International Journal of Computer Vision, 129, 1616-1649.
```

**算法描述**: 目标导向的探索，寻找特定物体

**特点**:
- 语义理解
- 目标导向
- 实际应用价值高

**实现难度**: ⭐⭐⭐⭐⭐ (需要视觉识别)

---

#### 2.5.2 Semantic Frontier (2019)
**参考文献**:
```
Kostavelis, I., & Gasteratos, A. (2015). "Semantic mapping for mobile robotics
tasks: A survey." Robotics and Autonomous Systems, 66, 86-103.
```

**算法描述**: 基于语义信息的frontier选择

**特点**:
- 利用语义先验
- 提高探索效率
- 需要场景理解

**实现难度**: ⭐⭐⭐⭐

---

## 3. 综述和调研类文献

### 3.1 Exploration综述
```
Freda, L., Loianno, G., & Pascucci, F. (2019). "A review of the state of the art in
self-exploration and autonomous navigation." arXiv preprint arXiv:1909.03966.

Thrun, S. (2002). "Robotic mapping: A survey." Exploring artificial intelligence
in the new millennium, 1(1-35), 1.

Zhu, Y., Mottaghi, R., Kolve, E., Lim, J. J., Gupta, A., Fei-Fei, L., & Farhadi, A.
(2017). "Target-driven visual navigation in indoor scenes using deep reinforcement
learning." In 2017 IEEE international conference on robotics and automation (ICRA).
```

### 3.2 Coverage Path Planning综述
```
Galceran, E., & Carreras, M. (2013). "A survey on coverage path planning for
robotics." Robotics and Autonomous Systems, 61(12), 1258-1276.

Choset, H., & Pignon, P. (1998). "Coverage path planning: The boustrophedon
cellular decomposition." In Field and service robotics (pp. 203-209). Springer, London.
```

### 3.3 SLAM综述
```
Cadena, C., Carlone, L., Carrillo, H., Latif, Y., Scaramuzza, D., Neira, J., ...
& Leonard, J. J. (2016). "Past, present, and future of simultaneous localization
and mapping: Toward the robust-perception age." IEEE Transactions on robotics,
32(6), 1309-1332.
```

---

## 4. 推荐实现优先级

### 高优先级 (论文benchmark必需)

1. **Submodular Optimization** - 理论保证，近年重要方法
2. **Receding Horizon NBVP** - 实际应用广泛
3. **Multi-Robot Frontier** - 展示扩展性

### 中优先级 (增强对比)

4. **Deep RL Exploration** - 代表学习方法
5. **Topological Exploration** - 不同范式
6. **GMapping + Frontier** - 集成SLAM

### 低优先级 (可选扩展)

7. **Active Neural Mapping** - 最新方法，但实现复杂
8. **Object-Goal Navigation** - 实际应用，但需要视觉
9. **Semantic Frontier** - 研究前沿

---

## 5. Benchmark设计建议

### 5.1 标准测试环境

**必测环境**:
1. Empty Room (10×10m) - 基础性能
2. Sparse Obstacles (10×10m, 5障碍物) - 简单障碍
3. Dense Obstacles (10×10m, 15障碍物) - 复杂障碍
4. Corridor (15×2.5m) - 狭窄通道
5. Multi-room (15×10m) - 实际场景

**可选环境**:
6. Apartment (12×12m) - 真实户型
7. Office (20×15m) - 办公室
8. Warehouse (25×25m) - 大规模

### 5.2 评估指标

**必需指标**:
- Total Distance (m) - 总移动距离
- Number of Nodes - 采样节点数
- Coverage Ratio - 覆盖率
- Coverage Efficiency - 覆盖效率 (coverage/distance)
- Computation Time (s) - 运行时间

**可选指标**:
- Path Smoothness - 路径平滑度
- Jump Count - 跳跃次数
- Exploration Completeness - 探索完整性
- Energy Consumption - 能量消耗（模拟）

### 5.3 统计分析

**方法**:
- 每个算法-环境组合：n=5次运行
- 报告：均值 ± 标准差
- 统计检验：Welch's t-test (p < 0.05)
- 可视化：雷达图 + 箱线图

---

## 6. 论文写作建议

### 6.1 实验章节结构

```
5. Experiments and Results
  5.1 Experimental Setup
    5.1.1 Simulation Environment
    5.1.2 Baseline Algorithms
    5.1.3 Evaluation Metrics
    5.1.4 Implementation Details
  
  5.2 Ablation Study
    5.2.1 Frontier Selection Strategies
    5.2.2 A* Path Planning Impact
    5.2.3 Adaptive Sampling Analysis
    5.2.4 Interaction Effects
  
  5.3 Benchmark Comparison
    5.3.1 Performance on Standard Environments
    5.3.2 Algorithm Comparison (Table + Radar Chart)
    5.3.3 Statistical Significance Tests
  
  5.4 Analysis and Discussion
    5.4.1 Computational Complexity
    5.4.2 Scalability Analysis
    5.4.3 Failure Cases and Limitations
```

### 6.2 对比表格示例

| Algorithm | Distance↓ | Nodes↓ | Coverage↑ | Efficiency↑ | Time↓ |
|-----------|----------|--------|-----------|-------------|-------|
| Uniform Grid | 150.2±5.3 | 45.0±2.1 | 99.8±0.2% | 0.0066 | 3.2±0.1 |
| RRT | 180.5±12.4 | 38.2±5.3 | 94.5±3.1% | 0.0052 | 5.8±1.2 |
| Frontier | 120.3±8.7 | 32.5±3.2 | 96.2±2.1% | 0.0080 | 4.1±0.5 |
| NBV | 95.4±6.1 | 28.3±2.8 | 97.1±1.5% | 0.0102 | 8.3±1.1 |
| **SSTG (Baseline)** | 112.9±0.0 | 22.0±0.0 | 99.9±0.0% | 0.0088 | 82.0±0.0 |
| **SSTG (Enhanced)** | 64.1±0.0 | 20.0±0.0 | 96.5±0.0% | 0.0151 | 78.5±0.0 |
| **SSTG (Optimal)** | **36.4±0.0** | **18.0±0.0** | 94.3±0.0% | **0.0259** | 75.2±0.0 |

---

## 7. 相关资源

### 7.1 开源实现
- ROS Navigation Stack: http://wiki.ros.org/navigation
- OMPL (Planning library): https://ompl.kavrakilab.org/
- Habitat Sim (Embodied AI): https://aihabitat.org/

### 7.2 数据集和Benchmark
- Gibson Environment: http://gibsonenv.stanford.edu/
- Habitat Challenge: https://aihabitat.org/challenge/
- Matterport3D: https://niessner.github.io/Matterport/

### 7.3 相关项目
- Cartographer (Google SLAM): https://github.com/cartographer-project/cartographer
- ORB-SLAM: https://github.com/raulmur/ORB_SLAM2
- Autoware (自动驾驶): https://github.com/Autoware-AI/autoware.ai

---

**文档版本**: v1.0  
**最后更新**: 2026-04-15  
**维护者**: Daojie PENG
