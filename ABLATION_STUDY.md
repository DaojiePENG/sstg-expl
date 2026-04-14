# SSTG Explorer Ablation Study: Frontier Selection Strategies

**版本**: Baseline + 改进方案设计  
**日期**: 2026-04-15  
**目的**: 记录不同frontier选择策略，用于消融实验对比

---

## 1. Baseline 策略（当前实现）

### 1.1 数学模型

当前实现的frontier选择基于优先级队列，优先级计算公式为：

$$
P_{baseline}(f) = S_{explore}(f) \cdot W_{dist}(f)
$$

其中各项定义如下：

#### 探索强度 $S_{explore}(f)$

探索强度由碰撞检测器计算，反映frontier的可行性：

$$
S_{explore}(f) = \begin{cases}
0 & \text{if hard obstacle (硬障碍物)} \\
\frac{d_{nearest} - d_{repel}}{r_{view} - d_{repel}} & \text{if soft obstacle (距已探索点近)} \\
1 & \text{otherwise (完全可行)}
\end{cases}
$$

参数说明：
- $d_{nearest}$: frontier到最近已探索节点的距离
- $d_{repel} = r_{view} - overlap$: 排斥距离（默认1.75m）
- $r_{view}$: 视野半径（默认2.0m）
- $overlap$: 重叠距离（默认0.25m）

#### 距离权重 $W_{dist}(f)$

距离权重用于倾向选择距离当前位置更近的frontier：

$$
W_{dist}(f) = \frac{1}{1 + \left(\frac{d_{curr}(f)}{r_{view}}\right)^\alpha}
$$

参数说明：
- $d_{curr}(f)$: frontier到**当前位置**的欧氏距离
- $\alpha$: 距离衰减指数，随覆盖率自适应变化：

$$
\alpha(\rho) = \begin{cases}
2.0 & \text{if } \rho < 0.3 \text{ (早期探索)} \\
1.0 & \text{if } 0.3 \leq \rho < 0.7 \text{ (中期探索)} \\
0.5 & \text{if } \rho \geq 0.7 \text{ (后期探索)}
\end{cases}
$$

其中 $\rho$ 为当前覆盖率。

### 1.2 Frontier更新机制

每次机器人移动到新位置后：

1. **生成新frontiers**: 从当前位置沿 $n = 360°/d_{theta}$ 个方向生成候选frontier
2. **更新所有优先级**: 对队列中所有已有frontiers重新计算优先级（因为 $d_{curr}$ 改变了）
3. **移除无效frontiers**: 删除被新探索区域覆盖的frontiers

### 1.3 存在的问题 ⚠️

通过实际观察发现，**机器人会频繁跳动，走大量无用路程**，原因分析：

#### 问题1：距离权重影响不足
尽管公式中包含距离权重 $W_{dist}$，但：
- 当 $d_{curr} = r_{view}$ 时：$W_{dist} = 0.5$（α=2.0）或 $0.67$（α=1.0）
- 当 $d_{curr} = 2 \cdot r_{view}$ 时：$W_{dist} = 0.2$（α=2.0）或 $0.33$（α=1.0）

**结论**: 距离权重衰减不够快，导致远处的高 $S_{explore}$ frontier仍可能被选中。

#### 问题2：探索强度占主导
如果远处有 $S_{explore} = 1.0$ 的frontier（完全可行），而近处有 $S_{explore} = 0.8$ 的frontier：
- 远处：$P = 1.0 \times 0.3 = 0.3$（假设距离2倍r_view）
- 近处：$P = 0.8 \times 0.9 = 0.72$（假设距离0.5倍r_view）

虽然近处优先级更高，但**如果远处有多个高质量frontiers，可能轮流被选中**。

#### 问题3：缺乏路径长度惩罚
当前只考虑**直线距离**，没有考虑：
- 累计路径长度（总旅行距离）
- 往返成本
- 区域集中性

导致机器人可能在地图两端反复跳跃。

---

## 2. 改进方案：距离优先策略

### 2.1 方案A：强化距离权重

增强距离衰减的影响，引入更激进的距离惩罚：

$$
P_{A}(f) = S_{explore}(f) \cdot W_{dist}^{enhanced}(f)
$$

其中：

$$
W_{dist}^{enhanced}(f) = \exp\left(-\beta \cdot \frac{d_{curr}(f)}{r_{view}}\right)
$$

参数说明：
- $\beta$: 距离惩罚系数（建议范围：0.5 - 2.0）
- $\beta$ 越大，距离惩罚越强，越倾向局部探索

**特性分析**：
| 距离 | β=0.5 | β=1.0 | β=2.0 |
|------|-------|-------|-------|
| $d = r_{view}$ | 0.61 | 0.37 | 0.14 |
| $d = 2r_{view}$ | 0.37 | 0.14 | 0.02 |
| $d = 3r_{view}$ | 0.22 | 0.05 | 0.002 |

**优点**：
- 指数衰减更符合"局部优先"直觉
- 容易调参（单一参数β）

**缺点**：
- 可能过度局部化，错过远处的优质frontier
- 需要调参确定合适的β值

---

### 2.2 方案B：双因子加权

分离"探索质量"和"距离成本"，使用可调权重：

$$
P_{B}(f) = \lambda \cdot S_{explore}(f) + (1-\lambda) \cdot W_{dist}(f)
$$

参数说明：
- $\lambda \in [0, 1]$: 探索质量权重
- $1-\lambda$: 距离成本权重
- $\lambda$ 可随覆盖率自适应调整

**自适应策略**：

$$
\lambda(\rho) = \begin{cases}
0.3 & \text{if } \rho < 0.3 \text{ (早期：优先距离)} \\
0.5 & \text{if } 0.3 \leq \rho < 0.7 \text{ (中期：平衡)} \\
0.7 & \text{if } \rho \geq 0.7 \text{ (后期：优先质量)}
\end{cases}
$$

**优点**：
- 探索质量和距离成本解耦，更易理解
- 可根据探索阶段灵活调整权衡

**缺点**：
- 线性加权可能不如乘积形式自然
- 两个分量量纲不同（归一化问题）

---

### 2.3 方案C：累计距离惩罚

考虑机器人的累计移动距离，避免长距离跳跃：

$$
P_{C}(f) = S_{explore}(f) \cdot W_{dist}(f) \cdot W_{travel}(f)
$$

其中：

$$
W_{travel}(f) = \exp\left(-\gamma \cdot \frac{D_{total} + d_{curr}(f)}{D_{avg}}\right)
$$

参数说明：
- $D_{total}$: 当前累计移动总距离
- $d_{curr}(f)$: 到frontier的距离
- $D_{avg}$: 平均每步移动距离（滑动窗口估计）
- $\gamma$: 旅行成本惩罚系数（建议：0.1 - 0.5）

**优点**：
- 考虑全局路径效率
- 避免过长的跳跃
- 鼓励渐进式探索

**缺点**：
- 引入全局状态，计算复杂度略高
- 可能在地图某区域"卡住"

---

### 2.4 方案D：局部聚类优先

倾向于先完成局部区域的探索，再移动到远处：

$$
P_{D}(f) = S_{explore}(f) \cdot W_{dist}(f) \cdot C_{cluster}(f)
$$

其中，聚类因子 $C_{cluster}(f)$ 衡量frontier周围的frontier密度：

$$
C_{cluster}(f) = 1 + \eta \cdot \frac{N_{nearby}(f, r_{cluster})}{N_{total}}
$$

参数说明：
- $N_{nearby}(f, r_{cluster})$: 距离frontier $r_{cluster}$ 范围内的其他frontier数量
- $N_{total}$: 总frontier数量
- $\eta$: 聚类奖励系数（建议：0.5 - 2.0）
- $r_{cluster}$: 聚类半径（建议：$2 \cdot r_{view}$）

**优点**：
- 自然地实现区域集中式探索
- 避免频繁跨区域跳跃

**缺点**：
- 需要额外计算frontier间距离
- 可能延迟对孤立区域的探索

---

### 2.5 方案E：混合自适应策略

结合方案A和方案D，根据探索阶段自适应切换：

$$
P_{E}(f) = S_{explore}(f) \cdot W_{dist}^{enhanced}(f) \cdot [1 + \omega(\rho) \cdot C_{cluster}(f)]
$$

其中：

$$
\omega(\rho) = \begin{cases}
0.0 & \text{if } \rho < 0.2 \text{ (初期：不考虑聚类)} \\
0.5 & \text{if } 0.2 \leq \rho < 0.8 \text{ (中期：考虑聚类)} \\
1.0 & \text{if } \rho \geq 0.8 \text{ (后期：强调聚类)}
\end{cases}
$$

**策略解读**：
- **早期（ρ < 20%）**: 仅强化距离权重，快速覆盖大部分区域
- **中期（20% ≤ ρ < 80%）**: 兼顾距离和聚类，平衡效率和完整性
- **后期（ρ ≥ 80%）**: 强调聚类，集中清理剩余区域

**优点**：
- 结合多种策略优势
- 自适应于探索进度

**缺点**：
- 参数较多，调参复杂
- 实现和调试成本高

---

## 3. 实验设计

### 3.1 对比指标

评估不同策略时，关注以下指标：

| 指标 | 符号 | 说明 | 目标 |
|------|------|------|------|
| **总移动距离** | $D_{total}$ | 机器人累计移动距离 | ↓ 越小越好 |
| **探索节点数** | $N_{nodes}$ | 采样点数量 | ↓ 越少越好（相同覆盖率下） |
| **覆盖率** | $\rho$ | 视野覆盖的自由空间比例 | ↑ 越高越好 |
| **覆盖效率** | $\eta_{cov} = \rho / D_{total}$ | 单位距离覆盖率 | ↑ 越高越好 |
| **路径平滑度** | $\sigma_{step}$ | 步长标准差 | ↓ 越小越稳定 |
| **跳跃次数** | $N_{jump}$ | 步长超过阈值（如1.5×r_view）的次数 | ↓ 越少越好 |
| **完成时间** | $T_{total}$ | 算法运行时间 | ↓ 越快越好 |

### 3.2 测试环境

设计多种复杂度的环境进行测试：

| 环境 | 尺寸 | 复杂度 | 特点 |
|------|------|--------|------|
| **Empty Room** | 10×10m | 简单 | 无障碍，测试基础策略 |
| **Sparse Obstacles** | 10×10m | 中等 | 5个障碍物，测试避障 |
| **Dense Obstacles** | 10×10m | 复杂 | 15个障碍物，测试鲁棒性 |
| **Corridor** | 15×2.5m | 狭长 | 测试狭窄空间 |
| **Multi-room** | 15×10m | 中等 | 多房间，测试跨区域 |
| **Apartment** | 12×12m | 复杂 | 真实户型，综合测试 |

### 3.3 参数配置

每个策略的推荐参数范围：

| 策略 | 关键参数 | 推荐值 | 调参范围 |
|------|---------|--------|---------|
| **Baseline** | α | 2.0 / 1.0 / 0.5 | - |
| **方案A** | β | 1.0 | 0.5 - 2.0 |
| **方案B** | λ | 0.3 / 0.5 / 0.7 | 0.1 - 0.9 |
| **方案C** | γ | 0.2 | 0.1 - 0.5 |
| **方案D** | η, r_cluster | 1.0, 4.0m | 0.5-2.0, 2-6m |
| **方案E** | β, η, ω | 1.0, 1.0, [0,0.5,1] | - |

### 3.4 统计分析

对于每个环境和策略组合：
- 运行 **N=10** 次独立实验（不同随机种子）
- 计算各指标的 **均值** 和 **标准差**
- 使用 **配对t检验** 比较策略间显著性差异（p < 0.05）

---

## 4. 实现建议

### 4.1 代码架构

建议在 `src/core/explorer.py` 中添加策略选择接口：

```python
class FrontierSelectionStrategy(Enum):
    BASELINE = "baseline"
    ENHANCED_DISTANCE = "enhanced_distance"  # 方案A
    DUAL_FACTOR = "dual_factor"              # 方案B
    CUMULATIVE_DISTANCE = "cumulative_distance"  # 方案C
    CLUSTER_PRIORITY = "cluster_priority"    # 方案D
    HYBRID_ADAPTIVE = "hybrid_adaptive"      # 方案E

class SSTGExplorer:
    def __init__(self, ..., strategy: FrontierSelectionStrategy = BASELINE):
        self.strategy = strategy
        # ...
    
    def _compute_priority(self, ...):
        if self.strategy == FrontierSelectionStrategy.BASELINE:
            return self._priority_baseline(...)
        elif self.strategy == FrontierSelectionStrategy.ENHANCED_DISTANCE:
            return self._priority_enhanced_distance(...)
        # ... 其他策略
```

### 4.2 配置参数

在 `src/config.py` 中添加策略专用参数：

```python
@dataclass
class ExplorerConfig:
    # ... 现有参数 ...
    
    # Frontier selection strategy
    frontier_strategy: str = "baseline"
    
    # Strategy-specific parameters
    beta: float = 1.0        # 方案A: 距离惩罚系数
    lambda_weight: float = 0.5  # 方案B: 探索质量权重
    gamma: float = 0.2       # 方案C: 旅行成本惩罚
    eta: float = 1.0         # 方案D: 聚类奖励系数
    r_cluster: float = 4.0   # 方案D: 聚类半径
```

### 4.3 实验脚本

创建专门的实验脚本 `examples/ablation_study.py`：

```python
def run_ablation_study():
    strategies = [
        "baseline",
        "enhanced_distance",
        "dual_factor",
        "cumulative_distance",
        "cluster_priority",
        "hybrid_adaptive"
    ]
    
    environments = ["empty", "obstacles", "corridor", ...]
    
    results = {}
    for env in environments:
        for strategy in strategies:
            for run_id in range(10):
                # 运行实验，记录指标
                result = run_single_experiment(env, strategy, run_id)
                # 保存结果
    
    # 生成对比图表和统计分析
    generate_comparison_report(results)
```

---

## 5. 预期结果

### 5.1 假设

基于理论分析，预期各策略表现：

| 策略 | 总距离 | 覆盖效率 | 跳跃次数 | 适用场景 |
|------|--------|---------|---------|---------|
| **Baseline** | 较高 | 中等 | 多 | 一般 |
| **方案A** | 低 | 高 | 少 | 开阔空间 |
| **方案B** | 中等 | 中等 | 中等 | 通用 |
| **方案C** | 低 | 高 | 少 | 长期任务 |
| **方案D** | 中等 | 中高 | 少 | 多房间 |
| **方案E** | **最低** | **最高** | **最少** | **所有场景** |

### 5.2 权衡分析

不同策略的权衡：

- **Baseline**: 均衡但不突出，作为参考基线
- **方案A**: 简单高效，但可能过度局部
- **方案B**: 灵活可控，但线性加权不够自然
- **方案C**: 考虑全局效率，但实现复杂
- **方案D**: 自动区域聚焦，但密度计算有开销
- **方案E**: 综合最优，但参数多、调试难

---

## 6. 下一步计划

1. **✅ 已完成**: 记录baseline策略（当前版本）
2. **📝 待实现**: 实现5种改进方案
3. **🧪 待测试**: 在6种环境下运行ablation study
4. **📊 待分析**: 统计分析和可视化对比
5. **📄 待撰写**: 实验报告和论文章节

---

## 7. 参考文献

### 相关工作

1. **Frontier-based Exploration**:
   - Yamauchi, B. (1997). "A Frontier-Based Approach for Autonomous Exploration"
   
2. **Coverage Path Planning**:
   - Galceran, E., & Carreras, M. (2013). "A survey on coverage path planning for robotics"
   
3. **Distance-aware Strategies**:
   - Burgard, W., et al. (2000). "Coordinated Multi-Robot Exploration"
   - Holz, D., et al. (2010). "Evaluating the Efficiency of Frontier-based Exploration Strategies"

---

## 附录A：符号表

| 符号 | 含义 | 单位 |
|------|------|------|
| $P(f)$ | Frontier优先级 | - |
| $S_{explore}(f)$ | 探索强度 | [0, 1] |
| $W_{dist}(f)$ | 距离权重 | [0, 1] |
| $d_{curr}(f)$ | 到当前位置距离 | m |
| $d_{nearest}$ | 到最近已探索点距离 | m |
| $r_{view}$ | 视野半径 | m |
| $\rho$ | 覆盖率 | [0, 1] |
| $\alpha$ | 距离衰减指数 | - |
| $\beta$ | 距离惩罚系数 | - |
| $\lambda$ | 探索质量权重 | [0, 1] |
| $\gamma$ | 旅行成本惩罚 | - |
| $\eta$ | 聚类奖励系数 | - |
| $D_{total}$ | 累计移动距离 | m |
| $N_{nodes}$ | 探索节点数 | - |

---

**文档版本**: v1.0  
**最后更新**: 2026-04-15  
**作者**: Daojie PENG & Claude Sonnet 4.5
