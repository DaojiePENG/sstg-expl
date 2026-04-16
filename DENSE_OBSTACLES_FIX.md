# SSTG Explorer - Dense Obstacles环境改进总结

**日期**: 2026-04-15  
**问题**: SSTG算法在Dense Obstacles环境下完全失败（仅9.3% coverage）

---

## 问题诊断

### 1. 环境生成Bug
**发现**: 起始位置(5.0, 5.0)被障碍物占据  
**原因**: `RoomWithObstacles`类随机放置障碍物时未避开起始位置  
**影响**: SSTG从障碍物内部开始，无法移动

**修复**: [simulation/simple_env.py:141-175](simulation/simple_env.py)
- 在放置障碍物前检查与起始位置的距离
- 保持0.8m safe_radius区域清空
- 添加corner距离检查防止障碍物边缘重叠起始点

```python
# Check if obstacle overlaps with start position safe zone
safe_radius = 0.8  # Keep 0.8m radius around start clear
dist_to_start = np.sqrt((obs_center_x - start_x)**2 + (obs_center_y - start_y)**2)
if dist_to_start < safe_radius or min_corner_dist < safe_radius:
    continue  # Skip this obstacle placement
```

### 2. 优先级阈值过高
**发现**: `min_priority_threshold = 0.1`，但dense环境中前沿点优先级只有0.022-0.081  
**原因**: 密集障碍物导致exploration_strength降低  
**影响**: 提前终止探索

**修复**: [src/config.py:72-76](src/config.py)
- 降低阈值：0.1 → 0.02
- 添加自适应阈值机制
- 根据环境密度动态调整

```python
min_priority_threshold: float = 0.02  # Lowered from 0.1
adaptive_threshold: bool = True       # Enable adaptive adjustment
density_threshold: float = 0.20       # Trigger at 20% occupied
```

### 3. 停止条件过于严格
**发现**: 单一优先级检查导致过早停止  
**原因**: 未考虑环境复杂度和当前覆盖率  
**影响**: 在密集环境下过早放弃

**修复**: [src/core/explorer.py:498-526](src/core/explorer.py)
- 在密集环境(>20% occupied)下放宽条件
- 如果覆盖率<85%且优先级>阈值×0.3，继续探索
- 自适应阈值计算：`threshold / (1.0 + (density - 0.20) / 0.10)`

```python
if hasattr(self, 'environment_density') and self.environment_density > density_thresh:
    coverage = self._get_current_coverage()
    if coverage < 0.85 and max_priority > threshold * 0.3:
        return False  # Keep going
```

### 4. 前沿点评分未考虑环境密集度
**发现**: 密集环境中前沿点优先级被过度惩罚  
**原因**: distance weight衰减过快  
**影响**: 所有前沿点优先级都很低

**修复**: [src/core/explorer.py:618-647](src/core/explorer.py)
- 在Enhanced Distance策略中添加density bonus
- 密集度>20%时应用最高3×boost
- 鼓励在困难环境下继续探索

```python
if self.environment_density > density_thresh:
    density_excess = self.environment_density - density_thresh
    density_bonus = 1.0 + min(density_excess / 0.10, 2.0)  # Up to 3x
    priority *= density_bonus
```

### 5. 直线路径检查在密集环境下失败（关键问题）
**发现**: SSTG Enhanced生成的前沿点优先级0.3679（很高），但探索仍停在第一步  
**原因**: 使用直线路径检查，被障碍物阻挡  
**影响**: 即使有高优先级前沿点也无法到达

**修复**: [src/config.py:63-64](src/config.py)
- 默认启用A*路径规划：`use_astar = True`（从False改为True）
- A*可以找到绕过障碍物的可行路径
- 仅增加少量计算开销，但显著提升鲁棒性

---

## 改进效果

### Dense Obstacles环境（15个障碍物）

| 算法 | 改进前 Coverage | 改进后 Coverage | 提升幅度 |
|------|----------------|----------------|---------|
| SSTG (Baseline) | 9.3% | **50.5%** | **+444%** ✅ |
| SSTG (Enhanced) | 9.3% | **52.7%** | **+467%** ✅ |
| SSTG (Optimal) | 46.2% | 46.2% | 0% |

### 关键指标改善

**SSTG (Enhanced)** 改进前后对比：
- Coverage: 9.3% → 52.7% (+43.4 percentage points)
- Distance: 0.00m → 12.90m (从无法移动到正常探索)
- Nodes: 1 → 7.0 (从卡在起点到探索7个节点)
- Status: ❌ 失败 → ✅ 部分成功

### 与Baseline算法对比

虽然改进后仍未达到baseline水平，但差距显著缩小：

| 算法 | Coverage | Gap to Best |
|------|----------|------------|
| **Uniform Grid** | 96.9% | - |
| **Frontier** | 96.7% | -0.2% |
| **NBV** | 96.1% | -0.8% |
| **SSTG (Enhanced)** | **52.7%** | **-44.2%** |
| ~~SSTG (Enhanced) before~~ | ~~9.3%~~ | ~~-87.6%~~ |

---

## 改进清单

### ✅ 已完成
1. ✅ 修复环境生成bug - 起始位置避开障碍物
2. ✅ 降低min_priority_threshold: 0.1 → 0.02
3. ✅ 添加环境密集度检测和自适应阈值
4. ✅ 改进停止条件 - 在密集环境下更宽松
5. ✅ 前沿点评分增加density bonus
6. ✅ 默认启用A*路径规划（最关键）

### 📋 后续改进方向
1. ⬜ 进一步调优前沿点评分函数
2. ⬜ 改进narrow passage检测和处理
3. ⬜ 增加局部重规划机制
4. ⬜ 考虑多尺度采样策略
5. ⬜ 添加obstacle-aware frontier generation

---

## 技术总结

### 最关键的改进
**启用A*路径规划** - 这是从9%提升到50%的关键因素。直线路径检查在密集障碍环境下成功率极低，A*的绕行能力至关重要。

### 设计决策
1. **默认启用A***: 虽然增加计算开销，但鲁棒性提升远超成本
2. **自适应阈值**: 避免"一刀切"的停止条件，根据环境复杂度调整
3. **Density bonus**: 在困难环境下给予算法更多"耐心"
4. **保守的safe_radius**: 0.8m确保即使在视野2.0m的情况下起始位置也有充足缓冲

### 性能平衡
- **Coverage**: 9.3% → 52.7% (+43.4%)
- **Time**: 0.02s → ~1-2s (可接受的增长)
- **Distance**: 0m → 12.90m (实际执行了探索任务)

---

## 结论

通过5个核心改进（环境修复、阈值调整、自适应机制、评分优化、A*启用），SSTG在Dense Obstacles环境下的表现从**完全失败**提升到**部分成功**。

虽然仍未达到Uniform Grid (96.9%)的水平，但已经证明了SSTG在复杂环境下的可行性。剩余的差距主要来自：
1. 前沿点生成策略在密集障碍下的局限性
2. 局部最优问题（被困在某个区域）
3. 缺少全局规划层次

**下一步建议**: 重新运行完整benchmark (175实验)来验证改进在所有环境下的影响。

---

**文件修改清单**:
- `simulation/simple_env.py` - 修复障碍物放置逻辑
- `src/config.py` - 调整默认参数
- `src/core/explorer.py` - 改进停止条件和优先级计算
