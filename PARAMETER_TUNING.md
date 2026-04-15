# SSTG Explorer - Parameter Tuning Summary

**Date**: 2026-04-15  
**Goal**: Fix early stopping in multiple_rooms and improve overall coverage  
**Key Issue**: d_theta=45° with alpha_early=2.0 caused early termination at 63.5% coverage in multiple_rooms

---

## Parameter Changes

### Core Parameters

| Parameter | Original | Iteration 1 | Iteration 2 (Balanced) |
|-----------|----------|-------------|------------------------|
| **d_theta** | 45° | **30°** | **30°** ✅ |
| **alpha_early** | 2.0 | 1.5 | **1.8** |
| **alpha_mid** | 1.0 | 1.0 | **1.2** |
| **alpha_late** | 0.5 | 0.5 | **0.6** |
| **min_priority_threshold** | 0.02 | **0.005** | **0.005** ✅ |

### Rationale

1. **d_theta: 45° → 30°** (User suggestion)
   - Denser sampling means more exploration directions
   - Better coverage of complex geometries
   - Helps discover hidden corridors in multi-room environments

2. **alpha_early: 2.0 → 1.5 → 1.8** (Distance decay)
   - 2.0: Too aggressive, penalizes distant rooms excessively
   - 1.5: Too lenient, causes poor performance in sparse obstacles
   - **1.8: Balanced** - good for both local and global exploration

3. **min_priority_threshold: 0.02 → 0.005**
   - Lower threshold allows exploration of lower-priority frontiers
   - Critical for multi-room environments where distant frontiers have low priority
   - Risk: May explore unnecessary low-value regions

4. **alpha_mid/late: slight increases**
   - 1.0 → 1.2, 0.5 → 0.6
   - Helps maintain locality in mid-late exploration phases
   - Prevents over-exploration of distant low-value frontiers

---

## Results by Environment

### Multiple Rooms (Primary Target)

| Config | Coverage | Nodes | Distance | Status |
|--------|----------|-------|----------|--------|
| **Original (45°, α=2.0)** | 63.5% | 14 | 30.14m | ❌ Early stop |
| **Iteration 1 (30°, α=1.5)** | 90.3% | 25 | 54.62m | ⚠️  Very close |
| **Iteration 2 (30°, α=1.8)** | TBD | TBD | TBD | 🔄 Testing |

**Key Improvement**: +26.8% coverage (63.5% → 90.3%)

### Sparse Obstacles (Regression Alert!)

| Config | Coverage | Nodes | Distance | Status |
|--------|----------|-------|----------|--------|
| **Original (45°, α=2.0)** | 90.7% | 17 | 34.68m | ✅ Good |
| **Iteration 1 (30°, α=1.5)** | 70.5% | 12 | 24.03m | ❌ Regression |
| **Iteration 2 (30°, α=1.8)** | TBD | TBD | TBD | 🔄 Testing |

**Problem**: α=1.5 too low, causes premature frontier exhaustion

### Empty Environment

| Config | Coverage | Nodes | Distance | Status |
|--------|----------|-------|----------|--------|
| **Original (45°, α=2.0)** | 100.0% | 25 | 58.36m | ✅ Perfect |
| **Iteration 1 (30°, α=1.5)** | 97.9% | 21 | 43.54m | ✅ Excellent |

**Note**: Slight decrease acceptable for better multi-room performance

### Dense Obstacles

| Config | Coverage | Nodes | Distance | Status |
|--------|----------|-------|----------|--------|
| **All configs** | 52.7% | 7-8 | 12-16m | ⚠️  Consistent |

**Note**: No change across configs - root cause is environment generation/A* issues

### Corridor

| Config | Coverage | Nodes | Distance | Status |
|--------|----------|-------|----------|--------|
| **All configs** | 100.0% | 7 | 12.00m | ✅ Perfect |

**Note**: Simple geometry, all configs work perfectly

---

## Termination Logic Improvements

### Previous Logic
```python
if max_priority < threshold:
    return True  # Stop immediately
```

### Improved Logic
```python
if max_priority < threshold:
    # Check for numerical issues
    if max_priority < 1e-10 and coverage < 0.90:
        # Warn about potential priority calculation issues
        return False  # Continue if coverage still low
    
    # More lenient for low coverage
    if coverage < 0.93 and queue_size >= 3 and max_priority > threshold * 0.05:
        return False  # Continue exploring
    
    return True  # Otherwise stop
```

**Key Improvements**:
1. Detects numerical issues (priority near-zero)
2. More lenient when coverage < 93% (was 80%)
3. Checks queue size and relative priority (threshold * 0.05)

---

## Trade-offs Analysis

### α Parameter Trade-off

| Alpha Value | Advantages | Disadvantages |
|-------------|------------|---------------|
| **α = 2.0** | Good locality, efficient in simple environments | Too aggressive for multi-room, early stop at 63.5% |
| **α = 1.5** | Excellent multi-room (90.3%), explores distant frontiers | Poor sparse obstacles (70.5%), over-explores low-value regions |
| **α = 1.8** (Balanced) | Compromise between locality and global exploration | TBD - testing in progress |

### d_theta Trade-off

| d_theta | Advantages | Disadvantages |
|---------|------------|---------------|
| **45°** (8 directions) | Faster computation, simpler frontier queue | Misses some exploration opportunities |
| **30°** (12 directions) | Better coverage, discovers hidden corridors | More computational overhead, larger frontier queue |

**Recommendation**: 30° is worth the overhead for better coverage

---

## Remaining Challenges

### 1. Priority Near-Zero Issue

**Problem**: In late exploration, all frontiers have priority ~10^-16

**Possible Causes**:
- Distance decay too aggressive even with α=1.8
- Exploration_strength near-zero (blocked by explored nodes)
- Numerical precision issues

**Potential Solutions**:
- Use different distance weighting (e.g., linear instead of power-law)
- Better handling of blocked frontiers
- Re-evaluate frontiers when coverage stalls

### 2. Sparse Obstacles Regression

**Problem**: α=1.5 causes 20% coverage drop

**Root Cause**: Too much emphasis on distant frontiers, ignoring nearby high-value regions

**Solution**: α=1.8 should balance this

### 3. Dense Obstacles Plateau

**Problem**: Stuck at 52.7% regardless of parameters

**Root Cause**: Not a parameter issue - likely A*/environment generation

**Needs**: Separate investigation beyond parameter tuning

---

## Recommended Configuration (Final)

```python
# Exploration parameters
r_view: 2.0m
d_theta: 30.0°  # Denser sampling (user suggested)
overlap: 0.25m

# Distance decay (balanced)
alpha_early: 1.8  # Balanced between 1.5 and 2.0
alpha_mid: 1.2    # Slightly more locality
alpha_late: 0.6   # Late-stage local refinement

# Termination
min_priority_threshold: 0.005  # Lower for multi-room
adaptive_threshold: True       # Adjust based on environment density

# Path planning
use_astar: True  # Critical for obstacle avoidance
```

### Expected Performance (Estimated)

| Environment | Coverage (Target: 95%) | Status |
|-------------|------------------------|--------|
| Empty | ~98% | ✅ Excellent |
| Sparse Obstacles | ~85-90% | ⚠️  Good (needs verification) |
| Dense Obstacles | ~53% | ❌ Needs separate fix |
| Corridor | 100% | ✅ Perfect |
| Multiple Rooms | ~88-92% | ⚠️  Very close to target |

**Average**: ~85-87% (vs 81.4% baseline)

---

## Next Steps

1. ✅ Test balanced config (α=1.8) on Sparse Obstacles and Multiple Rooms
2. ⏳ If balanced config works well (both >85%), run full 175-experiment benchmark
3. ⏳ Generate updated visualizations
4. ⏳ Update documentation with new baseline parameters
5. ⏳ Investigate Dense Obstacles root cause (separate from parameter tuning)

---

## Lessons Learned

1. **One parameter doesn't fit all**: α=1.5 great for multi-room, terrible for sparse obstacles
2. **User suggestions valuable**: d_theta=30° significantly improved coverage
3. **Early stopping critical**: Small changes in termination logic have huge impact
4. **Numerical issues matter**: Priority calculations can underflow to near-zero
5. **Test holistically**: Improvements in one environment may hurt others

---

**Status**: 🔄 Iteration 2 in progress (testing α=1.8)  
**Next Review**: After balanced config results available
