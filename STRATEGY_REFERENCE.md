# Frontier Selection Strategies - Quick Reference

This document provides a quick reference for the 6 frontier selection strategies implemented for the ablation study.

## Strategy Overview

| Strategy | Key Feature | Best For | Parameters |
|----------|-------------|----------|------------|
| **Baseline** | Original balanced approach | General purpose | α (adaptive) |
| **Enhanced Distance** | Exponential distance decay | Open spaces, local exploration | β = 1.0 |
| **Dual Factor** | Decoupled quality/distance | Flexible weighting | λ = 0.5 (adaptive) |
| **Cumulative Distance** | Global path efficiency | Minimizing travel | γ = 0.2 |
| **Cluster Priority** | Region-focused | Multi-room environments | η = 1.0, r_cluster = 4.0m |
| **Hybrid Adaptive** | Combined A+D | All scenarios (expected best) | β = 1.0, η = 1.0, ω (adaptive) |

## Mathematical Formulas

### Baseline
```
P = S_explore(f) × W_dist(f)
where:
  W_dist(f) = 1 / (1 + (d_curr/r_view)^α)
  α = 2.0 (early) → 1.0 (mid) → 0.5 (late)
```

### Strategy A: Enhanced Distance
```
P = S_explore(f) × exp(-β × d_curr/r_view)
where:
  β = distance penalty coefficient (default: 1.0)
```

### Strategy B: Dual Factor
```
P = λ × S_explore(f) + (1-λ) × W_dist(f)
where:
  λ = exploration quality weight
  λ = 0.3 (early) → 0.5 (mid) → 0.7 (late)
```

### Strategy C: Cumulative Distance
```
P = S_explore(f) × W_dist(f) × exp(-γ × (D_total + d_curr)/D_avg)
where:
  γ = travel cost penalty (default: 0.2)
  D_total = cumulative distance traveled
  D_avg = average step distance
```

### Strategy D: Cluster Priority
```
P = S_explore(f) × W_dist(f) × (1 + η × N_nearby/N_total)
where:
  η = cluster reward coefficient (default: 1.0)
  N_nearby = frontiers within r_cluster
  r_cluster = cluster radius (default: 4.0m = 2×r_view)
```

### Strategy E: Hybrid Adaptive
```
P = S_explore(f) × exp(-β×d/r_view) × [1 + ω(ρ) × η×N_nearby/N_total]
where:
  Combines exponential distance (A) + clustering (D)
  ω(ρ) = 0.0 (ρ<20%) → 0.5 (20%≤ρ<80%) → 1.0 (ρ≥80%)
```

## Quick Usage

### Python Code
```python
from src.core.explorer import SSTGExplorer
from src.config import FrontierSelectionStrategy
from simulation.simple_env import create_environment

# Create environment
env = create_environment('obstacles', width=10.0, height=10.0, num_obstacles=5)
grid = env.get_occupancy_map()
start = env.get_start_pose()

# Use specific strategy
explorer = SSTGExplorer(
    r_view=2.0,
    d_theta=45.0,
    overlap=0.25,
    frontier_strategy=FrontierSelectionStrategy.ENHANCED_DISTANCE  # or any other
)

result = explorer.explore(grid, start)
```

### Command Line

#### Quick Comparison (all strategies, one environment)
```bash
python examples/strategy_comparison.py --mode quick --env sparse_obstacles
```

#### Full Ablation Study
```bash
python examples/strategy_comparison.py --mode full --runs 10
```

Output saved to: `outputs/ablation_study/results/ablation_study_results.json`

## Evaluation Metrics

The comparison tool computes the following metrics:

| Metric | Description | Target |
|--------|-------------|--------|
| **total_distance** | Cumulative travel distance | ↓ minimize |
| **num_nodes** | Number of exploration nodes | ↓ fewer is better |
| **coverage_ratio** | Percentage of free space covered | ↑ maximize |
| **coverage_efficiency** | Coverage per unit distance | ↑ maximize |
| **step_mean** | Average step length | - |
| **step_std** | Step length standard deviation | ↓ more consistent |
| **jump_count** | Steps > 1.5×r_view | ↓ minimize |
| **total_time** | Algorithm runtime | ↓ faster is better |

## Expected Performance

Based on theoretical analysis:

| Metric | Baseline | Enhanced (A) | Dual (B) | Cumulative (C) | Cluster (D) | Hybrid (E) |
|--------|----------|--------------|----------|----------------|-------------|------------|
| Distance | Medium | **Low** | Medium | **Low** | Medium | **Lowest** |
| Jumps | High | **Low** | Medium | **Low** | Low | **Lowest** |
| Efficiency | Medium | High | Medium | **High** | Medium-High | **Highest** |
| Adaptability | Medium | Low | High | Medium | Medium | **Highest** |

## Parameter Tuning

If you want to experiment with different parameters, modify `src/config.py`:

```python
@dataclass
class ExplorerConfig:
    # Strategy A
    beta: float = 1.0  # Try: 0.5, 1.0, 1.5, 2.0
    
    # Strategy B
    lambda_weight: float = 0.5  # Try: 0.3, 0.5, 0.7
    
    # Strategy C
    gamma: float = 0.2  # Try: 0.1, 0.2, 0.3, 0.5
    
    # Strategy D
    eta: float = 1.0  # Try: 0.5, 1.0, 1.5, 2.0
    r_cluster: float = 4.0  # Try: 2.0, 4.0, 6.0
```

## Troubleshooting

### Strategy not changing behavior?
- Check that you're passing `frontier_strategy` to SSTGExplorer
- Verify strategy enum value: `print(explorer.config.frontier_strategy)`

### Comparison script too slow?
- Use `--mode quick` for single environment test
- Reduce `--runs` parameter (default 5, try 3)
- Test on simpler environments first (empty, sparse_obstacles)

### Import errors?
```python
# Make sure to import from correct locations
from src.config import FrontierSelectionStrategy  # not ExplorerConfig
from src.core.explorer import SSTGExplorer
```

## Reference

For detailed mathematical derivations and experimental design, see:
- [ABLATION_STUDY.md](ABLATION_STUDY.md) - Full documentation with math formulas
- [examples/strategy_comparison.py](examples/strategy_comparison.py) - Implementation

---

**Version**: 1.0  
**Last Updated**: 2026-04-15  
**Status**: ✅ All strategies implemented and tested
