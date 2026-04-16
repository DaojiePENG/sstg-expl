#!/usr/bin/env python3
"""
Generate all visualizations for paper submission.
Saves results to outputs/final_results/visualizations/
"""
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from collections import defaultdict

# Set style for publication-quality figures
plt.style.use('seaborn-v0_8-paper')
sns.set_palette("husl")
plt.rcParams['figure.dpi'] = 300
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'serif'

OUTPUT_DIR = Path('/home/daojie/sstg-expl/outputs/final_results/visualizations')
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def load_benchmark_results(results_file):
    """Load benchmark results from JSON."""
    with open(results_file, 'r') as f:
        data = json.load(f)
    return data


def parse_results(data):
    """Parse benchmark results into structured format."""
    results_by_algo = defaultdict(lambda: defaultdict(list))

    for result in data['results']:
        algo = result['algorithm']
        env = result['environment']
        am = result.get('additional_metrics', {}) or {}
        results_by_algo[algo][env].append({
            'coverage': result['coverage_ratio'],
            'nodes': result['num_nodes'],
            'distance': result['total_distance'],
            'time': result['computation_time'],
            'avg_obstacle_distance': am.get('avg_obstacle_distance', None),
            'min_obstacle_distance': am.get('min_obstacle_distance', None),
            'mean_nn_distance': am.get('mean_nn_distance', None),
            'nn_distance_std': am.get('nn_distance_std', None),
            'dispersion_uniformity': am.get('dispersion_uniformity', None),
        })

    return results_by_algo


def plot_coverage_comparison(results_by_algo, output_path):
    """Plot 1: Coverage comparison across algorithms and environments."""
    fig, ax = plt.subplots(figsize=(12, 6))

    environments = sorted(list(results_by_algo[list(results_by_algo.keys())[0]].keys()))
    algorithms = sorted(results_by_algo.keys())

    x = np.arange(len(environments))
    width = 0.15

    for i, algo in enumerate(algorithms):
        coverages = []
        errors = []
        for env in environments:
            runs = results_by_algo[algo][env]
            cov_values = [r['coverage'] * 100 for r in runs]
            coverages.append(np.mean(cov_values))
            errors.append(np.std(cov_values))

        ax.bar(x + i * width, coverages, width, label=algo, yerr=errors, capsize=3)

    ax.set_xlabel('Environment')
    ax.set_ylabel('Coverage (%)')
    ax.set_title('Coverage Comparison Across Environments')
    ax.set_xticks(x + width * (len(algorithms) - 1) / 2)
    ax.set_xticklabels(environments, rotation=45, ha='right')
    ax.legend(loc='lower right')
    ax.grid(axis='y', alpha=0.3)
    ax.set_ylim([0, 105])

    plt.tight_layout()
    plt.savefig(output_path / 'coverage_comparison.png', dpi=300, bbox_inches='tight')
    plt.savefig(output_path / 'coverage_comparison.pdf', bbox_inches='tight')
    plt.close()
    print(f"✅ Saved: coverage_comparison.png/pdf")


def plot_efficiency_comparison(results_by_algo, output_path):
    """Plot 2: Efficiency (coverage per distance) comparison."""
    fig, ax = plt.subplots(figsize=(10, 6))

    algorithms = sorted(results_by_algo.keys())

    for algo in algorithms:
        efficiencies = []
        for env, runs in results_by_algo[algo].items():
            for run in runs:
                if run['distance'] > 0:
                    efficiency = (run['coverage'] * 100) / run['distance']
                    efficiencies.append(efficiency)

        if efficiencies:
            ax.boxplot([efficiencies], positions=[algorithms.index(algo)],
                      widths=0.6, patch_artist=True)

    ax.set_xlabel('Algorithm')
    ax.set_ylabel('Coverage Efficiency (% / meter)')
    ax.set_title('Exploration Efficiency Comparison')
    ax.set_xticks(range(len(algorithms)))
    ax.set_xticklabels(algorithms, rotation=45, ha='right')
    ax.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path / 'efficiency_comparison.png', dpi=300, bbox_inches='tight')
    plt.savefig(output_path / 'efficiency_comparison.pdf', bbox_inches='tight')
    plt.close()
    print(f"✅ Saved: efficiency_comparison.png/pdf")


def plot_time_comparison(results_by_algo, output_path):
    """Plot 3: Computation time comparison."""
    fig, ax = plt.subplots(figsize=(10, 6))

    environments = sorted(list(results_by_algo[list(results_by_algo.keys())[0]].keys()))
    algorithms = sorted(results_by_algo.keys())

    x = np.arange(len(environments))
    width = 0.15

    for i, algo in enumerate(algorithms):
        times = []
        errors = []
        for env in environments:
            runs = results_by_algo[algo][env]
            time_values = [r['time'] for r in runs]
            times.append(np.mean(time_values))
            errors.append(np.std(time_values))

        ax.bar(x + i * width, times, width, label=algo, yerr=errors, capsize=3)

    ax.set_xlabel('Environment')
    ax.set_ylabel('Computation Time (s)')
    ax.set_title('Computation Time Comparison')
    ax.set_xticks(x + width * (len(algorithms) - 1) / 2)
    ax.set_xticklabels(environments, rotation=45, ha='right')
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    ax.set_yscale('log')

    plt.tight_layout()
    plt.savefig(output_path / 'time_comparison.png', dpi=300, bbox_inches='tight')
    plt.savefig(output_path / 'time_comparison.pdf', bbox_inches='tight')
    plt.close()
    print(f"✅ Saved: time_comparison.png/pdf")


def plot_heatmap_performance(results_by_algo, output_path):
    """Plot 4: Performance heatmap (algorithm × environment)."""
    algorithms = sorted(results_by_algo.keys())
    environments = sorted(list(results_by_algo[list(results_by_algo.keys())[0]].keys()))

    # Create coverage matrix
    coverage_matrix = np.zeros((len(algorithms), len(environments)))
    for i, algo in enumerate(algorithms):
        for j, env in enumerate(environments):
            runs = results_by_algo[algo][env]
            avg_cov = np.mean([r['coverage'] * 100 for r in runs])
            coverage_matrix[i, j] = avg_cov

    fig, ax = plt.subplots(figsize=(10, 6))
    im = ax.imshow(coverage_matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=100)

    # Set ticks
    ax.set_xticks(np.arange(len(environments)))
    ax.set_yticks(np.arange(len(algorithms)))
    ax.set_xticklabels(environments, rotation=45, ha='right')
    ax.set_yticklabels(algorithms)

    # Add colorbar
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Coverage (%)', rotation=270, labelpad=20)

    # Add text annotations
    for i in range(len(algorithms)):
        for j in range(len(environments)):
            text = ax.text(j, i, f'{coverage_matrix[i, j]:.1f}',
                         ha="center", va="center", color="black", fontsize=8)

    ax.set_title('Coverage Performance Heatmap')
    plt.tight_layout()
    plt.savefig(output_path / 'performance_heatmap.png', dpi=300, bbox_inches='tight')
    plt.savefig(output_path / 'performance_heatmap.pdf', bbox_inches='tight')
    plt.close()
    print(f"✅ Saved: performance_heatmap.png/pdf")


def generate_summary_table(results_by_algo, output_path):
    """Generate summary statistics table."""
    algorithms = sorted(results_by_algo.keys())

    summary = []
    for algo in algorithms:
        all_coverages = []
        all_nodes = []
        all_distances = []
        all_times = []

        for env, runs in results_by_algo[algo].items():
            all_coverages.extend([r['coverage'] * 100 for r in runs])
            all_nodes.extend([r['nodes'] for r in runs])
            all_distances.extend([r['distance'] for r in runs])
            all_times.extend([r['time'] for r in runs])

        summary.append({
            'Algorithm': algo,
            'Avg Coverage (%)': f"{np.mean(all_coverages):.1f} ± {np.std(all_coverages):.1f}",
            'Avg Nodes': f"{np.mean(all_nodes):.1f} ± {np.std(all_nodes):.1f}",
            'Avg Distance (m)': f"{np.mean(all_distances):.1f} ± {np.std(all_distances):.1f}",
            'Avg Time (s)': f"{np.mean(all_times):.1f} ± {np.std(all_times):.1f}"
        })

    # Save as markdown table
    with open(output_path / 'summary_table.md', 'w') as f:
        f.write("# Benchmark Summary Statistics\n\n")
        f.write("| Algorithm | Avg Coverage (%) | Avg Nodes | Avg Distance (m) | Avg Time (s) |\n")
        f.write("|-----------|------------------|-----------|------------------|-------------|\n")
        for row in summary:
            f.write(f"| {row['Algorithm']} | {row['Avg Coverage (%)']} | {row['Avg Nodes']} | "
                   f"{row['Avg Distance (m)']} | {row['Avg Time (s)']} |\n")

    print(f"✅ Saved: summary_table.md")

    return summary


def plot_obstacle_distance_comparison(results_by_algo, output_path):
    """Plot: Average obstacle distance comparison across algorithms."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    environments = sorted(list(results_by_algo[list(results_by_algo.keys())[0]].keys()))
    algorithms = sorted(results_by_algo.keys())

    # Filter out algorithms with no spatial data
    has_data = False
    for algo in algorithms:
        for env in environments:
            runs = results_by_algo[algo][env]
            if runs and runs[0].get('avg_obstacle_distance') is not None:
                has_data = True
                break
        if has_data:
            break

    if not has_data:
        print("  ⚠️ No spatial metrics found, skipping obstacle distance plot")
        plt.close(fig)
        return

    x = np.arange(len(environments))
    width = 0.12
    n_algos = len(algorithms)

    # Plot 1: Average obstacle distance
    ax = axes[0]
    for i, algo in enumerate(algorithms):
        means = []
        stds = []
        for env in environments:
            runs = results_by_algo[algo][env]
            vals = [r['avg_obstacle_distance'] for r in runs if r.get('avg_obstacle_distance') is not None]
            means.append(np.mean(vals) if vals else 0)
            stds.append(np.std(vals) if vals else 0)

        offset = (i - n_algos / 2 + 0.5) * width
        bars = ax.bar(x + offset, means, width, yerr=stds,
                       label=algo, capsize=2, alpha=0.85)

    ax.set_xlabel('Environment')
    ax.set_ylabel('Avg Distance to Nearest Obstacle (m)')
    ax.set_title('Node-to-Obstacle Distance')
    ax.set_xticks(x)
    ax.set_xticklabels([e.replace('_', '\n') for e in environments], fontsize=8)
    ax.legend(fontsize=6, ncol=2, loc='upper right')
    ax.grid(axis='y', alpha=0.3)

    # Plot 2: Dispersion uniformity
    ax = axes[1]
    for i, algo in enumerate(algorithms):
        means = []
        stds = []
        for env in environments:
            runs = results_by_algo[algo][env]
            vals = [r['dispersion_uniformity'] for r in runs if r.get('dispersion_uniformity') is not None]
            means.append(np.mean(vals) if vals else 0)
            stds.append(np.std(vals) if vals else 0)

        offset = (i - n_algos / 2 + 0.5) * width
        bars = ax.bar(x + offset, means, width, yerr=stds,
                       label=algo, capsize=2, alpha=0.85)

    ax.set_xlabel('Environment')
    ax.set_ylabel('Dispersion Uniformity (higher = more uniform)')
    ax.set_title('Node Spacing Uniformity')
    ax.set_xticks(x)
    ax.set_xticklabels([e.replace('_', '\n') for e in environments], fontsize=8)
    ax.legend(fontsize=6, ncol=2, loc='upper right')
    ax.set_ylim(0, 1.05)
    ax.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    fig.savefig(output_path / 'spatial_quality_comparison.pdf', bbox_inches='tight')
    fig.savefig(output_path / 'spatial_quality_comparison.png', bbox_inches='tight')
    plt.close(fig)
    print("  ✅ Spatial quality comparison plot saved")


def plot_spatial_heatmap(results_by_algo, output_path):
    """Plot: Heatmap of spatial quality metrics across algorithms and environments."""
    environments = sorted(list(results_by_algo[list(results_by_algo.keys())[0]].keys()))
    algorithms = sorted(results_by_algo.keys())

    # Check if data exists
    has_data = False
    for algo in algorithms:
        for env in environments:
            runs = results_by_algo[algo][env]
            if runs and runs[0].get('avg_obstacle_distance') is not None:
                has_data = True
                break
        if has_data:
            break
    if not has_data:
        print("  ⚠️ No spatial metrics, skipping spatial heatmap")
        return

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    metrics = [
        ('avg_obstacle_distance', 'Avg Obstacle Distance (m)', 'YlGn'),
        ('mean_nn_distance', 'Mean NN Distance (m)', 'YlOrRd'),
        ('dispersion_uniformity', 'Dispersion Uniformity', 'Blues'),
    ]

    for ax, (metric_key, title, cmap) in zip(axes, metrics):
        matrix = np.zeros((len(algorithms), len(environments)))
        for i, algo in enumerate(algorithms):
            for j, env in enumerate(environments):
                runs = results_by_algo[algo][env]
                vals = [r[metric_key] for r in runs if r.get(metric_key) is not None]
                matrix[i, j] = np.mean(vals) if vals else 0

        sns.heatmap(matrix, annot=True, fmt='.3f', cmap=cmap,
                    xticklabels=[e.replace('_', '\n') for e in environments],
                    yticklabels=algorithms, ax=ax, cbar_kws={'shrink': 0.8})
        ax.set_title(title, fontsize=10)
        ax.tick_params(labelsize=7)

    plt.tight_layout()
    fig.savefig(output_path / 'spatial_quality_heatmap.pdf', bbox_inches='tight')
    fig.savefig(output_path / 'spatial_quality_heatmap.png', bbox_inches='tight')
    plt.close(fig)
    print("  ✅ Spatial quality heatmap saved")


def main():
    """Main function to generate all visualizations."""
    print("="*70)
    print("Generating Paper Visualizations")
    print("="*70)

    # Find most recent benchmark results
    results_dir = Path('/home/daojie/sstg-expl/outputs/benchmarks/results')
    result_files = sorted(results_dir.glob('benchmark_*.json'))

    if not result_files:
        print("❌ No benchmark results found!")
        return

    latest_result = result_files[-1]
    print(f"\nLoading: {latest_result.name}")

    # Load and parse results
    data = load_benchmark_results(latest_result)
    results_by_algo = parse_results(data)

    print(f"\nFound {len(results_by_algo)} algorithms, {len(data['results'])} total experiments")

    # Generate all plots
    print("\nGenerating visualizations...")
    plot_coverage_comparison(results_by_algo, OUTPUT_DIR)
    plot_efficiency_comparison(results_by_algo, OUTPUT_DIR)
    plot_time_comparison(results_by_algo, OUTPUT_DIR)
    plot_heatmap_performance(results_by_algo, OUTPUT_DIR)
    plot_obstacle_distance_comparison(results_by_algo, OUTPUT_DIR)
    plot_spatial_heatmap(results_by_algo, OUTPUT_DIR)

    # Generate summary table
    print("\nGenerating summary table...")
    generate_summary_table(results_by_algo, OUTPUT_DIR)

    print(f"\n{'='*70}")
    print(f"✅ All visualizations saved to: {OUTPUT_DIR}")
    print(f"{'='*70}")


if __name__ == '__main__':
    main()
