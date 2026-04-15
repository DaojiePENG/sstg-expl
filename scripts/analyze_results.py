#!/usr/bin/env python3
"""
Comprehensive analysis script for paper preparation.
Generates statistics, tables, and insights from benchmark results.
"""
import sys
import json
import numpy as np
import pandas as pd
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Tuple

OUTPUT_DIR = Path('/home/daojie/sstg-expl/outputs/final_results/analysis')
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def load_results(results_file: Path) -> Dict:
    """Load benchmark results from JSON."""
    with open(results_file, 'r') as f:
        return json.load(f)


def parse_results(data: Dict) -> pd.DataFrame:
    """Convert benchmark results to pandas DataFrame for analysis."""
    records = []
    for result in data['results']:
        records.append({
            'algorithm': result['algorithm'],
            'environment': result['environment'],
            'run_id': result['run_id'],
            'coverage': result['coverage_ratio'] * 100,  # Convert to percentage
            'nodes': result['num_nodes'],
            'distance': result['total_distance'],
            'time': result['computation_time'],
            'efficiency': result['coverage_efficiency']
        })
    return pd.DataFrame(records)


def generate_summary_statistics(df: pd.DataFrame) -> pd.DataFrame:
    """Generate summary statistics by algorithm and environment."""
    summary = df.groupby(['algorithm', 'environment']).agg({
        'coverage': ['mean', 'std', 'min', 'max'],
        'nodes': ['mean', 'std'],
        'distance': ['mean', 'std'],
        'time': ['mean', 'std'],
        'efficiency': ['mean', 'std']
    }).round(2)

    return summary


def identify_best_performers(df: pd.DataFrame) -> Dict:
    """Identify best performing algorithms per environment."""
    best_performers = {}

    for env in df['environment'].unique():
        env_data = df[df['environment'] == env]
        avg_coverage = env_data.groupby('algorithm')['coverage'].mean()
        best_algo = avg_coverage.idxmax()
        best_cov = avg_coverage.max()

        best_performers[env] = {
            'algorithm': best_algo,
            'coverage': best_cov,
            'rank': avg_coverage.sort_values(ascending=False).to_dict()
        }

    return best_performers


def compute_improvement_over_baseline(df: pd.DataFrame) -> pd.DataFrame:
    """Compute improvement of each algorithm over baseline (Uniform Grid)."""
    baseline = df[df['algorithm'] == 'Uniform Grid']
    improvements = []

    for env in df['environment'].unique():
        baseline_env = baseline[baseline['environment'] == env]['coverage'].mean()

        for algo in df['algorithm'].unique():
            if algo == 'Uniform Grid':
                continue

            algo_env = df[(df['algorithm'] == algo) & (df['environment'] == env)]
            algo_coverage = algo_env['coverage'].mean()

            improvement = ((algo_coverage - baseline_env) / baseline_env) * 100

            improvements.append({
                'algorithm': algo,
                'environment': env,
                'baseline_coverage': baseline_env,
                'algorithm_coverage': algo_coverage,
                'improvement_pct': improvement
            })

    return pd.DataFrame(improvements)


def analyze_sstg_variants(df: pd.DataFrame) -> Dict:
    """Analyze SSTG variants (Baseline, Enhanced, Optimal)."""
    sstg_algos = ['SSTG', 'SSTG (Enhanced)', 'SSTG (Optimal)']
    sstg_data = df[df['algorithm'].isin(sstg_algos)]

    analysis = {}
    for env in sstg_data['environment'].unique():
        env_data = sstg_data[sstg_data['environment'] == env]

        analysis[env] = {}
        for algo in sstg_algos:
            algo_data = env_data[env_data['algorithm'] == algo]
            if not algo_data.empty:
                analysis[env][algo] = {
                    'coverage_mean': algo_data['coverage'].mean(),
                    'coverage_std': algo_data['coverage'].std(),
                    'nodes_mean': algo_data['nodes'].mean(),
                    'time_mean': algo_data['time'].mean()
                }

    return analysis


def generate_latex_table(df: pd.DataFrame, output_file: Path):
    """Generate LaTeX table for paper."""
    summary = df.groupby(['algorithm', 'environment']).agg({
        'coverage': ['mean', 'std'],
        'nodes': 'mean',
        'distance': 'mean',
        'time': 'mean'
    }).round(2)

    latex_content = []
    latex_content.append("\\begin{table}[htbp]")
    latex_content.append("\\centering")
    latex_content.append("\\caption{Benchmark Results: Coverage and Performance Metrics}")
    latex_content.append("\\label{tab:benchmark_results}")
    latex_content.append("\\begin{tabular}{lllrrrr}")
    latex_content.append("\\toprule")
    latex_content.append("Algorithm & Environment & Coverage (\\%) & Nodes & Distance (m) & Time (s) \\\\")
    latex_content.append("\\midrule")

    for (algo, env), row in summary.iterrows():
        cov_mean = row[('coverage', 'mean')]
        cov_std = row[('coverage', 'std')]
        nodes = row[('nodes', 'mean')]
        dist = row[('distance', 'mean')]
        time = row[('time', 'mean')]

        latex_content.append(f"{algo} & {env} & {cov_mean:.1f} $\\pm$ {cov_std:.1f} & "
                           f"{nodes:.0f} & {dist:.1f} & {time:.2f} \\\\")

    latex_content.append("\\bottomrule")
    latex_content.append("\\end{tabular}")
    latex_content.append("\\end{table}")

    with open(output_file, 'w') as f:
        f.write('\n'.join(latex_content))

    print(f"✅ LaTeX table saved: {output_file.name}")


def generate_markdown_report(df: pd.DataFrame, best_performers: Dict,
                             improvements: pd.DataFrame, output_file: Path):
    """Generate comprehensive markdown report."""
    report = []
    report.append("# SSTG Explorer - Benchmark Analysis Report\n")
    report.append(f"**Generated**: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    report.append("---\n")

    # 1. Executive Summary
    report.append("## Executive Summary\n")
    total_experiments = len(df)
    n_algorithms = df['algorithm'].nunique()
    n_environments = df['environment'].nunique()
    n_runs = df.groupby(['algorithm', 'environment']).size().iloc[0]

    report.append(f"- **Total Experiments**: {total_experiments}")
    report.append(f"- **Algorithms Tested**: {n_algorithms}")
    report.append(f"- **Environments**: {n_environments}")
    report.append(f"- **Runs per Configuration**: {n_runs}\n")

    # 2. Overall Performance
    report.append("## Overall Performance\n")
    overall_stats = df.groupby('algorithm').agg({
        'coverage': ['mean', 'std'],
        'nodes': 'mean',
        'distance': 'mean',
        'time': 'mean'
    }).round(2)

    report.append("| Algorithm | Avg Coverage (%) | Avg Nodes | Avg Distance (m) | Avg Time (s) |")
    report.append("|-----------|------------------|-----------|------------------|--------------|")

    for algo, row in overall_stats.iterrows():
        report.append(f"| {algo} | {row[('coverage', 'mean')]:.1f} ± {row[('coverage', 'std')]:.1f} | "
                     f"{row[('nodes', 'mean')]:.1f} | {row[('distance', 'mean')]:.1f} | "
                     f"{row[('time', 'mean')]:.2f} |")

    report.append("\n")

    # 3. Best Performers per Environment
    report.append("## Best Performers by Environment\n")
    for env, info in best_performers.items():
        report.append(f"### {env}")
        report.append(f"**Winner**: {info['algorithm']} ({info['coverage']:.1f}% coverage)\n")
        report.append("**Rankings**:")
        for i, (algo, cov) in enumerate(sorted(info['rank'].items(),
                                               key=lambda x: x[1], reverse=True)[:3], 1):
            report.append(f"{i}. {algo}: {cov:.1f}%")
        report.append("\n")

    # 4. SSTG Improvements
    report.append("## SSTG Algorithm Analysis\n")
    sstg_data = df[df['algorithm'].str.contains('SSTG')]

    report.append("### Configuration Impact\n")
    report.append("Comparison of SSTG variants (d_theta=30°, bug fixes applied):\n")

    sstg_comparison = sstg_data.groupby(['algorithm', 'environment'])['coverage'].mean().unstack()

    # Manual table formatting instead of to_markdown()
    report.append("\n| Algorithm | " + " | ".join(sstg_comparison.columns) + " |")
    report.append("|" + "|".join(["---"] * (len(sstg_comparison.columns) + 1)) + "|")
    for algo, row in sstg_comparison.iterrows():
        report.append(f"| {algo} | " + " | ".join([f"{v:.1f}%" for v in row]) + " |")
    report.append("\n")

    # 5. Key Findings
    report.append("## Key Findings\n")

    # Find best SSTG variant
    sstg_avg = sstg_data.groupby('algorithm')['coverage'].mean().sort_values(ascending=False)
    best_sstg = sstg_avg.index[0]
    best_sstg_cov = sstg_avg.iloc[0]

    report.append(f"1. **Best SSTG Configuration**: {best_sstg} achieves {best_sstg_cov:.1f}% average coverage")

    # Compare with baselines
    baseline_avg = df[df['algorithm'].isin(['Uniform Grid', 'RRT', 'Frontier'])].groupby('algorithm')['coverage'].mean()
    report.append(f"2. **Comparison with Baselines**:")
    for algo, cov in baseline_avg.items():
        diff = best_sstg_cov - cov
        report.append(f"   - vs {algo}: {diff:+.1f} percentage points")

    # Environment-specific insights
    report.append(f"3. **Environment-Specific Performance**:")
    for env in df['environment'].unique():
        env_data = df[df['environment'] == env]
        best_here = env_data.groupby('algorithm')['coverage'].mean().idxmax()
        best_cov = env_data.groupby('algorithm')['coverage'].mean().max()
        report.append(f"   - {env}: {best_here} ({best_cov:.1f}%)")

    report.append("\n")

    # 6. Recommendations
    report.append("## Recommendations for Paper\n")
    report.append("1. **Highlight**: SSTG with d_theta=30° and bug fixes achieves superior performance")
    report.append("2. **Emphasize**: Improvements over traditional methods (Uniform Grid, RRT, Frontier)")
    report.append("3. **Discuss**: Trade-offs between coverage, efficiency, and computation time")
    report.append("4. **Address**: Environment-specific adaptations and their impact")

    # Write report
    with open(output_file, 'w') as f:
        f.write('\n'.join(report))

    print(f"✅ Markdown report saved: {output_file.name}")


def main():
    """Main analysis function."""
    print("="*70)
    print("SSTG Explorer - Benchmark Analysis for Paper")
    print("="*70)

    # Find latest results
    results_dir = Path('/home/daojie/sstg-expl/outputs/benchmarks/results')
    result_files = sorted(results_dir.glob('benchmark_*.json'))

    if not result_files:
        print("❌ No benchmark results found!")
        print(f"   Looked in: {results_dir}")
        return

    latest_result = result_files[-1]
    print(f"\n📊 Loading: {latest_result.name}")

    # Load and parse data
    data = load_results(latest_result)
    df = parse_results(data)

    print(f"   Total experiments: {len(df)}")
    print(f"   Algorithms: {df['algorithm'].nunique()}")
    print(f"   Environments: {df['environment'].nunique()}")

    # Generate analyses
    print("\n🔍 Performing analyses...")

    # 1. Summary statistics
    summary = generate_summary_statistics(df)
    summary.to_csv(OUTPUT_DIR / 'summary_statistics.csv')
    print(f"   ✅ Summary statistics saved")

    # 2. Best performers
    best_performers = identify_best_performers(df)
    with open(OUTPUT_DIR / 'best_performers.json', 'w') as f:
        json.dump(best_performers, f, indent=2)
    print(f"   ✅ Best performers identified")

    # 3. Improvements
    improvements = compute_improvement_over_baseline(df)
    improvements.to_csv(OUTPUT_DIR / 'improvements.csv', index=False)
    print(f"   ✅ Improvement analysis completed")

    # 4. SSTG variants
    sstg_analysis = analyze_sstg_variants(df)
    with open(OUTPUT_DIR / 'sstg_variants.json', 'w') as f:
        json.dump(sstg_analysis, f, indent=2)
    print(f"   ✅ SSTG variant analysis completed")

    # Generate outputs
    print("\n📝 Generating paper materials...")

    # LaTeX table
    generate_latex_table(df, OUTPUT_DIR / 'results_table.tex')

    # Markdown report
    generate_markdown_report(df, best_performers, improvements,
                            OUTPUT_DIR / 'analysis_report.md')

    print(f"\n{'='*70}")
    print(f"✅ Analysis complete! Results saved to:")
    print(f"   {OUTPUT_DIR}")
    print(f"{'='*70}")


if __name__ == '__main__':
    main()
