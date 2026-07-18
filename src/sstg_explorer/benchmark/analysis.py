"""
Visualization and statistical analysis tools for benchmark results.

This module provides tools for analyzing and visualizing benchmark results,
including radar charts, box plots, and statistical significance tests.
"""

import json
import os
from typing import Dict, List, Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy import stats


class BenchmarkAnalyzer:
    """
    Analyzer for benchmark results.

    Provides statistical analysis and visualization of exploration
    algorithm performance across multiple metrics and environments.

    Args:
        results_file: Path to benchmark results JSON file
    """

    def __init__(self, results_file: Optional[str] = None):
        """Initialize analyzer."""
        self.results = []
        self.metrics = [
            'total_distance',
            'num_nodes',
            'coverage_ratio',
            'coverage_efficiency',
            'computation_time'
        ]

        if results_file is not None:
            self.load_results(results_file)

    def load_results(self, filename: str):
        """Load results from JSON file."""
        with open(filename, 'r') as f:
            data = json.load(f)
        self.results = data['results']
        print(f"Loaded {len(self.results)} benchmark results")

    def get_algorithms(self) -> List[str]:
        """Get list of algorithms in results."""
        return sorted(list(set(r['algorithm'] for r in self.results)))

    def get_environments(self) -> List[str]:
        """Get list of environments in results."""
        return sorted(list(set(r['environment'] for r in self.results)))

    def filter_results(
        self,
        algorithm: Optional[str] = None,
        environment: Optional[str] = None
    ) -> List[Dict]:
        """
        Filter results by algorithm and/or environment.

        Args:
            algorithm: Algorithm name (None = all)
            environment: Environment name (None = all)

        Returns:
            Filtered results list
        """
        filtered = self.results

        if algorithm is not None:
            filtered = [r for r in filtered if r['algorithm'] == algorithm]

        if environment is not None:
            filtered = [r for r in filtered if r['environment'] == environment]

        return filtered

    def compute_statistics(
        self,
        algorithm: str,
        environment: str,
        metric: str
    ) -> Dict:
        """
        Compute statistics for a specific configuration.

        Args:
            algorithm: Algorithm name
            environment: Environment name
            metric: Metric name

        Returns:
            Dictionary with mean, std, min, max, median
        """
        results = self.filter_results(algorithm, environment)
        values = [r[metric] for r in results if metric in r]

        if len(values) == 0:
            return None

        return {
            'mean': np.mean(values),
            'std': np.std(values),
            'min': np.min(values),
            'max': np.max(values),
            'median': np.median(values),
            'n': len(values)
        }

    def plot_radar_chart(
        self,
        environment: str,
        algorithms: Optional[List[str]] = None,
        save_path: Optional[str] = None
    ):
        """
        Create radar chart comparing algorithms.

        Args:
            environment: Environment to analyze
            algorithms: List of algorithms (None = all)
            save_path: Path to save figure (None = display only)
        """
        if algorithms is None:
            algorithms = self.get_algorithms()

        # Metrics for radar chart (normalized)
        radar_metrics = [
            ('Coverage', 'coverage_ratio', False),  # Higher is better
            ('Efficiency', 'coverage_efficiency', False),  # Higher is better
            ('Distance', 'total_distance', True),  # Lower is better (invert)
            ('Nodes', 'num_nodes', True),  # Lower is better (invert)
            ('Time', 'computation_time', True)  # Lower is better (invert)
        ]

        n_metrics = len(radar_metrics)
        angles = np.linspace(0, 2 * np.pi, n_metrics, endpoint=False).tolist()
        angles += angles[:1]  # Close the circle

        # Collect and normalize data
        algo_data = {}
        for algo in algorithms:
            values = []
            for label, metric, invert in radar_metrics:
                stats = self.compute_statistics(algo, environment, metric)
                if stats is None:
                    values.append(0.0)
                else:
                    val = stats['mean']
                    values.append(val)

            algo_data[algo] = values

        # Normalize values to [0, 1]
        normalized_data = {}
        for i, (label, metric, invert) in enumerate(radar_metrics):
            all_values = [algo_data[algo][i] for algo in algorithms]
            vmin = min(all_values)
            vmax = max(all_values)

            for algo in algorithms:
                if algo not in normalized_data:
                    normalized_data[algo] = []

                val = algo_data[algo][i]
                if vmax > vmin:
                    norm_val = (val - vmin) / (vmax - vmin)
                else:
                    norm_val = 1.0

                if invert:
                    norm_val = 1.0 - norm_val

                normalized_data[algo].append(norm_val)

        # Plot radar chart
        fig, ax = plt.subplots(figsize=(10, 8), subplot_kw=dict(projection='polar'))

        colors = plt.cm.tab10(np.linspace(0, 1, len(algorithms)))

        for idx, algo in enumerate(algorithms):
            values = normalized_data[algo]
            values += values[:1]  # Close the circle
            ax.plot(angles, values, 'o-', linewidth=2, label=algo, color=colors[idx])
            ax.fill(angles, values, alpha=0.15, color=colors[idx])

        # Customize plot
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels([label for label, _, _ in radar_metrics])
        ax.set_ylim(0, 1)
        ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_yticklabels(['0.2', '0.4', '0.6', '0.8', '1.0'])
        ax.grid(True)

        plt.title(f'Algorithm Comparison - {environment}', size=16, pad=20)
        plt.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Saved radar chart to {save_path}")
        else:
            plt.show()

        plt.close()

    def plot_box_plots(
        self,
        metric: str,
        environment: str,
        algorithms: Optional[List[str]] = None,
        save_path: Optional[str] = None
    ):
        """
        Create box plots for a specific metric.

        Args:
            metric: Metric to plot
            environment: Environment to analyze
            algorithms: List of algorithms (None = all)
            save_path: Path to save figure
        """
        if algorithms is None:
            algorithms = self.get_algorithms()

        # Collect data
        data = []
        labels = []

        for algo in algorithms:
            results = self.filter_results(algo, environment)
            values = [r[metric] for r in results if metric in r]
            if len(values) > 0:
                data.append(values)
                labels.append(algo)

        # Create box plot
        fig, ax = plt.subplots(figsize=(10, 6))

        bp = ax.boxplot(data, labels=labels, patch_artist=True,
                        showmeans=True, meanline=True)

        # Customize colors
        colors = plt.cm.tab10(np.linspace(0, 1, len(data)))
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)

        # Customize plot
        ax.set_xlabel('Algorithm', fontsize=12)
        ax.set_ylabel(metric.replace('_', ' ').title(), fontsize=12)
        ax.set_title(f'{metric.replace("_", " ").title()} - {environment}',
                     fontsize=14)
        ax.grid(axis='y', alpha=0.3)
        plt.xticks(rotation=45, ha='right')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Saved box plot to {save_path}")
        else:
            plt.show()

        plt.close()

    def statistical_significance_test(
        self,
        algorithm1: str,
        algorithm2: str,
        environment: str,
        metric: str,
        alpha: float = 0.05
    ) -> Tuple[bool, float]:
        """
        Perform statistical significance test between two algorithms.

        Uses Welch's t-test (does not assume equal variances).

        Args:
            algorithm1: First algorithm name
            algorithm2: Second algorithm name
            environment: Environment name
            metric: Metric to compare
            alpha: Significance level (default 0.05)

        Returns:
            Tuple of (is_significant, p_value)
        """
        results1 = self.filter_results(algorithm1, environment)
        results2 = self.filter_results(algorithm2, environment)

        values1 = [r[metric] for r in results1 if metric in r]
        values2 = [r[metric] for r in results2 if metric in r]

        if len(values1) < 2 or len(values2) < 2:
            print("Warning: Not enough samples for statistical test")
            return False, 1.0

        # Welch's t-test
        t_stat, p_value = stats.ttest_ind(values1, values2, equal_var=False)

        is_significant = p_value < alpha

        return is_significant, p_value

    def print_comparison_table(
        self,
        environment: str,
        algorithms: Optional[List[str]] = None
    ):
        """
        Print formatted comparison table.

        Args:
            environment: Environment to analyze
            algorithms: List of algorithms (None = all)
        """
        if algorithms is None:
            algorithms = self.get_algorithms()

        print(f"\n{'='*100}")
        print(f"Algorithm Comparison - {environment}")
        print(f"{'='*100}")
        print(f"{'Algorithm':<20} {'Distance (m)':<18} {'Nodes':<12} "
              f"{'Coverage':<12} {'Efficiency':<15} {'Time (s)':<12}")
        print(f"{'-'*100}")

        for algo in algorithms:
            dist_stats = self.compute_statistics(algo, environment, 'total_distance')
            node_stats = self.compute_statistics(algo, environment, 'num_nodes')
            cov_stats = self.compute_statistics(algo, environment, 'coverage_ratio')
            eff_stats = self.compute_statistics(algo, environment, 'coverage_efficiency')
            time_stats = self.compute_statistics(algo, environment, 'computation_time')

            if all(s is not None for s in [dist_stats, node_stats, cov_stats, eff_stats, time_stats]):
                print(f"{algo:<20} "
                      f"{dist_stats['mean']:6.2f} ± {dist_stats['std']:5.2f}    "
                      f"{node_stats['mean']:5.1f} ± {node_stats['std']:4.1f}  "
                      f"{cov_stats['mean']:5.1%} ± {cov_stats['std']:4.1%}  "
                      f"{eff_stats['mean']:7.5f} ± {eff_stats['std']:6.5f}  "
                      f"{time_stats['mean']:5.2f} ± {time_stats['std']:4.2f}")

        print(f"{'='*100}\n")

    def generate_full_report(
        self,
        output_dir: str = 'outputs/benchmarks/analysis'
    ):
        """
        Generate complete analysis report with all visualizations.

        Args:
            output_dir: Output directory for report
        """
        os.makedirs(output_dir, exist_ok=True)

        algorithms = self.get_algorithms()
        environments = self.get_environments()

        print(f"\n{'='*80}")
        print("Generating Benchmark Analysis Report")
        print(f"{'='*80}\n")

        # Print comparison tables
        for env in environments:
            self.print_comparison_table(env, algorithms)

        # Generate radar charts
        print("Generating radar charts...")
        for env in environments:
            save_path = f'{output_dir}/radar_{env}.png'
            self.plot_radar_chart(env, algorithms, save_path)

        # Generate box plots for each metric
        print("Generating box plots...")
        for env in environments:
            for metric in self.metrics:
                save_path = f'{output_dir}/boxplot_{env}_{metric}.png'
                self.plot_box_plots(metric, env, algorithms, save_path)

        print(f"\n{'='*80}")
        print(f"Report generated in: {output_dir}")
        print(f"{'='*80}\n")
