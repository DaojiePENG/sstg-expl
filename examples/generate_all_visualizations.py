"""
Generate visualizations for all algorithm-environment combinations.

This script runs each algorithm on each environment and saves:
1. Static result visualization (PNG)
2. Exploration animation (GIF)

Output structure:
    outputs/visualizations/
        ├── {environment}/
        │   ├── static/
        │   │   ├── {algorithm}.png
        │   ├── animations/
        │   │   ├── {algorithm}.gif
        └── index.html (browse all results)
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from pathlib import Path
from simulation.benchmark import BenchmarkRunner
from simulation.simple_env import create_environment
from src.utils.visualization import visualize_exploration, create_exploration_animation

print("="*80)
print("SSTG Explorer - Comprehensive Visualization Generator")
print("="*80)
print()

# Create output directory
viz_dir = Path('outputs/visualizations')
viz_dir.mkdir(parents=True, exist_ok=True)

# Define algorithms
algorithms = [
    'uniform_grid',
    'rrt',
    'frontier',
    'nbv',
    'sstg',
    'sstg_enhanced',
    'sstg_optimal',
]

# Define environments
environments = [
    create_environment('empty', width=10.0, height=10.0),
    create_environment('obstacles', width=10.0, height=10.0, num_obstacles=5, seed=42),
    create_environment('obstacles', width=10.0, height=10.0, num_obstacles=15, seed=43),
    create_environment('corridor', length=15.0, width=2.5),
    create_environment('multiple_rooms', width=15.0, height=10.0),
    create_environment('l_corridor'),
    create_environment('maze', width=12.0, height=12.0),
    create_environment('dense_obstacles', width=10.0, height=10.0, num_obstacles=15, seed=43),
    create_environment('narrow_passages', width=15.0, height=10.0),
    create_environment('warehouse', width=15.0, height=12.0),
]

environments[0].name = 'empty'
environments[1].name = 'sparse_obstacles'
environments[2].name = 'dense_obstacles_v1'
environments[3].name = 'corridor'
environments[4].name = 'multiple_rooms'
environments[5].name = 'l_shaped_corridor'
environments[6].name = 'maze'
environments[7].name = 'dense_obstacles'
environments[8].name = 'narrow_passages'
environments[9].name = 'warehouse'

# Algorithm parameters
algorithm_kwargs = {
    'uniform_grid': {'grid_spacing': 2.0, 'visit_order': 'nearest'},
    'rrt': {'max_iterations': 500, 'step_size': 1.0},
    'frontier': {'target_coverage': 0.95, 'max_iterations': 500},
    'nbv': {'n_candidates': 50, 'target_coverage': 0.95, 'max_iterations': 500},
    'sstg': {'d_theta': 30.0, 'target_coverage': 0.95},
    'sstg_enhanced': {'d_theta': 30.0, 'target_coverage': 0.95, 'beta': 1.0},
    'sstg_optimal': {'d_theta': 30.0, 'target_coverage': 0.95, 'beta': 1.0}
}

# Create runner
runner = BenchmarkRunner(
    output_dir='outputs/benchmarks',
    num_runs=1,  # Only 1 run for visualization
    seed=42
)

total_combos = len(algorithms) * len(environments)
current = 0

print(f"Generating visualizations for {len(algorithms)} algorithms × {len(environments)} environments")
print(f"Total: {total_combos} visualizations")
print(f"Output directory: {viz_dir}")
print("="*80)
print()

# Track results for summary
viz_summary = []

for env in environments:
    print(f"\n{'='*80}")
    print(f"Environment: {env.name.upper()}")
    print(f"{'='*80}")

    # Create subdirectories for this environment
    env_dir = viz_dir / env.name
    static_dir = env_dir / 'static'
    anim_dir = env_dir / 'animations'

    static_dir.mkdir(parents=True, exist_ok=True)
    anim_dir.mkdir(parents=True, exist_ok=True)

    for algo_name in algorithms:
        current += 1
        progress = (current / total_combos) * 100

        print(f"\n  [{current}/{total_combos}] {algo_name}... ", end='', flush=True)

        try:
            # Create algorithm instance
            kwargs = algorithm_kwargs.get(algo_name, {})
            kwargs['r_view'] = 2.0
            algorithm = runner.create_algorithm(algo_name, **kwargs)

            # Get environment data
            occupancy_grid = env.get_occupancy_map()
            start_pose = env.get_start_pose()

            # Handle data type differences
            if 'sstg' in algorithm.name.lower():
                grid_input = occupancy_grid
            else:
                grid_input = occupancy_grid.data if hasattr(occupancy_grid, 'data') else occupancy_grid

            # Run exploration (no visualization during run)
            result = algorithm.explore(grid_input, start_pose, visualizer=None)

            # Extract metrics
            metadata = result.get('metadata', {})
            nodes = result.get('nodes', [])
            coverage = metadata.get('coverage_ratio', 0.0)
            distance = metadata.get('total_distance', 0.0)

            # Generate clean algorithm name for filenames
            algo_clean_name = algorithm.name.replace(' ', '_').replace('(', '').replace(')', '').lower()

            # Create title with metrics
            title = (f"{algorithm.name} - {env.name}\n"
                    f"Coverage: {coverage:.1%} | Distance: {distance:.1f}m | Nodes: {len(nodes)}")

            # 1. Generate static visualization
            static_path = static_dir / f"{algo_clean_name}.png"
            visualize_exploration(
                occupancy_grid=occupancy_grid,
                explored_nodes=nodes,
                r_view=kwargs.get('r_view', 2.0),
                show_coverage=True,
                show_connections=True,
                save_path=str(static_path),
                figsize=(10, 10),
                dpi=150,
                title=title
            )

            # 2. Generate animation (only if we have multiple nodes)
            if len(nodes) > 1:
                anim_path = anim_dir / f"{algo_clean_name}.gif"
                try:
                    create_exploration_animation(
                        occupancy_grid=occupancy_grid,
                        explored_nodes=nodes,
                        r_view=kwargs.get('r_view', 2.0),
                        save_path=str(anim_path),
                        figsize=(10, 10),
                        fps=2  # 2 frames per second for smooth viewing
                    )
                    has_animation = True
                except Exception as anim_error:
                    print(f"⚠️ Animation failed: {anim_error} ", end='')
                    has_animation = False
            else:
                has_animation = False

            status = "✅" if coverage > 0.5 else "⚠️" if coverage > 0.1 else "❌"
            print(f"{status} Cov:{coverage:.1%} Nodes:{len(nodes)} {'(+anim)' if has_animation else ''}")

            # Record for summary
            viz_summary.append({
                'environment': env.name,
                'algorithm': algorithm.name,
                'coverage': coverage,
                'distance': distance,
                'nodes': len(nodes),
                'static_file': str(static_path.relative_to(viz_dir)),
                'anim_file': str(anim_path.relative_to(viz_dir)) if has_animation else None
            })

        except Exception as e:
            print(f"❌ Error: {str(e)}")
            continue

print(f"\n\n{'='*80}")
print("VISUALIZATION SUMMARY")
print(f"{'='*80}")

# Group by environment
for env in environments:
    print(f"\n{env.name.upper()}:")
    env_vizs = [v for v in viz_summary if v['environment'] == env.name]
    for v in env_vizs:
        status = "✅" if v['coverage'] > 0.5 else "⚠️" if v['coverage'] > 0.1 else "❌"
        anim_mark = " (+anim)" if v['anim_file'] else ""
        print(f"  {status} {v['algorithm']:<20} Coverage: {v['coverage']:>5.1%}  Nodes: {v['nodes']:>3}{anim_mark}")

print(f"\n{'='*80}")
print(f"All visualizations saved to: {viz_dir}")
print(f"Total files: {len(viz_summary)}")
print(f"{'='*80}")

# Generate HTML index for easy browsing
html_content = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>SSTG Explorer - Algorithm Visualizations</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        h1 {
            color: #333;
            border-bottom: 3px solid #4CAF50;
            padding-bottom: 10px;
        }
        h2 {
            color: #555;
            margin-top: 30px;
            background-color: #e0e0e0;
            padding: 10px;
            border-left: 5px solid #4CAF50;
        }
        .env-section {
            background-color: white;
            padding: 20px;
            margin: 20px 0;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .algorithm-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }
        .algorithm-card {
            border: 1px solid #ddd;
            border-radius: 8px;
            overflow: hidden;
            background-color: white;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .algorithm-card img {
            width: 100%;
            height: auto;
            display: block;
        }
        .algorithm-info {
            padding: 15px;
            background-color: #fafafa;
        }
        .algorithm-name {
            font-weight: bold;
            font-size: 16px;
            margin-bottom: 8px;
            color: #333;
        }
        .metrics {
            font-size: 14px;
            color: #666;
            line-height: 1.6;
        }
        .metric-badge {
            display: inline-block;
            padding: 3px 8px;
            margin: 2px;
            border-radius: 3px;
            font-size: 12px;
        }
        .coverage-high { background-color: #4CAF50; color: white; }
        .coverage-medium { background-color: #FF9800; color: white; }
        .coverage-low { background-color: #f44336; color: white; }
        .summary-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }
        .summary-table th, .summary-table td {
            padding: 8px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }
        .summary-table th {
            background-color: #4CAF50;
            color: white;
        }
        .summary-table tr:hover {
            background-color: #f5f5f5;
        }
    </style>
</head>
<body>
    <h1>🗺️ SSTG Explorer - Algorithm Comparison Visualizations</h1>
    <p><strong>Generated:</strong> """ + str(Path.cwd()) + """</p>
    <p><strong>Total Algorithms:</strong> """ + str(len(algorithms)) + """ | <strong>Environments:</strong> """ + str(len(environments)) + """</p>
"""

for env in environments:
    html_content += f'\n    <div class="env-section">\n'
    html_content += f'        <h2>📍 {env.name.upper().replace("_", " ")}</h2>\n'
    html_content += '        <div class="algorithm-grid">\n'

    env_vizs = [v for v in viz_summary if v['environment'] == env.name]
    for v in env_vizs:
        coverage_class = 'coverage-high' if v['coverage'] > 0.8 else 'coverage-medium' if v['coverage'] > 0.5 else 'coverage-low'

        html_content += '            <div class="algorithm-card">\n'

        # Show animation if available, otherwise static image
        if v['anim_file']:
            html_content += f'                <img src="{v["anim_file"]}" alt="{v["algorithm"]} animation">\n'
            html_content += '                <div class="algorithm-info">\n'
            html_content += f'                    <div class="algorithm-name">{v["algorithm"]} 🎬</div>\n'
            html_content += f'                    <div style="font-size:12px; color:#666; margin-bottom:8px;">Static: <a href="{v["static_file"]}" target="_blank">View</a></div>\n'
        else:
            html_content += f'                <img src="{v["static_file"]}" alt="{v["algorithm"]}">\n'
            html_content += '                <div class="algorithm-info">\n'
            html_content += f'                    <div class="algorithm-name">{v["algorithm"]}</div>\n'

        html_content += '                    <div class="metrics">\n'
        html_content += f'                        <span class="metric-badge {coverage_class}">Coverage: {v["coverage"]:.1%}</span>\n'
        html_content += f'                        <span class="metric-badge">Distance: {v["distance"]:.1f}m</span>\n'
        html_content += f'                        <span class="metric-badge">Nodes: {v["nodes"]}</span>\n'
        html_content += '                    </div>\n'
        html_content += '                </div>\n'
        html_content += '            </div>\n'

    html_content += '        </div>\n'
    html_content += '    </div>\n'

# Add summary table
html_content += '\n    <div class="env-section">\n'
html_content += '        <h2>📊 Summary Table</h2>\n'
html_content += '        <table class="summary-table">\n'
html_content += '            <tr><th>Environment</th><th>Algorithm</th><th>Coverage</th><th>Distance (m)</th><th>Nodes</th></tr>\n'

for v in viz_summary:
    html_content += f'            <tr><td>{v["environment"]}</td><td>{v["algorithm"]}</td>'
    html_content += f'<td>{v["coverage"]:.1%}</td><td>{v["distance"]:.1f}</td><td>{v["nodes"]}</td></tr>\n'

html_content += '        </table>\n'
html_content += '    </div>\n'

html_content += """
</body>
</html>
"""

# Save HTML index
index_path = viz_dir / 'index.html'
with open(index_path, 'w') as f:
    f.write(html_content)

print(f"\n📄 HTML index created: {index_path}")
print(f"   Open in browser to view all visualizations")
print()
