#!/bin/bash
# Monitor improved benchmark progress

LOG_FILE="outputs/benchmarks/benchmark_improved_run.log"

echo "=================================="
echo "  SSTG Benchmark Progress Monitor"
echo "  (Improved Version)"
echo "=================================="
echo ""

if [ ! -f "$LOG_FILE" ]; then
    echo "❌ Log file not found: $LOG_FILE"
    echo "Benchmark may not have started yet."
    exit 1
fi

# Count completed experiments
completed=$(grep -c "\[.*%\]" "$LOG_FILE" 2>/dev/null || echo "0")
total=175
progress=$(awk "BEGIN {printf \"%.1f\", ($completed/$total)*100}")

echo "📊 Progress: $completed / $total experiments ($progress%)"
echo ""

# Show progress bar
bar_length=50
filled=$(awk "BEGIN {printf \"%.0f\", ($completed/$total)*$bar_length}")
bar=$(printf "%-${bar_length}s" "$(printf '#%.0s' $(seq 1 $filled))")
echo "[$bar] $progress%"
echo ""

# Estimate time remaining
if [ $completed -gt 0 ]; then
    # Get start time
    start_line=$(head -1 "$LOG_FILE")
    # Assume ~1 minute per experiment for SSTG variants
    remaining=$((total - completed))
    est_minutes=$(awk "BEGIN {printf \"%.0f\", $remaining * 0.5}")
    echo "⏱️  Estimated time remaining: ~${est_minutes} minutes"
    echo ""
fi

# Show recent activity
echo "Recent experiments:"
echo "---"
tail -20 "$LOG_FILE" | grep -E "(Algorithm:|Run [0-9]+/|Coverage:|COMPLETE)" | tail -10
echo "---"
echo ""

# Check if complete
if grep -q "Benchmark complete!" "$LOG_FILE" 2>/dev/null; then
    echo "✅ BENCHMARK COMPLETE!"
    echo ""
    echo "Results and analysis saved to:"
    echo "  - JSON: outputs/benchmarks/results/benchmark_*.json"
    echo "  - Analysis: outputs/benchmarks/analysis/"
    exit 0
fi

echo "⏰ $(date '+%Y-%m-%d %H:%M:%S')"
echo ""
echo "💡 Run this script again to check progress"
echo "   or use: watch -n 30 ./scripts/monitor_benchmark.sh"
