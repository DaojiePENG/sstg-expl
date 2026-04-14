#!/bin/bash
# Monitor benchmark progress

LOG_FILE="outputs/benchmarks/benchmark_run.log"

while true; do
    clear
    echo "=================================="
    echo "  SSTG Benchmark Progress Monitor"
    echo "=================================="
    echo ""

    if [ ! -f "$LOG_FILE" ]; then
        echo "❌ Log file not found: $LOG_FILE"
        break
    fi

    # Count completed experiments
    completed=$(grep -c "\[.*%\]" "$LOG_FILE" 2>/dev/null || echo "0")
    echo "📊 Completed: $completed / 175 experiments"
    echo ""

    # Show last 20 lines
    echo "Recent output:"
    echo "---"
    tail -n 20 "$LOG_FILE"
    echo "---"

    # Check if complete
    if grep -q "Benchmark complete!" "$LOG_FILE" 2>/dev/null; then
        echo ""
        echo "✅ BENCHMARK COMPLETE!"
        break
    fi

    echo ""
    echo "⏰ $(date)"
    echo "Press Ctrl+C to exit monitor"

    sleep 30
done
