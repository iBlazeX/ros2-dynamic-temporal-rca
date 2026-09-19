#!/usr/bin/env bash
# Usage: scripts/system.sh start|stop|status [logfile]
# Starts/stops the full RCA system (pipeline + diagnostic monitor) in the background.
WS=$(cd "$(dirname "$0")/.." && pwd)
LOG=${2:-$WS/runs/system.log}
case "$1" in
  start)
    mkdir -p "$WS/runs"
    cd "$WS" && source /opt/ros/jazzy/setup.bash && source install/setup.bash
    setsid nohup ros2 launch rca_test_system system.launch.py > "$LOG" 2>&1 < /dev/null &
    echo "started (log: $LOG)"; sleep 1 ;;
  stop)
    pkill -f "ros2 launch rca_test_system" 2>/dev/null
    sleep 2
    pkill -9 -f "rca_test_system/lib/rca_test_system|diagnostic_monitor/lib/diagnostic_monitor/monitor_node" 2>/dev/null
    sleep 1; echo "stopped" ;;
  status)
    pgrep -af "rca_test_system/lib|diagnostic_monitor/lib" | sed "s#.*/lib/[^/]*/##" | cut -d" " -f1 ;;
  *) echo "usage: $0 start|stop|status [logfile]"; exit 1 ;;
esac
