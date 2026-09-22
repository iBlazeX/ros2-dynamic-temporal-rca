#!/usr/bin/env bash
# Usage: scripts/sim.sh start|stop|status [logfile] [launch args...]
# Starts/stops the simulation + diagnostic monitor (rca_sim sim_system.launch.py) in the background.
WS=$(cd "$(dirname "$0")/.." && pwd)
LOG=${2:-$WS/runs/simulation/sim_system.log}
case "$1" in
  start)
    mkdir -p "$(dirname "$LOG")"
    cd "$WS" && source /opt/ros/jazzy/setup.bash && source install/setup.bash
    if [ $# -ge 2 ]; then shift 2; else shift $#; fi
    setsid nohup ros2 launch rca_sim sim_system.launch.py "$@" > "$LOG" 2>&1 < /dev/null &
    echo "started (log: $LOG)"; sleep 1 ;;
  stop)
    pkill -f "launch rca_sim " 2>/dev/null
    sleep 2
    pkill -9 -f "rca_sim/lib/rca_sim|diagnostic_monitor/lib/diagnostic_monitor/monitor_node" 2>/dev/null
    sleep 1; echo "stopped" ;;
  status)
    pgrep -af "rca_sim/lib/rca_sim|diagnostic_monitor/lib" | sed "s#.*/lib/[^/]*/##" | cut -d" " -f1 ;;
  *) echo "usage: $0 start|stop|status [logfile] [launch args...]"; exit 1 ;;
esac
