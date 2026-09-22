#!/usr/bin/env bash
# Usage: scripts/gazebo_sim.sh start|stop|status|topics [logfile] [launch args...]
#
# Starts/stops the Gazebo Harmonic backend together with the (unchanged)
# diagnostic monitor: rca_gazebo gazebo_system.launch.py.
#   start   launch gz sim + ros_gz bridge + sensor drivers + pipeline + monitor
#   stop    terminate the launch, the gz server and any leftover node processes
#   status  list the running backend processes
#   topics  list the application ROS topics the monitor can see
WS=$(cd "$(dirname "$0")/.." && pwd)
LOG=${2:-$WS/runs/gazebo/gazebo_system.log}
export LIBGL_ALWAYS_SOFTWARE=1

case "$1" in
  start)
    mkdir -p "$(dirname "$LOG")"
    cd "$WS" && source /opt/ros/jazzy/setup.bash && source install/setup.bash
    if [ $# -ge 2 ]; then shift 2; else shift $#; fi
    setsid nohup ros2 launch rca_gazebo gazebo_system.launch.py "$@" > "$LOG" 2>&1 < /dev/null &
    echo "started (log: $LOG)"; sleep 1 ;;
  stop)
    pkill -f "launch rca_gazebo " 2>/dev/null
    sleep 2
    pkill -9 -f "rca_gazebo/lib/rca_gazebo|rca_sim/lib/rca_sim|diagnostic_monitor/lib/diagnostic_monitor/monitor_node" 2>/dev/null
    pkill -9 -f "ros_gz_bridge/parameter_bridge" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "gz-sim-server" 2>/dev/null
    sleep 1; echo "stopped" ;;
  status)
    pgrep -af "rca_gazebo/lib|rca_sim/lib|diagnostic_monitor/lib|parameter_bridge|gz-sim-server" \
      | sed "s#.*/lib/[^/]*/##" | cut -d" " -f1-2 ;;
  topics)
    cd "$WS" && source /opt/ros/jazzy/setup.bash && source install/setup.bash
    ros2 topic list | grep -v -E "^/gz/|^/rca/|^/parameter_events|^/rosout" ;;
  *) echo "usage: $0 start|stop|status|topics [logfile] [launch args...]"; exit 1 ;;
esac
