#!/usr/bin/env bash
# Usage: scripts/overhead.sh [rca_sim|gazebo]
#
# Clean, uncontaminated overhead measurement for one simulator backend.
#
# Method (see scripts/measure_overhead.py): CPU comes from /proc/<pid>/stat
# utime+stime deltas over the sampling window (NOT `ps %cpu`, which averages over
# the whole process lifetime), RSS is sampled every second from /proc/<pid>/statm.
# CPU is reported as raw process % (one busy core = 100 %), as core-equivalents
# and as a share of the machine's total capacity, so "85 %" is never ambiguous.
#
# Before every measurement the expected process count of every group is asserted
# with --expect; a stray duplicate aborts the run instead of silently inflating
# the numbers. All PIDs are recorded in the artifact.
#
# Stages, same simulator workload, same 30 s window:
#   1 backend only
#   2 backend + diagnostic monitor,          nominal, then during a LiDAR-noise fault
#   3 backend + diagnostic monitor + 1 TUI,  nominal, then during a LiDAR-noise fault
set +u
BACKEND=${1:-rca_sim}
WS=$(cd "$(dirname "$0")/.." && pwd)
cd "$WS" && source /opt/ros/jazzy/setup.bash && source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

case "$BACKEND" in
  rca_sim) SIM_LAUNCH="rca_sim sim.launch.py"; HELPER=scripts/sim.sh
           OUT=$WS/runs/simulation/overhead.json; NBACK=1; NPIPE=4; SETTLE=40 ;;
  gazebo)  SIM_LAUNCH="rca_gazebo gazebo_sim.launch.py"; HELPER=scripts/gazebo_sim.sh
           OUT=$WS/runs/gazebo/overhead.json;     NBACK=7; NPIPE=4; SETTLE=95 ;;
  *) echo "usage: $0 [rca_sim|gazebo]"; exit 1 ;;
esac
mkdir -p "$(dirname "$OUT")"; rm -f "$OUT"
LOGDIR=$(dirname "$OUT")

stop_everything() {
  tmux kill-server 2>/dev/null
  bash scripts/sim.sh stop        > /dev/null 2>&1
  bash scripts/gazebo_sim.sh stop > /dev/null 2>&1
  sleep 4
}

measure() {   # measure <label> <expect...>
  local label=$1; shift
  if ! python3 "$WS/scripts/measure_overhead.py" --out "$OUT" --duration 30 --label "$label" "$@"; then
    echo "ABORTING: process uniqueness check failed for $label"
    stop_everything
    exit 2
  fi
}

echo "=== 0. clean slate ==="
stop_everything
python3 "$WS/scripts/measure_overhead.py" --list

echo "=== 1. $BACKEND only (no monitor, no TUI) ==="
setsid nohup ros2 launch $SIM_LAUNCH > "$LOGDIR/overhead_backend_only.log" 2>&1 < /dev/null &
sleep 45
measure "${BACKEND}_only" --expect diagnostic_monitor=0 --expect tui=0 --expect pipeline=$NPIPE
stop_everything

echo "=== 2. $BACKEND + diagnostic monitor ==="
rm -f events.db events.db-wal events.db-shm
bash "$HELPER" start "$LOGDIR/overhead_backend_monitor.log" > /dev/null
sleep $SETTLE
measure "${BACKEND}_monitor_nominal" --expect diagnostic_monitor=1 --expect tui=0 --expect pipeline=$NPIPE
ros2 run rca_test_system fault_injector --target lidar_node --type noise --param 8 --duration 45 > /dev/null 2>&1
sleep 3
measure "${BACKEND}_monitor_fault" --expect diagnostic_monitor=1 --expect tui=0 --expect pipeline=$NPIPE
ros2 run rca_test_system fault_injector --target all --type clear > /dev/null 2>&1
sleep 10

echo "=== 3. $BACKEND + diagnostic monitor + exactly one TUI ==="
tmux new-session -d -s ov -x 170 -y 46 \
  "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && ros2 run diagnostic_tui rca_tui"
sleep 8
measure "${BACKEND}_monitor_tui_nominal" --expect diagnostic_monitor=1 --expect tui=1 --expect pipeline=$NPIPE
ros2 run rca_test_system fault_injector --target lidar_node --type noise --param 8 --duration 45 > /dev/null 2>&1
sleep 3
measure "${BACKEND}_monitor_tui_fault" --expect diagnostic_monitor=1 --expect tui=1 --expect pipeline=$NPIPE
ros2 run rca_test_system fault_injector --target all --type clear > /dev/null 2>&1
tmux send-keys -t ov q; sleep 2; tmux kill-session -t ov 2>/dev/null
echo "done -> $OUT"
python3 "$WS/scripts/show_overhead.py" "$OUT"
