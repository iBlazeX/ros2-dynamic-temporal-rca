#!/usr/bin/env bash
# Usage: scripts/tui_demo.sh [rca_sim|gazebo] [scenario,scenario,...]
#
# Runs the professor demo scenarios against an ALREADY RUNNING backend while a
# single rca_tui instance stays up the whole time, capturing the terminal every
# 1.5 s. Proves the TUI updates live, without restarting, for either simulator.
# Frames and a state transition summary go to runs/<backend>/tui_demo/.
set +u
BACKEND=${1:-rca_sim}
WS=$(cd "$(dirname "$0")/.." && pwd)
cd "$WS" && source /opt/ros/jazzy/setup.bash && source install/setup.bash

case "$BACKEND" in
  rca_sim) PKG=rca_sim;  OUT=$WS/runs/simulation/tui_demo
           DEFAULT="lidar_noise,perception_crash,camera_dropout_gated_perception" ;;
  gazebo)  PKG=rca_gazebo; OUT=$WS/runs/gazebo/tui_demo
           DEFAULT="lidar_noise,perception_crash,camera_dropout" ;;
  *) echo "usage: $0 [rca_sim|gazebo] [scenarios]"; exit 1 ;;
esac
SCENARIOS=${2:-$DEFAULT}

mkdir -p "$OUT"; rm -f "$OUT"/frame_*.txt
tmux kill-session -t demo 2>/dev/null
tmux new-session -d -s demo -x 170 -y 46 \
  "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && ros2 run diagnostic_tui rca_tui"
sleep 5
echo "TUI started (backend=$BACKEND, scenarios=$SCENARIOS)"

setsid nohup ros2 run "$PKG" scenario_runner --repetitions 1 --warmup 5 --soak 0 \
  --scenarios "$SCENARIOS" --out "$OUT/results.json" > "$OUT/runner.log" 2>&1 < /dev/null &
RUNNER=$!
i=0
while kill -0 $RUNNER 2>/dev/null; do
  tmux capture-pane -t demo -p > "$OUT/frame_$(printf %03d $i).txt"
  i=$((i+1)); sleep 1.5
done
tmux capture-pane -t demo -p > "$OUT/frame_final.txt"
tmux send-keys -t demo q
echo "frames captured: $i"
python3 "$WS/scripts/tui_frames.py" "$OUT" | tee "$OUT/transitions.txt"
