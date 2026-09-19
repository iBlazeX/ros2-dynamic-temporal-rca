#!/usr/bin/env bash
# Runs the automated benchmark against an already-running system (scripts/system.sh start).
# Usage: ./run_benchmark.sh [extra experiment_controller args, e.g. --include-crash]
source /opt/ros/jazzy/setup.bash
source /home/lynx/ros2_rca_ws/install/setup.bash
cd /home/lynx/ros2_rca_ws
ros2 run rca_test_system experiment_controller --warmup 15 --out runs/results_$(date +%Y%m%d_%H%M%S).json "$@"
