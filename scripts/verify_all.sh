#!/usr/bin/env bash
# Full clean verification. Usage: scripts/verify_all.sh
# Builds from scratch-ish (colcon), runs every test suite, validates the saved
# artifacts and prints the git status. Does NOT commit anything.
set +u
WS=$(cd "$(dirname "$0")/.." && pwd)
cd "$WS" && source /opt/ros/jazzy/setup.bash

echo "############ 1. environment"
lsb_release -ds; echo "ROS_DISTRO=$ROS_DISTRO"; gz sim --versions | head -1
g++ --version | head -1; cmake --version | head -1; python3 --version

echo "############ 2. colcon build"
colcon build --symlink-install 2>&1 | tail -3 || exit 1
source install/setup.bash

echo "############ 3. pytest (all Python packages)"
python3 -m pytest src/diagnostic_monitor/tests src/rca_test_system/tests \
                  src/rca_sim/tests src/rca_gazebo/tests -q 2>&1 | tail -3

echo "############ 4. TUI gtest"
./build/diagnostic_tui/test_dashboard_state 2>&1 | tail -3

echo "############ 5. colcon test"
colcon test 2>&1 | tail -3
colcon test-result 2>&1 | tail -2

echo "############ 6. launch files validate"
for f in src/rca_gazebo/launch/*.launch.py src/rca_sim/launch/*.launch.py \
         src/diagnostic_monitor/launch/*.launch.py src/rca_test_system/launch/*.launch.py; do
  python3 -c "
import importlib.util, sys
spec = importlib.util.spec_from_file_location('lf', '$f')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
m.generate_launch_description()
print('  OK $f')" || echo "  FAIL $f"
done

echo "############ 7. world SDF validates"
gz sdf -k src/rca_gazebo/worlds/rca_arena.sdf && echo "  OK rca_arena.sdf"

echo "############ 8. git status (nothing is committed by this script)"
git status --short | head -40
echo "--- diff --stat ---"
git diff --stat | tail -5
echo "--- diff --check (whitespace errors) ---"
git diff --check && echo "  clean"
