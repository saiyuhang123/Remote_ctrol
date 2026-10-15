#!/usr/bin/env bash
set -euo pipefail
DESKTOP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="${ROBOT_PROJECT_ROOT:-$(cd -- "$DESKTOP_ROOT/.." && pwd)}"
BUILD_DIR="${DESKTOP_BUILD_DIR:-$DESKTOP_ROOT/build}"
# ROS setup scripts can reference unset variables.
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi
set -u
cmake -S "$DESKTOP_ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
cmake --build "$BUILD_DIR" --parallel "${DESKTOP_BUILD_JOBS:-2}"
ctest --test-dir "$BUILD_DIR" --output-on-failure
