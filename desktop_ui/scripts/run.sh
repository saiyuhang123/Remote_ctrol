#!/usr/bin/env bash
set -euo pipefail
DESKTOP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="${ROBOT_PROJECT_ROOT:-$(cd -- "$DESKTOP_ROOT/.." && pwd)}"
BUILD_DIR="${DESKTOP_BUILD_DIR:-$DESKTOP_ROOT/build}"
if [[ ! -x "$BUILD_DIR/robot_desktop" ]]; then
  echo "请先运行：$DESKTOP_ROOT/scripts/build.sh" >&2
  exit 1
fi
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then source "$PROJECT_ROOT/ros2_ws/install/setup.bash"; fi
set -u
export ROBOT_PROJECT_ROOT="$PROJECT_ROOT"
source "$PROJECT_ROOT/config/ros_domain.sh"
exec "$BUILD_DIR/robot_desktop" --resource-dir "$DESKTOP_ROOT" --project-root "$PROJECT_ROOT" "$@"
