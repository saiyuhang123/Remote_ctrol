#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
source "$PROJECT_ROOT/config/ros_domain.sh"
exec ros2 "$@"
