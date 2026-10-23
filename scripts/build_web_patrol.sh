#!/usr/bin/env bash
# Build web inspection interfaces and device/business nodes without launching.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$project_dir/ros2_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
colcon build --symlink-install --executor sequential --packages-up-to \
  robot_patrol robot_gateway vision_events audio_player ocr_node capture_server \
  --event-handlers console_cohesion+ status- \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
# PTZ inspection leases are implemented by the existing chassis PTZ node.
set +u
source install/setup.bash
set -u
colcon build --symlink-install --packages-select chassis_nav2 \
  --event-handlers console_cohesion+ status- --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
