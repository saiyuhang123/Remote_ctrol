#!/usr/bin/env bash
# Isolated tests only: real DDS/ffplay, dummy soundcard, no vehicle commands.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
cd "$ROOT"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=219
export ROBOT_AUDIO_INTEGRATION=1
export ROBOT_PATROL_INTEGRATION=0
export SDL_AUDIODRIVER=dummy
python3 -m pytest ros2_ws/src/audio_player/test -q
# Existing gateway command tests require their own isolated domain.
export ROS_DOMAIN_ID=226
export ROBOT_GATEWAY_INTEGRATION=1
python3 -m pytest ros2_ws/src/robot_gateway/test/test_gateway_commands.py \
  ros2_ws/src/robot_gateway/test/test_intercom.py -q
python3 -m pytest ros2_ws/src/robot_patrol/test -q
PYTHONPATH="$ROOT/ros2_ws/src/chassis_nav2:${PYTHONPATH:-}" \
  python3 -m pytest ros2_ws/src/chassis_nav2/tests/test_navigation_integration.py \
  -k 'standard_navigation_starts_audio or bringup_has_one_tf_authority or nav2_uses_measured_footprint or slam_map_uses_amcl or cartographer_localization' -q
python3 -m pytest tests/test_startup_ros_environment.py -q
