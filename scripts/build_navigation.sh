#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# ROS setup scripts can read unset variables.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$project_dir/ros2_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
export MAKEFLAGS=${MAKEFLAGS:--j2}
colcon build --symlink-install --executor sequential --packages-select \
  rslidar_msg imu_msg yunle_msgs rslidar_sdk wit_ros2_imu \
  fast_lio_localization pcd2pgm msf_localizer fixed_route_navigation chassis_nav2 chassis_driver \
  chassis_description robot_common audio_player robot_gateway \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble \
  -DBUILD_TESTING=OFF -DPOINT_TYPE=XYZIRT -DDISABLE_PCAP_PARSE=ON -DWITH_LIVOX=OFF \
  "$@"
