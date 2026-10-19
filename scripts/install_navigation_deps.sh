#!/usr/bin/env bash
set -euo pipefail
# Run on the real development/deployment computer; requires sudo.
sudo apt-get update
sudo apt-get install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-amcl \
  ros-humble-nav2-smac-planner ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-robot-localization ros-humble-pointcloud-to-laserscan \
  ros-humble-twist-mux ros-humble-tf-transformations ros-humble-sensor-msgs-py \
  ros-humble-pcl-conversions ros-humble-robot-state-publisher \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libpcap-dev \
  python3-open3d python3-numpy python3-serial python3-yaml python3-pil python3-websockets python3-pip

# The vision stack installs NumPy 1.26.4. Ubuntu's transforms3d 0.3.1
# imports np.float and crashes tf_transformations before ICP can start.
python3 -m pip install --user 'transforms3d==0.4.2'
python3 - <<'PY'
import transforms3d
print('transforms3d coordinate transform dependency imports successfully')
PY
