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
  python3-open3d python3-numpy python3-serial python3-yaml python3-pil python3-websockets
