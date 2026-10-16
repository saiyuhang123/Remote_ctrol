#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake qtbase5-dev qtbase5-private-dev \
  libqt5svg5-dev libxcb1-dev xvfb xauth
echo "桌面依赖已安装。ROS2 Humble、RViz2 和项目消息包沿用现有工作空间。"
