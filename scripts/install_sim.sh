#!/usr/bin/env bash
# 仿真环境一键安装（WSL2 / x64 Ubuntu 22.04 + ROS2 Humble）
# 对应《WSL2仿真环境交接文档.md》2.4 节
set -e

sudo apt update
sudo apt install -y \
  ffmpeg \
  v4l2-utils \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-twist-mux \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  liburdfdom-tools

echo "==> 安装完成。验证："
gazebo --version
