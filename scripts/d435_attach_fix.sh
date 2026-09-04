#!/bin/bash
# D435 attach 后的 WSL 侧一键修复：加载驱动 + 放开设备权限
# 本 WSL 不处理 udev 规则，所以每次 attach 后都要跑一遍：
#   bash scripts/d435_attach_fix.sh
set -e
echo "加载 uvcvideo 驱动…"
sudo modprobe uvcvideo
sleep 1
if [ ! -e /dev/video0 ]; then
  echo "错误：/dev/video* 未出现，请先在 Windows 执行 usbipd attach --wsl --busid 1-19"
  exit 1
fi
sudo chmod 666 /dev/video*
echo "D435 就绪：$(ls /dev/video* | tr '\n' ' ')"
echo "推流: bash scripts/push_d435_wsl.sh"
