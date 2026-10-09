#!/bin/bash
# M1 一键启动：MediaMTX + 真实摄像头 + Web 服务 + 遥控网关
# 用法：bash scripts/start_m1.sh    （Ctrl-C 全部停止）
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export ROBOT_PROJECT_ROOT="$ROOT"
export ROBOT_CONFIG="$ROOT/config/config.yaml"
trap 'kill $(jobs -p) 2>/dev/null || true' EXIT

"$ROOT/bin/mediamtx" "$ROOT/deploy/mediamtx.yml" &
sleep 1   # 等流媒体服务就绪再推流

# 摄像头不可用时报告错误。
"$ROOT/scripts/push_camera.sh" &
PUSH_PID=$!
sleep 3
if ! kill -0 $PUSH_PID 2>/dev/null; then
  echo "真实摄像头推流失败，请检查设备和采集格式。" >&2
  exit 1
fi
python3 -m http.server 8888 --directory "$ROOT/web" &
ros2 run robot_gateway gateway --ros-args -p use_sim_time:=false &

echo "=========================================================="
echo " M1 已启动。电脑浏览器访问： http://<本机IP>:8888"
echo "=========================================================="
wait
