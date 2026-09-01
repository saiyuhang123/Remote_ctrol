#!/bin/bash
# M1 一键启动：MediaMTX + 测试视频源 + Web 服务 + 遥控网关
# 用法：bash scripts/start_m1.sh    （Ctrl-C 全部停止）
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

"$ROOT/bin/mediamtx" "$ROOT/deploy/mediamtx.yml" &
sleep 1   # 等流媒体服务就绪再推流

# 优先用真实摄像头，探测不到则回退测试源
"$ROOT/scripts/push_camera.sh" &
PUSH_PID=$!
sleep 3
if ! kill -0 $PUSH_PID 2>/dev/null; then
  echo "未发现摄像头，回退到测试视频源"
  "$ROOT/scripts/push_test_video.sh" &
fi
python3 -m http.server 8888 --directory "$ROOT/web" &
ros2 run robot_gateway gateway &

trap 'kill $(jobs -p) 2>/dev/null || true' EXIT
echo "=========================================================="
echo " M1 已启动。电脑浏览器访问： http://<本机IP>:8888"
echo "=========================================================="
wait
