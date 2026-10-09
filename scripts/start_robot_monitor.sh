#!/usr/bin/env bash
# 实机监控入口：业务后端 + 遥控网关；视频、识别和音频按需开启。
# 底盘、雷达、TF、定位/导航由 chassis_nav2 在另一终端启动。
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENABLE_VIDEO="${ENABLE_VIDEO:-0}"
ENABLE_DETECTOR="${ENABLE_DETECTOR:-0}"
ENABLE_CAPTURE="${ENABLE_CAPTURE:-$ENABLE_VIDEO}"
ENABLE_AUDIO="${ENABLE_AUDIO:-0}"
VIDEO_SOURCE="${VIDEO_SOURCE:-rtsp://127.0.0.1:8554/cam}"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export ROBOT_PROJECT_ROOT="$ROOT"
export ROBOT_CONFIG="$ROOT/config/config.yaml"

if [ "$ENABLE_VIDEO" = 1 ] && [ ! -x "$ROOT/bin/mediamtx" ]; then
  echo "缺少 $ROOT/bin/mediamtx，请先准备本机架构的 MediaMTX 可执行文件。" >&2
  exit 1
fi

pids=()
cleanup() {
  local pid
  for pid in "${pids[@]}"; do
    kill -- "-$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() { setsid "$@" & pids+=("$!"); }

start python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8888 --app-dir "$ROOT"
start ros2 run robot_gateway gateway --ros-args -p use_sim_time:=false

if [ "$ENABLE_VIDEO" = 1 ]; then
  start "$ROOT/bin/mediamtx" "$ROOT/deploy/mediamtx.yml"
  sleep 1
  if [ -n "${CAMERA_RTSP:-}" ]; then
    # 海康等网络摄像头：转发实际 RTSP 视频到本机 MediaMTX。
    start ffmpeg -hide_banner -loglevel warning -rtsp_transport tcp -i "$CAMERA_RTSP" \
      -map 0:v:0 -c:v copy -an -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/cam
  else
    start bash "$ROOT/scripts/push_camera.sh"
  fi
fi
if [ "$ENABLE_CAPTURE" = 1 ]; then
  start ros2 run capture_server capture_server --ros-args \
    -p use_sim_time:=false -p "video_source:=$VIDEO_SOURCE"
fi
if [ "$ENABLE_DETECTOR" = 1 ]; then
  start ros2 run detector_node detector --ros-args \
    -p use_sim_time:=false -p "video_source:=$VIDEO_SOURCE"
fi
if [ "$ENABLE_AUDIO" = 1 ]; then
  start ros2 run audio_player tts_node --ros-args -p use_sim_time:=false
fi

echo "实机监控入口：http://<本机IP>:8888/monitor.html"
echo "请另行启动实际底盘、传感器和定位；地图与电池未接入时页面显示未接入状态。"
# 任一组件退出时结束本次启动，避免组件失败后仍显示整个服务已正常运行。
wait -n "${pids[@]}"
