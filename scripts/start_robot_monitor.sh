#!/usr/bin/env bash
# 实机监控入口：网页后端、遥控网关、音频和网页视频；识别按需开启。
# 底盘、雷达、TF、定位/导航由 chassis_nav2 在另一终端启动。
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENABLE_VIDEO="${ENABLE_VIDEO:-auto}"
if [ "$ENABLE_VIDEO" = auto ]; then
  if [ -x "$ROOT/bin/mediamtx" ]; then
    ENABLE_VIDEO=1
  else
    ENABLE_VIDEO=0
    echo "未找到 MediaMTX，网页视频暂不启动；其他网页功能继续启动。" >&2
  fi
fi
ENABLE_DETECTOR="${ENABLE_DETECTOR:-0}"
ENABLE_AUDIO="${ENABLE_AUDIO:-1}"
ENABLE_CAPTURE="${ENABLE_CAPTURE:-$ENABLE_VIDEO}"
VIDEO_SOURCE="${VIDEO_SOURCE:-rtsp://127.0.0.1:8554/cam}"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export ROBOT_PROJECT_ROOT="$ROOT"
export ROBOT_CONFIG="$ROOT/config/config.yaml"
export ROBOT_PATROL_DB="${ROBOT_PATROL_DB:-$HOME/.local/share/robot_patrol/patrol.sqlite3}"
export ROBOT_WEB_MAP_DIR="${ROBOT_WEB_MAP_DIR:-$ROOT/web/maps}"
export ROBOT_AUDIO_DIR="${ROBOT_AUDIO_DIR:-$ROOT/data/audio}"
if [ -n "${ROBOT_TLS_CERT:-}" ]; then
  export ROBOT_BACKEND_URL="${ROBOT_BACKEND_URL:-http://127.0.0.1:8887}"
else
  export ROBOT_BACKEND_URL="${ROBOT_BACKEND_URL:-http://127.0.0.1:8888}"
fi

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

tls_args=()
if [ -n "${ROBOT_TLS_CERT:-}${ROBOT_TLS_KEY:-}" ]; then
  if [ -z "${ROBOT_TLS_CERT:-}" ] || [ -z "${ROBOT_TLS_KEY:-}" ]; then
    echo "ROBOT_TLS_CERT 和 ROBOT_TLS_KEY 必须同时设置" >&2
    exit 1
  fi
  tls_args=(--ssl-certfile "$ROBOT_TLS_CERT" --ssl-keyfile "$ROBOT_TLS_KEY")
  # Local recorders keep a loopback-only HTTP endpoint; no TLS verification
  # bypass is needed for uploads or retry queues.
  start python3 -m uvicorn backend.main:app --host 127.0.0.1 --port 8887 --app-dir "$ROOT"
  export MTX_WEBRTCENCRYPTION=true
  export MTX_WEBRTCSERVERKEY="$ROBOT_TLS_KEY"
  export MTX_WEBRTCSERVERCERT="$ROBOT_TLS_CERT"
fi
start python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8888 --app-dir "$ROOT" "${tls_args[@]}"
start ros2 run robot_gateway gateway --ros-args -p use_sim_time:=false

if [ "$ENABLE_VIDEO" = 1 ]; then
  start python3 "$ROOT/scripts/start_web_video.py"
fi
if [ "$ENABLE_CAPTURE" = 1 ]; then
  start ros2 run capture_server capture_server --ros-args \
    -p use_sim_time:=false -p "video_source:=$VIDEO_SOURCE"
fi
if [ "$ENABLE_DETECTOR" = 1 ]; then
  start ros2 run vision_events recorder --ros-args -p upload_enabled:=true \
    -p "backend_url:=$ROBOT_BACKEND_URL"
  start ros2 run detector_node detector --ros-args \
    -p use_sim_time:=false -p input_mode:=video -p "video_source:=$VIDEO_SOURCE"
fi
if [ "$ENABLE_AUDIO" = 1 ]; then
  start ros2 run audio_player tts_node --ros-args -p "audio_dir:=$ROBOT_AUDIO_DIR" \
    -p alert_on_person:=false
fi

if [ -n "${ROBOT_TLS_CERT:-}" ]; then
  echo "实机监控入口：https://<证书对应主机>:8888/console.html"
else
  echo "实机监控入口：http://<本机IP>:8888/console.html"
fi
echo "请另行启动实际底盘、传感器、定位和 robot_patrol；未配置网页地图或定位时页面会明确提示。"
# 任一组件退出时结束本次启动，避免组件失败后仍显示整个服务已正常运行。
wait -n "${pids[@]}"
