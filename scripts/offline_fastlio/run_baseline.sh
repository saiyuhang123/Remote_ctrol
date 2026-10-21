#!/usr/bin/env bash
# Replay the recording through the CURRENT FAST-LIO2 frontend and save a raw PCD.
# This is the baseline stage, not the loop-closure stage.
set -euo pipefail

ROOT="${ROBOT_PROJECT_ROOT:-/home/jkbot/Remote_ctrol}"
BAG="${1:-$ROOT/bags/fastlio_z_20261021_160456}"
OUT_DIR="${2:-$ROOT/maps/offline/fastlio_bag_$(date +%Y%m%d_%H%M%S)}"
RATE="${RATE:-1.0}"
START_OFFSET="${START_OFFSET:-0.0}"
PLAY_DURATION="${PLAY_DURATION:-0}"
PARAMS="$ROOT/scripts/offline_fastlio/offline_velodyne.yaml"

# ROS setup scripts are not nounset-safe.
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

[[ -d "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 2; }
[[ -f "$PARAMS" ]] || { echo "params not found: $PARAMS" >&2; exit 2; }
mkdir -p "$OUT_DIR"
OUT_PCD="$OUT_DIR/map_raw.pcd"
MAP_LOG="$OUT_DIR/fastlio_mapping.log"
BAG_LOG="$OUT_DIR/bag_play.log"

map_pid=""
cleanup() {
  if [[ -n "$map_pid" ]] && kill -0 "$map_pid" 2>/dev/null; then
    kill -INT "$map_pid" 2>/dev/null || true
    wait "$map_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "bag : $BAG"
echo "out : $OUT_DIR"
echo "rate: $RATE"
echo "start_offset: $START_OFFSET"
echo "play_duration: ${PLAY_DURATION:-0} (0=full bag)"
echo "params: $PARAMS"

ros2 run fast_lio_localization fastlio_mapping --ros-args \
  --params-file "$PARAMS" \
  -p map_file_path:="$OUT_PCD" \
  >"$MAP_LOG" 2>&1 &
map_pid=$!

# Wait until the map save service exists before feeding the bag.
for _ in $(seq 1 50); do
  if ros2 service list 2>/dev/null | grep -qx '/map_save'; then break; fi
  if ! kill -0 "$map_pid" 2>/dev/null; then
    echo "fastlio_mapping exited during startup" >&2
    tail -80 "$MAP_LOG" >&2
    exit 3
  fi
  sleep 0.2
done
ros2 service list 2>/dev/null | grep -qx '/map_save' || {
  echo "/map_save did not appear" >&2; tail -80 "$MAP_LOG" >&2; exit 4;
}

play_args=(--rate "$RATE" --disable-keyboard-controls --read-ahead-queue-size 1000 --start-offset "$START_OFFSET")
echo "playing bag ..."
if [[ "$PLAY_DURATION" != "0" ]]; then
  timeout "$PLAY_DURATION" ros2 bag play "$BAG" "${play_args[@]}" >"$BAG_LOG" 2>&1 || {
    rc=$?; [[ "$rc" == "124" ]] || exit "$rc"
  }
else
  ros2 bag play "$BAG" "${play_args[@]}" >"$BAG_LOG" 2>&1
fi
# Give the mapping timer a moment to consume the final frames.
sleep 3

echo "saving raw PCD ..."
timeout 180 ros2 service call /map_save std_srvs/srv/Trigger '{}' | tee "$OUT_DIR/map_save.log"

for _ in $(seq 1 120); do
  [[ -s "$OUT_PCD" ]] && break
  sleep 0.5
done
[[ -s "$OUT_PCD" ]] || { echo "PCD was not created: $OUT_PCD" >&2; tail -120 "$MAP_LOG" >&2; exit 5; }

echo "baseline PCD: $OUT_PCD"
stat -c 'size=%s bytes mtime=%y' "$OUT_PCD"
echo "logs: $MAP_LOG"
