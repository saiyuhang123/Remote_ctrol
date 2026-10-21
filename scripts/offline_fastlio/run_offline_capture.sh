#!/usr/bin/env bash
# Full offline capture: replay bag -> FAST-LIO -> keyframes + raw PCD.
# Designed for a bag containing /velodyne_points and /imu/data only.
set -euo pipefail

ROOT="${ROBOT_PROJECT_ROOT:-/home/jkbot/Remote_ctrol}"
BAG="${1:-$ROOT/bags/fastlio_z_20261021_160456}"
OUT_DIR="${2:-$ROOT/offline_outputs/fastlio_$(date +%Y%m%d_%H%M%S)}"
RATE="${RATE:-1.0}"
START_OFFSET="${START_OFFSET:-0.0}"
PLAY_DURATION="${PLAY_DURATION:-0}"   # 0 => full bag
KEYFRAME_DISTANCE="${KEYFRAME_DISTANCE:-0.8}"
KEYFRAME_ANGLE="${KEYFRAME_ANGLE:-12.0}"
KEYFRAME_MAX_GAP="${KEYFRAME_MAX_GAP:-2.0}"
PARAMS="$ROOT/scripts/offline_fastlio/offline_velodyne.yaml"

set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

[[ -d "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 2; }
[[ -f "$PARAMS" ]] || { echo "params not found: $PARAMS" >&2; exit 2; }
mkdir -p "$OUT_DIR"
RAW_PCD="$OUT_DIR/map_raw.pcd"
MAP_LOG="$OUT_DIR/fastlio_mapping.log"
REC_LOG="$OUT_DIR/keyframe_recorder.log"
BAG_LOG="$OUT_DIR/bag_play.log"

map_pid=""; rec_pid=""; bag_pid=""
cleanup() {
  # Stop readers/recorders before the mapping process.
  if [[ -n "${bag_pid:-}" ]] && kill -0 "$bag_pid" 2>/dev/null; then kill -INT "$bag_pid" 2>/dev/null || true; fi
  if [[ -n "${rec_pid:-}" ]] && kill -0 "$rec_pid" 2>/dev/null; then kill -INT "$rec_pid" 2>/dev/null || true; fi
  sleep 1
  if [[ -n "${map_pid:-}" ]] && kill -0 "$map_pid" 2>/dev/null; then kill -INT -"$map_pid" 2>/dev/null || kill -INT "$map_pid" 2>/dev/null || true; fi
  sleep 1
  for p in "${bag_pid:-}" "${rec_pid:-}" "${map_pid:-}"; do
    [[ -n "$p" ]] && kill -KILL "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "bag : $BAG"
echo "out : $OUT_DIR"
echo "rate: $RATE start=$START_OFFSET duration=$PLAY_DURATION"
echo "keyframe trigger: ${KEYFRAME_DISTANCE}m / ${KEYFRAME_ANGLE}deg / ${KEYFRAME_MAX_GAP}s"

# Use separate process groups so each test session can be cleaned up.
setsid ros2 run fast_lio_localization fastlio_mapping --ros-args \
  --params-file "$PARAMS" -p map_file_path:="$RAW_PCD" >"$MAP_LOG" 2>&1 &
map_pid=$!
setsid python3 "$ROOT/scripts/offline_fastlio/record_keyframes.py" \
  --output-dir "$OUT_DIR/keyframes" \
  --distance "$KEYFRAME_DISTANCE" --angle-deg "$KEYFRAME_ANGLE" \
  --max-gap "$KEYFRAME_MAX_GAP" >"$REC_LOG" 2>&1 &
rec_pid=$!

for _ in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -qx '/map_save'; then break; fi
  if ! kill -0 "$map_pid" 2>/dev/null; then
    echo "fastlio_mapping exited during startup" >&2
    tail -100 "$MAP_LOG" >&2
    exit 3
  fi
  sleep 0.25
done
ros2 service list 2>/dev/null | grep -qx '/map_save' || {
  echo "/map_save did not appear" >&2; tail -100 "$MAP_LOG" >&2; exit 4;
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

# Let final scans be processed, then stop the recorder cleanly before saving.
sleep 3
kill -INT -"$rec_pid" 2>/dev/null || kill -INT "$rec_pid" 2>/dev/null || true
sleep 1
kill -KILL -"$rec_pid" 2>/dev/null || true
rec_pid=""

echo "saving raw PCD ..."
timeout 180 ros2 service call /map_save std_srvs/srv/Trigger '{}' | tee "$OUT_DIR/map_save.log"
for _ in $(seq 1 120); do [[ -s "$RAW_PCD" ]] && break; sleep 0.5; done
[[ -s "$RAW_PCD" ]] || { echo "PCD was not created: $RAW_PCD" >&2; tail -120 "$MAP_LOG" >&2; exit 5; }

kill -INT -"$map_pid" 2>/dev/null || kill -INT "$map_pid" 2>/dev/null || true
sleep 1
kill -KILL -"$map_pid" 2>/dev/null || kill -KILL "$map_pid" 2>/dev/null || true
map_pid=""

echo "raw PCD: $RAW_PCD"
stat -c 'size=%s bytes mtime=%y' "$RAW_PCD"
echo "keyframes: $OUT_DIR/keyframes/keyframes.jsonl"
echo "manifest count: $(wc -l < "$OUT_DIR/keyframes/keyframes.jsonl")"
