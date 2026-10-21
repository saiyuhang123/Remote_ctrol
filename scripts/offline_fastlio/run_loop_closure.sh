#!/usr/bin/env bash
# Run loop-closure post-processing over a recorded keyframe set.
set -euo pipefail
ROOT="${ROBOT_PROJECT_ROOT:-/home/jkbot/Remote_ctrol}"
KEYFRAMES="${1:-$ROOT/offline_outputs/fastlio_bag_full/keyframes}"
shift || true
source /opt/ros/humble/setup.bash
exec python3 "$ROOT/scripts/offline_fastlio/loop_closure_pcd.py" \
  --keyframes "$KEYFRAMES" "$@"
