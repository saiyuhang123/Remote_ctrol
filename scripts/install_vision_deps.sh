#!/usr/bin/env bash
set -euo pipefail
VISION_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m pip install --user -r "$VISION_PROJECT_ROOT/ros2_ws/src/detector_node/requirements-cpu.txt"
python3 "$VISION_PROJECT_ROOT/scripts/verify_vision_models.py"
