#!/usr/bin/env bash
set -euo pipefail
VISION_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m pip install --user -r "$VISION_PROJECT_ROOT/ros2_ws/src/detector_node/requirements-cpu.txt"
# Download plate weights during installation, so offline duty needs no network.
python3 - <<'PY'
import hyperlpr3
model = hyperlpr3.LicensePlateCatcher()
print('CPU detection dependencies and plate models ready; no camera connection was made.')
PY
