import shutil
import subprocess
from pathlib import Path

import pytest


def test_web_map_projection_supports_origin_yaw():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js not installed")
    root = Path(__file__).resolve().parents[4]
    script = root / "tests" / "web" / "map_transform_test.js"
    result = subprocess.run([node, str(script)], text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stderr + result.stdout
    assert "0, 30, 90" in result.stdout
