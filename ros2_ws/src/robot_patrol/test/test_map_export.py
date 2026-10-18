import hashlib
import json
from pathlib import Path

from PIL import Image
import pytest
import yaml

from robot_patrol.map_export import export_web_map


def _make_map(tmp_path: Path, yaw: float):
    image = tmp_path / "map.pgm"
    Image.new("L", (7, 5), 205).save(image)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(yaml.safe_dump({
        "image": "map.pgm", "resolution": 0.1,
        "origin": [-2.5, 3.25, yaw], "negate": 0,
        "occupied_thresh": 0.65, "free_thresh": 0.25,
    }))
    return yaml_path


@pytest.mark.parametrize("yaw", [0.0, 30.0 * 3.141592653589793 / 180.0,
                                 90.0 * 3.141592653589793 / 180.0])
def test_web_map_export_preserves_origin_yaw_and_writes_atomic_pair(tmp_path, yaw):
    yaml_path = _make_map(tmp_path, yaw)
    output = tmp_path / "web"
    meta = export_web_map(yaml_path, output, map_id="map_test", revision=4,
                          backend="fastlio")
    assert meta["origin"] == [-2.5, 3.25, yaw]
    assert meta["map_id"] == "map_test"
    assert meta["revision"] == 4
    assert meta["backend"] == "fastlio"
    assert meta["image"] == "active.png"
    assert (output / "active.png").is_file()
    assert json.loads((output / "meta.json").read_text()) == meta
    assert meta["sha256"] == hashlib.sha256((output / "active.png").read_bytes()).hexdigest()
    assert meta["width"] == 7 and meta["height"] == 5
