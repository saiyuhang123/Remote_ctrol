"""Export the active Nav2 occupancy image to the local web monitor.

The occupied/free raster itself is copied without changing row order.  The
world-to-image affine transform is described by Nav2's complete origin,
including origin yaw.  Both output files are replaced atomically.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any
import tempfile

import yaml


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def export_web_map(map_yaml: str | Path, output_dir: str | Path, *,
                   map_id: str = "", revision: int = 0,
                   backend: str = "") -> dict[str, Any]:
    try:
        from PIL import Image
    except ImportError as error:  # pragma: no cover - deployment dependency
        raise RuntimeError("地图网页导出需要 Pillow：pip install Pillow") from error

    map_yaml = Path(map_yaml).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    configured = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    if not isinstance(configured, dict):
        raise ValueError("地图 YAML 必须是对象")
    resolution = configured.get("resolution")
    origin = configured.get("origin")
    if (not isinstance(resolution, (int, float)) or isinstance(resolution, bool)
            or not math.isfinite(float(resolution)) or float(resolution) <= 0):
        raise ValueError("地图 resolution 必须是正的有限数值")
    if (not isinstance(origin, list) or len(origin) < 3
            or not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                       and math.isfinite(float(value)) for value in origin[:3])):
        raise ValueError("地图 origin 必须是包含三个有限数值的数组")

    image_name = configured.get("image")
    if not isinstance(image_name, str) or not image_name:
        raise ValueError("地图 YAML 缺少 image")
    source = Path(image_name).expanduser()
    if not source.is_absolute():
        source = map_yaml.parent / source
    source = source.resolve()
    if not source.is_file():
        raise ValueError(f"地图图片不存在：{source}")

    output_dir.mkdir(parents=True, exist_ok=True)
    with Image.open(source) as opened:
        image = opened.convert("L")
        width, height = image.size
        if width <= 0 or height <= 0:
            raise ValueError("地图图片尺寸无效")
        # PNG is the browser's canonical active image. Keep the source raster
        # row order unchanged; the page applies the Nav2 origin rotation.
        temporary_png = output_dir / ".active.png.tmp"
        image.save(temporary_png, format="PNG", optimize=False)
    png_bytes = temporary_png.read_bytes()
    temporary_png.unlink(missing_ok=True)
    target_png = output_dir / "active.png"
    _atomic_write(target_png, png_bytes)

    metadata = {
        "schema_version": 1,
        "map_id": str(map_id or ""),
        "revision": int(revision or 0),
        "backend": str(backend or ""),
        "image": "active.png",
        "resolution": float(resolution),
        "origin": [float(value) for value in origin[:3]],
        "width": width,
        "height": height,
        "sha256": hashlib.sha256(png_bytes).hexdigest(),
        "updated_at": int(__import__("time").time()),
    }
    _atomic_write(output_dir / "meta.json",
                  (json.dumps(metadata, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    return metadata


def default_web_map_dir() -> Path:
    project = os.environ.get("ROBOT_PROJECT_ROOT", str(Path.home() / "Remote_ctrol"))
    return Path(project).expanduser().resolve() / "web" / "maps"
