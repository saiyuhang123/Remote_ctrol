"""Read and fingerprint an existing Nav2 YAML/PGM map without ROS spin."""
from __future__ import annotations

from pathlib import Path
from typing import Any
import hashlib
import math

import yaml

from .models import PatrolError


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_pgm(path: Path) -> tuple[int, int, int, bytes]:
    """Return width, height, maxval and the 8-bit raster bytes."""
    data = path.read_bytes()
    if not data.startswith(b"P5"):
        raise PatrolError("INVALID_DATA", f"当前仅支持 P5 PGM 地图：{path}")

    position = 2
    tokens: list[bytes] = []
    while len(tokens) < 3:
        while position < len(data):
            byte = data[position:position + 1]
            if byte in b" \t\r\n":
                position += 1
            elif byte == b"#":
                while position < len(data) and data[position:position + 1] not in b"\r\n":
                    position += 1
            else:
                break
        if position >= len(data):
            raise PatrolError("INVALID_DATA", f"PGM 文件头不完整：{path}")
        start = position
        while position < len(data) and data[position:position + 1] not in b" \t\r\n":
            position += 1
        tokens.append(data[start:position])

    if tokens[2] != b"255":
        raise PatrolError("INVALID_DATA", f"当前仅支持 maxval=255 的 PGM：{path}")
    width, height = int(tokens[0]), int(tokens[1])
    if width <= 0 or height <= 0 or width * height > 25_000_000:
        raise PatrolError("INVALID_DATA", f"PGM 尺寸无效：{path}")

    # PGM has exactly one whitespace byte between maxval and the raster. Be
    # lenient about CRLF, which some writers use.
    if data[position:position + 2] == b"\r\n":
        position += 2
    elif position < len(data) and data[position:position + 1] in b" \t\r\n":
        position += 1
    else:
        raise PatrolError("INVALID_DATA", f"PGM 数据区起始位置无效：{path}")
    raster = data[position:position + width * height]
    if len(raster) != width * height:
        raise PatrolError("INVALID_DATA", f"PGM 数据长度无效：{path}")
    return width, height, 255, raster


def _load_map_files(navigation_yaml: str) -> tuple[Path, dict[str, Any], Path, int, int, int, bytes]:
    yaml_path = Path(navigation_yaml).expanduser().resolve()
    if not yaml_path.is_file():
        raise PatrolError("INVALID_DATA", f"导航地图 YAML 不存在：{yaml_path}")
    try:
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except Exception as error:
        raise PatrolError("INVALID_DATA", f"地图 YAML 解析失败：{error}") from error
    if not isinstance(metadata, dict):
        raise PatrolError("INVALID_DATA", "地图 YAML 必须是映射结构")
    image_value = metadata.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise PatrolError("INVALID_DATA", "地图 YAML 缺少 image 字段")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise PatrolError("INVALID_DATA", f"地图图片不存在：{image_path}")
    width, height, maxval, raster = _parse_pgm(image_path)
    return yaml_path, metadata, image_path, width, height, maxval, raster


def read_map_binding(navigation_yaml: str, name: str, backend: str,
                     localization_file: str = "") -> dict[str, Any]:
    yaml_path, metadata, image_path, width, height, _maxval, _raster = \
        _load_map_files(navigation_yaml)
    try:
        resolution = float(metadata["resolution"])
        origin = metadata["origin"]
        if not isinstance(origin, list) or len(origin) != 3:
            raise ValueError("origin 必须是三个元素")
        origin_x, origin_y, origin_yaw = (float(value) for value in origin)
    except Exception as error:
        raise PatrolError("INVALID_DATA", f"地图分辨率或原点无效：{error}") from error
    if not all(math.isfinite(value) for value in (resolution, origin_x, origin_y, origin_yaw)) or resolution <= 0:
        raise PatrolError("INVALID_DATA", "地图分辨率或原点必须是有限有效数值")

    localization_path = Path(localization_file).expanduser().resolve() if localization_file else None
    if localization_path and not localization_path.is_file():
        raise PatrolError("INVALID_DATA", f"定位文件不存在：{localization_path}")
    return {
        "name": (name or "").strip() or yaml_path.stem,
        "backend": (backend or "").strip() or "unknown",
        "frame_id": "map",
        "navigation_yaml": str(yaml_path),
        "image_path": str(image_path),
        "localization_file": str(localization_path) if localization_path else "",
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "origin_yaw": origin_yaw,
        "width": width,
        "height": height,
        "navigation_yaml_sha256": file_sha256(yaml_path),
        "image_sha256": file_sha256(image_path),
        "localization_sha256": file_sha256(localization_path) if localization_path else "",
        "verified": True,
    }


def _occupancy_lut(metadata: dict[str, Any]) -> bytes:
    """Build the exact 0..255 RGB/lightness -> OccupancyGrid value table.

    Nav2 map_server follows the map_mode.hpp definitions. For PGM maps the
    pixel value is lightness on a 0..255 scale. Trinary and scale modes compare
    occupancy (1-lightness), while raw mode treats 0..100 as direct occupancy
    values and values above 100 as unknown.
    """
    mode = str(metadata.get("mode", "trinary")).strip().lower()
    try:
        negate = bool(int(metadata.get("negate", 0)))
        free_thresh = float(metadata.get("free_thresh", 0.196))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
    except Exception as error:
        raise PatrolError("INVALID_DATA", f"地图阈值或 negate 无效：{error}") from error
    if not (math.isfinite(free_thresh) and math.isfinite(occupied_thresh)) or \
            not (0.0 <= free_thresh < occupied_thresh <= 1.0):
        raise PatrolError("INVALID_DATA", "地图 free_thresh/occupied_thresh 无效")
    if mode not in ("trinary", "scale", "raw"):
        raise PatrolError("INVALID_DATA", f"不支持的 Nav2 地图 mode：{mode}")

    values = bytearray(256)
    for pixel in range(256):
        if mode == "raw":
            # nav2_map_server's raw mode consumes the 0..100 channel value
            # directly and does not apply the negate option.
            value = pixel if 0 <= pixel <= 100 else -1
        else:
            # Match nav2_map_server exactly: convert the channel to brightness
            # first, then compute occupancy. Algebraically (255-p)/255 looks
            # equivalent but has different floating-point rounding at exact
            # threshold boundaries (for example gray 204, free_thresh=0.2).
            shade = float(pixel) / 255.0
            occupancy = shade if negate else 1.0 - shade
            if mode == "trinary":
                if occupancy > occupied_thresh:
                    value = 100
                elif occupancy < free_thresh:
                    value = 0
                else:
                    value = -1
            else:  # scale
                if occupancy > occupied_thresh:
                    value = 100
                elif occupancy < free_thresh:
                    value = 0
                else:
                    value = int(round(
                        (occupancy - free_thresh) /
                        (occupied_thresh - free_thresh) * 100.0
                    ))
        values[pixel] = value & 0xFF
    return bytes(values)


def expected_occupancy_sha256(navigation_yaml: str) -> tuple[int, int, str]:
    """Return the (width, height, digest) Nav2 should publish for a map.

    The digest covers the semantic occupancy raster in message storage order.
    Row 0 in OccupancyGrid is the bottom image row, matching nav2_map_server.
    Using a 256-byte translation table keeps this fast for production-size maps.
    """
    _yaml_path, metadata, _image_path, width, height, _maxval, raster = \
        _load_map_files(navigation_yaml)
    lut = _occupancy_lut(metadata)
    digest = hashlib.sha256()
    row_bytes = width
    for out_row in range(height):
        source_row = height - 1 - out_row
        start = source_row * row_bytes
        digest.update(raster[start:start + row_bytes].translate(lut))
    return width, height, digest.hexdigest()
