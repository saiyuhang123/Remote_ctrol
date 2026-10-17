#!/usr/bin/env python3
"""Create a synthetic Nav2 map for map-editor screenshots/manual checks."""
from argparse import ArgumentParser
from pathlib import Path


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--width", type=int, default=220)
    parser.add_argument("--height", type=int, default=140)
    args = parser.parse_args()
    width, height = args.width, args.height
    if width <= 0 or height <= 0:
        raise SystemExit("width/height must be positive")

    pixels = bytearray([254]) * (width * height)

    def put(x, y, value):
        if 0 <= x < width and 0 <= y < height:
            pixels[y * width + x] = value

    for x in range(12, width - 12):
        put(x, 12, 0)
        put(x, height - 13, 0)
    for y in range(12, height - 12):
        put(12, y, 0)
        put(width - 13, y, 0)
    for x in range(58, width - 40):
        put(x, 62, 0)
    for y in range(62, height - 30):
        put(132, y, 0)
    for y in range(18, 38):
        for x in range(150, 200):
            put(x, y, 128)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    image = args.output_dir / "map.pgm"
    image.write_bytes(b"P5\n%d %d\n255\n" % (width, height) + bytes(pixels))
    metadata = args.output_dir / "map.yaml"
    metadata.write_text(
        "image: map.pgm\n"
        "mode: trinary\n"
        "resolution: 0.05\n"
        "origin: [-5.0, -3.0, 0.0]\n"
        "negate: 0\n"
        "free_thresh: 0.25\n"
        "occupied_thresh: 0.65\n",
        encoding="utf-8",
    )
    print(metadata)


if __name__ == "__main__":
    main()
