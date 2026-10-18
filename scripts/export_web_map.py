#!/usr/bin/env python3
"""Export a Nav2 map and exact origin (including yaw) for the web monitor."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "robot_patrol"))
from robot_patrol.map_export import default_web_map_dir, export_web_map  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml")
    parser.add_argument("--output-dir", default=str(default_web_map_dir()))
    parser.add_argument("--map-id", default="")
    parser.add_argument("--revision", type=int, default=0)
    parser.add_argument("--backend", default="")
    args = parser.parse_args()
    result = export_web_map(
        args.map_yaml, args.output_dir, map_id=args.map_id,
        revision=args.revision, backend=args.backend,
    )
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
