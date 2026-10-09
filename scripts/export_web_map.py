#!/usr/bin/env python3
"""Export a Nav2 map image and its exact geometry to the remote monitor."""
import argparse
import json
import math
from pathlib import Path
import yaml
from PIL import Image


def export(map_yaml, output_dir):
    map_yaml = Path(map_yaml).expanduser().resolve()
    cfg = yaml.safe_load(map_yaml.read_text())
    resolution, origin = cfg['resolution'], cfg['origin']
    if (not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0
            or not isinstance(origin, list) or len(origin) != 3
            or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in origin)):
        raise ValueError('Invalid map resolution or origin')
    if abs(origin[2]) > 1e-9:
        raise ValueError('The web map currently requires origin yaw=0; use the unrotated PCD projection')
    source = Path(cfg['image']).expanduser()
    if not source.is_absolute():
        source = map_yaml.parent/source
    image = Image.open(source).convert('L')
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    filename = 'site.png'
    image.save(output_dir/(filename+'.tmp'), format='PNG')
    (output_dir/(filename+'.tmp')).replace(output_dir/filename)
    metadata = {'image': filename, 'resolution': float(resolution),
                'origin': origin, 'width': image.width, 'height': image.height}
    pending = output_dir/'meta.json.tmp'
    pending.write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    pending.replace(output_dir/'meta.json')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_yaml')
    parser.add_argument('--output-dir', default=str(Path(__file__).resolve().parents[1]/'web/maps'))
    args = parser.parse_args()
    result = export(args.map_yaml, args.output_dir)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
