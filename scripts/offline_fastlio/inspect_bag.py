#!/usr/bin/env python3
"""Inspect a ROS2 sqlite bag without replaying the whole file.

Prints topic totals, duration, frame IDs, point fields/ring range and IMU
sample fields. Intended for LiDAR+IMU FAST-LIO bags.
"""
from __future__ import annotations
import argparse
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    args = ap.parse_args()
    bag = Path(args.bag).expanduser().resolve()
    db = next(bag.glob('*.db3'), None)
    if db is None:
        raise SystemExit(f'no .db3 in {bag}')
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    con.row_factory = sqlite3.Row
    print(f'bag: {bag}')
    print(f'db : {db} ({db.stat().st_size/1024**3:.2f} GiB)')
    for t in con.execute('select id,name,type from topics order by id'):
        count = con.execute('select count(*) from messages where topic_id=?', (t['id'],)).fetchone()[0]
        print(f"topic {t['name']}  type={t['type']}  count={count}")

    for topic, msg_type in (('/imu/data', Imu), ('/velodyne_points', PointCloud2)):
        row = con.execute(
            'select m.data from messages m join topics t on t.id=m.topic_id '
            'where t.name=? order by m.id limit 1', (topic,)).fetchone()
        if row is None:
            print(f'{topic}: not present')
            continue
        msg = deserialize_message(bytes(row['data']), msg_type)
        print(f'--- {topic} first ---')
        print('frame_id:', repr(msg.header.frame_id),
              'stamp:', f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        if msg_type is Imu:
            print('acc:', [round(float(x), 4) for x in (
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)])
            print('gyro:', [round(float(x), 4) for x in (
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)])
        else:
            print('width/height/point_step:', msg.width, msg.height, msg.point_step)
            print('fields:', [(f.name, f.offset, f.datatype, f.count) for f in msg.fields])
            names = [f.name for f in msg.fields]
            wanted = [x for x in ('x', 'y', 'z', 'intensity', 'ring', 'time') if x in names]
            raw = np.asarray(list(point_cloud2.read_points(msg, field_names=wanted, skip_nans=True)))
            if len(raw):
                xyz = np.column_stack((raw['x'], raw['y'], raw['z'])) if 'x' in names else raw
                print('sample points:', len(raw))
                for n in wanted:
                    v = raw[n]
                    print(f'  {n}: min={float(np.min(v)):.6g} max={float(np.max(v)):.6g}')
                if 'ring' in wanted:
                    print('  rings:', np.unique(raw['ring']).tolist())
                if 'time' in wanted:
                    t = raw['time']
                    print(f'  time span (raw): {float(np.max(t)-np.min(t)):.6f}')
    con.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
