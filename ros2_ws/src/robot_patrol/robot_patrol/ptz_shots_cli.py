"""Record one or two low-zoom views at a saved patrol point, through ROS services."""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from patrol_interfaces.srv import PointShots

from .cli import call_service


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('show', 'clear'):
        sub.add_parser(command).add_argument('point_id')
    p = sub.add_parser('record', help='只读取当前位置并保存，不移动云台或底盘')
    p.add_argument('point_id')
    p.add_argument('--shot-id', required=True)
    p.add_argument('--name', default='')
    p.add_argument('--pan-sign', type=int, choices=(-1, 1),
                   help='兼容旧命令：只能校验设备统一配置，无需每次填写，不能覆盖设备配置')
    p.add_argument('--settle-seconds', type=float, default=1.5)
    p = sub.add_parser('remove')
    p.add_argument('point_id')
    p.add_argument('--shot-id', required=True)
    parsed = parser.parse_args(args)
    rclpy.init(args=[])
    node = Node('ptz_shots_calibration')
    try:
        client = node.create_client(PointShots, '/patrol/point_shots')
        state = call_service(node, client, PointShots.Request(command=PointShots.Request.GET, point_id=parsed.point_id))
        if not state.success:
            raise RuntimeError(state.message)
        if parsed.command != 'show':
            request = PointShots.Request(point_id=parsed.point_id,
                map_id=state.point.map_id, map_revision=state.map_revision,
                expected_revision=state.point.revision)
            if parsed.command == 'record':
                if parsed.pan_sign is not None and parsed.pan_sign != state.device_pan_sign:
                    raise RuntimeError('--pan-sign 与设备统一配置不一致，请先完成设备配置，不要逐点设置')
                request.command = PointShots.Request.RECORD
                request.shot_id = parsed.shot_id
                request.shot_name = parsed.name or parsed.shot_id
                request.settle_seconds = parsed.settle_seconds
            elif parsed.command == 'remove':
                request.command = PointShots.Request.REMOVE
                request.shot_id = parsed.shot_id
            else:
                request.command = PointShots.Request.CLEAR
            state = call_service(node, client, request, timeout=15.)
            if not state.success:
                raise RuntimeError(state.message)
        print(json.dumps(dict(point_id=state.point.point_id, point_name=state.point.name,
            point_revision=state.point.revision, map_id=state.point.map_id, map_revision=state.map_revision,
            inspection_enabled=state.point.inspection_enabled, device_pan_sign=state.device_pan_sign,
            shots=json.loads(state.shots_json)),
            ensure_ascii=False, indent=2))
        return 0
    except (RuntimeError, ValueError) as error:
        print(f'未保存：{error}', file=sys.stderr)
        return 1
    except Exception as error:
        print(f'视角操作失败：{error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
