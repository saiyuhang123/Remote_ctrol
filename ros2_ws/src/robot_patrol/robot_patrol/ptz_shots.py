"""Validated calibration records for the first low-zoom shot sequence."""
from dataclasses import asdict, dataclass
import json
import math
import re

from .models import PatrolError


@dataclass(frozen=True)
class Shot:
    shot_id: str
    name: str
    pan_cal: float
    yaw_cal: float  # radians, map -> base_link
    x_cal: float
    y_cal: float
    tilt: float
    zoom: float
    pan_sign: int
    map_revision: int
    settle_seconds: float = 1.5

    def as_dict(self):
        return asdict(self)

    def target_pan(self, yaw_now):
        delta = math.degrees(math.remainder(yaw_now - self.yaw_cal, 2 * math.pi))
        return (self.pan_cal - self.pan_sign * delta) % 360.


def parse_shots(value):
    """Keep the MVP small: at most two explicitly signed, 1..4x views."""
    try:
        data = json.loads(value) if isinstance(value, str) else value
        if not isinstance(data, list) or len(data) > 2:
            raise ValueError('首版每个点最多保存两个视角')
        shots = []
        for item in data:
            shot = item if isinstance(item, Shot) else Shot(**item)
            if not isinstance(shot.shot_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', shot.shot_id):
                raise ValueError('shot_id 只允许 1～64 位字母、数字、下划线或短横线')
            if not isinstance(shot.name, str) or not shot.name.strip() or len(shot.name) > 120:
                raise ValueError('视角名称不能为空或超过 120 字')
            for name in ('pan_cal', 'yaw_cal', 'x_cal', 'y_cal', 'tilt', 'zoom', 'settle_seconds'):
                number = getattr(shot, name)
                if type(number) not in (int, float) or not math.isfinite(number):
                    raise ValueError(f'{name} 必须是有限数值')
            if not 0 <= shot.pan_cal < 360 or not -math.pi <= shot.yaw_cal <= math.pi:
                raise ValueError('pan_cal 使用 0～360 度，yaw_cal 使用 -π～π 弧度')
            if not -90 <= shot.tilt <= 25 or not 1 <= shot.zoom <= 4:
                raise ValueError('首版俯仰范围 -90～25°，倍率只允许 1～4×')
            if type(shot.pan_sign) is not int or shot.pan_sign not in (-1, 1):
                raise ValueError('pan_sign 必须明确设为 -1 或 1')
            if type(shot.map_revision) is not int or shot.map_revision < 1:
                raise ValueError('必须记录有效地图版本')
            if not 0 <= shot.settle_seconds <= 10:
                raise ValueError('对焦等待应为 0～10 秒')
            shots.append(shot)
        if len({s.shot_id for s in shots}) != len(shots):
            raise ValueError('视角 ID 不能重复')
        if len({s.pan_sign for s in shots}) > 1:
            raise ValueError('同一台云台的两个视角应使用相同 pan_sign')
        return shots
    except (ValueError, TypeError) as error:
        raise PatrolError('INVALID_SHOTS', f'拍摄序列无效：{error}') from error


def shots_json(shots):
    return json.dumps([s.as_dict() for s in parse_shots(shots)], ensure_ascii=False, allow_nan=False)


def device_pan_sign(node, required=False):
    """One deployment setting shared by all points and recording clients."""
    value = node.get_parameter('inspection_shot_pan_sign').value if node.has_parameter('inspection_shot_pan_sign') else 0
    if type(value) is not int or value not in (-1, 0, 1):
        raise PatrolError('INVALID_CONFIG', '设备水平角方向配置无效，请检查 inspection_shot_pan_sign')
    if required and value == 0:
        raise PatrolError('DEVICE_NOT_CALIBRATED', '设备方向尚未完成一次性配置，请确认实机方向后统一设置')
    return value


def recorded_shot(point, map_revision, args, before, first, second, after):
    """Validate two actual readbacks and stationary TF, shared by UI and CLI."""
    if not all(math.isfinite(value) for value in (*before[:3], *after[:3])):
        raise PatrolError('INVALID_SHOTS', '车辆定位包含无效数值')
    samples = []
    for actual in (first, second):
        if not actual.success:
            raise PatrolError('INVALID_SHOTS', actual.message or '读取实际云台位置失败')
        if not math.isfinite(actual.pan) or not 0 <= actual.pan <= 360:
            raise PatrolError('INVALID_SHOTS', '实际水平角读数无效')
        sample = Shot(shot_id=args.shot_id, name=args.name or args.shot_id,
            pan_cal=float(actual.pan) % 360., tilt=float(actual.tilt), zoom=float(actual.zoom),
            yaw_cal=after[2], x_cal=after[0], y_cal=after[1], pan_sign=args.pan_sign,
            map_revision=map_revision, settle_seconds=args.settle_seconds)
        parse_shots([sample])
        samples.append(sample)
    if (math.hypot(after[0]-before[0], after[1]-before[1]) > .03 or
            abs(math.degrees(math.remainder(after[2]-before[2], 2*math.pi))) > .5):
        raise PatrolError('INVALID_SHOTS', '标定期间车辆位姿不稳定，请停稳后重试')
    if (abs(math.remainder(second.pan-first.pan, 360.)) > .15 or
            abs(second.tilt-first.tilt) > .15 or abs(second.zoom-first.zoom) > .1):
        raise PatrolError('INVALID_SHOTS', '云台或镜头仍在运动，请松开按钮并等待稳定')
    if math.hypot(after[0]-point.x, after[1]-point.y) > .3:
        raise PatrolError('INVALID_SHOTS', '请停到距所选点位 0.3 米内再记录')
    return samples[-1]
