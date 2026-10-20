"""Passive recorder. A draft never grants permission to move."""
from __future__ import annotations
import math

from .geometry import angle, distance, pose, motion_blocks, validate_blocks
from ..models import PatrolError


class Recorder:
    def __init__(self, map_id, map_revision, start_node='', end_node='', name='道路'):
        self.map_id, self.map_revision = map_id, map_revision
        self.start_node, self.end_node, self.name = start_node, end_node, name
        self.samples = []
        self.state = 'RECORDING'
        self.error = ''
        self.pause_pose = None
        self.last_raw = None

    def add(self, value):
        value = dict(value, **pose(value))
        if not all(math.isfinite(float(value[k])) for k in ('t', 'v')):
            raise PatrolError('STALE_DATA', '录制时间或速度无效')
        if self.state != 'RECORDING':
            return
        if self.last_raw:
            dt = value['t']-self.last_raw['t']
            if dt == 0:
                return
            if dt < 0 or dt > 1.5 or distance(value, self.last_raw) > max(0.25, dt*0.8):
                self.state, self.error = 'INVALID', '定位跳变、时间跳变或采样中断，请结束并补录'
                raise PatrolError('STALE_DATA', self.error)
        self.last_raw = value
        if len(self.samples) >= 200000:
            self.state, self.error = 'INVALID', '录制点数达到上限，请分段录制'
            raise PatrolError('INVALID_DATA', self.error)
        # Retain stationary feedback at least every 0.5 s as evidence of
        # settling, even when spatial decimation would discard those poses.
        if not self.samples or distance(value, self.samples[-1]) >= 0.035 or \
                abs(angle(value['yaw']-self.samples[-1]['yaw'])) >= 0.035 or \
                value['t']-self.samples[-1]['t'] >= 0.5 or \
                (abs(value['v']) < 0.02) != (abs(self.samples[-1]['v']) < 0.02):
            self.samples.append(value)

    def pause(self, current):
        if self.state != 'RECORDING':
            raise PatrolError('INVALID_STATE', '当前不在录制中')
        self.pause_pose, self.state = pose(current), 'PAUSED'

    def resume(self, current):
        if self.state != 'PAUSED':
            raise PatrolError('INVALID_STATE', '录制未暂停')
        if distance(current, self.pause_pose) > 0.04 or abs(angle(current['yaw']-self.pause_pose['yaw'])) > 0.08:
            raise PatrolError('OFF_ROAD', '暂停期间车辆已移动，请结束此段并重新录制，不能直线连接')
        self.last_raw, self.state = None, 'RECORDING'
        self.error = ''

    def finish(self, radius):
        if self.state not in ('RECORDING', 'PAUSED', 'REVIEW'):
            raise PatrolError('INVALID_STATE', self.error or '录制草稿不可发布')
        blocks = motion_blocks(self.samples)
        validate_blocks(blocks, radius)
        self.state = 'REVIEW'
        return blocks
