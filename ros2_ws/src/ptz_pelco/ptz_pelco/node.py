#!/usr/bin/env python3
"""云台 PELCO-D 节点（RS485）：绝对定位 + 方向点动。

服务（与 hk_camera 同名同类型 → 上位机 UI 不需要改任何代码）：
    /hk_camera/get_ptz_pose   hk_camera/srv/GetPTZPose
    /hk_camera/set_ptz_pose   hk_camera/srv/SetPTZPose
    /hk_camera/control_ptz    hk_camera/srv/ControlPTZ
    /ptz/get_pose             hk_camera/srv/GetPTZPose   （巡检/标定用别名）
    /ptz/set_pose             hk_camera/srv/SetPTZPose

点动（上下左右）用 PELCO-D 连续运动命令（0x02 右/0x04 左/0x08 上/0x10 下，
0x00 停止）：按住发一次，期间每 250ms 重发保活，松开或租约到期立即发停止命令
——云台自己匀速走，不会一顿一顿。

镜头变倍默认通过 /hk_camera/sdk/* 控制和读取。厂商说明书将 485 Z 定义为
光学倍率（无符号二进制数 / 100），但本机实测与相机反馈不一致，尚未确认
原因，当前不把异常 Z 值作为有效倍率。SDK 连续变倍由相机平滑执行，
两层 500ms 租约保证松开、断线或节点退出后停止。zoom_backend=pelco 仅用于
确实由 485 控制镜头、且 Z 寄存器单位为倍率的其他设备。

巡航期间只读取 485 角度；串口轮询和租约回调分开执行，收发仍由同一把
串口锁串行化。合并 SDK 倍率的位置接口只在静止时同步。

协议（厂商文档）：
    查询 X  FF 01 00 51 00 00 52  → FF 01 00 59 XH XL SS
    查询 Y  FF 01 00 53 00 00 54  → FF 01 00 5B YH YL SS
    查询 Z  FF 01 00 55 00 00 56  → FF 01 00 5D ZH ZL SS
    设置 X  FF 01 00 4B XH XL SS
    设置 Y  FF 01 00 4D YH YL SS
    设置 Z  FF 01 00 4F ZH ZL SS
    SS = (地址 + 0x00 + 命令 + D1 + D2) & 0xFF
半双工：发一条等一条，不能连发。
"""
from __future__ import annotations

import json
import math
import threading
import time

import fcntl
import rclpy
import serial
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from hk_camera.srv import ControlPTZ, ControlPTZPatrol, GetPTZPose, SetPTZPose

RESP = {0x51: 0x59, 0x53: 0x5B, 0x55: 0x5D}
CMD_PAN, CMD_TILT, CMD_ZOOM = 0x4B, 0x4D, 0x4F

# ControlPTZ.command → PELCO-D 连续运动命令位（0x00 为停止）
# 实测这台设备认这套：0x02 右 / 0x04 左 / 0x08 上 / 0x10 下 / 0x20 放大 / 0x40 缩小
CAM_UP, CAM_DOWN, CAM_LEFT, CAM_RIGHT = 0x08, 0x10, 0x04, 0x02
CAM_ZOOM_IN, CAM_ZOOM_OUT = 0x20, 0x40
DIRECTIONS = {
    21: (0, CAM_UP),
    22: (0, CAM_DOWN),
    23: (CAM_LEFT, 0),
    24: (CAM_RIGHT, 0),
    25: (CAM_LEFT, CAM_UP),
    26: (CAM_RIGHT, CAM_UP),
    27: (CAM_LEFT, CAM_DOWN),
    28: (CAM_RIGHT, CAM_DOWN),
    11: (CAM_ZOOM_IN, 0),
    12: (CAM_ZOOM_OUT, 0),
}


class PelcoSerial:
    """线程安全的 PELCO-D 串口封装：发前听总线、校验和、重试。"""

    def __init__(self, port: str, baud: int, address: int, retries: int = 3):
        self.port = port
        self.baud = baud
        self.address = address
        self.retries = retries
        # Recovery reopens the port while a transaction owns the lock.
        self.lock = threading.RLock()
        self.ser: serial.Serial | None = None
        self.last_reply = 0.0
        self.stats = {'ok': 0, 'no_reply': 0, 'bad_frame': 0, 'bad_sum': 0, 'busy': 0}

    # ---------- 连接 ----------
    def open(self) -> tuple[bool, str]:
        with self.lock:
            if self.ser and self.ser.is_open:
                return True, 'already open'
            ser = None
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.03, write_timeout=0.2)
                fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as exc:
                if ser is not None:
                    ser.close()
                return False, f'{self.port} 打不开或被占用: {exc}'
            except serial.SerialException as exc:
                if ser is not None:
                    ser.close()
                return False, f'{self.port} 打不开: {exc}'
            self.ser = ser
            return True, 'ok'

    def close(self):
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                finally:
                    self.ser = None

    # ---------- 帧 ----------
    def _frame(self, cmd: int, d1: int = 0, d2: int = 0) -> bytes:
        body = [self.address, 0x00, cmd, d1, d2]
        return bytes([0xFF] + body + [sum(body) & 0xFF])

    @staticmethod
    def _enc(value_x100: float) -> tuple[int, int]:
        v = int(round(value_x100)) % 36000
        return v >> 8, v & 0xFF

    @staticmethod
    def _dec(hi: int, lo: int, signed: bool) -> float:
        v = (hi << 8) | lo
        if signed and v > 18000:
            v -= 36000
        return v / 100.0

    # ---------- 读 ----------
    def _query_once(self, cmd: int, timeout: float = 0.30):
        ser = self.ser
        if ser is None:
            return None
        try:
            ser.reset_input_buffer()
            ser.write(self._frame(cmd))
            deadline = time.monotonic() + timeout
            buf = b''
            while time.monotonic() < deadline:
                chunk = ser.read(max(1, min(64, ser.in_waiting)))
                buf += chunk
                while len(buf) >= 7:
                    if buf[0] != 0xFF:
                        buf = buf[1:]
                        self.stats['bad_frame'] += 1
                        continue
                    frame = buf[:7]
                    if (sum(frame[1:6]) & 0xFF) != frame[6]:
                        self.stats['bad_sum'] += 1
                        buf = buf[1:]
                        continue
                    buf = buf[7:]
                    if frame[1] != self.address or frame[2] != 0 or frame[3] != RESP[cmd]:
                        self.stats['bad_frame'] += 1
                        continue
                    self.stats['ok'] += 1
                    self.last_reply = time.monotonic()
                    return frame[4], frame[5]
        except (OSError, serial.SerialException):
            self._recover()
            return None
        self.stats['no_reply'] += 1
        return None

    def query(self, cmd: int, retries=None, timeout=0.30):
        with self.lock:
            for _ in range(self.retries if retries is None else retries):
                if self.ser is None:
                    return None
                try:
                    self.ser.reset_input_buffer()
                    if self.ser.read(64):      # 30ms idle window, not a 300ms delay per axis
                        self.stats['busy'] += 1
                        time.sleep(0.03)
                        continue
                except (OSError, serial.SerialException):
                    self._recover()
                    continue
                axis = self._query_once(cmd, timeout=timeout)
                if axis:
                    return axis
                time.sleep(0.05)
            return None

    def read_angles(self):
        x = self.query(0x51)
        time.sleep(0.03)
        y = self.query(0x53)
        if x is None or y is None:
            return None
        return self._dec(*x, signed=False), self._dec(*y, signed=True)

    def read_xyz(self):
        angles = self.read_angles()
        time.sleep(0.03)
        z = self.query(0x55)
        if angles is None or z is None:
            return None
        zoom = self._dec(*z, signed=False)
        for _ in range(2):                      # 变倍过程中偶发超范围值，重读
            if 0.0 <= zoom <= 25.0:
                break
            time.sleep(0.15)
            z2 = self.query(0x55)
            if z2:
                zoom = self._dec(*z2, signed=False)
        return (*angles, zoom)

    # ---------- 写 ----------
    def _wait_turnaround(self):
        # This head drops movement frames sent immediately after its reply.
        # Leave the verified half-duplex turnaround window before writing.
        remaining = .12 - (time.monotonic() - self.last_reply)
        if remaining > 0:
            time.sleep(remaining)

    def _recover(self):
        """串口报 'readiness but no data' 时，关掉重开（CH340 会卡在这个状态）。"""
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.open()

    def move(self, cmd_byte: int, pan_speed: int, tilt_speed: int) -> bool:
        """连续运动命令：按住发一次即可，松开用 stop_move()。"""
        frame = self._frame(cmd_byte, pan_speed, tilt_speed)
        with self.lock:
            if self.ser is None:
                self.open()
                if self.ser is None:
                    return False
            try:
                self._wait_turnaround()
                self.ser.write(frame)
            except (OSError, serial.SerialException):
                self._recover()
                return False
            time.sleep(0.05)
            return True

    def stop_move(self) -> bool:
        return self.move(0x00, 0x00, 0x00)

    def set_axis(self, cmd: int, value: float, repeat: int = 2) -> bool:
        """下发绝对坐标。

        实测：空闲后的第一帧经常被设备吞掉（tilt/zoom 生效、pan 不生效），
        绝对坐标命令是幂等的，所以每条发两遍，间隔一小段。
        """
        hi, lo = self._enc(value * 100.0)
        frame = self._frame(cmd, hi, lo)
        with self.lock:
            if self.ser is None:
                return False
            try:
                self._wait_turnaround()
                for i in range(max(1, repeat)):
                    if i:
                        time.sleep(0.06)
                    self.ser.write(frame)
            except (OSError, serial.SerialException):
                self._recover()
                return False
            time.sleep(0.08)                    # 半双工：给设备让出总线
            return True


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PtzPelcoNode(Node):
    def __init__(self):
        super().__init__('ptz_pelco')
        self.declare_parameter('port', '/dev/ptz485')
        self.declare_parameter('baud', 2400)
        self.declare_parameter('address', 1)
        self.declare_parameter('retries', 3)
        self.declare_parameter('jog_rate_hz', 10.0)
        self.declare_parameter('jog_step_deg', 1.5)
        self.declare_parameter('zoom_step', 1.0)
        self.declare_parameter('lease_ms', 500)
        self.declare_parameter('min_pan', 0.0)
        self.declare_parameter('max_pan', 360.0)
        self.declare_parameter('min_tilt', -90.0)
        self.declare_parameter('max_tilt', 25.0)
        self.declare_parameter('min_zoom', 1.0)
        self.declare_parameter('max_zoom', 23.0)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('publish_aliases', True)
        self.declare_parameter('zoom_backend', 'sdk')
        self.declare_parameter('sdk_service_timeout', 0.4)

        gp = lambda n: self.get_parameter(n).value
        self.jog_rate = float(gp('jog_rate_hz'))
        self.jog_step = float(gp('jog_step_deg'))
        self.zoom_step = float(gp('zoom_step'))
        self.lease = float(gp('lease_ms')) / 1000.0
        self.limits = {
            'pan': (float(gp('min_pan')), float(gp('max_pan'))),
            'tilt': (float(gp('min_tilt')), float(gp('max_tilt'))),
            'zoom': (float(gp('min_zoom')), float(gp('max_zoom'))),
        }
        self.pan_sign = -1.0 if gp('invert_pan') else 1.0
        self.tilt_sign = -1.0 if gp('invert_tilt') else 1.0
        self.zoom_backend = str(gp('zoom_backend'))
        if self.zoom_backend not in ('sdk', 'pelco'):
            raise ValueError('zoom_backend must be sdk or pelco')
        self.sdk_timeout = max(0.05, float(gp('sdk_service_timeout')))
        # Replies and timeout timers must run while a public service awaits
        # the camera; the public callbacks remain mutually exclusive.
        self.sdk_group = ReentrantCallbackGroup()
        self.sdk_get = self.create_client(GetPTZPose, '/hk_camera/sdk/get_ptz_pose',
                                          callback_group=self.sdk_group)
        self.sdk_set = self.create_client(SetPTZPose, '/hk_camera/sdk/set_ptz_pose',
                                          callback_group=self.sdk_group)
        self.sdk_control = self.create_client(ControlPTZ, '/hk_camera/sdk/control_ptz',
                                              callback_group=self.sdk_group)

        self.link = PelcoSerial(gp('port'), int(gp('baud')), int(gp('address')),
                                int(gp('retries')))

        # 位置缓存（点动靠它递进；读到真值后覆盖）
        self.pan = 0.0
        self.tilt = 0.0
        self.zoom = 1.0
        self.pos_valid = False
        self.jog = None
        self.zoom_jog = 0
        self.zoom_speed = 3
        self.zoom_target = None
        self.last_zoom_command = 0.0
        self.last_pan_command = 0.0
        self.stop_pending = False
        self.stop_head_only = False
        self.patrol = None
        self.patrol_fault = ''
        self.patrol_tilt_direction = 1
        self.motion_lock = threading.RLock()
        self.head_stamp = 0.0
        self.lease_deadline = 0.0
        self.last_move = 0.0
        self.last_sync = 0.0
        self._last_state = None
        self.last_error = ''

        self.create_service(GetPTZPose, '/hk_camera/get_ptz_pose', self.on_get)
        self.create_service(SetPTZPose, '/hk_camera/set_ptz_pose', self.on_set)
        self.create_service(ControlPTZ, '/hk_camera/control_ptz', self.on_control)
        self.create_service(ControlPTZPatrol, '/ptz/control_patrol', self.on_patrol_control)
        self.create_service(GetPTZPose, '/ptz/get_angles', self.on_get_angles)
        if gp('publish_aliases'):
            self.create_service(GetPTZPose, '/ptz/get_pose', self.on_get)
            self.create_service(SetPTZPose, '/ptz/set_pose', self.on_set)
            self.create_service(ControlPTZ, '/ptz/control', self.on_head_control)

        self.pub_state = self.create_publisher(String, '/ptz/state', 10)
        self.create_timer(1.0 / max(self.jog_rate, 0.5), self.on_jog_tick)
        # At 2400 baud, feedback + bus turnaround can exceed the timer period.
        # Keep blocking patrol I/O out of the group serving renewals and stops.
        self.patrol_io_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / max(self.jog_rate, 0.5), self._patrol_tick,
                          callback_group=self.patrol_io_group)
        self.create_timer(1.0, self.on_state_tick)
        self.create_timer(2.0, self.on_connect_tick)

        ok, msg = self.link.open()
        self.get_logger().info(f'串口 {msg}' if ok else f'串口打开失败：{msg}（每 2 秒重试）')

    # ---------- 连接 / 同步 ----------
    def on_connect_tick(self):
        if self.link.ser is None:
            ok, msg = self.link.open()
            if ok:
                self.pos_valid = False
                self.get_logger().info(f'串口已连接：{self.link.port}')
            else:
                self.get_logger().warn(f'串口仍未就绪：{msg}', throttle_duration_sec=10.0)

    async def _sdk_request(self, client, request):
        if not client.service_is_ready():
            self.last_error = '相机镜头 SDK 服务未就绪，请重启更新后的相机节点'
            return None
        future = client.call_async(request)
        timer = self.create_timer(self.sdk_timeout, future.cancel, callback_group=self.sdk_group)
        try:
            result = await future
            if result is None:
                self.last_error = '相机镜头 SDK 服务响应超时'
            elif not result.success:
                self.last_error = result.message
            return result
        finally:
            timer.cancel()
            self.destroy_timer(timer)
            if future.cancelled():
                client.remove_pending_request(future)

    async def sync_position(self):
        if self.jog is not None or self.zoom_jog or self.patrol is not None or self.stop_pending:
            self.last_error = '请先松开方向 / 变倍按钮，再读取实际位置'
            return False
        if self.zoom_backend == 'sdk':
            angles = self.link.read_angles()
            if angles is None:
                self.last_error = '485 角度读取失败，请检查接线 / 端口 / 波特率'
                self.pos_valid = False
                return False
            lens = await self._sdk_request(self.sdk_get, GetPTZPose.Request())
            if lens is None or not lens.success:
                self.pos_valid = False
                return False
            if lens.limits_valid and (math.isfinite(lens.zoom_min) and math.isfinite(lens.zoom_max)
                                      and 1 <= lens.zoom_min < lens.zoom_max <= 999.9):
                self.limits['zoom'] = (float(lens.zoom_min), float(lens.zoom_max))
            xyz = (*angles, float(lens.zoom))
        else:
            xyz = self.link.read_xyz()
            if xyz is None:
                self.last_error = '485 位置读取失败，请检查接线 / 端口 / 波特率'
                self.pos_valid = False
                return False
        # Never replace an invalid measurement with a target or stale cache.
        for name, value in zip(('pan', 'tilt', 'zoom'), xyz):
            lo, hi = self.limits[name]
            if not math.isfinite(value) or not lo - 0.01 <= value <= hi + 0.01:
                self.last_error = f'设备返回无效 {name}={value}，有效范围 {lo}~{hi}'
                self.pos_valid = False
                return False
        self.pan, self.tilt, self.zoom = xyz
        self.zoom_target = self.zoom
        self.pos_valid = True
        self.last_sync = time.monotonic()
        sig = (round(xyz[0], 1), round(xyz[1], 1), round(xyz[2], 1))
        if sig != self._last_state:
            self._last_state = sig
            self.get_logger().info(
                f'云台：pan {xyz[0]:.2f}°  tilt {xyz[1]:.2f}°  zoom {xyz[2]:.2f}x')
        return True

    # ---------- 服务：读 ----------
    async def on_get(self, request, response):
        if not await self.sync_position():
            response.success = False
            response.message = self.last_error
            return response
        response.success = True
        response.message = ('485 云台实际角度 / SDK 实际光学倍率' if self.zoom_backend == 'sdk'
                            else 'PELCO-D absolute pose (485)')
        response.pan, response.tilt, response.zoom = self.pan, self.tilt, self.zoom
        # 上报真实可用范围：pan 0~360、tilt -90~25、zoom 1~23（实测本机可到 23x）
        response.limits_valid = True
        response.pan_min, response.pan_max = self.limits['pan']
        response.tilt_min, response.tilt_max = self.limits['tilt']
        response.zoom_min, response.zoom_max = self.limits['zoom']
        return response

    # ---------- 服务：绝对设置 ----------
    async def on_set(self, request, response):
        if self.jog is not None or self.zoom_jog or self.patrol is not None or self.stop_pending:
            response.message = '请先松开方向 / 变倍按钮，再设置绝对位置'
            return response
        pan, tilt, zoom = float(request.pan), float(request.tilt), float(request.zoom)
        for name, value in (('pan', pan), ('tilt', tilt), ('zoom', zoom)):
            lo, hi = self.limits[name]
            if not math.isfinite(value) or not lo <= value <= hi:
                response.message = f'{name} 超出有效范围 {lo}~{hi}'
                return response
        # Direction inversion applies to jogs only. Absolute coordinates must
        # round-trip unchanged, including negative tilt and a 360-degree pan.
        pan = 0.0 if pan == 360.0 else pan
        if self.link.ser is None:
            response.message = '串口未连接'
            return response
        sent = []
        was_valid = self.pos_valid
        # A write acknowledges transmission only; it is never actual feedback.
        self.pos_valid = False
        # On this IPZoom + RS485 head, even an unchanged SDK zoom target stops
        # an in-flight absolute head move. Skip unchanged zoom and send any
        # required lens command BEFORE the head coordinates.
        if not was_valid or not self._same(zoom, self.zoom):
            if self.zoom_backend == 'sdk':
                result = await self._sdk_request(
                    self.sdk_set, SetPTZPose.Request(pan=0., tilt=0., zoom=zoom))
                ok = result is not None and result.success
            else:
                ok = self.link.set_axis(CMD_ZOOM, zoom)
            if not ok:
                response.message = '镜头变倍设置失败：' + (self.last_error or '485 发送失败')
                self.last_move = time.monotonic()
                return response
            sent.append(f'zoom {zoom:.2f}')
        for name, cmd, value, current in (('pan', CMD_PAN, pan, self.pan),
                                          ('tilt', CMD_TILT, tilt, self.tilt)):
            if was_valid and self._same(value, current):
                continue
            if not self.link.set_axis(cmd, value):
                response.message = f'{name} 串口发送失败，已发送的轴需读取确认'
                self.last_move = time.monotonic()
                return response
            sent.append(f'{name} {value:.2f}')
        self.last_move = time.monotonic()
        response.success = True
        response.message = ('目标已下发，需读回确认到位：' + ', '.join(sent) if sent else
                            '目标与上次有效读数一致，无需重复下发')
        if sent:
            self.get_logger().info('绝对定位下发顺序：' + ', '.join(sent))
        return response

    @staticmethod
    def _same(a: float, b: float) -> bool:
        return abs(a - b) < 0.05

    # ---------- 服务：点动（连续运动命令）----------
    async def on_head_control(self, request, response):
        """RS485 head only; no SDK dependency, including stop acknowledgments."""
        if request.stop:
            response.success = await self._stop_jog(force=True, head_only=True)
            if response.success:
                self.patrol_fault = ''
            response.message = '485 云台已停止' if response.success else '485 停止发送失败，将重试'
            return response
        if int(request.command) not in range(21, 29):
            response.message = '此接口仅控制 485 云台方向，变倍请使用相机接口'
            return response
        return await self.on_control(request, response)

    def on_get_angles(self, request, response):
        # Patrol already samples both axes. Extra reads delay endpoint changes.
        if self.patrol is not None:
            angles = ((self.pan, self.tilt) if
                      time.monotonic() - self.head_stamp < 1.5 else None)
        else:
            angles = self.link.read_angles()
        if angles is None or not self._valid_angles(*angles):
            response.message = '485 实际角度读取失败'
            return response
        self.pan, self.tilt = angles
        self.head_stamp = time.monotonic()
        response.success = True
        response.message = '485 云台角度；此接口不读取镜头倍率'
        response.pan, response.tilt = angles
        response.limits_valid = True
        response.pan_min, response.pan_max = self.limits['pan']
        response.tilt_min, response.tilt_max = self.limits['tilt']
        return response

    def _valid_angles(self, pan, tilt):
        return (math.isfinite(pan) and math.isfinite(tilt) and
                self.limits['pan'][0] <= pan <= self.limits['pan'][1] and
                self.limits['tilt'][0] <= tilt <= self.limits['tilt'][1])

    def _patrol_response(self, response, ok, message):
        response.success, response.message = bool(ok), message
        patrol = self.patrol
        response.moving = patrol is not None
        response.positioning = patrol is not None and patrol['positioning']
        response.pose_valid = self.head_stamp > 0 and time.monotonic() - self.head_stamp < 1.5
        response.pan, response.tilt = float(self.pan), float(self.tilt)
        return response

    async def on_patrol_control(self, request, response):
        if request.stop:
            ok = await self._stop_jog(force=True, head_only=True)
            if ok:
                self.patrol_fault = ''
            return self._patrol_response(response, ok, '485 巡航已停止' if ok else '485 停止失败，将重试')
        lo, hi = float(request.tilt_min), float(request.tilt_max)
        if (request.command not in (23, 24) or not 1 <= request.pan_speed <= 7 or
                not 1 <= request.tilt_speed <= 7 or not math.isfinite(lo) or not math.isfinite(hi) or
                (request.tilt_enabled and not self.limits['tilt'][0] <= lo < hi <= self.limits['tilt'][1]) or
                (request.tilt_enabled and hi - lo < 2.)):
            return self._patrol_response(response, False, '485 巡航方向、速度或俯仰范围无效')
        config = (request.pan_speed, request.tilt_enabled, lo, hi, request.tilt_speed)
        with self.motion_lock:
            if self.patrol_fault:
                return self._patrol_response(response, False, self.patrol_fault + '；请先停止再重新启动')
            if self.stop_pending:
                return self._patrol_response(response, False, '上次停止尚未确认')
            if self.patrol is not None:
                if config != self.patrol['config']:
                    return self._patrol_response(response, False, '请先停止再更改俯仰范围或速度')
                self.patrol['pan_command'] = request.command
                self.lease_deadline = time.monotonic() + self.lease
                self.last_move = time.monotonic()
                return self._patrol_response(response, True, '485 巡航租约已续期')
        if self.patrol is None:
            if self.jog is not None or self.zoom_jog:
                if not await self._stop_jog():
                    return self._patrol_response(response, False, '无法停止前一动作')
            angles = self.link.read_angles()
            if angles is None or not self._valid_angles(*angles):
                return self._patrol_response(response, False, '启动前无法读取 485 角度，拒绝盲目摆动')
            self.pan, self.tilt = angles
            self.head_stamp = time.monotonic()
            positioning = bool(request.tilt_enabled and not lo <= self.tilt <= hi)
            target = clamp(self.tilt, lo, hi)
            patrol = dict(config=config, pan_command=request.command,
                pan_speed=request.pan_speed * 9, tilt_speed=request.tilt_speed * 9,
                lo=lo, hi=hi, tilt_enabled=request.tilt_enabled,
                tilt_direction=self.patrol_tilt_direction,
                positioning=positioning, target=target, start=time.monotonic(),
                last_tilt=self.tilt, last_sample=time.monotonic(),
                last_pan_query=time.monotonic(), last_command=0., command=None,
                last_tilt_speed=None, approaching=False)
            patrol.update(pan_progress=self.pan, tilt_progress=self.tilt,
                          pan_progress_time=time.monotonic(), tilt_progress_time=time.monotonic())
            self.pos_valid = False
            if positioning:
                if not self.link.stop_move() or not self.link.set_axis(CMD_TILT, target):
                    self.patrol_fault = '初始俯仰定位发送失败'
                    await self._stop_jog(force=True, head_only=True)
                    return self._patrol_response(response, False, self.patrol_fault)
            with self.motion_lock:
                self.lease_deadline = time.monotonic() + self.lease
                self.last_move = time.monotonic()
                self.patrol = patrol
        return self._patrol_response(response, True, '485 巡航租约已续期')

    async def _fail_patrol(self, patrol, message):
        await self._stop_jog(force=True, head_only=True,
                             expected_patrol=patrol, fault=message)

    async def _patrol_tick(self):
        patrol = self.patrol
        if patrol is None or self.stop_pending:
            return
        now = time.monotonic()
        if now > self.lease_deadline:
            await self._fail_patrol(patrol, '485 巡航续租超时，已停止')
            return
        # Short, single-attempt reads keep stop/lease callbacks responsive.
        y = self.link.query(0x53, retries=1, timeout=.12)
        if self.patrol is not patrol:
            return
        tilt = self.link._dec(*y, signed=True) if y is not None else None
        if tilt is None or not self._valid_angles(self.pan, tilt):
            await self._fail_patrol(patrol, '485 俯仰反馈无效，已停止巡航')
            return
        sample_time = time.monotonic()
        delta = tilt - patrol['last_tilt']
        dt = max(.01, sample_time - patrol['last_sample'])
        rate = delta / dt
        self.tilt = tilt
        if now - patrol['last_pan_query'] >= .8:
            x = self.link.query(0x51, retries=1, timeout=.12)
            if self.patrol is not patrol:
                return
            if x is None:
                await self._fail_patrol(patrol, '485 水平反馈丢失，已停止巡航')
                return
            self.pan = self.link._dec(*x, signed=False)
            if not self._valid_angles(self.pan, tilt):
                await self._fail_patrol(patrol, '485 水平反馈无效，已停止巡航')
                return
            patrol['last_pan_query'] = time.monotonic()
        self.head_stamp = time.monotonic()
        patrol['last_sample'], patrol['last_tilt'] = sample_time, tilt
        if patrol['positioning']:
            if abs(tilt - patrol['target']) <= .3:
                patrol['positioning'] = False
                patrol['pan_progress_time'] = patrol['tilt_progress_time'] = time.monotonic()
            elif now - patrol['start'] > 15.:
                await self._fail_patrol(patrol, '15 秒内未到达俯仰巡航范围，已停止')
            return
        if abs(math.remainder(self.pan - patrol['pan_progress'], 360.)) >= .1:
            patrol['pan_progress'], patrol['pan_progress_time'] = self.pan, time.monotonic()
        if abs(tilt - patrol['tilt_progress']) >= .1:
            patrol['tilt_progress'], patrol['tilt_progress_time'] = tilt, time.monotonic()
        if patrol['command'] is not None and (now - patrol['pan_progress_time'] > 3. or
                (patrol['tilt_enabled'] and now - patrol['tilt_progress_time'] > 3.)):
            await self._fail_patrol(patrol, '485 指令已发送但角度持续无变化，已停止巡航')
            return
        if patrol['tilt_enabled']:
            if not patrol['lo'] - .3 <= tilt <= patrol['hi'] + .3:
                await self._fail_patrol(patrol, '俯仰超出巡航范围，已停止')
                return
            # Slow the approach so feedback can reach the requested endpoint
            # instead of reversing several degrees early at cruising speed.
            previous_direction = patrol['tilt_direction']
            if tilt >= patrol['hi'] - .1:
                patrol['tilt_direction'] = -1
            elif tilt <= patrol['lo'] + .1:
                patrol['tilt_direction'] = 1
            if previous_direction != patrol['tilt_direction']:
                patrol['approaching'] = False
        pan_bit = CAM_LEFT if patrol['pan_command'] == 23 else CAM_RIGHT
        if self.pan_sign < 0:
            pan_bit ^= CAM_LEFT | CAM_RIGHT
        tilt_bit = 0
        tilt_speed = 0
        if patrol['tilt_enabled']:
            # On this head, DOWN increases the signed Y coordinate. Manual
            # button inversion must not invert feedback-based positioning.
            tilt_bit = CAM_DOWN if patrol['tilt_direction'] > 0 else CAM_UP
            remaining = (patrol['hi'] - tilt if patrol['tilt_direction'] > 0 else tilt - patrol['lo'])
            slow_window = min((patrol['hi'] - patrol['lo']) / 2., max(3., abs(rate) * 1.2 + .5))
            patrol['approaching'] = patrol['approaching'] or remaining <= slow_window
            tilt_speed = 1 if patrol['approaching'] else patrol['tilt_speed']
        cmd = pan_bit | tilt_bit
        if (cmd != patrol['command'] or tilt_speed != patrol['last_tilt_speed'] or
                now - patrol['last_command'] >= .2):
            with self.link.lock:
                # Stop cancels this generation before waiting for the bus.
                # A queued old sample must never restart after the stop ACK.
                if self.patrol is not patrol or self.stop_pending:
                    return
                fresh = time.monotonic() <= self.lease_deadline
                ok = fresh and self.link.move(cmd, patrol['pan_speed'], tilt_speed)
            if not ok:
                await self._fail_patrol(patrol, '485 巡航方向发送失败，已停止' if fresh
                                       else '485 巡航续租超时，已停止')
                return
            patrol['command'], patrol['last_command'] = cmd, time.monotonic()
            patrol['last_tilt_speed'] = tilt_speed
        self.last_move = time.monotonic()

    async def on_control(self, request, response):
        if request.stop:
            response.success = await self._stop_jog(force=True)
            response.message = '已停止' if response.success else '停止发送失败，将继续重试'
            return response
        if self.patrol is not None:
            response.message = '485 巡航正在占用云台，请先停止巡航'
            return response
        if self.stop_pending and not await self._stop_jog(force=True):
            response.message = '上一次停止尚未确认，请检查云台 / 相机连接'
            return response
        d = DIRECTIONS.get(int(request.command))
        if d is None:
            response.success = False
            response.message = f'不支持的方向命令 {request.command}'
            return response
        # Match the public SDK service's 1..7 speed scale. Accept the older
        # RS485 UI's 8..100 percentage values for compatibility.
        requested_speed = float(request.speed or 3)
        speed_v = (requested_speed / 7.0 if requested_speed <= 7
                   else clamp(requested_speed, 8., 100.) / 100.)
        if int(request.command) in (11, 12):
            direction = +1 if int(request.command) == 11 else -1
            if self.jog is not None or (self.zoom_jog and self.zoom_jog != direction):
                if not await self._stop_jog():
                    response.message = '无法停止上一次云台 / 镜头动作'
                    return response
            if not self.zoom_jog:
                if self.zoom_backend == 'sdk':
                    self.zoom_speed = max(1, min(7, round(speed_v * 7)))
                    result = await self._sdk_request(self.sdk_control, ControlPTZ.Request(
                        command=int(request.command), speed=self.zoom_speed, stop=False))
                    if result is None or not result.success:
                        response.message = self.last_error
                        return response
                    self.last_zoom_command = time.monotonic()
                else:
                    if not await self.sync_position():
                        response.message = self.last_error
                        return response
                    self.zoom_target = self.zoom
                self.zoom_jog = direction
                self.pos_valid = False
            self.lease_deadline = time.monotonic() + self.lease
            self.last_move = time.monotonic()
            response.success = True
            response.message = '镜头变倍租约已续期'
            return response
        speed = int(speed_v * 63.0)
        speed = max(1, min(63, speed))
        cmd_byte = d[0] | d[1]
        if self.pan_sign < 0:
            cmd_byte ^= (CAM_LEFT | CAM_RIGHT) if d[0] else 0
        if self.tilt_sign < 0:
            cmd_byte ^= (CAM_UP | CAM_DOWN) if d[1] else 0
        if self.zoom_jog or (self.jog is not None and self.jog != (cmd_byte, speed)):
            if not await self._stop_jog():
                response.message = '无法停止上一次云台 / 镜头动作'
                return response
        if self.jog == (cmd_byte, speed):
            ok = True
        else:
            ok = self.link.move(cmd_byte, speed, speed)
            self.last_pan_command = time.monotonic()
        if not ok:
            response.success = False
            response.message = '串口发送失败（检查 485/端口）'
            return response
        self.jog = (cmd_byte, speed)
        self.pos_valid = False
        self.lease_deadline = time.monotonic() + self.lease
        self.last_move = time.monotonic()
        response.success = True
        response.message = f'连续运动 命令 0x{cmd_byte:02X} 速度 {speed}/63'
        return response

    async def _stop_jog(self, force=False, head_only=False, expected_patrol=None, fault=''):
        ok = True
        with self.motion_lock:
            if expected_patrol is not None and self.patrol is not expected_patrol:
                return True
            if fault:
                self.patrol_fault = fault
            active_head = self.jog is not None or self.patrol is not None or force
            if self.patrol is not None:
                self.patrol_tilt_direction = self.patrol['tilt_direction']
            self.patrol = None
            self.stop_pending = True
            self.stop_head_only = head_only
            self.lease_deadline = 0.0
        # Share the same critical section as the patrol's final movement check.
        with self.link.lock:
            if active_head:
                ok = self.link.stop_move()
                if ok:
                    self.jog = None
            if head_only:
                self.stop_pending = not ok
                self.last_move = time.monotonic()
                return ok
        if not head_only and self.zoom_backend == 'sdk' and (self.zoom_jog or force):
            result = await self._sdk_request(self.sdk_control,
                ControlPTZ.Request(command=0, speed=self.zoom_speed, stop=True))
            zoom_ok = result is not None and result.success
            if zoom_ok:
                self.zoom_jog = 0
            ok = ok and zoom_ok
        elif self.zoom_backend == 'pelco':
            self.zoom_jog = 0
        # A failed stop stays pending with an expired lease so watchdog ticks
        # retry; canceled patrol work must never resume while retrying.
        self.lease_deadline = 0.0
        self.stop_pending = not ok
        self.stop_head_only = head_only
        self.last_move = time.monotonic()

        return ok

    async def on_jog_tick(self):
        """Renew continuous commands only while the operator lease is fresh."""
        if self.stop_pending:
            await self._stop_jog(force=True, head_only=self.stop_head_only)
            return
        patrol = self.patrol
        if patrol is not None:
            if time.monotonic() > self.lease_deadline:
                await self._fail_patrol(patrol, '485 巡航续租超时，已停止')
            return
        if self.jog is None and not self.zoom_jog:
            return
        now = time.monotonic()
        if now > self.lease_deadline:
            await self._stop_jog()
            self.get_logger().debug('点动租约到期，已停止')
            return
        if self.zoom_jog:
            if self.zoom_backend == 'sdk':
                if now - self.last_zoom_command >= 0.2:
                    result = await self._sdk_request(self.sdk_control, ControlPTZ.Request(
                        command=11 if self.zoom_jog > 0 else 12, speed=self.zoom_speed, stop=False))
                    if result is None or not result.success:
                        self.lease_deadline = 0.0
                        await self._stop_jog()
                        return
                    self.last_zoom_command = time.monotonic()
            else:
                target = clamp(self.zoom_target + self.zoom_jog * self.zoom_step, *self.limits['zoom'])
                if not self._same(target, self.zoom_target):
                    if not self.link.set_axis(CMD_ZOOM, target, repeat=1):
                        await self._stop_jog()
                        return
                    self.zoom_target = target
            self.last_move = now
            return
        if now - self.last_pan_command >= 0.25:
            if not self.link.move(self.jog[0], self.jog[1], self.jog[1]):
                self.lease_deadline = 0.0
                await self._stop_jog()
            self.last_move = now
            self.last_pan_command = now

    # ---------- 状态 ----------
    async def on_state_tick(self):
        if self.link.ser is None:
            self.on_connect_tick()
        # 静止且距上次同步 > 3 秒，才读一次真值（纠正点动累积误差，同时别淹没总线）
        if self.jog is None and not self.zoom_jog and self.patrol is None and not self.stop_pending and time.monotonic() - max(self.last_move, self.last_sync) > 3.0:
            if not await self.sync_position():
                self.get_logger().warn(self.last_error, throttle_duration_sec=10.)
        st = {
            'pan': round(self.pan, 2), 'tilt': round(self.tilt, 2), 'zoom': round(self.zoom, 2),
            'valid': self.pos_valid, 'jog': self.jog,
            'zoom_jog': self.zoom_jog, 'zoom_backend': self.zoom_backend,
            'patrol': self.patrol is not None, 'patrol_fault': self.patrol_fault,
            'bus': self.link.stats, 'connected': self.link.ser is not None,
        }
        self.pub_state.publish(String(data=json.dumps(st)))
        self.link.stats = {'ok': 0, 'no_reply': 0, 'bad_frame': 0, 'bad_sum': 0, 'busy': 0}


def main():
    rclpy.init()
    node = PtzPelcoNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGTERM can invalidate the context while Humble creates a wait set.
        # Preserve ordinary runtime failures; context teardown is expected.
        if rclpy.ok():
            raise
    finally:
        try:
            # SDK futures need executor callbacks, which no longer run after
            # shutdown starts. An unbounded wait here leaves the driver alive,
            # advertising dead services and retaining the exclusive 485 port.
            executor.shutdown(timeout_sec=1.)
        finally:
            try:
                # The camera has its own zoom lease even if ROS is shut down.
                if node.link.ser is not None:
                    node.link.stop_move()
            finally:
                node.link.close()
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
