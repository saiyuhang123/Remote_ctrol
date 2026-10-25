#!/usr/bin/env python3
"""云台 PELCO-D 节点（RS485）：绝对定位 + 方向点动。

服务（与 hk_camera 同名同类型 → 上位机 UI 不需要改任何代码）：
    /hk_camera/get_ptz_pose   hk_camera/srv/GetPTZPose
    /hk_camera/set_ptz_pose   hk_camera/srv/SetPTZPose
    /hk_camera/control_ptz    hk_camera/srv/ControlPTZ
    /ptz/get_pose             hk_camera/srv/GetPTZPose   （巡检/标定用别名）
    /ptz/set_pose             hk_camera/srv/SetPTZPose

点动（上下左右/变倍）用"增量绝对定位"实现：按住时每 jog_rate_hz 把目标角
加/减一小步并下发绝对坐标；松开后不再递进，云台自然停在上一个目标。
不依赖厂商是否支持连续运动命令，速度也可控。

协议（厂商文档）：
    查询 X  FF 01 00 51 00 00 52  → FF 01 00 59 XH XL SS
    查询 Y  FF 01 00 53 00 00 54  → FF 01 00 5B YH YL SS
    查询 Z  FF 01 00 55 00 00 54  → FF 01 00 5D ZH ZL SS
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
from std_msgs.msg import String

from hk_camera.srv import ControlPTZ, GetPTZPose, SetPTZPose

RESP = {0x51: 0x59, 0x53: 0x5B, 0x55: 0x5D}
CMD_PAN, CMD_TILT, CMD_ZOOM = 0x4B, 0x4D, 0x4F

# ControlPTZ.command → (d_pan, d_tilt, d_zoom)，单位是"步"
DIRECTIONS = {
    21: (0, +1, 0),      # up
    22: (0, -1, 0),      # down
    23: (-1, 0, 0),      # left
    24: (+1, 0, 0),      # right
    25: (-1, +1, 0),     # up-left
    26: (+1, +1, 0),     # up-right
    27: (-1, -1, 0),     # down-left
    28: (+1, -1, 0),     # down-right
    11: (0, 0, +1),      # zoom in
    12: (0, 0, -1),      # zoom out
}


class PelcoSerial:
    """线程安全的 PELCO-D 串口封装：发前听总线、校验和、重试。"""

    def __init__(self, port: str, baud: int, address: int, retries: int = 3):
        self.port = port
        self.baud = baud
        self.address = address
        self.retries = retries
        self.lock = threading.Lock()
        self.ser: serial.Serial | None = None
        self.stats = {'ok': 0, 'no_reply': 0, 'bad_frame': 0, 'bad_sum': 0, 'busy': 0}

    # ---------- 连接 ----------
    def open(self) -> tuple[bool, str]:
        with self.lock:
            if self.ser and self.ser.is_open:
                return True, 'already open'
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.3)
                fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as exc:
                return False, f'{self.port} 打不开或被占用: {exc}'
            except serial.SerialException as exc:
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
        ser.reset_input_buffer()
        ser.write(self._frame(cmd))
        deadline = time.monotonic() + timeout
        buf = b''
        while time.monotonic() < deadline and len(buf) < 7:
            chunk = ser.read(7 - len(buf))
            if not chunk:
                break
            buf += chunk
        if len(buf) < 7:
            self.stats['no_reply'] += 1
            return None
        if buf[0] != 0xFF or buf[1] != self.address or buf[3] != RESP[cmd]:
            self.stats['bad_frame'] += 1
            return None
        if (sum(buf[1:6]) & 0xFF) != buf[6]:
            self.stats['bad_sum'] += 1
            return None
        self.stats['ok'] += 1
        return buf[4], buf[5]

    def query(self, cmd: int):
        with self.lock:
            for _ in range(self.retries):
                if self.ser is None:
                    return None
                self.ser.reset_input_buffer()
                if self.ser.read(64):          # 总线忙（相机在发），让一让
                    self.stats['busy'] += 1
                    time.sleep(0.05)
                    continue
                axis = self._query_once(cmd)
                if axis:
                    return axis
                time.sleep(0.05)
            return None

    def read_xyz(self):
        x = self.query(0x51)
        time.sleep(0.10)
        y = self.query(0x53)
        time.sleep(0.10)
        z = self.query(0x55)
        if not (x and y and z):
            return None
        zoom = self._dec(*z, signed=False)
        for _ in range(2):                      # 变倍过程中偶发超范围值，重读
            if 0.0 <= zoom <= 25.0:
                break
            time.sleep(0.15)
            z2 = self.query(0x55)
            if z2:
                zoom = self._dec(*z2, signed=False)
        return (self._dec(*x, signed=False), self._dec(*y, signed=True), zoom)

    # ---------- 写 ----------
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
                for i in range(max(1, repeat)):
                    if i:
                        time.sleep(0.06)
                    self.ser.write(frame)
            except serial.SerialException:
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
        self.declare_parameter('jog_rate_hz', 5.0)
        self.declare_parameter('jog_step_deg', 1.5)
        self.declare_parameter('zoom_step', 0.2)
        self.declare_parameter('lease_ms', 500)
        self.declare_parameter('min_pan', 0.0)
        self.declare_parameter('max_pan', 360.0)
        self.declare_parameter('min_tilt', -90.0)
        self.declare_parameter('max_tilt', 25.0)
        self.declare_parameter('min_zoom', 1.0)
        self.declare_parameter('max_zoom', 20.0)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('publish_aliases', True)

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

        self.link = PelcoSerial(gp('port'), int(gp('baud')), int(gp('address')),
                                int(gp('retries')))

        # 位置缓存（点动靠它递进；读到真值后覆盖）
        self.pan = 0.0
        self.tilt = 0.0
        self.zoom = 1.0
        self.pos_valid = False
        self.jog_dir = (0, 0, 0)
        self.lease_deadline = 0.0
        self.last_move = 0.0
        self.last_sync = 0.0
        self._last_state = None

        self.create_service(GetPTZPose, '/hk_camera/get_ptz_pose', self.on_get)
        self.create_service(SetPTZPose, '/hk_camera/set_ptz_pose', self.on_set)
        self.create_service(ControlPTZ, '/hk_camera/control_ptz', self.on_control)
        if gp('publish_aliases'):
            self.create_service(GetPTZPose, '/ptz/get_pose', self.on_get)
            self.create_service(SetPTZPose, '/ptz/set_pose', self.on_set)
            self.create_service(ControlPTZ, '/ptz/control', self.on_control)

        self.pub_state = self.create_publisher(String, '/ptz/state', 10)
        self.create_timer(1.0 / max(self.jog_rate, 0.5), self.on_jog_tick)
        self.create_timer(1.0, self.on_state_tick)
        self.create_timer(2.0, self.on_connect_tick)

        ok, msg = self.link.open()
        self.get_logger().info(f'串口 {msg}' if ok else f'串口打开失败：{msg}（每 2 秒重试）')
        if ok:
            self.sync_position()

    # ---------- 连接 / 同步 ----------
    def on_connect_tick(self):
        if self.link.ser is None:
            ok, msg = self.link.open()
            if ok:
                self.get_logger().info(f'串口已连接：{self.link.port}')
                self.sync_position()
            else:
                self.get_logger().warn(f'串口仍未就绪：{msg}', throttle_duration_sec=10.0)

    def sync_position(self):
        xyz = self.link.read_xyz()
        if xyz is None:
            self.get_logger().warn('读取云台初始位置失败（稍后重试）')
            return False
        self.pan, self.tilt, self.zoom = xyz
        self.pos_valid = True
        self.last_sync = time.monotonic()
        sig = (round(xyz[0], 1), round(xyz[1], 1), round(xyz[2], 1))
        if sig != self._last_state:
            self._last_state = sig
            self.get_logger().info(
                f'云台：pan {xyz[0]:.2f}°  tilt {xyz[1]:.2f}°  zoom {xyz[2]:.2f}x')
        return True

    # ---------- 服务：读 ----------
    def on_get(self, request, response):
        xyz = self.link.read_xyz()
        if xyz is None:
            if self.pos_valid:
                response.success = False
                response.message = '读取失败，返回缓存值（检查总线/端口）'
                response.pan, response.tilt, response.zoom = self.pan, self.tilt, self.zoom
            else:
                response.success = False
                response.message = '读取失败且没有缓存（检查 485 接线/端口/波特率）'
            return response
        self.pan, self.tilt, self.zoom = xyz
        self.pos_valid = True
        response.success = True
        response.message = 'PELCO-D absolute pose (485)'
        response.pan, response.tilt, response.zoom = xyz
        return response

    # ---------- 服务：绝对设置 ----------
    def on_set(self, request, response):
        pan = clamp(float(request.pan) * self.pan_sign, *self.limits['pan'])
        tilt = clamp(float(request.tilt) * self.tilt_sign, *self.limits['tilt'])
        zoom = clamp(float(request.zoom), *self.limits['zoom'])
        sent = []
        if self.link.set_axis(CMD_PAN, pan) and not self._same(pan, self.pan):
            sent.append(f'pan {pan:.2f}')
        if self.link.set_axis(CMD_TILT, tilt) and not self._same(tilt, self.tilt):
            sent.append(f'tilt {tilt:.2f}')
        if self.link.set_axis(CMD_ZOOM, zoom) and not self._same(zoom, self.zoom):
            sent.append(f'zoom {zoom:.2f}')
        if self.link.ser is None:
            response.success = False
            response.message = '串口未连接'
            return response
        self.pan, self.tilt, self.zoom = pan, tilt, zoom
        self.pos_valid = True
        self.last_move = time.monotonic()
        response.success = True
        response.message = '已下发：' + (', '.join(sent) if sent else '无变化')
        return response

    @staticmethod
    def _same(a: float, b: float) -> bool:
        return abs(a - b) < 0.05

    # ---------- 服务：点动 ----------
    def on_control(self, request, response):
        if request.stop:
            self.jog_dir = (0, 0, 0)
            response.success = True
            response.message = '已停止'
            return response
        d = DIRECTIONS.get(int(request.command))
        if d is None:
            response.success = False
            response.message = f'不支持的方向命令 {request.command}'
            return response
        speed = clamp(float(request.speed or 50), 1.0, 100.0) / 50.0
        self.jog_dir = (d[0] * speed, d[1] * speed, d[2] * speed)
        self.lease_deadline = time.monotonic() + self.lease
        response.success = True
        response.message = f'方向 {request.command}，速度 {request.speed}'
        return response

    def on_jog_tick(self):
        if self.jog_dir == (0, 0, 0):
            return
        now = time.monotonic()
        if now > self.lease_deadline:          # 上位机 500ms 没续租 → 松手了
            self.jog_dir = (0, 0, 0)
            self.get_logger().info('点动租约到期，停止')
            return
        dp, dt, dz = self.jog_dir
        pan = clamp(self.pan + dp * self.jog_step, *self.limits['pan'])
        tilt = clamp(self.tilt + dt * self.jog_step, *self.limits['tilt'])
        zoom = clamp(self.zoom + dz * self.zoom_step, *self.limits['zoom'])
        moved = []
        if dp:
            self.link.set_axis(CMD_PAN, pan)
            moved.append('pan')
        if dt:
            self.link.set_axis(CMD_TILT, tilt)
            moved.append('tilt')
        if dz:
            self.link.set_axis(CMD_ZOOM, zoom)
            moved.append('zoom')
        self.pan, self.tilt, self.zoom = pan, tilt, zoom
        self.last_move = now
        if moved and now - getattr(self, '_last_log', 0.0) > 2.0:
            self._last_log = now
            self.get_logger().info(
                f'点动中 pan {pan:.1f}° tilt {tilt:.1f}° zoom {zoom:.2f}x')

    # ---------- 状态 ----------
    def on_state_tick(self):
        if self.link.ser is None:
            self.on_connect_tick()
        # 静止且距上次同步 > 3 秒，才读一次真值（纠正点动累积误差，同时别淹没总线）
        if self.jog_dir == (0, 0, 0) and time.monotonic() - max(self.last_move, self.last_sync) > 3.0:
            self.sync_position()
        st = {
            'pan': round(self.pan, 2), 'tilt': round(self.tilt, 2), 'zoom': round(self.zoom, 2),
            'valid': self.pos_valid, 'jog': list(self.jog_dir),
            'bus': self.link.stats, 'connected': self.link.ser is not None,
        }
        self.pub_state.publish(String(data=json.dumps(st)))
        self.link.stats = {'ok': 0, 'no_reply': 0, 'bad_frame': 0, 'bad_sum': 0, 'busy': 0}


def main():
    rclpy.init()
    node = PtzPelcoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.link.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
