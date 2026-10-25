#!/usr/bin/env python3
"""云台 PELCO-D 串口探针 / 控制器（RS485 半双工）。

先接线：球机预留的 485 口（A/B）→ USB-485 适配器 → /dev/ttyUSB0。
然后先探：

    python3 tools/ptz_pelco_probe.py --scan                # 扫波特率+地址，找球机
    python3 tools/ptz_pelco_probe.py --read                # 读 X/Y/Z
    python3 tools/ptz_pelco_probe.py --pan 90              # 绝对转到 90°
    python3 tools/ptz_pelco_probe.py --pan 90 --read       # 转完读回来校验

协议（来自厂商文档）：
    查询 X  FF 01 00 51 00 00 52   →  FF 01 00 59 XH XL SS
    查询 Y  FF 01 00 53 00 00 54   →  FF 01 00 5B YH YL SS
    查询 Z  FF 01 00 55 00 00 56   →  FF 01 00 5D ZH ZL SS
    设置 X  FF 01 00 4B XH XL SS
    设置 Y  FF 01 00 4D YH YL SS
    设置 Z  FF 01 00 4F ZH ZL SS
    SS = (adr + 0x00 + cmd + d1 + d2) & 0xFF

半双工：每次发完要等应答窗口，不能连发。
"""
from __future__ import annotations

import argparse
import fcntl
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit('需要 pyserial:  pip3 install pyserial')

BAUDS = (9600, 4800, 2400, 19200, 38400, 115200)
RESP = {0x51: 0x59, 0x53: 0x5B, 0x55: 0x5D}
STATS = {'ok': 0, 'no_reply': 0, 'bad_frame': 0, 'bad_sum': 0, 'busy': 0}


def open_port(port: str, baud: int) -> serial.Serial:
    """打开串口并加独占锁：两个进程同时开同一个 tty 会互相抢字节，
    pyserial 会报 'multiple access on port'。"""
    ser = serial.Serial(port, baud, timeout=0.3)
    try:
        fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        ser.close()
        raise SystemExit(f'{port} 正被另一个进程使用（可能是你另开了一个循环在读）')
    return ser


def frame(addr: int, cmd: int, d1: int = 0, d2: int = 0) -> bytes:
    body = [addr, 0x00, cmd, d1, d2]
    return bytes([0xFF] + body + [sum(body) & 0xFF])


def encode(value_x100: float) -> tuple[int, int]:
    v = int(round(value_x100)) % 36000
    return v >> 8, v & 0xFF


def decode(hi: int, lo: int, signed: bool) -> float:
    v = (hi << 8) | lo
    if signed and v > 18000:
        v -= 36000
    return v / 100.0


def bus_idle(ser: serial.Serial, window: float = 0.03) -> bool:
    """发之前先监听一小段：有数据说明别的设备（相机）正在占用总线。"""
    ser.reset_input_buffer()
    return len(ser.read(64)) == 0


def query_once(ser: serial.Serial, addr: int, cmd: int, timeout: float = 0.30):
    """发一次查询并解析。返回 (hi, lo)；失败返回 None，同时置 STATS 里的计数。"""
    ser.reset_input_buffer()
    ser.write(frame(addr, cmd))
    deadline = time.monotonic() + timeout
    buf = b''
    while time.monotonic() < deadline and len(buf) < 7:
        chunk = ser.read(7 - len(buf))
        if not chunk:
            break
        buf += chunk
    if len(buf) < 7:
        STATS['no_reply'] += 1
        return None
    if buf[0] != 0xFF or buf[1] != addr or buf[3] != RESP[cmd]:
        STATS['bad_frame'] += 1
        return None
    if (sum(buf[1:6]) & 0xFF) != buf[6]:
        STATS['bad_sum'] += 1
        return None
    STATS['ok'] += 1
    return buf[4], buf[5]


def query(ser: serial.Serial, addr: int, cmd: int, retries: int = 3):
    """带总线和校验的重试：先听空闲、再发，失败重来。"""
    for attempt in range(retries):
        if not bus_idle(ser):
            STATS['busy'] += 1
            time.sleep(0.05)
            continue
        axis = query_once(ser, addr, cmd)
        if axis:
            return axis
        time.sleep(0.05)
    return None


def read_xyz(ser: serial.Serial, addr: int, retries: int = 3):
    x = query(ser, addr, 0x51, retries)
    time.sleep(0.10)
    y = query(ser, addr, 0x53, retries)
    time.sleep(0.10)
    z = query(ser, addr, 0x55, retries)
    if not (x and y and z):
        return None
    # Z 偶尔会返回超出文档范围的值（云台正在变倍时）——重读一次
    for _ in range(2):
        zz = decode(*z, signed=False)
        if 0.0 <= zz <= 25.0:
            break
        time.sleep(0.15)
        z2 = query(ser, addr, 0x55, retries)
        if z2:
            z = z2
    return (decode(*x, signed=False), decode(*y, signed=True), decode(*z, signed=False))


def set_axis(ser: serial.Serial, addr: int, cmd: int, value_x100: float):
    hi, lo = encode(value_x100)
    ser.write(frame(addr, cmd, hi, lo))
    time.sleep(0.08)          # 半双工：给球机让出总线


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ptz485')
    ap.add_argument('--baud', type=int)
    ap.add_argument('--addr', type=int, default=1)
    ap.add_argument('--scan', action='store_true', help='扫全部波特率（1~16 地址）找球机')
    ap.add_argument('--read', action='store_true', help='读 X/Y/Z')
    ap.add_argument('--pan', type=float)
    ap.add_argument('--tilt', type=float)
    ap.add_argument('--zoom', type=float)
    ap.add_argument('--retries', type=int, default=3, help='单次查询失败重试次数（总线被相机占用时有用）')
    a = ap.parse_args()

    if a.scan:
        for baud in BAUDS:
            try:
                ser = open_port(a.port, baud)
            except Exception as exc:
                print(f'{baud:6d}: 打不开 {exc}')
                continue
            found = []
            for addr in range(1, 17):
                if query(ser, addr, 0x51, a.retries):
                    found.append(addr)
                time.sleep(0.05)
            ser.close()
            print(f'{baud:6d}: {"球机地址 " + str(found) if found else "无应答"}')
            if found:
                print(f'\n→ 用这个： --baud {baud} --addr {found[0]}')
                return
        print('\n全部波特率都没有应答：检查 A/B 是否接反、球机是否上电、485 口是否接对')
        return

    baud = a.baud or 2400
    try:
        ser = open_port(a.port, baud)
    except SystemExit:
        raise
    except Exception as exc:
        raise SystemExit(f'打不开 {a.port}: {exc}')

    if a.pan is not None:
        set_axis(ser, a.addr, 0x4B, a.pan * 100)
    if a.tilt is not None:
        set_axis(ser, a.addr, 0x4D, a.tilt * 100)
    if a.zoom is not None:
        set_axis(ser, a.addr, 0x4F, a.zoom * 100)
    if a.pan is not None or a.tilt is not None or a.zoom is not None:
        print('已发送，等 2 秒后读回…')
        time.sleep(2.0)

    if a.read or a.pan is not None or a.tilt is not None or a.zoom is not None:
        try:
            xyz = read_xyz(ser, a.addr, a.retries)
        except serial.SerialException as exc:
            raise SystemExit(f'串口读失败（适配器可能掉线，拔插一次或做 USB 复位）: {exc}')
        if xyz is None:
            print('读失败：校验后没有有效应答')
        else:
            zoom = xyz[2]
            # 厂商说明书：100~2000 对应 1~20x；零不等于 1x。
            note = '' if 1.0 <= zoom <= 20.0 else '   <- 超出说明书 1~20x 范围，不能视为有效倍率'
            print(f'X(水平) {xyz[0]:7.2f}°   Y(垂直) {xyz[1]:7.2f}°   Z(变倍) {zoom:6.2f}x{note}')
    st = STATS
    if sum(st.values()):
        print(f"[总线统计] 成功 {st['ok']}  无应答 {st['no_reply']}  帧头/地址错 {st['bad_frame']}  "
              f"校验错 {st['bad_sum']}  总线忙 {st['busy']}")
    ser.close()


if __name__ == '__main__':
    sys.exit(main())
