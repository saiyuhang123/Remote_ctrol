#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GQ-OTK9633 gas detector: serial acquisition and parameter control.

Standalone file, Python >= 3.8; Ubuntu dependency: sudo apt install python3-serial
Based on the photographed product manual (pages 6 and 7).
Read-only commands: ports, read, snapshot, watch, scan.
"""
import argparse
import json
import logging
import math
import os
import struct
import threading
import time
from typing import Optional

LOG = logging.getLogger("gq_otk9633")
UNITS = {0: "%LEL", 1: "ppm", 3: "%VOL", 5: "ug/m3"}
REGISTERS = {"value": 0x00, "state": 0x01, "type_unit": 0x02,
             "precision": 0x03, "range": 0x05, "low_alarm": 0x07,
             "high_alarm": 0x08}
WRITABLE = {0x02, 0x03, 0x05, 0x07, 0x08, 0x0D, 0x10, 0x11}


def stable_port_path(device):
    """Prefer a USB identity over the tty number assigned at plug-in time."""
    from pathlib import Path
    directory = Path("/dev/serial/by-id")
    if directory.is_dir():
        for path in sorted(directory.iterdir()):
            if os.path.realpath(path) == os.path.realpath(device):
                return str(path)
    return device


def resolve_port(port=None):
    """Select the configured detector or an unambiguous USB serial adapter."""
    if port:
        return port
    configured = os.environ.get("GQ_OTK9633_PORT")
    if configured:
        return configured
    if os.path.exists("/dev/gq_otk9633"):
        return "/dev/gq_otk9633"
    from serial.tools import list_ports
    ports = sorted((p for p in list_ports.comports() if p.vid is not None),
                   key=lambda p: p.device)
    # This detector's CH343 adapter is 1a86:55d3; the CH340 IMU is 1a86:7523.
    detector_ports = [p for p in ports if (p.vid, p.pid) == (0x1A86, 0x55D3)]
    candidates = detector_ports or ports
    if len(candidates) == 1:
        return stable_port_path(candidates[0].device)
    if not candidates:
        raise ValueError("未找到USB串口，请接上485转换器，或用 --port 指定串口")
    raise ValueError("检测到多个USB串口，请用 --port 指定气体检测仪：%s" %
                     ", ".join(stable_port_path(p.device) for p in candidates))


class ProtocolError(Exception):
    pass

class ResponseTimeout(ProtocolError):
    pass

class DeviceException(ProtocolError):
    def __init__(self, function, code):
        self.function, self.code = function, code
        super().__init__("Device exception: function=0x%02X code=0x%02X" % (function, code))


def check_int(value, minimum, maximum, name):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError("%s 必须是 %s~%s 的整数" % (name, minimum, maximum))
    return value


def crc16(data):
    """Modbus CRC16; the model manual specifies low CRC byte first."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame_with_crc(payload):
    return payload + struct.pack("<H", crc16(payload))


class GQOTK9633:
    """GQ-OTK9633, 2400/8N1. Sampling gap is strictly above 200ms."""

    def __init__(self, port: Optional[str] = None, address=1, baudrate=2400,
                 timeout=0.2, parity="N", transport=None, allow_write03=False):
        self.address = check_int(address, 1, 254, "从机地址")
        check_int(baudrate, 1, 4000000, "波特率")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("超时必须为有限正数（秒）")
        if parity not in ("N", "E", "O"):
            raise ValueError("校验位必须为N/E/O")
        self.timeout = timeout
        # Configure once before transmission. A short read timeout lets the
        # monotonic deadline work without reconfiguring CH343 during reception.
        self.read_timeout = min(timeout, 0.02)
        self.allow_write03 = allow_write03
        # 型号说明书要求采样间隔>200ms；同时满足RTU帧间隔。
        self.frame_gap = max(0.21, 3.5 * (10 if parity == "N" else 11) / baudrate)
        self._last_io = 0.0
        self._lock = threading.RLock()
        if transport is not None:
            self.serial = transport
            self.serial.timeout = self.read_timeout
        else:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("缺少pyserial；Ubuntu请运行：sudo apt install python3-serial") from exc
            port = resolve_port(port)
            LOG.debug("PORT %s -> %s; %d/8%s1, address=%d",
                      port, os.path.realpath(port), baudrate, parity, self.address)
            try:
                self.serial = serial.Serial(
                    port=port, baudrate=baudrate, bytesize=8, parity=parity,
                    stopbits=1, timeout=self.read_timeout, write_timeout=timeout,
                    xonxoff=False, rtscts=False, dsrdtr=False,
                    exclusive=True if os.name == "posix" else None)
            except serial.SerialException as exc:
                if os.name == "posix" and os.path.exists(port) and not os.access(port, os.R_OK | os.W_OK):
                    raise RuntimeError(
                        "没有串口访问权限：%s。请运行 sudo usermod -aG dialout %s，"
                        "然后注销并重新登录；也可安装随附的 gq_otk9633.rules。" %
                        (port, os.environ.get("USER", "<用户名>"))) from exc
                raise

    def close(self):
        with self._lock:
            self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _read_exact(self, size, deadline):
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ResponseTimeout("未收齐响应，已收：%s" % data.hex(" "))
            chunk = self.serial.read(size - len(data))
            data.extend(chunk)
            if time.monotonic() > deadline:
                raise ResponseTimeout("响应超过 %.3f 秒期限，已收：%s" %
                                      (self.timeout, data.hex(" ") or "无数据"))
        return bytes(data)

    def _exchange(self, function, register, value, reply_address=None):
        request = frame_with_crc(struct.pack(">BBHH", self.address, function, register, value))
        with self._lock:
            delay = self.frame_gap - (time.monotonic() - self._last_io)
            if delay > 0:
                time.sleep(delay)
            try:
                self.serial.reset_input_buffer()
                LOG.debug("TX %s", request.hex(" "))
                if self.serial.write(request) != len(request):
                    raise ProtocolError("串口未发送完整报文；写入结果未知")
                self.serial.flush()
                deadline = time.monotonic() + self.timeout
                header = self._read_exact(3, deadline)
                if header[0] not in ({self.address} if reply_address is None else {self.address, reply_address}):
                    raise ProtocolError("响应从机地址不匹配：%d" % header[0])
                if header[1] == (function | 0x80):
                    response = header + self._read_exact(2, deadline)
                elif header[1] != function and not (function == 6 and header[1] == 3 and self.allow_write03):
                    raise ProtocolError("响应功能码不匹配：0x%02X" % header[1])
                elif function == 3:
                    if header[2] != value * 2:
                        raise ProtocolError("响应数据字节数不匹配：%d" % header[2])
                    response = header + self._read_exact(header[2] + 2, deadline)
                else:
                    response = header + self._read_exact(5, deadline)
                LOG.debug("RX %s", response.hex(" "))
                if frame_with_crc(response[:-2]) != response:
                    raise ProtocolError("响应CRC校验失败：%s" % response.hex(" "))
                if response[1] & 0x80:
                    raise DeviceException(function, response[2])
                if function == 6 and response[2:6] != request[2:6]:
                    raise ProtocolError("写入回显不一致：设备不允许设置或地址/数据不匹配")
                if function == 6 and response[1] == 3:
                    LOG.warning("Non-standard 03 write reply accepted with valid CRC")
                return response
            finally:
                self._last_io = time.monotonic()


    def read_registers(self, start=0, count=1):
        check_int(start, 0, 65535, "Register address")
        check_int(count, 1, min(125, 65536 - start), "Register count")
        response = self._exchange(3, start, count)
        return list(struct.unpack(">%dH" % count, response[3:-2]))

    def write_register(self, register, value):
        """Raw 06 write; values are raw uint16 words, never engineering units."""
        if register not in WRITABLE:
            raise ValueError("Register is not documented as writable")
        check_int(value, 0, 65535, "Register value")
        with self._lock:
            if register == 0x11:
                check_int(value, 1, 254, "New slave address")
            response = self._exchange(6, register, value,
                                      value if register == 0x11 else None)
            if register == 0x11:
                self.address = value
            return response.hex(" ")

    def snapshot(self):
        """Read only documented words separately (unknown gaps may be unsupported)."""
        with self._lock:
            raw = {name: self.read_registers(reg, 1)[0]
                   for name, reg in REGISTERS.items()}
        decimals = raw["precision"] >> 8
        state = raw["state"] >> 8
        unit = raw["type_unit"] & 0xFF
        # The manual shows unsigned examples, but does not define negative encoding.
        # Always retain raw words. Decimal-scaled values are explicitly estimates
        # until confirmed against the device display.
        scale = 10 ** decimals if decimals <= 3 else None
        return {
            "address": self.address,
            "raw_value": raw["value"],
            "value_scaled": raw["value"] / scale if scale else None,
            "decimals": decimals,
            "decimal_scaling_verified": False,
            "unit_code": unit, "unit": UNITS.get(unit, "unknown"),
            "type_code": raw["type_unit"] >> 8,
            "state_code": state,
            "state": {0: "normal", 1: "low_alarm", 2: "high_alarm"}.get(state, "unknown"),
            "range_raw": raw["range"], "low_alarm_raw": raw["low_alarm"],
            "high_alarm_raw": raw["high_alarm"], "raw_registers": raw,
        }

    def set_parameter(self, name, raw_value):
        """Set range/low_alarm/high_alarm in RAW units, exactly as manual examples."""
        if name not in ("range", "low_alarm", "high_alarm"):
            raise ValueError("Supported parameters: range, low_alarm, high_alarm")
        check_int(raw_value, 0, 65535, "Raw value")
        with self._lock:
            low = raw_value if name == "low_alarm" else self.read_registers(7)[0]
            high = raw_value if name == "high_alarm" else self.read_registers(8)[0]
            span = raw_value if name == "range" else self.read_registers(5)[0]
            if not 0 <= low <= high <= span:
                raise ValueError("Require 0 <= low alarm <= high alarm <= range")
            return self.write_register(REGISTERS[name], raw_value)

    def set_unit(self, unit):
        if unit not in UNITS:
            raise ValueError("Documented unit codes: 0=%LEL, 1=ppm, 3=%VOL, 5=ug/m3")
        with self._lock:
            current = self.read_registers(2)[0]
            return self.write_register(2, (current & 0xFF00) | unit)

    def set_decimals(self, decimals):
        check_int(decimals, 0, 3, "Decimal places")
        with self._lock:
            current = self.read_registers(3)[0]
            return self.write_register(3, (decimals << 8) | (current & 0xFF))

    def zero(self):
        """Manual zero-calibration command, ONLY call under correct zero-gas conditions."""
        return self.write_register(0x0D, 0x00AA)

    def calibrate(self, standard_raw):
        """Manual span calibration with supplied standard gas; argument is a raw word."""
        check_int(standard_raw, 1, 65535, "Standard-gas raw concentration")
        return self.write_register(0x10, standard_raw)

    def change_address(self, new_address):
        """Use old or new address in a VALID acknowledgement; timeout leaves result unknown."""
        return self.write_register(0x11, new_address)

    def scan_addresses(self, start=1, end=254, first=False):
        check_int(start, 1, 254, "Start address")
        check_int(end, start, 254, "End address")
        found = {}
        with self._lock:
            original = self.address
            try:
                for address in range(start, end + 1):
                    self.address = address
                    try:
                        found[address] = {"raw_value": self.read_registers(0)[0]}
                    except ResponseTimeout:
                        pass
                    except DeviceException as exc:
                        found[address] = {"exception_code": exc.code}
                    except ProtocolError as exc:
                        LOG.warning("Address %d: %s", address, exc)
                    if address in found:
                        LOG.warning("Device found: %d %s", address, found[address])
                        if first:
                            break
                    if (address - start + 1) % 32 == 0:
                        LOG.warning("Scan progress: %d/%d", address, end)
            finally:
                self.address = original
        return found


def integer(text):
    return int(text, 16) if text.lower().startswith("0x") else int(text, 10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="串口路径；默认使用 GQ_OTK9633_PORT、"
                        "/dev/gq_otk9633 或自动识别USB串口")
    parser.add_argument("--address", type=integer, default=1)
    parser.add_argument("--baudrate", type=int, default=2400)
    parser.add_argument("--parity", choices=["N", "E", "O"], default="N")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--allow-write03", action="store_true",
                        help="Allow non-standard 03 write echo, still require valid CRC")
    subs = parser.add_subparsers(dest="command", required=True)
    subs.add_parser("ports", help="List serial ports")
    read = subs.add_parser("read", help="Read raw word(s), defaults to real-time register 0")
    read.add_argument("register", type=integer, nargs="?", default=0)
    read.add_argument("count", type=integer, nargs="?", default=1)
    subs.add_parser("snapshot", help="Read measurement, alarm status and documented parameters")
    watch = subs.add_parser("watch", help="Repeated snapshots; Ctrl+C to finish")
    watch.add_argument("--interval", type=float, default=1)
    watch.add_argument("--count", type=int, default=0,
                       help="采集次数；默认0表示持续采集")
    scan = subs.add_parser("scan", help="Read-only address discovery")
    scan.add_argument("--start", type=int, default=1)
    scan.add_argument("--end", type=int, default=254)
    scan.add_argument("--first", action="store_true")
    setting = subs.add_parser("set", help="Set parameter in RAW register units")
    setting.add_argument("parameter", choices=["range", "low_alarm", "high_alarm"])
    setting.add_argument("raw_value", type=integer)
    unit = subs.add_parser("unit", help="Set unit: 0=%%LEL, 1=ppm, 3=%%VOL, 5=ug/m3")
    unit.add_argument("code", type=integer)
    decimals = subs.add_parser("decimals", help="Set decimal places; affects numeric interpretation")
    decimals.add_argument("places", type=int)
    subs.add_parser("zero", help="Perform zero calibration; requires proper zero-gas conditions")
    cal = subs.add_parser("calibrate", help="Perform span calibration while supplying standard gas")
    cal.add_argument("standard_raw", type=integer)
    addr = subs.add_parser("address", help="Change device address")
    addr.add_argument("new_address", type=integer)
    write = subs.add_parser("write", help="Raw 06 write to documented writable register")
    write.add_argument("register", type=integer)
    write.add_argument("value", type=integer)
    args = parser.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.WARNING,
                        format="%(levelname)s %(message)s")
    if args.command == "watch" and (not math.isfinite(args.interval) or args.interval <= 0):
        parser.error("Interval must be a finite positive number")
    if args.command == "watch" and args.count < 0:
        parser.error("Count must be non-negative")
    try:
        if args.command == "ports":
            from serial.tools import list_ports
            for port in sorted(list_ports.comports(), key=lambda p: p.device):
                print("%s\t%s\t%s\t%s" % (
                    port.device, port.description, stable_port_path(port.device),
                    "可读写" if os.access(port.device, os.R_OK | os.W_OK) else "无读写权限"))
            return
        with GQOTK9633(args.port, args.address, args.baudrate, args.timeout, args.parity,
                      allow_write03=args.allow_write03) as device:
            if args.command == "scan":
                print(json.dumps(device.scan_addresses(args.start, args.end, args.first)))
            elif args.command == "read":
                print(json.dumps(device.read_registers(args.register, args.count)))
            elif args.command in ("snapshot", "watch"):
                samples = 0
                while True:
                    result = device.snapshot()
                    result["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
                    print(json.dumps(result, ensure_ascii=False), flush=True)
                    samples += 1
                    if args.command == "snapshot" or (args.count and samples >= args.count):
                        break
                    time.sleep(args.interval)
            else:
                if args.command == "set":
                    reply = device.set_parameter(args.parameter, args.raw_value)
                elif args.command == "unit":
                    reply = device.set_unit(args.code)
                elif args.command == "decimals":
                    reply = device.set_decimals(args.places)
                elif args.command == "zero":
                    reply = device.zero()
                elif args.command == "calibrate":
                    reply = device.calibrate(args.standard_raw)
                elif args.command == "address":
                    reply = device.change_address(args.new_address)
                elif args.command == "write":
                    reply = device.write_register(args.register, args.value)
                print(json.dumps({"write_acknowledged": True, "reply": reply,
                                  "current_address": device.address}))
    except KeyboardInterrupt:
        print("\nStopped")
    except (ProtocolError, ValueError, RuntimeError, OSError, ImportError) as exc:
        parser.exit(1, "ERROR: %s\n" % exc)


if __name__ == "__main__":
    main()
