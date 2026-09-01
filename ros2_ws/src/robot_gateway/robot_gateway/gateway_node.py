#!/usr/bin/env python3
"""遥控网关节点：浏览器 WebSocket 指令 -> /cmd_vel。

安全机制（方案 4.3 节）：
- 心跳失控保护：超过 cmd_timeout 未收到指令即发布零速度；
- 速度限幅：远程指令强制限制在 max_linear / max_angular 以内；
- 急停：收到 estop 立即清零，直到摇杆再次输入。
"""
import asyncio
import json
import os
import threading
import time
from pathlib import Path

import rclpy
import websockets
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node

DEFAULT_CONFIG = str(Path.home() / 'Remote_ctrol' / 'config' / 'config.yaml')


def _clamp(value, limit):
    return max(-limit, min(limit, float(value)))


class GatewayNode(Node):
    def __init__(self, cfg):
        super().__init__('robot_gateway')
        ctrl = cfg['control']
        self._max_linear = float(ctrl['max_linear'])
        self._max_angular = float(ctrl['max_angular'])
        self._timeout = float(ctrl['cmd_timeout'])

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._lock = threading.Lock()
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_time = 0.0
        self._stale_logged = True  # 看门狗日志只在状态翻转时打一次

        self.create_timer(0.05, self._on_timer)  # 20Hz 输出 + 失控保护
        self.get_logger().info(
            f'网关就绪：限幅 ±{self._max_linear} m/s / ±{self._max_angular} rad/s，'
            f'超时 {self._timeout}s 自动刹车')

    def set_cmd(self, linear, angular):
        with self._lock:
            self._linear = _clamp(linear, self._max_linear)
            self._angular = _clamp(angular, self._max_angular)
            self._last_cmd_time = time.monotonic()
            self._stale_logged = False

    def estop(self):
        with self._lock:
            self._linear = 0.0
            self._angular = 0.0
            self._last_cmd_time = 0.0
        self.get_logger().warn('收到急停指令')

    def _on_timer(self):
        with self._lock:
            stale = (time.monotonic() - self._last_cmd_time) > self._timeout
            linear, angular = (0.0, 0.0) if stale else (self._linear, self._angular)
            just_stale = stale and not self._stale_logged
            if stale:
                self._stale_logged = True
        if just_stale:
            self.get_logger().warn('指令超时，自动刹车')
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._pub.publish(msg)


async def _handle_client(node, ws):
    peer = ws.remote_address
    node.get_logger().info(f'后台已接入: {peer}')
    try:
        async for raw in ws:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue
            msg_type = data.get('type')
            if msg_type == 'cmd_vel':
                node.set_cmd(data.get('linear', 0.0), data.get('angular', 0.0))
            elif msg_type == 'estop':
                node.estop()
    finally:
        node.get_logger().info(f'后台已断开: {peer}')


def _run_ws_server(node, port):
    async def _serve():
        async with websockets.serve(lambda ws: _handle_client(node, ws), '0.0.0.0', port):
            node.get_logger().info(f'WebSocket 监听 :{port}')
            await asyncio.Future()  # 常驻
    asyncio.run(_serve())


def main():
    config_path = os.environ.get('ROBOT_CONFIG', DEFAULT_CONFIG)
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    node = GatewayNode(cfg)
    threading.Thread(
        target=_run_ws_server, args=(node, int(cfg['ports']['ws'])), daemon=True
    ).start()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
