#!/usr/bin/env python3
"""遥控网关节点：浏览器 WebSocket 指令 -> /cmd_vel。

安全机制（方案 4.3 节）：
- 心跳失控保护：超过 cmd_timeout 未收到指令即发布零速度；
- 速度限幅：远程指令强制限制在 max_linear / max_angular 以内；
- 急停：收到 estop 立即清零，直到摇杆再次输入。
"""
import asyncio
import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
import websockets
import yaml
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# 配置文件候选路径（按优先级），也可用环境变量 ROBOT_CONFIG 指定
CONFIG_CANDIDATES = [
    Path.home() / 'ros2Project' / 'Remote_ctrol' / 'config' / 'config.yaml',
    Path.home() / 'Remote_ctrol' / 'config' / 'config.yaml',
]

# 遥测目标坐标系：map（接 AMCL 定位后与地图图片对齐）；
# map 系不可用时（Nav2 未启动）回退 odom
TARGET_FRAME = 'map'
FALLBACK_FRAME = 'odom'


def _clamp(value, limit):
    return max(-limit, min(limit, float(value)))


class GatewayNode(Node):
    def __init__(self, cfg):
        super().__init__('robot_gateway')
        ctrl = cfg['control']
        self._max_linear = float(ctrl['max_linear'])
        self._max_angular = float(ctrl['max_angular'])
        self._timeout = float(ctrl['cmd_timeout'])

        # Twist 是 geometry_msgs 功能包提供的一个标准消息类型，专门用来表示空间中的速度  线速度 和角速度
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._lock = threading.Lock()
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_time = 0.0
        self._stale_logged = True  # 看门狗日志只在状态翻转时打一次

        self.create_timer(0.05, self._on_timer)  # 20Hz 输出 + 失控保护

        # ---- 遥测：位姿 + 雷达点 -> WebSocket 推送（远程地图监控用）----
        self._odom = None
        self._scan = None
        self._ws_clients = set()   # 已接入的浏览器
        self._ws_loop = None       # ws 服务所在线程的 event loop
        # Nav2 导航 action client（网页目标点下发）
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_status = 'idle'  # idle/sent/driving/succeeded/aborted/canceled/rejected/error
        self.create_subscription(
            Odometry, '/odom', lambda m: setattr(self, '_odom', m),
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan', lambda m: setattr(self, '_scan', m),
            qos_profile_sensor_data)
        # 识别事件透传：detector_node -> 浏览器
        self.create_subscription(
            String, '/detection_events', self._on_detection_event, 10)
        # 抓拍：浏览器 -> /capture_request；/capture_done -> 浏览器
        self._pub_capture = self.create_publisher(String, '/capture_request', 10)
        self.create_subscription(
            String, '/capture_done', self._on_capture_done, 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.1, self._make_telemetry)  # 10Hz 遥测

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

    def set_initial_pose(self, x, y, yaw):
        """网页下发的初始定位（AMCL /initialpose）"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2)
        # 小协方差：信任操作员点选
        msg.pose.covariance[0] = 0.25    # x
        msg.pose.covariance[7] = 0.25    # y
        msg.pose.covariance[35] = 0.07   # yaw
        self._pub_initialpose.publish(msg)
        self.get_logger().info(f'初始定位已下发: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

    # ---- Nav2 目标点下发 / 取消 ----
    def send_nav_goal(self, x, y, yaw):
        if not self._nav_client.server_is_ready():
            self._nav_status = 'error'
            self.get_logger().error('Nav2 未就绪，目标点下发失败')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2)
        self._nav_status = 'sent'
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_nav_response)
        self.get_logger().info(f'导航目标已下发: ({x:.2f}, {y:.2f})')

    def _on_nav_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self._nav_status = 'rejected'
            self.get_logger().warn('导航目标被拒绝')
            return
        self._nav_goal_handle = handle
        self._nav_status = 'driving'
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, fut):
        status = fut.result().status
        self._nav_status = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(
            status, f'code{status}')
        self._nav_goal_handle = None
        self.get_logger().info(f'导航结束: {self._nav_status}')

    def cancel_nav(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_status = 'canceling'
            self.get_logger().info('取消导航')

    def _on_timer(self):
        with self._lock:
            stale = (time.monotonic() - self._last_cmd_time) > self._timeout
            linear, angular = (0.0, 0.0) if stale else (self._linear, self._angular)
            just_stale = stale and not self._stale_logged
            if stale:
                self._stale_logged = True
            # 闲置静默：从未收到指令、或刹车已持续 2s 以上，则停止发布，
            # 把 /cmd_vel 让给 Nav2（正式的优先级仲裁由 twist_mux 接管，此为过渡方案）
            silent = stale and (
                self._last_cmd_time == 0.0
                or time.monotonic() - self._last_cmd_time > self._timeout + 2.0)
        if just_stale:
            self.get_logger().warn('指令超时，自动刹车')
        if silent:
            return
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._pub.publish(msg)

    # ---- 遥测组装与推送 ----
    @staticmethod
    def _yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def _make_telemetry(self):
        if self._odom is None or self._scan is None or not self._ws_clients:
            return
        tw = self._odom.twist.twist
        # 位姿：直接用 tf 查 目标系->车架（map 不可用时回退 odom）
        frame = None
        tf_base = None
        for cand in (TARGET_FRAME, FALLBACK_FRAME):
            try:
                tf_base = self._tf_buffer.lookup_transform(
                    cand, 'Frame', Time())
                frame = cand
                break
            except Exception:
                continue
        if tf_base is None:
            return  # tf 树未就绪，跳过本帧
        p = tf_base.transform.translation
        pose = {'x': round(p.x, 3), 'y': round(p.y, 3),
                'yaw': round(self._yaw_of(tf_base.transform.rotation), 3)}

        # 雷达点转到同一坐标系（抽稀：隔点取）
        try:
            tf = self._tf_buffer.lookup_transform(
                frame, self._scan.header.frame_id, Time())
        except Exception:
            return
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        tyaw = self._yaw_of(tf.transform.rotation)
        cos_t, sin_t = math.cos(tyaw), math.sin(tyaw)
        pts = []
        angle = self._scan.angle_min
        for i, r in enumerate(self._scan.ranges):
            if i % 2 == 0 and self._scan.range_min < r < self._scan.range_max:
                lx, ly = r * math.cos(angle), r * math.sin(angle)
                pts.append([round(tx + lx * cos_t - ly * sin_t, 3),
                            round(ty + lx * sin_t + ly * cos_t, 3)])
            angle += self._scan.angle_increment

        payload = json.dumps({'type': 'telemetry', 'pose': pose, 'scan': pts,
                              'speed': round(tw.linear.x, 3),
                              'turn': round(tw.angular.z, 3),
                              'nav': self._nav_status})
        if self._ws_loop is not None:
            self._ws_loop.call_soon_threadsafe(self._broadcast, payload)

    def _broadcast(self, payload):
        """只在 ws 线程的 event loop 里被调用"""
        for ws in list(self._ws_clients):
            asyncio.ensure_future(self._safe_send(ws, payload))

    async def _safe_send(self, ws, payload):
        try:
            await ws.send(payload)
        except Exception:
            self._ws_clients.discard(ws)

    def _on_detection_event(self, msg):
        """detector_node 的识别事件原样透传给所有浏览器（入库由各节点 HttpReporter 直报后端）"""
        if self._ws_loop is not None and self._ws_clients:
            self._ws_loop.call_soon_threadsafe(self._broadcast, msg.data)

    def request_capture(self, label='manual'):
        """网页手动抓拍"""
        self._pub_capture.publish(String(data=label))
        self.get_logger().info(f'抓拍请求: {label}')

    def _on_capture_done(self, msg):
        if self._ws_loop is not None and self._ws_clients:
            self._ws_loop.call_soon_threadsafe(self._broadcast, msg.data)


async def _handle_client(node, ws):
    peer = ws.remote_address
    node._ws_clients.add(ws)
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
            elif msg_type == 'initial_pose':
                node.set_initial_pose(
                    float(data.get('x', 0.0)),
                    float(data.get('y', 0.0)),
                    float(data.get('yaw', 0.0)))
            elif msg_type == 'nav_goal':
                node.send_nav_goal(
                    float(data.get('x', 0.0)),
                    float(data.get('y', 0.0)),
                    float(data.get('yaw', 0.0)))
            elif msg_type == 'nav_cancel':
                node.cancel_nav()
            elif msg_type == 'capture':
                node.request_capture(str(data.get('label', 'manual')))
    finally:
        node._ws_clients.discard(ws)
        node.get_logger().info(f'后台已断开: {peer}')


def _run_ws_server(node, port):
    async def _serve():
        node._ws_loop = asyncio.get_running_loop()
        # 兼容 websockets 9.x（handler 收 ws,path 两个参数）与新版（只收 ws）
        async with websockets.serve(
                lambda ws, *args: _handle_client(node, ws), '0.0.0.0', port):
            node.get_logger().info(f'WebSocket 监听 :{port}')
            await asyncio.Future()  # 常驻
    asyncio.run(_serve())


def main():
    config_path = os.environ.get('ROBOT_CONFIG')
    if not config_path:
        for cand in CONFIG_CANDIDATES:
            if cand.exists():
                config_path = str(cand)
                break
    if not config_path:
        raise FileNotFoundError(
            '未找到 config.yaml，请设置 ROBOT_CONFIG 环境变量，候选路径：'
            + ', '.join(str(c) for c in CONFIG_CANDIDATES))
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
