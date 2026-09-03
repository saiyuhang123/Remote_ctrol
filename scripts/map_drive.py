#!/usr/bin/env python3
"""建图巡航：按时间分段发 /cmd_vel，走"外圈矩形 + 左右双圆"路线覆盖场地。
用法：先启动 slam.launch.py（仿真+slam_toolbox），再 python3 scripts/map_drive.py
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (时长s, 线速度, 角速度)：外圈 -> 右圆 -> 左圆
ROUTE = [
    (14.0, 0.40, 0.0), (4.5, 0.35, 0.35),      # 东行，左转90
    (9.0, 0.40, 0.0), (4.5, 0.35, 0.35),       # 北行，左转90
    (14.0, 0.40, 0.0), (4.5, 0.35, 0.35),      # 西行，左转90
    (9.0, 0.40, 0.0), (4.5, 0.35, 0.35),       # 南行，左转90
    (7.0, 0.40, 0.0), (4.5, 0.35, 0.35),       # 回中段，左转90
    (4.5, 0.35, -0.35), (7.5, 0.40, 0.0),      # 右转90朝东，去右圆心
    (4.5, 0.35, -0.35), (36.0, 0.35, -0.175),  # 右转朝南，右圆 r≈2
    (6.0, 0.40, 0.0), (4.5, 0.35, 0.35),       # 直行，左转朝西
    (13.0, 0.40, 0.0), (4.5, 0.35, 0.35),      # 西行去左圆心，左转朝南
    (27.0, 0.35, 0.233),                       # 左圆 r≈1.5
]


def main():
    rclpy.init()
    node = Node('map_drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    time.sleep(1.0)
    for dur, v, w in ROUTE:
        t0 = time.time()
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        while time.time() - t0 < dur:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
    pub.publish(Twist())
    print('建图巡航完成')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
