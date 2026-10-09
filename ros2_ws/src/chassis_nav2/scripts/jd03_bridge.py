#!/usr/bin/env python3
"""JD03 Twist/Ecu adapter; jd03_odometry handles measured wheel odometry."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yunle_msgs.msg import Ecu, VehicleStatus
from std_msgs.msg import Bool
from chassis_nav2.geometry import ackermann_command


class JD03Bridge(Node):
    def __init__(self):
        super().__init__('jd03_bridge')
        defaults = {'wheel_base': 0.5, 'max_steering_deg': 27., 'max_speed': .3,
                    'cmd_timeout': .5, 'command_steering_sign': -1.,
                    'feedback_steering_sign': -1., 'enable_control': False}
        for name, value in defaults.items():
            setattr(self, name, self.declare_parameter(name, value).value)
        if self.wheel_base <= 0 or not 0 < self.max_steering_deg <= 27:
            raise ValueError('JD03 wheel_base must be positive; steering must be within encoder limit 27 degrees')
        self.command = None
        self.feedback_time = None
        self.ecu_pub = None
        self.estopped = False
        self.create_subscription(VehicleStatus, '/vehicle_status', self.feedback, 10)
        if self.enable_control:
            self.ecu_pub = self.create_publisher(Ecu, '/ecu', 10)
            self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
            self.create_subscription(Bool, '/emergency_stop', self.emergency_stop, 10)
            self.create_timer(.05, self.send)

    def emergency_stop(self, msg):
        self.estopped = msg.data
        if self.estopped:
            self.send()

    def receive(self, msg):
        self.command = time.monotonic(), msg.linear.x, msg.angular.z

    def send(self):
        now = time.monotonic()
        active = (not self.estopped and self.command is not None and now-self.command[0] <= self.cmd_timeout
                  and self.feedback_time is not None and now-self.feedback_time <= .5)
        v, w = self.command[1:] if active else (0., 0.)
        motor, steer, gear, brake = ackermann_command(
            v, w, self.wheel_base, self.max_steering_deg, self.max_speed, self.command_steering_sign)
        msg = Ecu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.motor, msg.steer, msg.shift, msg.brake = motor, steer, gear, brake
        msg.rear_wheel_flag, msg.set_torque = False, False
        self.ecu_pub.publish(msg)

    def feedback(self, msg):
        if math.isfinite(msg.cur_speed) and math.isfinite(msg.cur_steer):
            self.feedback_time = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = JD03Bridge()
    try:
        rclpy.spin(node)
    finally:
        if node.ecu_pub is not None:
            node.command = None
            node.send()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
