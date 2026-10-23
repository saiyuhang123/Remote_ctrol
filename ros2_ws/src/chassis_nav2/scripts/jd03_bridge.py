#!/usr/bin/env python3
"""JD03 Twist/Ecu adapter; jd03_odometry handles measured wheel odometry."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yunle_msgs.msg import Ecu, VehicleStatus
from std_msgs.msg import Bool
from chassis_nav2.geometry import ackermann_command, ackermann_command_4ws
from chassis_nav2.drive_control import GearInterlock


class JD03Bridge(Node):
    def __init__(self):
        super().__init__('jd03_bridge')
        defaults = {'wheel_base': 0.5, 'max_steering_deg': 27., 'max_speed': .3,
                    'cmd_timeout': .5, 'command_steering_sign': -1.,
                    'feedback_steering_sign': -1., 'enable_control': False,
                    'four_wheel_steering': False, 'rear_steering_sign': -1.,
                    'four_wheel_curvature_gain': 2., 'steering_rate_dps': 45.,
                    'max_reverse_speed': .3, 'feedback_speed_scale': 1.,
                    'steering_ready_tolerance_deg': 3.,
                    'gear_stop_speed': .03, 'gear_stop_duration': .25,
                    'gear_settle_duration': .20}
        for name, value in defaults.items():
            setattr(self, name, self.declare_parameter(name, value).value)
        if (self.wheel_base <= 0 or not 0 < self.max_steering_deg <= 27
                or self.rear_steering_sign not in (-1., 1.)):
            raise ValueError('JD03 wheel_base must be positive; steering must be within encoder limit 27 degrees')
        if self.steering_rate_dps <= 0:
            raise ValueError('steering_rate_dps 必须为正数（舵机转角速率上限，度/秒）')
        if any(not math.isfinite(v) or v<=0 for v in
               (self.max_speed, self.max_reverse_speed, self.feedback_speed_scale,
                self.four_wheel_curvature_gain, self.steering_ready_tolerance_deg)):
            raise ValueError('JD03 speed limits, feedback scale and curvature gain must be positive')
        self.gear_interlock = GearInterlock(self.gear_stop_speed, self.gear_stop_duration,
                                           self.gear_settle_duration)
        self.command = None
        self.feedback_time = None
        self.feedback_speed = 0.
        self.feedback_steer = 0.
        self.ecu_pub = None
        self.estopped = False
        self.last_steer = 0.
        self.last_rear = 0.
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
        limit = min(self.max_speed, self.max_reverse_speed) if v<0 else self.max_speed
        msg = Ecu()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.four_wheel_steering:
            motor, steer, rear, gear, brake = ackermann_command_4ws(
                v, w, self.wheel_base, self.max_steering_deg, limit,
                self.command_steering_sign, self.rear_steering_sign,
                self.four_wheel_curvature_gain)
        else:
            motor, steer, gear, brake = ackermann_command(
                v, w, self.wheel_base, self.max_steering_deg, limit, self.command_steering_sign)
            rear = 0.
        gear, drive = self.gear_interlock.update(gear if not brake else None,
                                                self.feedback_speed, now, active)
        if not drive:
            motor, brake = 0., True
        if (drive and abs(self.feedback_speed)<=self.gear_interlock.stop_speed and
                abs(steer-self.feedback_steer)>self.steering_ready_tolerance_deg):
            # Pre-steer while stopped. Short reversing arcs cannot tolerate
            # starting in the previous leg's wheel angle while the servo slews.
            motor, brake = 0., True
        if brake and (not active or abs(v)<.01):
            # A reverse stop must not select D or recenter the wheels between
            # controller frames. Hold steering until the next driving command.
            steer, rear = self.last_steer, self.last_rear
        # 转向速率限制：低速时 δ=atan(L·w/(gain·v)) 对 w 抖动极度敏感，
        # 控制器每帧跳动会导致舵机疯狂摆动；限速后转角平滑逼近目标
        max_delta = self.steering_rate_dps * 0.05  # 定时器周期 0.05s
        steer = self.last_steer + max(-max_delta, min(max_delta, steer - self.last_steer))
        rear = self.last_rear + max(-max_delta, min(max_delta, rear - self.last_rear))
        self.last_steer, self.last_rear = steer, rear
        msg.rear_steer = rear
        msg.rear_wheel_flag = self.four_wheel_steering
        msg.motor, msg.steer, msg.shift, msg.brake = motor, steer, gear, brake
        msg.set_torque = False
        self.ecu_pub.publish(msg)

    def feedback(self, msg):
        if math.isfinite(msg.cur_speed) and math.isfinite(msg.cur_steer):
            if self.feedback_time is None and msg.shift_level in (1, 3):
                self.gear_interlock.gear = msg.shift_level
            self.feedback_time = time.monotonic()
            self.feedback_speed = msg.cur_speed * self.feedback_speed_scale
            self.feedback_steer = msg.cur_steer


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
