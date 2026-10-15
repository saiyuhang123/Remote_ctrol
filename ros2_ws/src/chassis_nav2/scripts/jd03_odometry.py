#!/usr/bin/env python3
"""Receive real JD03 feedback; publish wheel odometry without sending commands."""
import math
from copy import deepcopy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from yunle_msgs.msg import VehicleStatus
from chassis_nav2.wheel_odometry import WheelOdometry


def covariance6(planar):
    values = [0.]*36
    for i in (2, 3, 4):
        values[i*6+i] = 1e3
    for i, ros_i in enumerate((0, 1, 5)):
        for j, ros_j in enumerate((0, 1, 5)):
            values[ros_i*6+ros_j] = float(planar[i, j])
    return values


class JD03Odometry(Node):
    def __init__(self):
        super().__init__('jd03_odometry')
        defaults = {'wheel_base': 0., 'rear_axle_translation': [0., 0.],
                    'speed_scale': 1., 'steering_scale': 1., 'steering_bias_deg': 0.,
                    'feedback_steering_sign': -1., 'max_feedback_gap': .5,
                    'speed_stddev': .05, 'steering_stddev_deg': 1.,
                    'lateral_velocity_stddev': .02, 'min_yaw_rate_stddev': .01,
                    'stamp_source': 'receipt',
                    'feedback_topic': '/vehicle_status', 'odom_topic': '/odom_wheel',
                    'odom_frame': 'wheel_odom', 'base_frame': 'base_link',
                    'comparison_topic': '', 'publish_tf': False,
                    'four_wheel_steering': False, 'four_wheel_yaw_gain': 2.}
        for name, value in defaults.items():
            setattr(self, name, self.declare_parameter(name, value).value)
        numeric = (self.wheel_base, self.speed_scale, self.steering_scale,
                   self.feedback_steering_sign, self.max_feedback_gap, self.speed_stddev,
                   self.steering_stddev_deg, self.lateral_velocity_stddev,
                   self.min_yaw_rate_stddev)
        if (not all(math.isfinite(x) and x > 0 for x in numeric[:3]+numeric[4:])
                or self.feedback_steering_sign not in (-1., 1.)
                or not math.isfinite(self.steering_bias_deg)
                or len(self.rear_axle_translation) != 2
                or not all(math.isfinite(x) for x in self.rear_axle_translation)
                or self.stamp_source not in ('receipt', 'header')):
            raise ValueError('Invalid JD03 odometry geometry, scales, noise or timestamp source')
        self.integrator = WheelOdometry(self.wheel_base, self.rear_axle_translation,
            self.speed_stddev, self.steering_stddev_deg,
            self.lateral_velocity_stddev, self.max_feedback_gap,
            four_wheel_steering=self.four_wheel_steering,
            four_wheel_yaw_gain=self.four_wheel_yaw_gain,
            min_yaw_rate_stddev=self.min_yaw_rate_stddev)
        self.publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.comparison = (self.create_publisher(Odometry, self.comparison_topic, 10)
                           if self.comparison_topic and self.comparison_topic != self.odom_topic else None)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(VehicleStatus, self.feedback_topic, self.receive, 10)
        self.get_logger().info(f'JD03 measured wheel odometry -> {self.odom_topic}; TF={self.publish_tf}')

    def receive(self, feedback):
        stamp = feedback.header.stamp if self.stamp_source == 'header' else self.get_clock().now().to_msg()
        if self.stamp_source == 'header' and stamp.sec == 0 and stamp.nanosec == 0:
            self.get_logger().warning('Feedback header has no timestamp; use stamp_source=receipt')
            return
        steering = math.radians(self.feedback_steering_sign * (
            feedback.cur_steer*self.steering_scale+self.steering_bias_deg))
        sample = self.integrator.update(stamp.sec+stamp.nanosec*1e-9,
            feedback.cur_speed*self.speed_scale, steering, feedback.shift_level)
        if sample is None:
            return
        pose, velocity, pose_cov, twist_cov, gap = sample
        if gap:
            self.get_logger().warning('JD03 feedback gap: skipped missing motion and increased pose uncertainty')
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, self.odom_frame, self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, pose[:2])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = map(float, velocity)
        msg.pose.covariance, msg.twist.covariance = covariance6(pose_cov), covariance6(twist_cov)
        self.publisher.publish(msg)
        if self.tf:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = msg.header, self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = map(float, pose[:2])
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(tf)
        if self.comparison:
            comparison = deepcopy(msg)
            comparison.header.frame_id = 'wheel_odom'
            self.comparison.publish(comparison)


def main(args=None):
    rclpy.init(args=args)
    node = JD03Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
