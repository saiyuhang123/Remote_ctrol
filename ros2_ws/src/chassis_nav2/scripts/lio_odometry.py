#!/usr/bin/env python3
"""Publish real base_link odometry from FAST-LIO's IMU-body pose."""
import math
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from chassis_nav2.geometry import inverse, quaternion


class LioOdometry(Node):
    def __init__(self):
        super().__init__('lio_odometry')
        values = self.declare_parameter('base_from_body_matrix', np.eye(4).reshape(-1).tolist()).value
        self.body_from_base = inverse(np.asarray(values).reshape(4, 4))
        self.previous = None
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        topic = self.declare_parameter('odom_topic', '/odom').value
        comparison = self.declare_parameter('comparison_topic', '/odom_lio').value
        self.publisher = self.create_publisher(Odometry, topic, 10)
        self.comparison = self.create_publisher(Odometry, comparison, 10) if comparison and comparison != topic else None
        self.tf = TransformBroadcaster(self) if self.declare_parameter('publish_tf', True).value else None
        self.create_subscription(Odometry, '/Odometry', self.receive, 10)

    def receive(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.previous and stamp <= self.previous[0]:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        n = x*x+y*y+z*z+w*w
        if not math.isfinite(n) or n < 1e-12:
            return
        s = 2/n
        matrix = np.eye(4)
        matrix[:3, :3] = [[1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
                          [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
                          [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]]
        matrix[:3, 3] = [p.x, p.y, p.z]
        matrix = matrix @ self.body_from_base
        if not np.isfinite(matrix).all():
            return
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id, out.child_frame_id = self.odom_frame, 'base_link'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, matrix[:3, 3])
        q = quaternion(matrix)
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = q
        out.pose.covariance = msg.pose.covariance
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        if self.previous:
            old_stamp, old_matrix, old_yaw = self.previous
            dt = stamp - old_stamp
            if dt < 1.0:
                velocity = matrix[:3, :3].T @ ((matrix[:3, 3]-old_matrix[:3, 3])/dt)
                out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, velocity)
                out.twist.twist.angular.z = math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))/dt
        self.previous = stamp, matrix, yaw
        self.publisher.publish(out)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = out.header, out.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, matrix[:3, 3])
        tf.transform.rotation = out.pose.pose.orientation
        if self.tf:
            self.tf.sendTransform(tf)
        if self.comparison:
            comparison = deepcopy(out)
            comparison.header.frame_id = 'lio_odom'
            self.comparison.publish(comparison)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
