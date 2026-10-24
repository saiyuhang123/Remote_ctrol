#!/usr/bin/env python3
"""Offline NDT -> CSV recorder -> Pure Pursuit integration on loopback/domain 87.

Requires built msf_localizer/fixed_route_navigation and a sourced ROS workspace.
Uses synthetic points and odometry; never launches devices or publishes /ecu.
"""
import math
import os
import random
import signal
import subprocess
import tempfile
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/ndt_fixed_route_verification_logs')

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from fixed_route_navigation.node import FixedRoute
from fixed_route_navigation.route import load_csv


class SyntheticRobot(Node):
    def __init__(self, points):
        super().__init__('synthetic_ndt_port_robot')
        self.points = points
        self.x = self.y = self.yaw = 0.
        self.origin = np.eye(4)
        angle = .15
        self.origin[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        self.origin[:3, 3] = [1.2, -.7, 0.]
        self.base_lidar = np.eye(4)
        angle = .27
        self.base_lidar[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        self.base_lidar[:3, 3] = [.32, 0., .46]
        self.manual_speed = 0.
        self.automatic = self.obstacle = False
        self.send_scan = self.send_cloud = True
        self.invalid_scan = False
        self.command = (0., 0., 0.)
        self.commands, self.health = [], []
        self.map_received = False
        self.last_time, self.iteration = time.monotonic(), 0
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/velodyne_points', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.receive_command, 10)
        self.create_subscription(Bool, '/localization/healthy', lambda msg: self.health.append(msg.data), 1)
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'velodyne'
        transform.transform.translation.x, transform.transform.translation.z = .32, .46
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(.27/2), math.cos(.27/2)
        self.static.sendTransform(transform)
        self.create_timer(.05, self.publish)

    def receive_command(self, msg):
        self.command = (msg.linear.x, msg.angular.z, time.monotonic())
        self.commands.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def map_base(self):
        odom = np.eye(4)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        odom[:2, :2] = [[c, -s], [s, c]]
        odom[:3, 3] = [self.x, self.y, 0.]
        return self.origin@odom

    def initial_pose(self):
        matrix = self.map_base()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(matrix[0, 3]+.12), float(matrix[1, 3]-.08)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])+.03
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.initial_pub.publish(msg)

    def subscribe_late_to_map(self):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, '/map_cloud', lambda msg: setattr(self, 'map_received', msg.width > 0), qos)

    def publish(self):
        now = time.monotonic()
        dt, self.last_time = min(.1, now-self.last_time), now
        velocity, angular = (self.command[:2] if now-self.command[2] < .3 else (0., 0.)) if self.automatic else (self.manual_speed, 0.)
        self.x += velocity*math.cos(self.yaw)*dt
        self.y += velocity*math.sin(self.yaw)*dt
        self.yaw += angular*dt
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp, odom.child_frame_id = 'odom', stamp, 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = velocity, angular
        self.odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.dynamic.sendTransform(tf)
        if self.send_scan:
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = 'base_link', stamp
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
            scan.range_min, scan.range_max, scan.scan_time = .1, 15., .05
            scan.ranges = [math.inf]*361
            if self.obstacle:
                scan.ranges[180] = .65
            if self.invalid_scan:
                scan.ranges[180] = math.nan
            self.scan_pub.publish(scan)
        if self.send_cloud and self.iteration % 4 == 0:
            lidar_map = np.linalg.inv(self.map_base()@self.base_lidar)
            points = self.points@lidar_map[:3, :3].T+lidar_map[:3, 3]
            self.cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='velodyne'), points))
        self.iteration += 1


def main():
    rng = random.Random(20261024)
    points = []
    for _ in range(130):
        center = [rng.uniform(-5, 6), rng.uniform(-5, 5), rng.uniform(-.5, 3)]
        for _ in range(45):
            points.append([center[0]+rng.gauss(0, .16), center[1]+rng.gauss(0, .12), center[2]+rng.gauss(0, .1)])
    points = np.asarray(points)
    with tempfile.TemporaryDirectory(prefix='ndt_port_verify_') as directory:
        directory = Path(directory)
        mapfile = directory/'map.pcd'
        with mapfile.open('w') as stream:
            stream.write(f'# .PCD v0.7\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
            np.savetxt(stream, points, fmt='%.6f')
        logfile = (directory/'ndt.log').open('w+')
        executable = Path(get_package_prefix('msf_localizer'))/'lib/msf_localizer/multi_sensor_fusion'
        process = subprocess.Popen([str(executable), '--ros-args', '-p', f'pcd_map:={mapfile}',
                                    '-p', 'scan_leaf_size:=0.15', '-p', 'map_leaf_size:=0.1',
                                    '-p', 'localization_timeout:=1.0', '-p', 'max_fitness_score:=0.15'],
                                   stdout=logfile, stderr=subprocess.STDOUT)
        rclpy.init()
        robot, route, executor = None, None, SingleThreadedExecutor()
        try:
            robot = SyntheticRobot(points)
            route = FixedRoute(parameter_overrides=[Parameter('route_file', value=str(directory/'route.csv')),
                              Parameter('obstacle_clear_time', value=.2)])
            executor.add_node(robot); executor.add_node(route)

            def spin(duration):
                deadline = time.monotonic()+duration
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'NDT process exited'
                    executor.spin_once(timeout_sec=.01)

            def until(condition, timeout, label):
                deadline = time.monotonic()+timeout
                while time.monotonic() < deadline:
                    spin(.02)
                    if condition():
                        print('PASS:', label, flush=True)
                        return
                raise AssertionError(f'{label}; state={route.state}; reason={route.reason}')

            def service(name, expected=True):
                client = robot.create_client(Trigger, '/fixed_route/'+name)
                until(lambda: client.service_is_ready(), 2., name+' service discovery')
                future = client.call_async(Trigger.Request())
                until(future.done, 2., name+' service response')
                result = future.result()
                robot.destroy_client(client)
                assert result.success == expected, f'{name}: {result.message}'
                return result

            spin(.6)
            assert route.state == 'IDLE' and not route.healthy and not robot.commands
            service('start', expected=False)
            print('PASS: no motion before initialization or an explicit start', flush=True)
            robot.subscribe_late_to_map()
            until(lambda: robot.map_received, 3., 'late subscriber receives transient map')
            robot.initial_pose()
            until(lambda: route.healthy and route.pose is not None, 8., 'NDT converges using sensor extrinsic and odometry')
            expected = robot.map_base()
            error = math.hypot(route.pose.x-expected[0, 3], route.pose.y-expected[1, 3])
            assert error < .12, f'NDT position error {error:.3f}m'
            print(f'PASS: NDT position error {error:.4f}m', flush=True)
            service('start_recording')
            robot.manual_speed = .15
            spin(4.0)
            robot.manual_speed = 0.
            spin(.3)
            service('stop_recording')
            assert len(load_csv(directory/'route.csv')) >= 8
            assert not (directory/'route.csv.partial').exists()
            service('start_recording', expected=False)
            service('start', expected=False)
            print('PASS: CSV recording is finalized atomically; existing routes and wrong starts are rejected', flush=True)
            robot.x, robot.y, robot.yaw = 0., 0., 0.
            robot.initial_pose()
            until(lambda: route.healthy and route.pose is not None and abs(route.pose.x-1.2) < .12, 8., 'reinitialize at recorded start')
            spin(.3)
            robot.automatic = True
            service('start')
            until(lambda: robot.command[0] > .05, 3., 'explicit start produces bounded forward commands')
            robot.obstacle = True
            until(lambda: robot.command[0] == 0. and '障碍' in route.reason, 2., 'full footprint collision stops the vehicle')
            spin(.25)
            assert robot.command[0] == 0.
            robot.obstacle = False
            until(lambda: robot.command[0] > .05, 3., 'clear original route resumes after obstacle wait')
            until(lambda: route.state == 'COMPLETED', 8., 'fixed-route tracking reaches endpoint and confirms stopped odometry')
            assert abs(robot.x-.6) < .15
            spin(.1)
            assert robot.command[0] == 0.
            robot.automatic = False
            robot.x, robot.y, robot.yaw = 0., 0., 0.
            robot.initial_pose()
            until(lambda: route.healthy and route.pose is not None and abs(route.pose.x-1.2) < .12, 8., 'reinitialize for fault checks')
            spin(.3)
            service('start')
            robot.automatic = True
            robot.send_scan = False
            until(lambda: route.state == 'PAUSED' and '扫描过期' in route.reason, 2., 'stale scan pauses and stops')
            robot.send_scan = True
            spin(.7)
            assert route.state == 'PAUSED' and robot.command[0] == 0.
            service('resume')
            until(lambda: robot.command[0] > .03, 3., 'resume requires explicit service call')
            robot.invalid_scan = True
            until(lambda: route.state == 'PAUSED' and '无效扫描' in route.reason, 2., 'invalid measurements in travel direction pause and stop')
            robot.invalid_scan = False
            spin(.7)
            service('resume')
            robot.send_cloud = False
            until(lambda: route.state == 'PAUSED' and 'NDT' in route.reason, 3., 'NDT match timeout pauses and stops')
            robot.send_cloud = True
            until(lambda: route.healthy, 5., 'NDT recovers on fresh clouds')
            spin(.3)
            service('resume')
            teleop = Twist(); teleop.linear.x = .1; robot.teleop_pub.publish(teleop)
            until(lambda: route.state == 'PAUSED' and '手动' in route.reason, 2., 'teleoperation pauses route instead of silently advancing')
            service('stop')
            assert route.state == 'IDLE'
            print('All NDT + fixed route ROS 2 integration checks passed.', flush=True)
        except BaseException:
            logfile.flush(); logfile.seek(0)
            print('NDT process log:\n'+logfile.read(), flush=True)
            raise
        finally:
            if route is not None:
                if route.record_stream:
                    route.record_stream.close()
                route.destroy_node()
            if robot is not None:
                robot.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            logfile.close()


if __name__ == '__main__':
    main()
