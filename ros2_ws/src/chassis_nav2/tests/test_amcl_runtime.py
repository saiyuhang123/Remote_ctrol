"""Run real AMCL/map_server/Nav2 with synthetic scans and TF in ROS domain 214.

No sensor driver, chassis driver or control bridge is started; no navigation goal
is sent. Run after building and sourcing this workspace.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import yaml

os.environ['ROS_DOMAIN_ID'] = '214'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import numpy as np
from PIL import Image
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from ament_index_python.packages import get_package_share_directory


class AmclRuntimeTest(unittest.TestCase):
    def test_initial_pose_tracking_relocalization_and_nav2_activation(self):
        with tempfile.TemporaryDirectory(prefix='amcl-regression-') as directory:
            root = Path(directory)
            grid = np.zeros((140, 160), dtype=np.uint8)
            grid[:2, :] = grid[-2:, :] = 1
            grid[:, :2] = grid[:, -2:] = 1
            grid[15:55, 100:103] = 1
            grid[100:110, 25:65] = 1
            Image.fromarray(np.flipud(np.where(grid, 0, 254)).astype(np.uint8)).save(root/'map.pgm')
            (root/'map.yaml').write_text(
                'image: map.pgm\nresolution: 0.05\norigin: [-4.0, -3.5, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
            # Keep the real planner/plugins, with a smaller lookup cache for
            # this small synthetic map. Production navigation params stay intact.
            nav = Path(get_package_share_directory('chassis_nav2'))
            params = yaml.safe_load((nav/'config/nav2_2d_localization.yaml').read_text())
            params['planner_server']['ros__parameters']['GridBased']['lookup_table_size'] = 3.0
            (root/'nav2.yaml').write_text(yaml.safe_dump(params))
            with (root/'runtime.log').open('w+') as log:
                # Exercise the old CLI alias too; its map_file points nowhere.
                process = subprocess.Popen([
                    'ros2', 'launch', 'chassis_nav2', 'slam_2d_localization.launch.py',
                    'start_sensors:=false', 'start_chassis:=false', 'enable_control:=false',
                    'rviz:=false', 'map:=' + str(root/'map.yaml'),
                    'nav2_params:=' + str(root/'nav2.yaml'),
                    'map_file:=' + str(root/'not_a_posegraph'),
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                rclpy.init(args=[])
                node = rclpy.create_node('amcl_synthetic_fixture')
                scans = node.create_publisher(LaserScan, '/scan', rclpy.qos.qos_profile_sensor_data)
                initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
                broadcaster = TransformBroadcaster(node)
                buffer = Buffer()
                listener = TransformListener(buffer, node)
                poses = []
                subscription = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', poses.append, 10)
                odom = [0., 0., 0.]
                world = [.2, -.5, .2]
                angles = np.linspace(-2.7489, 2.7489, 361)
                distances = np.arange(.3, 15., .025)

                def publish():
                    stamp = node.get_clock().now().to_msg()
                    tf = TransformStamped()
                    tf.header.stamp = stamp
                    tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
                    tf.transform.translation.x, tf.transform.translation.y = odom[:2]
                    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(odom[2]/2), math.cos(odom[2]/2)
                    broadcaster.sendTransform(tf)
                    ix = np.floor((world[0] + np.cos(angles[:, None]+world[2])*distances + 4.)/.05).astype(int)
                    iy = np.floor((world[1] + np.sin(angles[:, None]+world[2])*distances + 3.5)/.05).astype(int)
                    outside = (ix < 0) | (ix >= 160) | (iy < 0) | (iy >= 140)
                    hits = outside | (grid[np.clip(iy, 0, 139), np.clip(ix, 0, 159)] != 0)
                    scan = LaserScan()
                    scan.header.stamp, scan.header.frame_id = stamp, 'base_link'
                    scan.angle_min, scan.angle_max = float(angles[0]), float(angles[-1])
                    scan.angle_increment = float(angles[1]-angles[0])
                    scan.scan_time, scan.range_min, scan.range_max = .1, .3, 15.
                    scan.ranges = np.where(hits.any(axis=1), distances[hits.argmax(axis=1)], 15.).astype(np.float32).tolist()
                    scans.publish(scan)

                timer = node.create_timer(.1, publish)

                def wait_for(condition, seconds=15):
                    deadline = time.monotonic() + seconds
                    while time.monotonic() < deadline:
                        if condition():
                            return
                        if process.poll() is not None:
                            raise AssertionError('Launch exited: ' + str(process.returncode))
                        rclpy.spin_once(node, timeout_sec=.05)
                    raise AssertionError('Timed out waiting for AMCL/Nav2')

                def active(name):
                    client = node.create_client(GetState, '/' + name + '/get_state')
                    try:
                        wait_for(client.service_is_ready)
                        state = None
                        deadline = time.monotonic() + 15
                        while time.monotonic() < deadline:
                            future = client.call_async(GetState.Request())
                            wait_for(future.done, 3)
                            state = future.result().current_state
                            if state.id == 3:
                                return
                            rclpy.spin_once(node, timeout_sec=.1)
                        self.fail(f'{name} did not activate: {state}')
                    finally:
                        node.destroy_client(client)

                def initialize():
                    poses.clear()
                    msg = PoseWithCovarianceStamped()
                    msg.header.frame_id = 'map'
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.pose.pose.position.x, msg.pose.pose.position.y = world[:2]
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(world[2]/2), math.cos(world[2]/2)
                    msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = .0025
                    initial.publish(msg)
                    wait_for(lambda: bool(poses) and math.hypot(
                        poses[-1].pose.pose.position.x-world[0],
                        poses[-1].pose.pose.position.y-world[1]) < .2)

                try:
                    active('map_server')
                    active('amcl')
                    self.assertFalse(buffer.can_transform('map', 'base_link', rclpy.time.Time()))
                    wait_for(lambda: initial.get_subscription_count() > 0)
                    initialize()
                    wait_for(lambda: buffer.can_transform('map', 'base_link', rclpy.time.Time()))
                    for name in ('controller_server', 'planner_server',
                                 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother'):
                        active(name)
                    for step in range(1, 13):
                        odom[0] = .05*step
                        world[0], world[1] = .2+odom[0]*math.cos(.2), -.5+odom[0]*math.sin(.2)
                        until = time.monotonic()+.12
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=.03)
                    wait_for(lambda: bool(poses) and math.hypot(
                        poses[-1].pose.pose.position.x-world[0], poses[-1].pose.pose.position.y-world[1]) < .2)
                    world[:] = [-1., .6, -.3]
                    initialize()
                    wait_for(lambda: abs(buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation.x+1.) < .25)
                    names = node.get_node_names()
                    self.assertIn('amcl', names)
                    self.assertNotIn('slam_toolbox', names)
                    self.assertNotIn('chassis_driver', names)
                    self.assertNotIn('jd03_bridge', names)
                    print('AMCL: map loaded; no TF before initialization; initial pose, '
                          '0.6 m simulated tracking, reinitialization and all Nav2 lifecycle nodes verified.')
                except Exception:
                    log.flush()
                    log.seek(0)
                    print(log.read()[-10000:])
                    raise
                finally:
                    node.destroy_timer(timer)
                    node.destroy_subscription(subscription)
                    del listener
                    node.destroy_node()
                    rclpy.shutdown()
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
