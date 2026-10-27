#!/usr/bin/env python3

import array
import math

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_file_path', '')
        self.declare_parameter('interval', 5)
        self.declare_parameter('publish_once', True)
        self.declare_parameter('map_voxel_size', 0.1)
        path = self.get_parameter('map_file_path').value
        interval = self.get_parameter('interval').value
        voxel = self.get_parameter('map_voxel_size').value
        publish_once = self.get_parameter('publish_once').value
        if not math.isfinite(voxel) or voxel <= 0:
            raise ValueError('map_voxel_size must be finite and positive')
        if not publish_once and (not math.isfinite(interval) or interval <= 0):
            raise ValueError('interval must be positive when publish_once=false')

        self.map_points = None
        if path:
            try:
                pcd = o3d.io.read_point_cloud(path)
                if pcd.is_empty():
                    raise ValueError('PCD map contains no readable points')
                raw = len(pcd.points)
                # 与 global_localization 的 map_voxel_size 一致，定位和显示都用降采样后的地图
                pcd = pcd.voxel_down_sample(voxel)
                self.map_points = np.ascontiguousarray(np.asarray(pcd.points), dtype=np.float32)
                self.get_logger().info(f'Loaded map from: {path} ({raw} -> {len(self.map_points)} points, voxel={voxel})')
            except Exception as e:
                self.get_logger().error(f'Failed to load PCD: {e}')
                raise
        else:
            self.get_logger().warn('No map_file_path provided; map not loaded')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub_map = self.create_publisher(PointCloud2, '/global_map', qos)
        # Build and serialize the static map once. TRANSIENT_LOCAL retains
        # this sample for RViz and localization nodes that join or restart later.
        # Keep this publisher alive; no periodic application-level publish is
        # required while the map is unchanged.
        self._map_message = self.build_map_message()
        self.publish_map()
        self._publish_timer = None
        if not publish_once:
            self._publish_timer = self.create_timer(interval, self.publish_map)
            self.get_logger().info(f'Cached map republish interval: {interval} seconds')
        else:
            self.get_logger().info('Static map published once; retained for late subscribers')
        self.get_logger().info('Map Publisher Node Initialized')

    def build_map_message(self):
        if self.map_points is None or len(self.map_points) == 0:
            return None
        cloud = PointCloud2()
        # A static map has no acquisition time. Zero requests the latest TF
        # when an RViz display is enabled long after the map was loaded.
        cloud.header = Header(frame_id='map')
        cloud.height = 1
        cloud.width = int(self.map_points.shape[0])
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.is_dense = True
        # array('B') uses the generated ROS message's buffer fast path,
        # avoiding per-byte Python validation of a large bytes object.
        cloud.data = array.array('B', self.map_points.tobytes())
        return cloud

    def publish_map(self):
        if self._map_message is not None:
            self.pub_map.publish(self._map_message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapPublisherNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
