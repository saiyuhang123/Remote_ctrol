#!/usr/bin/env python3

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
        self.declare_parameter('map_voxel_size', 0.1)
        path = self.get_parameter('map_file_path').value
        interval = self.get_parameter('interval').value
        voxel = self.get_parameter('map_voxel_size').value

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
        self.create_timer(interval, self.publish_map)
        self.get_logger().info(f'Interval for publishing map: {interval} seconds')
        self.get_logger().info('Map Publisher Node Initialized')

    def publish_map(self):
        if self.map_points is None or len(self.map_points) == 0:
            self.get_logger().warn('Global map is not loaded; skipping publish')
            return
        # 直接用 numpy 连续内存打包，避免 tolist() 逐个 Python 对象转换
        cloud = PointCloud2()
        cloud.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
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
        cloud.data = self.map_points.tobytes()
        self.pub_map.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
