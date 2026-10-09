#!/usr/bin/env python3
# coding=utf-8
"""点云裁剪过滤节点：切除 base_link 系下固定盒子区域内的点（默认挖掉车尾升降台的自遮挡）。

输入 /cloud_registered_base (base_link 系) -> 输出 /cloud_registered_base_filtered。
与 Nav2 costmap 的 min_obstacle_height 等过滤正交：这里只做空间盒裁剪。
"""
import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class CloudCropBox(Node):
    _PF2NP = {
        PointField.INT8: np.int8, PointField.UINT8: np.uint8,
        PointField.INT16: np.int16, PointField.UINT16: np.uint16,
        PointField.INT32: np.int32, PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
    }

    def __init__(self):
        super().__init__('cloud_crop_box')
        # 负盒：盒子内的点被删除。默认覆盖车尾升降台（base_link 系，实测障碍点 x≈-0.65m）
        self.declare_parameter('min_x', -0.80)
        self.declare_parameter('max_x', -0.50)
        self.declare_parameter('min_y', -0.25)
        self.declare_parameter('max_y', 0.25)
        self.declare_parameter('min_z', -0.15)
        self.declare_parameter('max_z', 1.60)
        self.declare_parameter('negative', True)  # True=删盒内(去自遮挡); False=只保留盒内

        g = self.get_parameter
        self.min_x, self.max_x = g('min_x').value, g('max_x').value
        self.min_y, self.max_y = g('min_y').value, g('max_y').value
        self.min_z, self.max_z = g('min_z').value, g('max_z').value
        self.negative = g('negative').value

        self.pub = self.create_publisher(PointCloud2, '/cloud_registered_base_filtered', 10)
        self.create_subscription(
            PointCloud2, '/cloud_registered_base', self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'cloud_crop_box ready. negative={self.negative}, '
            f'x[{self.min_x},{self.max_x}] y[{self.min_y},{self.max_y}] z[{self.min_z},{self.max_z}]')

    def cb(self, msg: PointCloud2):
        names, formats, offsets = [], [], []
        for f in msg.fields:
            dt = self._PF2NP[f.datatype]
            formats.append((dt, (f.count,)) if f.count > 1 else dt)
            names.append(f.name)
            offsets.append(f.offset)
        dtype = np.dtype({'names': names, 'formats': formats,
                          'offsets': offsets, 'itemsize': msg.point_step})
        arr = np.frombuffer(msg.data, dtype=dtype)
        x, y, z = arr['x'], arr['y'], arr['z']

        inside = ((x >= self.min_x) & (x <= self.max_x) &
                  (y >= self.min_y) & (y <= self.max_y) &
                  (z >= self.min_z) & (z <= self.max_z))
        keep = inside if not self.negative else ~inside
        # 顺手清掉 NaN/Inf
        keep &= np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        out = arr[keep]
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width = 1, out.shape[0]
        cloud.fields = msg.fields
        cloud.is_bigendian = msg.is_bigendian
        cloud.point_step = msg.point_step
        cloud.row_step = msg.point_step * out.shape[0]
        cloud.data = array.array('B', np.ascontiguousarray(out).tobytes())
        cloud.is_dense = True
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = CloudCropBox()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
