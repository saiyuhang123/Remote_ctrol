#!/usr/bin/env python3
"""从 /velodyne_points 抠掉车身固定物（摄像头支柱），重发 /velodyne_points_cropped 给 /scan 链路。

背景：Nav2 的 /scan 由原始点云直接转换（pointcloud_to_laserscan），不经过 FAST-LIO 的
blind 过滤；车上的摄像头支柱（实测位于 base_link 系 x≈-0.16, y≈+0.34，距雷达仅 0.36m）
会被当成永远贴着车的幽灵障碍堵死局部代价地图。LIO 建图/定位有 1.5m 盲区不受影响，
所以只在 /scan 链路入口裁剪，LIO 仍用原始话题。

裁剪盒在 base_link 系定义（直观）；代码内按 hardware.yaml 的外参平移到 velodyne 系
（velodyne = base_link - (0.32, 0, 0.46)，旋转为单位阵，盒子平移后仍轴对齐）。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

# 与 hardware.yaml sensor_calibration.base_from_lidar 一致；改安装位置要同步改这里
BASE_FROM_LIDAR = np.array([0.32, 0.0, 0.46])


class CloudCropBox(Node):
    def __init__(self):
        super().__init__('cloud_crop_box')
        # base_link 系裁剪盒。实测支柱点簇 x∈[-0.21,-0.09], y∈[+0.30,+0.39]，加 6cm 余量
        self.declare_parameter('crop_min', [-0.27, 0.24, -0.5])   # base_link 系 xmin, ymin, zmin
        self.declare_parameter('crop_max', [-0.03, 0.45, 2.0])    # base_link 系 xmax, ymax, zmax
        bmin = np.array(self.get_parameter('crop_min').value, dtype=float)
        bmax = np.array(self.get_parameter('crop_max').value, dtype=float)
        if bmin.shape != (3,) or bmax.shape != (3,) or not (np.all(bmin < bmax) and np.all(np.isfinite(bmin)) and np.all(np.isfinite(bmax))):
            raise ValueError('crop_min/crop_max 必须为 3 个有限数值且 min < max')
        # 转到 velodyne 系（旋转为单位阵，只减平移）
        self.vmin = bmin - BASE_FROM_LIDAR
        self.vmax = bmax - BASE_FROM_LIDAR
        self.pub = self.create_publisher(PointCloud2, '/velodyne_points_cropped', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/velodyne_points', self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'cloud_crop_box 就绪：base_link 盒 {bmin.tolist()} ~ {bmax.tolist()}，'
            f'/velodyne_points -> /velodyne_points_cropped')

    def cb(self, msg):
        offs = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offs.keys():
            return
        n = msg.width * msg.height
        if n == 0:
            return
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [np.float32] * 3,
                       'offsets': [offs['x'], offs['y'], offs['z']], 'itemsize': msg.point_step})
        pts = np.frombuffer(msg.data, dtype=dt, count=n)
        x, y, z = pts['x'], pts['y'], pts['z']
        inside = ((x >= self.vmin[0]) & (x <= self.vmax[0]) &
                  (y >= self.vmin[1]) & (y <= self.vmax[1]) &
                  (z >= self.vmin[2]) & (z <= self.vmax[2]))
        keep = np.flatnonzero(~inside)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(len(keep))
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        out.is_dense = True
        out.data = raw[keep].tobytes()
        self.pub.publish(out)


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
