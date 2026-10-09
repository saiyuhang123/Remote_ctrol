#!/usr/bin/env python3
# coding=utf-8
"""把 /cloud_registered 从 FAST-LIO 世界系(W_L) 变换到真实 base_link 后重发。

背景: /cloud_registered 的 frame_id 写作 "odom"，但它实际在 LIO 自己的世界系里；
      TF 里的 odom 是 EKF 的 odom 系，两者随里程分歧 → costmap 障碍位置漂移（P0-2）。
      本节点不查 TF，直接用 /Odometry（LIO 自身位姿）求逆后转到 base_link，
      base_from_body_* 参数用于补偿 LIO body 到 base_link 的已标定外参。
"""
import array
from collections import OrderedDict
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

def quat_to_mat(x, y, z, w, tx, ty, tz):
    """四元数+平移 → 4x4（逐元素运算，不触发 BLAS，与 transform_fusion 风格一致）"""
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0.0 else 0.0
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([[1.0 - (yy + zz), xy - wz, xz + wy, tx],
                     [xy + wz, 1.0 - (xx + zz), yz - wx, ty],
                     [xz - wy, yz + wx, 1.0 - (xx + yy), tz],
                     [0.0, 0.0, 0.0, 1.0]])


class CloudToBaseLink(Node):
    def __init__(self):
        super().__init__('cloud_to_base_link')
        # 按当前 /Odometry 的 base_link 约定默认单位变换；禁止沿用 MID360 安装偏移。
        self.declare_parameter('base_from_body_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('base_from_body_quaternion', [0.0, 0.0, 0.0, 1.0])
        translation = self.get_parameter('base_from_body_translation').value
        rotation = self.get_parameter('base_from_body_quaternion').value
        self.base_from_body = quat_to_mat(*rotation, *translation)
        self._odoms = OrderedDict()
        self._pending_clouds = OrderedDict()
        self.pub = self.create_publisher(PointCloud2, '/cloud_registered_base', 1)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 1)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, 1)
        self.get_logger().info('cloud_to_base_link ready')

    def on_odom(self, msg):
        stamp = self._stamp_ns(msg)
        self._odoms[stamp] = msg
        if stamp in self._pending_clouds:
            self._publish_matched_cloud(self._pending_clouds.pop(stamp), msg)
        while len(self._odoms) > 20:
            self._odoms.popitem(last=False)

    def on_cloud(self, msg):
        stamp = self._stamp_ns(msg)
        if stamp in self._odoms:
            self._publish_matched_cloud(msg, self._odoms[stamp])
        else:
            self._pending_clouds[stamp] = msg
            while len(self._pending_clouds) > 5:
                self._pending_clouds.popitem(last=False)

    @staticmethod
    def _stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _publish_matched_cloud(self, msg, odom):
        offs = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offs.keys():
            return

        o = odom.pose.pose
        T_wl_body = quat_to_mat(o.orientation.x, o.orientation.y, o.orientation.z,
                                o.orientation.w, o.position.x, o.position.y, o.position.z)
        R, t = T_wl_body[:3, :3], T_wl_body[:3, 3]
        # T(base<-W_L) = T(base<-body) @ inv(T(W_L<-body))
        R_T = self.base_from_body[:3, :3] @ R.T
        t_T = self.base_from_body[:3, 3] - R_T @ t

        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': [np.float32] * 3,
                          'offsets': [offs['x'], offs['y'], offs['z']],
                          'itemsize': msg.point_step})
        buf = array.array('B', msg.data)          # 一次 C 级拷贝
        xyz = np.frombuffer(buf, dtype=dtype)     # array.array 可写 → 原地改 x/y/z
        if xyz.size == 0:
            return
        p = np.stack([xyz['x'], xyz['y'], xyz['z']], axis=-1).astype(np.float64)
        p = np.einsum('ij,kj->ki', R_T, p) + t_T  # 等价 p @ R_T.T + t_T，不触发 BLAS
        xyz['x'], xyz['y'], xyz['z'] = p[:, 0], p[:, 1], p[:, 2]

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'         # ← 关键：不再叫 odom
        out.height, out.width = msg.height, msg.width
        out.fields, out.is_bigendian = msg.fields, msg.is_bigendian
        out.point_step, out.row_step, out.is_dense = msg.point_step, msg.row_step, msg.is_dense
        out.data = buf
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToBaseLink()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
