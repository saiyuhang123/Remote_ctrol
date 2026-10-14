#!/usr/bin/env python3
# coding=utf-8

import copy
import threading
import time

import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros

from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        self.declare_parameter('freq_pub_localization', 50.0)
        self.FREQ_PUB_LOCALIZATION = self.get_parameter('freq_pub_localization').value
        translation = self.declare_parameter('base_from_body_translation', [0., 0., 0.]).value
        rotation = self.declare_parameter('base_from_body_quaternion', [0., 0., 0., 1.]).value
        self.base_from_body = self._quat_to_mat(*rotation, *translation)
        self.broadcast_lio_frame = self.declare_parameter('broadcast_lio_frame', False).value

        self.lock = threading.Lock()
        self.cur_odom_fastlio = None        # FastLIO /Odometry: odom_FL -> body
        self.cur_map_to_odom_fl = None      # ICP 结果: map -> odom_FL

        # tf buffer 用于获取 EKF 发布的 odom->base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_tf_warn = 0.0   # warn 节流用 (monotonic 秒)

        # 구독자
        self.create_subscription(Odometry, '/Odometry', self.cb_save_fastlio_odom, 1)
        self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, 1)

        # 퍼블리셔, 브로드캐스터
        self.pub_localization = self.create_publisher(Odometry, '/localization', 1)
        self.tf_broadcaster    = tf2_ros.TransformBroadcaster(self)

        # 주기 타이머
        period = 1.0 / self.FREQ_PUB_LOCALIZATION
        self.create_timer(period, self.timer_callback)
        self.get_logger().info('Transform Fusion Node Initialized')

    def cb_save_fastlio_odom(self, msg: Odometry):
        with self.lock:
            self.cur_odom_fastlio = msg

    def cb_save_map_to_odom(self, msg: Odometry):
        with self.lock:
            self.cur_map_to_odom_fl = msg

    # ─── 手写小矩阵运算 ─────────────────────────────────────────
    # tf_transformations 的 *_from_matrix 走 decompose→np.linalg.inv,
    # 高频小矩阵 BLAS 调用会让 OpenBLAS worker 线程在 sched_yield 上自旋烧核,
    # 这里全部用纯逐元素 numpy 运算替代 (不触发 BLAS 线程)
    @staticmethod
    def _quat_to_mat(x, y, z, w, tx, ty, tz):
        """四元数+平移 → 4x4 齐次矩阵"""
        n = x*x + y*y + z*z + w*w
        s = 2.0 / n if n > 0.0 else 0.0
        xx, yy, zz = x*x*s, y*y*s, z*z*s
        xy, xz, yz = x*y*s, x*z*s, y*z*s
        wx, wy, wz = w*x*s, w*y*s, w*z*s
        return np.array([
            [1.0-(yy+zz),  xy-wz,        xz+wy,        tx],
            [xy+wz,        1.0-(xx+zz),  yz-wx,        ty],
            [xz-wy,        yz+wx,        1.0-(xx+yy),  tz],
            [0.0,          0.0,          0.0,          1.0],
        ])

    @staticmethod
    def _mat_to_quat(M):
        """4x4 齐次矩阵 → (x,y,z,w), trace 分支法"""
        R = M[:3, :3]
        t = R[0,0] + R[1,1] + R[2,2]
        if t > 0.0:
            s = np.sqrt(t + 1.0) * 2.0
            return np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s,
                             (R[1,0]-R[0,1])/s, 0.25*s])
        i = int(np.argmax((R[0,0], R[1,1], R[2,2])))
        if i == 0:
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            return np.array([0.25*s, (R[0,1]+R[1,0])/s,
                             (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
        if i == 1:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            return np.array([(R[0,1]+R[1,0])/s, 0.25*s,
                             (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
        s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
        return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s,
                         0.25*s, (R[1,0]-R[0,1])/s])

    @staticmethod
    def _inv_mat(T):
        """齐次矩阵求逆: [R t;0 1]^-1 = [R.T -R.T·t;0 1]"""
        R = T[:3, :3]
        t = T[:3, 3]
        Ti = np.eye(4)
        Ti[:3, :3] = R.T
        Ti[:3, 3]  = -(R * t[:, None]).sum(axis=0)   # = -R.T @ t
        return Ti

    @staticmethod
    def _mat_mul(A, B):
        """einsum 走 C 循环, 不经过 BLAS"""
        return np.einsum('ij,jk->ik', A, B)

    def pose_to_mat(self, odom_msg: Odometry) -> np.ndarray:
        t = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        return self._quat_to_mat(q.x, q.y, q.z, q.w, t.x, t.y, t.z)

    def tf_to_mat(self, tf_msg: TransformStamped) -> np.ndarray:
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        return self._quat_to_mat(q.x, q.y, q.z, q.w, t.x, t.y, t.z)

    def timer_callback(self):
        with self.lock:
            fastlio_odom = copy.deepcopy(self.cur_odom_fastlio)
            map2odom_fl  = copy.deepcopy(self.cur_map_to_odom_fl)
        if fastlio_odom is None or map2odom_fl is None:
            return

        # FastLIO 的全局位姿: map -> body
        T_map_to_odom_fl   = self.pose_to_mat(map2odom_fl)
        T_odom_fl_to_body  = self.pose_to_mat(fastlio_odom)
        T_map_to_body      = self._mat_mul(T_map_to_odom_fl, T_odom_fl_to_body)
        T_map_to_base = self._mat_mul(T_map_to_body, self._inv_mat(self.base_from_body))

        # 与 LIO 的采样时刻对齐，避免转弯时混用历史 LIO 和最新 EKF 位姿。
        T_odom_ekf_to_body = None
        try:
            ekf_tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time.from_msg(fastlio_odom.header.stamp))
            T_odom_ekf_to_body = self.tf_to_mat(ekf_tf)
        except tf2_ros.ExtrapolationException:
            # 雷达时钟略快于系统时钟(~0.1s)，LIO 采样时刻偶在未来，回退到最新 TF
            try:
                ekf_tf = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                T_odom_ekf_to_body = self.tf_to_mat(ekf_tf)
            except Exception:
                pass
        except Exception as e:
            # 警告节流: EKF 不在时避免 50Hz 刷日志
            now = time.monotonic()
            if now - self._last_tf_warn > 2.0:
                self._last_tf_warn = now
                self.get_logger().warn(f'Cannot lookup odom->base_link from EKF: {e}')

        # 用 EKF 的 odom 定义来计算 map->odom
        if T_odom_ekf_to_body is not None:
            T_map_to_odom = self._mat_mul(T_map_to_base, self._inv_mat(T_odom_ekf_to_body))
        else:
            # 没有同一时刻的 EKF 位姿时，不把 LIO 的世界系冒充 EKF odom。
            return

        trans = T_map_to_odom[:3, 3]
        quat  = self._mat_to_quat(T_map_to_odom)

        # tf 브로드캐스트: map -> odom (与 EKF 的 odom 定义一致)
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id    = 'map'
        t_msg.child_frame_id     = 'odom'
        t_msg.transform.translation.x = trans[0]
        t_msg.transform.translation.y = trans[1]
        t_msg.transform.translation.z = trans[2]
        t_msg.transform.rotation.x    = quat[0]
        t_msg.transform.rotation.y    = quat[1]
        t_msg.transform.rotation.z    = quat[2]
        t_msg.transform.rotation.w    = quat[3]
        self.tf_broadcaster.sendTransform(t_msg)
        if self.broadcast_lio_frame:
            lio_tf = TransformStamped()
            lio_tf.header.stamp = t_msg.header.stamp
            lio_tf.header.frame_id, lio_tf.child_frame_id = 'map', 'lio_odom'
            xyz_lio = T_map_to_odom_fl[:3, 3]
            q_lio = self._mat_to_quat(T_map_to_odom_fl)
            lio_tf.transform.translation.x, lio_tf.transform.translation.y, lio_tf.transform.translation.z = map(float, xyz_lio)
            lio_tf.transform.rotation = Quaternion(x=float(q_lio[0]), y=float(q_lio[1]), z=float(q_lio[2]), w=float(q_lio[3]))
            self.tf_broadcaster.sendTransform(lio_tf)

        # fused localization (map -> body)
        xyz   = T_map_to_base[:3, 3]
        quat2 = self._mat_to_quat(T_map_to_base)

        loc_msg = Odometry()
        loc_msg.header.stamp          = fastlio_odom.header.stamp
        loc_msg.header.frame_id       = 'map'
        loc_msg.child_frame_id        = 'base_link'
        loc_msg.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        loc_msg.pose.pose.orientation = Quaternion(
            x=quat2[0], y=quat2[1], z=quat2[2], w=quat2[3]
        )
        loc_msg.twist = fastlio_odom.twist
        self.pub_localization.publish(loc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
