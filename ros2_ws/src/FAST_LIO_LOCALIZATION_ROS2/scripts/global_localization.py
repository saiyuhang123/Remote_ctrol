#!/usr/bin/env python3
# coding=utf-8

import array
from collections import OrderedDict
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Quaternion,
)


class GlobalLocalizationNode(Node):
    # PointField.datatype → numpy dtype, 用于向量化解析 PointCloud2
    _PF2NP = {
        PointField.INT8:    np.int8,
        PointField.UINT8:   np.uint8,
        PointField.INT16:   np.int16,
        PointField.UINT16:  np.uint16,
        PointField.INT32:   np.int32,
        PointField.UINT32:  np.uint32,
        PointField.FLOAT32: np.float32,
        PointField.FLOAT64: np.float64,
    }

    def __init__(self):
        super().__init__('fast_lio_localization')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('freq_localization', 0.5)      # Hz
        self.declare_parameter('localization_th', 0.9)
        self.declare_parameter('fov', 2 * np.pi)
        self.declare_parameter('fov_far', 100.0)

        self.map_voxel_size    = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size   = self.get_parameter('scan_voxel_size').value
        self.freq_localization = self.get_parameter('freq_localization').value
        self.localization_th   = self.get_parameter('localization_th').value
        self.FOV               = self.get_parameter('fov').value
        self.FOV_FAR           = self.get_parameter('fov_far').value
        translation = self.declare_parameter('base_from_body_translation', [0., 0., 0.]).value
        rotation = self.declare_parameter('base_from_body_quaternion', [0., 0., 0., 1.]).value
        self.base_from_body = tf_transformations.quaternion_matrix(rotation)
        self.base_from_body[:3, 3] = translation
        self.initial_base_z = self.declare_parameter('initial_base_z', 0.).value
        self.pending_initial_pose = None

        # ─── State Variables ─────────────────────────────────────
        self.global_map      = None
        self.map_points      = None   # 降采样后全局地图点 (N×3), crop 直接用
        self.map_normals     = None   # 与 map_points 对齐的法向量, 供 point-to-plane ICP
        self.initialized     = False
        self.T_map_to_odom   = np.eye(4)
        self.cur_odom        = None
        self.cur_scan        = None
        self.cur_scan_odom   = None  # 与 cur_scan 时间对齐的里程计
        self._odom_cache = OrderedDict()
        self._pending_scans = OrderedDict()
        self._scan_stamp = -1

        # ─── Publishers ──────────────────────────────────────────
        self.pub_pc_in_map   = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap      = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry,     '/map_to_odom', 1)

        # ─── Subscriptions ───────────────────────────────────────
        self.create_subscription(PointCloud2,                  '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry,                    '/Odometry',         self.cb_save_cur_odom,  1)
        self._map_sub  = self.create_subscription(PointCloud2, '/global_map',               self.cb_init_map,      1)
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_init_pose,
            1
        )

        self.get_logger().info('GlobalLocalizationNode initialized.')

    def pc2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N×3) NumPy array, numpy 向量化解析 (避免逐点 Python 循环)"""
        names, formats, offsets = [], [], []
        for f in pc_msg.fields:
            dt = self._PF2NP[f.datatype]
            formats.append((dt, (f.count,)) if f.count > 1 else dt)
            names.append(f.name)
            offsets.append(f.offset)
        dtype = np.dtype({
            'names': names, 'formats': formats,
            'offsets': offsets, 'itemsize': pc_msg.point_step,
        })
        dtype = dtype.newbyteorder('>' if pc_msg.is_bigendian else '<')
        arr = np.ndarray((pc_msg.height, pc_msg.width), dtype=dtype,
                         buffer=pc_msg.data, strides=(pc_msg.row_step, pc_msg.point_step))
        pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).reshape(-1, 3).astype(np.float32)
        pts = pts[np.isfinite(pts).all(axis=1)]
        return pts

    def cb_init_map(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd = self.voxel_down_sample(pcd, self.map_voxel_size)
        # 法向量只在加载时算一次, 之后 crop 按掩码同步裁剪, 供 point-to-plane ICP 使用
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=self.map_voxel_size * 3.0, max_nn=30))
        self.global_map  = pcd
        self.map_points  = np.asarray(pcd.points)
        self.map_normals = np.asarray(pcd.normals)
        self.get_logger().info(
            f'Global map received: {len(self.map_points)} points after downsampling, normals precomputed.')
        self.destroy_subscription(self._map_sub)
        self._try_initial_pose()

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn('Initial pose must be in map frame')
            return
        self.pending_initial_pose = msg
        self._try_initial_pose()

    def _try_initial_pose(self):
        if self.pending_initial_pose is None or self.global_map is None or self.cur_scan_odom is None:
            return
        msg = self.pending_initial_pose
        self.pending_initial_pose = None

        # /initialpose is map<-base, but ICP estimates map<-LIO world.
        initial_base = self.pose_to_mat(msg)
        if abs(initial_base[2, 3]) < 1e-9:
            initial_base[2, 3] = self.initial_base_z
        initial = initial_base @ self.base_from_body @ np.linalg.inv(self.pose_to_mat(self.cur_scan_odom))
        # 已初始化也允许重定位 (机器人被搬动/定位丢失时可从 RViz 重新给初始位姿)
        success = self.global_localization(initial)
        if success:
            if not self.initialized:
                self.initialized = True
                period = 1.0 / self.freq_localization
                self.create_timer(period, self.timer_callback)
                self.get_logger().info('Initial global localization succeeded.')
            else:
                self.get_logger().info('Re-localization succeeded.')
        elif self.initialized:
            # 重定位失败: 保持原 T_map_to_odom, 周期 ICP 继续跑
            self.get_logger().warn('Re-localization failed, keeping previous pose.')

    def cb_save_cur_odom(self, msg: Odometry):
        self.cur_odom = msg
        stamp = self._stamp_ns(msg)
        self._odom_cache[stamp] = msg
        if stamp in self._pending_scans:
            self._save_matched_scan(self._pending_scans.pop(stamp), msg)
        while len(self._odom_cache) > 20:
            self._odom_cache.popitem(last=False)

    def cb_save_cur_scan(self, msg: PointCloud2):
        stamp = self._stamp_ns(msg)
        if stamp in self._odom_cache:
            self._save_matched_scan(msg, self._odom_cache[stamp])
        else:
            self._pending_scans[stamp] = msg
            while len(self._pending_scans) > 5:
                self._pending_scans.popitem(last=False)

    @staticmethod
    def _stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _save_matched_scan(self, msg, odom):
        stamp = self._stamp_ns(msg)
        if stamp <= self._scan_stamp:
            return
        self._scan_stamp = stamp
        msg.header.frame_id = 'lio_odom'
        # 纯可视化话题, RViz 没订阅时跳过重发 (省每帧几百 KB 的序列化/传输)
        if self.pub_pc_in_map.get_subscription_count():
            self.pub_pc_in_map.publish(msg)

        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.cur_scan = pcd
        # /Odometry 与 /cloud_registered 是独立 DDS 话题，按相同 stamp 配对。
        self.cur_scan_odom = odom
        self._try_initial_pose()

    def timer_callback(self):
        self.global_localization(self.T_map_to_odom)

    def global_localization(self, pose_est):
        if self.cur_scan is None or self.cur_scan_odom is None or self.global_map is None:
            return False
        if not len(self.cur_scan.points):
            return False
        self.get_logger().info('Performing global localization via ICP...')
        # 单线程 executor, cur_scan 不会被并发替换; ICP/降采样均不改输入, 无需 deepcopy
        scan_copy = self.cur_scan

        # 使用与 scan 时间对齐的里程计，而非可能不同步的 latest odom
        matched_odom = self.cur_scan_odom
        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, matched_odom)

        vs = self.scan_voxel_size
        # 粗配准: 大体素 + point-to-point, 收敛域大、抗差
        T, _       = self.registration_at_scale(
            self.voxel_down_sample(scan_copy, vs * 5),
            self.voxel_down_sample(submap,    vs * 5),
            initial=pose_est, scale=5, point_to_plane=False)
        # 精配准: 正常体素 + point-to-plane, 精度高
        # submap 已带预计算法向量, 此处不再重采样 (voxel_down_sample 会丢法向量)
        T, fitness = self.registration_at_scale(
            self.voxel_down_sample(scan_copy, vs),
            submap,
            initial=T,        scale=1, point_to_plane=True)
        self.get_logger().info(f'ICP fitness: {fitness:.3f}')

        if fitness > self.localization_th:
            self.T_map_to_odom = T
            odom = Odometry()
            xyz  = tf_transformations.translation_from_matrix(T)
            quat = tf_transformations.quaternion_from_matrix(T)
            # 올바른 Odometry 메시지 필드 설정
            odom.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            odom.header.stamp          = matched_odom.header.stamp
            odom.header.frame_id       = 'map'
            self.pub_map_to_odom.publish(odom)
            return True

        self.get_logger().warn('Global localization failed (fitness below threshold).')
        return False

    def crop_global_map_in_FOV(self, scan, pose_est, odom):
        T_scan     = self.pose_to_mat(odom)
        T_map2scan = np.linalg.inv(pose_est @ T_scan)

        pts = self.map_points
        R = T_map2scan[:3, :3]
        t = T_map2scan[:3, 3]
        # 掩码只用到变换后的 x(/y), 按行点积, 不做 N×4 齐次矩阵乘
        if self.FOV >= 2*np.pi:
            xs = pts @ R[0] + t[0]
            mask = xs < self.FOV_FAR
        else:
            xy  = pts @ R[:2].T + t[:2]
            ang = np.arctan2(xy[:,1], xy[:,0])
            mask = (xy[:,0]>0)&(xy[:,0]<self.FOV_FAR)&(np.abs(ang)<self.FOV/2)

        subpts = pts[mask]
        submap = o3d.geometry.PointCloud()
        submap.points  = o3d.utility.Vector3dVector(subpts)
        submap.normals = o3d.utility.Vector3dVector(self.map_normals[mask])

        # 纯可视化: 无订阅者时跳过; array.array('B') 走 data setter 直通通道,
        # 避免 create_cloud_xyz32 + tolist 的逐点 Python 打包
        if self.pub_submap.get_subscription_count():
            viz = np.ascontiguousarray(subpts[::10], dtype=np.float32)
            cloud = PointCloud2()
            cloud.header.stamp    = self.get_clock().now().to_msg()
            cloud.header.frame_id = 'map'
            cloud.height, cloud.width = 1, viz.shape[0]
            cloud.fields = [
                PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(('x', 'y', 'z'))
            ]
            cloud.is_bigendian = False
            cloud.point_step, cloud.row_step = 12, 12 * viz.shape[0]
            cloud.data = array.array('B', viz.tobytes())
            cloud.is_dense = True
            self.pub_submap.publish(cloud)

        return submap

    def registration_at_scale(self, scan, submap, initial, scale, point_to_plane):
        if point_to_plane and submap.has_normals():
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        else:
            # point-to-plane 需要 target 法向量, 没有时退回 point-to-point
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        reg = o3d.pipelines.registration.registration_icp(
            scan, submap,
            max_correspondence_distance=1.0*scale,
            init=initial,
            estimation_method=estimation,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return reg.transformation, reg.fitness

    @staticmethod
    def pose_to_mat(pose_stamped):
        t = pose_stamped.pose.pose.position
        q = pose_stamped.pose.pose.orientation
        return tf_transformations.translation_matrix([t.x,t.y,t.z]) \
             @ tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])

    @staticmethod
    def voxel_down_sample(pcd, vs):
        try:
            return pcd.voxel_down_sample(vs)
        except:
            return o3d.geometry.voxel_down_sample(pcd, vs)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
