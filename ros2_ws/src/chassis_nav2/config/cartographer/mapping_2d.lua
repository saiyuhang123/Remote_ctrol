-- JD03 巡检车 2D 建图配置（cartographer）。
-- 数据源：/scan（pointcloud_to_laserscan 单帧 360°）、/odom（EKF 轮速+IMU 融合）、/imu/data（wit_ros2_imu）。
-- TF：cartographer 发 map->odom；odom->base_link 由 EKF 发布；base_link->imu_link 静态 TF。
-- 注意：cartographer 强制 IMU 系与 tracking_frame 位置重合（sensor_bridge.cpp CHECK，
-- 否则 SIGABRT），而本车 IMU 实测偏离 base_link [0.32, 0, 0.22]，故 tracking_frame 用
-- imu_link；published_frame 保持 base_link，对外发布的 map->odom 不受影响。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,   -- /odom 是 EKF 融合结果，与 /imu/data 同源（重复融合）；scan matching 主要靠 IMU 外推
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.3    -- 与 hardware.yaml scan.range_min 一致
TRAJECTORY_BUILDER_2D.max_range = 15.0   -- 与 hardware.yaml scan.range_max 一致
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 匹配窗口/运动滤波采用 kybot(601b) 实测好用的组合：窗口更大容忍外推误差，
-- 节点更密使相邻帧重叠更多、匹配更稳
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- 建图从严防错回环；定位在 localization_2d.lua 放宽到 0.6

return options
