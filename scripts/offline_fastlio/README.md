# FAST-LIO bag 离线回环 PCD 流程

本目录只做**离线复现和回环后处理**，不修改机器人现场运行节点。

## 已确认的 bag 条件

`bags/fastlio_z_20261021_160456`：

| 项目 | 值 |
|---|---|
| 时长 | 1298 s（约 21 分 38 秒） |
| 大小 | 10 GiB |
| `/velodyne_points` | 12982 帧，约 10 Hz |
| `/imu/data` | 129764 帧，约 100 Hz |
| LiDAR frame_id | `velodyne` |
| IMU frame_id | `imu_link` |
| 点字段 | `x y z intensity ring time` |
| ring | 0..15（16 线） |
| 点时间字段 | 秒，单帧跨度约 0.10 s |
| bag 内 TF | 无（离线 FAST-LIO 不需要） |

FAST-LIO 所需的外参来自 `chassis_nav2/config/hardware.yaml`：

```text
base_from_lidar = [0.32, 0.00, 0.46], rpy=[0,0,0]
base_from_imu   = [0.32, 0.00, 0.22], rpy=[0,0,0]
```

换算成 LiDAR 相对 IMU：

```text
T_imu_lidar translation = [0.0, 0.0, 0.24] m
T_imu_lidar rotation    = identity
```

离线参数使用：

```text
preprocess.scan_line = 16
preprocess.timestamp_unit = 0       # time field is seconds
common.time_offset_lidar_to_imu = 0.0
mapping.extrinsic_T = [0, 0, 0.24]
mapping.extrinsic_R = identity
mapping.extrinsic_est_en = false
```

## 阶段 1：完整 bag 建图 + 记录关键帧

```bash
cd /home/jkbot/Remote_ctrol

ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=0 \
RATE=1.0 \
KEYFRAME_DISTANCE=0.8 \
KEYFRAME_ANGLE=12.0 \
KEYFRAME_MAX_GAP=2.0 \
./scripts/offline_fastlio/run_offline_capture.sh \
  /home/jkbot/Remote_ctrol/bags/fastlio_z_20261021_160456 \
  /home/jkbot/Remote_ctrol/offline_outputs/fastlio_bag_full
```

产物：

```text
offline_outputs/fastlio_bag_full/map_raw.pcd
offline_outputs/fastlio_bag_full/keyframes/keyframes.jsonl
offline_outputs/fastlio_bag_full/keyframes/kf_*.npz
```

`kf_*.npz` 包含：

```text
xyz       当前扫描在 imu_link 坐标系的点
intensity 强度
stamp_ns  点云时间戳
pose      4x4 T_lio_odom_body
```

关键帧触发条件：移动超过 0.8 m、转动超过 12°，或非静止状态持续 2 秒。  
纯停车不会生成重复关键帧。

## 阶段 2：ICP 回环检测 + 位姿图优化 + 重建 PCD

先跑默认参数：

```bash
./scripts/offline_fastlio/run_loop_closure.sh \
  /home/jkbot/Remote_ctrol/offline_outputs/fastlio_bag_full/keyframes
```

默认会：

1. 用初始 FAST-LIO 轨迹找空间邻近且时间相隔至少 60 秒的历史关键帧；
2. 对候选历史关键帧构建局部子地图；
3. 用初始相对位姿做 ICP；
4. 接受 fitness、RMSE、修正量都在阈值内的回环；
5. 使用 Open3D 位姿图优化器（PoseGraph/GlobalOptimization）优化轨迹；
6. 用优化后的关键帧位姿重建并保存：

```text
offline_outputs/fastlio_bag_full/keyframes/loop_closure/
  loop_closures.json
  trajectory_optimized.tum
  loop_closed_map.pcd
```

## 调参顺序

如果回环没有接受：

1. 先看 `loop_closures.json` 是否为空；
2. 放宽 `--search-radius`（默认 8 m）；
3. 降低 `--min-fitness`（默认 0.30）；
4. 放宽 `--max-rmse`（默认 0.30 m）；
5. 再考虑调整 `--min-time-diff`（默认 60 s）和 `--min-index-diff`（默认 30）。

### 只做始末 6DOF 约束（适合 bag 终点离起点很近但不完全重合）

```bash
./scripts/offline_fastlio/run_loop_closure.sh   /home/jkbot/Remote_ctrol/offline_outputs/fastlio_bag_full4/keyframes   --manual-start-end   --manual-submap-half 10   --manual-min-fitness 0.35   --manual-max-rmse 0.55   --map-voxel 0.10
```

这条命令强制取 `最后一帧 -> 第一帧` 做一次 ICP，适合只有一处大闭环的场景。  
它不会伪造水平闭合：如果 bag 终点离起点还有一段距离，`diagnostics.json` 会保留水平残差。当前这个 bag 的实际结果是：

```text
原始终点相对起点：XY 差 9.18 m，Z 差 -6.38 m
6DOF 始末约束后：  Z 差 -0.03 m
水平 XY 残差仍约 7.73 m（因为 bag 还没走完最后一段）
```

因此当前结果可以修掉 Z 轴高度漂移，但要得到真正水平闭合的园区一圈 PCD，还应补录最后约 9 m 的回到起点的轨迹。

如果接受了很多错误回环：

1. 提高 `--min-fitness`；
2. 降低 `--search-radius`；
3. 增大 `--min-time-diff`；
4. 增加 `--min-index-diff`；
5. 检查 `loop_closures.json` 中的 `correction_translation` 和 `correction_yaw_rad`，异常大的修正应剔除。

## 重要边界

- 这是离线后处理，不替代实时 SLAM；
- 最终 PCD 由关键帧重建，密度取决于关键帧间距；默认 0.8 m；
- 当前 PCD 写入 `x y z intensity normals curvature`，intensity 在重建时填 0；
- 如果后续需要强度图或更密地图，可以把 `KEYFRAME_DISTANCE` 调小到 0.3~0.5 m，或增加“回放所有扫描并用优化后轨迹重建”的第二阶段；
- 回环 ICP 对 16 线稀疏点云比 MID360 更敏感，必须用 `loop_closures.json` 检查是否是真回环。
