# JD03 导航倾角辅助与默认地图部署

日期：2026-10-22。用户要求先保存现有代码，再启用导航倾角辅助，并部署已确认的 PCD 与 Nav2 切片。

## Git 基线

修改导航前，已将此前建图修复及 UI 保存链路提交为：

```text
a945dbc feat(fastlio): checkpoint gravity-aided mapping and validated map export
```

## 生效配置

UI 的「FAST-LIO 定位 / Nav2 导航」经 `navigation.launch.py` 调用统一 bringup，
使用 `ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/config/velodyne_test.yaml`。

```yaml
preprocess:
  scan_line: 16
gravity_aid:
  enable: true
  sigma_deg: 0.005
```

导航现在与建图使用相同的 WIT 重力方向观测，检查四元数、加速度模长和方向一致性。
不融合设备 yaw，不把车身强制设为水平。0.005° 表示优化权重，并非 IMU 真实测量精度。
外参继续由 `chassis_nav2/config/hardware.yaml` 统一提供：

```text
雷达→IMU：Rz(+90°)，T=[0,0,0.24] m
IMU→base_link：yaw=-90°，T=[0.32,0,0.22] m
```

已通过 colcon 重新构建/安装 `fast_lio_localization` 和 `chassis_nav2`。
安装配置使用源码软链接，完整定位回放会核对运行中的参数，避免只改源码未生效。

## UI 默认地图

UI 默认前缀：`/home/jkbot/Remote_ctrol/maps/fastlio/map`。

已部署用户认可的离线重建地图，保留原始归档：

```text
源：/home/jkbot/Remote_ctrol/maps/jd03_rebuilt_20261022/jd03_map.pcd
目标：/home/jkbot/Remote_ctrol/maps/fastlio/map.pcd
点数：1,554,307
大小：24,869,072 字节
SHA256：0f101c959613f96cf471dfcba9a04a06d2a06e6cf64d308cd640799dd4f15fb5
```

原 UI PCD/PGM/YAML 三件套已整体备份在：

```text
/home/jkbot/Remote_ctrol/maps/fastlio/map_backup_20261022_120226_before_tilt_navigation/
```

新 PCD 与归档 SHA256 一致，未做额外平移、旋转、缩放。源验证记录保存在
`map.validation.json`，部署来源及备份信息保存在 `map.deployment.json`。

## Nav2 切片

沿用 UI 实际调用的 `desktop_ui/scripts/export_fastlio_map.py` 和现有切片配置，
从当前已部署 PCD 重新生成，未复用旧栅格：

```text
/home/jkbot/Remote_ctrol/maps/fastlio/map.pgm
/home/jkbot/Remote_ctrol/maps/fastlio/map.yaml
```

- 高度范围：−0.10～2.00 m。
- 分辨率：0.05 m/cell（YAML 中保留 ROS float 的完整精度）。
- 半径离群过滤：0.2 m，最少 6 个邻点。
- 尺寸：6640 × 2526。
- origin：`[-291.64239501953125, -84.72772979736328, 0.0]`。
- 归档轨迹全部落在栅格内，轨迹中心没有落入占用像素；这只说明切片与轨迹的几何一致性。
- 原始 Nav2 栅格预览：`maps/fastlio/map_preview.png`。
- 详细切片检查：`maps/fastlio/slice_validation.json`。

网页地图也通过现有 `scripts/export_web_map.py` 从同一份 YAML 更新，地图原点与分辨率一致。

复现切片：

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
cd /home/jkbot/Remote_ctrol
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=3 \
python3 desktop_ui/scripts/export_fastlio_map.py \
  --prefix /home/jkbot/Remote_ctrol/maps/fastlio/map \
  --config /home/jkbot/Remote_ctrol/desktop_ui/config/fastlio_map_export.yaml
```

## 验证

桌面端 `desktop_ros_services` 和 `desktop_map_saver` 两项 CTest 已通过，
覆盖修正地图服务选择、服务超时策略、地图保存后 PGM/YAML 导出及相关失败处理。

新增 `scripts/validate_fastlio_localization.py`，在私有 ROS 域播放原 bag，
启动实际安装的定位链路和 Nav2 map_server，禁用传感器驱动、底盘通信与运动控制。
它核对实际参数、地图加载、倾角一致性、ICP 匹配比例与时间覆盖率，记录原始轨迹和报告。

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
cd /home/jkbot/Remote_ctrol
python3 -u scripts/validate_fastlio_localization.py \
  --bag /home/jkbot/bags/fastlio_sam_jd03_20261022_083713 \
  --output /home/jkbot/fastlio_sam_ws/navigation_tilt_20261022_repeat \
  --pcd /home/jkbot/Remote_ctrol/maps/fastlio/map.pcd \
  --map-yaml /home/jkbot/Remote_ctrol/maps/fastlio/map.yaml \
  --initial-poses /home/jkbot/Remote_ctrol/maps/jd03_rebuilt_20261022/keyframe_poses.npy \
  --rate 2.0 --domain 218
```

输出目录必须此前不存在。初始位姿来自归档地图的第一帧并换算到 base_link，
后续没有重复注入位姿；同 bag/同地图回放只验证内部一致性，不是外部真值。

本次回放目录：`/home/jkbot/fastlio_sam_ws/navigation_tilt_20261022_1205/`。
完整 bag 以 2 倍速回放通过，输出 `localization_validation.json` 与
`continuity_validation.json`，主要结果：

| 检查 | 结果 |
|---|---:|
| 有效前端轨迹时长 | 1098.62 s |
| 前端 odometry | 10,961 条 |
| 地图定位修正 | 1,096 次 |
| ICP 匹配成功比例 | 100% |
| ICP fitness 最小值 / 中位数 | 0.980 / 0.999 |
| 地图修正覆盖的轨迹时间比例 | 99.85% |
| IMU 与前端倾角差 95 分位 | 0.148° |
| 重力对齐后前端高度范围 | −0.416～+0.323 m |
| 相邻 ICP 修正在车辆位置产生的位置变化最大值 | 0.197 m |
| 与归档地图轨迹的 XY 差值 95 分位 | 0.112 m |
| 与归档地图轨迹的 Z 差值 95 分位 | 0.048 m |
| Nav2 实际加载的栅格原点、尺寸与分辨率 | 与部署 YAML 一致 |

这些结果是同 bag/同地图内部一致性检查，不能写成有独立真值的绝对导航精度。
瞬时倾角差最大 10.09°，10,894 个有效比较样本中有 4 个超过 3°，位于约
493～605 s；同期前端也存在短时较大倾角，现有记录不能区分实际颠簸、设备滤波延迟
等影响。全程 ICP 仍持续匹配成功，地图定位位置未出现对应的大幅跳变。
最长修正间隔 4.90 s 出现在启动初始化阶段，后续间隔最大约 1.10 s。
本次没有为获得低峰值而强行压平车辆倾角，实车仍需核对颠簸和坡道表现。

## 使用

重新打开 UI，选择「FAST-LIO 定位 / Nav2 导航」，默认地图前缀保持上述路径，
导航栅格使用同一前缀的 `map.yaml`。旧图坐标中的初始位姿和记录路线不能直接沿用，
需在新地图上重新设置初始位姿并检查实时点云贴合，再进行实车导航验证。
