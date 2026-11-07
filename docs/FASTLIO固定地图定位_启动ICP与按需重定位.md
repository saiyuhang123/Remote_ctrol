# FAST-LIO 固定地图定位：启动 ICP 与按需重新定位

适用机器：172.20.10.3，项目 `/home/jkbot/Remote_ctrol`。

## 1. 这次改造的行为

1. 启动时，FAST-LIO 加载保存的 PCD；短暂使用在线局部地图产生去畸变点云和里程计，等待初始位姿。
2. 在 RViz 使用 **2D Pose Estimate** 给出车在地图上的位置和车头方向，触发启动 ICP。数据尚未准备好时等待数据；匹配失败时最多尝试 5 次。
3. ICP 成功后，将固定 PCD 转换到已有 LIO 坐标系并建立搜索树。后续 FAST-LIO 持续匹配固定 PCD，不再向地图增点或删除地图区域。
4. 正常跟踪期间不运行周期 ICP；ICP 节点保留，等待下一次 `/initialpose` 请求。
5. 需要重新定位时，再使用一次 **2D Pose Estimate**。ICP 成功后替换搜索树并发布全局修正，保留 LIO 坐标系、滤波状态和 IMU 状态；失败时保留上次有效定位。

这条模式属于 FAST-LIO 定位支路。正常建图仍使用原来的在线增量地图。

## 2. 启动方式

桌面端可以继续使用原来的 FAST-LIO 定位、导航入口。先停止当前后端，再重新启动 FAST-LIO 后端，让新配置生效。

也可以使用项目包装器从终端启动。以下是机器上现有的一对 PCD / YAML 地图，实际操作时应选择作业区域对应的同坐标地图：

```bash
cd /home/jkbot/Remote_ctrol

bash scripts/ros2_local.sh launch chassis_nav2 bringup.launch.py \
  mode:=navigation \
  odom_source:=lio \
  start_sensors:=true \
  start_chassis:=true \
  enable_control:=true \
  compare_wheel_odometry:=false \
  pcd_map:=/home/jkbot/Remote_ctrol/maps/fastlio/map.pcd \
  map:=/home/jkbot/Remote_ctrol/maps/fastlio/map.yaml \
  rviz:=true \
  hardware_file:=/home/jkbot/Remote_ctrol/ros2_ws/src/chassis_nav2/config/hardware.yaml \
  lidar_config:=/home/jkbot/Remote_ctrol/ros2_ws/src/rslidar_sdk-v1.5.19/config/config.yaml
```

RViz 的 Fixed Frame 选择 `map`。使用 **2D Pose Estimate** 在小车实际位置按住鼠标，沿车头方向拖出箭头。等待初始化成功、点云与地图对齐后，再发送短距离导航目标。

本次保留了车辆原来的速腾雷达输入、WIT IMU 外参、`odom_source` 选择和 Nav2 TF 发布分工。

## 3. 检查状态

另开终端：

```bash
cd /home/jkbot/Remote_ctrol
bash scripts/ros2_local.sh topic echo /fastlio/prior_map_status \
  --qos-durability transient_local
```

初始等待阶段类似：

```text
{"mode":"bootstrap","revision":0,"map_points":14591}
```

初始化成功后类似：

```text
{"mode":"fixed_map","revision":1,"map_points":14591}
```

`map_points` 的具体值取决于所选地图和降采样参数。重新定位成功后 `revision` 增加，固定地图点数保持不变。

正常跟踪期间，日志不应再每秒出现 `Performing global localization via ICP...`。激光和 IMU 的实时匹配仍由 FAST-LIO 持续完成。

## 4. 按需重新定位

1. 暂停任务并停车。
2. 在 RViz 再次用 **2D Pose Estimate** 指定实际位置和车头方向。
3. 成功时日志出现 `Re-localization succeeded` 和 `Fixed prior map active`，状态的 `revision` 增加。
4. 检查点云贴合地图后继续导航。

如果匹配失败，旧搜索树和旧全局修正继续保留；修正初始位置、朝向后可再次尝试。出现 `Fixed-map matching weak` 时应检查地图、点云与初始位姿，不能把仍有里程计输出视作地图定位正常。

重启 FAST-LIO 进程时应停止并重新启动整个 FAST-LIO 定位后端，再给初始位姿；上次进程的对齐结果不属于新的 LIO 坐标系。

## 5. 配置位置与兼容模式

配置文件：

```text
/home/jkbot/Remote_ctrol/ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/config/velodyne_test.yaml
```

公共参数 `prior_map.enable: true` 同时控制 C++ 固定地图模式与 Python 按需 ICP 模式。

设置为 `false` 并重启整个定位后端，可恢复原来的在线局部地图 + 周期 ICP 方式；周期仍由 `freq_localization` 控制。安装目录若不是符号链接，改配置后需重新构建该包。

建图配置 `velodyne.yaml` 默认不启用固定地图模式，保存地图行为沿用原实现。

## 6. 整项回退到改造前版本

改造前基线：`db57c4e`，保留在原分支 `feature/live-6dof-loop-closure`。

改造分支：`codex/fastlio-prior-map-localization`。

先停止建图、定位、导航进程。工作区没有未保存改动时执行：

```bash
cd /home/jkbot/Remote_ctrol
git switch feature/live-6dof-loop-closure

source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select fast_lio_localization --executor sequential
source install/setup.bash
```

回退后重新启动定位后端。切分支后需要重新编译 C++，不能只重启界面。

## 7. 已完成的验证与边界

- 固定地图 C++ 测试：检查六自由度坐标变换的逆变换、重复重新定位不修改原地图、无效数据拒绝。
- ROS 运行测试：真实 ICP 初始化、空闲期间无周期 ICP、按需重定位、失败保持旧定位、无效对齐拒绝、后启动的 TF 消费者获取已接受修正。
- 原导航集成测试及地图缓存发布测试。
- 在独立 ROS 域 191、仅本机通信下回放现有 `/velodyne_points` 和 `/imu/data`。先从同一录包短段生成临时地图，再运行固定地图定位并触发一次重新定位。未启动底盘、控制和导航运动节点。
- 该回放中收到 212 帧里程计和 1052 条定位输出；固定地图 14591 点，状态版本从 0 到 1 到 2，最大相邻里程计位移约 0.078 米，与原链路回放接近。

回放数据用于验证处理链路、重新定位和状态连续性，不是定位精度真值测试；尚未据此给出整机 CPU 降幅或厘米级精度结论。场地上的长距离导航效果仍需实车复核。
