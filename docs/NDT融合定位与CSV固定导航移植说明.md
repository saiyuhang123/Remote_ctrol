# NDT 融合定位与 CSV 固定导航（ROS 2 Humble）

本次从 `/home/jkbot/catkin_ws` 移植的是 `navigation/msf_localizer` 的 NDT 地图匹配定位，以及 `lane_back` / `navigation/global_planner` / `navigation/control/purePursuitNode.cpp` 的 CSV 录点和沿线跟踪流程。代码放在 `/home/jkbot/Remote_ctrol/ros2_ws/src`。建图不在此次移植范围，现有建图入口未修改。

新增包 `msf_localizer` 和 `fixed_route_navigation`；启动文件放在已有 `chassis_nav2` 中。它们是独立的 NDT + CSV 入口，不需要启动 FAST-LIO，也不依赖旧 ROS 1 驱动、`move_base` 或 `my_nav_msgs`。现有 Nav2/网页录制路网功能继续使用原来的入口；新增 CSV 路线通过下文的 ROS 服务操作，未接入网页巡检数据库。

## 设备与坐标接口

所有安装外参、底盘几何、CAN 配置、速度比例和转角符号都沿用 `chassis_nav2/config/hardware.yaml`：

| 设备/功能 | 本车接口 | 处理 |
| --- | --- | --- |
| JD03 底盘 | `/vehicle_status` | 现有 `jd03_odometry.py` 解析实测速度/转角，使用本车速度比例 0.29、轴距 0.5m 和四轮转向参数 |
| 速腾 RSHELIOS_16P | `/velodyne_points`，`velodyne` | 直接输入 NDT，默认雷达位于车前 0.32m、车上 0.46m |
| 维特 IMU | `/imu/data`，`imu_link` | 由现有 `robot_localization` 融合角速度；外参使用本车安装偏航角 −π/2 |
| 局部融合里程计 | `/odom` | 默认轮速 + IMU EKF；发布 `odom → base_link` |
| NDT 全局修正 | `map → odom` | 使用扫描时间对应的里程计预测，与 PCD 匹配后更新修正 |
| 全局位姿 | `/current_pose` | `geometry_msgs/PoseStamped`，供录点和跟踪使用 |
| 平面障碍扫描 | `/scan` | 沿用点云裁剪和 `pointcloud_to_laserscan` |
| 固定路线速度 | `/cmd_vel_auto` | 经现有 `twist_mux → /cmd_vel → jd03_bridge.py → /ecu`，遥控优先级更高 |

TF 为 `map → odom → base_link → velodyne / imu_link`。NDT 不直接发布 `map → base_link`，也不发布 `/odom`，避免与 EKF 重复发布同一变换。三维地图使用 `/map_cloud`，与可选栅格地图 `/map` 分开。

NDT 采用 PCL CPU 实现，保留旧流程的“里程计预测 + NDT 校正”，并可通过 `secondary_odom_topic` 额外使用已有的 `/odom_lio` 预测，但本入口不会启动它。没有适配本车不存在的 GPS。原代码中使匹配检查无条件通过的 `|| true` 已移除，未收敛、分数过差、修正过大和过期的匹配不会更新定位。

## 构建

现有导航依赖安装脚本已包含 PCL、Eigen、`robot_localization` 和传感器依赖，无需新增第三方仓库。

```bash
cd /home/jkbot/Remote_ctrol/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install --packages-select msf_localizer fixed_route_navigation chassis_nav2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

首次完整部署也可使用已更新的 `bash scripts/build_navigation.sh`。

## 只启动 NDT 定位

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/local_setup.bash
ros2 launch chassis_nav2 ndt_localization.launch.py \
  pcd_map:=/home/jkbot/Remote_ctrol/maps/fastlio/map.pcd \
  start_chassis:=true rviz:=true
```

如果底盘厂家驱动已在另一终端启动，省略 `start_chassis:=true`。底盘驱动提供实际反馈，定位默认启动轮速 + IMU EKF。`enable_control` 默认为 false；如需通过当前底盘桥遥控车辆，追加 `enable_control:=true`。

使用自己修改后的建图产物时，将 `pcd_map` 指向对应的有效 PCD。也可用 `pcd_dir:=/绝对路径/point_map` 加载同一地图坐标系内的多个 PCD，目录参数优先于默认文件参数。旧地图可直接指定 `/home/jkbot/catkin_ws/src/navigation/config_data/point_map`；不自动复制 328MB 地图或旧车外参。

在 RViz 点 **2D Pose Estimate** 设置车辆当前实际位置和方向。必须等待 `/localization/healthy` 为 true 后才能录制或开始路线；不会以默认零位姿直接导航。地图 z 轴原点不在车体中心高度时，初始位姿的 z 也应与地图一致，可手动发布包含 z 的 `/initialpose`。检查命令：

```bash
ros2 topic echo /localization/healthy
ros2 topic echo /current_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

`map:=/绝对路径/map.yaml` 可选，只用于栅格地图显示，必须与 PCD 的坐标一致。固定路线跟踪不要求提供栅格地图。

## 录点并沿录点行驶

第一次录制时选择一个尚不存在的 CSV 文件：

```bash
ros2 launch chassis_nav2 fixed_route_navigation.launch.py \
  pcd_map:=/home/jkbot/Remote_ctrol/maps/fastlio/map.pcd \
  route_file:=/home/jkbot/Remote_ctrol/data/fixed_routes/route_01.csv \
  start_chassis:=true enable_control:=true rviz:=true
```

1. 用 **2D Pose Estimate** 初始化并等待 NDT 定位有效。
2. 调用开始录制服务，再通过现有网页/手柄遥控沿期望路线驾驶。
3. 到终点停车，然后调用结束录制。有效路线才会发布成最终 CSV。
4. 手动回到录制起点并对齐车头，停车后调用 `start`。导航按记录顺序经过整条路线，到末端确认停稳后结束。

```bash
ros2 service call /fixed_route/start_recording std_srvs/srv/Trigger '{}'
ros2 service call /fixed_route/stop_recording std_srvs/srv/Trigger '{}'
ros2 service call /fixed_route/start std_srvs/srv/Trigger '{}'
```

默认录点距离间隔 5cm，转角变化达到 0.05rad 时也会保留采样。最终 CSV 带 `x,y,z,qx,qy,qz,qw,speed` 表头。旧系统无表头的 7 列 CSV（同一字段但没有 speed）可直接加载；也支持 2/3 列坐标文件，此时按行驶方向推算车头朝向。重复静止点会去重，损坏行、无效四元数和超过 1m 的采样断点会报错，不会跨过损坏数据直接连线。

旧系统通过 `rostopic` 记录的 7 列文件通常含静止定位抖动，导入时默认做 8cm 空间去抖，保留路线首尾；参数为 `legacy_spacing`。已用原工作空间的真实 `lane_0.csv` 检查导入及道路几何。此处理只用于无速度的旧 7 列文件；新 8 列录制不会按该阈值抽稀，保留短距离换向边界。需要精确保留厘米级运动和方向时应使用新录制格式。

录制写入 `route_01.csv.partial`，结束并验证成功后原子发布为 `route_01.csv`，不会覆盖现有路线。录制中定位失效、跳变、重设初始位姿或取消时，保留 `.partial` 供检查，不能作为有效路线执行；请另选文件重新录制。

已有定位、扫描、底盘桥和 mux 正在运行时，只启动路线节点：

```bash
ros2 launch chassis_nav2 fixed_route_navigation.launch.py \
  start_localization:=false \
  route_file:=/home/jkbot/Remote_ctrol/data/fixed_routes/route_01.csv
```

这种方式要求现有定位提供 `/current_pose` 和 `/localization/healthy`；本次 NDT 入口已经提供。

其他控制命令：

```bash
ros2 service call /fixed_route/pause std_srvs/srv/Trigger '{}'
ros2 service call /fixed_route/resume std_srvs/srv/Trigger '{}'
ros2 service call /fixed_route/stop std_srvs/srv/Trigger '{}'
ros2 service call /fixed_route/reload std_srvs/srv/Trigger '{}'
ros2 topic echo /fixed_route/status
```

`/fixed_route/path` 是完整参考路径，`/fixed_route/target` 是当前前视目标，`/fixed_route/reached` 在完整路线结束且确认停稳后发布 true。默认速度上限 0.3m/s，记录的有效低速会保留。参数可通过 `route_params:=/绝对路径/fixed_route.yaml` 指定。

路线跟踪使用单调弧长进度，在闭环和交叉点附近只投影到当前局部路段，避免跳到路线末尾。曲率上限由本车实测最小转弯半径 0.69m 限制；不发原地转向命令。多段换向必须先确认停车再继续，短段停车容差随段长收紧。

遇障时检查完整车身沿预计刹停轨迹的扫掠范围，停车等待原路线恢复；本入口不生成偏离录点的绕行路径。定位、扫描、里程计失效，用户重设初始位姿、急停或手动接管时，任务暂停，恢复数据后仍需显式 `resume`。传感器实际盲区不会因软件检查消失，应按本车已有雷达安装条件验收。

默认仅允许前进。`allow_reverse:=true` 可加载倒车和换向路线，但仍要求扫描覆盖倒车方向；本车当前 `/scan` 屏蔽正后方 45°，会拒绝进入该盲区的倒车段。保持当前扫描配置时，应录制前进路线。

旧路线只能与原来同一坐标系的地图配对；新建或改变原点的地图需要重新录点。不能将旧车地图中的 `lane_0.csv` 直接叠到另一场地的新地图上。

## 与其他定位方案及回放的关系

NDT 与 FAST-LIO 全局定位、AMCL、Cartographer 定位共用 `map → odom` 和 `/odom` 等接口，运行时选择一个定位入口。`fixed_route_navigation.launch.py` 不启动 Nav2 的控制器；运行该 CSV 跟踪时也应停止其他自动导航任务。

已有外部里程计和 `odom → base_link` 时，使用 `odom_source:=external`，避免重复启动 EKF。回放传感器/里程计时可以这样启动：

```bash
ros2 launch chassis_nav2 ndt_localization.launch.py \
  pcd_map:=/绝对路径/map.pcd start_sensors:=false \
  odom_source:=external use_sim_time:=true
# 另一终端 ros2 bag play <bag路径> --clock；bag 需有点云、/odom 及其 TF。
```

若 bag 本身包含 `/scan`，再加 `start_scan:=false`；不要同时回放原系统的 `map → odom`。回放模式不允许 `enable_control:=true` 接管实车。

## 验证

```bash
cd /home/jkbot/Remote_ctrol/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --packages-select msf_localizer fixed_route_navigation --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select msf_localizer fixed_route_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/msf_localizer --verbose
colcon test-result --test-result-base build/fixed_route_navigation --verbose
cd ..
python3 scripts/verify_ndt_fixed_route.py
```

离线联调脚本固定使用本机环回、`ROS_DOMAIN_ID=87`，生成临时三维地图和模拟轮式运动，实际启动 NDT C++ 进程，经过真实 ROS 2 话题和服务验证完整链路，不启动设备、不发布 `/ecu`。覆盖未初始化禁止行驶、传感器外参、地图持久 QoS、CSV 保存及覆盖保护、路线完成、遇障停车恢复、扫描过期、NDT 过期和手动接管。

2026-10-24 本机验证：新增包和 `chassis_nav2` 编译通过；60 项 Python 单元/启动配置/现有导航回归检查及 3 项 C++ 融合几何检查通过；完整离线联调通过。合成地图初次 NDT 收敛位置误差约 0.042m，仅代表该测试场景。完整 launch 在未启动硬件的隔离域中成功加载本机现有 PCD（下采样后 817810 点）。旧工作空间的真实 `lane_0.csv` 导入后为 1233 点、118.88m 的一个前进运动段，未复制或修改旧地图与路线文件。

这些检查不能代替当前地图和本车的现场 NDT 参数、转弯轨迹与停车验收。`msf_localizer/config/ndt.yaml` 的分数、分辨率、匹配频率及超时阈值需要结合现场点云验证。

算法接口参考：[PCL NDT 官方教程](https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html)，[ROS REP-105 坐标约定源文档](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。
