# 巡检机器人远程控制系统

实机开发从 [使用手册](实机开发使用手册.md) 开始，包含安装、标定、建图、定位、双里程计对比和 Nav2 启动。

界面操作见[控制界面详细使用手册](控制界面详细使用手册.md)，按本地 Qt 和网页端逐项说明入口、操作步骤、成功标志及常见提示；现场验收使用[实车功能测试清单](实车功能测试清单.md)。

这是一个基于 ROS 2 Humble 的智能巡检机器人远程控制项目，用于实机遥控、视频监控、识别、抓拍与巡检业务管理。2026-10-09 已清理 Gazebo 仿真、测试场地地图和模拟执行逻辑。具体清单见 [实机开发说明](实机开发说明.md)。

> 当前状态：3D（FAST-LIO）与 2D（SLAM Toolbox 建图、Cartographer）三套建图导航链路已有实车记录；SLAM Toolbox 定位端现已切换 AMCL，完成隔离模拟验证，实车效果待验收。此前已完成轮速里程计卷尺标定和 Nav2 到点测试。三套方案的使用命令见 [建图导航命令](建图导航命令.md)（3D）和 [2D 建图导航](2d建图导航.md)（两套 2D）。本地路线执行、一键返回待命点和网页任务入口已接入 `robot_patrol`。

轮速里程计可与 FAST-LIO 同时记录，并通过 `odom_source:=lio/wheel/wheel_imu` 选择 Nav2 的主里程计，见 [底盘里程计说明](底盘里程计说明.md)。

## 功能概览

网页端没有账号/密码认证，只适用于受控工场内网或隔离 VPN。禁止将 8888/8765/8889 直接映射到公网；急停仍应在现场安全回路中保持独立有效。

- 浏览器二维地图、雷达、机器人位姿/速度、全局/局部路径和 Nav2 只读状态。
- 单网页控制权、本地桌面最高优先级、控制心跳超时和断网立即停车。
- 浏览器虚拟摇杆、W/S/A/D 和急停；网关只在网页取得控制权后创建 `/cmd_vel_teleop`。
- 本地已有路线立即执行、暂停/继续/取消；一键返回当前地图待命点。
- MediaMTX + FFmpeg 视频链路：RTSP 推流，浏览器 WebRTC 播放。
- 全上位机视觉：YuNet 人脸抓拍、OpenVINO YOLO26n 行人检测、LPD-YuNet 车牌检测 + 字符识别，三类独立启停。
- 网页只开关已运行时的人脸抓拍、行人、车牌检测，不启动视觉进程。
- 云台方向/变倍按住控制，气体检测实时只读显示。
- 网页显示底盘 BMS 电量、电压、电流和充电状态，失联后清空过期读数。
- 云台自转与到点巡检联动：到点停转、预置位拍照，离点恢复；最后一点正常结束仅恢复原已开启的自转。
- 地图激活时自动导出网页 PNG/meta，支持 Nav2 地图 `origin` 的任意朝向。
- 网页地图选点或选择已有巡检点，一键到达、转云台、抓拍识别，沿用巡航执行器和 guard 联锁。
- 任务结束自动生成巡检报告，网页查看、下载含内嵌图片的 HTML 或结构化 JSON。
- 人脸、行人、车牌图片与坐标自动上传、去重保存和分页查询，断网后补传。
- 网页上传音频后由机器人扬声器播放，支持停止及播放状态反馈。
- 网页新增、编辑和保存点位与路线，固定路网路线支持预览后启动；修改由网关调用巡航服务，HTTP 后台仍只读主库。
- 网页独立手动抓拍并保存、查询；双向对讲由网页开启，使用独立麦克风和音响。
- 音频与接收的视觉记录单独保存。不提供网页建图/导航启动、初始位姿或周期计划。

新增业务的更新、使用和数据目录见[网页巡检业务说明](web/巡检业务使用说明.md)。

## 系统结构

```text
浏览器监控台 / 巡检任务 / 巡检记录
  ├─ WebSocket :8765 ──> robot_gateway ──> /cmd_vel_teleop（仅取得控制权时存在）
  │                         ├─> /patrol/run_route (RunRoute)
  │                         └─> /patrol/return_home (ReturnHome)
  ├─ WebSocket :8765 <── 地图/雷达/TF/位姿、巡航状态、视觉/云台/气体状态
  ├─ WebRTC :8889 <──── MediaMTX <──── FFmpeg <──── 实际摄像头
  └─ HTTP :8888 <───── FastAPI 只读访问 robot_patrol SQLite
                              ↑
                patrol_node / inspection_recorder 写入业务表
```

项目采用单机器人、局域网/蒲公英组网场景下的轻量架构，不依赖 MQTT。浏览器、业务后台和机器人节点可以部署在同一台机器上，也可以根据网络情况分开部署。

## 目录说明

```text
Remote_ctrol/
├── backend/                 # FastAPI 静态服务和只读业务 API
├── config/                  # 机器人控制和端口配置
├── deploy/                  # MediaMTX 配置
├── scripts/                 # 安装、推流和一键启动脚本
├── web/                     # 监控台、巡检任务、巡检记录及自动导出地图资源
├── maps/                    # 三套方案的地图：fastlio / slam_toolbox / cartographer
├── desktop_ui/              # Qt + RViz2 桌面端（独立构建，含六种建图/导航显示配置）
├── ros2_ws/src/
│   ├── robot_gateway/       # WebSocket 控制权、任务代理和只读遥测网关
│   ├── robot_patrol/        # 本地点位/路线执行、返航、guard 和巡检摘要
│   ├── robot_common/        # HTTP 上报器和断网缓存
│   ├── detector_node/       # 上位机人脸/行人/车牌检测与号码识别
│   ├── vision_events/       # 独立视觉执勤、本地图片与小车位姿记录
│   ├── vision_interfaces/   # 历史查询和可选地图标记接口
│   ├── gas_detector/        # 独立气体检测与设备执勤 UI 接口（只读采集）
│   ├── capture_server/      # 视频抓拍服务
│   ├── chassis_description/ # 机器人 URDF 和网格模型
│   ├── chassis_nav2/        # JD03、统一实机启动、TF、控制仲裁及 Nav2
│   ├── FAST_LIO_LOCALIZATION_ROS2/ # FAST-LIO2 建图和 PCD 全局定位
│   ├── pcd2pgm/             # 同坐标 PCD 转导航栅格
│   └── 厂家驱动及 OCR 包    # 雷达、IMU、海康相机、底盘等扩展包
└── *.md                     # 启动、交接、开发计划和方案文档
```

## 环境要求

在实际机器人或工控机的 Ubuntu 22.04 / ROS 2 Humble 环境运行，按硬件准备驱动和底盘接口。

- Ubuntu 22.04（按实际机器人选择 ARM64 或 AMD64）
- ROS 2 Humble
- Nav2、Open3D、RViz2、twist_mux（联调定位/导航时需要）
- Python 3、pip、FFmpeg、V4L2
- MediaMTX 可执行文件：放置在 `bin/mediamtx`
- 识别节点依赖：OpenCV、NumPy、OpenVINO、ONNX Runtime（CPU；模型随包安装）

安装基础工具及 ROS 接口依赖。仅运行网页和网关时，仍需 `nav2_msgs` 等消息包；完整实机导航依赖可用 `scripts/install_navigation_deps.sh` 安装：

```bash
cd ~/Remote_ctrol
sudo apt install -y python3-pip python3-opencv ffmpeg v4l-utils libportaudio2 \
  python3-colcon-common-extensions ros-humble-nav2-msgs \
  ros-humble-tf2-ros ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-rviz2
```

安装业务后台和 ROS Python 节点依赖：

```bash
python3 -m pip install --user fastapi "uvicorn[standard]" \
  websockets PyYAML requests Pillow sounddevice 'numpy==1.26.4' 'transforms3d==0.4.2' python-multipart
```

NumPy 1.26.4 需要配套 `transforms3d==0.4.2`。Ubuntu 自带的 transforms3d 0.3.1 使用已移除的 `np.float`，会让 FAST-LIO 的 ICP 节点在导入 `tf_transformations` 时退出，随后 Nav2 持续提示 `map` 坐标系不存在。导航和视觉安装脚本已固定兼容版本；已有环境修复后需重新启动导航，并重新设置初始位姿。

如果要启用图像识别，还需要安装：

```bash
./scripts/install_vision_deps.sh
```

已有模型资源位于 `ros2_ws/src/detector_node/models/`。当前使用 YuNet、YOLO26n OpenVINO、LPD-YuNet 和独立车牌字符模型，不加载历史 SFace 或人脸身份库；运行时无需 PyTorch 和 HyperLPR3 Python 包。模型来源、校验和导出复现见 [模型清单](ros2_ws/src/detector_node/models/README.md)。CPU 依赖安装使用 `./scripts/install_vision_deps.sh`；独立启动、Qt 按钮、记录及验证见 [视觉执勤说明](ros2_ws/src/vision_events/README.md)。

## 首次构建

当前项目的 ROS 2 工作区是 `ros2_ws`。首次构建或代码更新后执行：

```bash
cd ~/Remote_ctrol
bash scripts/install_navigation_deps.sh
bash scripts/build_navigation.sh
source /opt/ros/humble/setup.bash

# 构建网页巡检、识别服务、音频及所需消息接口。
bash scripts/build_web_patrol.sh
# 首次使用独立视频抓拍或上位机检测时，另行构建这两个节点。
cd ros2_ws
colcon build --symlink-install --packages-select capture_server detector_node
cd ..
source ros2_ws/install/setup.bash
```

如果使用了新的终端，需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/Remote_ctrol/ros2_ws/install/setup.bash
```

## 一键启动

实机监控入口负责业务后端和遥控网关，底盘、传感器、TF 和定位/导航用本项目 `chassis_nav2` 在另一终端启动。

```bash
cd ~/Remote_ctrol
bash scripts/start_robot_monitor.sh
```

默认开启音频播放器；安装了 `bin/mediamtx` 时自动开启网页视频和独立抓拍，缺少二进制时保留其他网页功能。视频默认复用海康相机 YAML 中的地址、通道和登录信息，统一转为无音轨的 H.264；相机断流时自动重连。已有独立播放器时可设 `ENABLE_AUDIO=0`，已有视频服务时可设 `ENABLE_VIDEO=0`。使用 UVC 摄像头并启用检测时：

```bash
ROBOT_VIDEO_INPUT=uvc ENABLE_DETECTOR=1 bash scripts/start_robot_monitor.sh
```

相机配置可通过 `ROBOT_CAMERA_CONFIG` 指定其他 YAML；`CAMERA_RTSP` 可覆盖实际视频源，`ROBOT_VIDEO_STREAM` 可选择海康码流（1 主码流、2 子码流）。相机 H.265 会转码为 H.264，无需修改相机设备编码。如果视频流由其他进程提供，可设 `ENABLE_VIDEO=0`，通过 `VIDEO_SOURCE`、`ENABLE_CAPTURE=1`、`ENABLE_DETECTOR=1` 使用该视频流。网页开启前检查 `/api/video/status`，服务未启动或没有相机画面时显示明确提示。

`scripts/start_m1.sh` 保留为真实 UVC 摄像头 + 静态遥控页入口，没有业务 API。摄像头失败会报错；两个启动入口均已去掉测试视频回退。

启动后访问：

- 监控台：<http://localhost:8888/monitor.html>
- 巡检任务：<http://localhost:8888/tasks.html>
- 巡检记录：<http://localhost:8888/records.html>
- FastAPI 文档：<http://localhost:8888/docs>
- 健康检查：<http://localhost:8888/api/health>

监控、任务和记录统一在 `/console.html` 控制台中切换；上述旧地址会自动打开对应页面。站内切页保留同一个连接和控制权，因此启动巡航后可以切到监控台看地图、视频或查看记录。切页会结束本次手动速度和云台按住操作，不会取消自动巡航。关闭或刷新整个控制台、真正断网、心跳超时或本地接管，仍会释放网页控制权并取消网页任务，重连后不自动续跑。

任务圈数在前后端校验为 `0～100000` 的整数；拒绝或提交失败后可以直接重试。气体停止、异常或过期后读数显示 `—`；地图每 2 秒检查版本，首次加载失败会自动重试，版本变化后重新加载；云台按住期间持续续租，松开、切页或失去焦点后停止。模块和回归测试说明见 [网页端说明](web/README.md)。

网页、网关和 `robot_patrol` 必须使用同一个业务数据库。默认路径为 `$HOME/.local/share/robot_patrol/patrol.sqlite3`；如需修改，在所有相关终端统一设置 `ROBOT_PATROL_DB`。`scripts/start_robot_monitor.sh` 还会设置默认的 `ROBOT_WEB_MAP_DIR=$ROOT/web/maps`。`robot_patrol` 的启动命令示例：

```bash
source /opt/ros/humble/setup.bash
source ~/Remote_ctrol/ros2_ws/install/setup.bash
ros2 launch robot_patrol patrol.launch.py
```

按 `Ctrl+C` 可以停止一键启动脚本拉起的后台进程。实机监控入口中任一组件退出时会结束本次启动，需根据终端错误修复组件后重启。

### MediaMTX 准备

启动脚本会执行 `bin/mediamtx`，该二进制文件被 `.gitignore` 忽略，需要按照运行平台自行准备：

```text
bin/mediamtx
deploy/mediamtx.yml
```

其中 `deploy/mediamtx.yml` 配置了：

- RTSP 推流入口：`8554`
- WebRTC 播放入口：`8889`
- WebRTC 媒体传输：`8189`（UDP / TCP）；MediaMTX 状态 API 仅监听本机 `127.0.0.1:9997`。
- 流名称：`cam`

浏览器实际播放地址为 `http://<主机IP>:8889/cam/`，默认视频源为 `rtsp://127.0.0.1:8554/cam`。

## D435 摄像头在 WSL2 联调（可选）

Windows 管理员 PowerShell 中执行：

```powershell
usbipd list
usbipd bind --busid <BUSID>       # 首次绑定时执行
usbipd attach --wsl --busid <BUSID>
```

进入 WSL 后执行：

```bash
sudo modprobe uvcvideo
cd ~/Remote_ctrol
bash scripts/d435_attach_fix.sh
```

默认推流脚本使用 `/dev/video4` 的彩色图像，默认分辨率为 `640x480`。通过 usbipd 透传时，720p YUYV 带宽较高，优先使用 640x480：

```bash
DEV=/dev/video4 SIZE=640x480 FPS=30 \
  bash scripts/push_d435_wsl.sh
```

上述脚本采集的是真实 D435 图像；原生 Linux 实机可使用 `push_camera.sh`。没有有效摄像头或 RTSP 流时应排查视频源，项目不再生成测试画面。

## Web 页面功能

### 监控台 `monitor.html`

- 二维地图、雷达、机器人位置/朝向/速度、Nav2 目标数量和巡航任务状态。
- 全局/局部路径显示；支持地图选点与已有点位的一键到达巡检，由巡航执行器统一授权。
- 上传音频、一键机器人播报、停止及播放状态。
- 单网页控制权、虚拟摇杆、W/S/A/D、急停和 1 秒控制心跳超时。网页急停会触发本地联锁并释放网页控制权，解除急停必须在现场/本地桌面确认安全后操作。
- 实际摄像头视频按需开启/关闭。
- 视觉识别开关：人脸抓拍、行人检测、车牌检测；只控制本地已启动节点。
- 云台方向/变倍按住控制、停止、位置读取和自动巡航锁定状态。
- 气体实时数值、单位、原始寄存器、设备报警、连接和新鲜度；不写数据库。
- 电池电量、电压、电流、充电状态及反馈新鲜度；默认订阅 `/battery_status`，超过 5 秒清空旧读数。

### 巡检任务 `tasks.html`

- 显示当前活跃地图、地图版本一致性、有效返航点和本地已有路线。
- 点位和路线可新增、编辑、删除；路线支持点序调整和停留时间设置，保存时检查版本冲突。
- 自由路线可立即执行；固定路网路线先预览，再携带计划摘要启动。可设置圈数，支持暂停/继续/取消。
- 支持一键返回当前地图待命点；如果路线正在执行，会先取消路线、等待停车再返航。
- 不支持切换地图、创建定时计划或设置初始位姿。
- 路线地图/路线版本不一致时由网页和 `robot_patrol` 双重拒绝。

### 巡检记录 `records.html`

- 显示巡检摘要、每次任务自动报告、到点识别结果、人脸/行人/车牌完整图片与坐标，支持筛选和分页。
- 报告提供 HTML / JSON 下载，HTML 内嵌图片，可离线打开或打印保存 PDF。
- 不记录气体、云台、控制权、断网、视觉服务错误或任务状态事件日志。
- 旧网页 `data/robot.db` 和测试数据不再使用，不做迁移。

### 本地待命返航点

本地“路线巡航”窗口可把当前停稳位姿保存为当前地图返航点，也可清除和“一键返回”。主窗口标题栏同样提供“一键返回”。网页不能修改返航点。地图 revision 改变后旧返航点自动失效。

## 实机建图与导航（3D / 2D）

本车有三套**互斥**的建图定位方案，运行时三选一（都会抢 `map->odom` TF 和 `/scan`，不可同时启动）：

| 方案 | 定位原理 | 地图目录 | 建图 launch | 定位+导航 launch |
| --- | --- | --- | --- | --- |
| 3D：FAST-LIO | FAST-LIO + ICP 全局定位 | `maps/fastlio/`（map.pcd + map.pgm/yaml） | `slam.launch.py` | `navigation.launch.py odom_source:=wheel_imu enable_control:=true` |
| 2D：SLAM Toolbox + AMCL | AMCL 栅格地图定位 | `maps/slam_toolbox/`（map.pgm/yaml） | `slam_2d_mapping.launch.py` | `amcl_localization.launch.py` |
| 2D：cartographer | 子图匹配纯定位 | `maps/cartographer/`（map.pbstream + map.pgm/yaml） | `cartographer_2d_mapping.launch.py` | `cartographer_2d_localization.launch.py` |

### 公共前提

所有终端先加载环境；终端 1 先起底盘驱动并保持运行（建图/定位/导航都需要它）：

```bash
source /opt/ros/humble/setup.bash
source ~/Remote_ctrol/ros2_ws/install/setup.bash
ros2 launch chassis_driver chassis_driver.launch.py
```

2D 各 launch 的公共参数：`enable_control:=true`（接管底盘）、`rviz:=true`、`start_sensors`（默认开雷达/IMU）、`start_chassis`（默认 false，厂家驱动独立启动）、`start_nav2`（2D 建图默认 false 不起 Nav2，边建图边导航时传 true）。

切换方案前先 Ctrl+C 停干净，可用 `ros2 node list | sort | uniq -c` 确认无残留双栈。

### 方案一：3D FAST-LIO（推荐，无起点限制）

```bash
# 终端 2：建图（输出固定为 maps/fastlio/map.pcd，旧图自动备份）
ros2 launch chassis_nav2 slam.launch.py
# 终端 3：键盘遥控，低速匀速走回环
ros2 run chassis_nav2 jd03_keyboard.py
# 存图（建图保持运行时执行）
ros2 service call /map_save std_srvs/srv/Trigger '{}'

# PCD 转栅格地图（导航必做）+ 清理噪点
ros2 launch pcd2pgm pcd2pgm_launch.py z_min:=-0.10 z_max:=2.0 rviz:=true
ros2 run nav2_map_server map_saver_cli \
  -t /projected_map -f ~/Remote_ctrol/maps/fastlio/map
python3 ~/Remote_ctrol/scripts/clean_map.py

# 定位 + 导航（全局 ICP 定位，RViz 给初始位姿后即可发目标点）
ros2 launch chassis_nav2 navigation.launch.py \
  odom_source:=wheel_imu enable_control:=true rviz:=true
```

详细步骤、初始位姿操作顺序和注意事项见 [建图导航命令](建图导航命令.md)。

### 方案二：SLAM Toolbox 建图 / AMCL 定位

```bash
# 建图
ros2 launch chassis_nav2 slam_2d_mapping.launch.py enable_control:=true rviz:=true
# 存图：map_saver_cli 保存 YAML/PGM 供 AMCL 定位；序列化地图可保留续建图，见 2d建图导航.md
# 定位 + 导航
ros2 launch chassis_nav2 amcl_localization.launch.py enable_control:=true rviz:=true
# 先用 2D Pose Estimate 给实际位置和朝向，等待定位与 Nav2 激活后再发目标
```

### 方案三：2D cartographer

```bash
# 建图
ros2 launch chassis_nav2 cartographer_2d_mapping.launch.py enable_control:=true rviz:=true
# 存图（finish_trajectory → write_state → 转 pgm 三步，见 2d建图导航.md）
# 定位 + 导航
ros2 launch chassis_nav2 cartographer_2d_localization.launch.py enable_control:=true rviz:=true
```

两套 2D 方案的存图命令、初始化约束和选型建议见 [2D 建图导航](2d建图导航.md)。AMCL 等待 `/initialpose`，可在当前实际位置初始化；旧 `slam_2d_localization.launch.py` 命令兼容转入 AMCL。Cartographer 仍需在建图起点附近启动，本机版本不支持 `/initialpose`。

### 里程计链与关键标定

三套方案共用同一条数据链（改一处三套生效）：

```text
底盘 /vehicle_status(100Hz)
  → jd03_odometry（双阿克曼模型，只用 cur_speed/cur_steer）→ /odom_raw
  → EKF robot_localization（融合 IMU 航向）→ /odom + odom->base_link TF
雷达 /velodyne_points
  → cloud_crop_box（裁掉车身支柱）→ pointcloud_to_laserscan → /scan
```

已实车标定/确认的关键参数（`chassis_nav2/config/hardware.yaml`）：

- `speed_scale: 0.29`：cur_speed 反馈与真实车速的比例，卷尺 2m 直线多次标定，残差约 -1.3% 由定位吸收。
- 旋转精度：原地满舵一圈位置差约 1.3cm、角度差约 1.1°。
- `/scan` 角度窗口 ±157.5°：正后方 45° 扇区打到车身设备产生幽灵障碍，已屏蔽。
- Nav2 普通导航终点容差（三套各自的 nav2 配置）：xy 0.10 m，yaw 50°。
- 普通导航使用 `chassis_nav2::AckermannPursuitController`：基于 Humble RPP，按路径顺序执行前进/倒车段，保持实际可达曲率，折返点停车后换向。规划半径 0.85m，底盘实测极限 0.69m 用于纠偏；有效路径保留，目标改变、路径受阻或跟踪失败时重规划。
- 普通导航直线指令上限 0.6 m/s，倒车 0.3 m/s；控制器提前按方向限速，速度平滑按比例变化，JD03 桥限速时也保留曲率。底盘桥换向前确认低速 0.25s，换挡后等待 0.20s；起步等待实际转角接近目标（相差不超过 3°），停车保留原档位与转角。上述新跟踪与提速已通过隔离仿真，仍需实车复测。

尚未标定时可先只开驱动检查速腾和维特原始数据：`ros2 launch chassis_nav2 bringup.launch.py mode:=drivers`。外参、PCD 投影和网页地图导出的更多细节见 [实机导航移植说明](实机导航移植说明.md)。

## ROS 2 接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | 网关手动速度，进入仲裁 |
| `/cmd_vel` | `geometry_msgs/Twist` | 仲裁后的速度，JD03 桥换算为 `/ecu` |
| `/emergency_stop` | `std_msgs/Bool` | 保持急停并取消导航 |
| `/odom` | `nav_msgs/Odometry` | 机器人里程计，用于遥测 |
| `/scan` | `sensor_msgs/LaserScan` | 三维雷达截取生成的平面扫描 |
| `/patrol/status` | `patrol_interfaces/PatrolStatus` | 本地路线/返航任务状态 |
| `/patrol/run_route` | `patrol_interfaces/action/RunRoute` | 路线执行 Action |
| `/patrol/return_home` | `patrol_interfaces/action/ReturnHome` | 一键返回 Action |
| `/patrol/operator_heartbeat` | `patrol_interfaces/OperatorHeartbeat` | 操作端心跳（本地/网页） |
| `/vision/detector_status` | `std_msgs/String` | 上位机人脸/行人/车牌模型、视频与开关状态 |
| `/detector_node/set_parameters` | `rcl_interfaces/SetParameters` | 三类检测独立开关；Qt 使用原子参数服务 |
| `/gas_detector/data` | `std_msgs/String` | 气体实时只读数据 |
| `/gas_detector/status` | `std_msgs/String` | 气体采集连接状态 |
| `/ptz_patrol/status` | `std_msgs/String` | 云台自动巡航状态 |

## WebSocket 指令

网关监听 `0.0.0.0:8765`，网页连接格式为 `ws://<主机IP>:8765`。

网页必须先取得服务端随机 session token；除急停外，运动、路线、返航、云台和视觉开关命令都必须携带该 token。

```json
{"type": "control_claim", "request_id": "..."}
{"type": "control_heartbeat", "session_id": "web_...", "request_id": "..."}
{"type": "cmd_vel", "session_id": "web_...", "linear": 0.2, "angular": 0.0}
{"type": "estop"}
{"type": "route_start", "session_id": "web_...", "request_id": "...",
 "map_id": "...", "map_revision": 1, "route_id": "...",
 "route_revision": 1, "loop_count": 1}
{"type": "route_pause", "session_id": "web_...", "request_id": "..."}
{"type": "route_resume", "session_id": "web_...", "request_id": "..."}
{"type": "route_cancel", "session_id": "web_...", "request_id": "..."}
{"type": "return_home", "session_id": "web_...", "request_id": "...",
 "map_id": "...", "map_revision": 1}
{"type": "return_cancel", "session_id": "web_...", "request_id": "..."}
{"type": "ptz_jog", "session_id": "web_...", "action": "left"}
{"type": "ptz_stop", "session_id": "web_..."}
{"type": "vision_face_capture", "session_id": "web_...", "enabled": true}
{"type": "vision_person", "session_id": "web_...", "enabled": true}
{"type": "vision_plate", "session_id": "web_...", "enabled": true}
```

浏览器断开、控制心跳超过 1 秒、本地桌面启用操作或收到急停时：网关先移除手动速度发布者并停车；网页发起的路线/返航任务会被取消；本地发起的路线任务不由网关取消。

配置文件 `config/config.yaml` 中的 `max_linear`、`max_angular` 会限制远程指令幅度，`cmd_timeout` 控制失控刹车时间。默认值为：线速度 `0.5 m/s`、角速度 `1.0 rad/s`、超时 `1.0 s`。

## HTTP API 概览

- `GET /api/health`：后端、共享数据库和网页地图状态。
- `GET /api/local/state`：活跃地图、版本一致性、返航点和有效路线。
- `GET /api/local/maps`：本地已登记地图。
- `GET /api/local/maps/active`：当前活跃地图及网页 meta。
- `GET /api/local/maps/{map_id}/routes`：指定地图的本地只读路线。
- `GET /api/local/maps/{map_id}/home`：指定地图的有效待命点。
- `GET /api/inspection/records`：巡检摘要。
- `GET /api/inspection/reports`、`GET /api/inspection/reports/{run_id}`：报告列表和详情，`/download?format=html|json` 下载。
- `GET /api/inspection/results`：到点抓拍识别，`/{result_id}/image` 读取图片。
- `POST /api/records/detections`：幂等接收视觉事件及图片；`GET /api/vision/events`、`GET /api/vision/media/{event_id}` 查询。
- `GET/POST /api/audio/files`：已上传音频和上传；实际播放、停止通过有控制权的 WebSocket 命令执行。
- `GET /api/map/meta`：网页地图几何和版本，禁止缓存。

FastAPI 不直接执行运动控制或编辑点位/路线/计划。巡航主库由 `patrol_node` 和 `inspection_recorder` 写入，后端以 SQLite `mode=ro` 打开；后端只写独立的上传音频和视觉记录目录。旧 `data/robot.db` 不再使用。

完整请求参数和响应结构可以在 `/docs` 查看。

## 数据和配置

业务主库为本地 `robot_patrol` SQLite，网页只读打开：

```text
$HOME/.local/share/robot_patrol/patrol.sqlite3   # 默认主库，可由 ROBOT_PATROL_DB 覆盖
~/Remote_ctrol/web/maps/active.png               # 地图激活时自动导出
~/Remote_ctrol/web/maps/meta.json                # map_id/revision/origin/sha256
~/Remote_ctrol/data/vision/events.sqlite3        # 视觉模块内部媒体/事件索引
~/Remote_ctrol/data/vision/media/                # 视觉事件图片
~/Remote_ctrol/data/web_records/                 # 后台接收的视觉事件和图片
~/Remote_ctrol/data/audio/                       # 网页上传音频，供机器人播放器读取
~/Remote_ctrol/captures/hk_faces/                # 旧海康原生人脸文件（当前默认不补扫）
```

- 网页、网关和 `robot_patrol` 必须设置同一个 `ROBOT_PATROL_DB`。
- 网页地图目录可由 `ROBOT_WEB_MAP_DIR` 覆盖，默认是项目 `web/maps`。
- 旧 `data/robot.db`、网页测试数据、点位/路线/计划写接口已废弃，不做迁移。
- 巡检摘要只保存巡检日期、关联路线、人脸抓拍次数、行人次数和去重车牌。

主要配置位于 `config/config.yaml`：

```yaml
control:
  max_linear: 0.5
  max_angular: 1.0
  cmd_timeout: 1.0

ports:
  ws: 8765
  web: 8888

video:
  stream_name: cam
  encoder: libx264
```

手动启动 ROS 节点时可设置 `export ROBOT_PROJECT_ROOT=/实际路径/Remote_ctrol`；如修改业务数据库路径，需在所有终端同步设置 `ROBOT_PATROL_DB`。旧 WSL 目录和旧网页数据库不会自动迁移。

## 当前已知限制

1. 网页支持点位/路线维护、固定路线预览执行、一键到达巡检及返航，定时/周期计划尚未提供。新增抓拍、对讲和任务流程仍需现场验收。
2. JD03 转向符号和速度反馈已实车核对，轮速里程计已标定（`speed_scale: 0.29`）；普通导航前进上限 0.6 m/s、倒车 0.3 m/s。新跟踪与换挡处理已经隔离仿真验证，实车效果待复测，不能把指令上限当作实测速率。
3. 已接入单网页控制权、本地桌面优先、心跳超时和断网停车；厂家手持遥控的权限逻辑仍需实车确认。
4. Qt 和网页已接入 JD03 `/battery_status`；没有有效底盘反馈时显示等待/过期和空值，实际电量与充电状态需现场核对。
5. 统一使用 `base_link`；外参、轴距、转角与 footprint 已确认（`hardware.yaml` 各段 `confirmed: true`）。
6. 地图在本地激活时自动导出网页 PNG/meta，支持非零 `origin_yaw`。地图版本不一致时网页任务页禁止执行。说明见 [地图说明](web/maps/README.md)。
7. 人脸/行人/车牌均在上位机 CPU 推理，性能取决于处理器、视频分辨率和限帧配置；人脸只检测并保存小图，不做身份判断。
8. `bin/mediamtx` 不在 Git 中，需要准备实际运行平台的版本。

## 常见排查

### 页面打开但显示未连接

确认网关是否启动，并检查 WebSocket 端口：

```bash
ros2 node list | grep robot_gateway
ss -lntp | grep -E '8765|8888|8889'
```

### 页面没有机器人位置或雷达点

网关需要同时收到 `/odom`、`/scan` 和到 `base_link` 的 TF。地图上的机器人位置需要 FAST-LIO 全局匹配成功；先确认实际传感器、里程计和定位节点均已启动：

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic list
```

### 页面没有视频

确认 MediaMTX 正在运行，且 RTSP 流存在：

```bash
ffplay rtsp://127.0.0.1:8554/cam
```

D435 透传场景优先检查 `/dev/video4` 和 640x480 YUYV 格式。

### FastAPI 无法启动或页面 404

确认当前目录、Python 依赖和项目路径：

```bash
cd ~/Remote_ctrol
python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8888 --app-dir .
```

### ROS 2 找不到自定义节点

重新加载 ROS 2 和工作区环境：

```bash
source /opt/ros/humble/setup.bash
source ~/Remote_ctrol/ros2_ws/install/setup.bash
ros2 pkg list | grep -E 'robot_gateway|detector_node|capture_server|chassis_nav2'
```

## 相关文档

- [实机开发使用手册](实机开发使用手册.md)
- [启动命令速查](启动.md)
- [建图导航命令（3D FAST-LIO）](建图导航命令.md)
- [2D 建图导航（SLAM Toolbox + AMCL / Cartographer）](2d建图导航.md)
- [实机开发说明与清理清单](实机开发说明.md)
- [实机导航移植与启动步骤](实机导航移植说明.md)
- [底盘轮速里程计、对比与切换](底盘里程计说明.md)
- [阿克曼停车改造计划](阿克曼停车改造计划.md)
- [开发计划与里程碑](开发计划.md)
- [巡检机器人远程控制方案](巡检机器人远程控制方案（蒲公英组网版）.md)
- [Qt 桌面端说明](desktop_ui/README.md)

## 许可证与数据说明

项目中的部分厂家驱动、SDK、模型和第三方组件遵循各自许可证。人脸底库、抓拍图片、数据库和报告属于运行时数据，默认不提交到 Git；部署和分发前请确认相关模型、SDK 及个人数据的使用权限。
