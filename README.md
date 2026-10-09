# 巡检机器人远程控制系统

实机开发从 [使用手册](实机开发使用手册.md) 开始，包含安装、标定、建图、定位、双里程计对比和 Nav2 启动。

这是一个基于 ROS 2 Humble 的智能巡检机器人远程控制项目，用于实机遥控、视频监控、识别、抓拍与巡检业务管理。2026-10-09 已清理 Gazebo 仿真、测试场地地图和模拟执行逻辑。具体清单见 [实机开发说明](实机开发说明.md)。

> 当前状态：已移植 FAST-LIO2 建图/定位、JD03 控制桥、三维雷达转扫描及 Nav2。新车安装外参、车体几何和底盘通信仍待实测确认，尚未实机验证。步骤见 [实机导航移植说明](实机导航移植说明.md)。巡检执行器仍待接入。

2026-10-10 已补齐独立 JD03 轮速里程计，可与 FAST-LIO 同时记录，并通过 `odom_source:=lio/wheel/wheel_imu` 选择 Nav2 的主里程计，见 [底盘里程计说明](底盘里程计说明.md)。

## 功能概览

- 浏览器虚拟摇杆和 W/S/A/D 远程控制。
- WebSocket 遥控网关，支持速度限幅、心跳检测和 1 秒无指令自动刹车。
- Nav2 定位、Hybrid-A* 规划、Regulated Pure Pursuit 控制和目标点导航接口，使用外部实测地图。
- MediaMTX + FFmpeg 视频链路：RTSP 推流，浏览器 WebRTC 播放。
- YOLOv8n/OpenVINO 行人检测、YuNet + SFace 人脸识别、HyperLPR3 车牌识别。
- 识别事件通过 ROS 话题推送到网页，并通过 HTTP 上报业务后台。
- 手动/自动抓拍、按日期归档、磁盘轮转和断网缓存补传。
- 巡检点、路线和巡检计划管理，支持保存一次性、每日和按分钟周期规则；执行功能待实机接入。
- SQLite 记录识别和抓拍结果，保留已有任务记录及 HTML 报告查询。

## 系统结构

```text
浏览器监控台
  ├─ WebSocket :8765 ──> robot_gateway ──> /cmd_vel_teleop ──> twist_mux ──> JD03
  ├─ WebSocket :8765 <── robot_gateway <── /odom、/scan、TF、识别事件
  ├─ WebRTC :8889 <──── MediaMTX <──── FFmpeg <──── 实际摄像头
  └─ HTTP :8888 <───── FastAPI + SQLite
                              ↑
                detector_node / capture_server
```

项目采用单机器人、局域网/蒲公英组网场景下的轻量架构，不依赖 MQTT。浏览器、业务后台和机器人节点可以部署在同一台机器上，也可以根据网络情况分开部署。

## 目录说明

```text
Remote_ctrol/
├── backend/                 # FastAPI 业务后端、SQLite 数据层、计划管理和报告生成
├── config/                  # 机器人控制和端口配置
├── deploy/                  # MediaMTX 配置
├── scripts/                 # 安装、推流和一键启动脚本
├── web/                     # 监控台、点位、计划、记录页面及地图资源
├── ros2_ws/src/
│   ├── robot_gateway/       # WebSocket 遥控/遥测网关
│   ├── robot_common/        # HTTP 上报器和断网缓存
│   ├── detector_node/       # 行人、人脸、车牌识别
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
- 识别节点依赖：OpenCV、NumPy、PyTorch、Ultralytics、HyperLPR3

安装基础工具及 ROS 接口依赖。仅运行网页和网关时，仍需 `nav2_msgs` 等消息包；完整实机导航依赖可用 `scripts/install_navigation_deps.sh` 安装：

```bash
cd ~/Remote_ctrol
sudo apt install -y python3-pip python3-opencv ffmpeg v4l-utils \
  python3-colcon-common-extensions ros-humble-nav2-msgs \
  ros-humble-tf2-ros ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-rviz2
```

安装业务后台和 ROS Python 节点依赖：

```bash
python3 -m pip install --user fastapi "uvicorn[standard]" \
  websockets PyYAML requests numpy python-multipart
```

如果要启用图像识别，还需要安装：

```bash
python3 -m pip install --user torch ultralytics hyperlpr3
```

识别模型已经放在 `ros2_ws/src/detector_node/models/`，包括 YOLOv8、YuNet 和 SFace 模型。人脸底库使用单独目录保存，不提交到 Git。

## 首次构建

当前项目的 ROS 2 工作区是 `ros2_ws`。首次构建或代码更新后执行：

```bash
cd ~/Remote_ctrol
bash scripts/install_navigation_deps.sh
bash scripts/build_navigation.sh
source /opt/ros/humble/setup.bash

# 抓拍、识别、音频按需另行构建。
cd ros2_ws
colcon build --symlink-install --packages-select \
  capture_server detector_node audio_player
source install/setup.bash
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

默认不开视频、识别或音频。接入实际 UVC 摄像头后，可按需开启：

```bash
ENABLE_VIDEO=1 ENABLE_DETECTOR=1 ENABLE_AUDIO=1 \
  bash scripts/start_robot_monitor.sh
```

网络摄像头可通过 `CAMERA_RTSP` 传入实际 RTSP 地址；流转发要求摄像头使用浏览器支持的 H.264 编码。如果视频流由其他进程提供，可保持 `ENABLE_VIDEO=0`，设置 `VIDEO_SOURCE`、`ENABLE_CAPTURE=1`、`ENABLE_DETECTOR=1` 使用该实际视频流。

`scripts/start_m1.sh` 保留为真实 UVC 摄像头 + 静态遥控页入口，没有业务 API。摄像头失败会报错；两个启动入口均已去掉测试视频回退。

启动后访问：

- 监控台：<http://localhost:8888/monitor.html>
- 巡检点：<http://localhost:8888/points.html>
- 路线计划：<http://localhost:8888/plans.html>
- 识别/抓拍记录：<http://localhost:8888/records.html>
- FastAPI 文档：<http://localhost:8888/docs>
- 健康检查：<http://localhost:8888/api/health>

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

- 显示地图、机器人位置、雷达点云和导航状态。
- 鼠标拖拽选择初始定位位姿。
- 鼠标拖拽选择 Nav2 目标点，支持取消导航。
- 虚拟摇杆、W/S/A/D 和急停。
- 实际摄像头视频按需开启/关闭。
- 手动抓拍和实时识别告警。

### 点位和路线

在 `points.html` 中维护巡检点坐标、朝向和到点动作：

```json
[
  {"type": "dwell", "sec": 10},
  {"type": "capture"},
  {"type": "tts", "text": "开始检查"}
]
```

在 `plans.html` 中按顺序组合点位为路线，再保存巡检计划规则。实机巡检执行器尚未接入，计划不会自动执行，“立即执行”接口返回 HTTP 501。可保存的规则：

- `once`：一次性执行，格式为 `YYYY-MM-DD HH:MM`
- `daily`：每天执行，格式为 `HH:MM`
- `interval_min`：按分钟周期执行，例如 `120`

### 记录和报告

`records.html` 显示识别记录、抓拍记录和统计信息。计划页面可查询已有任务和报告；清理后的后台不会自动生成模拟任务或成功报告。

## 实机建图与导航

尚未标定时先检查速腾和维特原始数据：

```bash
ros2 launch chassis_nav2 bringup.launch.py mode:=drivers
```

填写新车 `hardware.yaml` 后选择建图、定位或完整导航之一：

```bash
ros2 launch chassis_nav2 mapping.launch.py pcd_map:=/实际路径/site.pcd rviz:=true
ros2 launch chassis_nav2 localization.launch.py pcd_map:=/实际路径/site.pcd rviz:=true
ros2 launch chassis_nav2 navigation.launch.py \
  pcd_map:=/实际路径/site.pcd map:=/实际路径/site.yaml rviz:=true
```

统一入口使用实际时钟，默认启动速腾和维特，底盘控制默认关闭。完整流程、PCD 保存、栅格/网页地图导出及启用 JD03 的参数见 [实机导航移植说明](实机导航移植说明.md)。定位初始位姿通过 `/initialpose` 给定。

## ROS 2 接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | 网关手动速度，进入仲裁 |
| `/cmd_vel` | `geometry_msgs/Twist` | 仲裁后的速度，JD03 桥换算为 `/ecu` |
| `/emergency_stop` | `std_msgs/Bool` | 保持急停并取消导航 |
| `/odom` | `nav_msgs/Odometry` | 机器人里程计，用于遥测 |
| `/scan` | `sensor_msgs/LaserScan` | 三维雷达截取生成的平面扫描 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 网页下发的 FAST-LIO 地图初始车体位姿 |
| `/detection_events` | `std_msgs/String` | JSON 格式识别事件 |
| `/capture_request` | `std_msgs/String` | 抓拍请求，内容为标签 |
| `/capture_done` | `std_msgs/String` | JSON 格式抓拍结果 |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 网页目标点导航 |

## WebSocket 指令

网关监听 `0.0.0.0:8765`，网页连接格式为 `ws://<主机IP>:8765`。

```json
{"type": "cmd_vel", "linear": 0.5, "angular": 0.2}
{"type": "estop"}
{"type": "initial_pose", "x": 0.0, "y": 0.0, "yaw": 0.0}
{"type": "nav_goal", "x": 1.0, "y": 2.0, "yaw": 1.57}
{"type": "nav_cancel"}
{"type": "capture", "label": "manual"}
```

配置文件 `config/config.yaml` 中的 `max_linear`、`max_angular` 会限制远程指令幅度，`cmd_timeout` 控制失控刹车时间。默认值为：线速度 `0.5 m/s`、角速度 `1.0 rad/s`、超时 `1.0 s`。

## HTTP API 概览

- `GET /api/health`：服务健康检查。
- `GET/POST /api/points`、`PUT/DELETE /api/points/{id}`：巡检点 CRUD。
- `GET/POST /api/routes`、`PUT/DELETE /api/routes/{id}`：路线 CRUD。
- `GET/POST /api/plans`、`PUT/DELETE /api/plans/{id}`：计划 CRUD。
- `POST /api/plans/{id}/run`：执行入口暂未接入，对存在的计划返回 HTTP 501。
- `GET /api/plans/tasks`、`GET /api/plans/tasks/{id}`：查询任务记录。
- `GET /api/plans/report/{id}`：生成或查看巡检报告。
- `GET /api/records/detections`：查询识别记录。
- `GET /api/records/captures`：查询抓拍记录。
- `GET /api/records/summary`：查询统计信息。
- `POST /api/records/detections`、`POST /api/records/captures`：写入识别和抓拍记录。

完整请求参数和响应结构可以在 `/docs` 查看。

## 数据和配置

业务后端根据本项目所在目录保存数据；ROS 节点由启动脚本设置 `ROBOT_PROJECT_ROOT`（手动运行时默认 `~/Remote_ctrol`）。运行时目录：

```text
~/Remote_ctrol/
├── data/
│   ├── robot.db             # FastAPI 主数据库
│   ├── detector_cache.db    # 识别上报失败缓存
│   └── capture_cache.db     # 抓拍上报失败缓存
├── web/captures/YYYY-MM-DD/ # 抓拍和识别图片
├── web/reports/             # HTML 巡检报告
└── face_db/                 # 人脸底库图片，不入 Git
```

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

手动启动 ROS 节点时可设置 `export ROBOT_PROJECT_ROOT=/实际路径/Remote_ctrol`；数据库、抓拍、报告和音频需使用同一项目目录。旧 WSL 目录中的历史数据不会自动迁移。

## 当前已知限制

1. 实机巡检执行器尚未接入；点位、路线和计划可管理，自动调度及计划执行暂停。
2. JD03 Twist/Ecu 桥和轮速反馈已接入代码；需核对实车 CAN 参数、转向符号、速度反馈与刹车。最大控制速度初始为 0.3m/s。
3. 已接入网页遥控/自主导航仲裁和急停；厂家手持遥控的权限逻辑仍需实车确认。
4. 电池反馈尚未接入，页面显示“未接入”和空值。
5. 统一使用 `base_link`。新车外参、轴距、转角与 footprint 未知，配置确认前对应功能不会启动；栅格投影质量需检查。
6. 仿真地图已删除。配置实测网页地图的方法见 [地图说明](web/maps/README.md)。无地图时禁止地图点选。
7. 识别默认使用 CPU 推理，性能取决于处理器和视频分辨率；人脸底库为空时只能识别为未登记人员或普通行人。
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
- [实机开发说明与清理清单](实机开发说明.md)
- [实机导航移植与启动步骤](实机导航移植说明.md)
- [底盘轮速里程计、对比与切换](底盘里程计说明.md)
- [开发计划与里程碑](开发计划.md)
- [巡检机器人远程控制方案](巡检机器人远程控制方案（蒲公英组网版）.md)

## 许可证与数据说明

项目中的部分厂家驱动、SDK、模型和第三方组件遵循各自许可证。人脸底库、抓拍图片、数据库和报告属于运行时数据，默认不提交到 Git；部署和分发前请确认相关模型、SDK 及个人数据的使用权限。
