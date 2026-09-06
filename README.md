# 巡检机器人远程控制系统

这是一个基于 ROS 2 Humble 的智能巡检机器人远程控制项目，面向 WSL2 仿真环境、x64 工控机和后续真车部署。项目目前已经打通了仿真底盘、Nav2 导航、浏览器远程控制、视频监控、图像识别、抓拍存档和巡检业务后台等主要链路。

> 当前状态：M1 视频与遥控、M2 业务后台、仿真 Nav2 链路和识别一期已完成；真车底盘适配、无人值守真实巡检、TTS、对讲和 3D 导航仍在开发中。

## 功能概览

- 浏览器虚拟摇杆和 W/S/A/D 远程控制。
- WebSocket 遥控网关，支持速度限幅、心跳检测和 1 秒无指令自动刹车。
- Gazebo Classic 阿克曼底盘仿真，集成 2D 激光雷达和地图。
- Nav2 定位、Hybrid-A* 规划、Regulated Pure Pursuit 控制和目标点导航。
- MediaMTX + FFmpeg 视频链路：RTSP 推流，浏览器 WebRTC 播放。
- YOLOv8n/OpenVINO 行人检测、YuNet + SFace 人脸识别、HyperLPR3 车牌识别。
- 识别事件通过 ROS 话题推送到网页，并通过 HTTP 上报业务后台。
- 手动/自动抓拍、按日期归档、磁盘轮转和断网缓存补传。
- 巡检点、路线和巡检计划管理，支持一次性、每日和按分钟周期执行。
- SQLite 记录识别、抓拍和任务结果，并自动生成 HTML 巡检报告。

## 系统结构

```text
浏览器监控台
  ├─ WebSocket :8765 ──> robot_gateway ──> /cmd_vel
  ├─ WebSocket :8765 <── robot_gateway <── /odom、/scan、TF、识别事件
  ├─ WebRTC :8889 <──── MediaMTX <──── FFmpeg <──── 摄像头/D435/测试视频
  └─ HTTP :8888 <───── FastAPI + SQLite
                              ↑
                detector_node / capture_server
```

项目采用单机器人、局域网/蒲公英组网场景下的轻量架构，不依赖 MQTT。浏览器、业务后台和机器人节点可以部署在同一台机器上，也可以根据网络情况分开部署。

## 目录说明

```text
Remote_ctrol/
├── backend/                 # FastAPI 业务后端、SQLite 数据层、计划调度和报告生成
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
│   ├── chassis_gazebo/      # Gazebo 仿真底盘和传感器
│   ├── chassis_nav2/        # SLAM、AMCL、Nav2 配置和地图
│   └── 厂家驱动及 OCR 包    # 雷达、IMU、海康相机、底盘等扩展包
├── patrol.*                 # 根目录地图/建图数据
└── *.md                     # 启动、交接、开发计划和方案文档
```

## 环境要求

推荐在 WSL2 Ubuntu 22.04 x64 中运行仿真。真车或 x64 工控机运行时，ROS 2 节点代码可以复用，但需要替换硬件驱动和底盘接口。

- Ubuntu 22.04 / WSL2 x64
- ROS 2 Humble
- Gazebo Classic、Nav2、slam_toolbox、RViz2
- Python 3、pip、FFmpeg、V4L2
- MediaMTX 可执行文件：放置在 `bin/mediamtx`
- 识别节点依赖：OpenCV、NumPy、PyTorch、Ultralytics、HyperLPR3

安装仿真相关系统依赖：

```bash
cd ~/ros2Project/Remote_ctrol
bash scripts/install_sim.sh
sudo apt install -y python3-pip python3-opencv ffmpeg v4l-utils \
  python3-colcon-common-extensions
```

安装业务后台和 ROS Python 节点依赖：

```bash
python3 -m pip install --user fastapi "uvicorn[standard]" \
  websockets PyYAML requests numpy
```

如果要启用图像识别，还需要安装：

```bash
python3 -m pip install --user torch ultralytics hyperlpr3
```

识别模型已经放在 `ros2_ws/src/detector_node/models/`，包括 YOLOv8、YuNet 和 SFace 模型。人脸底库使用单独目录保存，不提交到 Git。

## 首次构建

当前项目的 ROS 2 工作区是 `ros2_ws`。首次构建或代码更新后执行：

```bash
cd ~/ros2Project/Remote_ctrol
source /opt/ros/humble/setup.bash

cd ros2_ws
colcon build --symlink-install --continue-on-error
source install/setup.bash
```

如果使用了新的终端，需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2Project/Remote_ctrol/ros2_ws/install/setup.bash
```

## 一键启动

所有启动脚本都在 `scripts/` 下，脚本默认运行在 Linux/WSL 环境，不是 Windows PowerShell 脚本。

| 模式 | 命令 | 适用场景 |
| --- | --- | --- |
| M1 视频 + 遥控 | `bash scripts/start_m1.sh` | 只验证视频、网页和遥控网关；无 Nav2、识别和业务后台 |
| 仿真监控 | `bash scripts/start_sim_monitor.sh` | Gazebo 仿真 + 遥控 + 静态监控页面；不启动 FastAPI |
| Nav2 完整演示 | `bash scripts/start_nav2_monitor.sh` | 仿真 + Nav2 + FastAPI + 识别 + 抓拍 + 监控台，推荐使用 |

推荐启动完整演示：

```bash
cd ~/ros2Project/Remote_ctrol
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
bash scripts/start_nav2_monitor.sh
```

启动后访问：

- 监控台：<http://localhost:8888/monitor.html>
- 巡检点：<http://localhost:8888/points.html>
- 路线计划：<http://localhost:8888/plans.html>
- 识别/抓拍记录：<http://localhost:8888/records.html>
- FastAPI 文档：<http://localhost:8888/docs>
- 健康检查：<http://localhost:8888/api/health>

按 `Ctrl+C` 可以停止一键启动脚本拉起的后台进程。

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

## D435 摄像头接入 WSL2

Windows 管理员 PowerShell 中执行：

```powershell
usbipd list
usbipd bind --busid <BUSID>       # 首次绑定时执行
usbipd attach --wsl --busid <BUSID>
```

进入 WSL 后执行：

```bash
sudo modprobe uvcvideo
cd ~/ros2Project/Remote_ctrol
bash scripts/d435_attach_fix.sh
```

默认推流脚本使用 `/dev/video4` 的彩色图像，默认分辨率为 `640x480`。通过 usbipd 透传时，720p YUYV 带宽较高，优先使用 640x480：

```bash
DEV=/dev/video4 SIZE=640x480 FPS=30 \
  bash scripts/push_d435_wsl.sh
```

如果没有摄像头，M1 脚本会自动回退到测试视频；`start_nav2_monitor.sh` 会尝试启动 D435 推流，识别节点没有有效视频源时会持续重试。

## Web 页面功能

### 监控台 `monitor.html`

- 显示地图、机器人位置、雷达点云和导航状态。
- 鼠标拖拽选择初始定位位姿。
- 鼠标拖拽选择 Nav2 目标点，支持取消导航。
- 虚拟摇杆、W/S/A/D 和急停。
- D435 视频按需开启/关闭。
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

在 `plans.html` 中按顺序组合点位为路线，再配置巡检计划。支持：

- `once`：一次性执行，格式为 `YYYY-MM-DD HH:MM`
- `daily`：每天执行，格式为 `HH:MM`
- `interval_min`：按分钟周期执行，例如 `120`

### 记录和报告

`records.html` 显示识别记录、抓拍记录和统计信息。巡检任务完成后会生成 HTML 报告，可以从计划页面的任务列表打开。

## ROS 2 接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 网关输出的速度指令 |
| `/odom` | `nav_msgs/Odometry` | 机器人里程计，用于遥测 |
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光雷达数据 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 网页下发的 AMCL 初始位姿 |
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
- `POST /api/plans/{id}/run`：立即执行计划。
- `GET /api/plans/tasks`、`GET /api/plans/tasks/{id}`：查询任务记录。
- `GET /api/plans/report/{id}`：生成或查看巡检报告。
- `GET /api/records/detections`：查询识别记录。
- `GET /api/records/captures`：查询抓拍记录。
- `GET /api/records/summary`：查询统计信息。
- `POST /api/records/detections`、`POST /api/records/captures`：写入识别和抓拍记录。

完整请求参数和响应结构可以在 `/docs` 查看。

## 数据和配置

当前代码默认使用以下运行时目录：

```text
~/ros2Project/Remote_ctrol/
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

注意：目前 `backend`、报告生成器、抓拍节点和识别节点的默认数据路径包含 `~/ros2Project/Remote_ctrol`。为了保证一键脚本和 FastAPI 静态页面正常工作，建议在 WSL 中按该路径放置项目；如果使用其他目录，需要同步修改代码中的默认路径，或建立对应软链接。

## 当前已知限制

1. `backend/scheduler.py` 的巡检点执行器目前是模拟执行：模拟导航等待 1 秒，停留动作最多等待 10 秒，并记录任务和报告；还没有接入真实 Nav2 Waypoint Follower。
2. 当前网关面向仿真发布 `/cmd_vel`。真实 JD03 底盘使用自定义 `yunle_msgs/Ecu` 接口，仍需要 `Twist → Ecu` 阿克曼运动学适配节点。
3. `twist_mux` 的手持遥控、网页遥控和自主导航优先级仲裁尚未完整接入。
4. 监控台中的电池电量和电压目前是模拟值，真车联调时需要接入 `battery_status`。
5. `start_sim_monitor.sh` 使用 `python3 -m http.server`，只提供静态页面；需要识别、抓拍入库、计划调度和报告时使用 `start_nav2_monitor.sh`。
6. 识别节点是 CPU 推理，性能取决于处理器和视频分辨率；人脸底库为空时只能识别为未登记人员或普通行人。
7. `bin/mediamtx` 不在 Git 中，需要分别准备 ARM64 或 AMD64 版本，不能跨架构复用。

## 常见排查

### 页面打开但显示未连接

确认网关是否启动，并检查 WebSocket 端口：

```bash
ros2 node list | grep robot_gateway
ss -lntp | grep -E '8765|8888|8889'
```

### 页面没有机器人位置或雷达点

网关需要同时收到 `/odom`、`/scan` 和 TF。仿真/Nav2 启动初期需要等待节点和生命周期组件完成初始化：

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
cd ~/ros2Project/Remote_ctrol
python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8888 --app-dir .
```

### ROS 2 找不到自定义节点

重新加载 ROS 2 和工作区环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2Project/Remote_ctrol/ros2_ws/install/setup.bash
ros2 pkg list | grep -E 'robot_gateway|detector_node|capture_server|chassis_nav2'
```

## 相关文档

- [启动命令速查](启动.md)
- [WSL2 仿真环境交接文档](WSL2仿真环境交接文档.md)
- [开发计划与里程碑](开发计划.md)
- [巡检机器人远程控制方案](巡检机器人远程控制方案（蒲公英组网版）.md)

## 许可证与数据说明

项目中的部分厂家驱动、SDK、模型和第三方组件遵循各自许可证。人脸底库、抓拍图片、数据库和报告属于运行时数据，默认不提交到 Git；部署和分发前请确认相关模型、SDK 及个人数据的使用权限。
