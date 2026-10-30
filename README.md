# Remote_ctrol 巡检机器人使用与维护

适用设备：**jkbot-180**。工程目录：`/home/jkbot/Remote_ctrol`。维护基线：2026-10-30，Ubuntu 22.04 / ROS 2 Humble。

本文件是项目日常操作、构建、配置和排查的总入口。根目录旧版开发计划、交接记录和分散手册已归档，模块内部的操作说明、模型来源及许可文档继续保留。配置和代码以本车实际文件为准；更新另一台车时按文件增量应用，并保留各车硬件、地图和 ROS 通信设置。

## 常用启动

在机器人本机的图形桌面终端启动 Qt：

```bash
cd /home/jkbot/Remote_ctrol
bash desktop_ui/scripts/run.sh
```

需要网页监控、任务和记录时，在另一个终端启动：

```bash
cd /home/jkbot/Remote_ctrol
bash scripts/start_robot_monitor.sh
```

浏览器打开 `http://<机器人可达IP>:8888/console.html`。Qt 管理本地建图、定位、导航和设备；监控脚本启动网页后端、网关及所选视频/音频服务。正常入口不会自动发起巡航。

已存在对应进程时复用原会话；不要重复启动第二套网关、视频服务、底盘驱动或导航。关闭某个终端只停止该入口管理的进程，不代表所有外部进程都已停止。

## 本车配置

| 项目 | 本车值 |
|---|---|
| 主机 | `jkbot-180` |
| ROS_DOMAIN_ID | `80`，由 `config/ros_domain.sh` 设置 |
| CAN1 远端端口 | `4001` |
| CAN2 远端端口 | `4002` |
| CAN 本地端口 | `8001` |
| 默认 FAST-LIO 地图入口 | `maps/fastlio/map.pcd` 和 `maps/fastlio/map.yaml` |

两车已核对的差异：63 使用 ROS 域 63、CAN1/CAN2 为 4002/4001、本地端口 8002；180 使用 ROS 域 80、CAN1/CAN2 为 4001/4002、本地端口 8001。不要把“180”当成它的 ROS 域号。CAN 网卡、IP、标定和其他参数继续读取各车自己的配置，不根据主机名推算。

手动执行 ROS 命令前，每个终端都加载本车环境：

```bash
cd /home/jkbot/Remote_ctrol
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
source /home/jkbot/Remote_ctrol/config/ros_domain.sh
export ROS_LOCALHOST_ONLY=1
export ROBOT_PROJECT_ROOT=/home/jkbot/Remote_ctrol
```

也可直接使用项目包装器，例如 `bash scripts/ros2_local.sh node list`；它会加载 ROS、工作区和本车域配置。跨主机 ROS 通信需专门统一配置；网页通过 HTTP/WebSocket 连接，不要求浏览器加入 ROS 域。

## 工程与数据

| 路径 | 内容 |
|---|---|
| `desktop_ui/` | Qt5 + RViz2 桌面端、地图编辑、路线操作和设备管理 |
| `backend/`、`web/` | FastAPI 业务后台、网页控制台 |
| `scripts/` | 构建、监控启动、视频、地图维护和验证脚本 |
| `config/` | 网关、ROS 域及本机业务配置 |
| `ros2_ws/src/chassis_nav2/` | 统一启动、底盘桥、里程计、Nav2 和控制仲裁 |
| `ros2_ws/src/JD03/` | 厂家底盘驱动及 CAN 接口 |
| `ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/` | FAST-LIO 里程计及 PCD 全局定位 |
| `ros2_ws/src/fastlio_loop_closure/` | 建图关键帧、回环和地图导出，按本车安装包提供 |
| `ros2_ws/src/robot_patrol/` | 活跃地图、点位、任务、联锁、返航与报告 |
| `ros2_ws/src/detector_node/`、`vision_events/` | 上位机检测、抓拍小图与视觉记录 |
| `maps/fastlio/`、`maps/slam_toolbox/`、`maps/cartographer/` | 各定位方案使用的地图 |
| `deployment_logs/` | 历次部署的原件、清单、验证和回退记录 |
| `/home/jkbot/.ros/log/` | ROS 节点和启动日志 |
| `/home/jkbot/.local/share/robot_patrol/patrol.sqlite3` | 点位、路线、任务、返航、路网和报告主库 |
| `data/vision/` | 独立视觉数据库、图片与待处理事件 |
| `data/web_records/`、`data/audio/` | 网页接收的视觉记录和上传音频 |
| `web/maps/` | 激活地图自动生成的网页 PNG 和几何元数据 |

网页、网关、巡航执行器必须使用同一个 `ROBOT_PATROL_DB`。业务主库由巡航节点写入，HTTP 后台只读访问；上传媒体另存。地图、数据库和录制样本都是车辆数据，更新代码时单独保留。

## 构建与更新

已构建的车辆日常启动不需要重新安装依赖。新环境或依赖缺失时，使用项目安装脚本：

```bash
cd /home/jkbot/Remote_ctrol
bash scripts/install_navigation_deps.sh
bash scripts/install_vision_deps.sh
```

导航脚本会安装系统包，需要 sudo。业务后台还需 FastAPI、uvicorn、websockets、PyYAML、requests、Pillow、python-multipart；播放器需要 sounddevice 和 PortAudio。视觉依赖与模型校验以 `ros2_ws/src/detector_node/requirements-cpu.txt` 和安装脚本为准。当前 NumPy 为 1.26.4，`transforms3d` 使用兼容的 0.4.2，避免旧版 `np.float` 导入错误。

更新相关包后，先停止使用这些包的会话，再构建：

```bash
cd /home/jkbot/Remote_ctrol
bash scripts/build_navigation.sh
bash scripts/build_web_patrol.sh
source /opt/ros/humble/setup.bash
cd /home/jkbot/Remote_ctrol/ros2_ws
colcon build --symlink-install --executor sequential \
  --packages-up-to fastlio_loop_closure
colcon build --symlink-install --executor sequential \
  --packages-select detector_node capture_server
colcon build --symlink-install --executor sequential \
  --packages-up-to recorded_route_nav2
source install/setup.bash
cd /home/jkbot/Remote_ctrol
bash desktop_ui/scripts/build.sh
```

`fastlio_loop_closure` 用于当前 FAST-LIO 建图导出，`recorded_route_nav2` 用于录制路网；按需要构建对应功能。桌面构建脚本默认并行数为 2，并执行桌面 CTest。设备相机、气体、云台等包按各模块说明补齐。修改接口或 C++ 后必须重编译并重启相关进程；Python 的安装目录可能是链接或独立副本，应核对实际导入路径。

180 的识别包曾使用独立安装副本，63 使用源码链接；仅修改 `src/` 不能保证两车都加载新代码。更新不得整目录覆盖另一台车。

<a id="daily"></a>
<a id="desktop"></a>
## Qt 日常操作

1. 选择 FAST-LIO、SLAM Toolbox/AMCL 或 Cartographer 对应模式。切换显示模式不会替换已启动的后台算法。
2. 在“运行”页填写本次实际地图路径；选择是否启动雷达/IMU、底盘及控制桥。已有驱动时复用，避免重复打开串口、雷达和 CAN。
3. 单点导航与路线巡航分别选择。路线任务在启动前勾选“启用路线巡航联锁”，让同一启动入口管理执行器、guard 和 twist_mux 锁。
4. 启动并等待定位。FAST-LIO 和 AMCL 在实际位置设置初始位姿，确认扫描/点云与地图对齐、Nav2 就绪。
5. 需要驾驶时明确“启用操作”，核对底盘反馈及现场条件后解除急停，再发目标或开始任务。
6. 结束时取消任务、确认停稳，再停止本次会话；相机和独立设备按各自入口停止。

“启用操作”、解除急停和初始位姿是不同操作。初始位姿只初始化定位；Nav2 就绪和合法控制权限仍是行驶条件。Qt 观察模式可主动触发急停。手动接管会先取消自动目标；按住运动、松开或失焦停止当前输入。

主界面左侧为运行/导航/手动，中央为 RViz，右侧为相机/抓拍识别/网页入口，底部为速度、定位、巡航、电池和气体状态。“路线巡航”“地图编辑”“设备执勤”分别打开独立窗口。过期或无效数据会显示等待或 `—`，不应当作有效反馈。

Qt 已嵌入 RViz 时，命令行启动导航使用 `rviz:=false`，避免重复显示。隐藏相机页只暂停 UI 预览和缩放，独立识别仍按开关运行。

<a id="fastlio"></a>
## FAST-LIO 建图与导航

### 建图与保存

命令行建图采用一个底盘驱动和一个建图会话。已加载本车 ROS 环境后，在独立终端启动底盘驱动：

```bash
ros2 launch chassis_driver chassis_driver.launch.py
```

另一个终端启动建图：

```bash
ros2 launch chassis_nav2 fastlio_mapping.launch.py rviz:=true
```

该入口启动雷达、IMU、FAST-LIO 及当前回环后端，内部关闭底盘启动和控制桥。需要键盘低速驾驶时，再开终端运行 `ros2 run chassis_nav2 jd03_keyboard.py`，并确保没有其他 `/ecu` 控制发布者。W/S 选择前进/倒车，A/D 转向，X 回正，空格刹车，Q 退出；具体限幅由本车硬件配置读取。

当前建图链路由回环后端导出正式地图，保存命令为：

```bash
ros2 service call /loop_closure/save_map std_srvs/srv/Trigger '{}'
```

等待返回成功，检查正式 PCD 和校验结果后再退出。默认正式输出为 `maps/fastlio/map.pcd`；原始 FAST-LIO 输出另存，当前启动代码把原始保存服务重映射为 `/fastlio/raw_map_save`。旧文档中的 `/map_save` 不作为当前正式导出的操作入口。Qt “保存地图”按当前版本组织保存和栅格导出；失败提示应保留排查，不把原始图直接当作已校验成图。

需要另存地图时，在启动前指定 `pcd_map:=/绝对路径/site.pcd`。正式 PCD、对应栅格和地图版本需要保持同一坐标系。

### PCD 投影与地图编辑

推荐使用 Qt 保存/导出流程，按本场地高度带检查投影。手动维护时先确认没有第二个 `pcd2pgm` 实例，再启动：

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py \
  z_min:=-0.10 z_max:=2.0 rviz:=true
```

以上高度值是本工程已有切片参数示例，坐标为保存的 PCD 坐标系；换车、换图和外参变化后需重新核对地面与障碍。另开终端保存：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /projected_map -f /home/jkbot/Remote_ctrol/maps/fastlio/map
python3 /home/jkbot/Remote_ctrol/scripts/export_web_map.py \
  /home/jkbot/Remote_ctrol/maps/fastlio/map.yaml \
  --output-dir /home/jkbot/Remote_ctrol/web/maps
```

只在检查并备份后按需清理噪点。`scripts/clean_map.py` 会直接修改 PGM，使用时明确传入实际文件；它无参数时的默认路径不是当前 `maps/fastlio/map.pgm`，不要照旧文档无参数执行。

<a id="map-edit"></a>
Qt 地图编辑入口支持三态笔刷、直线、缩放、平移、撤销和重做。先打开 YAML，保存到原文件或另存新地图，核对分辨率、origin 和导航路径回填，再重新激活地图版本。绘制地图不会同步改动定位 PCD；地图运行中热切换和独立禁行区图层不属于当前已验收功能。

### 定位与导航

仅定位、不启用控制桥：

```bash
ros2 launch chassis_nav2 localization.launch.py \
  odom_source:=lio rviz:=true
```

若底盘驱动尚未运行，按需要加 `start_chassis:=true`。传感器已经运行时加 `start_sensors:=false`。

完整定位、导航和路线巡航，采用本入口启动底盘驱动：

```bash
ros2 launch chassis_nav2 navigation.launch.py \
  start_chassis:=true enable_control:=true odom_source:=lio \
  enable_patrol:=true enable_recorded_routes:=true rviz:=false
```

已有独立底盘驱动时用 `start_chassis:=false`；只做普通单点导航时可省略两个巡航参数。导航本身包含 FAST-LIO 和全局 ICP，不要再平行启动一套定位。默认读 `maps/fastlio/map.pcd` 和 `map.yaml`；其他地图通过 `pcd_map:=... map:=...` 一起指定，或使用界面所选的已登记地图。

等待 `Global map received`，通过 Qt 或 RViz 设置实际位置和朝向；出现 `Initial global localization succeeded.` 后，检查地图对齐、反馈及 Nav2 生命周期状态，再发目标。初始化前的缺少 `map` TF 与消息缓存提示可属于等待过程；定位成功后持续出现则需要排查。

里程计参数可选 `lio`、`wheel`、`wheel_imu`。本次近期 63 测试使用 `lio`，不能沿用旧手册把所有模式都写成 EKF 提供 `/odom`：`lio` 由 LIO 提供连续里程计，`wheel_imu` 使用轮速/IMU EKF，地图修正由各定位方案提供。通过 `compare_wheel_odometry:=true` 可以额外观察轮速。变更来源前结束旧会话，保持 TF 唯一。

<a id="2d"></a>
## 二维建图与定位

三种方案互斥运行，各自使用对应地图和参数。二维方案同样要处理本车驱动复用、控制许可和路线联锁，不能与 FAST-LIO 同时争用主 TF 和扫描话题。

| 方案 | 建图入口 | 定位导航入口 | 地图 |
|---|---|---|---|
| SLAM Toolbox → AMCL | `slam_2d_mapping.launch.py` | `amcl_localization.launch.py` | `maps/slam_toolbox/map.yaml` 及其引用栅格 |
| Cartographer | `cartographer_2d_mapping.launch.py` | `cartographer_2d_localization.launch.py` | `maps/cartographer/map.pbstream` 及 PGM/YAML |

例：底盘驱动已单独运行时，SLAM Toolbox 建图命令为 `ros2 launch chassis_nav2 slam_2d_mapping.launch.py enable_control:=true rviz:=true`。保存二维栅格：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/jkbot/Remote_ctrol/maps/slam_toolbox/map \
  --ros-args -p save_map_timeout:=10.0
```

需要保留续建图状态时，再保存 SLAM Toolbox posegraph/data；AMCL 定位不读取这两个文件。定位启动：

```bash
ros2 launch chassis_nav2 amcl_localization.launch.py \
  enable_control:=true rviz:=true
```

AMCL 等待 `/initialpose`，使用“2D Pose Estimate”在实际位置初始化。旧 `slam_2d_localization.launch.py` 为 AMCL 兼容入口，实际二维地图通过 `map:=/绝对路径/地图.yaml` 指定。

Cartographer 建图：`ros2 launch chassis_nav2 cartographer_2d_mapping.launch.py enable_control:=true rviz:=true`。保存顺序：

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \
  '{trajectory_id: 0}'
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/jkbot/Remote_ctrol/maps/cartographer/map.pbstream', include_unfinished_submaps: true}"
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=/home/jkbot/Remote_ctrol/maps/cartographer/map.pbstream \
  -map_filestem=/home/jkbot/Remote_ctrol/maps/cartographer/map
```

定位导航启动：`ros2 launch chassis_nav2 cartographer_2d_localization.launch.py enable_control:=true rviz:=true`。此前本机版本的使用记录要求靠近建图起点、朝向一致，不能套用 AMCL 的任意位置 `/initialpose` 重初始化流程；更换 Cartographer 版本后按实际接口重新核对。

独立 NDT 融合定位与 CSV 固定导航沿用单独入口，见 [模块说明](docs/NDT融合定位与CSV固定导航移植说明.md)，不与上述会话重复运行。

<a id="patrol"></a>
## 巡检任务与录制路线

### 自由导航、多点任务和返航

1. 在启动前启用路线巡航联锁，确认定位、Nav2、guard、twist_mux 及操作端心跳就绪。
2. 打开“路线巡航 → 地图与返航设置”，登记实际定位文件和导航 YAML，核对后设为活跃地图。
3. 车辆停稳后保存当前位置为导航点，或输入地图坐标；新建路线，按顺序添加点位和停留时间。
4. 自由导航模式按任务顺序规划到点。设置执行次数：1 为一次，N 为指定次数，0 为持续循环；预览与就绪检查通过后开始。
5. 暂停、继续、取消分别等待反馈。返航点在当前地图停稳后保存；地图版本变化后需要重新确认。

“一键到达与巡检”可选择已有点或地图任意可达目标；勾选到点识别时，停车后执行相机/云台/拍摄识别和报告。临时目标不写回原有点位或路线。一键返回结束当前任务，确认子目标取消后派发返航目标，不自动恢复旧任务。详见 [一键到达与返航](docs/ONE_CLICK_INSPECTION_20261029.md)。

### 录一遍，再沿录制道路巡检

在定位导航模式中启用 `enable_patrol:=true enable_recorded_routes:=true`，打开“路线巡航 → 录制路径”，填写名称，开始录制，用原有手动功能低速行驶，到路口/巡检点分段，停车后结束预览并保存。录制动作本身不发速度。检查方向、前进/倒车段、闭环和地图碰撞后执行保存的任务。

非闭环按趟往返，闭环按圈；返程可能保持车头朝向倒车沿原路返回，不等同于掉头。自动接入和每次换向须经过碰撞、曲率、停稳和联锁检查。高级路网还需要明确连接节点、绑定点位、校验发布版本并预览任务；位置接近不自动代表道路连通。

固定路线正常沿录制道路行驶，遇障停车并按当前参数检查受限绕行；不合格时等待或失败。固定路网返航需要有效的反向可达连接和相同版本，不能把拒绝返航当成可自动回退自由规划。

定位跳变、采样过期、曲率或道路连接校验失败时，保留草稿排查；不要扩大终点容差绕过道路质量检查。旧录制坐标不会因算法更新自动修复。几何、版本和接口细节见 [录制路网模块](ros2_ws/src/robot_patrol/robot_patrol/recorded_routes/README.md)。

## 相机、识别和设备

Qt “相机”页单独启动或复用海康节点，登录连接后显示视频；云台方向和变倍按住操作、松开停止。相机与导航分别管理。右侧“抓拍识别”提供独立上位机执勤，三类开关分别控制：

| 功能 | 实现 | 当前边界 |
|---|---|---|
| 人脸抓拍 | OpenCV YuNet | 检测并裁小图，不做姓名/身份比对 |
| 行人 | YOLO26n OpenVINO CPU | 按目标轨迹记录 |
| 车牌 | LPD-YuNet + 独立字符识别 | 连续检测确认及 OCR 缓存 |
| 数字仪表 | 按需 OCR 或到点在线视觉服务 | 与上述三个常驻模型分别启停 |

三个常驻模型共用 `/hk_camera/image_raw`，使用上位机 CPU，不依赖相机原生 facesnap 模式。当前默认各模型目标上限 1 Hz、CPU 线程参数为 2；三模型错峰，同模型不重复处理同一条图像消息。相机原图配置上限为 2 fps，轻量 UI 预览独立限帧；限发布频率不会把相机源解码自动降为 2 fps。

三开关全关会释放推理模型，检测节点和记录查询可继续存在；没有检测事件不代表模型没有处理画面。静止执勤不需要导航和车辆运动许可；记录坐标仍依赖有效地图、定位和时间同步。切页或隐藏识别窗口不会自动停模型。

命令行启动识别时可以显式选择功能，例（先启动相机，已有检测节点时通过其参数服务更改，不另起一套）：

```bash
ros2 run detector_node detector --ros-args \
  -p input_mode:=ros -p image_topic:=/hk_camera/image_raw \
  -p enable_person:=false -p enable_plate:=false -p enable_face:=true \
  -p max_inference_hz:=1.0 -p cpu_threads:=2
```

语音/图片记录、模型来源和设备操作参见 [视觉执勤](ros2_ws/src/vision_events/README.md)、[模型清单](ros2_ws/src/detector_node/models/README.md) 和 [气体/云台设备](ros2_ws/src/gas_detector/README.md)。气体值的缩放估算需与设备协议核对；云台电机反馈和相机镜头变倍由各自接口处理。

## 网页与音频

已接入 30 条本地固定语音提醒，包括到点、巡检完成、人员检测、返航、急停、电量和设备异常。标准二维/三维 Nav2 启动流程和网页监控均可启动统一音频服务，按 ROS 域复用；配置阈值、播报规则、对讲互斥与测试命令见 [运行状态语音提醒](ros2_ws/src/audio_player/README.md)。车上无需运行语音生成模型。

监控脚本默认 `ENABLE_DETECTOR=0`、`ENABLE_AUDIO=1`；存在 `bin/mediamtx` 时自动启用视频及独立抓拍。三类视觉开关的网页按钮操作已运行的检测节点，不会代替构建和启动检测进程。需要命令行视频模式检测时，可显式设置 `ENABLE_DETECTOR=1`，其负载和来源与 Qt 的 ROS 原图模式分别核对。

| 服务 | 地址或端口 |
|---|---|
| 网页总入口 | `http://<机器人IP>:8888/console.html` |
| API 与健康检查 | `/docs`、`/api/health` |
| 遥控网关 | WebSocket `8765` |
| MediaMTX | RTSP `8554`、WebRTC `8889`、媒体 `8189` |
| MediaMTX 本机状态 API | `127.0.0.1:9997` |

旧 `monitor.html`、`tasks.html`、`records.html` 地址仍作为对应页面入口。网页先取得控制权，Qt 本地启用操作会优先接管。控制台内部切页保留任务连接；关闭/刷新页面、断网、心跳过期或本地接管会触发控制权释放与相应任务取消。

相机地址和登录参数留在本机相机 YAML，通过 `ROBOT_CAMERA_CONFIG` 指定；浏览器兼容转码沿用 FFmpeg 软件 H.264，当前兼容转码目标为最大宽度 1280、15 fps，先降帧再缩放。H.264 输入可直接转发，保持源属性；硬件转码尚未启用。查询 `/api/video/status` 区分服务未启动、源断流及无画面。

音频上传和播放由项目 `audio_player` 提供。180 另有独立工程 `/home/jkbot/piper_tts_ros`，已完成构建和中文离线合成、ROS 消息与 PulseAudio 播放验证；它不由本工程构建脚本编译。试听时：

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/piper_tts_ros/install/setup.bash
ros2 run piper_tts_ros piper_tts_say --device pulse "你好，本地语音测试。"
```

USB 声卡已由 PulseAudio 管理，直接按 `USB` 名称选择硬件可能找不到可用输出；当前通过 `pulse` 通道送到默认 USB 声卡。

网页当前没有账号认证，使用受控内网或已有 VPN；运动操作必须保留现场急停、权限和反馈检查。业务字段和接口说明见 [网页模块](web/README.md) 与 [巡检业务](web/巡检业务使用说明.md)。

<a id="status"></a>
## 当前实测状态与性能限制

2026-10-30 已分别在两车增量应用：原图配置 8→2 fps、FFmpeg 先降帧再缩放、关闭未使用的 Nav2 `smoother_server` 和默认全局代价地图显示、三个识别模型错峰与末帧去重、车牌 ONNX Runtime 等待自旋关闭、OpenVINO 使用统计关闭。`velocity_smoother` 保留；定位算法、地图分辨率、CAN 和各车硬件参数保留原值。

本次优化验证了模型可加载、调度、图片停止后的行为及视频输出；没有据此认定真实巡逻性能达标。硬件编码当前未启用，CQP 支持检查不等于完成现有限码率参数下的稳定转码。

63 最近同图同路线的三轮测试（18:55–19:04）：

| 条件（现场描述） | 控制循环未达 20 Hz 的告警 | 全局定位平均 / 最大耗时 | 现场表现 |
|---|---:|---:|---|
| 三类识别全开 | 17 | 0.99 / 2.32 秒 | 明显画龙 |
| 三类全关 | 0 | 0.57 / 1.23 秒 | 未报告明显画龙 |
| 仅开人脸 | 5 | 0.88 / 1.80 秒 | 明显画龙 |

全局定位耗时是日志中裁剪、降采样及两级 ICP 的墙钟耗时，不是 CPU 使用时间；三轮匹配分数相近。视觉开启与导航时序变差相关，具体调度、数据延迟及转向反馈因果仍需排查。不能把此前“定位平均占整机约 6%”当成并发工况实时性保证，也不能把一次任务完成当成轨迹稳定。控制循环节拍告警不等同于 CAN 通信超时。

**63 的画龙问题尚未解决。** 当前现场巡逻可沿用已验证较平稳的全关识别条件；需要视觉时先静止执勤或停车识别，不能把仅开人脸视为已验证稳定方案。180 的小图表现不能直接证明换大图也稳定。

定时/周期无人值守调度、地图运行中热切换、人脸身份库等不列为已交付能力；未记录实车验收的模块，以对应现场结果为准。

<a id="validation"></a>
## 排查与验证

先在正确 ROS 域下做只读检查：

```bash
bash /home/jkbot/Remote_ctrol/scripts/ros2_local.sh node list
bash /home/jkbot/Remote_ctrol/scripts/ros2_local.sh topic hz /odom
bash /home/jkbot/Remote_ctrol/scripts/ros2_local.sh topic hz /scan
bash /home/jkbot/Remote_ctrol/scripts/ros2_local.sh topic echo /vision/detector_status --once
ss -lntp
```

| 现象 | 优先核对 |
|---|---|
| 找不到 ROS 包或服务 | 当前终端的 setup、`config/ros_domain.sh`、实际安装包与重复节点 |
| 一直没有 `map` TF | PCD 是否有效、全局地图加载、初始位姿、ICP 日志及 transforms3d 导入 |
| 地图与点云错位 | PCD/PGM/YAML 是否同次导出、origin、当前激活版本和重复投影进程 |
| 任务按钮不可用 | 活跃地图版本、定位/底盘反馈、急停、guard/mux、操作端心跳和已存在任务 |
| 页面连接失败 | 网关、8765/8888 端口、浏览器地址及控制权状态 |
| 没有视频或识别输入 | 相机/MediaMTX 状态、实际源、原图订阅及检测节点状态，不按事件数量推断输入帧率 |
| 开识别后画龙 | 对照控制告警、ICP 墙钟耗时、CPU/线程负载、TF/里程计新鲜度和转向反馈 |
| 合成成功但无声音 | 播放设备、PulseAudio 默认输出及静音状态；USB 硬件可能已由系统音频服务管理 |

现场回归顺序：启动/观察 → 相机与设备静止验证 → 定位与单点低速导航 → 同图同路巡逻、返航与取消 → 开关识别负载对照。每步记录实际车辆、地图版本、开关状态、起止时间和表现。

需要记录导航抖动时，在另一终端加载本车环境并录制关键数据：

```bash
ros2 bag record -o /home/jkbot/Remote_ctrol/bags/nav_check_$(date +%Y%m%d_%H%M%S) \
  /tf /tf_static /odom /odom_lio /map_to_odom /localization \
  /cmd_vel_nav /cmd_vel_auto /cmd_vel /cmd_vel_final \
  /ecu /vehicle_status /scan /plan /diagnostics /rosout \
  /vision/detector_status
```

录包本身增加 I/O 和 CPU，比较试验使用相同记录条件；话题以本会话实际列表为准。配合逐秒进程/线程 CPU 采样，才能区分平均占用、瞬时延迟和实际位姿/转向变化。

代码更新后使用相关模块已有离线/隔离检查；这些检查和模拟底盘测试不能替代实车验收。此次文档整理只核对文档、链接和文件完整性，不启动车辆来验证命令。

## 备份与回退

CPU 优化的原件、清单和回退脚本保存在每辆车自己的：

```text
/home/jkbot/Remote_ctrol/deployment_logs/cpu_opt_20261030_174600/
```

需要回退这次 CPU 优化时，先通过原入口停止导航、底盘控制、相机及识别，再执行该车目录内的 `rollback.sh`。脚本会校验文件仍匹配部署结果；后续已修改的文件需要人工核对，不直接覆盖。

本文档整理前的根目录 Markdown 和关联模块说明原件存放于：

```text
/home/jkbot/Remote_ctrol_docs_backup/20261030_194332_final_readme
```

该目录在工程之外，包含原件、SHA-256 清单、变更记录和 `restore_docs.py`。只恢复文档时执行 `python3 <上述备份目录>/restore_docs.py`；恢复前检查脚本的版本冲突提示。两车分别保存各自原件，没有用另一台车的文件作备份。

继续维护各模块时保留厂商、模型和第三方的许可、来源及 NOTICE 文件。
