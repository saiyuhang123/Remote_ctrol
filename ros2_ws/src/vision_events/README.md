# 独立视觉执勤与事件记录

人脸、行人、车牌都在上位机处理，相机只提供视频：**YuNet 人脸检测、小图 → YOLO26n 行人 → LPD-YuNet 车牌四角点 + HyperLPR3 字符识别模型**。不做人脸身份、姓名、是否登记比对，不加载 SFace 或人脸库。三个功能可分别启停；视觉模块不启动导航、底盘、巡航任务、云台运动或远程网关，支持静止执勤。

## 构建与依赖

项目根目录执行：

```bash
./scripts/install_vision_deps.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install --packages-select \
  vision_interfaces vision_events detector_node robot_gateway --parallel-workers 2
./desktop_ui/scripts/build.sh
```

CPU 依赖版本固定在 `detector_node/requirements-cpu.txt`。保持 NumPy 1.26.4 + transforms3d 0.4.2，避免影响 FAST-LIO 的 `tf_transformations`。无需在运行环境安装 PyTorch、Ultralytics、HyperLPR3 Python 包。五个实际使用的模型文件随包安装，启动时不下载；来源、许可证、hash 和导出复现见 [模型清单](../detector_node/models/README.md)。

## UI 独立使用

重新打开桌面端，在右侧 **“抓拍识别 → 独立视觉执勤”** 中：

- **“启动上位机人脸 / 停止人脸检测”**：只控制 YuNet 人脸检测，保存人脸小图。
- **“启动行人 / 车牌”**：按两个复选框启用对应功能，不改变当前人脸开关。
- **“停止上位机检测”**：关闭人脸、行人、车牌全部推理，释放模型；记录服务继续提供历史查询。
- **“查看视觉记录”**：打开本地分页列表、图片与坐标详情，可填写当前实际定位地图的标记。

不需要导航、巡航、车辆“启用操作”或解除车辆急停。首次启动复用或启动原相机，并准备记录服务。相机需要登录并持续输出 `/hk_camera/image_raw`，**人脸检测也需要取流**。无需设备 facesnap / 人脸报警规则；默认关闭旧 SDK 人脸接收，记录节点默认不再订阅或补扫海康原生人脸事件。旧的已归档记录仍可查。

切页、隐藏面板或关闭历史窗口不停止执勤。退出整个桌面端时停止它启动的识别/记录进程，外部节点保持运行。停止取流会让全部视觉检测等待输入。`--offline` 下禁用真实启停。若更新前已有外部检测/记录进程，结束它们后再启动，避免复用旧版本代码。

## 不通过 UI 启动

先加载环境，各终端按需执行。私有相机参数文件由操作者提供；示例不包含账号密码。

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
export ROBOT_PROJECT_ROOT=/home/jkbot/Remote_ctrol

# 终端 1：已有相机时不用重复启动，所有检测都需要视频。
ros2 run hk_camera hk_camera_node --ros-args \
  --params-file /绝对路径/hk_camera_params.yaml \
  -p auto_login:=true -p auto_stream:=true -p enable_face_capture:=false

# 终端 2：记录服务，无定位、无后台也能保存图片。
ros2 run vision_events recorder

# 终端 3：仅启动上位机人脸；行人/车牌按需改为 true。
ros2 run detector_node detector --ros-args \
  -p input_mode:=ros -p enable_face:=true \
  -p enable_person:=false -p enable_plate:=false

# 独立开关，不会启动导航、云台或底盘。
ros2 param set /detector_node enable_person true
ros2 param set /detector_node enable_plate true
ros2 param set /detector_node enable_face false

# 全部关闭后停止推理并释放模型，保留参数服务供再次启用。
ros2 param set /detector_node enable_face false
ros2 param set /detector_node enable_person false
ros2 param set /detector_node enable_plate false

ros2 run vision_events vision_cli list --type face_capture
ros2 run vision_events vision_cli context --map 场地A --revision v1
```

`ros2 launch vision_events vision.launch.py start_detector:=true` 一起启动记录和全部三类检测，不启动相机或导航。默认 `start_detector:=false` 仅启动记录服务。

录像测试可设置 `input_mode:=video video_source:=/绝对路径/test.mp4`，也保留 RTSP 输入入口。Qt 使用 ROS 最新帧，复用相机解码，不积压旧帧；RTSP/文件模式按顺序读取，录像结束会循环，低推理频率下并非实时播放速度。

## 模型、负载和记录频率

- 三类按开关延迟加载，不开对应功能就不加载模型。LPD-YuNet 直接找车牌，不先跑车辆检测；字符模型只读取矫正后的牌照，不重复跑 HyperLPR 检测器。
- 默认最多 4 次/秒，CPU 线程数 2；实际频率受推理耗时限制，不承诺 4 FPS。人脸默认输入长边最多 960，车牌最多 640，均保持整图比例；行人使用 640×640 letterbox。
- 人脸只做位置跟踪，连续可见时默认每 2 秒记录一次，离开再进入可再次记录；不会按身份过滤。可启动时配置 `face_capture_interval_sec`，最低 0.25 秒。
- 行人同一连续轨迹记录一次；车牌同轨迹同号码连续两帧确认后记录一次。OCR 默认接受 7/8 位、字符平均置信度至少 0.7 的输出。轻量 IoU 跟踪默认 2 秒超时，遮挡和漏检可能产生新轨迹。
- 当前字符入口针对单行中国车牌，双层牌、强反光、极斜或很小的目标需要现场验证。未确认号码不写成识别成功记录；暂未提供单独的“检测到车牌但号码未知”记录。

## 数据、时间与坐标

- 权威数据库：`$ROBOT_PROJECT_ROOT/data/vision/events.sqlite3`。
- 小图：`data/vision/media/YYYY-MM-DD/<event_id>/image.jpg`，推理 staging 成功归档后清理。
- 来源分别为 `host_yunet`、`host_yolo26n`、`host_lpd_yunet_ocr`；车牌额外保留号码、检测置信度和四角点，人脸无姓名/登记字段。
- 使用视频帧 `header.stamp` 关联历史 `map→base_link`，不读取 UI 当前坐标。**没有关联地图标记或没有新鲜定位 TF 时仍保存图片，坐标保持无效**。恢复定位不为旧图补造坐标。
- 地图标记只声明实际定位上下文，不启动/切换定位。地图变化需操作者明确重新关联，避免把不同地图坐标混用。默认位姿匹配最大时间差 0.3 秒。
- 海康 ROS 图像时间通常是驱动解码/发布时的主机时间，`time_basis=ros_image_stamp`，不能视为精确曝光位置。旧原生记录仍保留 `host_alarm_receipt` 时间来源。
- 同一事件消息重放、目录补扫保持同一 event_id；不按人脸身份去重。文件锁阻止同一目录有两个写入者。
- 图片不可读、磁盘满和队列溢出会报告错误，不伪造有效图片/坐标；首版不自动删除成功历史记录。

仅在需要兼容旧设备抓拍时，操作者可显式以 `accept_hikvision_faces:=true` 启动记录节点并配置 `face_dirs`。当前 UI 不启用此模式；默认只处理上位机 staging 和原始事件，不再补录旧海康目录。

## 本地 ROS 接口与网页

| 接口 | 用途 |
| --- | --- |
| `/hk_camera/image_raw` | 三类检测共用的实时视频输入 |
| `/detector_node/set_parameters`、`set_parameters_atomically` | `enable_face` / `enable_person` / `enable_plate` 独立开关 |
| `/vision/raw_detections` | 上位机原始事件及 staging 引用 |
| `/vision/events` | 已成功入库的统一事件 JSON |
| `/vision/status`、`/vision/detector_status` | 记录、模型、视频和实际/请求开关状态 |
| `/vision/query_events` | 分页、类别筛选、按 ID 查询 |
| `/vision/set_recording_context` | 可选地图标记 |
| `/detection_events` | 对网页/网关兼容的实时通知 |

网页监控页的三个开关使用同一检测节点参数服务，人脸不依赖海康 face_status。需要先在本地启动检测节点/记录和相机，再取得网页控制权；网页不远程启动系统进程。模型异常时仍可关掉失败功能。UI 主要通过状态、数量、历史小图显示结果，目前没有每次识别弹图/语音，也不保证预览叠加框。

## 当前网页后台与旧上传入口

网页后台现已接入 `/api/records/detections`，默认 `upload_enabled:=true`。记录器通过持久 outbox 自动上传事件及图片，后台按 event_id 去重，网页可查询完整图片、时间、类型和坐标。后台离线时继续本地记录，恢复后补传；可设 `upload_enabled:=false` 保持仅本地执勤。

`start_robot_monitor.sh` 的可选检测启动启用上传，也可通过 `ROBOT_BACKEND_URL` 指定后台。静止执勤记录同样上报，Qt 和 CLI 查询继续可用；原有 `inspection_recorder` 继续订阅统一事件并累计巡检摘要，不影响视觉独立运行。详见[网页巡检业务说明](../../../web/巡检业务使用说明.md)。

## 无实车验证

```bash
python3 scripts/verify_vision_models.py
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PYTHONPATH="$PWD:$PWD/ros2_ws/src/detector_node:$PWD/ros2_ws/src/vision_events:$PYTHONPATH" \
  ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 VISION_ROS_TESTS=1 \
  python3 -m pytest -q ros2_ws/src/vision_events/test
ctest --test-dir desktop_ui/build -R '^desktop_vision$' --output-on-failure
WEB_BROWSER_TESTS=1 python3 -m pytest -q tests/web/test_daily_flows.py
```

ROS 集成测试强制域 227，使用临时目录、假相机消息和 TF；Qt 域 228 使用假服务。下载模型清单中的三张公开样例后，可额外设置 `VISION_PUBLIC_SAMPLES=/图片目录`，运行真实模型 → ROS 输入 → 本地归档的三类端到端测试；未设置时只跳过这一项样例测试。公开样例的真实模型测试方法见模型清单。实机并行导航负载、夜间/距离/角度效果和长期磁盘容量仍需现场验证。
