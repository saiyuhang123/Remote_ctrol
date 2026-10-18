# 独立视觉执勤与事件记录

海康相机负责人脸检测和抓拍；上位机负责行人、车辆检测和车牌识别。无人脸身份比对，不加载 YuNet/SFace 或人脸库。视觉模块不启动导航、底盘、巡航任务、云台运动或远程网关，静止执勤时可独立运行。

## 构建与依赖

在项目根目录执行：

```bash
./scripts/install_vision_deps.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install --packages-select \
  vision_interfaces vision_events detector_node hk_camera --symlink-install --parallel-workers 2
./desktop_ui/scripts/build.sh
```

CPU 依赖固定在 `detector_node/requirements-cpu.txt`。使用已有 YOLO OpenVINO 导出模型，不要求 PyTorch/Ultralytics；只开海康人脸记录时也不加载上位机模型。HyperLPR3 模型在依赖安装时缓存到用户目录，之后可离线运行。

## UI 独立使用

打开桌面端，在右侧 **“抓拍识别 → 独立视觉执勤”** 中：

- “启动人脸抓拍 / 停止人脸抓拍”：显式控制海康 SDK 报警接收。
- “启动行人 / 车牌 / 停止上位机检测”：勾选需要的行人、车牌功能，使用同一个 YOLO 实例。
- “查看视觉记录”：本地分页列表、图片与坐标详情；可填写当前实际定位地图的标记。

这些按钮无需启动导航、选择导航方案、创建巡航任务、解除急停或勾选车辆“启用操作”。启动时复用已有相机，或使用原相机面板的私有配置启动相机；不会另起第二个同名相机。`--offline` 模式禁用真实启动和设备写操作。

切页或隐藏视觉面板不会停止执勤。关闭历史窗口不停止记录。退出整个桌面端时停止其拥有的识别/记录进程，外部节点保持运行。相机预览和视觉检测共用视频；手动“停止取流/停止相机”会影响上位机输入，界面会显示等待视频或状态过期。

相机设备端仍需配置 `facesnap`、规则及报警上传；接收通道开启不代表相机已经生成抓拍。软件不切换设备智能资源、不动云台。相机的 face_id 是设备目标编号，不是人员身份。

## 不通过 UI 启动

先加载 ROS2 与工作空间环境，各终端按需执行。私有相机参数文件由操作者提供；示例不包含账号密码。

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
export ROBOT_PROJECT_ROOT=/home/jkbot/Remote_ctrol

# 终端 1：已有相机节点时不用重复启动。
ros2 run hk_camera hk_camera_node --ros-args \
  --params-file /绝对路径/hk_camera_params.yaml \
  -p auto_login:=true -p auto_stream:=true -p enable_face_capture:=false

# 终端 2：仅启动本地记录，允许无定位、无视频、无后台。
ros2 run vision_events recorder

# 终端 3：按需打开人脸抓拍。
ros2 service call /hk_camera/set_face_capture std_srvs/srv/SetBool '{data: true}'

# 行人和车牌检测独立启动，可与人脸同时或分别运行。
ros2 run detector_node detector --ros-args \
  -p input_mode:=ros -p enable_person:=true -p enable_plate:=true

# 两者关闭后停止推理并释放模型；重新置 true 可再次启用。
ros2 param set /detector_node enable_person false
ros2 param set /detector_node enable_plate false

# 结束新的人脸接收；已经入队的事件仍会保存。
ros2 service call /hk_camera/set_face_capture std_srvs/srv/SetBool '{data: false}'

# 查询历史；关联地图只提供记录标记，不启动或修改导航。
ros2 run vision_events vision_cli list --type face_capture
ros2 run vision_events vision_cli context --map 场地A --revision v1
```

只做人脸执勤、不需要视频时，相机可以 `auto_stream:=false`。上位机行人/车牌需要视频，需启动取流。`ros2 launch vision_events vision.launch.py start_detector:=true` 可一起启动记录和上位机检测，不启动相机或导航。

录像测试使用 `input_mode:=video video_source:=/绝对路径/test.mp4`；原监控脚本已显式使用 video 模式，保留 RTSP 输入兼容。ROS 模式使用原图裁剪做车牌识别，YOLO 固定 640×640 letterbox；默认最多 4 Hz、OpenVINO 2 CPU 线程。ROS 输入采用最新帧策略，跳过旧视频帧会影响观测数量，但不会按身份合并海康事件。RTSP/录像保留顺序读取兼容入口，RTSP 延迟需另行现场验证。

## 数据、时间与坐标

- 权威数据库：`$ROBOT_PROJECT_ROOT/data/vision/events.sqlite3`。
- 成功归档的小图：`data/vision/media/YYYY-MM-DD/<event_id>/image.jpg`。
- 推理 staging 在成功归档后清理；海康原生图片与 JSON 保留在 `captures/hk_faces` 等配置目录。
- 海康原生目录可通过记录节点 `face_dirs` 参数配置。仅接收这些目录和 staging 下的媒体；外部相机使用自定义目录时需要同步配置记录节点。
- 每个不同海康抓拍文件分别记录；相同消息重放、目录补扫和网络重传保持同一 event_id，不按人脸/目标 ID/时间窗口去重。
- 人脸图缺失、文件不可读、定位不可用等分别保存异常信息；不会伪造成功图片或零坐标。查看状态中的 errors/overflows 和驱动 dropped 计数。
- 记录 `map→base_link` 的历史位姿，不读取 UI 最新坐标。**地图标记是可选关联；没有标记或没有新鲜 TF 时坐标为无效，图片照常保存。** 地图切换和记录会话需由操作者明确标记，UI 显示模式变化不自动修改记录。
- 海康时间是主机 SDK 报警接收时间，可能晚于实际拍摄；字段 `time_basis=host_alarm_receipt`。坐标是该时刻的近似，不是已校准的曝光位置。ROS 视频时间同样是驱动解码/发布帧的主机时间。
- 位姿最大匹配差默认 0.3 s；旧事件补扫时缺少历史定位就保持无效，不能用当前位置回填。
- 行人使用连续轨迹一次记录；车牌同一轨迹/号码两帧确认后一次记录，离开重进可再记录。跟踪采用轻量 IoU/2 s 超时，不做人员重识别，遮挡或漏检可能产生新轨迹。
- 文件锁阻止同一目录同时运行两个记录/检测写入者。磁盘满、消息溢出等会报告故障；不存在任意负载下零丢失保证。首版不自动删除历史成功记录。

## 本地 ROS 接口

原生人脸消息沿用 `hk_camera/msg/FaceCapture`；事件和状态使用版本化 JSON 的 `std_msgs/String`，避免重复定义已有消息。

| 接口 | 用途 |
| --- | --- |
| `/hk_camera/face_capture` | 原生人脸消息，reliable + transient-local |
| `/hk_camera/set_face_capture` | `std_srvs/SetBool`，真正控制 SDK 接收 |
| `/hk_camera/face_status` | 登录/接收状态、队列、保存/丢弃计数与错误 |
| `/vision/raw_detections` | 上位机原始事件及 staging 引用 |
| `/vision/events` | 已成功入库的统一事件 JSON |
| `/vision/status`、`/vision/detector_status` | 记录与推理状态 |
| `/vision/query_events` | `vision_interfaces/QueryEvents`，分页/类别筛选、按 ID 查询 |
| `/vision/set_recording_context` | `vision_interfaces/SetRecordingContext`，可选地图标记 |
| `/detection_events` | 对旧网页/网关兼容的实时通知 |

## 可选后台同步

```bash
ros2 run vision_events recorder --ros-args -p upload_enabled:=true \
  -p backend_url:=http://127.0.0.1:8888
```

默认为 false，不启动后端也不影响本地记录。启用后以事务 outbox 上传，断网/重启可重试，后端按 event_id 幂等，旧请求与旧记录仍兼容。后台提供 `/api/vision/media/{event_id}` 读取受控本地归档图片；目前要求后台与记录目录在同一机器。`VISION_DATA_DIR` 可指定后端使用的记录目录，异机媒体传输未实现。旧 `start_robot_monitor.sh` 开启 detector 时会同时启动记录及后台同步。

## 验证

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PYTHONPATH="$PWD:$PYTHONPATH" ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 VISION_ROS_TESTS=1 \
  python3 -m pytest -q ros2_ws/src/vision_events/test
ctest --test-dir desktop_ui/build --output-on-failure
```

集成测试强制隔离域 227，使用临时目录、假相机消息与 TF，不启动或连接真实设备。Qt 测试使用域 228，假服务验证不依赖导航/运动许可的启停、切页继续、历史查询与离线禁用。

2026-10-18 已在本机执行 OpenVINO 实模型测试：官方公开 `bus.jpg` 样例检出 3 个行人和 1 辆公交；1920×1080 空白图约 56～64 ms/次，样例单次约 103 ms。HyperLPR3 模型加载及空白图推理通过，尚未使用真实车牌样本验收。上述是单次环境结果，不是导航并行负载或识别准确率承诺。未建立旧版 PyTorch 链路的等条件性能基线。

人脸真实接收、移动时的位置误差、车牌识别距离/角度/夜间效果和长期运行容量，仍按计划 P6 在现场验证。
