# 海康相机 ROS2 包

相机默认地址为 `192.168.1.64:8000`，用户名为 `admin`。密码默认值已更新，
可在启动时通过 `password:=...` 覆盖。

## SDK

项目使用 HCNetSDK V6.1.11.30（20260805），库文件为 Linux x86_64 架构。
配套头文件放在 `../HK/C++demo/consoleDemo/include/`，库文件放在
`../HK/C++demo/consoleDemo/linux64/lib/`，保留 `HCNetSDKCom/` 子目录。

构建时会将 SDK 库安装到 `lib/hk_camera/hk_sdk/`。程序自动查找安装目录，
开发时也可使用项目中的 SDK，不需要手动设置 `LD_LIBRARY_PATH`。
SDK 使用 `RTLD_LOCAL | RTLD_DEEPBIND` 加载，避免其自带的 tinyxml2 与
ROS/Fast DDS 的 tinyxml2 发生 ABI 冲突。

## 编译

```bash
cd /home/jkbot/Remote_ctrol/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select ocr_interfaces hk_camera --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

默认同时编译 `hk_camera_node` 和 `hk_camera_gui`。仅需节点时，可添加
`-DBUILD_HK_GUI=OFF`。RTSP 图像使用 OpenCV 的 FFmpeg 后端解码，不依赖
NVIDIA 的 `nvvidconv` 插件；用户名和密码中的特殊字符会进行 URL 编码。

## 启动和查看图像

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
ros2 launch hk_camera hk_camera.launch.py
```

Launch 默认自动登录并取流。原始图像发布到 `/hk_camera/image_raw`，类型为
`sensor_msgs/msg/Image`，编码为 `rgb8`，坐标系名称为 `hk_camera`；只有存在订阅者时才转换和发布原图。`/hk_camera/image_preview` 提供独立的轻量预览，默认最大宽度 640、5 帧/秒，可通过 `preview_max_width`、`preview_publish_fps` 节点参数调整。两个话题都使用最新帧队列，预览不会改变原始图像、SDK 抓拍或 DeepSeek 巡检输入。

在另一个终端加载同一环境后查看原始图像或预览：

```bash
ros2 run rqt_image_view rqt_image_view /hk_camera/image_raw
ros2 run rqt_image_view rqt_image_view /hk_camera/image_preview
```

GUI 可单独启动：`ros2 run hk_camera hk_camera_gui`。

## 相机自带的人脸抓拍

节点默认在登录后建立 SDK 报警通道，接收 `COMM_UPLOAD_FACESNAP_RESULT`。
相机须先选 `facesnap` 智能资源，启用人脸抓拍规则、布防时间和上传中心。
程序不会切换相机资源或修改检测规则，也不使用电脑端算法生成检测框。

```bash
ros2 launch hk_camera hk_camera.launch.py \
  enable_face_capture:=true \
  face_capture_dir:=/home/jkbot/Remote_ctrol/captures/hk_faces
```

仅测试报警、不解码连续视频时，可以加 `auto_stream:=false`。
登录成功后应出现 `Face alarm reception active`。让完整、无遮挡的人脸进入
检测区并离开；部分抓拍模式会在目标离开后上传最佳图片。

每次报警使用本机接收时间和人脸 ID 生成文件名，保存：

- `*_face.jpg`：相机的人脸小图；相机仅传背景图时，按它给出的裁剪框生成。
- `*_background.jpg`：该次报警对应的原始背景图。
- `*_boxed.jpg`：在同一张背景图上叠加相机返回的人脸框。
- `*.json`：归一化和像素坐标、人脸 ID、质量分数、相机原始时间和图片路径。

SDK 图片指针只在回调期间有效，接收器先复制图片，再用有界队列交给 ROS
定时器保存。缺少背景图时会按人脸 ID 尝试 `NET_DVR_FindBackgroundPic`；
若仍无背景，只保存收到的人脸图和元数据。坐标无效时保留原图并报告原因，
不会把报警坐标叠加到另一个时刻的视频帧。普通 `/hk_camera/capture` 服务
仍用于手动抓取整张画面。

登录时读取主码流分辨率。如果背景图底部附加抓拍时间文字条，坐标换算使用
原视频区域的尺寸；JSON 的 `background_size` 和 `coordinate_size` 分别记录
整张 JPEG 和有效画面尺寸，避免附加文字条把框拉伸或向下移动。

在另一个终端加载 ROS 环境后，查看最新结果：

```bash
ros2 topic echo /hk_camera/face_capture --once \
  --qos-durability transient_local --qos-reliability reliable
ros2 run rqt_image_view rqt_image_view /hk_camera/face_image
```

`/hk_camera/face_capture` 为 `hk_camera/msg/FaceCapture`，只含坐标和路径；
`/hk_camera/face_image` 为带框场景图。两者使用 transient-local QoS 保留最后一条，
订阅器需使用相同 durability 才能读取历史结果。`rqt_image_view` 在新抓拍到达时
显示图像；查看已经保存的结果可以直接打开 `*_boxed.jpg`。
这个功能验证的是相机的人脸抓拍，不提供姓名比对。关闭接收可设置
`enable_face_capture:=false`。`face_queue_limit` 默认 8，队列满时报告丢弃数量。

测试解析、图片缓冲区复制、坐标边界、裁剪、文件保存和异常输入：

```bash
colcon test --packages-select hk_camera --ctest-args -R test_face_capture --output-on-failure
colcon test-result --verbose
```

## 手动登录和取流

```bash
ros2 launch hk_camera hk_camera.launch.py auto_login:=false
ros2 service call /hk_camera/login std_srvs/srv/Trigger '{}'
ros2 service call /hk_camera/start_stream std_srvs/srv/Trigger '{}'
ros2 service call /hk_camera/stop_stream std_srvs/srv/Trigger '{}'
ros2 service call /hk_camera/logout std_srvs/srv/Trigger '{}'
```

直接运行 `ros2 run hk_camera hk_camera_node` 时，默认等待上述服务调用。
也可使用包内参数文件自动启动：

```bash
ros2 run hk_camera hk_camera_node --ros-args --params-file \
  /home/jkbot/Remote_ctrol/ros2_ws/install/hk_camera/share/hk_camera/config/hk_camera_params.yaml
```
