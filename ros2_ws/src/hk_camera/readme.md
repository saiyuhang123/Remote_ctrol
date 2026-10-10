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

Launch 默认自动登录并取流。图像发布到 `/hk_camera/image_raw`，类型为
`sensor_msgs/msg/Image`，编码为 `rgb8`，坐标系名称为 `hk_camera`。

在另一个终端加载同一环境后查看：

```bash
ros2 run rqt_image_view rqt_image_view /hk_camera/image_raw
```

GUI 可单独启动：`ros2 run hk_camera hk_camera_gui`。

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
