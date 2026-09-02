# 1. 加载工作空间环境
source ~/kybot_ws/install/setup.bash

# 2. 设置海康 SDK 库路径 (运行时需要)
export LD_LIBRARY_PATH=/home/nvidia/kybot_ws/src/HK/C++demo/consoleDemo/linux64/lib:$LD_LIBRARY_PATH

# 3a. 启动 Qt GUI 版
ros2 run hk_camera hk_camera_gui

# 3b. 或者启动纯 ROS2 节点 (headless)
ros2 run hk_camera hk_camera_node

# 3c. 或者用 launch 文件 (可传参数)
ros2 launch hk_camera hk_camera.launch.py ip:=192.168.1.100 channel:=1
