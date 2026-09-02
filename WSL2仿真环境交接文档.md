# WSL2 仿真环境交接文档

> 目的：在 Windows 电脑的 WSL2（Ubuntu 22.04 x64）中搭建 Gazebo 仿真环境，承接 M3 自主巡逻开发。
> 背景文档：《巡检机器人远程控制方案（蒲公英组网版）.md》《开发计划.md》
> 原因：Gazebo Classic 在 ARM64 无官方包（[jammy 仅 amd64](https://packages.ubuntu.com/jammy/gazebo)），仿真开发转移到 x64 的 WSL2 上进行；逻辑代码与 Jetson/工控机完全通用。

---

## 一、当前项目状态速览（Jetson 开发机侧）

### 已完成并验证
- **M0 环境**：ROS2 Humble、ffmpeg、MediaMTX v1.20.1（`bin/`）、Python 依赖
- **M1 垂直切片**（局域网/蒲公英组网实测通过）：
  - 视频链：D435 摄像头 → FFmpeg → MediaMTX(RTSP:8554) → 浏览器 WebRTC(:8889)
  - 控制链：网页摇杆 → WebSocket(:8765) → `/cmd_vel`，1s 心跳失控刹车（实测 51 帧指令 + 超时归零）
  - 页面：`http://<板子IP>:8888`（虚拟摇杆 + WASD + 急停）
  - 启动：`bash ~/Remote_ctrol/scripts/start_m1.sh`
- **厂家软件包已就位并通过编译**（8/9）：底盘驱动、三维雷达、二维雷达、IMU、消息包
- **组网**：蒲公英免费版已组网（板子 172.16.2.198）；移动 4G 打洞失败（双层 NAT），待电信/联通卡单层复测，兜底蒲公英付费或 VPS

### 关键技术结论（已确认）
| 项 | 结论 |
|---|---|
| 技术栈 | ROS2 Humble / Ubuntu 22.04，厂家包全部 ROS2，无 ROS1 坑 |
| 底盘接口 | **不是 `/cmd_vel`**！订阅自定义 `yunle_msgs/Ecu`（速度+前轮转角+档位+刹车），需要 Twist→Ecu 适配节点（阿克曼运动学换算，M3 写） |
| 底盘反馈 | `vehicle_status`（车速/轮速/转角/档位/驾驶模式/刹车）、`battery_status`（电压/电流/电量%） |
| 三维雷达 | 速腾聚创 RoboSense，`rslidar_sdk` v1.5.19 |
| 二维雷达 | `hi_driver`（SE/HE/DE 系列） |
| 云台相机 | 海康威视；视频拉 RTSP 即可，SDK 主要用于 PTZ 控制 |
| IMU | 维特智能 `wit_ros2_imu` |
| URDF | `JD03/chassis`（ROS1 catkin 格式，已打 COLCON_IGNORE，按本文第四步移植） |

### 厂家/外部待办
- [ ] 有害气体模块 Modbus 寄存器表（找厂家要）
- [ ] 确认导航/SLAM 软件是否有第二批交付（合同点名 AMCL/Cartographer/TEB/室外3D PCD 导航源码）
- [ ] 电信/联通 SIM 单层 NAT 组网复测
- [x] `ocr_interfaces`/`ocr_node` —— 已从自己上个项目粘入 src/

---

## 二、WSL2 环境搭建（在 Windows 电脑上）

### 2.1 安装 WSL2 + Ubuntu 22.04

PowerShell（管理员）：
```powershell
wsl --install -d Ubuntu-22.04
wsl --version        # 确认 WSLg 可用（Win11 自带，GUI 应用直接显示）
```
装完设用户名密码进入系统。检查 GUI 支持：`echo $DISPLAY` 输出 `:0` 即可。

> 若 Gazebo 画面卡顿：编辑 Windows 用户目录下 `.wslconfig`：
> ```ini
> [wsl2]
> memory=12GB
> processors=8
> ```
> 然后 `wsl --shutdown` 重启 WSL。

### 2.2 Ubuntu 内换国内源（可选但建议）

```bash
sudo sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list
sudo apt update
```

### 2.3 安装 ROS2 Humble

```bash
sudo apt install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
ros2 --help   # 有输出即成功
```

### 2.4 安装 Gazebo + Nav2（x64 全部 apt 直装，这就是转 WSL2 的意义）

```bash
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-twist-mux ros-humble-xacro \
  ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy ros-humble-joy \
  liburdfdom-tools
```

### 2.5 验证环境（先跑官方 demo，别跳过）

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
Gazebo 窗口弹出、RViz 里能给导航目标点让 TurtleBot3 走起来 = 环境合格。

---

## 三、项目代码迁移（Jetson → WSL2）

`.gitignore` 已配好（bin/、ros2_ws 构建产物、运行时数据均忽略）。

**方式一：git（推荐）**
```bash
# Jetson 上：
cd ~/Remote_ctrol && git init && git add -A && git commit -m "M1: 视频+遥控垂直切片"
# 推到 GitHub/Gitee 私有仓后，WSL2 里 clone
```
**方式二：直接打包拷贝**
```bash
# Jetson 上：
cd ~ && tar czf remote_ctrol.tgz --exclude-vcs --exclude='Remote_ctrol/bin' \
  --exclude='Remote_ctrol/ros2_ws/build' --exclude='Remote_ctrol/ros2_ws/install' \
  --exclude='Remote_ctrol/ros2_ws/log' Remote_ctrol
# scp 到电脑（局域网或蒲公英虚拟IP都行）：
#   scp remote_ctrol.tgz 用户名@<电脑IP>:...
# Windows 资源管理器地址栏输 \\wsl$\Ubuntu-22.04\home\<用户名> 把包拖进去
```

WSL2 里解包后：
```bash
mkdir -p ~/Remote_ctrol && tar xzf remote_ctrol.tgz -C ~ --strip-components=1  # 若打包含顶层目录
cd ~/Remote_ctrol/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --continue-on-error
```
> 注意：Jetson 上 `bin/mediamtx` 是 arm64 版，WSL2 用不到可不装；要装就把下载命令里的 `arm64` 换成 `amd64`。

---

## 四、仿真搭建步骤（WSL2 内，对应开发计划 M3）

### 第 1 步：移植车体模型为 ROS2 描述包

```bash
cd ~/Remote_ctrol/ros2_ws/src
ros2 pkg create chassis_description --build-type ament_cmake
cp -r JD03/chassis/urdf JD03/chassis/meshes chassis_description/
mkdir -p chassis_description/launch
check_urdf chassis_description/urdf/chassis.urdf
```
- `CMakeLists.txt` 末尾加：`install(DIRECTORY urdf meshes launch DESTINATION share/${PROJECT_NAME})`
- 新建 `launch/display.launch.py`（robot_state_publisher + joint_state_publisher_gui + rviz2）
- 构建后 `ros2 launch chassis_description display.launch.py`，RViz 看到车模、滑块能动 8 个关节 = 通过
- **记录哪 4 个关节是转向、哪 4 个是轮子**（下一步要用）

### 第 2 步：Gazebo 化（新建 chassis_gazebo 包）

- 复制 URDF 为 `chassis_gazebo.urdf`，加 Gazebo 插件：
  - 运动：`libgazebo_ros_diff_drive.so`（差速近似，关节名填上一步确认的；轮距/轮径按 URDF 尺寸填）
  - 传感器：`libgazebo_ros_ray_sensor.so` 发 `/scan`（模板抄 turtlebot3 的 `turtlebot3_waffle.gazebo.xacro`）
- 写 `sim.launch.py`：Gazebo + spawn_entity + robot_state_publisher（结构参考 `/opt/ros/humble/share/nav2_bringup/launch/tb3_simulation_launch.py`）
- 验证：`ros2 launch chassis_gazebo sim.launch.py` + `ros2 run teleop_twist_keyboard teleop_twist_keyboard` 能开车

> 差速近似的原因：Nav2 只发 `/cmd_vel`，逻辑开发足够；阿克曼特性由第 3 步的 Hybrid-A* 规划器在路径层保证。真车联调时再换阿克曼插件。

### 第 3 步：接 Nav2

```bash
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml ~/Remote_ctrol/config/nav2_sim.yaml
```
- `planner_server` 换 **Smac Hybrid-A\***（生成阿克曼可行路径）
- `controller_server` 先用 RPP；合同点名的 TEB 后期源码编译接入
- 三条链路依次跑通：slam_toolbox 建图存图 → 地图加载 + AMCL 定位 → RViz Waypoint 模式多点巡航

### 第 4 步：巡逻逻辑（M3 核心产出）

- 新建 `patrol_manager` 包：计划加载 → Waypoint Follower 动作调用 → 点位动作序列（停留/抓拍/播报占位）→ 状态机 → 任务记录
- 新建 Twist→Ecu 适配节点（先对仿真输出日志，真车接上即用）
- 验收：仿真里定时触发 3 点路线，无人值守跑完并生成任务记录

---

## 五、WSL2 使用提示

- Windows 浏览器访问 WSL2 里的服务（如 web 页面）直接用 `localhost:端口`，默认转发
- 代码同步：后续 Jetson 与 WSL2 之间用 git 分支管理；**记住两边架构不同**，任何下载的二进制都要按各自架构来
- WSL2 里跑的东西将来原样搬到 i7 工控机（同为 x64 Ubuntu 22.04），零改动

## 六、出问题了找谁

- Gazebo/Nav2 环境问题 → 先重跑 2.5 官方 demo 定位是环境还是模型问题
- 厂家包/协议问题 → 问京控高科（附第一节待办清单）
- 其余 → 把报错贴给 Kimi Code
