# JD03 FAST-LIO 建图部署交接

交接日期：2026-10-22，Asia/Shanghai。现场工作目录：`/home/jkbot/Remote_ctrol`。

**状态：部署代码已修改并编译，正在做整包 1 倍速回放验收，尚不能宣布部署验收完成。用户要求先生成交接文档，由其他模型继续。请先检查正在运行的回放，不要重复启动或把现有修改全部覆盖。**

## 1. 用户目标与已经认可的结果

用户需要：以后使用日常实时 FAST-LIO 建图入口，实时看到稳定点云；结束建图点击保存后，自动得到类似本次离线修复结果的完整、重力竖直 PCD，不再每次人工离线修图。

用户已经看过并认可下面这张离线重建图的外观：园区完整，Z 方向正常。

```text
/home/jkbot/Remote_ctrol/maps/jd03_rebuilt_20261022/jd03_map.pcd
/home/jkbot/Remote_ctrol/maps/jd03_rebuilt_20261022/map_overview.png
/home/jkbot/Remote_ctrol/maps/jd03_rebuilt_20261022/README.md
/home/jkbot/Remote_ctrol/maps/jd03_rebuilt_20261022/validation.json
```

这张图有 1,554,307 点，24,869,072 字节，0.10 m 体素；SHA256：

```text
0f101c959613f96cf471dfcba9a04a06d2a06e6cf64d308cd640799dd4f15fb5
```

该图通过的是离线几何一致性检查，尚无实车定位、导航验收。轨迹高度范围 −0.283～+0.371 m，留出重叠源关键帧的位置残差最大 0.1044 m。这不是有外部真值的绝对定位精度。

用户明确说没有严格回到初始位置、初始朝向，估计结束距起点 1～5 m。新图轨迹终点约 `[4.56, 22.57, 0.31] m`，这个差异仍需现场核对，不能强制把终点拉回原点。

用户已明确授权“帮我部署”。不需要再询问是否可以继续本次部署，但应如实区分已编译与已完成验收。

## 2. 数据、硬件与确认的外参

```text
车辆：JD03，四轮转向
雷达：RS-Helios-16P
IMU：WIT，100 Hz
点云：/velodyne_points，PointCloud2，约 10 Hz
IMU：/imu/data，sensor_msgs/Imu，约 100 Hz
点云字段：x y z intensity ring time，time 单位秒

bag：/home/jkbot/bags/fastlio_sam_jd03_20261022_083713
时长：1099.289 s
消息：10993 条点云，109930 条 IMU
```

用户现场核对的关键事实：**IMU 外壳 Y+ 与雷达正向一致**。沿用雷达 X+ 朝车前、双方 Z+ 向上的约定：

```yaml
base_from_lidar:
  translation: [0.32, 0.0, 0.46]
  rpy: [0.0, 0.0, 0.0]
base_from_imu:
  translation: [0.32, 0.0, 0.22]
  rpy: [0.0, 0.0, -1.5707963267948966]

# 将雷达坐标转换到 IMU 坐标，Rz(+90°)：
extrinsic_T: [0.0, 0.0, 0.24]
extrinsic_R: [0.0, -1.0, 0.0,
              1.0,  0.0, 0.0,
              0.0,  0.0, 1.0]
```

硬件配置源：`ros2_ws/src/chassis_nav2/config/hardware.yaml`，外参已更新。时间偏移仍为 0.0，尚未完成精确时延和小角度联合标定。不要把软件权重写成 IMU 实测精度。

## 3. 必须分清两个系统

### 本次正在部署的日常上位机建图系统

```text
主仓库：/home/jkbot/Remote_ctrol
统一启动：chassis_nav2/launch/bringup.launch.py，mode:=mapping
前端：ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2
ROS 包名：fast_lio_localization
可执行：fastlio_mapping
后端：ros2_ws/src/fastlio_loop_closure
桌面程序：desktop_ui/build/robot_desktop
```

这是本次实际修改并编译的目标：IKFoM FAST-LIO2 前端 + 独立建图后端 + 桌面保存入口。

### 用户早先测试的独立 Fast-LIO-SAM

```text
/home/jkbot/fastlio_sam_ws/src/FAST-LIO-SAM
RightTr ROS2 移植，基础 044f4a1，本地稳定提交 395f891
```

这里只修正了 `config/mapping/velodyne_jd03.yaml` 和 `config/odom/velodyne_jd03.yaml` 的外参，并曾同步安装。**本次没有把倾角约束移植到其 SimpleEKF/GTSAM 中。不要告诉用户单独启动这套 Fast-LIO-SAM 就已经用上完整新流程。**

历史失败修图均在下面的隔离目录，不要用作导航地图：

```text
/home/jkbot/fastlio_sam_ws/failed_correction_20261022/
```

## 4. 已完成的代码修改

以下修改均在工作区，**没有提交 Git**。

### 4.1 主 FAST-LIO 前端

修改或新增：

```text
ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/src/laserMapping.cpp
ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/include/gravity_observation.hpp          新文件
ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/tests/gravity_observation_test.cpp      新文件
ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/CMakeLists.txt
ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/config/velodyne.yaml
```

做法：把先前离线成功的试验前端修改合入主源码。EKF 的 LiDAR 观测中追加重力方向观测，只使用 WIT 四元数提供的倾角，不融合设备 yaw。IKFoM 的流形状态更新仍由原滤波器处理。

- 检查四元数有效性、IMU 姿态协方差标记、加速度模长 9.0～10.6 和姿态/加速度方向一致性。
- 每帧用有效 IMU 上方向分量中位数抑制尖峰。
- `gravity_aid.enable` 默认 false，定位配置保持不启用。
- 日常建图 `velodyne.yaml` 显式启用，`gravity_aid.sigma_deg: 0.005`。
- 0.005° 是本次平缓园区路线验证过的强约束数值权重，不代表传感器真实精度，且观测与原始 IMU 有相关性。
- `preprocess.scan_line` 从占位 32 改为实际 16。
- `publish.dense_publish_en` 改 false，与成功回放配置一致；body 点云仍包含去畸变后的完整帧，供关键帧保存。

### 4.2 建图几何与保存后端

```text
ros2_ws/src/fastlio_loop_closure/fastlio_loop_closure/geometry.py               新文件
ros2_ws/src/fastlio_loop_closure/fastlio_loop_closure/live_loop_closure.py      重写
ros2_ws/src/fastlio_loop_closure/test/test_geometry.py                         新文件
ros2_ws/src/fastlio_loop_closure/package.xml                                  加 scipy 依赖
ros2_ws/src/fastlio_loop_closure/setup.py                                     更新描述
```

替换了旧的 Open3D 全 6DoF PoseGraph 和保存时单条首尾回环逻辑。旧代码还存在接受回环时引用未定义 `dt` 的问题，现已不使用该段代码。

新流程：

1. 精确按 stamp 配对 `/Odometry` 与 `/cloud_registered_body`，两个到达顺序均支持。
2. 订阅 `/imu/data` 取得初始世界重力参考，导出时整体旋转到重力竖直地图坐标。
3. 按位移 0.8 m / 转角 10° / 最大 2 s 抽关键帧，排除静止重复帧；body 点云以 0.15 m 体素保存 XYZ+intensity。
4. 关键帧和原始 odometry 写到 `<map>_session_<日期>_<PID>/`，便于恢复与诊断。
5. 后台匹配历史邻近子地图，排除地面，先 point-to-point ICP，再带 Huber 损失的 point-to-plane ICP。
6. 结束保存时完成待处理匹配，选取多条空间分开的真实相对位姿约束；每三条取一条入图，其余源帧留出验证。
7. 优化 xyz+yaw，固定前端已经约束的 roll/pitch，固定第一帧；没有 endpoint=origin 约束。
8. 检查优化变形、收敛、留出残差、倾角一致性、消息配对缺失等。
9. 检查通过才原子写入最终 `<map>.pcd`，保留 intensity，生成 `<map>.validation.json`。
10. 匹配不足/优化不通过则返回失败，并保留 `<map>_UNVERIFIED.pcd` 与会话；桌面不继续导出栅格。

当前参数以这条长园区路线为对象：历史时间差 300 s，搜索半径 5 m，索引差 30，目标子地图 ±3 帧；重复路段采样间距 2 m。非地面 fitness 使用 0.4 m 对应距离，要求 ≥0.85，RMSE ≤0.20 m。至少 6 条合格匹配。

单条匹配位姿修正门限：XY <2 m，Z <0.4 m，roll/pitch 合成 <1.5°，yaw <1°。图优化后最大位置调整 <2 m、最大旋转调整 <1°；留出位置残差 <0.4 m、yaw 残差 <0.6°；solver 必须成功。

**这是“实时前端显示 + 保存时自动优化”，不是把图优化结果实时反写 EKF 状态。** 修正地图/路径在保存后发布；显示仍用 `lio_odom` 坐标，磁盘 PCD 为重力竖直局部地图，报告保存 `map_from_lio_rotation`。

### 4.3 启动链路与服务

```text
ros2_ws/src/chassis_nav2/chassis_nav2/bringup.py
ros2_ws/src/chassis_nav2/launch/bringup.launch.py
ros2_ws/src/chassis_nav2/launch/fastlio_mapping.launch.py
ros2_ws/src/chassis_nav2/config/hardware.yaml
```

- 建图模式启动上述前端与新后端，使用已修正外参。
- 新增 `use_sim_time` 启动参数，默认 false，支持隔离回放。
- 建图中前端原始保存服务重映射为 `/fastlio/raw_map_save`，原始地图路径为 `<map>_fastlio_raw.pcd`。
- 后端同时提供 `/loop_closure/save_map` 与 `/map_save`，两者均执行检查与优化，避免旧终端命令直接把原图当最终图。
- `fastlio_mapping.launch.py` 的保存提示改为后端服务。

### 4.4 桌面端

```text
desktop_ui/src/adapters/ros_adapter.cpp
desktop_ui/tests/ros_services_test.cpp
desktop_ui/tests/map_saver_test.cpp
```

- FAST-LIO 保存固定调用 `/loop_closure/save_map`，没有服务就提示未就绪，不再静默回退原始地图。
- 此项服务超时至少 180 s；其他设备服务维持原超时。
- 保留原来的 PCD 成功保存后自动导出 PGM/YAML 和网页地图流程。
- 已更新 mock 服务测试和“后端不存在时不能回退原图”的测试，但这批 Qt 测试尚未执行。

### 4.5 端到端回放脚本

```text
scripts/validate_fastlio_mapping.py                                          新文件
```

该脚本启动**实际安装的统一 bringup**，禁用传感器驱动、底盘通信、控制和 RViz，播放指定 bag，最后自动调用 `/map_save`，检查导出的 PCD、点数、SHA256、报告及 odometry 消息覆盖率。

使用独立 ROS 域与 `ROS_LOCALHOST_ONLY=1`。运行中输出 `progress.json` 和 `launch.log`，结束后输出 `replay_validation.json`、`map.pcd`、`map.validation.json` 等。

## 5. 编译与已完成检查

已经成功执行：

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
cd /home/jkbot/Remote_ctrol/ros2_ws
colcon build --packages-select fast_lio_localization fastlio_loop_closure chassis_nav2 \
  --symlink-install --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release

cd /home/jkbot/Remote_ctrol
cmake --build desktop_ui/build --target robot_desktop -j2
cmake --build desktop_ui/build --target desktop_ros_services_test desktop_map_saver_test -j2
```

检查结果：

- `test_geometry.py`：6 项通过。涵盖真实非零相对闭环、保留坡度和真实终点、单边拒绝、矛盾留出匹配拒绝、低质量/大修正拒绝、PCD 原子替换失败保留旧图及 intensity。
- `ctest --test-dir ros2_ws/build/fast_lio_localization --output-on-failure`：2 项通过，重力 Jacobian/不引入 yaw 与原子保存。
- Python compileall 通过，`git diff --check` 通过。
- `ros2 launch chassis_nav2 bringup.launch.py --show-args` 通过。
- 已运行中的实际前端参数确认：gravity enable=true、sigma=0.005、scan_line=16、外参 Rz(+90°)、Tz=0.24、use_sim_time=true。
- Qt 两个测试目标已编译成功，**尚未执行**。
- 完整 1 倍速回放仍进行中，**尚无最终导图验收结果**。

环境有已有兼容性警告：系统 SciPy 1.8 对用户目录 NumPy 1.26.4 提示版本范围，相关单元测试与已有离线重建实际运行通过。不要为消除警告贸然改全局 Python 依赖。

## 6. 交接时正在运行的验收

快照时间约 2026-10-22 11:25，进程可能在接手时已经结束，**必须先看结果文件与 PID 现状**。

```text
回放主进程 PID：162037
FAST-LIO PID：162105
新后端 PID：162107
rosbag play PID：162260
launch 进程组：162087
回放进程组：162260
ROS_DOMAIN_ID：213（仅当前这次运行）
ROS_LOCALHOST_ONLY：1
```

当前运行命令：

```bash
python3 -u scripts/validate_fastlio_mapping.py \
  --bag /home/jkbot/bags/fastlio_sam_jd03_20261022_083713 \
  --output /home/jkbot/fastlio_sam_ws/deployment_20261022_111536/replay_1x
```

注意：脚本默认域后来改成 **217**，避免与桌面 `desktop_map_saver` CTest 默认域 213 冲突；已运行的这次仍然是 **213**，不会随磁盘脚本修改而改变。

输出目录与当前关键帧会话：

```text
/home/jkbot/fastlio_sam_ws/deployment_20261022_111536/replay_1x/
  launch.log
  playback.log
  progress.json
  runtime_parameters.json
  map_session_20261022_112219_162107/
    keyframes/
    keyframes.jsonl
    odometry.jsonl
    gravity_reference.json
    session.json
```

11:24:49 左右进度：点云 1472、IMU 14732、odom/body_cloud 各 1469，63 个关键帧，传感器进度约 146.8 s，Z≈−0.217 m，时间匹配缺失 0，错误为空。前端倾角观测 accepted=1477 / rejected=23。此时还未返回重叠路段，0 条回环是正常的。

正常全程约 18 分 20 秒，预计 11:41 左右完成，之后还需要导图时间；不要仅因为当前没有最终 PCD 就重启。

建议接手先执行：

```bash
cd /home/jkbot/Remote_ctrol
pgrep -af 'validate_fastlio_mapping|fastlio_mapping|live_loop_closure|ros2 bag play'
cat /home/jkbot/fastlio_sam_ws/deployment_20261022_111536/replay_1x/progress.json
tail -30 /home/jkbot/fastlio_sam_ws/deployment_20261022_111536/replay_1x/launch.log
ls -l /home/jkbot/fastlio_sam_ws/deployment_20261022_111536/replay_1x/
```

若结束，优先查看 `replay_validation.json` 和 `map.validation.json`。如果有失败报告或 `_UNVERIFIED.pcd`，应诊断失败原因，不能把候选文件改名当作通过。

当前回放由工具会话启动，前一模型的 unified exec 会话 ID 为 66551；新模型不一定能访问该会话，文件与进程状态更可靠。

## 7. 下一模型应完成的事项

1. 先等待/检查正在运行的整包回放结果。检查无崩溃、无位姿 NaN、关键帧覆盖完整，保存服务实际成功，报告几何检查通过。重新读 PCD 并检查点数、有限值、SHA256。
2. 为新部署导出的图生成或检查俯视/侧视图，与已认可的 `maps/jd03_rebuilt_20261022/` 结果比较。观察高度、建筑重影、园区结构和重复路段；不能只看首尾距离。
3. 执行已编译的 Qt 服务与地图保存测试。**当前回放使用域 213，不要同时直接运行默认域也是 213 的 `desktop_map_saver` CTest。** 可以用其他域运行二进制，或等待回放退出再运行 CTest。
4. 验证保存成功后 PCD→PGM/YAML 的实际导出链路，确保上位机使用新的后端服务。修改代码后重新编译对应目标，不要只改源码。
5. 更新 `ros2_ws/src/fastlio_loop_closure/README.md`：当前仍是旧说明，描述全 6DoF、首尾单边、无回环回退原图等，已经不适用。桌面/操作文档也应写清实时前端 + 保存时优化。
6. 审查长园区固定门限对下一次采集的适用性。当前 300 s 历史差、6 条合格匹配会使短路线或没有重复路段时无法输出“已检查最终图”；应明确提示并保留候选，必要时做经过验证的参数化，不要为了让验收通过直接放松质量门限。
7. 审查失败保存、重复保存、保存时继续收到新关键帧、旧 PCD 与验证报告备份一致性等边界。特别是 bringup 现有逻辑只备份旧 PCD，未同步备份旧 `.validation.json`，有遗留报告误读风险，尚未处理。
8. 确认现有地图、失败隔离目录未被误覆盖。交付时明确部署入口是主仓库上位机 FAST-LIO，独立 Fast-LIO-SAM 仍不是同一套完整流程。
9. 全部通过后才向用户报告“已部署并通过原 bag 验收”。实车新路线仍需正常定位验证；地图倾角正常不等于绝对几何精度已经有真值认证。

推荐的相关测试命令：

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
cd /home/jkbot/Remote_ctrol

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=3 \
PYTHONPATH=/home/jkbot/Remote_ctrol/ros2_ws/src/fastlio_loop_closure \
python3 -m pytest -q ros2_ws/src/fastlio_loop_closure/test/test_geometry.py

ctest --test-dir ros2_ws/build/fast_lio_localization --output-on-failure

# 当前回放域 213 运行期间，可在其他私有域单独执行桌面测试：
QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 \
  desktop_ui/build/desktop_map_saver_test
QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1 \
  desktop_ui/build/desktop_ros_services_test
```

若确需重新整包回放，先确认现有回放已经退出，用新的输出目录：

```bash
python3 -u scripts/validate_fastlio_mapping.py \
  --bag /home/jkbot/bags/fastlio_sam_jd03_20261022_083713 \
  --output /home/jkbot/fastlio_sam_ws/deployment_20261022_111536/replay_retry \
  --rate 1.0 --domain 217
```

脚本要求输出目录此前不存在。不要复制已有结果伪装成新回放成功。

## 8. 备份与复现依据

本次部署前已备份关键源码和旧前端二进制：

```text
/home/jkbot/fastlio_sam_ws/deployment_20261022_111536/
  fastlio_mapping.previous
  ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/...
  ros2_ws/src/fastlio_loop_closure/fastlio_loop_closure/live_loop_closure.py
  ros2_ws/src/chassis_nav2/chassis_nav2/bringup.py
  ros2_ws/src/chassis_nav2/config/hardware.yaml
  desktop_ui/src/adapters/ros_adapter.cpp
```

该备份是在外参已经正确更新之后创建的；不是最初 R=I 配置。旧二进制可供回退，但不能据此恢复失败修图为导航图。

同一目录另存了 `working_tree_at_handoff.patch`（已跟踪文件的工作区差异）、`new_files_at_handoff/`（新增源码快照）、`handoff_snapshot.json`（提交与状态快照），以及本文档副本。补丁不包含未跟踪文件，不能只应用补丁而遗漏新增源码。

先前已成功的离线试验代码与数据：

```text
/home/jkbot/fastlio_sam_ws/rebuild_20261022/
  gravity_ws/                         原先成功的隔离前端
  replay_reference.py                原 bag 回放与关键帧采集
  export_candidate.py                重力对齐、候选图输出
  check_repeated_road.py              重复路段非地面配准
  refine_real_overlaps.py             xyz+yaw 图优化与留出检查
  reference_yaw90_gravity/            成功前端回放，861 关键帧
    refined/                         先前成功优化结果
```

新部署不是简单复制静态修正后的 PCD，而是将上述有效方法接到日常启动和保存链路。它尚未完成整包验收；下一模型应在现有工作区继续收尾。
