# 独立气体检测与控制 UI

原有 `gq_otk9633.py` 串口驱动和全部命令保留。新增 ROS2 包 `gas_detector`，只封装读取，不设置阈值、单位、地址或标定。不会依赖导航、底盘、巡航执行器、HTTP 或网关。

## UI 入口

主窗口顶部 **设备执勤**，或 **工具 → 设备执勤**，打开独立窗口。

- 气体检测：打开“气体检测”页签，展开“连接参数”指定固定串口、设备地址、波特率和采样间隔，点击底部“启动气体检测”。读取不需要勾选车辆“启用操作”。默认 `/dev/gq_otk9633`、地址 1、2400/8N1、完整采样间隔 2 秒。
- 主界面底部常驻实时读数、单位和设备报警状态；缩放估算值带 `≈` 标记。停止、异常或过期显示 `—`，不会把旧值继续显示为实时浓度。卡片入口可打开设备窗口，切页不影响更新。
- 云台巡航：打开“云台巡航”页签，勾选主窗口“启用操作”，选择水平速度档位和单侧扫掠时间，点击“启动云台巡航”。水平和俯仰通过 **RS485** 控制；默认勾选“同时俯仰摆动”，范围 **−9°～+15°**，俯仰速度 1 档，可在停止后修改。取消勾选时仅水平往返。复用已有 `/ptz_patrol` 节点；不存在时由 UI 单独启动，并连接或启动相机和 `ptz_pelco`。
- 两项功能都可以在车辆静止、未启动导航时使用。切页或关闭设备窗口继续运行。退出主程序会请求停止本界面开启的功能，只关闭本界面创建的进程。
- 云台巡航期间，方向、变倍和绝对姿态设置按钮被锁定。点击“停止云台巡航”，或相机页“停止云台”，等待停止反馈后恢复。状态失联时保持锁定；恢复新鲜停止反馈后放行。取消“启用操作”也会停止本界面启动的云台巡航。
- 若发现外部气体节点，复用其实际串口参数，界面会显示实际串口/地址/波特率。不会启动第二个串口采集器；停止会关闭采集，不结束外部节点进程。

窗口顶部同时显示气体与云台状态摘要；每页参数区独立滚动，底部启停按钮固定显示，云台页说明当前无法启动的原因。

## 数值和状态

界面突出显示 **原始寄存器数值、设备上报单位、设备报警状态**。低报/高报由设备状态寄存器确定，不使用主机推算的阈值判断。`value_scaled` 是依原驱动推算的小数缩放值，仍标注“待与仪表核对”，不能视为已经完成实机量值确认。

没有新鲜采样时显示 `—`，不会填零或继续把旧值当实时数据。串口不存在、被占用、CRC/超时等错误会显示在状态中，并每 2 秒尝试重连。串口 I/O 在线程中串行执行，保留原驱动每条请求至少 210ms 的间隔；完整采样实际耗时可能超过设置周期。

## 构建与独立使用

```bash
cd /home/jkbot/Remote_ctrol
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install --packages-select gas_detector chassis_nav2 \
  --symlink-install --parallel-workers 2
source ros2_ws/install/setup.bash
```

单独读取，不需要 UI：

```bash
ros2 run gas_detector gas_monitor --ros-args \
  -p port:=/dev/gq_otk9633 -p enabled:=true
```

节点默认关闭，不打开串口。参数都是启动配置，运行中的启停使用服务：

```bash
ros2 service call /gas_detector/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /gas_detector/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 topic echo /gas_detector/data
ros2 topic echo /gas_detector/status
```

`/gas_detector/data` 是 `std_msgs/String` JSON，包含原驱动 snapshot 字段，以及 `sample_time_ns` 和 `port`；`/gas_detector/status` 每 0.5 秒报告 `enabled`、`connected`、`state`、`samples`、`last_error`。停止服务返回表示接受停止请求；`state=stopped` 且 `connected=false` 表示当前读取已结束并释放串口。

单独使用云台巡航的命令保持兼容：

```bash
ros2 run chassis_nav2 ptz_patrol.py
ros2 param set /ptz_patrol enabled true
ros2 param set /ptz_patrol enabled false
ros2 topic echo /ptz_patrol/status
```

云台状态 JSON 包含 `enabled`、`moving`、`active`、`state`、`last_error`、`control_transport=rs485`、实际水平速度/时间/范围，以及 `tilt_enabled`、`tilt_min`、`tilt_max`、`tilt_speed`、实际俯仰角。控制请求超时会先请求停止；停止失败不会报告已停止。节点每 0.2 秒续租 485 驱动的 500ms 租期。仅水平巡航通过 `/ptz/control`，两轴巡航通过 `/ptz/control_patrol`，二者均直接控制 485 云台，不改变镜头变倍。

两轴巡航启动前读取实际角度；俯仰在范围外时先绝对定位到最近的范围端点，随后用连续斜向命令同时旋转和俯仰。接近端点时只降低俯仰速度，实际反馈距端点不超过 0.1° 时换向，水平速度不变。串口轮询使用独立回调组，续租与停止不会排在轮询后面；总线读写仍加锁串行执行。俯仰反馈丢失、范围异常、角度持续不变或命令失败时停止两轴并关闭巡航，需重新启动。到点巡检会暂停两轴，确认停止后才拍照，离点后沿暂停前的俯仰方向继续；人工停止或巡检租期失效后保持停止。仅水平巡航的时间换向和可选水平边界仍保留；水平边界是换向参考，不是硬限位。原独立命令的 `home_*` 回位参数保留，UI 默认不配置回位。

角度读写依据根目录《RS485球机坐标设置与查询协议说明.pdf》：角度使用二进制 0.01°，负俯仰按 360° 回绕；−9° 为 `0x891C`，+15° 为 `0x05DC`。文档标称 9600bps，但本机只读核验在 2400bps 能收到校验正确的回复，9600bps 无回复，所以当前配置保留 2400bps。镜头变倍仍走相机网口；本机 485 Z 返回 0，不作为有效光学倍率。详细核验见[485 巡航修复记录](../../../desktop_ui/docs/rs485_patrol_fix_20261025.md)。

CLI 启用相同的两轴模式（先启动 `ptz_pelco`，参数设置时保持巡航关闭）：

```bash
ros2 run chassis_nav2 ptz_patrol.py --ros-args \
  -p tilt_enabled:=true -p tilt_min:=-9.0 -p tilt_max:=15.0 -p tilt_speed:=1
ros2 param set /ptz_patrol enabled true
ros2 param set /ptz_patrol enabled false
```

升级后需要重启相机/云台节点和桌面程序，以加载新增的 `hk_camera/srv/ControlPTZPatrol` 接口。

依赖 `python3-serial`。原设备说明、固定设备名与权限配置见 [串口说明](gq_otk9633_使用说明.md)。

## 隔离验证

没有连接真实串口、相机或底盘；测试使用假串口数据和隔离 ROS 服务。

```bash
DEVICE_ROS_TESTS=1 ROS_DOMAIN_ID=230 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest -q ros2_ws/src/gas_detector/test
ctest --test-dir desktop_ui/build --output-on-failure \
  -R 'desktop_(devices|camera|console_layout)'
```

测试覆盖只读 Modbus 报文、原始/估算值和报警状态、默认关闭、串口释放、失败不补零、PTZ 续租与换向、停止失败保持活动、启停竞态、UI 独立启停、隐藏继续、超时锁定及操作许可撤销停止。真实气体量值、设备报警和实际云台方向/角度仍需现场验证。

首次接入验证记录：节点测试 **5 passed**，桌面 CTest **13/13**；1366×768 离线界面检查通过，两个设备的启动和停止按钮均可见。另修复共享进程管理器的启动即停止请求丢失，并覆盖主窗口/相机面板退出时的进程回调顺序。


2026-10-19设备执勤界面更新：气体 / 云台独立页签、顶部状态摘要、气体连接参数折叠、角度输入按需展开、底部固定启停和云台启动条件说明。相关三组桌面检查通过；四种设备窗口尺寸及停止 / 异常清空旧读数均使用隔离模拟设备验证，现场结果继续按用户测试记录维护。
