# 机器人运行状态语音提醒

系统内置 30 条普通话女声 WAV，按运行事件自动播放。机器人仅解码本地文件，不运行 Breeze 或在线语音模型。原有网页音频上传、文件播放、停止和手动 `/tts_say` 接口保留。

## 构建和启动

在已完成项目导航依赖构建的工作空间中运行 `bash scripts/build_web_patrol.sh`。也可单独构建音频服务：

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/rosproject/Remote_ctrol/ros2_ws
colcon build --symlink-install --packages-up-to audio_player
source install/setup.bash
ros2 run audio_player tts_node
```

运行依赖包括 `ffplay`（Ubuntu 的 `ffmpeg` 软件包）和系统可用的音频输出。播放器使用系统默认声卡；需要指定 PulseAudio 输出时，在启动前设置 `PULSE_SINK`。USB 声卡的音量和静音状态由系统音频设置管理。

二维 AMCL、Cartographer 和三维 FAST-LIO 的标准 Nav2 启动流程会自动启动服务；网页监控脚本保留 `ENABLE_AUDIO=1`。同一用户、同一 `ROS_DOMAIN_ID` 下，第二个启动入口等待并复用现有服务，不创建第二个播放器；当前服务退出后，仍存活的备用入口接替。

提示观察节点名为 `/robot_voice_notifications`，播放器为 `/tts_node`。观察节点不发布速度、急停或导航指令；声音播放和语音合成均不占用导航执行线程。独立 NDT/CSV 工程的自定义路径执行没有冒充 Nav2 到点事件，需另行对接其结果接口。

## 事件与播报

| 类别 | 自动提示 | 触发依据 |
|---|---|---|
| 就绪 | 系统已就绪 | Nav2 Action 可用、地图到车体 TF、里程计、急停反馈和播放器状态新鲜并稳定 |
| 巡检任务 | 开始、暂停、继续、取消、全部完成、中断、失败 | `/patrol/status` 的实际状态变化，按执行实例和运行编号去重 |
| 巡检点 | 已到达、开始点位巡检、点位巡检完成 | `/patrol/events`；到点须先通过停稳确认，点位完成须完成识别和停留步骤 |
| 返航 | 开始返回、已返回 | 独立返航执行状态；返航成功不会播成巡检全部完成 |
| 普通 Nav2 单点 | 已到达巡检点、导航未完成 | 新目标执行结果；成功后还需新鲜里程计连续静止 0.5 秒，历史结果和巡检子目标不重复播报 |
| 视觉 | 检测到人员 | 新鲜 `/vision/events` 中的 `person` 事件，默认至少间隔 30 秒；人脸抓拍不等同于陌生人或入侵 |
| 安全和设备 | 急停触发/解除、电量偏低/不足、气体报警、定位/传感器/相机异常 | 实际反馈及明确故障；停用设备、无效电量、过期数据不会当作报警读数 |
| 操作 | 本地/远程控制切换、远程连接中断、照片已保存 | 控制权实际交接、活动会话失联和独立手动照片保存成功 |
| 报告 | 巡检报告已生成 | 主库报告事务成功提交后发布事件 |
| 对讲 | 对讲已接通、对讲已结束 | 声卡通道实际建立及关闭；启动失败不播“已接通” |

设备提醒覆盖现有接口实际提供的状态，不推断没有反馈接口的硬件故障。

## 音量、开关与规则

30 个音频位于本包 `prompts/`，安装到 `share/audio_player/prompts/`。它们是 24000 Hz、单声道、16 bit PCM WAV，已统一调整音量。

`config/prompts.json` 列出每条文案、文件名、`enabled`、`priority`、`cooldown` 和 `ttl`（待播有效秒数）。默认低电量阈值为 20%，严重低电量为 10%，带 3 个百分点回差；这些是可配置的软件提醒值，应按现场电池要求调整。气体报警使用仪表反馈的 `low_alarm` / `high_alarm`，不对估算浓度自行设定安全阈值。

```bash
# 停用自动状态提示，保留网页和手动音频播放；重启音频服务生效
export ROBOT_ENABLE_PROMPTS=0

# 自定义配置或录音目录，文件名仍由配置白名单决定
export ROBOT_PROMPT_CONFIG=/绝对路径/prompts.json
export ROBOT_PROMPT_DIR=/绝对路径/提示音目录

# 仅阻止标准导航入口自动启动音频服务
export ROBOT_ENABLE_AUDIO=0
# 网页脚本是否启动音频服务仍由 ENABLE_AUDIO=0/1 决定
```

修改阈值或开关后重启持有音频服务的启动流程；备用入口不会覆盖已经运行服务的配置。自定义 `odom_topic`、`battery_topic`、`base_frame` 也写在配置中。

播放队列有容量限制和有效期；常规提醒按顺序播放。急停、严重低电量和气体报警可打断较低优先级播报。暂停、取消、任务切换和中断会清除该任务过期的导航提示，既清待播队列，也停止正在播放的旧提示。

对讲优先：通话准备和进行期间丢弃其他状态提示，不在挂断后补播旧消息。接通提示是例外：播放短提示时暂时静音对讲的 PCM 输入输出，防止叠音和将提示声送回远端；提示结束后继续通话。设备报警状态仍由原监控界面显示，语音不替代原有安全联锁。

`/audio/notify` 是内部固定音频请求接口，仅接受配置中的键，不接受任意路径；原 `/audio_play` 继续接受上传目录内的文件名。`/audio/status` 额外提供当前 `notification` 键，便于观察实际播放和对讲静音状态。

## 验证

```bash
bash scripts/test_robot_voice.sh
```

脚本使用 localhost 的隔离 ROS 域 219 和 226，音频输出为 SDL dummy，不连接底盘或真实相机；验证真实 DDS、WAV 解码、事件触发、对讲互斥和双入口接替。巡检引擎的纯逻辑回归可另运行：

```bash
python3 -m pytest ros2_ws/src/robot_patrol/test/test_engine.py -q
```

上车后仍需通过实际声卡核对音量、现场可懂度，以及完整任务的到点、取消、返航与报警时序。
