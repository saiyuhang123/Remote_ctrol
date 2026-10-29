# 网页与本地设备通信的 ROS 域修正

现场进程环境确认：Qt、导航、巡航 guard、云台与气体节点均为 `ROS_DOMAIN_ID=80`；网页监督脚本、网关、音频节点缺少该环境变量，使用默认域 0。网页 HTTP/WebSocket 正常及“取得控制权”不代表网关已与本地 ROS 设备通信。

这会同时造成网页缺失位姿、BMS、气体与 guard 状态，找不到云台和 Qt 中继服务，也不能把导航和手动控制指令送到本地节点。音频节点原先与网关同在域 0，能接收播放请求；实际声卡输出须另行核验，不能将所有无声问题都归因于 ROS 域。

`start_robot_monitor.sh` 现与 Qt 入口一样，在 ROS/overlay 环境加载后自动读取设备自己的 `config/ros_domain.sh`，不依赖操作者在新终端手工 source。本机 .180 使用 80、.63 使用 63，该配置不跨设备复制。两种入口在调用 ROS setup 前默认设置 `ROS_LOCALHOST_ONLY=1`，避免 setup 将未指定值填成 0；显式传入的设置仍保留。

网页脚本启动时打印实际域与本机通信模式。Qt 和已有导航/设备节点不需为本次脚本修正重启；网页脚本须重启，浏览器重新取权，旧任务与控制命令不重放。急停保持不因本次修复而解除。

同一台机器的两个终端分别运行即可：

```bash
cd /home/jkbot/Remote_ctrol
bash desktop_ui/scripts/run.sh
```

```bash
cd /home/jkbot/Remote_ctrol
bash scripts/start_robot_monitor.sh
```

回归测试在临时项目内实际执行两个启动脚本，子进程全部替换为记录环境的假程序，不启动 ROS、HTTP、音频或车辆。覆盖 .63 / .180 的域、新终端 / 错误继承域，以及本机通信默认值 / 显式覆盖，验证网页后端、网关、音频和 Qt 均得到一致配置：

```bash
python3 -m pytest -q tests/test_startup_ros_environment.py
```
