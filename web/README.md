# 网页控制台与日常使用

逐项操作见[控制界面详细使用手册](../控制界面详细使用手册.md#十五网页控制台)，包括启动服务、本地交接控制权、视频、遥控、路线、返航和记录查询。

启动方式保持 `bash scripts/start_robot_monitor.sh`，本地先启动实际导航和需要使用的设备。浏览器访问机器人主机的 `http://<IP>:8888/console.html`。`monitor.html`、`tasks.html`、`records.html` 旧地址仍可使用，会自动进入控制台的对应页面。

现场按顺序验证本地、设备、导航和网页功能，使用[实车功能测试清单](../实车功能测试清单.md)。

## 全上位机视觉开关

监控台的人脸、行人、车牌三个开关均控制已运行的 `/detector_node`：分别对应 YuNet、YOLO26n、LPD-YuNet + 号码识别。先在 Qt 启动相机/检测/记录，再按原流程交接并取得网页控制权；人脸不再依赖海康抓拍规则或 face_status。网页仍不启动系统进程。模型异常时也可以关闭对应功能；旧人脸历史保留。当前默认不向旧业务后端重复上传，网页记录页读取原有巡检摘要，完整小图仍在 Qt“查看视觉记录”。

## 界面样式与布局

网页使用统一的深色青绿主题，样式集中在 `theme.css`，装饰图标由本地 `ui.js` 绘制，无需外部字体、CDN 或前端框架。控制台导航、连接/控制权状态、释放控制权和急停始终位于外层。

- 监控台：上方固定展示行驶、定位、导航和气体概览；地图为主要区域，附图例和“适应窗口”按钮；右侧设备面板可滚动查看视频、遥控、云台、视觉和气体详情。
- 任务页：左侧选择路线、设置圈数和查看执行反馈，右侧查看地图、控制状态和最近记录；路线选中态和执行/取消/返航按钮采用不同颜色。
- 记录页：四项统计卡片、巡检明细表格、车牌标签及明确的空记录和错误提示。
- 窄屏：状态卡片变为两列，地图与设备面板上下排列，表格在容器内横向滚动；外层导航换行，急停保持可见。已检查 1440、1024、768、390px 宽度。

这次界面改造后，原有 6 项浏览器功能回归通过。预览均使用模拟数据，未连接实车：[监控台](docs/console_monitor_simulated.png)、[巡检任务](docs/console_tasks_simulated.png)、[巡检记录](docs/console_records_simulated.png)、[手机视图](docs/console_mobile_simulated.png)。

## 页面与控制连接

`console.html` / `console.js` 管理唯一 WebSocket、控制权和心跳；三个 HTML 页面作为同源视图加载，通过 `page-bridge.js` 转发操作和反馈。原有页面导航及浏览器前进/后退只切视图，不关闭控制连接；记录页没有独立控制连接，但外层仍维持任务心跳。

切视图时先结束手动速度和云台按住操作，自动路线或返航继续运行。关闭、刷新整个控制台、真正断网、心跳丢失和本地接管仍按网关原策略取消网页任务。重连不恢复控制权或任务；本地任务不受网页断开影响。外层始终保留急停和释放控制权按钮。

## 本轮修复

- 圈数必须是 `0～100000` 整数，0 表示持续循环。地图/路线版本必须是正整数。后端在占用任务槽和取消旧路线之前校验；同步提交异常释放任务槽，允许重试。前端按 request_id 管理提交中的状态，启动失败不再一直禁用执行按钮。
- 气体显示同时检查采样和设备状态的新鲜度、采集/连接状态及错误；停止、异常、过期或断开时清空实时值和正常/报警判断。网络停更时也会按本地经过时间失效。未核对的小数缩放继续标注 `≈`。
- 云台按住每 200ms 续租；松开、失焦、切页停止，漏掉松开消息时网关的 750ms 按住租期也会终止续转。持续按住可以超过 5 秒。相机原有 500ms jog 租期继续生效。
- 地图每 2 秒检查 ID、revision、hash、尺寸、分辨率和原点；初始无图会自动恢复，有变化重载，无变化保留当前视图。元数据异常时停止显示旧图，避免旧图配新坐标。
- 急停反馈为 true 时仍显示已知的急停状态，不再误显示为“状态未知”。

## 回归测试

浏览器测试只绑定本机随机 HTTP/WS 端口，使用模拟地图、设备和任务数据，不连接 ROS、真实网关、相机或车辆。运行时不需要 Playwright，只有测试需要安装它。

```bash
python3 -m pip install --user playwright
python3 -m playwright install --only-shell chromium
WEB_BROWSER_TESTS=1 python3 -m pytest -q tests/web/test_daily_flows.py
node tests/web/map_transform_test.js
```

浏览器覆盖：巡航后切换监控/记录/任务而不断线，关闭控制台取消，错误圈数与拒绝后重试，气体停止/过期/恢复，地图延迟出现和版本更新，云台长按超过 5 秒，真正断线后取消且不自动恢复，以及无海康人脸服务时的上位机开关和模型异常后的关闭。

本轮验证：浏览器 **6 passed**、网关 **11 passed**、相关后端/存储/执行逻辑回归 **19 passed**，地图 yaw 0/30/90° 测试通过；网关 ROS 包构建成功。截图中的地图、任务、气体均为隔离测试数据：[任务视图](docs/console_tasks_simulated.png)、[监控视图](docs/console_monitor_simulated.png)。真实设备和实车效果仍需现场验收。

网关回归使用隔离 ROS 域和假的 Action 客户端，不连接真实服务：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROBOT_GATEWAY_INTEGRATION=1 ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1 \
  PYTHONPATH="$PWD/ros2_ws/src/robot_gateway:$PYTHONPATH" \
  python3 -m pytest -q ros2_ws/src/robot_gateway/test
```

默认 WS 端口 8765。需要不同部署端口或隔离测试时，控制台 URL 可以使用 `?ws_port=<端口>#monitor`；该参数不保存控制 token。
