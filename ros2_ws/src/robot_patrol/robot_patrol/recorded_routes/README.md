# 录制路网后台接口

用户操作见仓库根目录《固定路线录制与巡检使用说明.md》。本目录负责纯几何、拓扑、录制草稿、数据库、ROS 服务及现有任务执行器的适配；没有底盘速度发布者。

## 启用与数据

`patrol_node` 参数 `enable_recorded_routes` 默认 false。通过三套 `chassis_nav2` 导航入口同时启用 `enable_patrol:=true enable_recorded_routes:=true`，可一起装载 Nav2 插件和 guard/mux。不要只启动执行器而遗漏 Nav2 的 overlay。

SQLite schema v3，旧 v2 自动备份后迁移。道路、原始样本、草稿、事件及导航上下文都在原 PatrolStore 中，使用同一把数据库锁/事务，不启动第二个数据库写入者。新增路线字段 `navigation_mode`（FREE_NAV / RECORDED_NETWORK）、`network_id`、`network_revision`；旧路线默认 FREE_NAV。旧客户端请求缺少模式时不能改写固定路线。

网络 JSON schema_version=1，发布后还保存 `vehicle`（minimum_turning_radius、footprint）用于核对车辆几何配置：

```json
{
  "schema_version": 1,
  "map_stamp": {"resolution": 0.05, "origin_x": 0, "origin_y": 0, "origin_yaw": 0,
    "width": 100, "height": 100, "localization_sha256": "…", "image_sha256": "…", "navigation_yaml_sha256": "…"},
  "nodes": {"A": {"id": "A", "name": "起点", "x": 0, "y": 0, "yaw": 0}},
  "edges": {},
  "bindings": {},
  "allowed_polygon": []
}
```

边字段为 `id,name,from,to,enabled,speed,blocks,raw_samples`。block 为 `direction`（1/-1）和 `samples`（`x,y,yaw` 数组）；车头 yaw 不因倒车翻转。raw_samples 还带 `t,v`。业务点绑定 `point_id -> [node_id,…]`，最多 8 个状态。道路必须显式指定两端节点；不按距离自动生成反向边或路口。轨迹连接和车辆曲率均需验证。

一个任务最多 20 万个重采样点，单网络最多 3000 节点/1 万边/20 万重采样点；录制原始点上限 20 万，请适时分段。服务单请求上限 32MB。

停车样本至少每 0.5s 保留一次时间、轮速与位姿。对终点刹车后的定位过冲回摆，只在低速反馈、末端位姿已稳定至少 0.8s、末端 15cm 内且车头变化小于 0.10rad 时局部整理样本；保留最终实测终点，该终点到原有驶入轨迹必须在 6cm 内。反向轮速积分达到 2.5cm 时不做该整理，以保留真实慢速倒车。原始 `raw_samples` 和草稿坐标不改写，原始跳变/原地旋转检查、整理后的方向/曲率以及发布前的全车身碰撞检查仍生效。

## 服务

日常录制采用 `START_PATH → MARK_PATH_POINT（重复）→ SAVE_PATH`。开始时自动新建道路库；每次记录点位提交原始道路、导航点与绑定；结束时自动验证碰撞/曲率、生成显式原路返程边、发布道路并创建 `RECORDED_NETWORK` 任务。最后的空尾段可直接收尾。所有持久修改仍由现有 PatrolStore 完成；最终点位、路网、任务和草稿状态为单事务提交。

`inspection_path` 元数据保存 `edge_ids`（原始行驶顺序）、`point_ids`（节点→点位）、`route_id`、`closed` 和非闭环的 `reverse_edge_ids`。派生返程边带 `reverse_of`，采样倒序、运动方向取反，车头 yaw 不加 π；原始驾驶数据保留在原边。返程上限 0.10m/s。仅自动生成任务且点序未改变时使用顺序路径规划；普通高级任务继续使用有向路网规划，可任意排列点位。

顺序任务的计划中，`pass_legs` 和 `pass_step_indices` 保存最多两套逐趟模板。执行器和 RecordedDriver 按趟数选相同模板，不展开大量重复点，不在自交处寻找捷径。非闭环从任一端点开始，`loop_count=1/2/3` 表示单程/往返/再单程；闭环按圈执行。启动使用新鲜 PLAN 摘要并保留原 guard、权限及终止确认。

顺序任务现在支持自动接入：PLAN 在独立 ROS executor 上调用 `/compute_path_to_pose` 的 `RecordedEntry` 规划器，分别尝试两端（闭环仅起点），选择实际接入轨迹较短的可达端。精确 Reeds-Shepp 曲线保留连续车头朝向和全部换向点，必要时利用连续运动学 RRTConnect 搜索绕开障碍，2.5s 内未找到精确到达解时拒绝启动；后台再检查端点、曲率、车身、地图和允许区域。准备完成前仅做计算，不下发运动目标。PLAN 不持有道路全局锁，心跳和停止服务保持响应，规划期间车辆/地图/道路/任务变化会拒绝结果。

`initial_legs` 仅供第一趟使用，在原有完整道路前插入带 `entry:true` 的运动段；`entry` 摘要含目标端点、长度和换向次数。后续趟仅用原 `pass_legs`，接入不增加趟数。所有接入段使用同一 UUID 票据和 guard，接入使用独立的 RecordedEntryRoute 低速 RPP 参数，前进上限 0.20m/s、倒车上限 0.12m/s；规划半径比硬件最小半径增大 25%，为跟踪修正预留转向余量。换向及接入终点采用独立的 3cm / 0.08rad 目标检查，停稳后再次核对朝向。途中遇障等待，不允许绕过接入换向点。自动接入限于点序未改变的简化录制任务，高级路网任意点序与返航仍遵守原来的录制连接约束。

`/patrol/road_command` 类型 `patrol_interfaces/srv/RoadCommand`，请求 `operation` 与 `payload_json`，响应 `success,error_code,message,data_json`。JSON 对象中 `owner_session_id`、`command_id` 由客户端填充；最近 20 条有 command_id 的修改请求按操作/owner/ID 去重，ID 内容冲突拒绝。跨进程重启不提供永久幂等保证，应先 LIST/STATUS 核对后再重试。

网络修改请求携带 `network_id,expected_revision`；修改后网络待发布、版本递增。示例中的 ID 和 revision 应使用上一次真实响应。

| operation | 其他请求字段 | 返回或行为 |
|---|---|---|
| STATUS | 无 | enabled、录制 state/samples/length/error、当前道路 progress |
| LIST | 无 | 活跃地图的 networks 与未处理 drafts |
| START_PATH | name,owner_session_id | 新建并开始录制巡检路径，无需先 CREATE |
| MARK_PATH_POINT | network_id,expected_revision,owner_session_id | 分段、自动建点并继续录制；原地重复点击不建重复点 |
| SAVE_PATH | network_id,expected_revision,owner_session_id | 收尾、自动点位绑定、校验发布、创建任务；也可转换已有连续道路 |
| RECOVER_PATH | draft_id,owner_session_id | 恢复为暂停状态，支持空尾段收尾；原地对齐后可继续录制 |
| CREATE | name | 新的空路网 |
| SHOW_NETWORK | network_id | 在 RViz 发布所选路网 |
| START_RECORD | network_id,name,start_node（空表示新节点） | 开始被动录制，必须有非空 owner |
| PAUSE_RECORD / RESUME_RECORD | owner_session_id | 暂停/原地恢复，暂停期间移位会拒绝恢复 |
| FINISH_RECORD | owner_session_id | 冻结为 REVIEW，返回 review_blocks |
| SAVE_RECORD | network_id,expected_revision,end_node | 保存；空 end_node 新建，`__start__` 闭合本次起点 |
| MARK_NODE | 同 SAVE_RECORD，可选 next_name | 保存当前边，从同一终节点继续录下一段 |
| DISCARD_RECORD | owner_session_id | 放弃活动草稿，原始记录保留 |
| RECOVER_DRAFT | draft_id,owner_session_id | 将持久草稿载入 REVIEW/INVALID，不凭空补线 |
| BIND_POINT | network_id,expected_revision,point_id,node_ids | 替换此点的全部绑定；节点距业务点不超过 8cm |
| UNBIND_POINT | network_id,expected_revision,point_id,node_id | 移除一个绑定 |
| UPDATE_EDGE | network_id,expected_revision,edge_id,enabled,speed | 修改通行开关和速度上限 |
| SET_REGION | network_id,expected_revision,polygon | 设置地图坐标下的允许区域；[] 取消区域限制 |
| PUBLISH | network_id,expected_revision | 校验轨迹、绑定、静态地图完整车身碰撞后发布 |
| REVALIDATE | network_id,expected_revision | 同坐标基准下重新核对地图，保存后仍需 PUBLISH |
| EXPORT | network_id | 网络、map_stamp 与 sha256 |
| IMPORT | export（EXPORT 返回对象） | 同地图指纹、摘要和点位 ID 校验后创建待发布副本 |
| DELETE | network_id,expected_revision | 被路线引用时拒绝删除 |
| PLAN | route_id,loop_count | 校验全任务状态可达性，返回完整 legs/closure/digest |
| EVENTS | run_id | 道路状态变化记录 |

LOAD_LEG / CHECK_LEG 是 BT 内部接口，需要匹配当前真实 NavigateToPose goal UUID 的有效 ticket_id。票据绑定不可变 leg、地图文件签名、run 和进度；暂停、取消、失联、地图变化后停止授权。

录制操作端持续发布 `/patrol/operator_heartbeat`，操作禁用或 3 秒失联后自动暂停录制；该机制只暂停被动采样，不发送运动控制。恢复同一进程仍持有的暂停草稿时，其他在线 owner 会阻止接管。高级 `RECOVER_DRAFT` 进入预览后只可保存/放弃；简化 `RECOVER_PATH` 恢复为暂停状态，原地对齐后可继续采样，也可以直接结束保存。

## 命令行

```bash
source /opt/ros/humble/setup.bash
source /home/jkbot/Remote_ctrol/ros2_ws/install/setup.bash
ros2 run robot_patrol patrol_cli road LIST
ros2 run robot_patrol patrol_cli road CREATE --json '{"name":"厂区道路"}'
ros2 run robot_patrol patrol_cli road STATUS
ros2 run robot_patrol patrol_cli road PLAN --json '{"route_id":"route_实际ID","loop_count":1}'
```

`road` 支持 `--file JSON文件` 代替 `--json`，`--owner` 默认为 cli。CLI 单次录制调用不持续充当操作端；需要在同一会话另开持续 OperatorHeartbeat 发布者，或直接用 Qt 录制。所有命令仍受底盘急停、guard 和任务互锁约束。

`create-route --mode RECORDED_NETWORK --network-id ID --network-revision N` 创建固定路线；`run-route --plan-digest 实际摘要` 使用刚刚 PLAN 的摘要。移动起点、改任务圈数或版本后需重新预览。旧的 create-route/run-route 默认自由导航。

## 话题

| 话题 | 类型 | 内容 |
|---|---|---|
| /patrol/recording/status | std_msgs/String | JSON 录制状态和当前道路进度 |
| /patrol/road_network_preview | visualization_msgs/MarkerArray | 已选路网，可靠/瞬态本地 |
| /patrol/recorded_plan_preview | 同上 | 本次完整任务轨迹 |
| /patrol/recorded_detour_preview | 同上 | 当前锁定的绕行轨迹 |
| /patrol/recorded_progress | std_msgs/String | BT 进度，按 ticket_id 匹配后进入存储 |

正常跟踪有序弧长，不对整条路径全局寻找最近点。闭环、重复边和重叠倒车以当前 leg/block/索引区分。停靠的 20cm 尾段会原样接续到下一个任务段，不能用自由直线补接头。

## 测试

纯几何/有向路网/存储测试直接运行 `pytest ros2_ws/src/robot_patrol/test/test_recorded_roads.py`。真实 Nav2 测试必须显式启用隔离域：

```bash
cd /home/jkbot/Remote_ctrol
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=209 ROS_LOCALHOST_ONLY=1 ROBOT_RECORDED_NAV2_INTEGRATION=1 \
  python3 -m pytest -q ros2_ws/src/robot_patrol/test/test_recorded_nav2_runtime.py
```

该测试只启动真实 Nav2、patrol/guard/mux 与无硬件模拟器。输出在 `/inert_sim/cmd_vel`，不加载底盘桥、串口、CAN 或传感器硬件启动文件。ROS 域需独占且不超过 232；不要与现场运行会话共用域。日志、临时数据库及车辆轨迹在 pytest 临时目录，测试结束只关闭它自己启动的进程。
