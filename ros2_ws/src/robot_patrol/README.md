# robot_patrol

本地点位、路线和多点巡航执行器。它通过标准 Nav2 `NavigateToPose` Action 逐点驱动车辆，不依赖 HTTP 后端或网关。

## 构建

```bash
cd /home/jkbot/Remote_ctrol/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select patrol_interfaces robot_patrol --symlink-install
source install/setup.bash
```

## 启动

巡航执行器必须与独立 `patrol_guard` 一起使用。底盘 bringup 必须启用
`enable_patrol:=true`，这样现有 `twist_mux` 才会加载带 `/patrol/stop_lock`
的配置；默认未启用时保持原有单点导航/手动行为。

```bash
# 一条命令同时启动 patrol 仲裁 mux、巡航执行器和独立 guard
ros2 launch chassis_nav2 amcl_localization.launch.py \
  enable_control:=true enable_patrol:=true ...

# 如不使用底盘 bringup 的联动入口，也可单独启动配套节点
ros2 launch robot_patrol patrol.launch.py enable_guard:=true
```

`chassis_nav2` 在 `enable_patrol:=true` 时会一并加载
`robot_patrol/patrol.launch.py`，不会再出现“patrol 锁已启用但没有 guard
发布解锁消息”的半启动状态。默认 `enable_patrol:=false`，普通单点导航和
手动驾驶继续使用原 mux 配置。启用巡航联锁的会话只允许巡航执行器发起的
Nav2 目标；桌面端会禁用单点导航按钮，外部单点目标会被 guard 锁存并取消。

同一时刻只能有一套 `robot_patrol` / `patrol_guard`。如果已按上面的第二个
命令单独启动巡航服务，不要再让底盘 launch 启用 `enable_patrol:=true`；
桌面端会检测这两个节点和 `/patrol/stop_lock`、`/patrol/executor_heartbeat`
发布者并拒绝重复启动会话。

默认数据库在 `~/.local/share/robot_patrol/patrol.sqlite3`，可用参数覆盖：

```bash
ros2 run robot_patrol patrol_node --ros-args \
  -p database_path:=/tmp/robot_patrol/patrol.sqlite3
```

## 安全联锁与失联停车

- `patrol_guard` 独立发布 `/patrol/guard_ready` 和 `/patrol/stop_lock`。
- guard 通过 `/diagnostics` 校验现有 `twist_mux` 确实加载了
  `lock locks.patrol_guard -> /patrol/stop_lock`。已有导航会话若使用默认 mux
  配置，guard 会保持未就绪，不能通过 UI 启动巡航。
- `/patrol/stop_lock` 在现有 `twist_mux` 中配置为 priority 255、timeout 0.5s；
  guard 启动前、guard 死亡或停止发布超过 0.5s 时锁自动生效，阻断导航与手动
  速度输入，底盘桥随后因命令超时输出零速。
- 执行器心跳携带当前真实 Nav2 `NavigateToPose` 子目标 UUID。guard 只允许
  action status 中与该 UUID 匹配的非终止目标；进入 `PAUSED`、dwell 或其他
  无活动子目标的状态时 UUID 必须为空。任何额外或迟到目标都会被锁存为
  `UNAUTHORIZED_NAV_GOAL` 并持续 cancel-all。
- Nav2 取消/暂停使用独立的 `cancel_confirm_timeout_sec`（默认 2s）。取消被
  接受但子目标未终止时，执行器在心跳中上报 `stop_unconfirmed`，guard 立即
  锁车并锁存故障，不再等待 300s 的单点超时。
- 执行器通过 `/patrol/executor_heartbeat` 提供实例、run_id、owner 和状态；
  操作端通过 `/patrol/operator_heartbeat` 提供应用级会话心跳。活动任务期间
  任一心跳失联都会锁存联锁，不会自动恢复。
- 联锁复位服务：`/patrol/reset_interlock`。复位前 guard 会直接向 Nav2
  发送 cancel-all，并通过 action status 确认不存在 ACCEPTED/EXECUTING/
  CANCELING 目标；新执行器上报 IDLE 不能替代该核对。只有旧目标已终止、
  里程计新鲜且车辆停稳时才允许显式复位。
- guard 启动后不会因为“尚未收到 Nav2 状态”而按零目标放行。启动线程会在
  `startup_nav_goal_quiet_sec`（默认 2s）内持续探测 cancel-all 和 action
  status；只有整个安静窗口内都没有活动目标，才允许完成启动宽限校验并
  解锁。安静窗口结束后，guard 仍持续监视 action status：执行器心跳未活动
  时出现 Nav2 目标，会先立即锁 mux；若
  `unexpected_nav_goal_grace_sec`（默认 0.75s）内没有匹配的巡航执行器
  所有权，则锁存 `UNAUTHORIZED_NAV_GOAL` 并持续 cancel-all。该路径用于兜住
  任意时刻才被 Nav2 接受的旧 pending 请求。
- 急停、guard 失联和手动速度输入会统一中断巡航状态机；解除急停或操作端
  重连都不会自动继续任务。

## 基础 CLI

```bash
ros2 run robot_patrol patrol_cli register-map /绝对路径/map.yaml --name 场地 --backend amcl
ros2 run robot_patrol patrol_cli set-active-map <map_id> --revision 1
ros2 run robot_patrol patrol_cli list-points --map-id <map_id>
ros2 run robot_patrol patrol_cli add-point <map_id> A 1.0 2.0 0.0
ros2 run robot_patrol patrol_cli create-route <map_id> 路线A --steps <点A_id>:0,<点B_id>:5
ros2 run robot_patrol patrol_cli run-route <路线_id> <map_id> --loops 1
```

## 测试

```bash
cd /home/jkbot/Remote_ctrol/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select robot_patrol --event-handlers console_direct+
```

隔离 ROS Action 集成测试需显式开启：

```bash
ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1 ROBOT_PATROL_INTEGRATION=1 \
  python3 -m pytest -q src/robot_patrol/test/test_patrol_integration.py -s
```

测试使用临时地图和假 Nav2 Action，不启动实车驱动或发送真实运动目标。

## 当前限制

- 独立 guard、执行器心跳、操作端心跳和 twist_mux 死锁联动已实现；仍必须按上面的启动参数同时启用，否则执行器会因 guard 未就绪拒绝启动。
- 点位/路线 JSON 导入导出、CLAIM 租约、RViz Marker 预览尚未完成。
- 暂停通过取消当前 Nav2 goal 后重发同一点实现，最终实车行为需要现场验证。
- 本轮只在隔离 ROS 域中验证 guard、心跳、mux 锁语义和停止路径，未启动实车底盘或发送真实运动目标。
