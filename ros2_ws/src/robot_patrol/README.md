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

```bash
ros2 launch robot_patrol patrol.launch.py
```

默认数据库在 `~/.local/share/robot_patrol/patrol.sqlite3`，可用参数覆盖：

```bash
ros2 run robot_patrol patrol_node --ros-args \
  -p database_path:=/tmp/robot_patrol/patrol.sqlite3
```

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

- 服务端默认 `require_guard: true`，并要求 `/patrol/guard_ready`、新鲜急停和真实 `/map` 几何匹配；但独立 `patrol_guard` 与 `twist_mux` 锁仍未实现。当前版本不能作为实车安全巡航版本。
- 点位/路线 JSON 导入导出、CLAIM 租约、RViz Marker 预览尚未完成。
- 暂停通过取消当前 Nav2 goal 后重发同一点实现，最终实车行为需要现场验证。
