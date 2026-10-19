# Recorded route Nav2 plugins

ROS2 Humble 插件包；由 `enable_recorded_routes:=true` 按需加载，默认导航不依赖它。

- `recorded_route_nav2::RecordedController`：保留不可变运动段和单调进度，将局部窗口交给已安装的 RPP；限制轨迹偏离、朝向及运动方向，检查 BT 运动授权是否新鲜。
- `recorded_route_nav2::RecordedGoalChecker`：完整段剩余弧长和实际位置必须同时满足容差，防止闭环刚启动就成功。业务目标 0.20m，换向/绕行重入 0.08m。
- `recorded_route_bt` 中的 `FollowRecordedLeg`：从 robot_patrol 加载当前真实 Nav2 goal UUID 授权的道路票据，通过真实 FollowPath 跟踪；确认停稳后换向，到障碍前停车，调用独立 RecordedDetour Smac 规划器生成候选（直接及左右侧引导）并严格校验；直道还提供经过同样校验的精确阿克曼圆弧变道候选，回归后继续原参考段。

插件不直接发布 `/cmd_vel`。速度经过现有 controller_server → velocity_smoother → twist_mux → 底盘桥；guard 继续校验外层 NavigateToPose UUID。每次尝试生成私有、原子写入的 BT XML，终止后删除；外层和内部 FollowPath 都确认停止后才释放执行权。

`/patrol/recorded_motion_permit` 带当前计划时间戳，有效期 0.6s。BT/票据服务失联会停止内部跟踪；执行器另行检查 FollowPath 取消结果，内部停车未确认也进入既有 guard 联锁。此机制不取代独立 guard。

局部/全局代价地图必须新鲜；当前跟踪前方扫掠在局部和全局地图同时检查。完整候选在全局动态障碍地图检查，并在局部地图覆盖范围内额外核对局部障碍；未覆盖的后续候选不会被当作局部已观测，行驶接近后仍需通过新鲜局部检查。全局未知栅格、车身扫掠碰撞和允许区域外一律拒绝。

默认绕行边界：最大偏离 0.8m、参考弧长前移 4m、候选长度 10m、等待 120s。候选不得跳过本段路口、换向或业务停靠点；本期只允许前进绕行，未录制倒车候选拒绝。参数在 patrol_node 中名为 `recorded_detour_max_offset`、`recorded_detour_max_skip`、`recorded_detour_max_length`、`recorded_obstacle_wait_timeout`。

最小转弯半径及 footprint 由导航 launch 从 hardware.yaml 传给 patrol_node，并绑定到发布路网及运行票据。尺寸/半径变化后需要重新发布路网；运行中变更会停止。重入处的有向边界检查防止候选越过路口/业务点再绕回来。BT 目前使用本项目约定的 `map` / `base_link` / `/odom`，其他命名需要同步适配后再使用。

构建与几何测试：

```bash
cd /home/jkbot/Remote_ctrol/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select recorded_route_nav2
colcon test --packages-select recorded_route_nav2 --event-handlers console_direct+
colcon test-result --verbose
```

真实 Nav2/guard/mux 模拟测试见 robot_patrol/recorded_routes/README.md。离线验证不等同于已经完成实车转弯、掉头和停靠验收。
