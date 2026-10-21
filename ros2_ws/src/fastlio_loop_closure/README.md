# fastlio_loop_closure

FAST-LIO2 的实时 6DOF 关键帧回环后端。

它并行于 `fastlio_mapping` 运行，不阻塞前端 LiDAR/IMU 回调：

```text
/cloud_registered_body + /Odometry
→ 关键帧提取
→ 历史邻近候选
→ 局部子地图 ICP（包含 Z/roll/pitch）
→ Open3D 6DOF PoseGraph 优化
→ /loop_closure/corrected_map + /loop_closure/corrected_path
```

## 节点接口

订阅：

```text
/Odometry               nav_msgs/Odometry
/cloud_registered_body  sensor_msgs/PointCloud2
```

发布：

```text
/loop_closure/corrected_map   sensor_msgs/PointCloud2
/loop_closure/corrected_path  nav_msgs/Path
/loop_closure/status          std_msgs/String (JSON)
```

服务：

```text
/loop_closure/save_map        std_srvs/Trigger
```

## 保存行为

`/loop_closure/save_map` 会把当前关键帧轨迹按最新 6DOF 位姿重建为：

```text
output_map_path
```

如果在线过程中还没有检测到回环，保存前会尝试一次“最后关键帧 → 第一帧”的
6DOF 约束，适合记录终点稍微早于起点的情况；如果 ICP 质量不足，则保存原始
关键帧位姿的地图，并在返回消息里保留实际状态。

## UI 集成

`chassis_nav2` 的建图模式会同时启动本节点。FAST-LIO 自己的 PCD 使用：

```text
<map>.pcd -> <map>_fastlio_raw.pcd
```

本节点输出的修正地图使用：

```text
<map>.pcd
```

桌面端“保存地图”优先调用 `/loop_closure/save_map`；没有该服务时回退到
FAST-LIO 原始 `/map_save`。

## 重要边界

- 回环 ICP 依赖足够的历史重叠点云和良好的 6DOF 初值；
- 长直走廊、重复结构或终点未真正回到起点时，需要检查日志；
- 当前实现把前端实时里程计和修正地图分开，修正结果主要用于地图输出，
  不反写 FAST-LIO 的实时导航状态；
- 若必须严格闭环，记录时应完整回到起点，最后一段不要提前停止。
