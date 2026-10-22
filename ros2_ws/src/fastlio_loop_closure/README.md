# fastlio_loop_closure

FAST-LIO2 日常建图的保存时优化后端（重力保持 + 实测重叠约束）。

它与 `fastlio_mapping` 前端并行运行，不阻塞前端 LiDAR/IMU 回调。**工作方式是
“实时前端显示 + 保存时自动优化”：运行期间不做全 6DoF 位姿图反写，前端
EKF 状态不受本节点影响；修正只在点击保存后计算并写入磁盘地图。**

```text
/Odometry + /cloud_registered_body + /imu/data
→ 按 stamp 精确配对，提取关键帧（0.8 m / 10° / 最长 2 s，静止帧排除）
→ 后台匹配历史邻近非地面子地图（point-to-point ICP → Huber point-to-plane ICP）
→ 保存时：xyz+yaw 图优化（roll/pitch 沿用前端结果并固定，第一帧固定，
   无终点=原点约束），留出重叠残差验证
→ 检查通过才原子写入最终 PCD 与 <map>.validation.json
```

地图坐标：以首个有效 IMU 观测为世界重力参考，导出时整体旋转到重力竖直
的局部地图坐标，原点在初始 IMU 位置；报告中保存 `map_from_lio_rotation`。
在线显示的 `/loop_closure/corrected_map`/`corrected_path` 仍在 `lio_odom`
坐标系，磁盘 PCD 是重力竖直地图。

## 节点接口

订阅：

```text
/Odometry               nav_msgs/Odometry     （可靠，深度 200）
/cloud_registered_body  sensor_msgs/PointCloud2（best effort，深度 100）
/imu/data               sensor_msgs/Imu        （参数 imu_topic 可改）
```

发布（均为 latched / TRANSIENT_LOCAL）：

```text
/loop_closure/corrected_map   sensor_msgs/PointCloud2（保存成功后发布）
/loop_closure/corrected_path  nav_msgs/Path          （保存成功后发布）
/loop_closure/status          std_msgs/String (JSON)，每 5 s
```

服务（两个名字等价，均执行完整检查与优化）：

```text
/loop_closure/save_map        std_srvs/Trigger
/map_save                     std_srvs/Trigger
```

前端 FAST-LIO 自带的原始保存服务在建图模式被重映射为
`/fastlio/raw_map_save`，原始地图写到 `<map>_fastlio_raw.pcd`；
它不做任何几何检查，**不能当作最终导航地图**。

## 保存行为

`/loop_closure/save_map`（或 `/map_save`）触发：

1. 完成所有待处理重叠匹配，汇总合格约束（至少 6 条，每 3 条取 1 条入图，
   其余留出验证）；
2. 检查前端倾角与 IMU 一致性（p95 ≤ 3°）、点云/里程计时间匹配缺失率等；
3. xyz+yaw 最小二乘优化，检查优化变形（位移 <2 m、旋转 <1°）、solver 收敛、
   留出重叠残差（平移 <0.4 m、yaw <0.6°）；
4. 全部通过才原子写入 `output_map_path`（0.10 m 体素，XYZ+intensity），
   并生成 `<map>.validation.json`（含点数与 SHA256）。

失败时返回 `success=false`，最终文件名**不会被写入**；保留
`<map>_UNVERIFIED.pcd` 候选图与完整会话目录供诊断。桌面端在失败时不会
继续导出 PGM/YAML 栅格。

重叠匹配质量门限（当前按长园区路线标定）：非地面 fitness ≥0.85
（0.4 m 对应距离）、RMSE ≤0.20 m；单条修正 XY <2 m、Z <0.4 m、
roll/pitch 合成 <1.5°、yaw <1°。

**注意**：`min_time_diff_sec=300` 要求重复路段至少间隔 5 分钟。短路线或
没有重复经过有建筑/立柱路段时，会因合格匹配不足而无法输出已检查的最终图
——这是设计行为，候选图与会话保留，不要通过放松门限强行出图。

## 会话与诊断

运行期间所有关键帧与原始里程计写入：

```text
<map>_session_<日期>_<PID>/
  keyframes/*.npz        0.15 m 体素 body 点云（XYZ+intensity）与当时位姿
  keyframes.jsonl        关键帧清单
  odometry.jsonl         全部原始 /Odometry
  gravity_reference.json 初始重力参考与 map_from_lio_rotation
  session.json           工作流与参数声明
  validation.json        保存后的完整报告
  poses.npy              保存后的优化位姿
```

`/loop_closure/status` JSON 字段：keyframes、qualified_overlaps、
processed_keyframes、gravity_reference_ready、unmatched_clouds、error、
session。`error` 非空（如前端位姿 NaN、点云时间回退）时保存会被拒绝。

## 主要参数（bringup 建图模式默认值）

```text
output_map_path        最终 PCD 路径（必填，来自 pcd_map）
keyframe_distance      0.8 m        keyframe_angle_deg   10°
keyframe_max_gap_sec   2 s          静止排除 0.12 m / 2°
search_radius          5 m          min_time_diff_sec    300 s
min_index_diff         30           submap_half_window   3
overlap_spacing_m      2 m          map_voxel            0.10 m
imu_topic              /imu/data    lio_frame            lio_odom
```

## UI 集成

`chassis_nav2` 建图模式同时启动本节点与前端。桌面端“保存地图”固定调用
`/loop_closure/save_map`，服务不存在时提示未就绪，**不再回退到原始
`/map_save` 原图**；保存服务超时 ≥180 s，成功后才继续 PCD→PGM/YAML 与
网页地图导出。

## 重要边界

- 重叠 ICP 依赖真实重复路段与足够的非地面结构；长直空旷路线无法出
  已验证图属预期；
- 终点不要求回到起点，没有 endpoint=origin 约束；
- 报告中的几何检查是内部一致性验证（共享结构、留出残差），不是外部
  真值精度认证；上车后仍需做实际定位验收；
- 重复调用保存会被拒绝（`地图正在优化保存`），等待当前任务完成即可；
- 建图启动时若目标路径已有旧图，旧 PCD 与旧 `.validation.json` 会以同
  一时间戳成对备份，避免遗留报告被误读。
