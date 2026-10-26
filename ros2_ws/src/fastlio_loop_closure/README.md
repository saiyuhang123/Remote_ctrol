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
→ UI 保存独立地图包，分别标记回环通过、单程未验证或数据/优化异常
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

服务：

```text
/loop_closure/save_map        std_srvs/Trigger
/map_save                     std_srvs/Trigger
/loop_closure/save_bundle     std_srvs/Trigger（UI 使用，保存与回环验收分开）
```

前端 FAST-LIO 自带的原始保存服务在建图模式被重映射为
`/fastlio/raw_map_save`，原始地图写到 `<map>_fastlio_raw.pcd`；
它不做任何几何检查，**不能当作最终导航地图**。

## 保存行为

### UI：保存独立地图包

UI 调用 `/loop_closure/save_bundle`。关键帧足够且文件能正常写入时，单程、
短路线和没有合格回环都能保存 PCD，UI 随后给同一个 PCD 生成 PGM/YAML。
每次写入独立的 `session/exports/<时间>/map`，重复保存保留早先快照。
服务的 `success=true` 表示数据已保存，不等于回环或实车定位验收通过。

返回 `message` 为 `schema=fastlio.saved_map.v1` 的 JSON，包含实际文件前缀、
原请求前缀、状态、原因、下一步操作和是否允许载入进行定位核对。UI 校验
PCD SHA256 和报告一致后才导出配套切片，并显示：

- `loop_checked`：实测回环和留出检查通过，导出修正地图，可载入核对定位。
- `no_loop`：回环约束不足，导出前端单程地图，明确标记未完成回环验证，可载入核对。
- `geometry_rejected`：回环优化未通过，保存前端地图与拒绝报告；不导出失败优化位姿，不开放直接载入。
- `sensor_error`：传感器/时间戳/匹配处理异常，尽可能归档已有有效关键帧；显示处理办法，不开放直接载入。

没有足够关键帧、PCD 为空/非有限或实际文件写入失败仍返回失败，并提示目录和处理办法。
切片失败保留本次 PCD 和原始会话，可再次点击保存重试。保存和切片不会替换
原导航图，也不会更新网页当前导航图；“载入本次地图”仅选择这套 PCD/YAML，
不会自动启动导航或发送行驶目标。

### 兼容接口：严格导出

`/loop_closure/save_map`（或 `/map_save`）触发：

1. 完成所有待处理重叠匹配，汇总合格约束（至少 6 条，每 3 条取 1 条入图，
   其余留出验证）；
2. 检查前端倾角与 IMU 一致性（p95 ≤ 3°）、点云/里程计时间匹配缺失率等；
3. xyz+yaw 最小二乘优化，检查优化变形（普通位移 <2 m，经额外验证的中等漂移 <3.5 m，旋转 <1°）、solver 收敛、
   留出重叠残差（平移 <0.4 m、yaw <0.6°）；
4. 全部通过才原子写入 `output_map_path`（0.10 m 体素，XYZ+intensity），
   并生成 `<map>.validation.json`（含点数与 SHA256）。

失败时返回 `success=false`，最终文件名**不会被写入**；保留
`<map>_UNVERIFIED.pcd` 候选图与完整会话目录供诊断。这两个接口保留旧的
严格行为，供既有 CLI 和验收脚本兼容；新 UI 不使用它们。

普通重叠匹配质量门限：非地面 fitness ≥0.85
（0.4 m 对应距离）、RMSE ≤0.20 m；单条修正 XY <2 m、Z <0.4 m、
roll/pitch 合成 <1.5°、yaw <1°。

2026-10-26 实地数据补充了“部分重叠 / 中等漂移”路径：fitness ≥0.65、
RMSE ≤0.22 m，允许 XY <3 m、Z <2 m，但必须同时满足至少 500 个非地面内点、
至少两个初值收敛到相同结果、地面高度一致性 <0.10 m、正反向 ICP 平移闭合差
<0.15 m / 旋转闭合差 <0.5°。部分重叠还要求源观测覆盖至少 6 m，避免把
同一位置的多帧当作充分空间支持。图优化仍保留 solver、姿态变形和留出残差检查。

地面平面只用于生成高度初值和辅助核对，最终配准及 fitness 使用非地面结构。
尝试多个历史子地图及多个位置初值，处理最近子地图视角不足和局部最优。
这些检查有相关性，不构成外部真值验证。保存时会补查尚未尝试的最后一帧，
重复保存不重复加边；这不生成首尾相等约束。

历史时间间隔默认改为 `min_time_diff_sec=60`，同时保留至少 30 帧的索引间隔和
所有匹配质量检查。这只是筛选回环候选的条件，**不是保存地图的最短时长**。
无回环时保存单程图，不通过放松配准质量门限伪造“回环通过”。

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
  exports/<时间>/        UI 每次保存的独立地图包：PCD、报告、poses.npy、使用说明
                         以及 UI 随后导出的 PGM/YAML
  last_save.json         最近一次 UI 保存结果与实际路径
```

`/loop_closure/status` JSON 字段：keyframes、qualified_overlaps、
processed_keyframes、gravity_reference_ready、unmatched_clouds、error、
session、attempted_overlaps。`error` 非空（如前端位姿 NaN、点云时间回退）时保存会被拒绝。

## 主要参数（bringup 建图模式默认值）

```text
output_map_path        最终 PCD 路径（必填，来自 pcd_map）
keyframe_distance      0.8 m        keyframe_angle_deg   10°
keyframe_max_gap_sec   2 s          静止排除 0.12 m / 2°
search_radius          12 m         min_time_diff_sec    60 s
min_index_diff         30           submap_half_window   3
overlap_spacing_m      1 m          map_voxel            0.10 m
imu_topic              /imu/data    lio_frame            lio_odom
```

## UI 集成

`chassis_nav2` 建图模式同时启动本节点与前端。桌面端“保存地图”固定调用
`/loop_closure/save_bundle`，服务不存在时提示重新启动建图流程，不回退到原始
保存服务；保存服务超时 ≥180 s。UI 提供“打开保存目录”和“载入本次地图”，
后者须先停止建图流程，且只对回环通过或单程未验证的完整配套地图开放。

## 重要边界

- 重叠 ICP 依赖真实重复路段与足够的非地面结构；长直空旷路线无法出
  已验证图属预期；
- 终点不要求回到起点，没有 endpoint=origin 约束；
- 报告中的几何检查是内部一致性验证（共享结构、留出残差），不是外部
  真值精度认证；上车后仍需做实际定位验收；
- 保存进行中请等待，完成后再次保存不会重复添加同一条回环约束；
- 建图启动时若目标路径已有旧图，旧 PCD 与旧 `.validation.json` 会以同
  一时间戳成对复制备份，原路径继续保留旧图；只有新图通过检查才替换，
  新建图失败不会移走现有可用地图。
