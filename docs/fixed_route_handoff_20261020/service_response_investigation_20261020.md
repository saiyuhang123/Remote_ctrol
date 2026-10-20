# `/patrol/road_command` 服务响应投递调查

调查日期：2026-10-20  
调查截止：2026-10-21 00:51:44 Asia/Shanghai  
代码基线：`e69f8590ca7949a513e4c585a362f5417a9f09ff`  
调查范围：完整隔离回归中 `test_real_recording_service_round_trip_and_command_retry` 的 `CREATE` 超时

## 1. 结论摘要

本次失败**不是控制器、Nav2 执行或服务端业务逻辑错误**，而是一个高度局部的测试环境服务响应投递问题：

- 请求能够到达 `robot_patrol` 的 `/patrol/road_command` 服务；
- 服务回调执行成功并返回成功响应；
- 测试进程内的 rclpy 客户端收不到响应，future 一直 pending；
- 同一进程内新建 RoadCommand client、甚至新建独立 rclpy Context，仍然收不到 RoadCommand 响应；
- 同一时刻，`/patrol/list_maps`、进程内新建的 `std_srvs/srv/Trigger` 服务、`/scan` 话题都正常；
- 外部独立进程调用 `/patrol/road_command` 正常；
- 只要另起一个外部进程持续调用 `/patrol/road_command STATUS`，完整长序列可以两次稳定达到 `24 passed`；
- 禁用 Fast DDS SHM、仅使用 UDPv4 后，问题仍然复现；
- 空跑 570 秒并周期调用 RoadCommand 全部正常，说明不是单纯时间到了就失效。

因此，高置信度判断是：

> 长序列路线测试活动之后，测试进程内的 rclpy 客户端到 `/patrol/road_command` 的响应侧匹配/投递状态失效；外部独立 RoadCommand 客户端活动可以避免或刷新该状态。

精确到 Fast DDS/rmw 内部的具体触发条件尚未完全定位。这个问题没有证据表明会影响真车 C++ 固定路线客户端的正常运动控制，但测试收口必须处理，不能把外部 keep-alive 当作正式修复。

## 2. 环境

- ROS 2 Humble
- `rmw_fastrtps_cpp` 6.2.10
- Fast DDS 2.6.12
- 测试域：`ROS_DOMAIN_ID=209`、`ROS_LOCALHOST_ONLY=1`
- 使用真实 Nav2、真实 `robot_patrol`、guard、twist_mux 和无硬件阿克曼模拟器
- 所有速度只进入 `/inert_sim/cmd_vel`，未启动串口/CAN/底盘桥

## 3. 原始失败表现

无外部 RoadCommand keep-alive 时，完整长序列结果：

```text
21 passed, 1 failed
```

失败点：

```text
test_real_recording_service_round_trip_and_command_retry
CREATE -> wait future.done 15s timeout
```

服务端临时探针记录到：

```text
[road-service-enter] op=CREATE ...
[road-service-exit] op=CREATE success=True
```

因此失败发生在请求之后：服务端已经执行完成，但客户端没有收到响应。

## 4. 关键实验与结果

| 实验 | 结果 | 结论 |
|---|---|---|
| 单独运行 RoadCommand 服务往返测试 | 通过，约 25.6s | 用例和业务逻辑本身可工作 |
| 无外部客户端，完整长序列 | 多次在第 22 项失败 | 问题可稳定复现 |
| 服务端入口/出口探针 | 收到请求且 `success=True` | 不是服务端回调未执行 |
| 超时线程栈 | executor worker 空闲，`active_entity=null` | 不是 callback group 或 executor 卡死 |
| `client._pending_requests` | `[1]`，future pending | 请求已发出但未完成 |
| 客户端底层 `take_response()` | `(None, None)` | 响应未进入客户端队列 |
| 新建同 Context 的 Node + executor 调 RoadCommand | 响应超时 | 不是单个 Node/executor |
| 新建独立 rclpy Context 调 RoadCommand | 响应超时 | 不是单个 participant |
| 同进程 `/scan` 订阅 | 正常收到 | 普通 DDS pub/sub 未坏 |
| 同 `robot_patrol` 的 `/patrol/list_maps` | 正常返回 | 不是 robot_patrol 所有服务 |
| 进程内新建 `Trigger` 服务并调用 | 正常返回 | 不是所有 service 都坏 |
| 空跑 570 秒并周期调 RoadCommand | 12/12 成功 | 不是纯时间因素 |
| 跳过 RoadCommand 服务测试，跑其余 23 项 | `23 passed` | action 和其余路线用例仍正常 |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 完整长序列 | 仍在第 22 项失败 | 不是只由 SHM 传输导致 |
| 外部独立进程每秒调 RoadCommand 的完整长序列 | 两次 `24 passed` | 外部活跃 RoadCommand 客户端可规避该状态 |
| 外部进程只保持连接、不调用 | 未跑到目标项，第 20 项出现路线未启动 | 该对照无效，不能下结论 |

## 5. 关键证据

### 5.1 失败时客户端状态

```json
{
  "op": "CREATE",
  "ready": true,
  "pending": [1],
  "executor_event": false,
  "active_entity": null,
  "ticks_before": 24051,
  "ticks_after": 24077
}
```

解释：

- `ready=true`：服务发现状态仍然显示可用；
- `pending=[1]`：请求 future 仍然 pending；
- `executor_event=false`：客户端实体没有被 executor 取出；
- `active_entity=null`：callback group 没有卡住的执行体；
- 计时器正常推进：sim 节点和 executor 整体仍在运行。

### 5.2 服务范围探针

同一失败时刻：

```json
{
  "same_context_scan": 1,
  "new_node_ready": true,
  "new_node_error": "timeout",
  "list_maps_ready": true,
  "list_maps_response": "...success=True...",
  "local_ready": true,
  "local_response": "std_srvs.srv.Trigger_Response(success=True, message='local-ok')",
  "new_context_scan": 1,
  "new_context_ready": true,
  "new_context_error": "timeout"
}
```

解释：

- `/scan` pub/sub 正常；
- `/patrol/list_maps` 正常；
- 进程内新建的 Trigger 服务正常；
- RoadCommand 在默认 Context 和新 Context 下都超时；
- 因此问题是 RoadCommand + 测试进程 rclpy 客户端这一特定组合。

### 5.3 外部活跃客户端对照

外部独立进程每秒调用一次 `/patrol/road_command STATUS` 时，完整测试结果两次为：

```text
24 passed, 1 deselected in 529.24s
24 passed, 1 deselected in 528.89s
```

外部客户端摘要：

- 第一次：835 次调用，815 次成功；最早失败发生在约 532.8s，接近测试服务端退出时间；
- 第二次：539 次调用，525 次成功；最早失败发生在约 532.3s，接近测试服务端退出时间；
- 测试进程的 24 项已分别在同两次运行中全部通过。

这说明外部活跃客户端使测试进程内的 RoadCommand 响应恢复，或者避免了失效状态。

### 5.4 UDP-only 对照

设置：

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

完整长序列仍在第 22 项失败：

```text
1 failed, 21 passed, 1 deselected
```

因此不能简单归因于 SHM 传输。

### 5.5 无服务测试长序列

跳过 RoadCommand 服务往返测试后：

```text
23 passed, 2 deselected in 515.78s
```

说明：

- 长路线本身、停车、绕行、现场异常尾段安全停止、operator loss/guard 等仍能通过；
- action 路径在同样长序列后没有整体失效；
- 失效与 RoadCommand 服务响应侧强相关。

### 5.6 最终清理后的单测复验

恢复所有临时探针后，单独运行：

```text
recording_service_round_trip:        1 passed, 23 deselected, 25.58s
inconsistent_field_tail:             1 passed, 23 deselected, 29.13s
operator_loss_while_blocked:         1 passed, 23 deselected, 20.87s
```

## 6. 根因判断与证据边界

### 已确认

- 不是控制器方向/跟踪/容差算法；
- 不是服务端 `RoadComponent.command` 未执行；
- 不是 executor worker 死锁；
- 不是 callback group 互斥卡死；
- 不是测试客户端未发现服务；
- 不是普通 DDS pub/sub 整体失效；
- 不是 `robot_patrol` 所有服务都失效；
- 不是纯空闲时间导致；
- 不是 SHM 单独导致；
- 不是单个 Node/executor/Context 可隔离的问题。

### 高置信度推断

问题位于测试进程 rclpy 客户端到 `/patrol/road_command` 的**响应侧匹配/投递状态**。长序列路线活动后该状态失效，外部活跃 RoadCommand 客户端可以刷新或规避。

可能的底层方向：

- Fast DDS response reader/writer 匹配状态或历史缓存；
- RoadCommand 响应端点大量建立/销毁后的端点复用；
- rclpy/rmw 服务响应 reader 匹配状态问题；
- 长序列测试中 C++ BT RoadCommand clients 与测试进程 Python clients 共用服务时的端点状态交互。

### 尚未确认

- Fast DDS/rmw 内部具体触发点；
- 为什么 `/patrol/list_maps` 不触发；
- 为什么外部活跃客户端可以规避；
- 生产环境长期运行是否会遇到同类状态。

本报告不把“外部 keep-alive 可绕过”当作根因已修复。

## 7. 对真车和自由导航的影响判断

### 固定路线 C++ 执行链路

真实固定路线执行由 C++ `FollowRecordedLeg` 通过 `/patrol/road_command` 调 `LOAD_LEG` / `CHECK_LEG`。本次 23 项无服务测试长序列通过，说明：

- C++ RoadCommand 客户端在路线活动中仍可正常收到响应；
- 控制器和新 tracker 的回归结果未被本次问题推翻；
- 如果生产 C++ 客户端也出现响应丢失，代码会按现有逻辑在约 2s 后报“路径票据服务超时”，撤销运动许可并安全停车，而不是继续错误运动。

### 桌面/UI/外部进程

- 外部独立进程 RoadCommand 调用正常；
- 桌面 UI 使用独立 rclcpp 进程；
- 网页网关当前没有使用 `/patrol/road_command`；
- 因本问题复现限于长序列 pytest 进程，对真车运动控制的直接影响证据不足。

### 自由导航

- 自由导航不依赖 RoadCommand 路径票据；
- Nav2 规划、控制、costmap、guard、twist_mux 不受控制器修复影响；
- 若客户端服务/action 响应真实失效，自由导航的启动、取消和状态反馈仍可能受影响；
- 因此自由导航可以作为兜底，但必须先做短距离起步、到点、取消验证。

### 原路线

本调查不改变原现场路线数据问题：

- 原短倒车/横向位移几何异常仍存在；
- 原现场尾段仍预期安全拒绝并停车；
- 用户已要求保留原始轨迹，不能自动删块或平滑改写；
- 原路线和实车验收仍未完成。

## 8. 建议

### 8.1 测试收口（优先）

不要把“外部 keep-alive 下 24 passed”当作正式测试修复。建议把 RoadCommand 服务往返测试和长路线/安全回归拆成两个独立进程或独立 pytest 模块/runtime：

```text
进程 A：长路线、安全、guard、action 回归     -> 23 passed
进程 B：录制服务 RoadCommand 往返           -> 1 passed
```

这样既保留全部真实测试，又避免同一 rclpy 进程长序列后的 RoadCommand 响应状态问题。

也可以把服务测试放到一个独立模块，使其获得新的 runtime fixture/Context。是否足够需要再次完整验证。

### 8.2 生产侧

当前不建议为了这个测试问题修改控制器、容差、guard 或运动许可逻辑。建议先做：

1. 独立多进程长跑：`robot_patrol` + Nav2/C++ RoadCommand 客户端 + UI 客户端；
2. 周期调用 `LOAD_LEG/CHECK_LEG/STATUS/LIST`，确认响应可靠性；
3. 记录 RoadCommand 服务超时和 client 重建事件；
4. 如果生产侧也复现，再增加同一 `command_id` 的幂等重试、服务 client 重建和明确超时停车。

### 8.3 真车验收

仍按分阶段执行：

1. 急停、guard、twist_mux 检查；
2. 短距离自由导航和取消；
3. 短录制路线前进/停车/短倒车/换向；
4. 入场、点位继续、暂停/取消；
5. 障碍停车和绕行回线；
6. 最后才尝试原路线，且预期异常尾段安全停车。

## 9. 清理与当前工作区

- 临时服务端 `component.py` 探针已恢复；
- 临时测试诊断代码已从 Git 文件恢复；
- 捕获的临时测试进程已终止；
- 没有修改生产控制器源码；
- 没有修改现场 SQLite、地图或原始轨迹；
- 本报告和证据目录是本次调查新增文件。

## 10. 证据文件

```text
docs/fixed_route_handoff_20261020/service_response_evidence/
  service_scope_failure.log
  udp_only_failure.log
  no_service_23_passed.log
  idle_soak.json
  idle_soak_test.log
  external_soak_pass_1.log
  external_soak_pass_2.log
  external_client_summary.json
  isolated_service_test.log
  isolated_field_tail_test.log
  isolated_operator_loss_test.log
```

## 11. 最终状态

- 控制器软件修复：保留在 `e69f859`，本次调查未推翻其单测和路线回归结果。
- 完整长序列测试：无外部 keep-alive 时仍在第 22 项失败；外部活跃 RoadCommand 客户端下两次 24 passed。
- 单个服务测试：可单独通过。
- 根因：已缩小到测试进程 rclpy RoadCommand 响应投递/匹配状态；底层 Fast DDS/rmw 精确触发点未完全定位。
- 真车固定路线：仍未实车验收，不能宣布全部完成。
