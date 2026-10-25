# 巡检点云台多角度拍照方案（PTZ Shot Sequence）

> 第一版已按“最多两个低倍视角、记录原始 pan/yaw、到位后仅拍照”的范围实现，测试步骤见 [第一版使用说明](ptz_shot_sequence_v1.md)。下文保留原始设想；其中世界角度公式、精度和时间估算不作为本版验收依据。

> 目标：车停稳后，云台按一组"世界方位角"逐个转动并拍照，替代固定预置位。
> 好处：不再要求车头朝向精确——车停在 10 cm 内、朝向随便，云台按当前 yaw 补偿，
> 每次都拍向同一个世界方向。

---

## 1. 已具备的基础（2026-10-25 验证）

| 能力 | 状态 | 实测 |
|---|---|---|
| 485 绝对定位读写 | ✅ | pan 0~360°、tilt −90~25°、zoom 1~23×，精度 0.01° |
| 连续运动点动 | ✅ | 速度 40 时约 13°/s，63 满速约 25~30°/s |
| 变倍 | ⚠️ 慢 | 1×→20× 约 10 s，绝对设置可靠，读回要等停稳 |
| 拍照 | ✅ | `/hk_camera/capture`（quality + save_path） |
| 设备限制 | — | 运动过程中查询会失败（设备忙），只在静止时读 |
| 串口 | — | `/dev/ptz485`，2400 8N1，addr 1，单进程独占（flock） |

## 2. 坐标定义与补偿公式

云台 pan 是**相对车体**的角度；拍照要的是**世界方位角**。

```
pan_cmd = wrap360(azimuth_world − yaw_robot)        # 逆时针为正，需按标定定符号
```

- `azimuth_world`：镜头看向的世界方位角（0°=+x/map 东，逆时针为正）
- `yaw_robot`：到点停车时车头在地图里的朝向（`/localization` 的 yaw）
- 标定时：`azimuth_world = wrap360(pan_cal + yaw_cal)`，存进点位

**误差来源**（决定这个方案能不能成立）：

| 来源 | 量级 | 影响（目标距 3 m） |
|---|---|---|
| 定位 yaw 误差 | ±2~4° | 画面偏 10~21 cm |
| 停车位置误差 10 cm | `atan(0.1/3)` ≈ 2° | 10 cm |
| 云台角度精度 | 0.01° | 忽略 |

→ 3 m 处的综合偏差约 **20~30 cm**，对"设备在画面里"够用；对固定 ROI 的 OCR 需要再标一次偏移量。

## 3. 数据模型

点位（`patrol.yaml` / 数据库 `Point`）增加一个拍摄序列：

```yaml
points:
  - point_id: p_switch_room
    x: -6.92
    y: -18.85
    yaw: 2.356            # 到点车头朝向（只用于导航，不再要求精确）
    inspection_enabled: true
    shots:                # 为空则退回旧的 ptz_preset 流程
      - name: wide
        azimuth: 45.0     # 世界方位角（°）
        tilt: -8.0
        zoom: 2.0
        settle_ms: 1500
        analyze: false
      - name: meter
        azimuth: 52.0
        tilt: -14.0
        zoom: 12.0
        settle_ms: 2500
        analyze: true
        recognition_prompt: "读取电表读数"
      - name: side
        azimuth: 100.0
        tilt: -10.0
        zoom: 4.0
        settle_ms: 1500
```

字段说明：

| 字段 | 含义 | 备注 |
|---|---|---|
| `azimuth` | 世界方位角 | **唯一需要标定的方向量** |
| `tilt` / `zoom` | 俯仰、变倍 | 直接下发；车体姿态变化不影响 |
| `settle_ms` | 转到位后的等待 | 大角度/大变倍要给足（见 §5） |
| `analyze` | 是否跑识别 | false 只存图 |
| `recognition_prompt` | 识别提示词 | 沿用现有字段 |

## 4. 标定流程（每个 shot 一次）

```
1. 车正常导航停到点位（朝向随便）
2. 读 yaw_cal = /localization 的 yaw
3. 用 UI 的上下左右 / 输入框把云台转到正好对准目标
4. 读 pan_cal / tilt_cal / zoom_cal
5. 存: azimuth = wrap360(pan_cal + yaw_cal)，tilt=tilt_cal, zoom=zoom_cal
6. 重复 3~5 添加下一个 shot（同一台设备的多个仪表/视角）
```

标定工具建议做成 UI 按钮「记录当前角度为 shot」+「保存本点位序列」。

**符号标定（全车一次）**：车原地转 +10°，看 pan 需要变大还是变小才能保持对准 →
`pan_cmd = azimuth − s·yaw`，`s = ±1` 写进配置。

## 5. 时间预算

| 动作 | 耗时 | 依据 |
|---|---|---|
| pan 转 90° | 3.5 s（speed 40）/ 3 s（满速） | 实测 13~30°/s |
| tilt 转 30° | 2~4 s | 比 pan 慢 |
| zoom 1×→10× | 5~8 s | 实测很慢 |
| 拍照 + 存盘 | 0.3~0.8 s | `/hk_camera/capture` |
| 识别（OCR） | 0.5~2 s | 视模型 |

**策略**：`settle_ms` 之外再做**读回轮询**（每 300 ms 读一次，连续 2 次角度变化 < 0.3° 视为停稳），
最长等 10 s。原因：变倍时间不确定，固定等待要么浪费要么不够。

## 6. 执行流程

```
Nav2 到点 (SUCCEEDED)
  └─ 读 yaw_now
     └─ for shot in point.shots:
          pan_cmd = wrap360(shot.azimuth - s*yaw_now)
          ptz.set_pose(pan_cmd, shot.tilt, shot.zoom)
          wait_until_settled(timeout=10s)     # 轮询读回
          capture → saves/<task>/<point>/<shot>.jpg
          if shot.analyze: 跑识别 → 存结果
     └─ 恢复云台到最近各点的公共角度（可选）
```

**放在哪一侧实现**：推荐 **patrol 侧循环调用**（`ptz_pelco` 只提供 set/get），
理由：失败重试、存档路径、识别调用都在 patrol 侧，少一层接口。

## 7. 失败处理

| 情况 | 处理 |
|---|---|
| 串口未就绪/被占用 | 该 shot 失败，记录，继续下一个 |
| 转不到位（10 s 超时） | 读回当前角度，用当前角度拍照并标注 `degraded` |
| 拍照失败 | 重试 2 次，仍失败则记录 |
| 定位 yaw 不可用 | 回退到"车体相对角度"（用 pan_cal 直接下发）并告警 |

## 8. 集成点（改动清单）

| 文件 | 改动 |
|---|---|
| `robot_patrol/models.py` | `Point`/`RouteStep` 增加 `shots: list[Shot]` |
| `robot_patrol/store.py` / 数据库 | 序列化 shots（JSON 列或子表） |
| `robot_patrol/engine.py` | settle 阶段：有 shots → 走序列；否则原 preset 流程 |
| `robot_patrol/ptz_sequence.py`（新） | 角度换算、等待稳定、拍照、重试 |
| `desktop_ui` 相机面板 | 「记录当前角度为 shot」按钮 + 序列编辑（可后置） |
| `config/patrol.yaml` | 文档 + 示例 |

## 9. 分期与验收

| 阶段 | 内容 | 工时 | 验收 |
|---|---|---|---|
| M1 | 数据模型 + 标定工具（命令行） | 1 天 | 能对一个点标 2 个 shot 并生成 YAML |
| M2 | 执行器 `ptz_sequence.py` + 拍照 | 1~2 天 | 停车后自动转 2 个角度各拍 1 张 |
| M3 | 接入 patrol + 结果归档 | 1 天 | 完整巡检一次，照片按点位/角度命名 |
| M4 | 实车调优 | 1~2 天 | 5 次重复巡检，目标在画面中心 ±100 px 内 |

## 10. 风险

1. **定位 yaw 误差直接进画面**（±4° ≈ 3 m 处 20 cm）——想更准就要先提定位/到点一致性；
2. **变倍慢**：大变倍 shot 会明显拉长单点耗时，建议只在必要 shot 用高倍；
3. **总线独占**：拍照期间不能有别的进程读串口（已用 flock 限制）；
4. **云台限位**：某些 azimuth 可能撞支架，标定时要试转一圈确认安全范围。
