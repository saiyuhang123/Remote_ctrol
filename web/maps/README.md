# 网页地图资源

网页端只读取本目录下的：

```text
active.png
meta.json
```

这两个文件不再要求人工导出。运行时行为：

- 本地桌面在“保存地图”成功后自动调用 `scripts/export_web_map.py`。
- `robot_patrol` 在设置活跃地图时再次导出，并写入 `map_id`、地图 `revision` 和 `backend`。
- 导出使用临时文件加原子替换；`meta.json` 保留完整 Nav2 `origin`，包括 `origin[2]` 朝向。
- 浏览器使用同一套二维仿射变换绘制地图、雷达、机器人位姿和路径，因此支持 `origin_yaw` 非零。
- 监控页每 2 秒检查元数据，按地图 ID、revision、图片 hash 和几何变化重载；首次打开尚无地图时会持续重试，元数据不匹配时清除旧图，恢复后自动加载。未变化时保留当前缩放和平移。
- 网页通过 `/api/map/meta` 校验 PNG SHA-256、`map_id` 和 revision。图片与 meta 不一致、地图 revision 变化或路线 revision 过期时，任务页禁止执行并由 `robot_patrol` 再校验一次。

手工导出仅用于维护排查：

```bash
python3 scripts/export_web_map.py /实际路径/map.yaml \
  --output-dir web/maps --map-id 地图ID --revision 1 --backend fastlio
```

`resolution`、`origin`、`width`、`height` 均取自实际 Nav2 地图 YAML/栅格；不要在网页端手工修改这些几何值。Nav2 仍从原 YAML/PGM 加载地图，本目录只服务网页显示。
