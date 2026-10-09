# 实机地图

原 Gazebo 测试场地的 `patrol.png` 和 `meta.json` 已删除。没有实际地图时，监控台和点位页面显示“尚未配置实机地图”，并禁止地图点选。

实机建图后，将同一张地图转换为浏览器支持的 PNG/JPEG，放在本目录，并创建 `meta.json`：

推荐直接从 Nav2 使用的实际地图 YAML 导出，工具会保持分辨率、原点和图片尺寸：

```bash
python3 /home/jkbot/Remote_ctrol/scripts/export_web_map.py /实际路径/site.yaml
```

FAST-LIO PCD 转栅格的步骤见 [实机导航移植说明](../../实机导航移植说明.md)。手工配置格式如下：

```json
{
  "image": "map.png",
  "resolution": 0.05,
  "origin": [-10.0, -10.0, 0.0],
  "width": 1000,
  "height": 800
}
```

以上数值仅说明格式。`resolution`、`origin` 必须取自实测地图 YAML，`width`、`height` 必须与图片像素尺寸一致；图片保持地图原始行序，页面负责翻转 Y 轴。目前页面仅支持 `origin[2] = 0`。

Nav2 仍使用实测地图的 YAML/PGM 文件，通过 `map:=/实际路径/map.yaml` 传入，不读取此处的网页图片。网页显示的地图必须与 Nav2 使用的地图一致。
