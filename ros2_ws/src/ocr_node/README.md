# OCR Node — 基于 PaddleOCR 的 ROS2 文字/数字识别节点

完全解耦的独立 ROS2 节点，支持两种工作模式：

| 模式 | 默认 | 适用场景 |
|------|------|----------|
| **Service 按需识别** ✅ | 默认 | 「机器人走到某位置 → 拍一张 → 识别」 |
| **Streaming 流式识别** | 可选 | 持续监控画面中的文字变化 |

---

## 典型工作流（按需识别）

```
机器人导航决策节点                    ocr_node                       hk_camera_node
┌──────────────┐    /ocr/recognize   ┌────────────────┐  订阅帧      ┌──────────────┐
│ 到达目标位置  │ ──────────────────▶│ 取最新缓存帧    │ ◀─────────── │ 持续推流      │
│              │                    │ PaddleOCR 推理   │             │              │
│ 收到识别结果  │ ◀──────────────────│ 返回 detections  │             │              │
│ 做下一步决策  │                    └────────────────┘             └──────────────┘
└──────────────┘
```

### Python 调用示例

```python
from ocr_interfaces.srv import RecognizeText

# 在机器人到达目标点后，调用服务识别当前画面
cli = self.create_client(RecognizeText, '/ocr/recognize')

req = RecognizeText.Request()
req.conf_threshold = 0.0  # 0 = 使用节点默认阈值

future = cli.call_async(req)
# ... 等待结果 ...

if future.result().success:
    for det in future.result().detections:
        print(f"[{det.confidence:.2f}] {det.text}")
```

### 命令行调用

```bash
# 手动触发一次识别
ros2 service call /ocr/recognize ocr_interfaces/srv/RecognizeText "{conf_threshold: 0.0}"

# 返回示例:
# success: true
# message: 'OK: 3 text(s) in 180ms'
# detections:
#   - corners: [120.0, 50.0, 300.0, 50.0, ...]
#     text: "01-23号 货架"
#     confidence: 0.96
# processing_time_ms: 180.0
```

---

## 架构

```
src/ocr_interfaces/          # 纯接口包 (CMake, rosidl)
├── msg/
│   ├── OcrDetection.msg     # 单条检测: corners[8] + text + confidence
│   └── OcrResult.msg        # 一帧结果: header + detections[] + proc_time
└── srv/
    └── RecognizeText.srv    # 按需识别服务: conf_threshold → detections[]

src/ocr_node/                # Python 节点包 (ament_python)
├── ocr_node/
│   ├── __init__.py
│   └── ocr_node.py          # 主节点实现
├── config/
│   └── ocr_params.yaml      # 默认参数
├── launch/
│   └── ocr_node.launch.py   # 启动文件
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 安装

### 1. 安装 PaddleOCR

```bash
pip install paddlepaddle paddleocr      # CPU 版（推荐，通用）
# 或
pip install paddlepaddle-gpu paddleocr  # GPU 版
```

### 2. 编译

```bash
cd ~/kybot_ws
colcon build --packages-select ocr_interfaces ocr_node
source install/setup.bash
```

---

## 使用方式

### 方式一：Service 模式（默认，推荐）

```bash
# 启动 OCR 节点（纯 service 模式，不做持续识别）
ros2 launch ocr_node ocr_node.launch.py

# 需要识别时，主动调用服务
ros2 service call /ocr/recognize ocr_interfaces/srv/RecognizeText "{conf_threshold: 0.0}"
```

节点始终缓存相机的最新一帧，服务回调**在当前帧上运行 OCR，阻塞直到完成**（通常 100~500ms），然后返回结果。

### 方式二：Streaming + Service 并存

```bash
# 开启流式识别（持续识别 + 服务同时可用）
ros2 launch ocr_node ocr_node.launch.py enable_streaming:=true
```

流式识别结果通过 `/ocr/result` 话题发布，同时 `/ocr/recognize` 服务始终可用。

### 与 hk_camera 同时启动

在 `kybot_bringup` launch 文件中添加：

```python
ocr_node = Node(
    package='ocr_node',
    executable='ocr_node',
    name='ocr_node',
    output='screen',
    parameters=[{
        'lang': 'ch',
        'use_gpu': False,           # Jetson 建议 CPU
        'conf_threshold': 0.5,
        'enable_streaming': False,   # 按需模式
    }],
)
```

---

## ROS2 API

### 订阅（始终开启，用于缓存最新帧）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/hk_camera/image_raw` | `sensor_msgs/Image` | 海康相机图像流 |

### Service（核心）

| 服务 | 类型 | 说明 |
|------|------|------|
| `/ocr/recognize` | `ocr_interfaces/RecognizeText` | **按需识别**：取最新帧运行 OCR |

**Request:**
```
float32 conf_threshold   # 0.0 = 使用节点参数默认值；> 0 则覆盖
```

**Response:**
```
bool success                         # false = 无帧 / 推理失败
string message                       # 状态描述
OcrDetection[] detections            # 识别结果列表
  float32[8] corners                # 四边形角点 (x1,y1, …, x4,y4)
  string text
  float32 confidence
float32 processing_time_ms           # 推理耗时
```

### 话题（仅 `enable_streaming: true` 时）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ocr/result` | `ocr_interfaces/OcrResult` | 流式识别结果 |
| `/ocr/annotated_img` | `sensor_msgs/Image` | 标注预览图 |

### 参数

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `lang` | string | `"ch"` | `ch` 中英 / `en` 英文 |
| `use_gpu` | bool | `true` | GPU 加速 |
| `conf_threshold` | float | `0.5` | 置信度阈值 |
| `enable_streaming` | bool | `false` | **开启流式识别** |
| `processing_interval` | float | `0.5` | 流式模式下两次识别的间隔(秒) |
| `enable_annotated_img` | bool | `true` | 流式模式下发布标注图 |

---

## Jetson 调优

```yaml
ocr_node:
  ros__parameters:
    use_gpu: false            # Paddle-GPU 在 Jetson 上安装复杂，CPU 够用
    conf_threshold: 0.6       # 提高阈值减少误检
    enable_streaming: false   # 按需模式，只在需要时消耗算力
```
