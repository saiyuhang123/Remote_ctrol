# 上位机 CPU 模型清单

当前运行链路只使用以下五个文件。完整文件大小与 SHA-256 见 `model_manifest.json`；模型随包安装，不在启动时联网下载。仓库中原有 YOLOv8、SFace 仅作历史兼容保留，当前独立视觉链路不加载它们，也不读取人脸身份库。

| 功能 | 文件 | 来源与许可 |
| --- | --- | --- |
| 人脸位置、小图 | `face_detection_yunet.onnx` | [OpenCV Zoo YuNet](https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet)，MIT，见 `LICENSE.face-yunet` |
| 行人 class 0 | `yolo26n_openvino_model/yolo26n.xml`、`.bin` | [Ultralytics YOLO26n](https://docs.ultralytics.com/models/yolo26/)，AGPL-3.0，见 `LICENSE.ultralytics` |
| 中国车牌四角点 | `license_plate_detection_lpd_yunet_2023mar.onnx` | [OpenCV Zoo LPD-YuNet](https://github.com/opencv/opencv_zoo/tree/main/models/license_plate_detection_yunet)，Apache-2.0，见 `LICENSE.lpd-yunet` |
| 车牌字符识别 | `plate_recognition_hyperlpr3.onnx` | [HyperLPR3](https://github.com/szad670401/HyperLPR)，Apache-2.0，见 `LICENSE.hyperlpr3` |

LPD 原文件下载地址：
`https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/license_plate_detection_yunet/license_plate_detection_lpd_yunet_2023mar.onnx`

字符模型来自 HyperLPR3 0.1.3 安装缓存的 `20230229/onnx/rpv3_mdict_160_r3.onnx`，原始 SHA-256 为 `8fb08b5db2adeccf43b05006bbbf409e4659d08d72e46a62631c00ff751eaeb3`。部署文件使用 ONNX 1.17.0 将同时出现在 graph.input 和 initializer 中的 100 个常量移出 graph.input，权重、计算节点不变；部署文件的 hash 见清单。运行时直接用 ONNX Runtime 加载该文件，不启动 HyperLPR 自带检测、分类或下载逻辑。

## YOLO26n 导出复现

本次在隔离环境 `~/.cache/robot_vision_export` 导出，未给 ROS 的全局 Python 安装 PyTorch。源权重为官方 [v8.4.0/yolo26n.pt](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt)。工具版本为 CPU torch 2.6.0+cpu、torchvision 0.21.0+cpu、ultralytics 8.4.182、onnx 1.17.0、numpy 1.26.4。新机器复现时先创建独立 venv，安装这些版本；torch/torchvision 使用官方 CPU wheel 源。

导出环境执行，`YOLO_AUTOINSTALL=false` 可阻止工具额外安装依赖：

```python
from ultralytics import YOLO
YOLO('/绝对路径/yolo26n.pt').export(
    format='onnx', imgsz=640, batch=1, device='cpu',
    opset=17, simplify=False, dynamic=False, nms=False)
```

然后用运行环境中已固定的 OpenVINO **2024.6.0** 转换：

```python
import openvino as ov
model = ov.convert_model('/绝对路径/yolo26n.onnx')
ov.save_model(model, '/绝对路径/yolo26n_openvino_model/yolo26n.xml', compress_to_fp16=True)
```

实际输入是 `[1,3,640,640]`，输出是 `[1,300,6]` 的 `xyxy,score,class`；使用端到端检测头，后处理不再做第二次 NMS。运行端无需 torch、ultralytics 或 onnx 包。重新导出可能改变文件 hash，需重新验证后更新清单。

## 无设备验证

项目根目录运行 `python3 scripts/verify_vision_models.py`：验证 hash、加载全部模型并对 1080p 空白图推理，不连接相机或 ROS。可用 `--model-dir ros2_ws/install/detector_node/share/detector_node/models` 验证安装产物。

正样本验证使用公开图片，下载到仓库外的临时目录：

- `face.jpg`：[OpenCV lena.jpg](https://raw.githubusercontent.com/opencv/opencv/master/samples/data/lena.jpg)
- `bus.jpg`：[Ultralytics bus.jpg](https://raw.githubusercontent.com/ultralytics/assets/main/im/bus.jpg)
- `plate.jpg`：[HyperLPR test_img.jpg](https://raw.githubusercontent.com/szad670401/HyperLPR/master/resource/images/test_img.jpg)

再运行 `python3 scripts/verify_vision_models.py --samples-dir /图片目录`，验证人脸、行人检出和两帧车牌事件 `苏ED51712`。该验证不写业务记录，不将公开测试图混入实际抓拍目录。
