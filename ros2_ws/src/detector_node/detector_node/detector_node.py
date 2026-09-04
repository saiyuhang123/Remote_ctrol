#!/usr/bin/env python3
"""图像识别节点（纯 CPU）：行人/人脸/车牌。

链路：视频源(默认 MediaMTX RTSP) -> YOLOv8n(OpenVINO) 检测+ByteTrack 跟踪
      -> 行人分支(人脸质量门->SFace 比对) / 车辆分支(HyperLPR3 车牌)
      -> 按轨迹聚合一次性上报 + 时空冷却去重
      -> /detection_events 话题(JSON) + 抓拍存档 web/captures/

设计要点（对比 test/ 原型的修复）：
- track 每帧都跑，只有重分支抽帧（解决 ID 爆炸）
- 按轨迹聚合：收集期内攒最佳人脸匹配，一次性上报（不再 94 条 Stranger）
- 时空冷却：同区域同类别事件不重复上报
"""
import json
import os
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

# CPU 线程限流（i7 6核12线程，给其他任务留余量）
os.environ.setdefault('OMP_NUM_THREADS', '4')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '4')
os.environ.setdefault('MKL_NUM_THREADS', '4')

import torch
torch.set_num_threads(4)
cv2.setNumThreads(4)

from ultralytics import YOLO
import hyperlpr3 as pr3

YOLO_CLASSES = [0, 2, 5, 7]          # person, car, bus, truck
FACE_SCORE_REPORT = 0.5              # 达到即上报姓名
FACE_COLLECT_SEC = 2.5               # 人脸收集期（超时按现有最佳结果上报）
PERSON_MIN_H = 90                    # 人框最小高度(px)，太小不具备识别条件
CAR_MIN_W = 90
PLATE_MIN_CONF = 0.7
FACE_MIN_PX = 50                     # 人脸最小尺寸(px)
FACE_MIN_SCORE = 0.7                 # YuNet 置信度门
SFACE_THRESHOLD = 0.363              # SFace 余弦相似度阈值
COOLDOWN_SAME_IDENTITY = 1800.0      # 同一人/同一车牌 30 分钟内不重复上报
COOLDOWN_ZONE = 600.0                # 画面同区域"未识别人员"10 分钟冷却


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.declare_parameter('video_source', 'rtsp://127.0.0.1:8554/cam')
        self.declare_parameter('conf_threshold', 0.45)
        self.declare_parameter('infer_width', 640)
        self.declare_parameter('infer_height', 480)
        self.declare_parameter(
            'captures_dir',
            str(Path.home() / 'ros2Project' / 'Remote_ctrol' / 'web' / 'captures'))
        self.declare_parameter(
            'face_db_dir',
            str(Path.home() / 'ros2Project' / 'Remote_ctrol' / 'face_db'))

        self._src = self.get_parameter('video_source').value
        self._conf = float(self.get_parameter('conf_threshold').value)
        self._iw = int(self.get_parameter('infer_width').value)
        self._ih = int(self.get_parameter('infer_height').value)
        self._cap_dir = Path(self.get_parameter('captures_dir').value)
        self._cap_dir.mkdir(parents=True, exist_ok=True)

        share = get_package_share_directory('detector_node')
        models = Path(share) / 'models'
        face_db = self.get_parameter('face_db_dir').value or str(Path(share) / 'face_db')

        # 模型：YOLOv8n(OpenVINO 优先) + YuNet + SFace + HyperLPR3
        ov_dir = models / 'yolov8n_openvino_model'
        self._yolo = YOLO(str(ov_dir if ov_dir.exists() else models / 'yolov8n.pt'),
                          task='detect')
        self._tracker_yaml = str(Path(share) / 'config' / 'bytetrack_custom.yaml')

        self._face_det = cv2.FaceDetectorYN.create(
            str(models / 'face_detection_yunet.onnx'), '', (320, 320),
            score_threshold=FACE_MIN_SCORE)
        self._face_rec = cv2.FaceRecognizerSF.create(
            str(models / 'face_recognition_sface.onnx'), '')
        self._face_db = self._load_face_db(face_db)
        self._plate = pr3.LicensePlateCatcher()

        self._tracks = {}        # track_id -> dict(state)
        self._recent = {}        # 去重键 -> 上次上报时间
        self._events = queue.Queue(maxsize=100)   # 待发布事件
        self._pub = self.create_publisher(String, '/detection_events', 10)
        self.create_timer(0.1, self._flush_events)  # 10Hz 发布

        self.get_logger().info(
            f'识别节点就绪：视频源={self._src}，人脸库={list(self._face_db) or "空"}')
        threading.Thread(target=self._capture_loop, daemon=True).start()

    # ---------------- 人脸底库 ----------------
    def _load_face_db(self, db_dir):
        db = {}
        d = Path(db_dir)
        d.mkdir(parents=True, exist_ok=True)
        for f in d.iterdir():
            if f.suffix.lower() not in ('.jpg', '.png', '.jpeg'):
                continue
            img = cv2.imread(str(f))
            if img is None:
                continue
            self._face_det.setInputSize((img.shape[1], img.shape[0]))
            _, faces = self._face_det.detect(img)
            if faces is not None and len(faces):
                db[f.stem] = self._face_rec.feature(
                    self._face_rec.alignCrop(img, faces[0]))
        return db

    def _recognize_face(self, crop):
        """满足质量门才提特征比对；返回 (name, score) 或 None"""
        h, w = crop.shape[:2]
        self._face_det.setInputSize((w, h))
        _, faces = self._face_det.detect(crop)
        if faces is None or not len(faces):
            return None
        f = faces[0]
        fw, fh = f[2], f[3]
        if fw < FACE_MIN_PX or fh < FACE_MIN_PX:
            return None                      # 脸太小，不具备识别条件
        feat = self._face_rec.feature(self._face_rec.alignCrop(crop, f))
        best, score = None, 0.0
        for name, ref in self._face_db.items():
            s = self._face_rec.match(feat, ref, cv2.FaceRecognizerSF_FR_COSINE)
            if s > score:
                best, score = name, float(s)
        if best and score >= SFACE_THRESHOLD:
            return best, score
        return ('未登记人员', score) if self._face_db else None

    # ---------------- 上报（去重 + 存档 + 入队） ----------------
    def _dedup_ok(self, key, cooldown):
        now = time.time()
        if key in self._recent and now - self._recent[key] < cooldown:
            return False
        self._recent[key] = now
        # 顺带清理过期键
        for k in [k for k, t in self._recent.items() if now - t > 7200]:
            del self._recent[k]
        return True

    def _report(self, track, event, label, score, crop, frame_idx):
        self._save_and_emit(track, event, label, score, crop, frame_idx)
        track['reported'] = True

    def _save_and_emit(self, track, event, label, score, crop, frame_idx):
        day = time.strftime('%Y-%m-%d')
        out_dir = self._cap_dir / day
        out_dir.mkdir(parents=True, exist_ok=True)
        fname = f"{int(time.time())}_{event}_{label}_t{track['tid']}.jpg"
        path = out_dir / fname
        cv2.imwrite(str(path), crop)
        payload = {
            'type': 'detection', 'event': event, 'label': label,
            'score': round(float(score), 3), 'ts': int(time.time()),
            'image': f'captures/{day}/{fname}',
        }
        try:
            self._events.put_nowait(payload)
        except queue.Full:
            self.get_logger().warn('事件队列已满，丢弃')
        self.get_logger().info(f'上报: {event} {label} score={score:.2f}')

    def _flush_events(self):
        while True:
            try:
                payload = self._events.get_nowait()
            except queue.Empty:
                return
            self._pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ---------------- 主循环 ----------------
    def _capture_loop(self):
        cap = cv2.VideoCapture(self._src)
        while not cap.isOpened():
            self.get_logger().warn(f'视频源 {self._src} 未就绪，3s 后重试…')
            time.sleep(3)
            cap = cv2.VideoCapture(self._src)

        frame_idx = 0
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                # 文件源循环播放；流源重连
                if isinstance(self._src, str) and os.path.exists(self._src):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                else:
                    time.sleep(0.5)
                    cap = cv2.VideoCapture(self._src)
                continue
            frame_idx += 1
            try:
                self._process_frame(frame, frame_idx)
            except Exception as e:
                self.get_logger().error(f'帧处理异常: {e}')

    def _process_frame(self, frame, frame_idx):
        h0, w0 = frame.shape[:2]
        infer = cv2.resize(frame, (self._iw, self._ih))
        sx, sy = w0 / self._iw, h0 / self._ih
        now = time.time()

        results = self._yolo.track(
            source=infer, classes=YOLO_CLASSES, conf=self._conf,
            device='cpu', persist=True, tracker=self._tracker_yaml, verbose=False)

        if not results or results[0].boxes is None or results[0].boxes.id is None:
            self._expire_tracks(now)
            return

        boxes = results[0].boxes.xyxy.cpu().numpy()
        clss = results[0].boxes.cls.cpu().numpy().astype(int)
        tids = results[0].boxes.id.cpu().numpy().astype(int)

        for box, cls_id, tid in zip(boxes, clss, tids):
            x1 = max(0, int(box[0] * sx)); y1 = max(0, int(box[1] * sy))
            x2 = min(w0, int(box[2] * sx)); y2 = min(h0, int(box[3] * sy))
            tr = self._tracks.setdefault(
                tid, {'tid': tid, 'reported': False, 'first_seen': now,
                      'last_seen': now, 'best_face': None, 'cls': cls_id})
            tr['last_seen'] = now
            if tr['reported']:
                continue

            # 画面区域键（去重用）：目标中心按 200px 分桶
            zone_key = (int((x1 + x2) / 400), int((y1 + y2) / 400))

            if cls_id == 0:      # 行人
                self._person_branch(tr, frame[y1:y2, x1:x2], (x2 - x1, y2 - y1),
                                    now, frame_idx, zone_key)
            elif cls_id in (2, 5, 7):   # 车辆
                self._vehicle_branch(tr, frame[y1:y2, x1:x2], (x2 - x1, y2 - y1),
                                     frame_idx, zone_key)

        self._expire_tracks(now)

    def _person_branch(self, tr, crop, wh, now, frame_idx, zone_key):
        if wh[1] < PERSON_MIN_H or crop.size == 0:
            return
        # 收集期：质量门通过才识别，保留最佳结果
        face = self._recognize_face(crop)
        if face is not None:
            name, score = face
            if tr['best_face'] is None or score > tr['best_face'][1]:
                tr['best_face'] = (name, score)
            if score >= FACE_SCORE_REPORT and name != '未登记人员':
                if self._dedup_ok(('face', name), COOLDOWN_SAME_IDENTITY):
                    self._report(tr, 'person_face', name, score, crop, frame_idx)
                return
        # 收集期结束：按现有最佳结果一次性上报
        if now - tr['first_seen'] >= FACE_COLLECT_SEC:
            if tr['best_face'] is not None:
                name, score = tr['best_face']
                key = (('face', name) if name != '未登记人员'
                       else ('stranger_zone',) + zone_key)
                if self._dedup_ok(key, COOLDOWN_SAME_IDENTITY if name != '未登记人员'
                                  else COOLDOWN_ZONE):
                    self._report(tr, 'person_face', name, score, crop, frame_idx)
            else:
                # 没看到脸：报普通行人告警（按画面区域冷却去重）
                if self._dedup_ok(('person_zone',) + zone_key, COOLDOWN_ZONE):
                    self._report(tr, 'person', '行人', 0.0, crop, frame_idx)

    def _vehicle_branch(self, tr, crop, wh, frame_idx, zone_key):
        if wh[0] < CAR_MIN_W or crop.size == 0:
            return
        try:
            plates = self._plate(crop)
        except Exception:
            return
        if not plates:
            return
        plate_no, conf, color = plates[0][0], plates[0][1], plates[0][2]
        if conf < PLATE_MIN_CONF:
            return
        if self._dedup_ok(('plate', plate_no), COOLDOWN_SAME_IDENTITY):
            self._report(tr, 'vehicle_plate', f'{plate_no}({color})', conf,
                         crop, frame_idx)

    def _expire_tracks(self, now):
        for tid in [t for t, v in self._tracks.items() if now - v['last_seen'] > 10.0]:
            del self._tracks[tid]


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
