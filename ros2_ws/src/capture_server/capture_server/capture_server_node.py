#!/usr/bin/env python3
"""图片抓拍服务节点（合同第 10 条）。

常驻解码视频源并缓存最新帧：
- 订阅 /capture_request（std_msgs/String，data 为可选备注标签）
  → 保存最新帧到 web/captures/日期/，发布 /capture_done（JSON）
- 手动抓拍：网页按钮 -> 网关 -> /capture_request
- 自动抓拍：detector_node（识别联动）或 patrol_manager（到点抓拍）发同一话题

磁盘策略：按日期分目录；超过 MAX_GB 或超过 MAX_DAYS 天自动清最旧的。
"""
import json
import os
import re
import uuid
import threading
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_common.reporter import HttpReporter
from robot_common.paths import PROJECT_ROOT

MAX_DAYS = 14          # 抓拍图保留天数
MAX_GB = 5.0           # 抓拍目录总容量上限
MIN_FRAME_AGE = 1.0    # 缓存帧超过 1s 视为视频流断流，拒绝抓拍


class CaptureServerNode(Node):
    def __init__(self):
        super().__init__('capture_server')
        self.declare_parameter('video_source', 'rtsp://127.0.0.1:8554/cam')
        self.declare_parameter(
            'captures_dir',
            str(PROJECT_ROOT / 'web' / 'captures'))

        self._src = self.get_parameter('video_source').value
        self._cap_dir = Path(self.get_parameter('captures_dir').value)
        self._cap_dir.mkdir(parents=True, exist_ok=True)
        self.declare_parameter('backend_url', 'http://127.0.0.1:8888')
        self._reporter = HttpReporter(
            os.environ.get('ROBOT_BACKEND_URL', self.get_parameter('backend_url').value),
            cache_db=str(PROJECT_ROOT / 'data' / 'capture_cache.db'))

        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_time = 0.0

        self.create_subscription(
            String, '/capture_request', self._on_capture_request, 10)
        self._pub_done = self.create_publisher(String, '/capture_done', 10)
        self._pub_status = self.create_publisher(String, '/capture/status', 10)
        self._recent = {}
        self.create_timer(.5, self._publish_capture_status)

        self.get_logger().info(f'抓拍服务就绪：视频源={self._src}，存档={self._cap_dir}')
        threading.Thread(target=self._grab_loop, daemon=True).start()
        self.create_timer(3600.0, self._housekeeping)   # 每小时磁盘轮转

    # ---------------- 帧缓存 ----------------
    def _grab_loop(self):
        cap = cv2.VideoCapture(self._src)
        while not cap.isOpened():
            self.get_logger().warn(f'视频源 {self._src} 未就绪，3s 后重试…')
            time.sleep(3)
            cap = cv2.VideoCapture(self._src)
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                if isinstance(self._src, str) and Path(self._src).exists():
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                else:
                    time.sleep(0.5)
                    cap = cv2.VideoCapture(self._src)
                continue
            with self._lock:
                self._latest_frame = frame
                self._latest_time = time.time()

    # ---------------- 抓拍 ----------------
    def _publish_capture_status(self):
        with self._lock:
            ready = self._latest_frame is not None and time.time()-self._latest_time <= MIN_FRAME_AGE
        self._pub_status.publish(String(data=json.dumps(dict(ready=ready, stamp=time.time()))))

    def _on_capture_request(self, msg):
        try:
            value = json.loads(msg.data) if msg.data.lstrip().startswith('{') else {'label': msg.data.strip() or 'manual'}
            identifier = value.get('request_id') or uuid.uuid4().hex
            if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', identifier):
                raise ValueError('抓拍编号无效')
            label = str(value.get('label') or 'manual')[:200]
        except (ValueError, AttributeError, TypeError):
            return
        if identifier in self._recent:
            self._pub_done.publish(String(data=json.dumps(self._recent[identifier], ensure_ascii=False)))
            return
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            age = time.time() - self._latest_time
        if frame is None or age > MIN_FRAME_AGE:
            self.get_logger().warn('视频流无最新帧，抓拍失败')
            self._pub_done.publish(String(data=json.dumps(
                {'type': 'capture', 'ok': False, 'request_id': identifier, 'reason': '视频流无数据'},
                ensure_ascii=False)))
            return

        day = time.strftime('%Y-%m-%d')
        out_dir = self._cap_dir / day
        fname = f"{time.time_ns()}_{uuid.uuid4().hex}.jpg"
        path = out_dir / fname
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                raise OSError('图片写入失败')
        except (OSError, cv2.error) as error:
            self._pub_done.publish(String(data=json.dumps(dict(type='capture', ok=False, request_id=identifier, reason=str(error)))))
            return
        self.get_logger().info(f'抓拍已保存: {path.name}')
        result = dict(type='capture', ok=True, capture_id=identifier, request_id=identifier,
                      label=label, ts=time.time(), image=f'captures/{day}/{fname}')
        self._reporter.post('/api/records/captures', result)
        self._recent[identifier] = result
        while len(self._recent)>50:
            self._recent.pop(next(iter(self._recent)))
        self._pub_done.publish(String(data=json.dumps(result, ensure_ascii=False)))

    # ---------------- 磁盘轮转 ----------------
    def _housekeeping(self):
        try:
            now = time.time()
            dirs = sorted(
                [d for d in self._cap_dir.iterdir() if d.is_dir()],
                key=lambda d: d.name)
            # 超龄目录
            for d in dirs:
                try:
                    day_ts = time.mktime(time.strptime(d.name, '%Y-%m-%d'))
                except ValueError:
                    continue
                if now - day_ts > MAX_DAYS * 86400:
                    for f in d.iterdir():
                        f.unlink()
                    d.rmdir()
                    self.get_logger().info(f'清理过期抓拍目录: {d.name}')
            # 容量上限：清最旧
            def dir_bytes(p):
                return sum(f.stat().st_size for f in p.iterdir() if f.is_file())
            dirs = [d for d in self._cap_dir.iterdir() if d.is_dir()]
            while dirs and sum(dir_bytes(d) for d in dirs) > MAX_GB * 1e9:
                victim = dirs.pop(0)
                for f in victim.iterdir():
                    f.unlink()
                victim.rmdir()
                self.get_logger().info(f'容量超限，清理最旧目录: {victim.name}')
        except Exception as e:
            self.get_logger().warn(f'磁盘轮转异常: {e}')


def main():
    rclpy.init()
    node = CaptureServerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
