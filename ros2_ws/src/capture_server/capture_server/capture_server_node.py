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
import threading
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAX_DAYS = 14          # 抓拍图保留天数
MAX_GB = 5.0           # 抓拍目录总容量上限
MIN_FRAME_AGE = 1.0    # 缓存帧超过 1s 视为视频流断流，拒绝抓拍


class CaptureServerNode(Node):
    def __init__(self):
        super().__init__('capture_server')
        self.declare_parameter('video_source', 'rtsp://127.0.0.1:8554/cam')
        self.declare_parameter(
            'captures_dir',
            str(Path.home() / 'ros2Project' / 'Remote_ctrol' / 'web' / 'captures'))

        self._src = self.get_parameter('video_source').value
        self._cap_dir = Path(self.get_parameter('captures_dir').value)
        self._cap_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_time = 0.0

        self.create_subscription(
            String, '/capture_request', self._on_capture_request, 10)
        self._pub_done = self.create_publisher(String, '/capture_done', 10)

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
    def _on_capture_request(self, msg):
        label = msg.data.strip() or 'manual'
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            age = time.time() - self._latest_time
        if frame is None or age > MIN_FRAME_AGE:
            self.get_logger().warn('视频流无最新帧，抓拍失败')
            self._pub_done.publish(String(data=json.dumps(
                {'type': 'capture', 'ok': False, 'reason': '视频流无数据'},
                ensure_ascii=False)))
            return

        day = time.strftime('%Y-%m-%d')
        out_dir = self._cap_dir / day
        out_dir.mkdir(parents=True, exist_ok=True)
        fname = f"{int(time.time())}_capture_{label}.jpg"
        path = out_dir / fname
        cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.get_logger().info(f'抓拍已保存: {path.name}')
        self._pub_done.publish(String(data=json.dumps({
            'type': 'capture', 'ok': True, 'label': label,
            'ts': int(time.time()), 'image': f'captures/{day}/{fname}',
        }, ensure_ascii=False)))

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
