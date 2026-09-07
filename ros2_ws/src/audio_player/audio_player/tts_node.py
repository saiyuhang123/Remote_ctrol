#!/usr/bin/env python3
"""语音播报/智能提示节点（机器人端扬声器输出）。

- 订阅 /tts_say（std_msgs/String，纯文本）→ TTS 合成 → 扬声器播放（串行队列）
- 订阅 /detection_events（识别事件）→ 发现人员自动播报（冷却 30s，可关）
- 发布 /tts_status（String JSON：speaking/done/error）

TTS 引擎：edge-tts（在线，自然音质，结果按文本哈希缓存）；
          离线或失败自动回退 espeak-ng（机械音兜底）。
播放：ffplay（PulseAudio/WSLg 或真机声卡均可）。
"""
import hashlib
import json
import queue
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

VOICE = 'zh-CN-XiaoxiaoNeural'   # edge-tts 中文女声
ALERT_COOLDOWN = 30.0            # 识别播报冷却（秒）


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameter('voice', VOICE)
        self.declare_parameter('alert_on_person', True)      # 发现人员自动播报
        self.declare_parameter('alert_text', '发现人员，请注意')
        self.declare_parameter(
            'cache_dir',
            str(Path.home() / 'ros2Project' / 'Remote_ctrol' / 'data' / 'tts_cache'))

        self._voice = self.get_parameter('voice').value
        self._alert_on_person = bool(self.get_parameter('alert_on_person').value)
        self._alert_text = self.get_parameter('alert_text').value
        self._cache_dir = Path(self.get_parameter('cache_dir').value)
        self._cache_dir.mkdir(parents=True, exist_ok=True)

        self._q = queue.Queue(maxsize=20)
        self._last_alert = 0.0

        self.create_subscription(String, '/tts_say', self._on_say, 10)
        self.create_subscription(
            String, '/audio_play', self._on_audio_play, 10)
        self.declare_parameter(
            'audio_dir',
            str(Path.home() / 'ros2Project' / 'Remote_ctrol' / 'web' / 'audio'))
        self._audio_dir = Path(self.get_parameter('audio_dir').value)
        self._audio_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            String, '/detection_events', self._on_detection, 10)
        self._pub_status = self.create_publisher(String, '/tts_status', 10)

        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(
            f'TTS 节点就绪：语音={self._voice}，人员播报={"开" if self._alert_on_person else "关"}')

    # ---------------- 订阅 ----------------
    def _on_say(self, msg):
        text = msg.data.strip()
        if not text:
            return
        try:
            self._q.put_nowait(text)
        except queue.Full:
            self.get_logger().warn('播报队列已满，丢弃')

    def _on_audio_play(self, msg):
        """播放已上传的音频文件（文件名，只允许在 audio_dir 内）"""
        name = Path(msg.data.strip()).name
        path = self._audio_dir / name
        if not name or not path.exists():
            self.get_logger().warn(f'音频文件不存在: {msg.data}')
            self._publish_status('error', f'文件不存在: {name}')
            return
        try:
            self._q.put_nowait(('FILE', str(path)))
        except queue.Full:
            self.get_logger().warn('播报队列已满，丢弃')

    def _on_detection(self, msg):
        if not self._alert_on_person:
            return
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get('event') not in ('person', 'person_face'):
            return
        now = time.time()
        if now - self._last_alert < ALERT_COOLDOWN:
            return
        self._last_alert = now
        self._on_say(String(data=self._alert_text))

    # ---------------- 合成与播放 ----------------
    def _synth(self, text):
        """返回音频文件路径；edge-tts 优先，失败回退 espeak-ng。"""
        key = hashlib.md5(f'{self._voice}|{text}'.encode('utf-8')).hexdigest()
        mp3 = self._cache_dir / f'{key}.mp3'
        if mp3.exists() and mp3.stat().st_size > 0:
            return str(mp3)
        try:
            import edge_tts
            import asyncio
            async def _gen():
                await edge_tts.Communicate(text, self._voice).save(str(mp3))
            asyncio.run(_gen())
            return str(mp3)
        except Exception as e:
            self.get_logger().warn(f'edge-tts 失败（{e}），回退 espeak-ng')
            wav = self._cache_dir / f'{key}.wav'
            try:
                subprocess.run(['espeak-ng', '-v', 'zh', '-f', '/dev/stdin', '-w', str(wav)],
                               input=text.encode('utf-8'), check=True, timeout=15)
                return str(wav)
            except Exception as e2:
                self.get_logger().error(f'espeak-ng 也失败: {e2}')
                return None

    def _play(self, path):
        subprocess.run(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', path],
                       timeout=60)

    def _publish_status(self, status, text=''):
        self._pub_status.publish(String(data=json.dumps(
            {'type': 'tts', 'status': status, 'text': text, 'ts': int(time.time())},
            ensure_ascii=False)))

    def _worker(self):
        while rclpy.ok():
            try:
                item = self._q.get(timeout=1.0)
            except queue.Empty:
                continue
            if isinstance(item, tuple) and item[0] == 'FILE':
                # 已上传音频文件：直接播放，不走合成
                label = Path(item[1]).name
                self._publish_status('speaking', label)
                try:
                    self._play(item[1])
                    self._publish_status('done', label)
                except Exception as e:
                    self.get_logger().error(f'播放失败: {e}')
                    self._publish_status('error', label)
                continue
            text = item
            self._publish_status('speaking', text)
            path = self._synth(text)
            if path:
                try:
                    self._play(path)
                    self._publish_status('done', text)
                except Exception as e:
                    self.get_logger().error(f'播放失败: {e}')
                    self._publish_status('error', text)
            else:
                self._publish_status('error', text)


def main():
    rclpy.init()
    node = TtsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
