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
import os
import queue
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_common.paths import PROJECT_ROOT

VOICE = 'zh-CN-XiaoxiaoNeural'   # edge-tts 中文女声
ALERT_COOLDOWN = 30.0            # 识别播报冷却（秒）


class PlaybackStopped(Exception):
    pass


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameter('voice', VOICE)
        self.declare_parameter('alert_on_person', True)      # 发现人员自动播报
        self.declare_parameter('alert_text', '发现人员，请注意')
        self.declare_parameter(
            'cache_dir',
            str(PROJECT_ROOT / 'data' / 'tts_cache'))

        self._voice = self.get_parameter('voice').value
        self._alert_on_person = bool(self.get_parameter('alert_on_person').value)
        self._alert_text = self.get_parameter('alert_text').value
        self._cache_dir = Path(self.get_parameter('cache_dir').value)
        self._cache_dir.mkdir(parents=True, exist_ok=True)

        self._q = queue.Queue(maxsize=20)
        self._last_alert = 0.0
        self._audio_lock = threading.RLock()
        self._generation = 0
        self._play_process = None
        self._shutdown = threading.Event()
        self._audio_state = dict(state='idle', ready=True, filename='', request_id='', message='待命')

        self.create_subscription(String, '/tts_say', self._on_say, 10)
        self.create_subscription(
            String, '/audio_play', self._on_audio_play, 10)
        self.declare_parameter(
            'audio_dir',
            os.environ.get('ROBOT_AUDIO_DIR', str(PROJECT_ROOT / 'data' / 'audio')))
        self._audio_dir = Path(self.get_parameter('audio_dir').value).expanduser().resolve()
        self._audio_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            String, '/detection_events', self._on_detection, 10)
        self.create_subscription(String, '/audio_stop', self._on_audio_stop, 10)
        self._pub_audio = self.create_publisher(String, '/audio/status', 10)
        self.create_timer(1., self._publish_audio_state)
        self._pub_status = self.create_publisher(String, '/tts_status', 10)

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self._publish_audio_state()
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
        """Accept an uploaded filename or a correlated web playback request."""
        name, request_id = '', ''
        try:
            payload = json.loads(msg.data) if msg.data.lstrip().startswith('{') else {'filename': msg.data.strip()}
            name, request_id = payload.get('filename', ''), str(payload.get('request_id', ''))[:200]
            if not isinstance(name, str) or not name or Path(name).name != name:
                raise ValueError('音频文件名无效')
            path = (self._audio_dir / name).resolve()
            if path.parent != self._audio_dir or not path.is_file() or path.suffix.lower() not in ('.mp3', '.wav', '.ogg', '.m4a', '.flac'):
                raise ValueError('音频文件不存在')
            with self._audio_lock:
                self._q.put_nowait(('FILE', str(path), request_id, self._generation))
                self._set_audio_state('queued', name, request_id, '已加入机器人播报队列')
        except (ValueError, TypeError, AttributeError, queue.Full) as error:
            self._set_audio_state('error', str(name), request_id,
                                  '播报队列已满' if isinstance(error, queue.Full) else str(error))

    def _set_audio_state(self, state, filename='', request_id='', message=''):
        with self._audio_lock:
            self._audio_state.update(state=state, filename=filename, request_id=request_id, message=message)
        self._publish_audio_state()

    def _publish_audio_state(self):
        with self._audio_lock:
            value = dict(self._audio_state, queue=self._q.qsize(), stamp=time.time())
        self._pub_audio.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def _on_audio_stop(self, msg):
        with self._audio_lock:
            self._generation += 1
            process = self._play_process
            while True:
                try:
                    self._q.get_nowait()
                except queue.Empty:
                    break
            if process is not None and process.poll() is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
            self._set_audio_state('stopped', request_id=msg.data[:200], message='播报已停止，队列已清空')

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

    def _play(self, path, generation):
        with self._audio_lock:
            if generation != self._generation or self._shutdown.is_set():
                raise PlaybackStopped()
            process = subprocess.Popen(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', path],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self._play_process = process
        deadline = time.monotonic()+605
        try:
            while process.poll() is None:
                if generation != self._generation or self._shutdown.is_set():
                    raise PlaybackStopped()
                if time.monotonic() >= deadline:
                    raise TimeoutError('音频播放超时')
                time.sleep(.05)
            if generation != self._generation:
                raise PlaybackStopped()
            _, error = process.communicate()
            if process.returncode:
                raise RuntimeError('机器人音频输出失败：'+error.decode('utf-8', 'replace')[-300:])
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.)
            if process.stderr:
                process.stderr.close()
            with self._audio_lock:
                if self._play_process is process:
                    self._play_process = None

    def _publish_status(self, status, text=''):
        self._pub_status.publish(String(data=json.dumps(
            {'type': 'tts', 'status': status, 'text': text, 'ts': int(time.time())},
            ensure_ascii=False)))

    def _worker(self):
        while rclpy.ok() and not self._shutdown.is_set():
            try:
                item = self._q.get(timeout=.2)
            except queue.Empty:
                continue
            if isinstance(item, tuple) and item[0] == 'FILE':
                # 已上传音频文件：直接播放，不走合成
                label = Path(item[1]).name
                request_id, generation = item[2], item[3]
                if generation != self._generation:
                    continue
                self._set_audio_state('playing', label, request_id, '机器人正在播报')
                self._publish_status('speaking', label)
                try:
                    self._play(item[1], generation)
                    self._publish_status('done', label)
                    if generation == self._generation:
                        self._set_audio_state('done', label, request_id, '播报完成')
                except PlaybackStopped:
                    pass
                except Exception as e:
                    self.get_logger().error(f'播放失败: {e}')
                    self._publish_status('error', label)
                    if generation == self._generation:
                        self._set_audio_state('error', label, request_id, str(e))
                continue
            text = item
            generation = self._generation
            self._publish_status('speaking', text)
            path = self._synth(text)
            if path:
                try:
                    self._play(path, generation)
                    self._publish_status('done', text)
                except PlaybackStopped:
                    pass
                except Exception as e:
                    self.get_logger().error(f'播放失败: {e}')
                    self._publish_status('error', text)
            else:
                self._publish_status('error', text)

    def destroy_node(self):
        self._shutdown.set()
        self._on_audio_stop(String(data='shutdown'))
        self._thread.join(timeout=2.)
        return super().destroy_node()


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
