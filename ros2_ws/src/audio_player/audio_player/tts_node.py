#!/usr/bin/env python3
"""机器人固定提示音及手动播报的统一播放服务。

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
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from robot_common.paths import PROJECT_ROOT
from ament_index_python.packages import get_package_share_directory
from .notifications import Playback, PlaybackQueue, Speech, load_config

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

        self._q = PlaybackQueue(maxsize=20)
        self._last_alert = 0.0
        self._audio_lock = threading.RLock()
        self._generation = 0
        self._play_process = None
        self._current_item = None
        self._current_cancel = threading.Event()
        self._synth_slots = threading.BoundedSemaphore(2)
        self._intercom_busy = False
        self._intercom_time = 0.
        self._notice_seen = OrderedDict()
        self._scope_versions = {}
        share = Path(get_package_share_directory('audio_player'))
        self.declare_parameter('enable_prompts', os.environ.get('ROBOT_ENABLE_PROMPTS', '1') != '0')
        self.declare_parameter('prompt_config', os.environ.get('ROBOT_PROMPT_CONFIG', str(share/'config/prompts.json')))
        self.declare_parameter('prompt_dir', os.environ.get('ROBOT_PROMPT_DIR', str(share/'prompts')))
        self._prompt_config = load_config(self.get_parameter('prompt_config').value)
        self._prompt_dir = Path(self.get_parameter('prompt_dir').value).expanduser().resolve()
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
        self.create_subscription(String, '/audio/notify', self._on_notification, 20)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/audio/intercom_state', self._on_intercom_state, latched)

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self._publish_audio_state()
        self.get_logger().info(
            f'音频节点就绪：固定提示={"开" if self.get_parameter("enable_prompts").value else "关"}，'
            f'手动合成音色={self._voice}')

    # ---------------- 订阅 ----------------
    def _on_say(self, msg):
        text = msg.data.strip()
        if not text or self._call_busy():
            return
        try:
            self._q.put_nowait(Speech(text, self._generation))
        except queue.Full:
            self.get_logger().warn('播报队列已满，丢弃')

    def _on_audio_play(self, msg):
        """Accept an uploaded filename or a correlated web playback request."""
        name, request_id = '', ''
        try:
            if self._call_busy():
                raise ValueError('机器人正在对讲，暂不播放音频')
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
        try:
            self._pub_audio.publish(String(data=json.dumps(value, ensure_ascii=False)))
        except Exception:
            if rclpy.ok() and not self._shutdown.is_set():
                raise

    def _on_audio_stop(self, msg):
        with self._audio_lock:
            if msg.data == 'intercom':
                self._intercom_busy, self._intercom_time = True, time.monotonic()
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

    def _call_busy(self):
        # An absent/crashed gateway cannot permanently reserve the loudspeaker.
        return bool(getattr(self, '_intercom_busy', False) and
                    time.monotonic()-getattr(self, '_intercom_time', 0.) <= 3)

    def _interrupt_current(self):
        self._current_cancel.set()
        process = self._play_process
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass

    def _on_intercom_state(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            busy = bool(data.get('reserved'))
        except (ValueError, TypeError):
            return
        with self._audio_lock:
            was_busy = self._call_busy()
            self._intercom_busy, self._intercom_time = busy, time.monotonic()
            if busy and not was_busy:
                self._on_audio_stop(String(data='intercom'))

    def _on_notification(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            scope = str(data.get('scope', ''))[:200]
            with self._audio_lock:
                if data.get('clear') is True:
                    if scope:
                        self._scope_versions[scope] = self._scope_versions.get(scope, 0)+1
                        self._q.discard_scope(scope)
                        if isinstance(self._current_item, Playback) and self._current_item.scope == scope:
                            self._interrupt_current()
                    return
                key = str(data.get('key', ''))
                spec = self._prompt_config['clips'].get(key)
                if not spec or not spec.get('enabled', True):
                    return
                # The connected cue is a live-call exception. An ended cue may
                # arrive just before the player's matching idle-state sample.
                # Intercom mutes its PCM input/output while the connected cue plays.
                if self._call_busy() and key not in ('intercom_started', 'intercom_ended'):
                    return
                token = str(data.get('token', ''))[:200]
                identity = (key, token)
                if token and identity in self._notice_seen:
                    return
                # File names come only from the validated configuration. Do
                # not reject colcon --symlink-install's per-file asset links.
                path = self._prompt_dir/spec.get('file', key+'.wav')
                if not path.is_file():
                    raise ValueError('缺少提示音文件：'+str(path))
                item = Playback(str(path), ('prompt:'+key+':'+token)[:200], self._generation,
                                int(spec['priority']), time.monotonic()+spec['ttl'], scope, key, token,
                                self._scope_versions.get(scope, 0))
                self._q.put_nowait(item)
                if token:
                    self._notice_seen[identity] = None
                    while len(self._notice_seen) > 2048:
                        self._notice_seen.popitem(last=False)
                current_priority = self._current_item.priority if isinstance(self._current_item, Playback) else 20
                if item.priority >= 80 and item.priority > current_priority:
                    self._interrupt_current()
        except (ValueError, TypeError, KeyError, queue.Full) as error:
            self.get_logger().warn('提示音未加入播放队列：'+str(error))

    def _on_detection(self, msg):
        if not self._alert_on_person or bool(self.get_parameter('enable_prompts').value):
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
            async def bounded():
                await asyncio.wait_for(_gen(), timeout=8.)
            asyncio.run(bounded())
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

    def _synth_interruptible(self, text, generation, cancel):
        """A slow optional online TTS request cannot hold up a fixed alarm WAV."""
        if not self._synth_slots.acquire(blocking=False):
            return None
        done, result = threading.Event(), []
        def synthesize():
            try:
                result.append(self._synth(text))
            finally:
                self._synth_slots.release()
                done.set()
        threading.Thread(target=synthesize, daemon=True).start()
        while not done.wait(.05):
            if cancel.is_set() or generation != self._generation or self._shutdown.is_set():
                raise PlaybackStopped()
        if cancel.is_set() or generation != self._generation or self._shutdown.is_set():
            raise PlaybackStopped()
        return result[0] if result else None

    def _play(self, path, generation):
        cancel = getattr(self, '_current_cancel', None)
        with self._audio_lock:
            if generation != self._generation or self._shutdown.is_set() or (cancel and cancel.is_set()):
                raise PlaybackStopped()
            process = subprocess.Popen(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', path],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self._play_process = process
        deadline = time.monotonic()+605
        try:
            while process.poll() is None:
                if generation != self._generation or self._shutdown.is_set() or (cancel and cancel.is_set()):
                    raise PlaybackStopped()
                if time.monotonic() >= deadline:
                    raise TimeoutError('音频播放超时')
                time.sleep(.05)
            if generation != self._generation or (cancel and cancel.is_set()):
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
        try:
            self._pub_status.publish(String(data=json.dumps(
                {'type': 'tts', 'status': status, 'text': text, 'ts': int(time.time())},
                ensure_ascii=False)))
        except Exception:
            if rclpy.ok() and not self._shutdown.is_set():
                raise

    def _worker(self):
        while rclpy.ok() and not self._shutdown.is_set():
            try:
                item = self._q.get(timeout=.2)
            except queue.Empty:
                continue
            with self._audio_lock:
                self._current_item = item
                self._current_cancel = threading.Event()
                generation = item.generation if isinstance(item, (Playback, Speech)) else item[3] if isinstance(item, tuple) else self._generation
                if generation != self._generation:
                    continue
                if isinstance(item, Playback) and item.deadline <= time.monotonic():
                    continue
                if isinstance(item, Playback) and item.scope_version != self._scope_versions.get(item.scope, 0):
                    continue
                if self._call_busy() and not (isinstance(item, Playback) and item.key in ('intercom_started', 'intercom_ended')):
                    continue
            if isinstance(item, Playback) or (isinstance(item, tuple) and item[0] == 'FILE'):
                # 已上传音频文件：直接播放，不走合成
                path = item.path if isinstance(item, Playback) else item[1]
                label = Path(path).name
                request_id = item.request_id if isinstance(item, Playback) else item[2]
                if generation != self._generation:
                    continue
                self._audio_state['notification'] = item.key if isinstance(item, Playback) else ''
                self._set_audio_state('playing', label, request_id, '机器人正在播报')
                self._publish_status('speaking', label)
                try:
                    self._play(path, generation)
                    self._publish_status('done', label)
                    if generation == self._generation:
                        self._set_audio_state('done', label, request_id, '播报完成')
                except PlaybackStopped:
                    if generation == self._generation:
                        self._set_audio_state('stopped', label, request_id, '播报已被优先提示或状态变化打断')
                except Exception as e:
                    self.get_logger().error(f'播放失败: {e}')
                    self._publish_status('error', label)
                    if generation == self._generation:
                        self._set_audio_state('error', label, request_id, str(e))
                continue
            text = item.text if isinstance(item, Speech) else item
            self._audio_state['notification'] = ''
            self._publish_status('speaking', text)
            try:
                path = self._synth_interruptible(text, generation, self._current_cancel)
            except PlaybackStopped:
                continue
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
    import fcntl
    # Navigation and web launchers can coexist: exactly one player/observer per
    # user and ROS domain. A standby launcher takes over if the owner exits.
    lock_path = '/tmp/remote_ctrol_audio_%s_%s.lock' % (os.getuid(), os.environ.get('ROS_DOMAIN_ID', '0'))
    with open(lock_path, 'a') as lock:
        announced = False
        try:
            while True:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if not announced:
                        print('audio_player already running; standby reuses this ROS-domain service', flush=True)
                        announced = True
                    time.sleep(.5)
            rclpy.init()
            node = TtsNode()
            observer = None
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            try:
                if bool(node.get_parameter('enable_prompts').value):
                    from .prompt_node import StatusPrompts
                    observer = StatusPrompts(node.get_parameter('prompt_config').value)
                    executor.add_node(observer)
                executor.spin()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                executor.shutdown(timeout_sec=2.)
                if observer is not None:
                    observer.destroy_node()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
