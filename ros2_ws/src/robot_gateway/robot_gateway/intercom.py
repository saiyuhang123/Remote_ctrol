"""Browser initiated duplex PCM calls to a local PortAudio microphone/speaker.

No camera SDK, camera audio channel, automatic answering UI or always-on capture.
"""
import asyncio
import json
import os
import queue
import threading


class Intercom:
    def __init__(self, node, audio_module=None):
        self.node, self.audio_module = node, audio_module
        self.active = False
        self.reserved = False
        self.stop_requested = threading.Event()

    def stop(self):
        self.stop_requested.set()

    def module(self):
        if self.audio_module is None:
            try:
                import sounddevice
                self.audio_module = sounddevice
            except (ImportError, OSError) as error:
                raise ValueError('对讲依赖未安装：需要 sounddevice 和 PortAudio') from error
        return self.audio_module

    def devices(self):
        audio = self.module()
        devices = []
        for index, device in enumerate(audio.query_devices()):
            if any(word in device['name'].lower() for word in ('hikvision', '海康')):
                continue
            if device['max_input_channels'] or device['max_output_channels']:
                devices.append(dict(id=index, name=device['name'],
                    input_channels=device['max_input_channels'], output_channels=device['max_output_channels']))
        return devices

    def open_stream(self, source, sink, microphone, speaker):
        audio = self.module()
        selected = []
        for value, environment, direction in ((microphone, 'ROBOT_INTERCOM_INPUT_DEVICE', 'input'),
                                              (speaker, 'ROBOT_INTERCOM_OUTPUT_DEVICE', 'output')):
            value = os.environ.get(environment, '') if value in (None, '') else value
            if isinstance(value, str) and value.isdecimal():
                value = int(value)
            value = value if value != '' else None
            info = audio.query_devices(value, direction)
            if any(word in info['name'].lower() for word in ('hikvision', '海康')):
                raise ValueError('请选择独立声卡的麦克风和音响')
            selected.append(value)
        rate = 48000
        audio.check_input_settings(device=selected[0], channels=1, dtype='int16', samplerate=rate)
        audio.check_output_settings(device=selected[1], channels=1, dtype='int16', samplerate=rate)
        self.incoming = queue.Queue(maxsize=10)
        self.outgoing = queue.Queue(maxsize=10)
        self.rate, self.frame_bytes = rate, rate//50*2
        self.play_buffer = bytearray()
        def input_callback(data, frames, timing, status):
            if self.stop_requested.is_set():
                return
            try:
                self.outgoing.put_nowait(bytes(data))
            except queue.Full:
                pass
        def output_callback(data, frames, timing, status):
            required = len(data)
            while len(self.play_buffer) < required:
                try:
                    self.play_buffer.extend(self.incoming.get_nowait())
                except queue.Empty:
                    break
            if self.stop_requested.is_set():
                self.play_buffer.clear()
            count = min(required, len(self.play_buffer))
            data[:] = bytes(self.play_buffer[:count])+bytes(required-count)
            del self.play_buffer[:count]
        try:
            source.append(audio.RawInputStream(device=selected[0], samplerate=rate, blocksize=rate//50,
                channels=1, dtype='int16', callback=input_callback, latency='low'))
            sink.append(audio.RawOutputStream(device=selected[1], samplerate=rate, blocksize=rate//50,
                channels=1, dtype='int16', callback=output_callback, latency='low'))
            if self.stop_requested.is_set():
                raise ValueError('通话已取消')
            sink[0].start()
            source[0].start()
        except Exception:
            self.close_streams(source+sink)
            source.clear(); sink.clear()
            raise

    @staticmethod
    def close_streams(streams):
        for stream in streams:
            try:
                stream.abort()
            except Exception:
                pass
            try:
                stream.close()
            except Exception:
                pass

    def authorized(self, token):
        return self.node._owner_valid(token) and self.node._lease_valid()

    async def handle(self, websocket):
        source, sink, owns_call = [], [], False
        try:
            raw = await asyncio.wait_for(websocket.recv(), 10)
            auth = json.loads(raw)
            token = auth.get('session_id', '')
            if not self.authorized(token):
                raise ValueError('请先取得网页控制权')
            try:
                devices = await asyncio.to_thread(self.devices)
                await websocket.send(json.dumps(dict(type='devices', devices=devices, busy=self.reserved), ensure_ascii=False))
            except Exception as error:
                raise ValueError('机器人音频设备未就绪：'+str(error)) from error
            raw = await asyncio.wait_for(websocket.recv(), 60)
            start = json.loads(raw)
            if start.get('type') != 'start' or not self.authorized(token):
                raise ValueError('通话已取消或控制权已释放')
            if self.reserved:
                raise ValueError('机器人正在对讲')
            self.reserved, owns_call = True, True
            self.stop_requested.clear()
            # Uploaded announcements must not compete with a live call.
            self.node._audio_stop_pub.publish(self.node._string_type(data='intercom'))
            await asyncio.to_thread(self.open_stream, source, sink, start.get('input_device'), start.get('output_device'))
            if not self.authorized(token) or self.stop_requested.is_set():
                raise ValueError('开启期间控制权已变化')
            self.active = True
            await websocket.send(json.dumps(dict(type='ready', sample_rate=self.rate, channels=1, format='s16le')))
            while not self.stop_requested.is_set() and self.authorized(token):
                if any(not getattr(stream, 'active', True) for stream in source+sink):
                    raise ValueError('机器人麦克风或音响已断开')
                try:
                    raw = await asyncio.wait_for(websocket.recv(), .02)
                    if isinstance(raw, bytes):
                        if len(raw) != self.frame_bytes:
                            raise ValueError('对讲音频帧无效')
                        try:
                            self.incoming.put_nowait(raw)
                        except queue.Full:
                            self.incoming.get_nowait()
                            self.incoming.put_nowait(raw)
                    elif json.loads(raw).get('type') == 'stop':
                        break
                except asyncio.TimeoutError:
                    pass
                for _ in range(4):
                    try:
                        chunk = self.outgoing.get_nowait()
                    except queue.Empty:
                        break
                    await asyncio.wait_for(websocket.send(chunk), 1.)
            await websocket.send(json.dumps(dict(type='stopped', message='对讲已结束')))
        except Exception as error:
            try:
                await websocket.send(json.dumps(dict(type='error', message=str(error)), ensure_ascii=False))
            except Exception:
                pass
        finally:
            if owns_call:
                self.stop_requested.set()
                self.active = False
                try:
                    await asyncio.to_thread(self.close_streams, source+sink)
                finally:
                    self.reserved = False
            await websocket.close()
