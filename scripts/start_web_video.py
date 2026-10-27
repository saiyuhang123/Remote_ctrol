#!/usr/bin/env python3
"""Supervise the browser video relay, reusing the local camera configuration."""
from contextlib import suppress
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import tempfile
import threading
import time
from urllib.parse import quote, urlsplit

import yaml


def camera_source(project_dir, environment):
    mode = environment.get('ROBOT_VIDEO_INPUT', 'auto')
    if mode not in ('auto', 'hikvision', 'uvc'):
        raise ValueError('ROBOT_VIDEO_INPUT 必须是 auto、hikvision 或 uvc')
    if mode == 'uvc':
        return None
    override = environment.get('CAMERA_RTSP', '').strip()
    if override:
        if urlsplit(override).scheme not in ('rtsp', 'rtsps'):
            raise ValueError('CAMERA_RTSP 必须是 RTSP 视频地址')
        return override
    config_path = Path(environment.get('ROBOT_CAMERA_CONFIG', str(
        project_dir/'ros2_ws/src/hk_camera/config/hk_camera_params.yaml'))).expanduser()
    if not config_path.is_file():
        if mode == 'auto':
            return None
        raise ValueError('找不到海康相机配置文件')
    document = yaml.safe_load(config_path.read_text()) or {}
    params = document.get('hk_camera_node', {}).get('ros__parameters', {})
    host = str(params.get('ip', '')).strip()
    if not host:
        raise ValueError('海康相机配置缺少 IP')
    if ':' in host:
        host = '['+host+']'
    username, password = (str(params.get(key, '')) for key in ('username', 'password'))
    credentials = (quote(username, safe='')+':'+quote(password, safe='')+'@'
                   if username or password else '')
    channel = int(params.get('channel', 1))
    stream = int(environment.get('ROBOT_VIDEO_STREAM', int(params.get('stream_type', 0))+1))
    if channel < 1 or not 1 <= stream <= 99:
        raise ValueError('海康视频通道配置无效')
    return f'rtsp://{credentials}{host}:554/Streaming/Channels/{channel*100+stream}'


def relay_config(project_dir, source):
    config = yaml.safe_load((project_dir/'deploy/mediamtx.yml').read_text())
    config.update(api=True, apiAddress='127.0.0.1:9997')
    if source:
        config.setdefault('paths', {})['camera_source'] = {
            'source': source, 'sourceOnDemand': True, 'rtspTransport': 'tcp'}
    return config


def redact(message):
    return re.sub(r'(rtsps?://)[^\s/]*@', r'\1[redacted]@', message)


def video_settings(environment):
    try:
        max_width = int(environment.get('ROBOT_VIDEO_MAX_WIDTH', '1280'))
        fps = int(environment.get('ROBOT_VIDEO_FPS', '15'))
    except (TypeError, ValueError):
        raise ValueError('ROBOT_VIDEO_MAX_WIDTH 和 ROBOT_VIDEO_FPS 必须是整数') from None
    if not 320 <= max_width <= 3840 or max_width % 2 or not 1 <= fps <= 30:
        raise ValueError('视频最大宽度须为 320～3840 的偶数，帧率须为 1～30')
    return max_width, fps


def remux_command(timeout_option):
    return ['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-nostdin',
            '-rtsp_transport', 'tcp', timeout_option, '5000000',
            '-allowed_media_types', 'video', '-i', 'rtsp://127.0.0.1:8554/camera_source', '-map', '0:v:0',
            '-c:v', 'copy', '-an', '-f', 'rtsp', '-rtsp_transport', 'tcp',
            'rtsp://127.0.0.1:8554/cam']


def transcode_command(timeout_option, max_width=1280, fps=15):
    # H.265 and other browser-incompatible sources are always output as H.264
    # without B-frames; the local relay keeps credentials out of FFmpeg argv.
    return ['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-nostdin',
            '-rtsp_transport', 'tcp', timeout_option, '5000000', '-threads', '2',
            '-allowed_media_types', 'video', '-i', 'rtsp://127.0.0.1:8554/camera_source', '-map', '0:v:0',
            '-filter_threads', '1', '-vf', f"scale='min({max_width},iw)':-2,fps={fps}",
            '-c:v', 'libx264', '-threads', '2', '-preset', 'ultrafast',
            '-tune', 'zerolatency', '-profile:v', 'baseline', '-pix_fmt', 'yuv420p',
            '-bf', '0', '-g', str(fps*2), '-keyint_min', str(fps*2), '-sc_threshold', '0',
            '-b:v', '1500k', '-maxrate', '1500k', '-bufsize', '1500k', '-an',
            '-f', 'rtsp', '-rtsp_transport', 'tcp', 'rtsp://127.0.0.1:8554/cam']


def probe_video_codec(timeout_option):
    command = ['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp', timeout_option, '5000000',
               '-allowed_media_types', 'video', '-select_streams', 'v:0',
               '-show_entries', 'stream=codec_name', '-of', 'default=noprint_wrappers=1:nokey=1',
               'rtsp://127.0.0.1:8554/camera_source']
    result = subprocess.run(command, capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise ValueError(redact(result.stderr.strip())[:500] or 'ffprobe 无法读取视频')
    codec = result.stdout.strip().lower()
    if not re.fullmatch(r'[a-z0-9_]+', codec):
        raise ValueError('ffprobe 未返回有效视频编码')
    return codec


def command_for_codec(timeout_option, codec, max_width=1280, fps=15):
    if codec == 'h264':
        return remux_command(timeout_option)
    return transcode_command(timeout_option, max_width, fps)


def listening(port):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=.2):
            return True
    except OSError:
        return False


def main():
    project_dir = Path(__file__).resolve().parents[1]
    stopped = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.set())
    processes = []

    def launch(command, label):
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        processes.append(process)
        def output():
            for line in process.stdout:
                print(f'[{label}] {redact(line.rstrip())}', flush=True)
        threading.Thread(target=output, daemon=True).start()
        return process

    try:
        source = camera_source(project_dir, os.environ)
        if listening(8554) or listening(8889):
            raise ValueError('视频端口已被占用，请复用或先停止原视频转发服务')
        with tempfile.TemporaryDirectory(prefix='robot-web-video-') as directory:
            config_path = Path(directory)/'mediamtx.yml'
            config_path.write_text(yaml.safe_dump(relay_config(project_dir, source)))
            config_path.chmod(0o600)
            media = launch([str(project_dir/'bin/mediamtx'), str(config_path)], '网页视频')
            until = time.monotonic()+5
            while not listening(8554):
                if stopped.wait(.1):
                    return 0
                if media.poll() is not None or time.monotonic()>=until:
                    raise ValueError('网页视频服务启动失败，请查看视频日志')
            if source:
                max_width, fps = video_settings(os.environ)
                help_result = subprocess.run(['ffmpeg', '-hide_banner', '-h', 'demuxer=rtsp'],
                                             capture_output=True, text=True, timeout=5)
                timeout_option = '-stimeout' if '-stimeout ' in help_result.stdout else '-timeout'
            else:
                command = ['bash', str(project_dir/'scripts/push_camera.sh')]
            encoder = None
            while not stopped.is_set() and media.poll() is None:
                if encoder is None or encoder.poll() is not None:
                    if encoder is not None:
                        processes.remove(encoder)
                        print('相机视频中断，正在重新连接', flush=True)
                        if stopped.wait(2):
                            break
                    label = '视频转码'
                    if source:
                        try:
                            codec = probe_video_codec(timeout_option)
                            command = command_for_codec(timeout_option, codec, max_width, fps)
                            if codec == 'h264':
                                label = '视频转发'
                            print(f'网页视频源编码：{codec}；'+('无损转发' if codec == 'h264' else '兼容转码'), flush=True)
                        except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                            command = transcode_command(timeout_option, max_width, fps)
                            print('视频编码探测失败，改用软件转码：'+redact(str(error)), flush=True)
                    encoder = launch(command, label)
                stopped.wait(1)
            return 0 if stopped.is_set() else 1
    except yaml.YAMLError:
        print('网页视频配置格式错误，请检查相机和视频 YAML 文件', flush=True)
        return 1
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        print('网页视频启动失败：'+redact(str(error)), flush=True)
        return 1
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                with suppress(OSError):
                    process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    with suppress(OSError):
                        process.kill()
                    process.wait()


if __name__ == '__main__':
    raise SystemExit(main())
