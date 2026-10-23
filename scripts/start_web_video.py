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


def transcode_command(timeout_option):
    # The camera may provide H.265. Browser output is always H.264 without
    # B-frames; FFmpeg reads a local relay so credentials never enter its argv.
    return ['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-nostdin',
            '-rtsp_transport', 'tcp', timeout_option, '5000000', '-threads', '2',
            '-allowed_media_types', 'video', '-i', 'rtsp://127.0.0.1:8554/camera_source', '-map', '0:v:0',
            '-filter_threads', '1', '-vf', "scale='min(1280,iw)':-2,fps=15",
            '-c:v', 'libx264', '-threads', '2', '-preset', 'ultrafast',
            '-tune', 'zerolatency', '-profile:v', 'baseline', '-pix_fmt', 'yuv420p',
            '-bf', '0', '-g', '30', '-keyint_min', '30', '-sc_threshold', '0',
            '-b:v', '1500k', '-maxrate', '1500k', '-bufsize', '1500k', '-an',
            '-f', 'rtsp', '-rtsp_transport', 'tcp', 'rtsp://127.0.0.1:8554/cam']


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
                help_result = subprocess.run(['ffmpeg', '-hide_banner', '-h', 'demuxer=rtsp'],
                                             capture_output=True, text=True, timeout=5)
                timeout_option = '-stimeout' if '-stimeout ' in help_result.stdout else '-timeout'
                command = transcode_command(timeout_option)
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
                    encoder = launch(command, '视频转码')
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
