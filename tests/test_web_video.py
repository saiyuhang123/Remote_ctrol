"""Video source selection and browser codec settings, without camera access."""
from pathlib import Path
from types import SimpleNamespace as NS
from urllib.parse import unquote, urlsplit

import pytest
import yaml

from scripts.start_web_video import (
    camera_source, command_for_codec, probe_video_codec, relay_config, redact,
    transcode_command, video_settings,
)


def test_default_video_uses_camera_config_and_escapes_credentials(tmp_path):
    config = tmp_path/'camera.yaml'
    config.write_text(yaml.safe_dump({'hk_camera_node':{'ros__parameters':dict(
        ip='198.51.100.7',username='fixture@viewer',password='fixture:p@ss',channel=2,stream_type=1)}}))
    source = camera_source(tmp_path, {'ROBOT_CAMERA_CONFIG':str(config)})
    address = urlsplit(source)
    assert address.hostname == '198.51.100.7'
    assert unquote(address.username) == 'fixture@viewer'
    assert unquote(address.password) == 'fixture:p@ss'
    assert address.path == '/Streaming/Channels/202'
    assert camera_source(tmp_path, {'ROBOT_VIDEO_INPUT':'uvc'}) is None
    assert camera_source(tmp_path, {}) is None
    with pytest.raises(ValueError):
        camera_source(tmp_path, {'ROBOT_VIDEO_INPUT':'hikvision'})


def test_rtsp_override_and_private_relay_avoid_credentials_in_encoder_argv():
    source = 'rtsp://fixture:private%40password@198.51.100.9/video'
    project = Path(__file__).resolve().parents[1]
    assert camera_source(project, {'CAMERA_RTSP':source}) == source
    config = relay_config(project,source)
    assert config['paths']['camera_source']['source'] == source
    assert config['apiAddress'] == '127.0.0.1:9997'
    assert config['rtspTransports'] == ['tcp']
    assert config['webrtcLocalTCPAddress'] == ':8189'
    command = transcode_command('-stimeout')
    assert source not in ' '.join(command)
    assert command[command.index('-c:v')+1] == 'libx264'
    assert command[command.index('-bf')+1] == '0'
    assert '-an' in command
    assert command[-1] == 'rtsp://127.0.0.1:8554/cam'
    message = redact('failed reading '+source)
    assert 'private' not in message and 'fixture:' not in message


def test_invalid_video_override_is_rejected_without_exposing_value(tmp_path):
    with pytest.raises(ValueError,match='RTSP 视频地址'):
        camera_source(tmp_path, {'CAMERA_RTSP':'https://private.invalid/fixture'})


def test_h264_is_remuxed_and_other_codecs_keep_compatible_transcode():
    command = command_for_codec('-stimeout', 'h264')
    assert command[command.index('-c:v')+1] == 'copy'
    assert '-vf' not in command
    assert '-threads' not in command
    assert '-an' in command
    fallback = command_for_codec('-stimeout', 'hevc')
    assert fallback[fallback.index('-c:v')+1] == 'libx264'
    assert fallback[fallback.index('-bf')+1] == '0'


def test_probe_uses_private_loopback_and_rejects_invalid_output(monkeypatch):
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return NS(returncode=0, stdout='h264\n', stderr='')
    monkeypatch.setattr('scripts.start_web_video.subprocess.run', run)
    assert probe_video_codec('-stimeout') == 'h264'
    assert calls[0][-1] == 'rtsp://127.0.0.1:8554/camera_source'
    monkeypatch.setattr('scripts.start_web_video.subprocess.run',
                        lambda *args, **kwargs: NS(returncode=0, stdout='not a codec', stderr=''))
    with pytest.raises(ValueError, match='未返回有效视频编码'):
        probe_video_codec('-stimeout')


def test_probe_error_is_redacted(monkeypatch):
    monkeypatch.setattr('scripts.start_web_video.subprocess.run',
                        lambda *args, **kwargs: NS(returncode=1, stdout='',
                            stderr='failed rtsp://fixture:private@198.51.100.9/video'))
    with pytest.raises(ValueError) as error:
        probe_video_codec('-stimeout')
    assert 'private' not in str(error.value)


def test_transcode_dimensions_and_rate_are_configurable():
    assert video_settings({}) == (1280, 15)
    assert video_settings({'ROBOT_VIDEO_MAX_WIDTH':'960', 'ROBOT_VIDEO_FPS':'10'}) == (960, 10)
    command = transcode_command('-stimeout', 960, 10)
    assert "scale='min(960,iw)':-2,fps=10" in command
    assert command[command.index('-g')+1] == '20'
    for environment in ({'ROBOT_VIDEO_MAX_WIDTH':'961'}, {'ROBOT_VIDEO_MAX_WIDTH':'319'},
                        {'ROBOT_VIDEO_FPS':'0'}, {'ROBOT_VIDEO_FPS':'31'}, {'ROBOT_VIDEO_FPS':'bad'}):
        with pytest.raises(ValueError):
            video_settings(environment)
