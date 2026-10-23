"""Video source selection and browser codec settings, without camera access."""
from pathlib import Path
from urllib.parse import unquote, urlsplit

import pytest
import yaml

from scripts.start_web_video import camera_source, relay_config, redact, transcode_command


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
