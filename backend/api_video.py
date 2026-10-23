"""Read-only video readiness; camera startup remains in the monitor launcher."""
import json
import socket
from urllib.request import urlopen
from urllib.error import HTTPError

from fastapi import APIRouter
from fastapi.responses import JSONResponse

router = APIRouter(tags=['video'])


def video_status():
    try:
        with socket.create_connection(('127.0.0.1', 8889), timeout=.3):
            pass
    except OSError:
        return dict(available=False, ready=False, message='视频服务未启动，请在机器人端启动网页视频服务')
    try:
        with urlopen('http://127.0.0.1:9997/v3/paths/get/cam', timeout=.7) as response:
            status = json.load(response)
        ready = bool(status.get('ready')) and bool(status.get('tracks'))
        return dict(available=True, ready=ready, message='视频已就绪' if ready else '正在等待相机画面，请检查相机连接')
    except HTTPError as error:
        if error.code == 404:
            return dict(available=True, ready=False, message='正在等待相机画面，请检查相机连接')
        return dict(available=True, ready=None, message='视频服务已启动')
    except (OSError, ValueError):
        # External MediaMTX instances may not expose the optional local API.
        return dict(available=True, ready=None, message='视频服务已启动')


@router.get('/api/video/status')
def status():
    return JSONResponse(video_status(), headers={'Cache-Control': 'no-store'})
