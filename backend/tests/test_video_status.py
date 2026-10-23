from contextlib import nullcontext
from io import BytesIO
import json
from urllib.error import HTTPError

import pytest
from fastapi import FastAPI

from backend import api_video
from asgi_client import ASGIClient


@pytest.mark.parametrize('mode,available,ready', [
    ('stopped',False,False),('no_stream',True,False),('waiting',True,False),
    ('ready',True,True),('external',True,None)])
def test_video_readiness_distinguishes_missing_service_from_missing_frames(monkeypatch, mode, available, ready):
    def connection(*args, **kwargs):
        if mode=='stopped':
            raise ConnectionRefusedError()
        return nullcontext()
    def response(*args, **kwargs):
        if mode=='no_stream':
            raise HTTPError('http://localhost',404,'not found',None,None)
        if mode=='external':
            raise ConnectionRefusedError()
        return BytesIO(json.dumps(dict(ready=mode=='ready',tracks=['H264'] if mode=='ready' else [])).encode())
    monkeypatch.setattr(api_video.socket,'create_connection',connection)
    monkeypatch.setattr(api_video,'urlopen',response)
    app = FastAPI();app.include_router(api_video.router)
    result = ASGIClient(app).get('/api/video/status')
    assert result.status_code == 200 and result.headers['cache-control']=='no-store'
    value = result.json()
    assert value['available'] is available and value['ready'] is ready
    assert value['message']
