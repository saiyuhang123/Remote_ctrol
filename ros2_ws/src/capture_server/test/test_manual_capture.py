import json
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
from std_msgs.msg import String
from capture_server.capture_server_node import CaptureServerNode


def fixture(tmp_path,age=0):
    messages=[];uploads=[]
    node=NS(_cap_dir=tmp_path,_lock=threading.Lock(),_latest_frame=np.zeros((16,16,3),dtype=np.uint8),
        _latest_time=time.time()-age,_recent={},_pub_done=NS(publish=lambda msg:messages.append(json.loads(msg.data))),
        _reporter=NS(post=lambda path,payload:uploads.append((path,payload))),get_logger=lambda:NS(info=lambda msg:None,warn=lambda msg:None))
    return node,messages,uploads


def test_capture_correlates_saves_once_and_cannot_escape_directory(tmp_path):
    node,messages,uploads=fixture(tmp_path)
    request=String(data=json.dumps(dict(request_id='web_1',label='../../escape')))
    CaptureServerNode._on_capture_request(node,request)
    CaptureServerNode._on_capture_request(node,request)
    assert len(list(tmp_path.rglob('*.jpg')))==1 and len(uploads)==1
    assert len(messages)==2 and messages[0]==messages[1]
    assert messages[0]['ok'] and messages[0]['request_id']=='web_1'
    assert messages[0]['image'].startswith('captures/') and '../' not in messages[0]['image']


def test_stale_video_and_disk_failure_are_reported(tmp_path,monkeypatch):
    node,messages,uploads=fixture(tmp_path,age=3)
    request=String(data='{"request_id":"web_2"}')
    CaptureServerNode._on_capture_request(node,request)
    assert not messages[-1]['ok'] and not uploads
    node._latest_time=time.time()
    monkeypatch.setattr('capture_server.capture_server_node.cv2.imwrite',lambda *args:False)
    CaptureServerNode._on_capture_request(node,request)
    assert not messages[-1]['ok'] and messages[-1]['request_id']=='web_2' and not uploads
