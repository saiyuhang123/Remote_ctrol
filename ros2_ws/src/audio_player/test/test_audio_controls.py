from io import BytesIO
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace as NS

from std_msgs.msg import String
from audio_player.tts_node import TtsNode, PlaybackStopped


def player(tmp_path):
    node=object.__new__(TtsNode)
    node._audio_dir=tmp_path.resolve()
    node._audio_lock=threading.RLock();node._generation=0;node._play_process=None
    node._shutdown=threading.Event();node._q=queue.Queue(maxsize=20)
    node._audio_state={'ready':True,'state':'idle'}
    node._pub_audio=NS(publish=lambda message:None)
    return node


def test_uploaded_audio_is_queued_but_path_escape_is_rejected(tmp_path):
    node=player(tmp_path)
    (tmp_path/'hello.wav').write_bytes(b'fake audio')
    node._on_audio_play(String(data='{"filename":"hello.wav","request_id":"web1"}'))
    item=node._q.get_nowait()
    assert item[0]=='FILE' and item[2]=='web1' and node._audio_state['state']=='queued'
    node._on_audio_play(String(data='../hello.wav'))
    assert node._q.empty() and node._audio_state['state']=='error'


def test_stop_terminates_playback_and_clears_pending_files(tmp_path,monkeypatch):
    node=player(tmp_path)
    started=threading.Event()
    class Process:
        def __init__(self,*args,**kwargs):
            self.returncode=None;self.stderr=BytesIO();started.set()
        def poll(self):return self.returncode
        def terminate(self):self.returncode=-15
        def wait(self,timeout):return self.returncode
        def communicate(self):return b'',b''
    monkeypatch.setattr('audio_player.tts_node.subprocess.Popen',Process)
    node._q.put(('FILE','pending.wav','queued',0))
    outcomes=[]
    def play():
        try:node._play('fake.wav',0)
        except PlaybackStopped:outcomes.append('stopped')
    worker=threading.Thread(target=play);worker.start()
    assert started.wait(1.)
    node._on_audio_stop(String(data='stop1'))
    worker.join(.5)
    assert not worker.is_alive() and outcomes==['stopped']
    assert node._q.empty() and node._audio_state['state']=='stopped'
