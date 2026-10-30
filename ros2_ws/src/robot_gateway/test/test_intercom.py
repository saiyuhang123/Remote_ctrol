"""Fake soundcards exercise bidirectional PCM and release on every exit."""
import asyncio
import json
from types import SimpleNamespace as NS

from robot_gateway.intercom import Intercom


class Audio:
    def __init__(self,fail_output=False):self.streams=[];self.fail_output=fail_output
    def query_devices(self,device=None,kind=None):
        values=[dict(name='USB microphone',max_input_channels=1,max_output_channels=0),dict(name='USB speaker',max_input_channels=0,max_output_channels=1)]
        return values if kind is None else values[0 if kind=='input' else 1]
    def check_input_settings(self,**kwargs):pass
    def check_output_settings(self,**kwargs):pass
    def make(self,kind,options):
        if kind=='output' and self.fail_output:raise RuntimeError('speaker disconnected')
        stream=NS(kind=kind,options=options,closed=False,aborted=False)
        def start():
            if kind=='input':options['callback'](b'\x01\x02'*960,960,None,None)
        stream.start=start
        stream.abort=lambda:setattr(stream,'aborted',True)
        stream.close=lambda:setattr(stream,'closed',True)
        self.streams.append(stream);return stream
    def RawInputStream(self,**kwargs):return self.make('input',kwargs)
    def RawOutputStream(self,**kwargs):return self.make('output',kwargs)


def fixture(audio=None):
    owned=[True];announcements=[]
    node=NS(_owner_valid=lambda token:owned[0] and token=='owner',_lease_valid=lambda:owned[0],
        _string_type=NS,_audio_stop_pub=NS(publish=lambda msg:announcements.append(msg.data)))
    return Intercom(node,audio or Audio()),owned,announcements


class Socket:
    def __init__(self,call,owned,lose=False):self.sent=[];self.closed=False;self.step=0;self.call=call;self.owned=owned;self.lose=lose
    async def recv(self):
        self.step+=1
        if self.step==1:return json.dumps(dict(session_id='owner'))
        if self.step==2:return json.dumps(dict(type='start',input_device=0,output_device=1))
        if self.step==3:return b'\x03\x04'*960
        output=bytearray(1920)
        self.call.audio_module.streams[1].options['callback'](output,960,None,None)
        assert output==b'\x03\x04'*960
        if self.lose:
            self.owned[0]=False
            raise asyncio.TimeoutError()
        return json.dumps(dict(type='stop'))
    async def send(self,data):self.sent.append(data)
    async def close(self):self.closed=True


def test_pcm_flows_both_ways_and_stop_releases_soundcard():
    call,owned,announcements=fixture();socket=Socket(call,owned)
    asyncio.run(call.handle(socket))
    assert b'\x01\x02'*960 in socket.sent
    assert any(isinstance(item,str) and json.loads(item)['type']=='ready' for item in socket.sent)
    assert socket.closed and not call.active and not call.reserved
    assert announcements==['intercom']
    assert all(stream.closed and stream.aborted for stream in call.audio_module.streams)


def test_owner_loss_ends_capture_and_output():
    call,owned,_=fixture();socket=Socket(call,owned,lose=True)
    asyncio.run(call.handle(socket))
    assert socket.closed and not call.reserved and all(stream.closed for stream in call.audio_module.streams)


def test_partial_device_open_failure_closes_the_microphone():
    call,owned,_=fixture(Audio(fail_output=True));socket=Socket(call,owned)
    asyncio.run(call.handle(socket))
    assert not call.reserved and not call.active and call.audio_module.streams[0].closed
    assert any(isinstance(item,str) and 'speaker disconnected' in item for item in socket.sent)


def test_unauthorized_call_never_opens_audio_devices():
    call,owned,_=fixture();owned[0]=False;socket=Socket(call,owned)
    asyncio.run(call.handle(socket))
    assert not call.audio_module.streams and socket.closed


def test_connected_cue_mutes_pcm_and_microphone_then_call_resumes(monkeypatch):
    now = [100.]
    monkeypatch.setattr('robot_gateway.intercom.time.monotonic', lambda: now[0])
    call, _, _ = fixture()
    state = dict(available=True, state='playing', notification='intercom_started')
    call.node._audio_snapshot = lambda: state
    source, sink = [], []
    try:
        call.open_stream(source, sink, 0, 1)
        assert call.outgoing.empty()
        call.incoming.put(b'\x03\x04'*960)
        output = bytearray(1920)
        sink[0].options['callback'](output, 960, None, None)
        assert output == bytes(1920) and call.incoming.empty()
        state['state'] = 'done'
        now[0] += .3
        source[0].options['callback'](b'\x01\x02'*960, 960, None, None)
        assert call.outgoing.get_nowait() == b'\x01\x02'*960
        call.incoming.put(b'\x03\x04'*960)
        sink[0].options['callback'](output, 960, None, None)
        assert output == b'\x03\x04'*960
    finally:
        call.close_streams(source+sink)
