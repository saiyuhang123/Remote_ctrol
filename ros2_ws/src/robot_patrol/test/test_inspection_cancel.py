"""Cancellation never waits for a device or consumes its late service result."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time

import pytest

from robot_patrol.engine import ControlFlags, PatrolEngine, STATE_CANCELED, STATE_INTERRUPTED
from robot_patrol.inspection_client import InspectionClient
from robot_patrol.models import PatrolError
from test_engine import make_store, snapshot, FakeDriver


class Service:
    def __init__(self, response=None, pending=False):
        self.calls = []
        self.called = threading.Event()
        self.future = Future()
        self.removed = []
        self.response = response or NS(success=True, message='OK')
        self.pending = pending

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.calls.append(request)
        self.called.set()
        if not self.pending:
            self.future.set_result(self.response)
        return self.future

    def remove_pending_request(self, future):
        self.removed.append(future)


def fixture(tmp_path, pending='', settle=0.):
    client = object.__new__(InspectionClient)
    store, mapping = make_store(tmp_path)
    store.create_run('run1', 'route', mapping.map_id, 1, 1, {'steps': []}, 1, 1, 1)
    client.store = store
    client.node = NS(get_logger=lambda: NS(error=lambda message: None))
    for name in ('_PTZPreset', '_CapturePicture', '_RecognizeImage'):
        setattr(client, name, NS(Request=NS))
    client.preset = Service(pending=pending=='preset')
    client.capture = Service(NS(success=True, message='OK', image=NS()), pending=pending=='capture')
    client.recognize = Service(NS(success=True, message='OK', text='220 V', model='fake'), pending=pending=='recognize')
    client.image_dir = tmp_path/'images'
    client.service_timeout = client.capture_timeout = client.recognition_timeout = 150.
    client.preset_settle_sec = settle
    client.default_prompt = '读取仪表'
    return client, store, mapping


@pytest.mark.parametrize('pending', ['preset', 'capture', 'recognize'])
def test_cancel_pending_service_returns_promptly_and_retires_future(tmp_path, pending):
    client, store, mapping = fixture(tmp_path, pending)
    flags, outcomes = ControlFlags(), []
    step = NS(point_id='p', point_name='P', ptz_preset=1, recognition_prompt='')
    worker = threading.Thread(target=lambda: outcomes.append(client.run(step, 'run1', mapping.map_id, 1, 1, flags=flags)))
    worker.start()
    service = getattr(client, pending)
    assert service.called.wait(1.)
    started = time.monotonic()
    flags.request_cancel()
    worker.join(.6)
    assert not worker.is_alive(), 'cancel waited for the 150 second service deadline'
    assert time.monotonic()-started < .6
    assert outcomes[0].error_code == 'CANCELED'
    assert service.future.cancelled() and service.removed == [service.future]
    assert not store.list_inspection_results(mapping.map_id, 'run1')[0].success
    if pending=='preset':
        assert not client.capture.calls and not client.recognize.calls
    if pending=='capture':
        assert not client.recognize.calls
    store.close()


@pytest.mark.parametrize('operation', ['cancel', 'pause'])
def test_settle_wait_is_interruptible_and_does_not_capture(tmp_path, operation):
    client, store, mapping = fixture(tmp_path, settle=5.)
    flags, outcomes = ControlFlags(), []
    worker = threading.Thread(target=lambda: outcomes.append(client.run(
        NS(point_id='p', point_name='P', ptz_preset=1), 'run1', mapping.map_id, 1, 1, flags=flags)))
    worker.start()
    assert client.preset.called.wait(1.)
    getattr(flags, 'request_'+operation)()
    worker.join(.6)
    assert not worker.is_alive()
    assert outcomes[0].error_code == ('CANCELED' if operation=='cancel' else 'PAUSED')
    assert not client.capture.calls and not client.recognize.calls
    store.close()


@pytest.mark.parametrize('interrupt', [False, True])
def test_engine_keeps_cancel_or_interlock_terminal_state_during_inspection(tmp_path, interrupt):
    store, mapping = make_store(tmp_path)
    route = snapshot(['A', 'B'])
    route.map_id = mapping.map_id
    route.steps[0].inspection_enabled = True
    store.create_run('run1', 'route', mapping.map_id, 1, 1, route.as_dict(), 1, 1, 1)
    driver = FakeDriver()
    def inspection(*args, flags=None):
        flags.request_cancel('operator lost' if interrupt else '')
        return NS(success=False, error_code='CANCELED', message='取消')
    engine = PatrolEngine(store, route, 'run1', 1, 'owner', driver, lambda status: None,
                          min_dispatch_interval=0., inspection_cb=inspection)
    result = engine.run()
    assert not result.success and result.completed_steps == 0
    assert result.final_state == (STATE_INTERRUPTED if interrupt else STATE_CANCELED)
    assert driver.calls == ['A']
    store.close()


def test_pause_during_inspection_reconfirms_the_same_point_before_retry(tmp_path):
    store,mapping=make_store(tmp_path)
    route=snapshot(['A']);route.map_id=mapping.map_id;route.steps[0].inspection_enabled=True
    store.create_run('run1','route',mapping.map_id,1,1,route.as_dict(),1,1,1)
    driver,calls=FakeDriver(),[]
    def inspect(*args,flags=None):
        calls.append(args[0].point_id)
        if len(calls)==1:
            flags.request_pause()
            threading.Timer(.1,flags.request_resume).start()
            return NS(success=False,error_code='PAUSED',message='暂停')
        return NS(success=True,text='220 V')
    engine=PatrolEngine(store,route,'run1',1,'owner',driver,lambda status:None,
                       min_dispatch_interval=0.,inspection_cb=inspect)
    assert engine.run().success
    assert calls==['A','A'] and driver.calls==['A','A']
    store.close()


@pytest.mark.parametrize('operation', ['cancel', 'pause'])
def test_control_while_releasing_ptz_before_next_point_keeps_correct_state(tmp_path, operation):
    store, mapping = make_store(tmp_path)
    route = snapshot(['A','B'])
    route.map_id = mapping.map_id
    store.create_run('run1','route',mapping.map_id,1,1,route.as_dict(),1,1,1)
    driver, interrupted, finishes = FakeDriver(), [], []
    def depart(flags):
        if driver.calls == ['A'] and not interrupted:
            interrupted.append(True)
            getattr(flags,'request_'+operation)()
            if operation == 'pause':
                threading.Timer(.1,flags.request_resume).start()
            raise PatrolError(operation.upper(),operation)
    engine = PatrolEngine(store,route,'run1',1,'owner',driver,lambda status:None,
        min_dispatch_interval=0.,inspection_depart_cb=depart,
        inspection_finish_cb=lambda success:finishes.append(success))
    result = engine.run()
    assert result.success is (operation == 'pause')
    assert finishes == [operation == 'pause']
    assert driver.calls == (['A','B'] if operation == 'pause' else ['A'])
    if operation == 'cancel':
        assert result.final_state == STATE_CANCELED
    store.close()
