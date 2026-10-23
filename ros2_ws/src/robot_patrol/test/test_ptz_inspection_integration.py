"""Actual ROS clients and PTZ node, with fake camera services in domain 229 only."""
import importlib.util
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from hk_camera.srv import ControlPTZ, PTZPreset, CapturePicture
from ocr_interfaces.srv import RecognizeImage
from patrol_interfaces.srv import PtzInspectionControl
from robot_patrol.engine import PatrolEngine
from robot_patrol.inspection_client import InspectionClient
from test_engine import FakeDriver, make_store, snapshot

pytestmark = pytest.mark.skipif(os.environ.get('ROBOT_PTZ_INTEGRATION') != '1', reason='isolated PTZ opt-in required')
if os.environ.get('ROBOT_PTZ_INTEGRATION') == '1':
    assert os.environ.get('ROS_DOMAIN_ID') == '229'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'

spec = importlib.util.spec_from_file_location('ptz_patrol_fixture', Path(__file__).resolve().parents[2]/'chassis_nav2/scripts/ptz_patrol.py')
ptz_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ptz_module)


def wait_for(predicate, timeout=4.):
    deadline = time.monotonic()+timeout
    while not predicate() and time.monotonic()<deadline:
        time.sleep(.02)
    assert predicate(), 'isolated ROS condition timed out'


@pytest.fixture
def rig(tmp_path):
    rclpy.init()
    h = NS(events=[], violations=[], recognition_delay=0.)
    h.stop_gate, h.recognition_gate = threading.Event(), threading.Event()
    h.stop_gate.set(); h.recognition_gate.set()
    h.stop_requested, h.recognizing = threading.Event(), threading.Event()
    h.ptz = ptz_module.PtzPatrol()
    server, client = Node('fake_ptz_camera'), Node('ptz_inspection_test_client')
    group = ReentrantCallbackGroup()

    def control(request, response):
        if request.stop:
            h.stop_requested.set()
            h.stop_gate.wait(5.)
        h.events.append(('stop_ack' if request.stop else 'jog_ack', None))
        response.success = True
        return response

    def preset(request, response):
        if not h.ptz._inspection_owner or not h.ptz._inspection_stopped():
            h.violations.append('preset sent before sweep stop ACK')
        h.events.append(('preset', request.preset_no))
        response.success = True
        return response

    def capture(request, response):
        if not h.ptz._inspection_owner or not h.ptz._inspection_stopped():
            h.violations.append('capture competing with sweep')
        h.events.append(('capture', None))
        Path(request.save_path).write_bytes(b'fake jpg')
        response.success = True
        return response

    def recognize(request, response):
        h.recognizing.set()
        h.recognition_gate.wait(5.)
        delay, h.recognition_delay = h.recognition_delay, 0.
        if delay:
            time.sleep(delay)
        if not h.ptz._inspection_owner:
            h.events.append(('late_recognition', None))
        h.events.append(('recognize', None))
        response.success, response.text, response.model = True, '220 V', 'fixture'
        return response

    for service, name, callback in [(ControlPTZ,'/hk_camera/control_ptz',control),
            (PTZPreset,'/hk_camera/preset',preset), (CapturePicture,'/hk_camera/capture',capture),
            (RecognizeImage,'/vision/recognize_image',recognize)]:
        server.create_service(service,name,callback,callback_group=group)
    for name,value in dict(inspection_image_dir=str(tmp_path/'images'),inspection_service_timeout_sec=2.,
            inspection_capture_timeout_sec=2.,inspection_recognition_timeout_sec=10.,inspection_preset_settle_sec=.03,
            inspection_preset_service='/hk_camera/preset',inspection_capture_service='/hk_camera/capture',
            inspection_recognition_service='/vision/recognize_image',inspection_default_prompt='读取仪表').items():
        client.declare_parameter(name,value)
    h.store,h.mapping = make_store(tmp_path)
    h.inspection = InspectionClient(client,h.store,callback_group=group)
    h.protocol = client.create_client(PtzInspectionControl,'/ptz_patrol/inspection_control',callback_group=group)
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (h.ptz,server,client):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    assert h.ptz._ctrl.wait_for_service(timeout_sec=3.)
    assert h.inspection.preset.wait_for_service(timeout_sec=3.)
    assert h.protocol.wait_for_service(timeout_sec=3.)

    def enabled(value):
        assert h.ptz.set_parameters([Parameter('enabled',value=value)])[0].successful
        if value:
            wait_for(lambda:h.ptz._moving)
    h.enabled = enabled

    def command(kind,owner='protocol',resume=False,lease=3.):
        request = PtzInspectionControl.Request(command=kind,owner_id=owner,resume=resume,lease_seconds=lease)
        future = h.protocol.call_async(request)
        wait_for(future.done)
        return future.result()
    h.command = command

    def engine(points=('A',), dwell=.1):
        route = snapshot(points,dwell=dwell)
        route.map_id = h.mapping.map_id
        for i,step in enumerate(route.steps):
            step.inspection_enabled,step.ptz_preset = True,7+i
        h.store.create_run('run1','route',h.mapping.map_id,1,1,route.as_dict(),1,1,1)
        class Driver(FakeDriver):
            def navigate(self,step,flags,status_cb):
                if h.ptz._inspection_owner:
                    h.violations.append('hold survived next departure')
                h.events.append(('navigate',step.point_id))
                # Give the PTZ timer a chance to jog between inspection points.
                time.sleep(.25)
                return super().navigate(step,flags,status_cb)
        def status(value):
            if value.get('state')==5:
                if not h.ptz._inspection_owner or not h.ptz._inspection_stopped():
                    h.violations.append('sweep resumed during dwell')
                h.events.append(('dwell',value.get('current_point_name')))
        return PatrolEngine(h.store,route,'run1',1,'owner',Driver(),status,min_dispatch_interval=0.,
            inspection_cb=h.inspection.run,inspection_depart_cb=h.inspection.depart,
            inspection_finish_cb=h.inspection.finish)
    h.engine = engine
    try:
        yield h
    finally:
        h.stop_gate.set();h.recognition_gate.set()
        h.inspection.finish(False)
        executor.shutdown(timeout_sec=7.)
        thread.join(3.)
        for node in (client,server,h.ptz):
            node.destroy_node()
        rclpy.try_shutdown()
        h.store.close()


def test_two_point_stop_ack_long_recognition_dwell_and_restore(rig):
    h = rig
    h.enabled(True)
    h.stop_gate.clear()
    h.recognition_delay = 3.3  # Longer than the inspection lease; requires renewals.
    engine,outcomes = h.engine(('A','B')),[]
    worker = threading.Thread(target=lambda:outcomes.append(engine.run()))
    worker.start()
    try:
        assert h.stop_requested.wait(3.)
        time.sleep(.1)
        assert not any(event[0] in ('preset','capture') for event in h.events)
        h.stop_gate.set()
        worker.join(10.)
        assert not worker.is_alive()
        assert outcomes[0].success, outcomes[0]
        wait_for(lambda:not h.ptz._inspection_owner and h.ptz._moving)
        assert h.ptz.get_parameter('enabled').value is True
        assert not h.violations and not h.ptz._inspection_fault
        names = [event[0] for event in h.events]
        assert names.index('stop_ack') < names.index('preset') < names.index('capture') < names.index('recognize')
        next_nav = h.events.index(('navigate','B'))
        second_preset = h.events.index(('preset',8))
        assert ('jog_ack',None) in h.events[next_nav:second_preset]
        assert len(h.store.list_inspection_results(h.mapping.map_id,'run1')) == 2
    finally:
        h.stop_gate.set()
        engine.request_cancel()
        worker.join(10.)


@pytest.mark.parametrize('initially_enabled,manual_stop', [(False,False),(True,True)])
def test_completion_preserves_original_off_and_operator_stop(rig, initially_enabled, manual_stop):
    h = rig
    if initially_enabled:
        h.enabled(True)
    h.recognition_gate.clear()
    engine,outcomes = h.engine(),[]
    worker = threading.Thread(target=lambda:outcomes.append(engine.run()))
    worker.start()
    try:
        assert h.recognizing.wait(3.)
        if manual_stop:
            h.enabled(False)
        h.recognition_gate.set()
        worker.join(5.)
        assert not worker.is_alive() and outcomes[0].success
        wait_for(lambda:not h.ptz._inspection_owner)
        time.sleep(.3)
        assert not h.ptz.get_parameter('enabled').value and not h.ptz._moving
        assert not h.violations
    finally:
        h.recognition_gate.set();engine.request_cancel();worker.join(5.)


def test_cancel_during_recognition_returns_promptly_and_keeps_sweep_off(rig):
    h = rig
    h.enabled(True)
    h.recognition_gate.clear()
    engine,outcomes = h.engine(('A','B')),[]
    worker = threading.Thread(target=lambda:outcomes.append(engine.run()))
    worker.start()
    try:
        assert h.recognizing.wait(3.)
        started = time.monotonic()
        engine.request_cancel()
        worker.join(.6)
        assert not worker.is_alive() and time.monotonic()-started<.6
        assert not outcomes[0].success
        wait_for(lambda:not h.ptz._inspection_owner and not h.ptz.get_parameter('enabled').value)
        h.recognition_gate.set()
        time.sleep(.3)
        assert not h.ptz._moving and ('navigate','B') not in h.events
        assert sum(event[0]=='capture' for event in h.events)==1
        results = h.store.list_inspection_results(h.mapping.map_id,'run1')
        assert len(results)==1 and not results[0].success
    finally:
        h.recognition_gate.set();engine.request_cancel();worker.join(5.)


def test_lost_lease_and_late_acquire_never_restart_sweep(rig):
    h = rig
    h.enabled(True)
    assert h.command(1,lease=1.).success
    wait_for(lambda:bool(h.ptz._inspection_fault),timeout=3.)
    assert not h.ptz.get_parameter('enabled').value
    assert not h.command(1).success  # Expired owner cannot reacquire.
    assert h.command(3,resume=True).success
    time.sleep(.3)
    assert not h.ptz._moving and not h.ptz.get_parameter('enabled').value
    assert h.command(3,owner='late',resume=False).success
    assert not h.command(1,owner='late').success  # Cancel overtook acquire.
