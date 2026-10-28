"""Real ROS clients, fake camera and TF provider; never connects to hardware."""
import json
import math
import os
import subprocess
import sys
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
from sensor_msgs.msg import Image
from hk_camera.srv import ControlPTZ, GetPTZPose, SetPTZPose, CapturePicture
from ocr_interfaces.srv import RecognizeImage
from patrol_interfaces.srv import PointShots, PtzInspectionControl
from robot_patrol.engine import ControlFlags, PatrolEngine
from robot_patrol.inspection_client import InspectionClient
from robot_patrol.inspection_prompts import DIGITAL_METER_PROMPT
from robot_patrol.models import PatrolError, RouteSnapshot
from robot_patrol.ptz_sequence import DEFAULTS
from robot_patrol.ptz_shots import shots_json
from test_engine import FakeDriver, make_store
from test_ptz_shots import shot

pytestmark = pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '232' or
    os.environ.get('ROS_LOCALHOST_ONLY') != '1', reason='requires isolated ROS domain 232')


def wait_for(predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), 'isolated shot sequence condition timed out'


@pytest.fixture
def rig(tmp_path):
    rclpy.init()
    h = NS(events=[], actual=dict(pan=100., tilt=0., zoom=1.), pose=[0., 0., math.radians(2.)],
        motion_pose=[0., 0., 0.],
        apply_set=True, apply_zoom=True, capture_failures=0, delay_set=False, set_gate=threading.Event(),
        setting=threading.Event(), owner='', renewals=0, moving=False, stop_ok=True,
        capture_count=0, empty_capture_image=False, vision_ok=True, vision_text=None,
        read_failures_after_capture=0, failed_post_capture_reads=0,
        read_error='相机镜头 SDK 服务响应超时',
        delay_recognition=False, recognizing=threading.Event(), recognition_gate=threading.Event(),
        recognition_finished=threading.Event(),
        block_late_stop=False, stop_gate=threading.Event(), late_stopping=threading.Event(), extra_nodes=[])
    h.set_gate.set()
    h.stop_gate.set()
    h.recognition_gate.set()
    server, node = Node('fake_shot_devices'), Node('shot_client_fixture')
    group = ReentrantCallbackGroup()
    def get(request, response):
        if h.capture_count and h.read_failures_after_capture:
            h.read_failures_after_capture -= 1
            h.failed_post_capture_reads += 1
            response.message = h.read_error
            if getattr(h, 'on_failed_read', None):
                h.on_failed_read()
            return response
        response.success = True
        for name, value in h.actual.items():
            setattr(response, name, value)
        if getattr(h, 'on_get', None):
            h.on_get()
        return response
    def set_pose(request, response):
        h.setting.set()
        if h.delay_set:
            assert h.set_gate.wait(3.)
        values = dict(pan=float(request.pan), tilt=float(request.tilt), zoom=float(request.zoom))
        h.events.append(('set', values))
        if h.apply_set:
            if not h.apply_zoom:
                values['zoom']=h.actual['zoom']
            h.actual = values
            h.moving = True
        response.success = True
        return response
    def stop(request, response):
        assert request.stop
        h.events.append(('stop', None))
        if h.block_late_stop and sum(kind=='stop' for kind,_ in h.events)>1:
            h.late_stopping.set()
            assert h.stop_gate.wait(3.)
        if h.stop_ok:
            h.moving = False
        response.success = h.stop_ok
        return response
    def capture(request, response):
        assert h.owner, 'capture without sweep hold'
        h.events.append(('capture', dict(h.actual)))
        if h.capture_failures:
            h.capture_failures -= 1
            response.message = 'fake capture failed'
            return response
        Path(request.save_path).write_bytes(b'fake jpeg')
        response.success = True
        h.capture_count += 1
        if not h.empty_capture_image:
            response.image = Image(height=1, width=1, encoding='bgr8', step=3,
                                   data=bytes([h.capture_count, 0, 0]))
        if getattr(h, 'on_capture', None):
            h.on_capture()
        return response
    def recognize(request, response):
        assert h.owner, 'recognition without sweep hold'
        h.events.append(('recognize', dict(prompt=request.prompt, image=bytes(request.image.data))))
        h.recognizing.set()
        if h.delay_recognition:
            assert h.recognition_gate.wait(3.)
        response.success = h.vision_ok
        response.text = h.vision_text if h.vision_text is not None else f'仪表读数：{request.image.data[0]}.20 V'
        response.model = 'deepseek-fake'
        response.message = 'OK' if h.vision_ok else 'fake network failure'
        response.processing_time_ms = 12.5
        h.recognition_finished.set()
        return response
    def inspection(request, response):
        if request.command == 3:
            h.owner = ''
            h.events.append(('release', request.resume))
        else:
            h.owner = request.owner_id
            h.renewals += int(request.command == 2)
        response.success, response.stopped = True, True
        return response
    for kind, name, callback in [
        (GetPTZPose,'/hk_camera/get_ptz_pose',get), (SetPTZPose,'/hk_camera/set_ptz_pose',set_pose),
        (ControlPTZ,'/hk_camera/control_ptz',stop), (CapturePicture,'/hk_camera/capture',capture),
        (RecognizeImage,'/vision/recognize_image',recognize),
        (PtzInspectionControl,'/ptz_patrol/inspection_control',inspection)]:
        server.create_service(kind, name, callback, callback_group=group)
    for key, value in dict(inspection_image_dir=str(tmp_path/'images'),inspection_service_timeout_sec=1.,
        inspection_capture_timeout_sec=1.,inspection_recognition_timeout_sec=1.,inspection_preset_settle_sec=0.,
        inspection_preset_service='/hk_camera/preset',inspection_capture_service='/hk_camera/capture',
        inspection_recognition_service='/vision/recognize_image',inspection_default_prompt=DIGITAL_METER_PROMPT).items():
        node.declare_parameter(key, value)
    for name, value in DEFAULTS.items():
        node.declare_parameter(name, {'arrival_timeout_sec':.4,'poll_sec':.02,'stable_sec':.03,
            'stop_timeout_sec':.4}.get(name.removeprefix('inspection_shot_'),value))
    h.store, h.mapping = make_store(tmp_path)
    h.point = h.store.upsert_point('', h.mapping.map_id, 'P', 0., 0., 0., 'manual', 1)
    h.point = h.store.replace_point_shots(h.point.point_id, h.mapping.map_id, 1, h.point.revision,
                                        [shot(), shot('side', pan_cal=85., tilt=-5., zoom=3.)])
    h.store.create_run('run1', 'route', h.mapping.map_id, 1, 1, {'steps':[]}, 1, 1, 1)
    h.step = NS(point_id=h.point.point_id, point_name='P', shots=h.point.shots,
        inspection_enabled=True, ptz_preset=0, x=0., y=0., yaw=0., dwell_seconds=0.)
    h.inspection = InspectionClient(node,h.store,group,pose_provider=lambda:tuple(h.pose),
                                   motion_pose_provider=lambda:tuple(h.motion_pose))
    h.executor = MultiThreadedExecutor(num_threads=6)
    for member in (node,server):
        h.executor.add_node(member)
    worker = threading.Thread(target=h.executor.spin,daemon=True)
    worker.start()
    for client in (h.inspection.sequence.get,h.inspection.sequence.set,h.inspection.sequence.stop,
                   h.inspection.capture,h.inspection.recognize,h.inspection.ptz_lease.client):
        assert client.wait_for_service(timeout_sec=3.)
    h.run = lambda flags=None: h.inspection.run(h.step,'run1',h.mapping.map_id,1,1,flags=flags)
    h.node = node
    try:
        yield h
    finally:
        h.set_gate.set()
        h.stop_gate.set()
        h.recognition_gate.set()
        h.inspection.finish(False)
        h.executor.shutdown(timeout_sec=4.)
        worker.join(4.)
        for member in (*h.extra_nodes,node,server):
            member.destroy_node()
        rclpy.try_shutdown()
        h.store.close()


def test_temporary_post_capture_sdk_timeout_recovers_before_recognition(rig):
    rig.read_failures_after_capture = 2
    result = rig.run()
    assert result.success
    assert rig.failed_post_capture_reads == 2
    assert rig.capture_count == 2
    assert len([value for kind, value in rig.events if kind == 'recognize']) == 2
    assert all(result.success for result in rig.store.list_inspection_results(rig.mapping.map_id, 'run1'))


def test_persistent_post_capture_sdk_timeout_keeps_photo_and_skips_recognition(rig):
    rig.read_failures_after_capture = 100
    result = rig.run()
    assert not result.success and result.error_code == 'INSPECTION_PTZ_READ_FAILED'
    assert rig.failed_post_capture_reads == 3
    assert rig.capture_count == 1
    assert Path(result.image_path).is_file()
    assert not any(kind == 'recognize' for kind, _ in rig.events)


def test_non_timeout_position_errors_are_not_retried(rig):
    rig.read_failures_after_capture = 100
    rig.read_error = '设备返回无效 zoom=0'
    result = rig.run()
    assert not result.success and rig.failed_post_capture_reads == 1
    assert not any(kind == 'recognize' for kind, _ in rig.events)


def test_recovered_read_must_match_the_target_instead_of_reusing_the_old_pose(rig):
    rig.read_failures_after_capture = 1
    rig.on_failed_read = lambda: rig.actual.update(zoom=4.)
    result = rig.run()
    assert not result.success and result.error_code == 'INSPECTION_PTZ_NOT_ARRIVED'
    assert not any(kind == 'recognize' for kind, _ in rig.events)


def test_cancel_during_sdk_read_retry_stops_without_recognition(rig):
    flags = ControlFlags()
    rig.read_failures_after_capture = 100
    rig.on_failed_read = lambda: flags.request_cancel()
    result = rig.run(flags)
    assert not result.success and result.error_code == 'CANCELED'
    assert rig.failed_post_capture_reads == 1
    assert not any(kind == 'recognize' for kind, _ in rig.events)


def test_sdk_retry_obeys_the_original_read_deadline(rig):
    rig.capture_count = 1
    rig.read_failures_after_capture = 100
    rig.inspection.sequence.settings['poll_sec'] = .3
    begin = time.monotonic()
    with pytest.raises(PatrolError, match='读回重试超时'):
        rig.inspection.sequence._get(ControlFlags(), timeout=.12)
    assert time.monotonic()-begin < .35
    assert rig.failed_post_capture_reads == 1


def test_two_views_run_through_engine_with_recognition_for_each_captured_image(rig):
    h=rig
    snapshot=RouteSnapshot('route',h.mapping.map_id,'route',1,[h.step])
    engine=PatrolEngine(h.store,snapshot,'run1',1,'owner',FakeDriver(),lambda _:None,min_dispatch_interval=0.,
        inspection_cb=h.inspection.run,inspection_depart_cb=h.inspection.depart,inspection_finish_cb=h.inspection.finish)
    assert engine.run().success
    wait_for(lambda:not h.owner)
    captures=[pose for kind,pose in h.events if kind=='capture']
    assert len(captures)==2
    assert captures[0]==pytest.approx(dict(pan=1.,tilt=-8.,zoom=2.))
    assert captures[1]==pytest.approx(dict(pan=87.,tilt=-5.,zoom=3.))
    recognitions=[request for kind,request in h.events if kind=='recognize']
    assert [request['image'] for request in recognitions]==[b'\x01\x00\x00',b'\x02\x00\x00']
    assert all(request['prompt']==DIGITAL_METER_PROMPT for request in recognitions)
    assert [kind for kind,_ in h.events if kind in ('capture','recognize')]==[
        'capture','recognize','capture','recognize']
    results=h.store.list_inspection_results(h.mapping.map_id,'run1')
    assert {r.shot_id for r in results}=={'wide','side'} and all(r.success for r in results)
    for result in results:
        assert result.metadata['robot_pose']['yaw']==pytest.approx(math.radians(2.))
        assert Path(result.image_path).read_bytes()==b'fake jpeg'
        assert result.model=='deepseek-fake' and result.prompt==DIGITAL_METER_PROMPT
        assert result.text in ('仪表读数：1.20 V','仪表读数：2.20 V')
        assert result.metadata['text']==result.text
        assert result.metadata['recognition']['success']
        assert result.metadata['recognition']['processing_time_ms']==12.5
    assert len(list(h.inspection.image_dir.rglob('*.json')))==2


@pytest.mark.parametrize('prompt',['','   ','只读取面板上的电压和电流，保留原样数字'])
def test_saved_point_prompt_or_digital_meter_default_reaches_vision_service(rig,prompt):
    rig.step.shots=[rig.point.shots[0]]
    rig.step.recognition_prompt=prompt
    assert rig.run().success
    expected=prompt.strip() or DIGITAL_METER_PROMPT
    requests=[request for kind,request in rig.events if kind=='recognize']
    assert requests[0]['prompt']==expected
    record=rig.store.list_inspection_results(rig.mapping.map_id,'run1')[0]
    assert record.prompt==expected and record.metadata['prompt']==expected


@pytest.mark.parametrize('failure',['api_failure','empty_text','timeout','empty_image'])
def test_recognition_failures_preserve_photo_and_diagnostics_and_stop_remaining_views(rig,failure):
    if failure=='api_failure':
        rig.vision_ok=False
    elif failure=='empty_text':
        rig.vision_text='   '
    elif failure=='timeout':
        rig.delay_recognition=True
        rig.recognition_gate.clear()
        rig.inspection.recognition_timeout=.15
    else:
        rig.empty_capture_image=True
    result=rig.run()
    assert not result.success
    assert result.error_code=={'timeout':'INSPECTION_TIMEOUT','empty_image':'INSPECTION_CAPTURE_FAILED'}.get(
        failure,'INSPECTION_RECOGNITION_FAILED')
    assert Path(result.image_path).read_bytes()==b'fake jpeg'
    records=rig.store.list_inspection_results(rig.mapping.map_id,'run1')
    assert len(records)==1 and not records[0].success
    assert records[0].metadata['stop_confirmed']
    assert not records[0].text
    if failure=='empty_image':
        assert not any(kind=='recognize' for kind,_ in rig.events)
    else:
        diagnostic=records[0].metadata['recognition']
        assert not diagnostic['success'] and diagnostic['error_code']==result.error_code
        assert diagnostic['finished_at_ns']>=diagnostic['started_at_ns']
    assert sum(kind=='capture' for kind,_ in rig.events)==1


@pytest.mark.parametrize('operation',['cancel','pause'])
def test_pending_recognition_is_interruptible_and_late_text_cannot_overwrite_result(rig,operation):
    rig.delay_recognition=True
    rig.recognition_gate.clear()
    flags,results=ControlFlags(),[]
    worker=threading.Thread(target=lambda:results.append(rig.run(flags)))
    worker.start()
    assert rig.recognizing.wait(2.)
    getattr(flags,'request_'+operation)()
    worker.join(.8)
    assert not worker.is_alive()
    assert results[0].error_code=={'cancel':'CANCELED','pause':'PAUSED'}[operation]
    record=rig.store.list_inspection_results(rig.mapping.map_id,'run1')[0]
    assert not record.success and not record.text and Path(record.image_path).is_file()
    rig.recognition_gate.set()
    assert rig.recognition_finished.wait(2.)
    saved=rig.store.list_inspection_results(rig.mapping.map_id,'run1')
    assert len(saved)==1 and saved[0].result_id==record.result_id and not saved[0].text
    assert sum(kind=='capture' for kind,_ in rig.events)==1


def test_localization_changes_after_capture_do_not_invalidate_the_saved_image(rig):
    rig.step.shots=[rig.point.shots[0]]
    rig.delay_recognition=True
    rig.recognition_gate.clear()
    results=[]
    worker=threading.Thread(target=lambda:results.append(rig.run()))
    worker.start()
    assert rig.recognizing.wait(2.)
    rig.pose[0]+=.2
    rig.motion_pose[0]+=.1
    rig.recognition_gate.set()
    worker.join(2.)
    assert not worker.is_alive() and results[0].success
    record=rig.store.list_inspection_results(rig.mapping.map_id,'run1')[0]
    assert record.metadata['pose_guard']['displacement_m']==0.
    assert record.metadata['capture_validated_at_ns']<=record.metadata['recognition']['started_at_ns']


def test_motion_during_capture_aborts_before_sending_image_to_deepseek(rig):
    rig.on_capture=lambda:rig.motion_pose.__setitem__(0,.1)
    result=rig.run()
    assert not result.success and result.error_code=='INSPECTION_BASE_MOVED'
    assert not any(kind=='recognize' for kind,_ in rig.events)


def test_stable_but_wrong_feedback_never_captures(rig):
    rig.apply_set=False
    result=rig.run()
    assert not result.success and result.error_code=='INSPECTION_PTZ_NOT_ARRIVED'
    assert not any(kind=='capture' for kind,_ in rig.events)
    assert any(kind=='stop' for kind,_ in rig.events)
    assert len(rig.store.list_inspection_results(rig.mapping.map_id,'run1'))==1


@pytest.mark.parametrize('failure',['missing_pose','far_pose','stale_map','nan_feedback',
                                  'missing_motion_source','missing_motion_pose','nan_motion_pose'])
def test_invalid_pose_or_feedback_never_produces_normal_photo(rig,failure):
    if failure=='missing_pose':
        rig.inspection.sequence.pose_provider=lambda:None
    elif failure=='far_pose':
        rig.pose[0]=1.
    elif failure=='stale_map':
        rig.store._conn.execute('UPDATE maps SET revision=2')
    elif failure=='missing_motion_source':
        rig.inspection.sequence.motion_pose_provider=None
    elif failure=='missing_motion_pose':
        rig.inspection.sequence.motion_pose_provider=lambda:None
    elif failure=='nan_motion_pose':
        rig.motion_pose[0]=math.nan
    else:
        rig.actual['zoom']=math.nan
    assert not rig.run().success
    assert not any(kind=='capture' for kind,_ in rig.events)
    assert not any(kind=='set' for kind,_ in rig.events)


@pytest.mark.parametrize('operation',['cancel','pause','move','lost_odometry'])
def test_interrupt_pending_set_stops_now_and_after_late_set_reply(rig,operation):
    h=rig
    h.delay_set=True
    h.set_gate.clear()
    flags,results=ControlFlags(),[]
    worker=threading.Thread(target=lambda:results.append(h.run(flags)))
    worker.start()
    assert h.setting.wait(2.)
    if operation=='move':
        h.motion_pose[0]+=.1
    elif operation=='lost_odometry':
        h.inspection.sequence.motion_pose_provider=lambda:None
    else:
        getattr(flags,'request_'+operation)()
    worker.join(.8)
    assert not worker.is_alive()
    assert results[0].error_code=={'cancel':'CANCELED','pause':'PAUSED','move':'INSPECTION_BASE_MOVED',
                                 'lost_odometry':'INSPECTION_POSE_UNAVAILABLE'}[operation]
    assert not any(kind=='capture' for kind,_ in h.events)
    assert any(kind=='stop' for kind,_ in h.events)
    h.set_gate.set()
    wait_for(lambda:not h.inspection.sequence.late_sets)
    assert [kind for kind,_ in h.events][-2:]==['set','stop']
    assert not h.moving


@pytest.mark.parametrize('axis,change',[(0,.06),(1,.06),(2,math.radians(1.2))])
def test_vehicle_motion_aborts_without_capture(rig,axis,change):
    rig.step.shots=[rig.point.shots[0]]
    rig.inspection.sequence.settings['stable_sec']=.15
    results=[]
    worker=threading.Thread(target=lambda:results.append(rig.run()))
    worker.start()
    assert rig.setting.wait(2.)
    rig.motion_pose[axis]+=change
    worker.join(2.)
    assert not results[0].success and results[0].error_code=='INSPECTION_BASE_MOVED'
    assert not any(kind=='capture' for kind,_ in rig.events)
    result=rig.store.list_inspection_results(rig.mapping.map_id,'run1')[0]
    diagnostic=result.metadata['pose_guard']
    assert diagnostic['pose_source']=='odom→base_link'
    assert diagnostic['last_odom_pose'][('x','y','yaw')[axis]]==pytest.approx(change)
    assert diagnostic['map_displacement_m']==0. and diagnostic['map_rotation_deg']==0.
    assert diagnostic['translation_limit_m']==.05 and diagnostic['rotation_limit_deg']==1.
    assert diagnostic['displacement_m']==pytest.approx(.06 if axis<2 else 0.)
    assert diagnostic['rotation_deg']==pytest.approx(1.2 if axis==2 else 0.)
    manifest=next(rig.inspection.image_dir.rglob('*.json'))
    assert json.loads(manifest.read_text())['pose_guard']==diagnostic
    assert result.metadata['stop_confirmed']


def test_map_correction_during_positioning_does_not_count_as_vehicle_motion(rig):
    rig.step.shots=[rig.point.shots[0]]
    def correct_map():
        if rig.setting.is_set():
            rig.pose[:]=[.1,-.1,math.radians(5.)]
    rig.on_get=correct_map
    assert rig.run().success
    result=rig.store.list_inspection_results(rig.mapping.map_id,'run1')[0]
    assert result.metadata['target']['pan']==pytest.approx(1.)
    diagnostic=result.metadata['pose_guard']
    assert diagnostic['displacement_m']==0. and diagnostic['rotation_deg']==0.
    assert diagnostic['map_displacement_m']==pytest.approx(math.sqrt(.02))
    assert diagnostic['map_rotation_deg']==pytest.approx(3.)
    assert len([event for event in rig.events if event[0]=='capture'])==1


def test_angles_arrived_but_zoom_did_not_never_captures(rig):
    rig.apply_zoom=False
    result=rig.run()
    assert not result.success and result.error_code=='INSPECTION_PTZ_NOT_ARRIVED'
    assert not any(kind=='capture' for kind,_ in rig.events)


def test_failed_stop_blocks_new_commands_until_stop_is_confirmed(rig):
    rig.apply_set=False
    rig.stop_ok=False
    assert not rig.run().success
    assert rig.inspection.sequence.stop_unconfirmed
    commands=sum(kind=='set' for kind,_ in rig.events)
    rig.apply_set=True
    blocked=rig.run()
    assert not blocked.success and blocked.error_code=='INSPECTION_STOP_UNCONFIRMED'
    assert sum(kind=='set' for kind,_ in rig.events)==commands
    rig.stop_ok=True
    assert rig.run().success
    assert not rig.inspection.sequence.stop_unconfirmed


def test_new_run_waits_for_stop_ack_after_late_positioning(rig):
    rig.delay_set=True
    rig.set_gate.clear()
    flags,results=ControlFlags(),[]
    worker=threading.Thread(target=lambda:results.append(rig.run(flags)))
    worker.start()
    assert rig.setting.wait(2.)
    flags.request_cancel()
    worker.join(.8)
    assert not worker.is_alive()
    rig.block_late_stop=True
    rig.stop_gate.clear()
    rig.set_gate.set()
    assert rig.late_stopping.wait(2.)
    commands=sum(kind=='set' for kind,_ in rig.events)
    blocked=rig.run()
    assert blocked.error_code=='INSPECTION_PTZ_PENDING'
    assert sum(kind=='set' for kind,_ in rig.events)==commands
    rig.stop_gate.set()
    wait_for(lambda:not rig.inspection.sequence.late_sets)
    rig.delay_set=False
    assert rig.run().success


def test_capture_retries_and_repeated_runs_never_overwrite(rig):
    rig.capture_failures=2
    assert rig.run().success
    first={str(p) for p in rig.inspection.image_dir.rglob('*.jpg')}
    assert len(first)==2
    assert rig.run().success
    second={str(p) for p in rig.inspection.image_dir.rglob('*.jpg')}
    assert len(second)==4 and first < second


def test_point_shot_service_updates_owner_store_and_rejects_busy_writes(rig,tmp_path):
    from robot_patrol.node import PatrolNode
    node=PatrolNode(parameter_overrides=[Parameter('database_path',value=str(tmp_path/'owner.sqlite3'))])
    rig.extra_nodes.append(node)
    rig.executor.add_node(node)
    from test_store import map_data
    mapping=node._store.upsert_map(map_data())
    point=node._store.upsert_point('',mapping.map_id,'owner point',0.,0.,0.,'manual',1)
    node._active_map_id=mapping.map_id
    client=rig.node.create_client(PointShots,'/patrol/point_shots')
    assert client.wait_for_service(timeout_sec=3.)
    def call(request):
        future=client.call_async(request)
        wait_for(future.done)
        return future.result()
    request=PointShots.Request(command=1,point_id=point.point_id,map_id=mapping.map_id,
        map_revision=1,expected_revision=1,shots_json=shots_json([shot()]))
    node._run_reserved=True
    assert not call(request).success
    node._run_reserved=False
    configured=call(request)
    assert configured.success and configured.point.inspection_enabled
    assert not call(request).success  # Stale point revision.
    read=call(PointShots.Request(point_id=point.point_id))
    assert read.success and json.loads(read.shots_json)[0]['shot_id']=='wide'


def test_cli_calibration_and_saved_point_action_capture_both_views(rig,tmp_path):
    from geometry_msgs.msg import TransformStamped
    from rclpy.action import ActionClient
    from tf2_ros import StaticTransformBroadcaster
    from patrol_interfaces.action import InspectPoint
    from robot_patrol.node import PatrolNode
    from robot_patrol.map_metadata import read_map_binding
    from test_patrol_integration import FakeNav, make_map, wait_future
    owner=PatrolNode(parameter_overrides=[
        Parameter('database_path',value=str(tmp_path/'owner.sqlite3')),
        Parameter('web_map_dir',value=str(tmp_path/'web_maps')),
        Parameter('inspection_image_dir',value=str(tmp_path/'owner_images')),
        Parameter('inspection_shot_stable_sec',value=.04),
        Parameter('inspection_shot_poll_sec',value=.03),
        Parameter('inspection_shot_arrival_timeout_sec',value=2.),
        Parameter('settle_duration_sec',value=.1),
        Parameter('settle_timeout_sec',value=2.),
        Parameter('min_dispatch_interval_sec',value=0.),
        Parameter('inspection_shot_pan_sign',value=-1),
    ])
    fake=FakeNav(execute_duration=.1)
    fake.pose_x=fake.pose_y=-.5
    # FakeNav publishes map->base_link; a fixed map->odom also exposes the
    # continuous odometry frame used by the production motion guard.
    odom_tf=StaticTransformBroadcaster(fake)
    transform=TransformStamped()
    transform.header.frame_id='map'
    transform.child_frame_id='odom'
    transform.header.stamp=fake.get_clock().now().to_msg()
    transform.transform.rotation.w=1.
    odom_tf.sendTransform(transform)
    rig.extra_nodes.extend([owner,fake])
    rig.executor.add_node(owner)
    rig.executor.add_node(fake)
    mapping=owner._store.upsert_map(read_map_binding(make_map(tmp_path),name='shots map',backend='amcl'))
    owner._active_map_id=mapping.map_id
    owner._store.set_active_map(mapping.map_id)
    point=owner._store.upsert_point('',mapping.map_id,'two views',-.5,-.5,0.,'manual',1)
    client=rig.node.create_client(PointShots,'/patrol/point_shots')
    assert client.wait_for_service(timeout_sec=3.)
    state_future=client.call_async(PointShots.Request(point_id=point.point_id))
    wait_for(state_future.done)
    state=state_future.result()
    for shot_id in ('wide', 'side'):
        if shot_id == 'side':
            rig.actual=dict(pan=130.,tilt=-8.,zoom=2.)
        result=subprocess.run([sys.executable,'-m','robot_patrol.ptz_shots_cli','record',point.point_id,
            '--shot-id',shot_id,'--name',shot_id,'--settle-seconds','0'],
            capture_output=True,text=True,timeout=15.)
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout)['device_pan_sign'] == -1
    configured=client.call_async(PointShots.Request(point_id=point.point_id))
    wait_for(configured.done)
    point=configured.result().point
    action=ActionClient(rig.node,InspectPoint,'/patrol/inspect_point')
    assert action.wait_for_server(timeout_sec=3.)
    goal=InspectPoint.Goal(request_id='two-shot-test',owner_session_id='test',map_id=mapping.map_id,
        map_revision=mapping.revision,point_id=point.point_id,point_revision=point.revision)
    handle=wait_future(action.send_goal_async(goal))
    assert handle.accepted
    result=wait_future(handle.get_result_async(),timeout=8.).result
    assert result.success, result.message
    assert result.completed_steps==1
    records=owner._store.list_inspection_results(mapping.map_id,result.run_id)
    assert len(records)==2 and all(record.success for record in records)
    assert {record.shot_id for record in records}=={'wide','side'}
    assert all(record.metadata['pose_guard']['baseline']['x']==pytest.approx(-.5) for record in records)


def test_changed_device_direction_requires_recalibration_before_any_motion(rig):
    rig.node.declare_parameter('inspection_shot_pan_sign', 1)
    result = rig.run()
    assert not result.success and result.error_code == 'STALE_DATA'
    assert not any(kind in ('set', 'capture') for kind, _ in rig.events)
