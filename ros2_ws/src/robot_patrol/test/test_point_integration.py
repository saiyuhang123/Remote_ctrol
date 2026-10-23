"""Isolated fake-device integration for one-click inspection and return-home."""
import json
import os
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from hk_camera.srv import CapturePicture, PTZPreset
from ocr_interfaces.srv import RecognizeImage
from patrol_interfaces.action import InspectPoint, ReturnHome
from robot_patrol.engine import STATE_CANCELED
from robot_patrol.map_metadata import read_map_binding
from robot_patrol.node import PatrolNode
from test_patrol_integration import FakeNav, make_map, wait_future

pytestmark=pytest.mark.skipif(os.environ.get('ROBOT_PATROL_INTEGRATION')!='1',reason='explicit isolated ROS test required')


def test_return_interrupts_pending_inspection_without_waiting_for_recognition(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and os.environ.get('ROS_DOMAIN_ID') not in ('',None,'0')
    rclpy.init()
    patrol=fake=executor=spin=None
    recognize_started=threading.Event()
    recognize_release=threading.Event()
    try:
        patrol=PatrolNode(parameter_overrides=[
            Parameter('database_path',value=str(tmp_path/'patrol.sqlite3')),
            Parameter('web_map_dir',value=str(tmp_path/'web_maps')),
            Parameter('inspection_image_dir',value=str(tmp_path/'images')),
            Parameter('inspection_preset_settle_sec',value=0.),
            Parameter('inspection_recognition_timeout_sec',value=150.),
            Parameter('settle_duration_sec',value=.1),
            Parameter('settle_timeout_sec',value=2.),
            Parameter('feedback_max_age_sec',value=2.),
            Parameter('min_dispatch_interval_sec',value=0.),
        ])
        fake=FakeNav(execute_duration=.15)
        def preset(request,response):
            response.success=True;return response
        def capture(request,response):
            response.success=True
            image=Image();image.height=image.width=2;image.encoding='bgr8';image.step=6;image.data=bytes(12)
            response.image=image;return response
        def recognize(request,response):
            recognize_started.set();recognize_release.wait(8.)
            response.success=True;response.text='late result must be ignored';return response
        inspection_group=ReentrantCallbackGroup()
        services=[fake.create_service(PTZPreset,'/hk_camera/preset',preset,callback_group=inspection_group),
                  fake.create_service(CapturePicture,'/hk_camera/capture',capture,callback_group=inspection_group),
                  fake.create_service(RecognizeImage,'/vision/recognize_image',recognize,callback_group=inspection_group)]
        executor=MultiThreadedExecutor(num_threads=8)
        executor.add_node(fake);executor.add_node(patrol)
        spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
        binding=read_map_binding(make_map(tmp_path),name='test map',backend='amcl')
        mapping=patrol._store.upsert_map(binding)
        patrol._active_map_id=mapping.map_id
        patrol._store.set_active_map(mapping.map_id)
        patrol._store.set_home_point(mapping.map_id,mapping.revision,0.,0.,0.)
        time.sleep(.5)
        client=ActionClient(fake,InspectPoint,'/patrol/inspect_point')
        assert client.wait_for_server(timeout_sec=3.)
        goal=InspectPoint.Goal();goal.request_id='one-click';goal.owner_session_id='pytest'
        goal.map_id=mapping.map_id;goal.map_revision=mapping.revision
        goal.x=goal.y=-.1;goal.inspection_enabled=True;goal.ptz_preset=1
        feedback=[]
        handle=wait_future(client.send_goal_async(goal,feedback_callback=lambda data:feedback.append(data.feedback.status.state)))
        assert handle.accepted
        result_future=handle.get_result_async()
        assert recognize_started.wait(4.), 'one-click did not reach inspection'
        home=ActionClient(fake,ReturnHome,'/patrol/return_home')
        assert home.wait_for_server(timeout_sec=2.)
        home_goal=ReturnHome.Goal();home_goal.request_id='home';home_goal.owner_session_id='pytest'
        home_goal.map_id=mapping.map_id;home_goal.map_revision=mapping.revision
        started=time.monotonic()
        home_handle=wait_future(home.send_goal_async(home_goal))
        assert home_handle.accepted
        home_result=wait_future(home_handle.get_result_async(),timeout=4.)
        assert home_result.result.success,home_result.result.message
        assert time.monotonic()-started<4., 'return waited for the recognition service'
        result=wait_future(result_future).result
        assert result.final_state==STATE_CANCELED and not result.success
        assert 14 in feedback, 'InspectPoint feedback used the wrong action message type'
        saved=patrol._store.list_inspection_results(mapping.map_id,result.run_id)
        assert len(saved)==1 and saved[0].error_code=='CANCELED' and not saved[0].success
        report=json.loads(patrol._store._conn.execute('SELECT payload_json FROM inspection_reports WHERE run_id=?',(result.run_id,)).fetchone()[0])
        assert report['run']['state']==STATE_CANCELED
        assert len(fake.goals)==2
        recognize_release.set();time.sleep(.1)
        assert not patrol._store.list_inspection_results(mapping.map_id,result.run_id)[0].success
        goal.request_id='one-click-complete'
        successful=wait_future(client.send_goal_async(goal))
        assert successful.accepted
        completed=wait_future(successful.get_result_async(),timeout=4.).result
        assert completed.success and completed.completed_steps==1
        completed_report=json.loads(patrol._store._conn.execute(
            'SELECT payload_json FROM inspection_reports WHERE run_id=?',(completed.run_id,)).fetchone()[0])
        assert completed_report['inspection_success_count']==1
        goal.request_id='stale-map';goal.map_revision+=1
        stale=wait_future(client.send_goal_async(goal))
        rejected=wait_future(stale.get_result_async(),timeout=2.).result
        assert not rejected.success and rejected.error_code=='STALE_DATA'
        assert len(fake.goals)==3, 'stale map target dispatched a navigation goal'
    finally:
        recognize_release.set()
        if executor:executor.shutdown(timeout_sec=3.)
        if spin:spin.join(3.)
        if patrol:patrol.destroy_node()
        if fake:fake.destroy_node()
        rclpy.try_shutdown()
