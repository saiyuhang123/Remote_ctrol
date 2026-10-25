"""UI recording service uses real feedback, with fake devices in domain 232."""
import json
import math
import os
import threading
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.action import GoalResponse
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from patrol_interfaces.srv import PointShots
from robot_patrol.node import PatrolNode
from test_ptz_sequence import rig, wait_for
from test_store import map_data

pytestmark = pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '232' or
    os.environ.get('ROS_LOCALHOST_ONLY') != '1', reason='requires isolated ROS domain 232')


@pytest.fixture
def owner(rig, tmp_path):
    node = PatrolNode(parameter_overrides=[Parameter('database_path', value=str(tmp_path/'owner.sqlite3')),
                                          Parameter('inspection_shot_pan_sign', value=-1)])
    rig.extra_nodes.append(node)
    rig.executor.add_node(node)
    mapping = node._store.upsert_map(map_data())
    node._active_map_id = mapping.map_id
    point = node._store.upsert_point('', mapping.map_id, 'UI point', 0., 0., 0., 'manual', 1)
    rig.tf_age = 0.

    def transform(*_):
        result = TransformStamped()
        result.header.stamp = (node.get_clock().now()-Duration(seconds=rig.tf_age)).to_msg()
        result.transform.translation.x, result.transform.translation.y = rig.pose[:2]
        result.transform.rotation.z = math.sin(rig.pose[2]/2)
        result.transform.rotation.w = math.cos(rig.pose[2]/2)
        return result

    node.tf_buffer = NS(lookup_transform=transform)
    client = rig.node.create_client(PointShots, '/patrol/point_shots')
    assert client.wait_for_service(timeout_sec=3.)

    def request(command=PointShots.Request.RECORD, shot_id='main', **changes):
        current = node._store.get_point(point.point_id)
        values = dict(command=command, point_id=point.point_id, map_id=mapping.map_id,
            map_revision=mapping.revision, expected_revision=current.revision,
            shot_id=shot_id, shot_name='仪表正面', settle_seconds=1.5)
        values.update(changes)
        return PointShots.Request(**values)

    def call(message):
        future = client.call_async(message)
        wait_for(future.done, timeout=4.)
        return future.result()

    return NS(node=node, point=point, mapping=mapping, request=request, call=call, client=client)


def test_ui_records_single_actual_view_overwrites_in_order_and_deletes(rig, owner):
    route = owner.node._store.upsert_route('', owner.mapping.map_id, 'route',
        [NS(point_id=owner.point.point_id, dwell_seconds=0.)])
    result = owner.call(owner.request(shots_json='[{"pan_cal":234}]'))
    assert result.success, result.message
    views = json.loads(result.shots_json)
    assert len(views) == 1 and result.point.inspection_enabled
    assert views[0]['pan_cal'] == rig.actual['pan']  # UI cannot supply fabricated angles.
    assert views[0]['yaw_cal'] == pytest.approx(rig.pose[2])
    assert result.device_pan_sign == -1 and views[0]['pan_sign'] == -1
    assert owner.node._store.get_route(route.route_id).revision == route.revision+1
    rig.actual = dict(pan=250., tilt=-8., zoom=3.)
    assert owner.call(owner.request(shot_id='second')).success
    assert not owner.call(owner.request(shot_id='third')).success
    rig.actual['pan'] = 155.
    result = owner.call(owner.request())
    assert result.success
    views = json.loads(result.shots_json)
    assert [view['shot_id'] for view in views] == ['main', 'second']
    assert views[0]['pan_cal'] == 155. and views[1]['pan_cal'] == 250.
    result = owner.call(owner.request(command=PointShots.Request.REMOVE, shot_id='second'))
    assert result.success and len(json.loads(result.shots_json)) == 1
    result = owner.call(owner.request(command=PointShots.Request.CLEAR))
    assert result.success and json.loads(result.shots_json) == []
    assert not result.point.inspection_enabled
    assert not owner.node._shot_recording
    assert not rig.events  # Recording never sends movement or capture commands.


@pytest.mark.parametrize('failure', ['base_motion', 'head_motion', 'zoom', 'nan', 'stale_tf', 'far'])
def test_ui_rejects_invalid_or_moving_feedback_without_saving(rig, owner, failure):
    if failure == 'base_motion':
        rig.on_get = lambda: rig.pose.__setitem__(0, rig.pose[0]+.08)
    elif failure == 'head_motion':
        rig.on_get = lambda: rig.actual.__setitem__('pan', rig.actual['pan']+2.)
    elif failure == 'zoom':
        rig.actual['zoom'] = 5.
    elif failure == 'nan':
        rig.actual['pan'] = math.nan
    elif failure == 'stale_tf':
        rig.tf_age = 5.
    else:
        rig.pose[0] = 1.
    result = owner.call(owner.request())
    assert not result.success
    assert owner.node._store.get_point(owner.point.point_id).revision == 1
    assert owner.node._store.get_point(owner.point.point_id).shots == []
    assert not owner.node._shot_recording
    assert not rig.events


def test_recording_blocks_task_start_and_concurrent_edits_but_allows_reading(rig, owner):
    entered, release = threading.Event(), threading.Event()
    def block_read():
        entered.set()
        assert release.wait(3.)
    rig.on_get = block_read
    future = owner.client.call_async(owner.request())
    try:
        assert entered.wait(2.)
        assert owner.node._goal_callback(NS()) == GoalResponse.REJECT
        assert owner.node._return_goal_callback(NS()) == GoalResponse.REJECT
        assert not owner.call(owner.request()).success
        assert owner.call(owner.request(command=PointShots.Request.GET)).success
    finally:
        release.set()
    wait_for(future.done)
    assert future.result().success
    assert not owner.node._shot_recording


def test_point_changed_during_readback_cannot_receive_stale_calibration(rig, owner):
    def change_point_once():
        rig.on_get = None
        point = owner.node._store.get_point(owner.point.point_id)
        owner.node._store.upsert_point(point.point_id, point.map_id, 'changed', 0., 0., 0.,
            'manual', 2, point.revision)
    rig.on_get = change_point_once
    result = owner.call(owner.request())
    assert not result.success and '版本' in result.message
    assert not owner.node._store.get_point(owner.point.point_id).shots
    assert not owner.node._shot_recording


def test_device_direction_is_required_once_and_cannot_be_overridden_per_view(rig, owner):
    owner.node.set_parameters([Parameter('inspection_shot_pan_sign', value=0)])
    state = owner.call(owner.request(command=PointShots.Request.GET))
    assert state.success and state.device_pan_sign == 0
    result = owner.call(owner.request(pan_sign=-1))
    assert not result.success and '一次性' in result.message
    assert not owner.node._store.get_point(owner.point.point_id).shots
    owner.node.set_parameters([Parameter('inspection_shot_pan_sign', value=1)])
    result = owner.call(owner.request(pan_sign=-1))  # Old caller cannot override the device.
    assert result.success, result.message
    assert result.device_pan_sign == 1
    assert json.loads(result.shots_json)[0]['pan_sign'] == 1
    assert owner.call(owner.request(shot_id='another')).success  # No per-view direction input.


def test_device_configuration_changed_during_recording_does_not_save_stale_direction(rig, owner):
    def change_once():
        rig.on_get = None
        owner.node.set_parameters([Parameter('inspection_shot_pan_sign', value=1)])
    rig.on_get = change_once
    result = owner.call(owner.request())
    assert not result.success and '设备配置已变化' in result.message
    assert not owner.node._store.get_point(owner.point.point_id).shots
