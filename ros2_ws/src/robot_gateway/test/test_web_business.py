"""Gateway commands exercise real patrol handlers and a temporary store."""
import asyncio
from concurrent.futures import Future
import json
from types import SimpleNamespace as NS

import pytest
from rclpy.parameter import Parameter
from patrol_interfaces.srv import (DeletePoint, DeleteRoute, ListMaps, ListPoints,
                                   ListRoutes, RoadCommand, UpsertPoint, UpsertRoute)
from robot_patrol.node import PatrolNode
from test_gateway_commands import gateway, Action

pytestmark=pytest.mark.skipif(__import__('os').environ.get('ROBOT_GATEWAY_INTEGRATION')!='1',reason='isolated ROS tests only')


class Direct:
    def __init__(self, kind, callback):self.kind,self.callback=kind,callback
    def service_is_ready(self):return True
    def call_async(self, request):
        result=Future()
        result.set_result(self.callback(request,self.kind.Response()))
        return result


@pytest.fixture
def business(gateway,tmp_path):
    node,ws,data,results=gateway
    patrol=PatrolNode(parameter_overrides=[Parameter('database_path',value=str(tmp_path/'patrol.sqlite3')),
        Parameter('web_map_dir',value=str(tmp_path/'maps'))])
    mapping=patrol._store.upsert_map(dict(name='Test',backend='amcl',frame_id='map',navigation_yaml='test.yaml',image_path='test.pgm',localization_file='',
        resolution=.05,origin_x=0.,origin_y=0.,origin_yaw=0.,width=200,height=200,navigation_yaml_sha256='a'*64,image_sha256='b'*64,localization_sha256='',verified=True))
    patrol._active_map_id=mapping.map_id;patrol._store.set_active_map(mapping.map_id)
    bridge=node._business
    bridge.maps=Direct(ListMaps,patrol._on_list_maps)
    bridge.points=Direct(ListPoints,patrol._on_list_points)
    bridge.routes=Direct(ListRoutes,patrol._on_list_routes)
    for key,kind,method in [('point_save',UpsertPoint,patrol._on_upsert_point),('point_delete',DeletePoint,patrol._on_delete_point),
                            ('route_save',UpsertRoute,patrol._on_upsert_route),('route_delete',DeleteRoute,patrol._on_delete_route)]:
        bridge.clients[key]=Direct(kind,method)
    payload={key:data[key] for key in ('session_id','request_id')}
    payload.update(map_id=mapping.map_id,map_revision=mapping.revision)
    yield node,ws,payload,results,patrol,mapping
    patrol.destroy_node()


def command(fixture,kind,**values):
    node,ws,data,results,*_=fixture
    asyncio.run(node.handle_command(ws,dict(data,type=kind,**values)))
    return results[-1]


def test_edit_points_routes_conflicts_and_referenced_deletion(business):
    result=command(business,'point_save',name='仪表',x=1.,y=1.,yaw=.2,inspection_enabled=True,ptz_preset=7)
    assert result['ok'],result
    point=result['data']['point'];identifier=point['point_id']
    result=command(business,'route_save',name='路线',steps=[dict(point_id=identifier,dwell_seconds=3.)])
    assert result['ok'],result
    route=result['data']['route']
    assert command(business,'point_delete',point_id=identifier,expected_revision=point['revision'])['ok'] is False
    changed=command(business,'point_save',point_id=identifier,expected_revision=point['revision'],name='仪表修改',x=1.2,y=1.1,yaw=.3)
    assert changed['ok']
    stale=command(business,'point_save',point_id=identifier,expected_revision=point['revision'],name='覆盖',x=1.,y=1.,yaw=0.)
    assert not stale['ok'] and '修改' in stale['message']
    store=business[4]._store
    assert store.get_point(identifier).name=='仪表修改'
    current=store.get_route(route['route_id'])
    assert current.revision>route['revision']
    saved=command(business,'route_save',route_id=current.route_id,expected_revision=current.revision,name='新的顺序',steps=[dict(point_id=identifier,dwell_seconds=1),dict(point_id=identifier,dwell_seconds=2)])
    assert saved['ok'] and len(saved['data']['route']['steps'])==2
    assert command(business,'route_delete',route_id=current.route_id,expected_revision=saved['data']['route']['revision'])['ok']
    assert command(business,'point_delete',point_id=identifier,expected_revision=changed['data']['point']['revision'])['ok']


def test_map_bounds_versions_and_active_task_block_edits(business):
    payload=dict(name='P',x=-1.,y=1.,yaw=0.)
    assert not command(business,'point_save',**payload)['ok']
    assert not command(business,'point_save',**(payload|{'x':1.,'map_revision':999}))['ok']
    node=business[0];node._patrol_status={'active':True}
    assert not command(business,'point_save',**(payload|{'x':1.}))['ok']
    assert business[4]._store.list_points(business[5].map_id)==[]


def test_fixed_preview_digest_is_owned_and_matches_execution(business):
    node,ws,data,results,patrol,mapping=business
    digest='a'*64
    def preview(request,response):
        response.success=True
        forward=dict(blocks=[dict(direction=1,samples=[dict(x=.5,y=1.,yaw=0.),dict(x=1.,y=1.,yaw=0.)])])
        backward=dict(blocks=[dict(direction=-1,samples=[dict(x=1.,y=1.,yaw=0.),dict(x=.5,y=1.,yaw=0.)])])
        response.data_json=json.dumps(dict(digest=digest,route_id='fixed',route_revision=2,map_id=mapping.map_id,map_revision=mapping.revision,loop_count=2,
            legs=[forward],initial_legs=[forward],pass_legs=[[forward],[backward]]))
        return response
    node._business.road=Direct(RoadCommand,preview)
    result=command(business,'route_preview',route_id='fixed',loop_count=2)
    assert result['ok'] and [item['direction'] for item in result['data']['blocks']]==[1,-1]
    node._route_client=Action()
    result=command(business,'route_start',route_id='fixed',route_revision=2,loop_count=1,plan_digest=digest)
    assert not result['ok'] and not node._route_client.goals
    result=command(business,'route_start',route_id='fixed',route_revision=2,loop_count=2,plan_digest=digest)
    assert result['ok'] and node._route_client.goals[0].plan_digest==digest
