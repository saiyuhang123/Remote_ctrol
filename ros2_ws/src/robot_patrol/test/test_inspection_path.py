"""Teaching workflow and ping-pong execution, isolated from the vehicle."""
import copy
import json
import math
from pathlib import Path
import sqlite3
import threading
import time
from types import SimpleNamespace as NS

import pytest

from robot_patrol.engine import PatrolEngine
from robot_patrol.models import PatrolError, RouteSnapshot
from robot_patrol.store import PatrolStore
from robot_patrol.recorded_routes.component import RoadComponent
from robot_patrol.recorded_routes.geometry import motion_blocks, validate_blocks
from robot_patrol.recorded_routes.graph import validate_network
from robot_patrol.recorded_routes.inspection_path import (
    path_edges, path_nodes, prepare_path, plan_path, plan_leg,
)
from test_store import map_data


@pytest.fixture
def teaching(tmp_path):
    store = PatrolStore(tmp_path/'paths.sqlite3')
    data = map_data(str(tmp_path/'map.yaml'))
    data.update(origin_x=-4., origin_y=-4., width=240, height=240)
    mapping = store.upsert_map(data)
    params = dict(enable_recorded_routes=True, recorded_min_turning_radius=.69,
                  recorded_footprint='[[0.4475,0.28],[0.4475,-0.28],[-0.4475,-0.28],[-0.4475,0.28]]')
    comp = RoadComponent.__new__(RoadComponent)
    grid = NS(info=NS(resolution=.05, width=240, height=240,
                     origin=NS(position=NS(x=-4.,y=-4.), orientation=NS(z=0.,w=1.))), data=[0]*57600)
    comp.node = NS(get_parameter=lambda key:NS(value=params[key]), _active_map=lambda:mapping,
                   _verify_map=lambda _:None, _verify_runtime_map=lambda _:None,
                   _require_execution_safety=lambda:None, _run_lock=threading.RLock(),
                   _run_reserved=False, _active_engine=None, _return_active=False,
                   _hazard_driver=None, _latest_map=grid)
    comp.store=store;comp.lock=threading.RLock();comp.recorder=None
    comp.owner='';comp.network_id='';comp.draft_id='';comp.last_record='';comp.progress={}
    comp.plans={};comp.command_cache={};comp.operator_seen={'operator':(time.monotonic(),True)}
    comp.live=dict(x=0.,y=0.,yaw=0.,t=1.,v=0.)
    comp.current=lambda:copy.deepcopy(comp.live)
    comp.publish_preview=lambda _:None
    comp.publish_plan=lambda _:None
    yield comp
    store.close()


def start(comp):
    comp.dispatch('START_PATH',dict(name='厂区巡检',owner_session_id='operator'))


def request(comp, op):
    net=comp.store.get_network(comp.network_id)
    return comp.dispatch(op,dict(network_id=comp.network_id,expected_revision=net['revision'],
                                 owner_session_id='operator'))


def drive(comp, x1, x2):
    t=comp.live['t']
    count=max(1,round(abs(x2-x1)/.02))
    for i in range(1,count+1):
        comp.live=dict(x=x1+(x2-x1)*i/count,y=0.,yaw=0.,t=t+i*.1,
                       v=.2 if x2>x1 else -.2)
        comp.recorder.add(comp.live)


def finish_two_segments(comp):
    start(comp);drive(comp,0.,1.)
    request(comp,'MARK_PATH_POINT')
    drive(comp,1.,2.)
    return request(comp,'SAVE_PATH')


def test_one_workflow_creates_points_bindings_published_route_and_reload(teaching):
    comp=teaching
    result=finish_two_segments(comp)
    assert comp.recorder is None
    net=result['network'];doc=net['document']
    assert net['published'] and result['point_count']==3 and not result['closed']
    assert len(comp.store.list_points(net['map_id']))==3
    route=comp.store.get_route(result['route_id'])
    assert route.navigation_mode=='RECORDED_NETWORK' and route.network_revision==net['revision']
    assert [s.x for s in route.steps]==[0.,1.,2.]
    assert len(doc['edges'])==4
    validate_network(doc)
    reloaded=PatrolStore(comp.store.path)
    assert reloaded.get_route(route.route_id)==route
    assert reloaded.list_road_drafts(net['map_id'])==[]
    reloaded.close()


def test_split_immediately_creates_point_and_stationary_tail_finishes(teaching):
    comp=teaching;start(comp);drive(comp,0.,1.)
    split=request(comp,'MARK_PATH_POINT')
    assert split['point_count']==2
    assert len(comp.store.list_points(split['network']['map_id']))==2
    t=comp.live['t']
    for i in range(1,81):
        comp.live=dict(x=1.+.008*math.sin(i),y=.006*math.cos(i),yaw=0.,t=t+i*.1,
                       v=[0.,.03,-.03][i%3])
        comp.recorder.add(comp.live)
    result=request(comp,'SAVE_PATH')
    assert len(path_edges(result['network']['document']))==1
    assert result['point_count']==2


def test_braking_rebound_can_split_continue_and_publish_the_measured_path(teaching):
    comp=teaching;start(comp)
    samples=json.loads((Path(__file__).parent/'data/recorded_stop_rebound.json').read_text())
    # These are persisted, spatially decimated samples, not the original
    # 10 Hz input. Restore them through the draft recovery workflow.
    comp.recorder.samples=[dict(p,t=p['t']+1.1) for p in samples]
    comp.live=copy.deepcopy(comp.recorder.samples[-1])
    comp._save_draft()
    draft_id=comp.draft_id
    comp.recorder=None
    comp.dispatch('RECOVER_PATH',dict(draft_id=draft_id,owner_session_id='operator'))
    request(comp,'RESUME_RECORD')
    measured=copy.deepcopy(comp.live)
    split=request(comp,'MARK_PATH_POINT')
    first=path_edges(split['network']['document'])[0]
    assert [b['direction'] for b in first['blocks']]==[1]
    assert first['blocks'][-1]['samples'][-1]=={k:measured[k] for k in ('x','y','yaw')}
    assert len(first['raw_samples'])>len(samples)-10
    # Continue from the actual settled split point, with no endpoint jump.
    for i in range(1,51):
        comp.live=dict(x=measured['x']+i*.02*math.cos(measured['yaw']),
                       y=measured['y']+i*.02*math.sin(measured['yaw']),
                       yaw=measured['yaw'], t=measured['t']+i*.1,v=.2)
        comp.recorder.add(comp.live)
    result=request(comp,'SAVE_PATH')
    assert result['network']['published'] and result['point_count']==3
    validate_network(result['network']['document'],.69)


def test_empty_tail_can_be_recovered_after_backend_restart(teaching):
    comp=teaching;start(comp);drive(comp,0.,1.)
    request(comp,'MARK_PATH_POINT')
    draft_id=comp.draft_id
    comp.recorder=None
    recovered=comp.dispatch('RECOVER_PATH',dict(draft_id=draft_id,owner_session_id='operator'))
    assert recovered['state']=='PAUSED'
    result=request(comp,'SAVE_PATH')
    assert result['point_count']==2
    assert not comp.store.list_road_drafts(result['network']['map_id'])


@pytest.mark.parametrize('radius',[.3,.7,.9])
@pytest.mark.parametrize('noise',[0.,.015])
def test_curvature_verdict_is_invariant_under_exact_reverse(radius,noise):
    points=[dict(x=radius*math.sin(i*.045),y=radius*(1-math.cos(i*.045)),
                 yaw=i*.045+noise*math.sin(i*2.1)) for i in range(81)]
    blocks=motion_blocks(points)
    def verdict(values):
        try:validate_blocks(values,.69);return True
        except PatrolError:return False
    reverse=[dict(direction=-b['direction'],samples=list(reversed(b['samples']))) for b in reversed(blocks)]
    assert verdict(blocks)==verdict(reverse)
    if radius==.3:assert not verdict(reverse)


@pytest.mark.parametrize('start_at,loops',[(0.,1),(0.,2),(0.,3),(2.,1),(2.,2),(2.,3)])
def test_exact_path_order_and_body_heading_for_both_start_endpoints(teaching,start_at,loops):
    comp=teaching;result=finish_two_segments(comp)
    route=comp.store.get_route(result['route_id'])
    comp.live.update(x=start_at,v=0.)
    plan=comp.preview(route.route_id,loops)
    assert plan['traversal']=='SHUTTLE'
    first=[0.,1.,2.] if start_at==0 else [2.,1.,0.]
    all_calls=[]
    class Driver:
        def cancel_current(self):pass
        def navigate(self,step,flags,status_cb):
            status=engine.status
            leg=plan_leg(plan,status['loop_index'],status['step_index']-1)
            assert leg['point_id']==step.point_id
            assert leg['target']['x']==step.x
            for block in leg['blocks']:
                assert all(p['yaw']==0. for p in block['samples'])
                dx=block['samples'][-1]['x']-block['samples'][0]['x']
                assert dx*block['direction']>0
            all_calls.append(step.x)
            return 'SUCCEEDED'
        def settle(self,step,flags,status_cb):return 'OK',(step.x,step.y,step.yaw)
    snap=RouteSnapshot(route.route_id,route.map_id,route.name,route.revision,route.steps,
                       navigation_mode='RECORDED_NETWORK',recorded_plan=plan)
    comp.store.create_run('test-run',route.route_id,route.map_id,route.revision,1,snap.as_dict(),loops,1,1)
    engine=PatrolEngine(comp.store,snap,'test-run',loops,'operator',Driver(),lambda _:None,min_dispatch_interval=0.)
    execution=engine.run()
    assert execution.success and execution.completed_loops==loops
    assert all_calls==[x for i in range(loops) for x in (first if i%2==0 else reversed(first))]


def test_reverse_preserves_real_cusps_and_yaw():
    pts=[dict(x=i*.05,y=0.,yaw=0.) for i in range(21)]
    pts += [dict(x=1.-i*.05,y=0.,yaw=0.) for i in range(1,11)]
    pts += [dict(x=.5+i*.05,y=0.,yaw=0.) for i in range(1,21)]
    doc=dict(schema_version=1,nodes={'a':dict(id='a',x=0.,y=0.,yaw=0.),'b':dict(id='b',x=1.5,y=0.,yaw=0.)},
             edges={'ab':dict(id='ab',**{'from':'a','to':'b'},name='转向',speed=.2,blocks=motion_blocks(pts))},bindings={})
    prepare_path(doc,'转向',True)
    validate_network(doc)
    back=doc['edges']['return_ab']
    assert [b['direction'] for b in back['blocks']]==[-1,1,-1]
    for old,new in zip(reversed(doc['edges']['ab']['blocks']),back['blocks']):
        assert new['samples']==list(reversed(old['samples']))


def test_closed_path_stays_forward_and_traverses_whole_loop(teaching):
    comp=teaching;start(comp)
    # A closed forward circle with the same body pose at the seam.
    for i in range(1,201):
        a=i*2*math.pi/200
        comp.live=dict(x=.9*math.sin(a),y=.9*(1-math.cos(a)),yaw=a,t=1.+i*.1,v=.28)
        comp.recorder.add(comp.live)
    result=request(comp,'SAVE_PATH')
    assert result['closed']
    route=comp.store.get_route(result['route_id'])
    plan=comp.preview(route.route_id,2)
    assert plan['traversal']=='CLOSED' and len(plan['pass_legs'])==1
    for loop in (1,2,3):
        leg=plan_leg(plan,loop,0)
        assert leg['blocks'] and all(b['direction']==1 for b in leg['blocks'])
    assert len(result['network']['document']['edges'])==1


def test_auto_publish_collision_failure_keeps_draft_and_no_partial_task(teaching):
    comp=teaching;start(comp);drive(comp,0.,1.)
    comp.node._latest_map.data[:]=[100]*len(comp.node._latest_map.data)
    with pytest.raises(PatrolError):request(comp,'SAVE_PATH')
    assert comp.recorder is not None
    assert comp.store.list_points(comp.recorder.map_id)==[]
    assert comp.store.list_routes(comp.recorder.map_id)==[]
    assert not comp.store.get_network(comp.network_id)['published']
    comp.node._latest_map.data[:]=[0]*len(comp.node._latest_map.data)
    assert request(comp,'SAVE_PATH')['network']['published']


def test_bundle_rolls_back_points_network_draft_when_route_insert_fails(teaching):
    comp=teaching;start(comp);drive(comp,0.,1.)
    request(comp,'MARK_PATH_POINT');drive(comp,1.,2.)
    before=comp.store.get_network(comp.network_id)
    comp.store._conn.execute("CREATE TRIGGER fail_task BEFORE INSERT ON routes BEGIN SELECT RAISE(ABORT,'test failure'); END")
    with pytest.raises(sqlite3.IntegrityError):request(comp,'SAVE_PATH')
    assert comp.store.get_network(comp.network_id)==before
    assert len(comp.store.list_points(before['map_id']))==2
    assert comp.store.list_routes(before['map_id'])==[]
    assert comp.recorder is not None
    comp.store._conn.execute('DROP TRIGGER fail_task')
    assert request(comp,'SAVE_PATH')['network']['published']


def test_stale_revision_or_wrong_owner_cannot_save_path(teaching):
    comp=teaching;start(comp);drive(comp,0.,1.)
    for owner,revision in [('other',1),('operator',0)]:
        with pytest.raises(PatrolError):
            comp.dispatch('SAVE_PATH',dict(network_id=comp.network_id,expected_revision=revision,owner_session_id=owner))
    assert comp.recorder is not None and not comp.store.list_points(comp.recorder.map_id)


def test_duplicate_save_command_does_not_create_duplicate_route_or_points(teaching):
    comp=teaching;start(comp);drive(comp,0.,1.)
    net=comp.store.get_network(comp.network_id)
    import json
    req=NS(operation='SAVE_PATH',payload_json=json.dumps(dict(network_id=comp.network_id,
        expected_revision=net['revision'],owner_session_id='operator',command_id='save-once')))
    one=comp.command(req,NS());two=comp.command(req,NS())
    assert one.success and two.success and one.data_json==two.data_json
    assert len(comp.store.list_routes(net['map_id']))==1
    assert len(comp.store.list_points(net['map_id']))==2


def test_continuous_shuttle_keeps_alternating_until_explicit_cancel(teaching):
    comp=teaching;result=finish_two_segments(comp)
    route=comp.store.get_route(result['route_id']);comp.live.update(x=0.)
    plan=comp.preview(route.route_id,0)
    snap=RouteSnapshot(route.route_id,route.map_id,route.name,route.revision,route.steps,
                       navigation_mode='RECORDED_NETWORK',recorded_plan=plan)
    calls=[]
    class Driver:
        def cancel_current(self):pass
        def navigate(self,step,flags,status_cb):calls.append(step.x);return 'SUCCEEDED'
        def settle(self,step,flags,status_cb):return 'OK',(step.x,step.y,step.yaw)
    def status(value):
        if value.get('completed_loops')==3:engine.request_cancel()
    comp.store.create_run('continuous',route.route_id,route.map_id,1,1,snap.as_dict(),0,1,1)
    engine=PatrolEngine(comp.store,snap,'continuous',0,'operator',Driver(),status,min_dispatch_interval=0.)
    execution=engine.run()
    assert execution.completed_loops==3 and execution.error_code=='CANCELED'
    assert calls==[0.,1.,2.,2.,1.,0.,0.,1.,2.]


def test_recorded_dwell_resume_forwards_the_engine_status_callback(monkeypatch):
    from robot_patrol.nav2_client import Nav2Driver
    from robot_patrol.recorded_routes.driver import RecordedDriver
    driver=RecordedDriver.__new__(RecordedDriver)
    calls=[]
    monkeypatch.setattr(Nav2Driver,'reconfirm',lambda self,step,flags,cb:calls.append((step,flags,cb)) or 'AT_POINT')
    step,flags,callback=object(),object(),lambda **_:None
    assert driver.reconfirm(step,flags,callback)=='AT_POINT'
    assert calls==[(step,flags,callback)]


def test_legacy_three_segment_chain_can_be_converted_without_new_recording(teaching):
    comp=teaching;result=finish_two_segments(comp)
    net=result['network'];doc=copy.deepcopy(net['document'])
    # Represent a legacy recording with roads only, no bindings or task.
    doc['edges']={k:e for k,e in doc['edges'].items() if not e.get('reverse_of')}
    doc.pop('inspection_path');doc['bindings']={}
    net=comp.store.save_network(net['map_id'],'已有道路',doc)
    comp.network_id=net['network_id']
    saved=request(comp,'SAVE_PATH')
    assert saved['network']['published']
    assert [s.x for s in comp.store.get_route(saved['route_id']).steps]==[0.,1.,2.]


def test_no_path_and_branch_or_moved_start_are_rejected(teaching):
    comp=teaching;start(comp)
    with pytest.raises(PatrolError):request(comp,'SAVE_PATH')
    result=finish_existing(comp)
    doc=copy.deepcopy(result['network']['document'])
    doc.pop('inspection_path')
    edge=copy.deepcopy(next(iter(doc['edges'].values())))
    edge['id']='branch';doc['edges']['branch']=edge
    with pytest.raises(PatrolError):path_edges(doc)
    route=comp.store.get_route(result['route_id'])
    for current in (dict(x=.5,y=0.,yaw=0.),dict(x=0.,y=0.,yaw=math.pi)):
        with pytest.raises(PatrolError):plan_path(result['network']['document'],route.steps,current,2)


def finish_existing(comp):
    drive(comp,0.,1.)
    return request(comp,'SAVE_PATH')
