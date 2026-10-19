from types import SimpleNamespace
import copy
import math
import sqlite3

import pytest

from robot_patrol.models import PatrolError
from robot_patrol.recorded_routes.geometry import motion_blocks, join_edges, validate_blocks
from robot_patrol.recorded_routes.graph import NetworkPlanner, validate_network
from robot_patrol.recorded_routes.recorder import Recorder
from robot_patrol.store import PatrolStore
from test_store import map_data


def samples(x1, x2, yaw=0.0):
    n = int(abs(x2-x1)/0.05)+1
    return [dict(x=x1+(x2-x1)*i/n, y=0.0, yaw=yaw) for i in range(n+1)]


def network():
    nodes = {n: dict(id=n, name=n, x=float(x), y=0., yaw=0.) for n,x in [('A',0),('B',1),('C',2)]}
    edges = {}
    # Reverse gear is explicitly recorded, preserving body heading; not an
    # automatically reversed forward lane.
    for a,b in [('A','B'),('B','C'),('C','B'),('B','A')]:
        pts = samples(nodes[a]['x'], nodes[b]['x'])
        edges[a+b] = dict(id=a+b, name=a+b, **{'from': a, 'to': b}, speed=.2,
                         blocks=motion_blocks(pts), enabled=True)
    return dict(schema_version=1,nodes=nodes,edges=edges,bindings={n:[n] for n in nodes})


def steps(*names):
    return [SimpleNamespace(point_id=n,point_name=n,x={'A':0,'B':1,'C':2}[n],y=0.) for n in names]


def test_arbitrary_stop_order_uses_roads_and_preserves_business_order():
    doc = network()
    validate_network(doc)
    plan = NetworkPlanner(doc).plan(steps('A','C','B'), dict(x=0.,y=0.,yaw=0.),2)
    assert [leg['point_id'] for leg in plan['legs']] == ['A','C','B']
    assert plan['legs'][1]['edge_ids'] == ['AB','BC']
    assert plan['legs'][2]['edge_ids'] == ['CB']
    assert plan['closure']['edge_ids'] == ['BA']
    assert plan['legs'][1]['blocks'][0]['barriers']  # B remains a junction.


def test_no_reverse_connection_and_no_free_fallback():
    doc=network()
    del doc['edges']['CB']
    with pytest.raises(PatrolError,match='缺少'):
        NetworkPlanner(doc).plan(steps('A','C','B'),dict(x=0.,y=0.,yaw=0.))


def test_same_xy_different_heading_does_not_create_a_junction():
    doc=network()
    doc['nodes']['B2']=dict(id='B2',name='B2',x=1.,y=0.,yaw=math.pi)
    doc['bindings']['B']=['B2']
    with pytest.raises(PatrolError):
        NetworkPlanner(doc).plan(steps('B'),dict(x=0.,y=0.,yaw=0.))


def test_cycle_checked_and_duplicate_stops_are_not_merged():
    doc=network()
    planner=NetworkPlanner(doc)
    plan=planner.plan(steps('A','A','C'),dict(x=0.,y=0.,yaw=0.),0)
    assert len(plan['legs'])==3 and plan['legs'][1]['blocks']==[]
    del doc['edges']['BA']
    with pytest.raises(PatrolError):
        NetworkPlanner(doc).plan(steps('A','C'),dict(x=0.,y=0.,yaw=0.),2)


def test_single_stop_loop_cannot_succeed_at_the_coincident_start():
    doc=network()
    plan=NetworkPlanner(doc).plan(steps('A'),dict(x=0.,y=0.,yaw=0.),3)
    assert plan['legs'][0]['edge_ids']==['AB','BA']
    assert plan['closure']['edge_ids']==['AB','BA']
    # Return-home has a distinct semantic: already at home should stay there.
    home=NetworkPlanner(doc).plan(steps('A'),dict(x=0.,y=0.,yaw=0.),single_stop_loop=False)
    assert home['legs'][0]['blocks']==[]


def test_nearby_endpoint_is_not_a_teleporting_start_connection():
    doc=network()
    doc['nodes']['near']=dict(id='near',name='near',x=.1,y=0.,yaw=0.)
    candidates=NetworkPlanner(doc).start_candidates(dict(x=0.,y=0.,yaw=0.))
    assert [entry[0] for entry in candidates]==['A']


def test_orientation_state_dp_does_not_greedily_pick_a_dead_end():
    doc=network()
    doc['nodes']['B_dead']=dict(id='B_dead',name='Bdead',x=1.,y=0.,yaw=0.)
    edge=copy.deepcopy(doc['edges']['AB'])
    edge.update(id='dead',to='B_dead')
    doc['edges']['dead']=edge
    doc['bindings']['B']=['B_dead','B']
    plan=NetworkPlanner(doc).plan(steps('B','C'),dict(x=0.,y=0.,yaw=0.))
    assert plan['legs'][0]['target_node']=='B'


def test_cusps_keep_body_heading_and_split_direction():
    values=samples(0,1)+samples(1,0)[1:]+samples(0,.5)[1:]
    blocks=motion_blocks(values)
    assert [b['direction'] for b in blocks]==[1,-1,1]
    assert blocks[0]['samples'][-1]==blocks[1]['samples'][0]
    validate_blocks(blocks,.69)


def test_recorder_rejects_jumps_pause_motion_and_nan():
    r=Recorder('map',1)
    r.add(dict(x=0.,y=0.,yaw=0.,t=1.,v=0.))
    r.pause(dict(x=0.,y=0.,yaw=0.))
    with pytest.raises(PatrolError,match='已移动'):
        r.resume(dict(x=.2,y=0.,yaw=0.))
    r.resume(dict(x=0.,y=0.,yaw=0.))
    r.add(dict(x=.05,y=0.,yaw=0.,t=10.,v=.1))
    with pytest.raises(PatrolError):
        r.add(dict(x=2.,y=0.,yaw=0.,t=10.1,v=.1))
    assert r.state=='INVALID'


def test_storage_fixed_mode_revision_and_backup(tmp_path):
    path=tmp_path/'patrol.sqlite3'
    store=PatrolStore(path)
    mapping=store.upsert_map(map_data())
    a=store.upsert_point('',mapping.map_id,'A',0,0,0,'manual',1)
    doc=network()
    doc['bindings']={a.point_id:['A']}
    net=store.save_network(mapping.map_id,'场地道路',doc,published=True)
    route=store.upsert_route('',mapping.map_id,'固定',
                             [SimpleNamespace(point_id=a.point_id,dwell_seconds=0)],
                             navigation_mode='RECORDED_NETWORK',network_id=net['network_id'],network_revision=1)
    assert store.get_route(route.route_id).navigation_mode=='RECORDED_NETWORK'
    with pytest.raises(PatrolError,match='禁止降级'):
        store.upsert_route(route.route_id,mapping.map_id,'旧客户端',route.steps,route.revision)
    with pytest.raises(PatrolError):
        store.save_network(mapping.map_id,'错版本',doc,net['network_id'],0)
    with pytest.raises(PatrolError):
        store.delete_network(net['network_id'],1)
    store._conn.execute('UPDATE schema_version SET version=2')
    store.close()
    store=PatrolStore(path)
    assert list(tmp_path.glob('*.pre_v3.*.bak'))
    assert store.get_route(route.route_id).network_id==net['network_id']
    store.close()


def test_preview_rejects_vehicle_changes_and_moved_bindings(tmp_path):
    from robot_patrol.recorded_routes.component import RoadComponent
    comp=RoadComponent.__new__(RoadComponent)
    params={'enable_recorded_routes':True,'recorded_min_turning_radius':.69,
            'recorded_footprint':'[[0.4475,0.28],[0.4475,-0.28],[-0.4475,-0.28],[-0.4475,0.28]]'}
    comp.store=PatrolStore(tmp_path/'patrol.sqlite3');mapping=comp.store.upsert_map(map_data())
    comp.node=SimpleNamespace(get_parameter=lambda key:SimpleNamespace(value=params[key]),_active_map=lambda:mapping)
    comp._map=lambda:mapping;comp.current=lambda:dict(x=0.,y=0.,yaw=0.);comp.publish_plan=lambda plan:None
    import threading
    comp.recorder=None;comp.plans={};comp.lock=threading.RLock()
    a=comp.store.upsert_point('',mapping.map_id,'A',0,0,0,'manual',1)
    doc=network();doc['bindings']={a.point_id:['A']};doc['vehicle']=comp.vehicle_stamp()
    net=comp.store.save_network(mapping.map_id,'道路',doc,published=True)
    route=comp.store.upsert_route('',mapping.map_id,'固定',
        [SimpleNamespace(point_id=a.point_id,dwell_seconds=0)],navigation_mode='RECORDED_NETWORK',
        network_id=net['network_id'],network_revision=net['revision'])
    plan=comp.preview(route.route_id,1)
    assert comp.take_plan(route,plan['digest'],1)['digest']==plan['digest']
    params['recorded_min_turning_radius']=1.2
    with pytest.raises(PatrolError,match='底盘'):comp.take_plan(route,plan['digest'],1)
    params['recorded_min_turning_radius']=.69
    comp.store.upsert_point(a.point_id,mapping.map_id,'A moved',.5,0,0,'manual',2,a.revision)
    with pytest.raises(PatrolError,match='绑定失效'):comp.take_plan(route,plan['digest'],1)
    comp.store.close()


def test_unaccepted_internal_dispatch_cannot_be_cleared_by_an_empty_status():
    from robot_patrol.recorded_routes.driver import RecordedDriver
    import threading,time
    driver=RecordedDriver.__new__(RecordedDriver)
    driver._pending_follow_unknown=True;driver.stop_unconfirmed=True
    driver._current_handle=None;driver._handle_lock=threading.Lock()
    driver._follow_active=set();driver._follow_received=time.monotonic()
    # A request still in transport may not yet exist in the action status or
    # cancel-all response. Neither is proof that it cannot be accepted later.
    assert not driver._confirm_internal_stop()
    assert not driver.stop_safe()
