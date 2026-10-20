"""Automatic entry: geometry, endpoint fallback and one-time execution."""
import copy
import math
import threading
import time
from types import SimpleNamespace as NS

import pytest

from robot_patrol.models import PatrolError
from robot_patrol.recorded_routes.entry import connector_blocks
from robot_patrol.recorded_routes.inspection_path import choose_entry, plan_path, plan_leg
from test_inspection_path import teaching, finish_two_segments


def line(a, b):
    return [dict(x=a['x']+(b['x']-a['x'])*i/40,
                 y=a['y']+(b['y']-a['y'])*i/40, yaw=a['yaw']) for i in range(41)]


def test_connector_preserves_reverse_cusp_body_heading_and_limits_speed():
    points=[dict(x=i*.02,y=0.,yaw=0.) for i in range(21)]
    points += [dict(x=.4-i*.02,y=0.,yaw=0.) for i in range(1,31)]
    blocks=connector_blocks(points,points[0],points[-1],.69)
    assert [b['direction'] for b in blocks]==[1,-1]
    assert blocks[0]['samples'][-1]==blocks[1]['samples'][0]
    assert all(b['entry'] for b in blocks)
    assert [b['speed'] for b in blocks]==[.20,.12]
    assert all(p['yaw']==0. for b in blocks for p in b['samples'])
    assert blocks[-1]['entry_target']==points[-1]


@pytest.mark.parametrize('kind',['rotate','sideways','curvature','gap','wrong_end','wrong_start','nonfinite'])
def test_bad_connector_cannot_create_a_motion_leg(kind):
    start=dict(x=0.,y=0.,yaw=0.);target=dict(x=1.,y=0.,yaw=0.)
    pts=line(start,target)
    if kind=='rotate':pts.insert(1,dict(start,yaw=math.pi/2))
    if kind=='sideways':pts[10]['y']=.1
    if kind=='curvature':pts[10]['yaw']=.4
    if kind=='gap':pts=[start,target]
    if kind=='wrong_end':target['yaw']=math.pi
    if kind=='wrong_start':start['y']=.5
    if kind=='nonfinite':pts[10]['x']=math.nan
    with pytest.raises(PatrolError):connector_blocks(pts,start,target,.69)


def test_entry_geometry_accepts_forward_and_reverse_ackermann_arcs():
    points=[dict(x=math.sin(i*.025),y=1-math.cos(i*.025),yaw=i*.025) for i in range(41)]
    assert connector_blocks(points,points[0],points[-1],.69)[0]['direction']==1
    reverse=list(reversed(points))
    assert connector_blocks(reverse,reverse[0],reverse[-1],.69)[0]['direction']==-1


def test_entry_is_not_repeated_or_counted_as_an_extra_patrol_pass(teaching):
    comp=teaching;saved=finish_two_segments(comp)
    route=comp.store.get_route(saved['route_id']);doc=saved['network']['document']
    comp.live.update(x=-.5,v=0.)
    entry=choose_entry(doc,comp.live,line,lambda blocks:None)
    assert entry[0]==0
    plan=plan_path(doc,route.steps,comp.live,3,entry=entry)
    assert plan['entry']['required'] and plan['entry']['length']==pytest.approx(.5)
    assert plan_leg(plan,1,0)['blocks'][0]['entry']
    assert not plan_leg(plan,2,0)['blocks'] and not plan_leg(plan,3,0)['blocks']
    assert plan['pass_step_indices']==[[0,1,2],[2,1,0]]
    assert [p['x'] for p in plan_leg(plan,1,1)['blocks'][0]['samples']]==[
        p['x'] for p in plan_leg(plan,3,1)['blocks'][0]['samples']]


def test_unreachable_nearest_endpoint_falls_back_to_other_end(teaching):
    comp=teaching;saved=finish_two_segments(comp);doc=saved['network']['document']
    route=comp.store.get_route(saved['route_id']);comp.live.update(x=-.5)
    calls=[]
    def compute(start,target):
        calls.append(target['x'])
        if target['x']==0.:raise PatrolError('ENTRY_UNREACHABLE','障碍物')
        return line(start,target)
    index,blocks=choose_entry(doc,comp.live,compute,lambda blocks:None)
    assert calls==[0.,2.] and index==2
    plan=plan_path(doc,route.steps,comp.live,2,entry=(index,blocks))
    assert plan['pass_step_indices']==[[2,1,0],[0,1,2]]
    assert plan_leg(plan,1,0)['target']['x']==2.
    assert plan_leg(plan,1,1)['blocks'][0]['direction']==-1


def test_aligned_endpoint_needs_no_planner_but_wrong_heading_does(teaching):
    comp=teaching;saved=finish_two_segments(comp);doc=saved['network']['document']
    calls=[]
    def fail(start,target):
        calls.append(target)
        raise PatrolError('ENTRY_UNREACHABLE','无可用转弯空间')
    assert choose_entry(doc,dict(x=0.,y=0.,yaw=0.),fail,lambda b:None)==(0,[])
    assert not calls
    with pytest.raises(PatrolError,match='无法自动接入'):
        choose_entry(doc,dict(x=0.,y=0.,yaw=math.pi),fail,lambda b:None)
    assert len(calls)==2


def test_full_body_collision_rejects_entry_even_if_centerline_is_free(teaching):
    comp=teaching;saved=finish_two_segments(comp);doc=saved['network']['document']
    comp.live.update(x=-1.)
    grid=comp.node._latest_map
    grid.data[round((.20+4)/.05)*240+round((-.5+4)/.05)]=100
    def check(blocks):
        comp._check_map_collision(dict(edges={'e':dict(name='接入',blocks=blocks)}))
    with pytest.raises(PatrolError,match='车身扫掠'):
        choose_entry(doc,comp.live,line,check)


def test_preview_keeps_callbacks_responsive_and_rejects_pose_change(teaching):
    comp=teaching;saved=finish_two_segments(comp)
    comp.planning_lock=threading.Lock()
    comp.live.update(x=-.5)
    entered,release=threading.Event(),threading.Event()
    def entry(doc,current):
        entered.set();assert release.wait(3)
        return choose_entry(doc,current,line,lambda b:None)
    comp._path_entry=entry
    result=[]
    def plan():
        req=NS(operation='PLAN',payload_json='{"route_id":"'+saved['route_id']+'"}')
        result.append(comp.command(req,NS()))
    thread=threading.Thread(target=plan);thread.start()
    assert entered.wait(1)
    # STATUS must finish while the planner is still waiting.
    before=time.monotonic()
    assert comp.command(NS(operation='STATUS',payload_json='{}'),NS()).success
    assert time.monotonic()-before<.2
    with pytest.raises(PatrolError,match='正在规划'):
        comp.preview(saved['route_id'],1)
    comp.live['x']+=.3
    release.set();thread.join(3)
    assert not thread.is_alive() and result[0].error_code=='OFF_ROAD' and not comp.plans


def test_entry_seam_on_closed_route_keeps_all_recorded_roads(teaching):
    from robot_patrol.recorded_routes.inspection_path import prepare_path,route_point_ids
    from robot_patrol.recorded_routes.geometry import motion_blocks
    # One complete circle is one recorded edge, with its first point repeated.
    samples=[dict(x=math.sin(i*math.pi/80),y=1-math.cos(i*math.pi/80),yaw=i*math.pi/80)
             for i in range(161)]
    node=dict(samples[0],id='a',name='起点')
    doc=dict(nodes={'a':node},edges={'circle':dict(id='circle',name='圆环',**{'from':'a','to':'a'},
             blocks=motion_blocks(samples),speed=.2)},bindings={},vehicle={'minimum_turning_radius':.69},
             inspection_path={'edge_ids':['circle'],'closed':True})
    prepare_path(doc,'圆环',True)
    steps=[NS(point_id=p) for p in route_point_ids(doc)]
    start=dict(x=-.5,y=0.,yaw=0.)
    entry=choose_entry(doc,start,line,lambda b:None)
    plan=plan_path(doc,steps,start,2,entry=entry)
    assert len(plan_leg(plan,1,0)['blocks'])==2
    assert plan_leg(plan,1,0)['blocks'][0]['entry']
    assert len(plan_leg(plan,2,0)['blocks'])==1
    assert plan_leg(plan,1,0)['blocks'][1]==plan_leg(plan,2,0)['blocks'][0]
