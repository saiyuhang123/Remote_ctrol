from types import SimpleNamespace
import copy
import json
import math
from pathlib import Path
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


def timed_pose(x, y=0., yaw=0., t=0., v=0.):
    return dict(x=x, y=y, yaw=yaw, t=t, v=v)


def test_recorded_stationary_noise_does_not_invent_reverse_legs():
    raw = [timed_pose(0., t=1.)]
    for i in range(1, 21):
        raw.append(timed_pose(.004*math.sin(i), .006*math.cos(i),
                              t=1.+i*.1, v=[.04, -.05, 0.][i % 3]))
    for i in range(1, 51):
        raw.append(timed_pose(i*.02, .002*math.sin(i), t=3.+i*.1, v=.2))
    for i in range(1, 301):
        raw.append(timed_pose(1.+.026*math.sin(i*.3), .012*math.cos(i*.7),
                              .001*math.sin(i), 8.+i*.1,
                              [.015, -.025, .055, -.06, 0., .022][i % 6]))
    raw.append(timed_pose(1.002, .005, t=38.1))
    original = copy.deepcopy(raw)
    blocks = motion_blocks(raw)
    validate_blocks(blocks, .69)
    assert [b['direction'] for b in blocks] == [1]
    assert blocks[0]['samples'][0]['x'] == raw[0]['x']
    assert blocks[-1]['samples'][-1] == {k: raw[-1][k] for k in ('x', 'y', 'yaw')}
    assert raw == original  # The persisted draft remains available for diagnosis.


@pytest.mark.parametrize('reverse_speed', [.1, .025])
def test_short_real_reversal_after_long_stop_is_retained(reverse_speed):
    raw = [timed_pose(i*.01, t=1.+i*.1, v=.1) for i in range(101)]
    for i in range(1, 301):
        raw.append(timed_pose(1., t=11.+i*.1, v=[.015, -.015, 0.][i % 3]))
    for i in range(1, 6):
        raw.append(timed_pose(1.-i*.01, t=41.+i*.01/reverse_speed, v=-reverse_speed))
    for i in range(1, 46):
        raw.append(timed_pose(.95+i*.01, t=41.+.05/reverse_speed+i*.1, v=.1))
    blocks = motion_blocks(raw)
    validate_blocks(blocks, .69)
    assert [b['direction'] for b in blocks] == [1, -1, 1]
    assert blocks[0]['samples'][-1]['x'] == pytest.approx(1.)
    assert blocks[1]['samples'][-1]['x'] == pytest.approx(.95)
    assert blocks[0]['samples'][-1] == blocks[1]['samples'][0]
    assert blocks[1]['samples'][-1] == blocks[2]['samples'][0]


def test_stopped_noise_cannot_be_saved_as_a_road():
    raw = [timed_pose(.012*math.sin(i), .01*math.cos(i), t=1.+i*.1,
                      v=[.03, -.04, 0.][i % 3]) for i in range(300)]
    with pytest.raises(PatrolError, match='有效位移|长度不足'):
        motion_blocks(raw)


def test_large_lateral_drift_at_stop_is_not_hidden_by_zero_velocity():
    raw = [timed_pose(i*.02, t=1.+i*.1, v=.2) for i in range(51)]
    raw.extend([timed_pose(1., .06, t=6.2), timed_pose(1., .125, t=6.5)])
    raw.extend(timed_pose(1., .10, t=7.+i*.1) for i in range(100))
    with pytest.raises(PatrolError, match='位移方向.*横移'):
        motion_blocks(raw)


def test_short_lateral_pose_correction_at_gear_cusp_does_not_reject_the_road():
    # 2026-10-20 live recording: 5.3 cm step with 4.9 cm lateral component at a
    # reverse/forward cusp, measured while reversing at 0.28 m/s. The step is
    # shorter than DIRECTION_BASELINE, so it cannot establish a sideways road;
    # the surrounding per-gear windows must still be accepted.
    raw = [timed_pose(*p) for p in [
        (-.520048, -5.334984, -.782478, 1.000000, .658290),
        (-.401879, -5.439109, -.682269, 1.200520, .754555),
        (-.334799, -5.492902, -.649387, 1.300081, .865089),
        (-.281984, -5.536322, -.628711, 1.401497, .675308),
        (-.213477, -5.618478, -.614169, 1.599682, .517471),
        (-.189574, -5.660193, -.614420, 1.800025, .204581),
        (-.208739, -5.629403, -.614146, 1.999287, -.313131),
        (-.230057, -5.565998, -.615464, 2.301120, -.276542),
        (-.218397, -5.514201, -.646377, 2.500266, -.113310),
        (-.159501, -5.516015, -.716342, 2.699731, .301270),
        (-.101292, -5.548072, -.773181, 2.799341, .643533),
        (-.055684, -5.589162, -.843002, 2.900044, .615995),
        (-.012727, -5.645452, -.910345, 3.000922, .698647),
        (.038059, -5.749504, -.970671, 3.103838, .797196),
        (.061994, -5.813979, -1.021659, 3.200384, .671959),
        (.073268, -5.861173, -1.058735, 3.301616, .466405),
    ]]
    blocks = motion_blocks(raw)
    validate_blocks(blocks, .69)
    assert [b['direction'] for b in blocks] == [1, -1, 1]
    assert blocks[-1]['samples'][-1] == {k: raw[-1][k] for k in ('x', 'y', 'yaw')}
    doc = dict(schema_version=1, nodes={}, edges={}, bindings={}, allowed_polygon=[],
               vehicle=dict(minimum_turning_radius=.69,
                            footprint=[[.4475,.28],[.4475,-.28],[-.4475,-.28],[-.4475,.28]]))
    doc['nodes']['a'] = dict(blocks[0]['samples'][0], id='a', name='a')
    doc['nodes']['b'] = dict(blocks[-1]['samples'][-1], id='b', name='b')
    doc['edges']['r'] = dict(id='r', name='r', **{'from': 'a', 'to': 'b'},
                             blocks=blocks, speed=.2, enabled=True)
    validate_network(doc, .69)


@pytest.mark.parametrize('gear', [1, -1])
@pytest.mark.parametrize('rotation', [0., 2.7])
def test_real_braking_rebound_preserves_settled_endpoint_without_false_reverse(gear, rotation):
    # A reduced real recording: near-zero wheel feedback, a 12 cm pose rebound
    # after braking, then a stable endpoint. Previously the final 11.7 cm
    # chord became an invalid reverse leg and blocked the entire recording.
    raw = json.loads((Path(__file__).parent/'data/recorded_stop_rebound.json').read_text())
    c, s = math.cos(rotation), math.sin(rotation)
    for p in raw:
        x, y = gear*p['x'], gear*p['y']
        p.update(x=c*x-s*y+2., y=s*x+c*y-3., yaw=p['yaw']+rotation, v=gear*p['v'])
    original = copy.deepcopy(raw)
    blocks = motion_blocks(raw)
    validate_blocks(blocks, .69)
    assert [b['direction'] for b in blocks] == [gear]
    for actual, measured in [(blocks[0]['samples'][0], raw[0]), (blocks[-1]['samples'][-1], raw[-1])]:
        assert actual['x'] == measured['x'] and actual['y'] == measured['y']
        assert math.atan2(math.sin(actual['yaw']-measured['yaw']),
                          math.cos(actual['yaw']-measured['yaw'])) == pytest.approx(0.)
    assert raw == original


@pytest.mark.parametrize('speed', [.1, .025, .009])
def test_terminal_real_reverse_is_not_filtered_as_braking_rebound(speed):
    raw = [timed_pose(i*.01, t=1.+i*.1, v=.1) for i in range(101)]
    raw.extend(timed_pose(1.-i*.01, t=11.+i*.01/speed, v=-speed) for i in range(1, 7))
    t = raw[-1]['t']
    raw.extend(timed_pose(.94, t=t+i*.1) for i in range(1, 21))
    blocks = motion_blocks(raw)
    validate_blocks(blocks, .69)
    assert [b['direction'] for b in blocks] == [1, -1]
    assert blocks[0]['samples'][-1]['x'] == pytest.approx(1.)
    assert blocks[-1]['samples'][-1]['x'] == pytest.approx(.94)


@pytest.mark.parametrize('kind', ['lateral_shift', 'moving', 'unsettled'])
def test_terminal_rebound_filter_does_not_hide_invalid_motion(kind):
    raw = json.loads((Path(__file__).parent/'data/recorded_stop_rebound.json').read_text())
    if kind == 'lateral_shift':
        # Keep the rebound's longitudinal component but move the final stopped
        # pose outside the observed approach corridor.
        for p in raw[-10:]:
            p['y'] += .12
    elif kind == 'moving':
        raw[-1]['v'] = .1
    else:
        # No settled terminal hold; the same transient remains invalid.
        raw = raw[:30]
    with pytest.raises(PatrolError):
        motion_blocks(raw)


def test_recorder_retains_stationary_feedback_for_stop_confirmation():
    r = Recorder('map', 1)
    for i in range(51):
        r.add(timed_pose(0., t=1.+i*.1))
    assert len(r.samples) >= 10
    assert max(b['t']-a['t'] for a,b in zip(r.samples, r.samples[1:])) <= .6
    with pytest.raises(PatrolError, match='有效位移|长度不足'):
        r.finish(.69)


@pytest.mark.parametrize('kind', ['sideways', 'gap', 'rotation'])
def test_recording_denoising_keeps_invalid_motion_checks(kind):
    if kind == 'sideways':
        raw = [timed_pose(0., i*.01, t=1.+i*.1, v=.1) for i in range(31)]
    elif kind == 'gap':
        raw = [timed_pose(0., t=1.), timed_pose(.8, t=2.)]
    else:
        raw = [timed_pose(0., yaw=i*.2, t=1.+i*.1) for i in range(5)]
    with pytest.raises(PatrolError):
        motion_blocks(raw)


@pytest.mark.parametrize('radius,valid', [(.3,False), (.9,True)])
def test_timed_recordings_still_obey_turning_radius(radius, valid):
    raw = [timed_pose(radius*math.sin(i*.04), radius*(1-math.cos(i*.04)),
                      yaw=i*.04, t=1.+i*.1, v=radius*.4) for i in range(81)]
    blocks = motion_blocks(raw)
    if valid:
        validate_blocks(blocks,.69)
    else:
        with pytest.raises(PatrolError,match='曲率'):
            validate_blocks(blocks,.69)


def test_brief_yaw_blip_is_not_a_sustained_tight_turn():
    # A 15 deg one-frame yaw blip at a segment start is 1.75 rad/m over the
    # old 15 cm baseline, but only 1.05 rad/m over the 25 cm baseline.
    samples = [dict(x=i*.05, y=0., yaw=0.) for i in range(9)]
    for p in samples[3:]:
        p['yaw'] = math.radians(15.)
    validate_blocks([{'direction': 1, 'samples': samples}], .69)


def test_sustained_over_curvature_is_still_rejected():
    samples = [dict(x=i*.05, y=0., yaw=i*.05*2.) for i in range(9)]
    with pytest.raises(PatrolError, match='曲率'):
        validate_blocks([{'direction': 1, 'samples': samples}], .69)


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
    comp._idle=lambda:None
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


def test_segment_anchor_uses_continuous_odom_pose():
    from robot_patrol.recorded_routes.component import RoadComponent
    import time as clock
    comp = RoadComponent.__new__(RoadComponent)
    calls = []

    class Node:
        def _current_pose(self):
            calls.append('map')
            return 10., 20., .5, 1

        def _current_odom_pose(self):
            calls.append('odom')
            return 1., 2., .3, 2

        def latest_odom(self):
            value = SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
                linear=SimpleNamespace(x=.2))))
            return value, clock.monotonic()

    comp.node = Node()
    comp.anchor = None
    live = comp.current()
    assert (live['x'], live['y']) == (10., 20.)
    calls.clear()

    target = dict(x=4., y=-1., yaw=.9)
    anchor = comp._capture_anchor(target)
    first = comp.current()
    assert first['x'] == pytest.approx(4.)
    assert first['y'] == pytest.approx(-1.)
    assert first['yaw'] == pytest.approx(.9)
    assert 'map' not in calls  # The ICP-corrected map pose no longer contributes.

    # A 10 cm odom translation stays 10 cm in the anchored map frame; its
    # direction is the frozen anchor yaw, not a live map->odom correction.
    moved = comp.anchored_pose(anchor, 1.1, 2., .3)
    dx, dy = moved['x']-first['x'], moved['y']-first['y']
    assert math.hypot(dx, dy) == pytest.approx(.1)
    assert math.atan2(dy, dx) == pytest.approx(anchor['yaw'])


def test_anchor_prefers_lio_pose_over_drifting_wheel_imu_ekf():
    from robot_patrol.recorded_routes.component import RoadComponent
    import time as clock
    comp = RoadComponent.__new__(RoadComponent)

    class Node:
        def latest_lio_odom(self):
            message = SimpleNamespace(
                pose=SimpleNamespace(pose=SimpleNamespace(
                    position=SimpleNamespace(x=1., y=2.),
                    orientation=SimpleNamespace(x=0., y=0.,
                                                z=math.sin(.15), w=math.cos(.15)))),
                header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
            return message, clock.monotonic()

        def _current_odom_pose(self):
            raise AssertionError('EKF odom must not anchor a wheel_imu segment')

        def _current_pose(self):
            return 10., 20., .5, 1

        def latest_odom(self):
            value = SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
                linear=SimpleNamespace(x=.2))))
            return value, clock.monotonic()

    comp.node = Node()
    comp.anchor = None
    comp._capture_anchor(dict(x=4., y=-1., yaw=.9))
    first = comp.current()
    assert (first['x'], first['y']) == pytest.approx((4., -1.))
    assert first['yaw'] == pytest.approx(.9)


def test_auto_stop_holds_the_stop_pose_instead_of_recording_drift():
    from robot_patrol.recorded_routes.component import RoadComponent
    from robot_patrol.recorded_routes.recorder import Recorder
    comp = RoadComponent.__new__(RoadComponent)
    comp.stationary = False
    comp.stop_since = 0.
    comp.stop_pose = None
    comp.stop_map = None
    comp.moving_ticks = 0
    comp.node = SimpleNamespace(_current_pose=lambda: (.07, 0., 0., 1))
    r = Recorder('m', 1)
    comp._freeze_at_stop(r, dict(x=0., y=0., yaw=0., t=1., v=0.))
    comp._freeze_at_stop(r, dict(x=.07, y=0., yaw=0., t=1.6, v=0.))
    assert len(r.samples) == 2
    assert r.samples[-1]['x'] == 0.  # Held stop pose, not the drifted pose.
    assert r.samples[-1]['t'] == 1.6  # The sample clock stays continuous.
    assert comp.stop_map['x'] == .07


def test_auto_stop_split_requires_distance_and_real_pose_drift():
    from robot_patrol.recorded_routes.component import RoadComponent
    from robot_patrol.recorded_routes.recorder import Recorder
    import time
    comp = RoadComponent.__new__(RoadComponent)
    comp.path_mode = True
    comp.stationary = True
    comp.stop_since = time.monotonic()-1.
    r = Recorder('m', 1)
    r.samples = [dict(x=i*.05, y=0., yaw=0., t=1.+i*.1, v=.2) for i in range(21)]
    assert comp._stop_split_needed(r, dict(x=1.07, y=0., yaw=0.))
    assert not comp._stop_split_needed(r, dict(x=1.02, y=0., yaw=0.))  # No drift.
    comp.stationary = False
    assert not comp._stop_split_needed(r, dict(x=1.07, y=0., yaw=0.))


def test_segment_end_pulls_back_to_live_map_without_a_step():
    from robot_patrol.recorded_routes.component import RoadComponent
    from robot_patrol.recorded_routes.recorder import Recorder
    comp = RoadComponent.__new__(RoadComponent)
    comp.anchor = {'x': 0., 'y': 0., 'yaw': 0.}
    comp.node = SimpleNamespace(_current_pose=lambda: (2., .1, 0., 1))
    r = Recorder('m', 1)
    r.samples = [dict(x=i*.05, y=0., yaw=0., t=1.+i*.1, v=.2) for i in range(41)]
    comp._close_anchor_gap(r)
    assert r.samples[0]['y'] == 0.
    assert r.samples[-1]['x'] == pytest.approx(2.)
    assert r.samples[-1]['y'] == pytest.approx(.1)
    # The 10 cm map correction is ramped over 2 m, not left as a step.
    blocks = motion_blocks(r.samples)
    validate_blocks(blocks, .69)


def test_segment_end_gap_beyond_drift_is_rejected():
    from robot_patrol.recorded_routes.component import RoadComponent
    from robot_patrol.recorded_routes.recorder import Recorder
    comp = RoadComponent.__new__(RoadComponent)
    comp.anchor = {'x': 0., 'y': 0., 'yaw': 0.}
    comp.node = SimpleNamespace(_current_pose=lambda: (2., .2, 0., 1))
    r = Recorder('m', 1)
    r.samples = [dict(x=i*.05, y=0., yaw=0., t=1.+i*.1, v=.2) for i in range(41)]
    with pytest.raises(PatrolError, match='定位偏差'):
        comp._close_anchor_gap(r)


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
