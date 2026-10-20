"""Opt-in real Nav2/guard/mux tests. All velocity goes to an inert simulator.

ROS_DOMAIN_ID=209 ROS_LOCALHOST_ONLY=1 ROBOT_RECORDED_NAV2_INTEGRATION=1 pytest ...
No chassis driver/bridge or hardware launch is ever started.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get('ROBOT_RECORDED_NAV2_INTEGRATION') != '1',
                                reason='explicit isolated real Nav2 integration required')


def wait(predicate, timeout=15):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        value=predicate()
        if value:
            return value
        time.sleep(.03)
    raise AssertionError('timed out waiting for real ROS state')


@pytest.fixture(scope='module')
def runtime(tmp_path_factory):
    assert 1 <= int(os.environ.get('ROS_DOMAIN_ID','0')) <= 232
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionClient
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from patrol_interfaces.msg import OperatorHeartbeat
    from patrol_interfaces.action import RunRoute
    from robot_patrol.node import PatrolNode
    from robot_patrol.guard import PatrolGuard
    from robot_patrol.map_metadata import read_map_binding
    from chassis_nav2.recorded_routes import nav2_overlay,clearance_rewrites
    from ament_index_python.packages import get_package_share_directory
    from PIL import Image

    tmp=tmp_path_factory.mktemp('real_recorded_nav2')
    source=Path(__file__).resolve().parents[2]/'chassis_nav2'
    config=source/'config/nav2.yaml'
    hw=yaml.safe_load((source/'config/hardware.yaml').read_text())
    image=Image.new('L',(320,320),255)
    image.save(tmp/'map.pgm')
    map_yaml=tmp/'map.yaml'
    map_yaml.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-8.,-8.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    empty_bt=tmp/'default.xml'
    empty_bt.write_text('<root main_tree_to_execute="Empty"><BehaviorTree ID="Empty"><AlwaysFailure/></BehaviorTree></root>')
    params=yaml.safe_load(config.read_text())
    for name in ('controller_server','bt_navigator','planner_server'):
        params[name]['ros__parameters'].update(nav2_overlay(name,config,hw))
    params['bt_navigator']['ros__parameters'].update(plugin_lib_names=['recorded_route_bt'],
       default_nav_to_pose_bt_xml=str(empty_bt),default_nav_through_poses_bt_xml=str(empty_bt))
    params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']=.69
    footprint=json.dumps(hw['robot_geometry']['footprint'])
    for name in ('local_costmap','global_costmap'):
        cost=params[name][name]['ros__parameters']
        cost.update(footprint=footprint,update_frequency=15.,publish_frequency=8.,resolution=.05)
        if name=='local_costmap':
            cost.update(width=6,height=6,plugins=['static_layer','obstacle_layer','inflation_layer'])
            cost['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
        cost['obstacle_layer']['scan'].update(topic='/scan',inf_is_valid=True,raytrace_max_range=8.,obstacle_max_range=7.8)
        cost['inflation_layer'].update({key:float(value) for key,value in clearance_rewrites().items()})
    params['map_server']={'ros__parameters':{'yaml_filename':str(map_yaml),'use_sim_time':False}}
    params['lifecycle_manager']={'ros__parameters':{'autostart':True,'node_names':['map_server','controller_server','planner_server','bt_navigator'],'bond_timeout':4.}}
    params_path=tmp/'nav2.yaml';params_path.write_text(yaml.safe_dump(params))
    processes=[];logs=[]
    def start(package, executable, extra=()):
        log=(tmp/(executable+'.log')).open('w');logs.append(log)
        process=subprocess.Popen(['ros2','run',package,executable,'--ros-args',*extra],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
    rclpy.init()

    class Sim(Node):
        def __init__(self):
            super().__init__('inert_recorded_road_simulator')
            self.x=self.y=self.yaw=0.;self.v=self.w=0.;self.last=0.;self.history=[]
            self.command_delay=0.;self.speed_deadband=0.;self.steering_rate=0.;self.steer=0.;self.pending_commands=[]
            self.operator=True;self.obstacles=[];self.guard_ready=False;self.progress={}
            self.controller_progress=[];self.controller_faults=[]
            self.broadcaster=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.;tf.header.stamp=self.get_clock().now().to_msg();self.static.sendTransform(tf)
            self.odom=self.create_publisher(Odometry,'/odom',10)
            self.scan=self.create_publisher(LaserScan,'/scan',10)
            self.estop=self.create_publisher(Bool,'/emergency_stop',10)
            self.heartbeat=self.create_publisher(OperatorHeartbeat,'/patrol/operator_heartbeat',10)
            self.create_subscription(Twist,'/inert_sim/cmd_vel',self.command,10)
            self.create_subscription(Bool,'/patrol/guard_ready',lambda msg:setattr(self,'guard_ready',msg.data),10)
            from std_msgs.msg import String
            self.create_subscription(String,'/patrol/recorded_progress',self.on_progress,10)
            self.create_subscription(String,'/patrol/recorded_controller_progress',
                                     lambda msg:self.controller_progress.append(json.loads(msg.data)),10)
            self.create_subscription(String,'/patrol/recorded_controller_fault',
                                     lambda msg:self.controller_faults.append(json.loads(msg.data)),10)
            self.tick_count=0;self.create_timer(.02,self.tick)
        def on_progress(self,msg):
            self.progress=json.loads(msg.data)
        def command(self,msg):
            now=time.monotonic()
            self.last=now
            self.pending_commands.append((now+self.command_delay,msg.linear.x,msg.angular.z))
        def tick(self):
            now=time.monotonic()
            while self.pending_commands and self.pending_commands[0][0]<=now:
                _,self.v,self.w=self.pending_commands.pop(0)
            # Same curvature saturation as the Ackermann chassis, without
            # starting the real bridge or any serial/CAN drivers.
            self.w=max(-abs(self.v)/.69,min(abs(self.v)/.69,self.w))
            v,w=(self.v,self.w) if time.monotonic()-self.last<.3 else (0.,0.)
            if abs(v)<self.speed_deadband:v=w=0.
            if self.steering_rate:
                target=math.atan(.5*w/(1.63*v)) if abs(v)>.001 else 0.
                step=math.radians(self.steering_rate)*.02
                self.steer+=max(-step,min(step,target-self.steer))
                w=v*1.63*math.tan(self.steer)/.5
            self.x+=v*math.cos(self.yaw)*.02;self.y+=v*math.sin(self.yaw)*.02;self.yaw+=w*.02
            self.history.append((self.x,self.y,self.yaw,v,w))
            stamp=self.get_clock().now().to_msg()
            tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=stamp
            tf.transform.translation.x=self.x;tf.transform.translation.y=self.y
            tf.transform.rotation.z=math.sin(self.yaw/2);tf.transform.rotation.w=math.cos(self.yaw/2)
            self.broadcaster.sendTransform(tf)
            msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.header.stamp=stamp
            msg.pose.pose.position.x=self.x;msg.pose.pose.position.y=self.y;msg.pose.pose.orientation=tf.transform.rotation
            msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w;self.odom.publish(msg)
            self.tick_count+=1
            if self.tick_count%5==0:
                self.estop.publish(Bool(data=False))
                self.heartbeat.publish(OperatorHeartbeat(session_id='recorded_test_owner',client_id='test',enabled=self.operator,stamp=stamp))
                scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=stamp
                scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=math.pi/180;scan.range_min=.05;scan.range_max=8.
                ranges=[]
                for k in range(361):
                    theta=self.yaw-math.pi+k*math.pi/180;ux,uy=math.cos(theta),math.sin(theta)
                    hit=math.inf
                    for ox,oy,radius in self.obstacles:
                        dx,dy=ox-self.x,oy-self.y;along=dx*ux+dy*uy
                        perp=dx*dx+dy*dy-along*along
                        if along>0 and perp<radius*radius:
                            hit=min(hit,along-math.sqrt(radius*radius-perp))
                    ranges.append(float(hit))
                scan.ranges=ranges;self.scan.publish(scan)

    sim=Sim()
    patrol=PatrolNode(parameter_overrides=[Parameter('database_path',value=str(tmp/'patrol.sqlite3')),
        Parameter('web_map_dir',value=str(tmp/'webmap')),Parameter('enable_recorded_routes',value=True),
        Parameter('point_timeout_sec',value=120.),Parameter('recorded_obstacle_wait_timeout',value=60.)])
    mapping=patrol._store.upsert_map(read_map_binding(str(map_yaml),'测试','amcl',''))
    patrol._active_map_id=mapping.map_id;patrol._store.set_active_map(mapping.map_id)
    guard=PatrolGuard()
    executor=MultiThreadedExecutor(num_threads=10)
    for node in (sim,patrol,guard): executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        mux_config=source/'config/twist_mux_patrol.yaml'
        start('twist_mux','twist_mux',['--params-file',str(mux_config),'-r','cmd_vel_out:=/inert_sim/cmd_vel'])
        for package,exe in [('nav2_map_server','map_server'),('nav2_controller','controller_server'),
                            ('nav2_planner','planner_server'),('nav2_bt_navigator','bt_navigator')]:
            extra=['--params-file',str(params_path)]
            if exe=='controller_server':extra+=['-r','cmd_vel:=/cmd_vel_auto']
            start(package,exe,extra)
        start('nav2_lifecycle_manager','lifecycle_manager',['--params-file',str(params_path)])
        client=ActionClient(sim,RunRoute,'/patrol/run_route')
        wait(lambda:sim.guard_ready,40)
        wait(lambda:patrol._latest_map is not None)
        yield SimpleNamespace(sim=sim,patrol=patrol,guard=guard,client=client,mapping=mapping,tmp=tmp)
    finally:
        (tmp/'trace.json').write_text(json.dumps({'history':sim.history,'progress':sim.progress,
            'controller_progress':sim.controller_progress,'controller_faults':sim.controller_faults}))
        if patrol._active_engine:
            patrol._active_engine.request_cancel()
            try:wait(lambda:patrol._active_engine is None,10)
            except AssertionError:pass
        for process in processes:
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2)
        for log in logs:log.close()
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        for node in (sim,patrol,guard):node.destroy_node()
        rclpy.shutdown()
        print('Real Nav2 logs:',tmp)


def make_route(rt, coords, stop_indices, loops=1, close_loop=False, blocks_override=None):
    from robot_patrol.recorded_routes.geometry import motion_blocks
    from robot_patrol.recorded_routes.graph import NetworkPlanner
    from robot_patrol.models import new_id
    doc={'schema_version':1,'nodes':{},'edges':{},'bindings':{},'map_stamp':rt.patrol.roads.map_stamp(rt.mapping),
         'vehicle':rt.patrol.roads.vehicle_stamp()}
    # Each test recording explicitly supplies its path, including direction.
    for i,p in enumerate(coords):doc['nodes'][str(i)]=dict(p,id=str(i),name=str(i))
    for i in range(len(coords)-1):
        a,b=coords[i:i+2];n=max(2,math.ceil(math.hypot(b['x']-a['x'],b['y']-a['y'])/.04))
        pts=[dict(x=a['x']+(b['x']-a['x'])*j/n,y=a['y']+(b['y']-a['y'])*j/n,
                  yaw=a['yaw']+(b['yaw']-a['yaw'])*j/n) for j in range(n+1)]
        doc['edges'][str(i)]={'id':str(i),'name':str(i),'from':str(i),'to':str(i+1),'blocks':motion_blocks(pts),'speed':.2}
        if blocks_override is not None:
            assert len(coords)==2
            doc['edges'][str(i)]['blocks']=blocks_override
    if close_loop:
        assert coords[0]==coords[-1]
        doc['edges'][str(len(coords)-2)]['to']='0'
        doc['nodes'].pop(str(len(coords)-1))
    points=[]
    for index in stop_indices:
        p=coords[index]
        point=rt.patrol._store.upsert_point('',rt.mapping.map_id,str(index),p['x'],p['y'],p['yaw'],'test',1)
        doc['bindings'][point.point_id]=['0' if close_loop and index==len(coords)-1 else str(index)];points.append(point)
    net=rt.patrol._store.save_network(rt.mapping.map_id,'测试录制',doc,published=True)
    route=rt.patrol._store.upsert_route('',rt.mapping.map_id,'测试任务',
        [SimpleNamespace(point_id=p.point_id,dwell_seconds=0) for p in points],navigation_mode='RECORDED_NETWORK',
        network_id=net['network_id'],network_revision=net['revision'])
    plan=rt.patrol.roads.preview(route.route_id,loops)
    return route,plan


def start_route(rt,route,plan,loops=1):
    from patrol_interfaces.action import RunRoute
    goal=RunRoute.Goal(request_id='test_'+str(time.time_ns()),owner_session_id='recorded_test_owner',
        route_id=route.route_id,map_id=rt.mapping.map_id,route_revision=route.revision,map_revision=rt.mapping.revision,
        loop_count=loops,plan_digest=plan['digest'])
    future=rt.client.send_goal_async(goal);wait(future.done)
    handle=future.result();assert handle.accepted
    return handle


def make_teaching_route(rt, end_x=.8, loops=1):
    from robot_patrol.recorded_routes.geometry import motion_blocks
    from robot_patrol.recorded_routes.inspection_path import prepare_path
    doc=dict(schema_version=1, nodes={}, edges={}, bindings={}, allowed_polygon=[],
             map_stamp=rt.patrol.roads.map_stamp(rt.mapping), vehicle=rt.patrol.roads.vehicle_stamp(),
             inspection_path=dict(edge_ids=['ab'], closed=False))
    doc['nodes']={'a':dict(id='a',name='A',x=0.,y=0.,yaw=0.),
                  'b':dict(id='b',name='B',x=end_x,y=0.,yaw=0.)}
    pts=[dict(x=end_x*i/40,y=0.,yaw=0.) for i in range(41)]
    doc['edges']['ab']=dict(id='ab',name='AB',**{'from':'a','to':'b'},blocks=motion_blocks(pts),speed=.2)
    points=prepare_path(doc,'自动接入测试',make_reverse=True)
    net=rt.patrol._store.save_network(rt.mapping.map_id,'自动接入测试',doc)
    saved=rt.patrol._store.save_path_bundle(net,doc,points,publish=True)
    route=rt.patrol._store.get_route(doc['inspection_path']['route_id'])
    return route,rt.patrol.roads.preview(route.route_id,loops)


def test_real_entry_planner_handles_large_heading_error_without_motion(runtime,monkeypatch):
    rt=runtime;reset_sim(rt)
    from robot_patrol.recorded_routes.entry import EntryPlanner
    original=EntryPlanner.compute
    captures=[]
    def capture(self,start,target):
        points=original(self,start,target)
        captures.append(dict(start=start,target=target,points=points))
        (rt.tmp/'entry_candidates.json').write_text(json.dumps(captures))
        return points
    monkeypatch.setattr(EntryPlanner,'compute',capture)
    rt.sim.x=.11;rt.sim.y=-.03;rt.sim.yaw=-1.8
    time.sleep(.5)
    begin=len(rt.sim.history)
    route,plan=make_teaching_route(rt)
    assert plan['entry']['required']
    assert plan['entry']['length']>.2
    assert plan['initial_legs'][0]['blocks'][-1]['entry_target']['yaw']==0.
    assert all(abs(v)<.001 for _,_,_,v,_ in rt.sim.history[begin:]),'planning must never send motion'
    assert rt.sim.guard_ready,'a long plan must not starve executor heartbeats'


def test_real_entry_then_two_shuttle_passes_never_repeat_connector(runtime):
    rt=runtime;reset_sim(rt,-.4)
    route,plan=make_teaching_route(rt,end_x=.6,loops=2)
    assert plan['entry']['required']
    begin=len(rt.sim.history)
    handle=start_route(rt,route,plan,loops=2)
    result=handle.get_result_async();wait(result.done,65)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    assert result.result().result.completed_loops==2
    trace=rt.sim.history[begin:]
    assert min(x for x,_,_,_,_ in trace)>-.45
    assert -.1<=rt.sim.x<=.25
    assert any(v<-.03 for _,_,_,v,_ in trace)


def test_real_entry_planner_searches_around_obstacles_without_motion(runtime):
    from robot_patrol.recorded_routes.entry import EntryPlanner,connector_blocks
    rt=runtime;reset_sim(rt,-2.)
    rt.sim.obstacles=[(-1.,0.,.12)]
    time.sleep(.8)
    planner=EntryPlanner(rt.patrol)
    try:
        start=rt.patrol.roads.current()
        target=dict(x=0.,y=0.,yaw=0.)
        points=planner.compute(start,target)
        blocks=connector_blocks(points,start,target,.69)
        assert max(abs(p['y']) for b in blocks for p in b['samples'])>.35
        assert abs(rt.sim.x+2.)<.001 and abs(rt.sim.v)<.001
    finally:
        planner.close();rt.sim.obstacles=[]


def test_real_entry_pause_resume_and_cancel_keep_the_same_connector(runtime):
    rt=runtime;reset_sim(rt,-1.0)
    route,plan=make_teaching_route(rt,end_x=.6)
    handle=start_route(rt,route,plan)
    wait(lambda:rt.sim.x>-.85,15)
    control(rt,1)
    wait(lambda:rt.patrol._active_engine and rt.patrol._active_engine.status['state']==7)
    x=rt.sim.x;time.sleep(.4);assert abs(rt.sim.x-x)<.02
    control(rt,2)
    wait(lambda:rt.sim.x>x+.08,15)
    cancel=handle.cancel_goal_async();wait(cancel.done)
    result=handle.get_result_async();wait(result.done,10)
    assert not result.result().result.success
    x=rt.sim.x;time.sleep(.8)
    assert abs(rt.sim.x-x)<.02 and x<0.


def test_real_entry_tracks_large_heading_correction_before_recorded_patrol(runtime):
    rt=runtime;reset_sim(rt)
    rt.sim.x=.11;rt.sim.y=-.03;rt.sim.yaw=-1.8
    time.sleep(.5)
    route,plan=make_teaching_route(rt,end_x=.6)
    assert plan['entry']['required'] and plan['entry']['direction_changes']>=1
    begin=len(rt.sim.history)
    handle=start_route(rt,route,plan)
    result=handle.get_result_async();wait(result.done,80)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    trace=rt.sim.history[begin:]
    assert any(v>.03 for _,_,_,v,_ in trace) and any(v<-.03 for _,_,_,v,_ in trace)
    assert abs(rt.sim.y)<.15 and abs(math.atan2(math.sin(rt.sim.yaw),math.cos(rt.sim.yaw)))<.25
    (rt.tmp/'entry_execution.json').write_text(json.dumps(dict(plan=plan,trace=trace)))


def test_real_entry_with_chassis_delay_deadband_and_steering_rate(runtime):
    rt=runtime;reset_sim(rt)
    rt.sim.command_delay=.10;rt.sim.speed_deadband=.035;rt.sim.steering_rate=45.;rt.sim.steer=0.
    rt.sim.x=.11;rt.sim.y=-.03;rt.sim.yaw=-1.8
    time.sleep(.5)
    try:
        route,plan=make_teaching_route(rt,end_x=.6)
        begin=len(rt.sim.history);started=time.monotonic()
        handle=start_route(rt,route,plan)
        result=handle.get_result_async();wait(result.done,80)
        trace=rt.sim.history[begin:]
        (rt.tmp/'entry_delayed_chassis.json').write_text(json.dumps(dict(plan=plan,trace=trace,elapsed=time.monotonic()-started)))
        assert result.result().result.success,(result.result().result,rt.sim.progress)
        assert max(v for _,_,_,v,_ in trace)>.12
        assert min(v for _,_,_,v,_ in trace)>=-.12001
        assert max(v for _,_,_,v,_ in trace)<=.20001
        assert abs(rt.sim.y)<.15 and abs(math.atan2(math.sin(rt.sim.yaw),math.cos(rt.sim.yaw)))<.25
    finally:
        rt.sim.command_delay=rt.sim.speed_deadband=rt.sim.steering_rate=0.


def test_real_entry_lateral_error_larger_than_minimum_lookahead(runtime):
    from robot_patrol.recorded_routes.geometry import motion_blocks
    rt=runtime;reset_sim(rt)
    rt.sim.x=.01;rt.sim.y=.10
    time.sleep(.4)
    points=[dict(x=i*.025,y=0.,yaw=0.) for i in range(61)]
    blocks=motion_blocks(points)
    blocks[0].update(entry=True, speed=.12)
    route,plan=make_route(rt,[points[0],points[-1]],[1],blocks_override=blocks)
    begin=len(rt.sim.history)
    handle=start_route(rt,route,plan);result=handle.get_result_async();wait(result.done,55)
    trace=rt.sim.history[begin:]
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    assert min(v for _,_,_,v,_ in trace)>=-.005
    assert abs(rt.sim.y)<.03
    assert all(abs(w)<=abs(v)/.69+1e-6 for _,_,_,v,w in trace)


@pytest.mark.parametrize('case',json.loads(
    (Path(__file__).parent/'data/recorded_entry_failures.json').read_text()),
    ids=lambda case:case['name'])
def test_real_field_entry_paths_with_chassis_delay(runtime,case):
    rt=runtime;reset_sim(rt)
    rt.sim.x=case['start']['x'];rt.sim.y=case['start']['y'];rt.sim.yaw=case['start']['yaw']
    rt.sim.command_delay=.10;rt.sim.speed_deadband=.035;rt.sim.steering_rate=45.;rt.sim.steer=0.
    time.sleep(.5)
    try:
        route,plan=make_route(rt,[case['start'],case['target']],[1],blocks_override=case['blocks'])
        begin=len(rt.sim.history)
        handle=start_route(rt,route,plan);result=handle.get_result_async();wait(result.done,65)
        assert result.result().result.success,(result.result().result,rt.sim.progress)
        trace=rt.sim.history[begin:]
        assert math.hypot(rt.sim.x-case['target']['x'],rt.sim.y-case['target']['y'])<.04
        assert abs(math.atan2(math.sin(rt.sim.yaw-case['target']['yaw']),
                              math.cos(rt.sim.yaw-case['target']['yaw'])))<.10
        if all(b['direction']==1 for b in case['blocks']):
            assert min(p[3] for p in trace)>=-.005
        (rt.tmp/(case['name']+'.json')).write_text(json.dumps(dict(plan=plan,trace=trace)))
    finally:
        rt.sim.command_delay=rt.sim.speed_deadband=rt.sim.steering_rate=0.


@pytest.mark.parametrize('reverse_length',[.05,.065364210706185])
def test_real_short_reverse_is_executed_after_precision_cusp_stop(runtime,reverse_length):
    from robot_patrol.recorded_routes.geometry import motion_blocks
    rt=runtime;reset_sim(rt)
    points=[]
    for start,end in [(0.,.6),(.6,.6-reverse_length),(.6-reverse_length,.8)]:
        count=math.ceil(abs(end-start)/.01)
        for i in range(count+1):
            p=dict(x=start+(end-start)*i/count,y=0.,yaw=0.)
            if not points or p!=points[-1]:points.append(p)
    blocks=motion_blocks(points)
    assert [b['direction'] for b in blocks]==[1,-1,1]
    route,plan=make_route(rt,[points[0],points[-1]],[1],blocks_override=blocks)
    begin=len(rt.sim.history)
    handle=start_route(rt,route,plan);result=handle.get_result_async();wait(result.done,45)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    trace=rt.sim.history[begin:]
    reverse=[p for p in trace if p[3]<-.005]
    assert reverse,'short recorded reversal must not be swallowed by goal tolerance'
    reverse_travel=sum(-p[3]*.02 for p in reverse)
    assert reverse_travel>=reverse_length*.70,(reverse_travel,reverse_length)
    first_reverse=next(i for i,p in enumerate(trace) if p[3]<-.005)
    assert sum(abs(p[3])<.005 for p in trace[max(0,first_reverse-60):first_reverse])>=35


def test_real_nav2_continuous_recorded_path_and_guard_uuid(runtime):
    rt=runtime
    route,plan=make_route(rt,[dict(x=0.,y=0.,yaw=0.),dict(x=.6,y=0.,yaw=0.),dict(x=1.2,y=0.,yaw=0.)],[2])
    handle=start_route(rt,route,plan)
    wait(lambda:rt.patrol._active_driver and rt.patrol._active_driver.current_nav_goal_id)
    uuid=rt.patrol._active_driver.current_nav_goal_id
    assert uuid
    result=handle.get_result_async();wait(result.done,30)
    assert result.result().result.success, (result.result().result,rt.sim.progress)
    assert 1.0<=rt.sim.x<=1.4 and abs(rt.sim.y)<.10
    assert not rt.patrol._hazard_driver
    assert rt.sim.progress['state']=='COMPLETED'


def test_real_nav2_explicit_reverse_segment_and_stop(runtime):
    rt=runtime;wait(lambda:rt.sim.guard_ready)
    x,y,yaw=rt.sim.x,rt.sim.y,rt.sim.yaw
    route,plan=make_route(rt,[dict(x=x,y=y,yaw=yaw),dict(x=x+.60,y=y,yaw=yaw),dict(x=x+.1,y=y,yaw=yaw)],[2])
    begin=len(rt.sim.history)
    handle=start_route(rt,route,plan);result=handle.get_result_async();wait(result.done,35)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    values=[p[3] for p in rt.sim.history[begin:]]
    assert any(v>.03 for v in values) and any(v<-.03 for v in values)
    first_reverse=next(i for i,v in enumerate(values) if v<-.03)
    assert sum(abs(v)<.005 for v in values[max(0,first_reverse-60):first_reverse])>=35


def test_real_nav2_cancel_stops_child_and_does_not_dispatch_next(runtime):
    rt=runtime;wait(lambda:rt.sim.guard_ready)
    x,y,yaw=rt.sim.x,rt.sim.y,rt.sim.yaw
    route,plan=make_route(rt,[dict(x=x,y=y,yaw=yaw),dict(x=x+2.,y=y,yaw=yaw),dict(x=x+3.,y=y,yaw=yaw)],[1,2])
    handle=start_route(rt,route,plan)
    wait(lambda:rt.sim.x>x+.15,15)
    cancel=handle.cancel_goal_async();wait(cancel.done)
    result=handle.get_result_async();wait(result.done,10)
    assert not result.result().result.success
    stopped_x=rt.sim.x;time.sleep(.8)
    assert abs(rt.sim.x-stopped_x)<.03
    assert rt.sim.x<x+1.0


def test_real_closed_recording_runs_three_loops_without_early_success(runtime):
    rt=runtime;wait(lambda:rt.sim.guard_ready)
    p=dict(x=rt.sim.x,y=rt.sim.y,yaw=rt.sim.yaw)
    q=dict(p,x=p['x']+.7)
    route,plan=make_route(rt,[p,q,p],[2],loops=3,close_loop=True)
    begin=len(rt.sim.history);handle=start_route(rt,route,plan,loops=3)
    result=handle.get_result_async();time.sleep(1.5)
    assert not result.done(), 'coincident endpoint must not immediately complete the loop'
    wait(result.done,55)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    assert result.result().result.completed_loops==3
    values=[p[3] for p in rt.sim.history[begin:]]
    reverse_starts=sum(v<-.03 and (i==0 or values[i-1]>=-.03) for i,v in enumerate(values))
    assert reverse_starts>=3


def test_real_multiple_stops_continue_the_recorded_tail(runtime):
    rt=runtime;wait(lambda:rt.sim.guard_ready)
    p=dict(x=rt.sim.x,y=rt.sim.y,yaw=rt.sim.yaw)
    route,plan=make_route(rt,[p,dict(p,x=p['x']+.7),dict(p,x=p['x']+1.4)],[1,2])
    handle=start_route(rt,route,plan);result=handle.get_result_async();wait(result.done,35)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    assert result.result().result.completed_steps==2


@pytest.mark.parametrize('start_x',[0.,2.86])
def test_real_bounded_detour_returns_to_the_original_road(runtime,start_x):
    rt=runtime;reset_sim(rt,start_x)
    p=dict(x=rt.sim.x,y=rt.sim.y,yaw=rt.sim.yaw)
    route,plan=make_route(rt,[p,dict(p,x=p['x']+3.)],[1])
    rt.sim.obstacles=[(p['x']+1.1,p['y'],.12)]
    begin=len(rt.sim.history)
    try:
        time.sleep(.5)
        handle=start_route(rt,route,plan);result=handle.get_result_async()
        wait(lambda:result.done() or rt.sim.progress.get('state')=='FOLLOWING_DETOUR',20)
        wait(result.done,90)
        assert result.result().result.success,(result.result().result,rt.sim.progress)
        trace=rt.sim.history[begin:]
        assert max(abs(value[1]-p['y']) for value in trace)>.25
        assert max(abs(value[1]-p['y']) for value in trace)<.9
        assert abs(rt.sim.y-p['y'])<.15
        clearances=[]
        for x,y,yaw,_,_ in trace:
            dx,dy=p['x']+1.1-x,p['y']-y
            bx=math.cos(yaw)*dx+math.sin(yaw)*dy;by=-math.sin(yaw)*dx+math.cos(yaw)*dy
            clearances.append(math.hypot(max(abs(bx)-.4475,0),max(abs(by)-.28,0))-.12)
        assert min(clearances)>=-.005,'actual Ackermann body collided with the simulated obstacle'
        states=[e['state'] for e in rt.patrol._store.road_events(result.result().result.run_id)]
        assert 'FOLLOWING_DETOUR' in states and 'REJOINING' in states
    finally:
        rt.sim.obstacles=[]


def reset_sim(rt,x=0.):
    wait(lambda: rt.patrol._active_engine is None and rt.sim.guard_ready)
    rt.sim.v=rt.sim.w=0.;rt.sim.last=0.
    rt.sim.x=x;rt.sim.y=rt.sim.yaw=0.
    time.sleep(.8)


def control(rt, command):
    from patrol_interfaces.srv import ControlRun
    client=rt.sim.create_client(ControlRun,'/patrol/control')
    request=ControlRun.Request(command=command,run_id=rt.patrol._active_engine.run_id,
                               owner_session_id='recorded_test_owner',command_id=str(time.time_ns()))
    future=client.call_async(request);wait(future.done)
    assert future.result().accepted,future.result().message
    rt.sim.destroy_client(client)


def test_real_pause_resume_preserves_reference_progress(runtime):
    rt=runtime;reset_sim(rt)
    p=dict(x=0.,y=0.,yaw=0.)
    route,plan=make_route(rt,[p,dict(p,x=1.5)],[1])
    handle=start_route(rt,route,plan);wait(lambda:rt.sim.x>.25)
    control(rt,1)
    wait(lambda:rt.patrol._active_engine and rt.patrol._active_engine.status['state']==7)
    x=rt.sim.x;time.sleep(.4);assert abs(rt.sim.x-x)<.02
    control(rt,2)
    result=handle.get_result_async();wait(result.done,25)
    assert result.result().result.success,(result.result().result,rt.sim.progress)


def test_real_return_home_uses_recorded_reverse_road(runtime):
    from patrol_interfaces.action import ReturnHome
    from rclpy.action import ActionClient
    rt=runtime;reset_sim(rt)
    p=dict(x=0.,y=0.,yaw=0.)
    route,plan=make_route(rt,[p,dict(p,x=1.2),p],[1],close_loop=True)
    rt.patrol._store.set_home_point(rt.mapping.map_id,rt.mapping.revision,0.,0.,0.)
    handle=start_route(rt,route,plan);result=handle.get_result_async();wait(result.done,25)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    begin=len(rt.sim.history)
    client=ActionClient(rt.sim,ReturnHome,'/patrol/return_home')
    goal=ReturnHome.Goal(request_id='home_'+str(time.time_ns()),owner_session_id='recorded_test_owner',
                         map_id=rt.mapping.map_id,map_revision=rt.mapping.revision)
    future=client.send_goal_async(goal);wait(future.done);assert future.result().accepted
    result=future.result().get_result_async();wait(result.done,25)
    assert result.result().result.success,(result.result().result,rt.sim.progress)
    assert abs(rt.sim.x)<.20 and any(p[3]<-.03 for p in rt.sim.history[begin:])


def test_real_recording_service_round_trip_and_command_retry(runtime):
    from patrol_interfaces.srv import RoadCommand
    from geometry_msgs.msg import Twist
    rt=runtime;reset_sim(rt)
    client=rt.sim.create_client(RoadCommand,'/patrol/road_command')
    teleop=rt.sim.create_publisher(Twist,'/cmd_vel_teleop',10)
    def command(op,payload=None,command_id=None,owner='recorded_test_owner',ok=True):
        value=dict(payload or {},owner_session_id=owner,command_id=command_id or str(time.time_ns()))
        future=client.call_async(RoadCommand.Request(operation=op,payload_json=json.dumps(value)));wait(future.done)
        response=future.result()
        assert response.success==ok,(response.error_code,response.message)
        return json.loads(response.data_json) if response.success else response
    def drive():
        until=time.monotonic()+2.5
        while time.monotonic()<until:
            msg=Twist();msg.linear.x=.12;teleop.publish(msg);time.sleep(.05)
        for _ in range(15):teleop.publish(Twist());time.sleep(.05)
    try:
        net=command('CREATE',{'name':'实际被动录制'})
        command('START_RECORD',{'network_id':net['network_id'],'name':'去程'})
        drive()
        review=command('FINISH_RECORD');assert review['state']=='REVIEW' and review['review_blocks']
        request={'network_id':net['network_id'],'expected_revision':net['revision']}
        saved=command('SAVE_RECORD',request,command_id='save_once')
        assert command('SAVE_RECORD',request,command_id='save_once')==saved
        net=saved['network'];end=saved['edge']['to'];port=net['document']['nodes'][end]
        point=rt.patrol._store.upsert_point('',rt.mapping.map_id,'录制终点',port['x'],port['y'],port['yaw'],'test',1)
        net=command('BIND_POINT',{'network_id':net['network_id'],'expected_revision':net['revision'],
                                 'point_id':point.point_id,'node_ids':[end]})
        net=command('PUBLISH',{'network_id':net['network_id'],'expected_revision':net['revision']})
        assert net['published']
        exported=command('EXPORT',{'network_id':net['network_id']})
        imported=command('IMPORT',{'export':exported});assert not imported['published']
        count=len(rt.patrol._store.list_networks(rt.mapping.map_id))
        exported['sha256']='corrupt'
        command('IMPORT',{'export':exported},ok=False)
        assert len(rt.patrol._store.list_networks(rt.mapping.map_id))==count
        command('START_RECORD',{'network_id':net['network_id'],'name':'断线草稿','start_node':end})
        drive();rt.sim.operator=False
        wait(lambda:rt.patrol.roads.recorder.state=='PAUSED',6)
        rt.sim.operator=True
        # An online old owner cannot be displaced by another local UI.
        time.sleep(.25)
        command('RECOVER_DRAFT',{'draft_id':rt.patrol.roads.draft_id},owner='new_owner',ok=False)
        rt.sim.operator=False;time.sleep(.25)
        recovered=command('RECOVER_DRAFT',{'draft_id':rt.patrol.roads.draft_id},owner='new_owner')
        assert recovered['state']=='REVIEW'
        command('DISCARD_RECORD',owner='new_owner')
    finally:
        rt.sim.operator=True;teleop.publish(Twist())
        rt.sim.destroy_client(client);rt.sim.destroy_publisher(teleop)


def test_real_inconsistent_field_tail_stops_without_deleting_short_blocks(runtime):
    rt=runtime;reset_sim(rt)
    blocks=json.loads((Path(__file__).parent/'data/recorded_field_tail.json').read_text())
    original=json.dumps(blocks,sort_keys=True)
    route,plan=make_route(rt,[blocks[0]['samples'][0],blocks[-1]['samples'][-1]],
                         [1],blocks_override=blocks)
    handle=start_route(rt,route,plan);result=handle.get_result_async();wait(result.done,40)
    # Field data contains a 6.5cm reversal with 5.1cm lateral displacement.
    # A controller fix must not turn this into a falsely completed route.
    assert not result.result().result.success
    assert rt.sim.progress['state']=='FAILED'
    assert '位姿未对齐' in rt.sim.progress['error'] or 'ENDPOINT_UNREACHABLE' in rt.sim.progress['error']
    assert result.result().result.completed_steps==0
    assert [b['direction'] for b in plan['legs'][0]['blocks']]==[1,-1,1]
    assert json.dumps(blocks,sort_keys=True)==original
    x,y=rt.sim.x,rt.sim.y
    time.sleep(.8)
    assert math.hypot(rt.sim.x-x,rt.sim.y-y)<.005 and abs(rt.sim.v)<.005


def test_real_operator_loss_while_blocked_latches_and_interrupts(runtime):
    rt=runtime;reset_sim(rt)
    p=dict(x=0.,y=0.,yaw=0.)
    route,plan=make_route(rt,[p,dict(p,x=3.)],[1])
    rt.sim.obstacles=[(1.5,0.,.70)]
    time.sleep(.5)
    handle=start_route(rt,route,plan)
    wait(lambda:rt.sim.progress.get('state') in ('WAITING_OBSTACLE','PLANNING_DETOUR'),15)
    rt.sim.operator=False
    try:
        result=handle.get_result_async();wait(result.done,10)
        assert not result.result().result.success
        assert result.result().result.final_state==12
        x=rt.sim.x;rt.sim.operator=True;time.sleep(.7)
        assert not rt.sim.guard_ready and abs(rt.sim.x-x)<.02
    finally:
        rt.sim.operator=True;rt.sim.obstacles=[]
