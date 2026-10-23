"""Real Nav2 + JD03 bridge against an inert, delayed 4WS simulator.

Run with ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1 ROBOT_FREE_NAV_TEST=1.
No chassis/CAN/sensor driver is started. All feedback and ECU traffic is fake
and confined to an explicitly isolated localhost ROS domain.
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

pytestmark = pytest.mark.skipif(os.environ.get('ROBOT_FREE_NAV_TEST') != '1',
                                reason='explicit isolated Nav2 simulation required')


def wait(predicate, timeout=20):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        value=predicate()
        if value:
            return value
        time.sleep(.025)
    raise AssertionError('timed out waiting for simulated navigation')


def pose(x,y=0.,yaw=0.):
    from geometry_msgs.msg import PoseStamped
    p=PoseStamped();p.header.frame_id='map'
    p.pose.position.x=float(x);p.pose.position.y=float(y)
    p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
    return p


@pytest.fixture(scope='module')
def runtime(tmp_path_factory):
    assert 100<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=232
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionClient
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry, Path as RosPath
    from nav2_msgs.action import FollowPath, NavigateToPose
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from yunle_msgs.msg import Ecu, VehicleStatus
    from PIL import Image

    source=Path(__file__).resolve().parents[1]
    tmp=tmp_path_factory.mktemp('free_nav_4ws')
    params=yaml.safe_load((source/'config/nav2.yaml').read_text().replace(
        '@CHASSIS_NAV2_SHARE@',str(source)))
    hw=yaml.safe_load((source/'config/hardware.yaml').read_text())
    Image.new('L',(320,320),255).save(tmp/'map.pgm')
    (tmp/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,
        origin=[-8.,-8.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    params['map_server']['ros__parameters']['yaml_filename']=str(tmp/'map.yaml')
    params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']=hw['chassis']['navigation_turning_radius']
    for name in ('local_costmap','global_costmap'):
        cost=params[name][name]['ros__parameters']
        cost['footprint']=json.dumps(hw['robot_geometry']['footprint'])
        cost['obstacle_layer']['scan'].update(inf_is_valid=True,raytrace_max_range=8.,obstacle_max_range=7.5)
    chassis=hw['chassis'];geom=hw['robot_geometry']
    params['jd03_bridge']={'ros__parameters':dict(enable_control=True,
        wheel_base=geom['wheel_base'],max_steering_deg=geom['max_steering_deg'],
        max_speed=chassis['max_speed'],max_reverse_speed=chassis['max_reverse_speed'],
        command_steering_sign=chassis['command_steering_sign'],four_wheel_steering=True,
        rear_steering_sign=chassis['rear_steering_sign'],
        four_wheel_curvature_gain=chassis['four_wheel_curvature_gain'],
        steering_rate_dps=chassis['steering_rate_dps'],feedback_speed_scale=hw['wheel_odometry']['speed_scale'])}
    names=['map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother']
    params['lifecycle_manager']={'ros__parameters':dict(autostart=True,node_names=names,bond_timeout=4.)}
    config=tmp/'params.yaml';config.write_text(yaml.safe_dump(params))
    processes=[];logs=[]

    def start(package,executable,remaps=()):
        log=(tmp/(executable+'.log')).open('w');logs.append(log)
        cmd=['ros2','run',package,executable,'--ros-args','--params-file',str(config)]
        for remap in remaps:cmd+=['-r',remap]
        processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))

    rclpy.init()

    class Sim(Node):
        def __init__(self):
            super().__init__('inert_free_navigation_sim')
            self.x=self.y=self.yaw=self.v=self.steer=0.;self.gear=1
            self.target=None;self.history=[];self.raw=[];self.plans=[];self.shift_violations=[]
            self.obstacles=[];self.tick_count=0;self.noise=0.
            self.broadcaster=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.
            tf.header.stamp=self.get_clock().now().to_msg();self.static.sendTransform(tf)
            self.odom=self.create_publisher(Odometry,'/odom',10)
            self.scan=self.create_publisher(LaserScan,'/scan',10)
            self.status=self.create_publisher(VehicleStatus,'/vehicle_status',10)
            self.create_subscription(Ecu,'/ecu',self.command,10)
            self.create_subscription(Twist,'/inert_nav/raw',lambda m:self.raw.append((time.monotonic(),m.linear.x,m.angular.z)),10)
            self.create_subscription(RosPath,'/plan',lambda m:self.plans.append(m),10)
            self.create_timer(.02,self.tick)

        def reset(self,x=0.,y=0.,yaw=0.,noise=0.):
            self.x,self.y,self.yaw=x,y,yaw
            self.v=self.steer=0.;self.target=None;self.obstacles=[];self.noise=noise
            self.history=[];self.raw=[];self.plans=[];self.shift_violations=[]

        def command(self,msg):
            self.target=(time.monotonic(),msg)

        def tick(self):
            now=time.monotonic();dt=.02
            target_v=target_steer=0.
            if self.target and now-self.target[0]<.3:
                msg=self.target[1]
                if msg.shift!=self.gear:
                    if abs(self.v)>.031:self.shift_violations.append((self.v,self.gear,msg.shift))
                    self.gear=msg.shift
                if not msg.brake:
                    target_v=round(msg.motor*10)/10/3.6*(1 if msg.shift==1 else -1)
                target_steer=-math.radians(msg.steer)
            # Model motor lag, CAN speed quantization, and steering slew + lag.
            dv=target_v-self.v;accel=.8 if abs(target_v)<abs(self.v) else .4
            self.v+=max(-accel*dt,min(accel*dt,dv))
            ds=max(-math.radians(45)*dt,min(math.radians(45)*dt,(target_steer-self.steer)*dt/.10))
            self.steer+=ds
            w=self.v*1.63*math.tan(self.steer)/.5
            self.x+=self.v*math.cos(self.yaw)*dt;self.y+=self.v*math.sin(self.yaw)*dt;self.yaw+=w*dt
            self.history.append((now,self.x,self.y,self.yaw,self.v,w,self.gear))
            stamp=self.get_clock().now().to_msg()
            # Small bounded localization jitter tests that preview ignores
            # measurement-scale corrections rather than repeatedly full-locking.
            noise=self.noise*math.sin(now*13.)
            tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=stamp
            tf.transform.translation.x=self.x;tf.transform.translation.y=self.y+noise
            tf.transform.rotation.z=math.sin((self.yaw+noise)/2);tf.transform.rotation.w=math.cos((self.yaw+noise)/2)
            self.broadcaster.sendTransform(tf)
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=stamp
            odom.pose.pose.position.x=self.x;odom.pose.pose.position.y=self.y+noise;odom.pose.pose.orientation=tf.transform.rotation
            odom.twist.twist.linear.x=self.v;odom.twist.twist.angular.z=w;self.odom.publish(odom)
            status=VehicleStatus();status.cur_speed=abs(self.v)/.29;status.cur_steer=-math.degrees(self.steer)
            status.shift_level=self.gear;self.status.publish(status)
            self.tick_count+=1
            if self.tick_count%5==0:
                scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=stamp
                scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=math.pi/180
                scan.range_min=.05;scan.range_max=8.;ranges=[]
                for k in range(361):
                    theta=self.yaw-math.pi+k*math.pi/180;ux,uy=math.cos(theta),math.sin(theta);hit=math.inf
                    for ox,oy,radius in self.obstacles:
                        dx,dy=ox-self.x,oy-self.y;along=dx*ux+dy*uy;perp=dx*dx+dy*dy-along*along
                        if along>0 and perp<radius*radius:hit=min(hit,along-math.sqrt(radius*radius-perp))
                    ranges.append(float(hit))
                scan.ranges=ranges;self.scan.publish(scan)

    sim=Sim();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(sim)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        for package,executable,remaps in (
            ('nav2_map_server','map_server',()),
            ('nav2_controller','controller_server',('cmd_vel:=/inert_nav/raw',)),
            ('nav2_planner','planner_server',()),
            ('nav2_behaviors','behavior_server',('cmd_vel:=/inert_nav/raw',)),
            ('nav2_bt_navigator','bt_navigator',()),
            ('nav2_velocity_smoother','velocity_smoother',('cmd_vel:=/inert_nav/raw','cmd_vel_smoothed:=/inert_nav/cmd_vel')),
            ('chassis_nav2','jd03_bridge.py',('cmd_vel:=/inert_nav/cmd_vel',)),
            ('nav2_lifecycle_manager','lifecycle_manager',())):
            start(package,executable,remaps)
        follow=ActionClient(sim,FollowPath,'/follow_path');navigate=ActionClient(sim,NavigateToPose,'/navigate_to_pose')
        wait(lambda:follow.server_is_ready() and navigate.server_is_ready(),40)
        wait(lambda:(tmp/'bt_navigator.log').exists() and 'Creating bond' in (tmp/'bt_navigator.log').read_text(),40)
        yield SimpleNamespace(sim=sim,follow=follow,navigate=navigate,tmp=tmp)
    finally:
        (tmp/'last_trace.json').write_text(json.dumps(dict(history=sim.history,raw=sim.raw)))
        for proc in processes:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for proc in processes:
            try:proc.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2)
        for log in logs:log.close()
        executor.shutdown(timeout_sec=3);thread.join(timeout=3);sim.destroy_node();rclpy.shutdown()
        print('Isolated Nav2 logs:',tmp)


def send_follow(rt,coords):
    from nav2_msgs.action import FollowPath
    goal=FollowPath.Goal();goal.path.header.frame_id='map'
    goal.path.poses=[pose(*p) for p in coords]
    goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
    future=rt.follow.send_goal_async(goal);wait(future.done)
    handle=future.result();assert handle.accepted
    return handle,handle.get_result_async()


def finish(rt,handle,result,timeout=35):
    try:
        wait(result.done,timeout)
        assert result.result().status==4
        wait(lambda:abs(rt.sim.v)<.01,3)
        assert not rt.sim.shift_violations
    finally:
        if not result.done():wait(handle.cancel_goal_async().done,5)


def directions(raw):
    values=[]
    for _,v,_ in raw:
        if abs(v)<.015:continue
        direction=1 if v>0 else -1
        if not values or values[-1]!=direction:values.append(direction)
    return values


@pytest.fixture(autouse=True)
def save_case_trace(request,runtime):
    yield
    sim=runtime.sim
    (runtime.tmp/(request.node.name+'.json')).write_text(json.dumps(dict(
        history=sim.history,raw=sim.raw,shift_violations=sim.shift_violations,
        plans=[[(p.pose.position.x,p.pose.position.y,
                 2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)) for p in plan.poses]
               for plan in sim.plans])))


def test_straight_tracking_at_higher_speed_with_heading_error_and_jitter(runtime):
    rt=runtime;rt.sim.reset(y=.12,yaw=math.radians(8),noise=.005);time.sleep(.6)
    handle,result=send_follow(rt,[(i*.04,0.,0.) for i in range(76)])
    finish(rt,handle,result)
    assert directions(rt.sim.raw)==[1]
    settled=[abs(row[2]) for row in rt.sim.history if .8<row[1]<2.5]
    assert max(settled)<.08
    assert .75<max(row[4] for row in rt.sim.history)<.81
    assert max(v for _,v,_ in rt.sim.raw)<=.800001
    assert math.hypot(rt.sim.x-3.,rt.sim.y)<.12


def test_reverse_straight_keeps_direction_with_offset_and_jitter(runtime):
    rt=runtime;rt.sim.reset(y=.10,yaw=math.radians(-6),noise=.005);time.sleep(.6)
    handle,result=send_follow(rt,[(-i*.04,0.,0.) for i in range(51)])
    finish(rt,handle,result)
    assert directions(rt.sim.raw)==[-1]
    assert min(row[4] for row in rt.sim.history)<-.28
    assert max(abs(row[2]) for row in rt.sim.history if -1.6<row[1]<-.8)<.08
    assert math.hypot(rt.sim.x+2.,rt.sim.y)<.12


def test_reverse_curve_then_forward_has_one_cusp_and_no_mid_leg_gear_flips(runtime):
    rt=runtime;rt.sim.reset(noise=.003);time.sleep(.6)
    coords=[(-math.sin(t),1-math.cos(t),-t) for t in [i*.02 for i in range(46)]]
    x,y,yaw=coords[-1]
    coords.extend((x+s*math.cos(yaw),y+s*math.sin(yaw),yaw) for s in [i*.04 for i in range(1,36)])
    handle,result=send_follow(rt,coords);finish(rt,handle,result,40)
    assert directions(rt.sim.raw)==[-1,1]
    assert min(row[4] for row in rt.sim.history)<-.20
    negative=[row for row in rt.sim.history if row[4]<-.04]
    assert max(abs(math.hypot(row[1],row[2]-1)-1) for row in negative)<.10
    assert math.hypot(rt.sim.x-coords[-1][0],rt.sim.y-coords[-1][1])<.12


def test_valid_path_is_retained_and_changed_goal_replans(runtime):
    from nav2_msgs.action import NavigateToPose
    rt=runtime;rt.sim.reset();time.sleep(.6)
    def send(target):
        goal=NavigateToPose.Goal();goal.pose=pose(*target)
        future=rt.navigate.send_goal_async(goal);wait(future.done)
        handle=future.result();assert handle.accepted
        return handle,handle.get_result_async()
    first,first_result=send((4.,0.,0.))
    second=None
    try:
        wait(lambda:rt.sim.x>.6,15)
        count=len(rt.sim.plans);time.sleep(2.1)
        assert count==1 and len(rt.sim.plans)==1
        second,result=send((3.,1.,0.))
        wait(lambda:len(rt.sim.plans)>=2,15)
        finish(rt,second,result,35)
    finally:
        handle=second or first
        wait(handle.cancel_goal_async().done,5)


def test_obstacle_invalidates_retained_path_and_triggers_replanning(runtime):
    from nav2_msgs.action import NavigateToPose
    rt=runtime;rt.sim.reset();time.sleep(.8)
    goal=NavigateToPose.Goal();goal.pose=pose(4.)
    future=rt.navigate.send_goal_async(goal);wait(future.done);handle=future.result();assert handle.accepted
    try:
        wait(lambda:rt.sim.x>.3,15)
        before=len(rt.sim.plans);rt.sim.obstacles=[(2.5,0.,.2)]
        wait(lambda:len(rt.sim.plans)>before,20)
        assert rt.sim.x<2.0
        assert not rt.sim.shift_violations
    finally:
        wait(handle.cancel_goal_async().done,5)
        wait(lambda:abs(rt.sim.v)<.01,5)


def test_actual_reeds_shepp_plan_reaches_a_lateral_parking_goal(runtime):
    from nav2_msgs.action import NavigateToPose
    rt=runtime;rt.sim.reset(noise=.003);time.sleep(.8)
    goal=NavigateToPose.Goal();goal.pose=pose(0.,.7,0.)
    future=rt.navigate.send_goal_async(goal);wait(future.done);handle=future.result();assert handle.accepted
    finish(rt,handle,handle.get_result_async(),45)
    # Smac may end within its separate 10 cm planning tolerance. The controller
    # must reach the resulting endpoint within its own 10 cm tolerance.
    end=rt.sim.plans[-1].poses[-1].pose.position
    assert math.hypot(rt.sim.x-end.x,rt.sim.y-end.y)<.11
    assert math.hypot(rt.sim.x,rt.sim.y-.7)<.20
    assert len(directions(rt.sim.raw))>=2


def test_through_poses_tree_accepts_and_reaches_remaining_goals(runtime):
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateThroughPoses
    rt=runtime;rt.sim.reset();time.sleep(.8)
    client=ActionClient(rt.sim,NavigateThroughPoses,'/navigate_through_poses')
    wait(client.server_is_ready,10)
    goal=NavigateThroughPoses.Goal();goal.poses=[pose(1.),pose(2.5,.5)]
    future=client.send_goal_async(goal);wait(future.done);handle=future.result();assert handle.accepted
    finish(rt,handle,handle.get_result_async(),35)
    assert math.hypot(rt.sim.x-2.5,rt.sim.y-.5)<.12
    client.destroy()
