"""Hardware-free ROS checks: actual ICP -> fixed-tree adoption -> retained correction."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
import tf_transformations as tf

PACKAGE = Path(__file__).resolve().parents[1]
CACHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(160 + os.getpid()%20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[])
    node = rclpy.create_node('prior_map_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    resources = dict(node=node, executor=executor, nodes=[], processes=[], streams=[])

    def spin_until(predicate, timeout=15):
        end = time.monotonic()+timeout
        while time.monotonic()<end and not predicate():
            executor.spin_once(timeout_sec=.05)
        assert predicate()

    def spin_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.05)

    # Three asymmetric planes, not a single plane that leaves yaw/translation unobservable.
    a,b=np.meshgrid(np.arange(9)*.35,np.arange(7)*.35)
    points=np.vstack([np.column_stack((a.ravel(),b.ravel(),np.zeros(a.size))),
        np.column_stack((a.ravel(),np.full(a.size,3.),b.ravel())),
        np.column_stack((np.full(a.size,4.),a.ravel(),b.ravel()))]).astype(np.float32)
    pcd=tmp_path/'prior.pcd'
    pcd.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n'+
        '\n'.join(' '.join(map(str,p)) for p in points)+'\n')
    log=(tmp_path/'lio.log').open('w+')
    resources['streams'].append(log)
    from ament_index_python.packages import get_package_prefix
    binary=Path(get_package_prefix('fast_lio_localization'))/'lib/fast_lio_localization/fastlio_mapping'
    process=subprocess.Popen([str(binary),'--ros-args','--params-file',str(PACKAGE/'config/velodyne_test.yaml'),
        '-p','map_file_path:='+str(pcd)],stdout=log,stderr=subprocess.STDOUT)
    resources['processes'].append(process)
    resources.update(points=points,spin_until=spin_until,spin_for=spin_for)
    try:
        yield resources
        assert process.poll() is None, Path(log.name).read_text()
    finally:
        for n in resources['nodes']:
            executor.remove_node(n)
            n.destroy_node()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        for p in resources['processes']:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try: p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    p.kill(); p.wait(timeout=3)
        for stream in resources['streams']: stream.close()


def test_startup_idle_relocalization_failure_and_late_fusion(rig):
    spec=importlib.util.spec_from_file_location('prior_map_icp',PACKAGE/'scripts/global_localization.py')
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    icp=module.GlobalLocalizationNode(parameter_overrides=[
        Parameter('prior_map.enable',value=True),Parameter('initialization_attempts',value=1),
        Parameter('scan_voxel_size',value=.02),Parameter('map_voxel_size',value=.02),
        Parameter('base_from_body_translation',value=[.32,0.,.22])])
    rig['nodes'].append(icp); rig['executor'].add_node(icp)
    node=rig['node']; spin=rig['spin_until']; wait=rig['spin_for']
    statuses=[]; corrections=[]
    node.create_subscription(String,'/fastlio/prior_map_status',lambda m:statuses.append(json.loads(m.data)),CACHED)
    node.create_subscription(Odometry,'/map_to_odom',corrections.append,CACHED)
    map_pub=node.create_publisher(PointCloud2,'/global_map',CACHED)
    scan_pub=node.create_publisher(PointCloud2,'/cloud_registered',1)
    odom_pub=node.create_publisher(Odometry,'/Odometry',1)
    pose_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    spin(lambda:statuses and map_pub.get_subscription_count() and scan_pub.get_subscription_count())
    map_pub.publish(create_cloud_xyz32(Header(frame_id='map'),rig['points']))
    spin(lambda:icp.global_map is not None)
    body=tf.translation_matrix([1.,-.5,.1])
    base_from_body=tf.translation_matrix([.32,0.,.22])

    def publish_pair(transform, stamp):
        pts=(np.linalg.inv(transform)@np.column_stack((rig['points'],np.ones(len(rig['points'])))).T).T[:,:3]
        header=Header(frame_id='lio_odom'); header.stamp.sec=stamp
        odom=Odometry(); odom.header=header
        xyz=body[:3,3]
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,xyz)
        odom.pose.pose.orientation.w=1.
        odom_pub.publish(odom)
        scan_pub.publish(create_cloud_xyz32(header,pts.astype(np.float32)))
        spin(lambda:icp.cur_scan_odom is not None and icp.cur_scan_odom.header.stamp.sec==stamp)

    def request(transform):
        base=transform@body@np.linalg.inv(base_from_body)
        msg=PoseWithCovarianceStamped(); msg.header.frame_id='map'
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,base[:3,3])
        q=tf.quaternion_from_matrix(base)
        msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=map(float,q)
        pose_pub.publish(msg)

    first=tf.translation_matrix([3.,-4.,2.])@tf.euler_matrix(.05,-.03,.4)
    publish_pair(first,1)
    request(first)
    spin(lambda:len(corrections)==1 and statuses[-1]['revision']==1)
    np.testing.assert_allclose(icp.pose_to_mat(corrections[-1]),first,atol=.025)
    assert statuses[-1]['mode']=='fixed_map'
    assert not list(icp.timers), 'On-demand mode created a periodic ICP timer'
    assert icp.cur_scan is not None
    decoded=icp.cur_scan
    for stamp in range(2,7): publish_pair(first,stamp)
    wait(2.2)
    assert len(corrections)==1 and icp.cur_scan is decoded, 'Idle mode still decodes/scans or runs ICP'
    count=statuses[-1]['map_points']

    second=tf.translation_matrix([6.,-2.,1.])@tf.euler_matrix(-.02,.04,-.2)
    publish_pair(second,7)
    request(second)
    spin(lambda:len(corrections)==2 and statuses[-1]['revision']==2)
    np.testing.assert_allclose(icp.pose_to_mat(corrections[-1]),second,atol=.025)
    assert statuses[-1]['map_points']==count

    # A failed request must retain the previous C++ tree and TF correction.
    request(tf.translation_matrix([1000.,1000.,1000.]))
    wait(1.)
    assert len(corrections)==2 and statuses[-1]['revision']==2
    np.testing.assert_allclose(icp.T_map_to_odom,second,atol=.025)

    # A late/restarted TF consumer receives the accepted correction without running ICP again.
    late=[]
    node.create_subscription(Odometry,'/map_to_odom',late.append,CACHED)
    spin(lambda:len(late)==1)
    np.testing.assert_allclose(icp.pose_to_mat(late[0]),second,atol=.025)


def test_invalid_alignment_is_not_applied_or_forwarded(rig):
    node=rig['node']; statuses=[]; corrections=[]
    node.create_subscription(String,'/fastlio/prior_map_status',lambda m:statuses.append(json.loads(m.data)),CACHED)
    node.create_subscription(Odometry,'/map_to_odom',corrections.append,CACHED)
    pub=node.create_publisher(Odometry,'/fastlio/prior_map_alignment',1)
    rig['spin_until'](lambda:statuses and pub.get_subscription_count())
    msg=Odometry(); msg.header.frame_id='map'; msg.child_frame_id='lio_odom'
    msg.pose.pose.orientation.w=0.
    pub.publish(msg)  # zero quaternion
    rig['spin_for'](.5)
    assert statuses[-1]['revision']==0 and not corrections
    msg.pose.pose.orientation.w=1.; msg.child_frame_id='base_link'
    pub.publish(msg)
    rig['spin_for'](.5)
    assert statuses[-1]['revision']==0 and not corrections
