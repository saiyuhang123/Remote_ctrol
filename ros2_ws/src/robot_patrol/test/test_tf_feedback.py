"""TF reception must continue while all patrol executor workers are busy."""
import os
import threading
import time

import pytest


def test_dedicated_tf_survives_blocked_patrol_callbacks_and_still_expires():
    if not (1 <= int(os.environ.get('ROS_DOMAIN_ID','0')) <= 232 and
            os.environ.get('ROS_LOCALHOST_ONLY')=='1'):
        pytest.skip('requires an isolated ROS domain and localhost-only DDS')
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
    from rclpy.time import Time
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    from robot_patrol.tf_feedback import TfFeedback

    context=Context();rclpy.init(context=context)
    parent=Node('blocked_patrol_test',context=context)
    publisher=Node('independent_test_odometry',context=context)
    feedback=TfFeedback(parent)
    static=StaticTransformBroadcaster(publisher);dynamic=TransformBroadcaster(publisher)
    msg=TransformStamped();msg.header.frame_id='map';msg.child_frame_id='odom';msg.transform.rotation.w=1.
    msg.header.stamp=publisher.get_clock().now().to_msg();static.sendTransform(msg)
    enabled=True
    def publish():
        if enabled:
            msg=TransformStamped();msg.header.frame_id='odom';msg.child_frame_id='base_link'
            msg.header.stamp=publisher.get_clock().now().to_msg();msg.transform.rotation.w=1.
            dynamic.sendTransform(msg)
    publisher.create_timer(.02,publish)
    pub_executor=SingleThreadedExecutor(context=context);pub_executor.add_node(publisher)
    pub_thread=threading.Thread(target=pub_executor.spin,daemon=True);pub_thread.start()
    blocked=0;blocked_lock=threading.Lock();release=threading.Event()
    def busy():
        nonlocal blocked
        with blocked_lock:blocked+=1
        release.wait(8)
    group=ReentrantCallbackGroup()
    timers=[parent.create_timer(.01,busy,callback_group=group) for _ in range(4)]
    main_executor=MultiThreadedExecutor(num_threads=4,context=context);main_executor.add_node(parent)
    main_thread=threading.Thread(target=main_executor.spin,daemon=True);main_thread.start()
    def age():
        tf=feedback.buffer.lookup_transform('map','base_link',Time())
        return (publisher.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
    try:
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            try:
                if blocked>=4 and age()<.2:break
            except Exception:pass
            time.sleep(.02)
        assert blocked>=4
        time.sleep(1.6)
        assert age()<.2,'busy patrol callbacks must not starve TF reception'
        enabled=False
        time.sleep(.40)
        assert age()>.30,'an actual stopped TF publisher must not be restamped as fresh'
    finally:
        for timer in timers:parent.destroy_timer(timer)
        release.set();main_executor.shutdown(timeout_sec=2);main_thread.join(2)
        feedback.close();feedback.close()
        assert not feedback.thread.is_alive()
        pub_executor.shutdown(timeout_sec=2);pub_thread.join(2)
        parent.destroy_node();publisher.destroy_node();rclpy.shutdown(context=context)


@pytest.mark.parametrize('age',[.05,2.,-1.])
def test_pose_age_errors_distinguish_old_tf_from_a_clock_ahead(age):
    from types import SimpleNamespace as NS
    from rclpy.clock import ClockType
    from rclpy.time import Time
    from geometry_msgs.msg import TransformStamped
    from robot_patrol.models import PatrolError
    from robot_patrol.node import PatrolNode
    now=Time(nanoseconds=3_000_000_000,clock_type=ClockType.ROS_TIME)
    tf=TransformStamped();tf.header.stamp=Time(nanoseconds=round((3-age)*1e9)).to_msg()
    tf.transform.rotation.w=1.
    node=NS(tf_buffer=NS(lookup_transform=lambda *args:tf),
            get_parameter=lambda key:NS(value={'map_frame':'map','base_frame':'base_link','tf_max_age_sec':1.5}[key]),
            get_clock=lambda:NS(now=lambda:now))
    if age==.05:
        assert PatrolNode._current_pose(node)[:3]==(0.,0.,0.)
    else:
        with pytest.raises(PatrolError) as error:PatrolNode._current_pose(node)
        assert ('延迟 2.00 秒' if age>0 else '时间超前 1.00 秒') in error.value.message
