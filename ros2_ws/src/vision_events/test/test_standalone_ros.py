"""Explicit opt-in; real nodes with fake camera/TF, never real hardware."""
import json
import os
from pathlib import Path
import time
import cv2
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('VISION_ROS_TESTS') != '1', reason='isolated ROS test opt-in')


@pytest.fixture(autouse=True)
def isolated_domain():
    assert os.environ.get('ROS_DOMAIN_ID') == '227' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'


def test_recorder_without_navigation_and_native_replay(tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from hk_camera.msg import FaceCapture
    from vision_interfaces.srv import QueryEvents
    from vision_events.node import Recorder
    rclpy.init()
    native = tmp_path/'native'; native.mkdir()
    recorder = Recorder(parameter_overrides=[Parameter('data_dir', value=str(tmp_path/'records')),
                                             Parameter('face_dirs', value=[str(native)]),
                                             Parameter('accept_hikvision_faces', value=True)])
    fixture = rclpy.create_node('standalone_vision_fixture')
    publisher = fixture.create_publisher(FaceCapture, '/hk_camera/face_capture', 10)
    query = fixture.create_client(QueryEvents, '/vision/query_events')
    executor = SingleThreadedExecutor(); executor.add_node(fixture); executor.add_node(recorder)
    try:
        for i in range(20):
            image = native/f'{i}_face.jpg'; cv2.imwrite(str(image), np.full((40, 40, 3), i*10, np.uint8))
            metadata = native/f'{i}.json'
            metadata.write_text(json.dumps(dict(source='hikvision_sdk_face_capture', received_ns=time.time_ns(),
                                                face_id=7, score=80, face_path=str(image))))
            message = FaceCapture(); message.metadata_path = str(metadata); publisher.publish(message)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and recorder.store.query()[2] < 20:
            executor.spin_once(timeout_sec=.05)
        assert recorder.store.query()[2] == 20
        assert all(not item['pose_valid'] and item['robot_x'] is None
                   and Path(item['image_path']).is_file() for item in recorder.store.query()[0])
        for _ in range(10):
            recorder.scan = None; recorder.scan_files(); executor.spin_once(timeout_sec=.02)
        assert recorder.store.query()[2] == 20
        future = query.call_async(QueryEvents.Request(limit=30))
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert future.result().success and future.result().total == 20
        assert not any('navigation' in name or 'patrol' in name for name in fixture.get_node_names())
    finally:
        recorder.shutdown_worker(); recorder.destroy_node(); fixture.destroy_node(); executor.shutdown(); rclpy.shutdown()


def test_detector_receives_ros_images_and_switches_off_without_navigation(tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from detector_node.detector_node import DetectorNode
    from detector_node.detection_pipeline import Pipeline
    rclpy.init()
    node = DetectorNode(parameter_overrides=[Parameter('staging_dir', value=str(tmp_path)),
                                             Parameter('enable_person', value=False), Parameter('enable_plate', value=False),
                                             Parameter('enable_face', value=False)])
    fixture = rclpy.create_node('standalone_detector_fixture')
    publisher = fixture.create_publisher(Image, '/hk_camera/image_raw', 10)
    pipeline = Pipeline(lambda frame: [(0, .91, [0, 0, 60, 100])])
    node.load_pipeline = lambda person, plate, face: pipeline
    node.set_parameters([Parameter('enable_person', value=True)])
    executor = SingleThreadedExecutor(); executor.add_node(node); executor.add_node(fixture)
    try:
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not list(tmp_path.glob('*.json')):
            message = CvBridge().cv2_to_imgmsg(np.zeros((120, 120, 3), np.uint8), 'bgr8')
            message.header.stamp = fixture.get_clock().now().to_msg(); publisher.publish(message)
            executor.spin_once(timeout_sec=.05)
        assert len(list(tmp_path.glob('*.json'))) == 1
        payload = json.loads(next(tmp_path.glob('*.json')).read_text())
        assert payload['event_type'] == 'person' and payload['confidence'] == .91
        node.set_parameters([Parameter('enable_person', value=False)])
        time.sleep(.5); before = node.state['frames']; time.sleep(.5)
        assert node.state['frames'] == before
    finally:
        node.shutdown_worker(); node.destroy_node(); fixture.destroy_node(); executor.shutdown(); rclpy.shutdown()


def test_delayed_capture_uses_historical_robot_pose(tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from vision_events.node import Recorder
    rclpy.init()
    native = tmp_path/'native'; native.mkdir()
    recorder = Recorder(parameter_overrides=[Parameter('data_dir', value=str(tmp_path/'records')),
                                             Parameter('face_dirs', value=[str(native)])])
    fixture = rclpy.create_node('vision_pose_fixture')
    broadcaster = TransformBroadcaster(fixture)
    executor = SingleThreadedExecutor(); executor.add_node(fixture); executor.add_node(recorder)
    try:
        recorder.pose.context('Map A')
        def publish(x, duration):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                transform = TransformStamped(); transform.header.stamp = fixture.get_clock().now().to_msg()
                transform.header.frame_id = 'map'; transform.child_frame_id = 'base_link'
                transform.transform.translation.x = x; transform.transform.rotation.w = 1.
                broadcaster.sendTransform(transform); executor.spin_once(timeout_sec=.02)
        publish(1., .5)
        assert recorder.pose.samples
        old_stamp = recorder.pose.samples[-1][0]
        publish(9., .2)
        image = native/'delayed.jpg'; cv2.imwrite(str(image), np.zeros((40,40,3), np.uint8))
        recorder.enqueue(dict(event_type='face_capture', source='test', source_event_key='delayed',
                              event_time_ns=old_stamp, image_path=str(image)))
        end = time.monotonic()+2
        while not recorder.store.query()[2] and time.monotonic() < end:
            executor.spin_once(timeout_sec=.03)
        event = recorder.store.query()[0][0]
        assert event['pose_valid'] and event['robot_x'] == 1.
        recorder.pose.context('Map B')
        assert not recorder.pose.associate(old_stamp)['pose_valid']
    finally:
        recorder.shutdown_worker(); recorder.destroy_node(); fixture.destroy_node(); executor.shutdown(); rclpy.shutdown()
