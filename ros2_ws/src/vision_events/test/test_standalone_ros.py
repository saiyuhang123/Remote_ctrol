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


def test_installed_ros2_run_face_only_starts_and_reports_ready(tmp_path):
    """Exercise the installed CLI used by Qt, not just the Python class."""
    import signal
    import subprocess
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    fixture = rclpy.create_node('installed_detector_startup_fixture')
    states = []
    fixture.create_subscription(String, '/vision/detector_status',
                                lambda msg: states.append(json.loads(msg.data)), 10)
    output = tmp_path/'detector-startup.log'
    process = None
    try:
        with output.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'detector_node', 'detector', '--ros-args',
                '-p', 'input_mode:=ros', '-p', 'image_topic:=/hk_camera/image_raw',
                '-p', 'enable_face:=true', '-p', 'enable_person:=false', '-p', 'enable_plate:=false',
                '-p', 'staging_dir:='+str(tmp_path/'staging')],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+10
            while time.monotonic() < deadline:
                assert process.poll() is None, output.read_text()
                rclpy.spin_once(fixture, timeout_sec=.05)
                if states and states[-1].get('ready') and states[-1].get('face'): break
            assert states and states[-1]['ready'] and states[-1]['face'], (states, output.read_text())
            assert states[-1]['face_requested'] and not states[-1]['person'] and not states[-1]['plate']
            assert states[-1]['recognition_mode'] == 'host' and not states[-1]['last_error']
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3)
        fixture.destroy_node(); rclpy.shutdown()


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


def test_all_features_off_closes_video_input_to_avoid_stale_frames_on_resume(tmp_path, monkeypatch):
    import threading
    import rclpy
    from rclpy.parameter import Parameter
    from detector_node.detector_node import DetectorNode
    from detector_node.detection_pipeline import Pipeline
    released = threading.Event()
    class Capture:
        def isOpened(self): return True
        def read(self): return True, np.zeros((120, 120, 3), np.uint8)
        def release(self): released.set()
    monkeypatch.setattr(cv2, 'VideoCapture', lambda source: Capture())
    rclpy.init()
    node = DetectorNode(parameter_overrides=[Parameter('staging_dir', value=str(tmp_path)),
                                             Parameter('input_mode', value='video'),
                                             Parameter('video_source', value='isolated-fake-video'),
                                             *[Parameter('enable_'+name, value=False)
                                               for name in ('face', 'person', 'plate')]])
    pipeline = Pipeline(lambda image: [(0, .9, [0, 0, 60, 100])])
    node.load_pipeline = lambda person, plate, face: pipeline
    try:
        node.set_parameters([Parameter('enable_person', value=True)])
        deadline = time.monotonic()+3
        while not node.state['frames'] and time.monotonic() < deadline: time.sleep(.02)
        assert node.state['frames'] and not released.is_set()
        node.set_parameters([Parameter('enable_person', value=False)])
        assert released.wait(2)
    finally:
        node.shutdown_worker(); node.destroy_node(); rclpy.shutdown()


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


@pytest.mark.parametrize('include_pose', [False, True])
def test_host_face_records_image_and_frame_pose_without_sdk_or_navigation(tmp_path, include_pose):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from cv_bridge import CvBridge
    from detector_node.detector_node import DetectorNode
    from detector_node.detection_pipeline import Pipeline
    from vision_events.node import Recorder
    rclpy.init()
    directory = tmp_path/'records'
    recorder = Recorder(parameter_overrides=[Parameter('data_dir', value=str(directory)),
                                             Parameter('map_id', value='test-map')])
    detector = DetectorNode(parameter_overrides=[Parameter('staging_dir', value=str(directory/'staging')),
                                                Parameter('enable_face', value=False),
                                                Parameter('enable_person', value=False),
                                                Parameter('enable_plate', value=False)])
    pipeline = Pipeline(face=lambda frame: [(-1, .95, [20, 25, 70, 85])])
    detector.load_pipeline = lambda person, plate, face: pipeline
    fixture = rclpy.create_node('host_face_fixture')
    publisher = fixture.create_publisher(Image, '/hk_camera/image_raw', 10)
    broadcaster = TransformBroadcaster(fixture)
    executor = SingleThreadedExecutor()
    for node in (fixture, recorder, detector): executor.add_node(node)
    def tick():
        stamp = fixture.get_clock().now().to_msg()
        transform = TransformStamped(); transform.header.stamp = stamp
        transform.header.frame_id = 'map'; transform.child_frame_id = 'base_link'
        transform.transform.translation.x = 3.25; transform.transform.rotation.w = 1.
        if include_pose: broadcaster.sendTransform(transform)
        message = CvBridge().cv2_to_imgmsg(np.full((120, 160, 3), 80, np.uint8), 'bgr8')
        message.header.stamp = stamp; publisher.publish(message)
        executor.spin_once(timeout_sec=.02)
    try:
        deadline = time.monotonic()+3
        if include_pose:
            while not recorder.pose.samples and time.monotonic() < deadline: tick()
            assert recorder.pose.samples
        assert recorder.face_sub is None
        detector.set_parameters([Parameter('enable_face', value=True)])
        deadline = time.monotonic()+5
        while not recorder.store.query()[2] and time.monotonic() < deadline: tick()
        event = recorder.store.query()[0][0]
        assert event['event_type'] == 'face_capture' and event['source'] == 'host_yunet'
        if include_pose:
            assert event['pose_valid'] and event['robot_x'] == 3.25
        else:
            assert not event['pose_valid'] and event['robot_x'] is None
        assert event['time_basis'] == 'ros_image_stamp'
        assert cv2.imread(event['image_path']).shape == (60, 50, 3)
        detector.set_parameters([Parameter('enable_face', value=False)])
        deadline = time.monotonic()+2
        while detector.state['face'] and time.monotonic() < deadline: tick()
        assert not detector.state['face']
        before = recorder.store.query()[2]
        recorder.on_raw(String(data=json.dumps(dict(event_type='face_capture', source='hikvision_sdk',
                                                     source_event_key='old-sdk', event_time_ns=time.time_ns()))))
        deadline = time.monotonic()+.4
        while time.monotonic() < deadline: tick()
        assert recorder.store.query()[2] == before
        assert not any('navigation' in name or 'patrol' in name for name in fixture.get_node_names())
    finally:
        detector.shutdown_worker(); recorder.shutdown_worker()
        for node in (detector, recorder, fixture): node.destroy_node()
        executor.shutdown(); rclpy.shutdown()


@pytest.mark.skipif(not os.environ.get('VISION_PUBLIC_SAMPLES'), reason='optional public model samples not configured')
def test_actual_models_archive_all_three_types_with_frame_pose(tmp_path):
    """Public sample inputs, installed weights, real detector and recorder nodes."""
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from cv_bridge import CvBridge
    from detector_node.detector_node import DetectorNode
    from vision_events.node import Recorder
    rclpy.init()
    directory = tmp_path/'records'
    recorder = Recorder(parameter_overrides=[Parameter('data_dir', value=str(directory)),
                                             Parameter('map_id', value='public-sample-map')])
    detector = DetectorNode(parameter_overrides=[Parameter('staging_dir', value=str(directory/'staging')),
                                                *[Parameter('enable_'+feature, value=False)
                                                  for feature in ('face', 'person', 'plate')]])
    fixture = rclpy.create_node('actual_models_public_sample_fixture')
    publisher = fixture.create_publisher(Image, '/hk_camera/image_raw', 10)
    broadcaster = TransformBroadcaster(fixture)
    executor = SingleThreadedExecutor()
    for node in (fixture, recorder, detector): executor.add_node(node)
    samples = Path(os.environ['VISION_PUBLIC_SAMPLES'])
    try:
        for feature, filename, kind, source in [('face', 'face.jpg', 'face_capture', 'host_yunet'),
                                                ('person', 'bus.jpg', 'person', 'host_yolo26n'),
                                                ('plate', 'plate.jpg', 'vehicle_plate', 'host_lpd_yunet_ocr')]:
            frame = cv2.imread(str(samples/filename)); assert frame is not None
            result = detector.set_parameters_atomically([Parameter('enable_'+name, value=(name == feature))
                                                         for name in ('face', 'person', 'plate')])
            assert result.successful
            deadline = time.monotonic()+12
            while not recorder.store.query(kind)[2] and time.monotonic() < deadline:
                stamp = fixture.get_clock().now().to_msg()
                transform = TransformStamped(); transform.header.stamp = stamp
                transform.header.frame_id = 'map'; transform.child_frame_id = 'base_link'
                transform.transform.translation.x = 2.75; transform.transform.rotation.w = 1.
                broadcaster.sendTransform(transform)
                message = CvBridge().cv2_to_imgmsg(frame, 'bgr8'); message.header.stamp = stamp
                publisher.publish(message)
                # Publish at 10 Hz and drain all callbacks, rather than flood
                # images/TF faster than a single-threaded test executor can read.
                until = time.monotonic()+.1
                while time.monotonic() < until: executor.spin_once(timeout_sec=.01)
            items, _, count = recorder.store.query(kind)
            assert count >= 1, detector.state
            assert all(item['source'] == source and item['pose_valid'] and item['robot_x'] == 2.75
                       and cv2.imread(item['image_path']) is not None for item in items), items
            if feature == 'plate': assert items[0]['plate_text'] == '苏ED51712'
    finally:
        detector.shutdown_worker(); recorder.shutdown_worker()
        for node in (detector, recorder, fixture): node.destroy_node()
        executor.shutdown(); rclpy.shutdown()
