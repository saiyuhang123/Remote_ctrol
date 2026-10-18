"""Standalone person/plate detection; human-face work belongs to Hikvision."""
import json
import os
from pathlib import Path
import threading
import time
import uuid
import fcntl
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from .detection_pipeline import Pipeline, YoloCpu


class DetectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('detector_node', **kwargs)
        project = Path(os.environ.get('ROBOT_PROJECT_ROOT', Path.home() / 'Remote_ctrol'))
        defaults = dict(input_mode='ros', image_topic='/hk_camera/image_raw',
                        video_source='rtsp://127.0.0.1:8554/cam', enable_person=True,
                        enable_plate=True, max_inference_hz=4., cpu_threads=2,
                        staging_dir=str(project/'data/vision/staging'), conf_threshold=.45,
                        track_ttl_sec=2., plate_confirmations=2)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        if self.get_parameter('input_mode').value not in ('ros', 'video'):
            raise ValueError('input_mode must be ros or video')
        if not .1 <= self.get_parameter('max_inference_hz').value <= 30.:
            raise ValueError('max_inference_hz must be 0.1..30')
        if not 1 <= self.get_parameter('cpu_threads').value <= 16:
            raise ValueError('cpu_threads must be 1..16')
        if not 1 <= self.get_parameter('plate_confirmations').value <= 10 or self.get_parameter('track_ttl_sec').value <= 0:
            raise ValueError('plate_confirmations must be 1..10 and track_ttl_sec must be positive')
        cv2.setNumThreads(self.get_parameter('cpu_threads').value)
        self.lock = threading.Lock()
        self.latest = None
        self.stop_event = threading.Event()
        self.instance = str(uuid.uuid4())
        self.staging = Path(self.get_parameter('staging_dir').value).expanduser().resolve()
        self.staging.mkdir(parents=True, exist_ok=True)
        self.ownership = (self.staging/'detector.lock').open('a')
        try:
            fcntl.flock(self.ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another detector owns this staging directory') from None
        self.pipeline = None
        self.state = dict(component='detector', ready=False, frames=0, events=0,
                          last_error='', input_ready=False, person=False, plate=False)
        self.outgoing = []
        self.pub = self.create_publisher(String, '/vision/raw_detections', 100)
        self.status_pub = self.create_publisher(String, '/vision/detector_status', 5)
        self.subscription = None
        self.last_frame_mono = 0.
        if self.get_parameter('input_mode').value == 'ros':
            self.subscription = self.create_subscription(Image, self.get_parameter('image_topic').value,
                                                          self.on_image, qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.create_timer(.1, self.publish_events)
        self.create_timer(1., self.publish_status)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name in ('enable_person', 'enable_plate') and not isinstance(parameter.value, bool):
                return SetParametersResult(successful=False, reason='feature switches must be bool')
            if parameter.name == 'max_inference_hz' and (not isinstance(parameter.value, (float, int)) or not .1 <= parameter.value <= 30):
                return SetParametersResult(successful=False, reason='inference rate must be 0.1..30')
            if parameter.name not in ('enable_person', 'enable_plate', 'max_inference_hz'):
                return SetParametersResult(successful=False, reason='restart detector to change input/model configuration')
        return SetParametersResult(successful=True)

    def on_image(self, message):
        with self.lock:
            self.latest = message
            self.last_frame_mono = time.monotonic()

    def publish_status(self):
        with self.lock:
            state = dict(self.state)
        state['input_ready'] = time.monotonic()-self.last_frame_mono < 3.
        self.status_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def publish_events(self):
        with self.lock:
            events, self.outgoing = self.outgoing, []
        for event in events:
            self.pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def load_pipeline(self, need_plate):
        if self.pipeline is None:
            share = Path(get_package_share_directory('detector_node'))
            self.pipeline = Pipeline(YoloCpu(share/'models/yolov8n_openvino_model/yolov8n.xml',
                                            self.get_parameter('cpu_threads').value,
                                            self.get_parameter('conf_threshold').value),
                                     track_ttl=self.get_parameter('track_ttl_sec').value,
                                     plate_confirmations=self.get_parameter('plate_confirmations').value)
        if need_plate and self.pipeline.plate is None:
            try:
                import hyperlpr3
                self.pipeline.plate = hyperlpr3.LicensePlateCatcher()
            except Exception as error:
                with self.lock:
                    self.state['last_error'] = 'plate model: ' + str(error)
        return self.pipeline

    def save_events(self, events, stamp, time_basis):
        for result in events:
            image = result.pop('image')
            key = str(uuid.uuid4())
            path = self.staging / (key + '.jpg')
            temp = self.staging / (key + '.tmp.jpg')
            if not cv2.imwrite(str(temp), image):
                raise RuntimeError('cannot write detection image')
            os.replace(temp, path)
            result.update(source='host_yolo_plate', source_instance_id=self.instance,
                          source_event_key=self.instance+'/'+key, event_time_ns=stamp,
                          received_time_ns=time.time_ns(), time_basis=time_basis, image_path=str(path))
            metadata = self.staging / (key + '.json')
            temporary = self.staging / (key + '.json.tmp')
            temporary.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False))
            os.replace(temporary, metadata)
            result['metadata_path'] = str(metadata)
            with self.lock:
                self.outgoing.append(result)
                self.state['events'] += 1

    def run(self):
        cap = None
        video = self.get_parameter('input_mode').value == 'video'
        source = self.get_parameter('video_source').value
        next_frame = time.monotonic()
        try:
            while not self.stop_event.is_set():
                person, plate = (self.get_parameter(key).value for key in ('enable_person', 'enable_plate'))
                if not person and not plate:
                    self.pipeline = None
                    with self.lock:
                        self.state.update(ready=True, person=False, plate=False)
                        self.latest = None
                    self.stop_event.wait(.1)
                    continue
                if self.stop_event.wait(max(0., next_frame-time.monotonic())):
                    break
                next_frame = time.monotonic() + 1./self.get_parameter('max_inference_hz').value
                try:
                    pipeline = self.load_pipeline(plate)
                    if not plate:
                        pipeline.plate = None
                    if video:
                        if cap is None or not cap.isOpened():
                            if cap:
                                cap.release()
                            cap = cv2.VideoCapture(source)
                            if not cap.isOpened():
                                raise RuntimeError('video source unavailable')
                            pipeline.reset()
                        ok, frame = cap.read()
                        stamp, basis = time.time_ns(), 'host_video_receipt'
                        if not ok:
                            if Path(source).is_file():
                                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                            else:
                                cap.release()
                            raise RuntimeError('video stream ended/disconnected')
                        self.last_frame_mono = time.monotonic()
                    else:
                        with self.lock:
                            message, self.latest = self.latest, None
                        if message is None:
                            continue
                        from cv_bridge import CvBridge
                        frame = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
                        stamp = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
                        basis = 'ros_image_stamp' if stamp else 'host_image_receipt'
                        stamp = stamp or time.time_ns()
                    start = time.monotonic()
                    self.save_events(pipeline.process(frame, person, plate), stamp, basis)
                    with self.lock:
                        self.state.update(ready=True, person=person, plate=plate and pipeline.plate is not None,
                                          inference_ms=(time.monotonic()-start)*1000)
                        if not plate or pipeline.plate is not None:
                            self.state['last_error'] = ''
                        self.state['frames'] += 1
                except Exception as error:
                    with self.lock:
                        self.state.update(ready=False, last_error=str(error))
                    self.stop_event.wait(1.)
        finally:
            if cap:
                cap.release()

    def shutdown_worker(self):
        self.stop_event.set()
        self.worker.join(timeout=8.)
        if not self.worker.is_alive():
            self.ownership.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.shutdown_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
