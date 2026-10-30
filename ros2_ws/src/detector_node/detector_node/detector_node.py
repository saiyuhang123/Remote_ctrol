"""Standalone CPU face/person/plate detection with local event records."""
import json
import math
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
from .detection_pipeline import Pipeline, YoloCpu, YunetCpu, LpdYunetCpu, PlateTextCpu, ModelSchedule


class DetectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('detector_node', **kwargs)
        project = Path(os.environ.get('ROBOT_PROJECT_ROOT', Path.home() / 'Remote_ctrol'))
        defaults = dict(input_mode='ros', image_topic='/hk_camera/image_raw',
                        video_source='rtsp://127.0.0.1:8554/cam', enable_person=True,
                        enable_plate=True, enable_face=True, max_inference_hz=1., cpu_threads=2,
                        staging_dir=str(project/'data/vision/staging'), conf_threshold=.45,
                        track_ttl_sec=2., plate_confirmations=2, plate_ocr_interval_sec=2.,
                        face_conf_threshold=.8, face_input_width=960, face_capture_interval_sec=2.,
                        plate_conf_threshold=.8, plate_input_width=640)
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
        plate_ocr_interval = self.get_parameter('plate_ocr_interval_sec').value
        if not math.isfinite(plate_ocr_interval) or not 0. <= plate_ocr_interval <= 60.:
            raise ValueError('plate_ocr_interval_sec must be 0..60')
        if not .1 <= self.get_parameter('face_conf_threshold').value <= 1.:
            raise ValueError('face_conf_threshold must be 0.1..1')
        if not 320 <= self.get_parameter('face_input_width').value <= 1920:
            raise ValueError('face_input_width must be 320..1920')
        if not .25 <= self.get_parameter('face_capture_interval_sec').value <= 3600.:
            raise ValueError('face_capture_interval_sec must be 0.25..3600')
        if not .1 <= self.get_parameter('plate_conf_threshold').value <= 1.:
            raise ValueError('plate_conf_threshold must be 0.1..1')
        if not 320 <= self.get_parameter('plate_input_width').value <= 1920:
            raise ValueError('plate_input_width must be 320..1920')
        cv2.setNumThreads(self.get_parameter('cpu_threads').value)
        self.lock = threading.Lock()
        self.latest = None
        self.image_sequence = 0
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
        self.model_errors = []
        self.state = dict(component='detector', ready=False, frames=0, events=0,
                          last_error='', input_ready=False, face=False, person=False, plate=False,
                          face_backend='opencv_yunet', person_backend='yolo26n_openvino',
                          plate_backend='lpd_yunet+hyperlpr3_rec', recognition_mode='host',
                          plate_ocr_calls=0, plate_ocr_skipped=0,
                          model_schedule='staggered', model_frames={}, model_inference_ms={})
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
            if parameter.name in ('enable_face', 'enable_person', 'enable_plate') and not isinstance(parameter.value, bool):
                return SetParametersResult(successful=False, reason='feature switches must be bool')
            if parameter.name == 'max_inference_hz' and (not isinstance(parameter.value, (float, int)) or not .1 <= parameter.value <= 30):
                return SetParametersResult(successful=False, reason='inference rate must be 0.1..30')
            if parameter.name not in ('enable_face', 'enable_person', 'enable_plate', 'max_inference_hz'):
                return SetParametersResult(successful=False, reason='restart detector to change input/model configuration')
        return SetParametersResult(successful=True)

    def on_image(self, message):
        with self.lock:
            self.latest = message
            self.image_sequence += 1
            self.last_frame_mono = time.monotonic()

    def publish_status(self):
        with self.lock:
            state = dict(self.state)
        state['input_ready'] = time.monotonic()-self.last_frame_mono < 3.
        for feature in ('face', 'person', 'plate'):
            state[feature + '_requested'] = self.get_parameter('enable_' + feature).value
            state[feature] = bool(state[feature] and state[feature + '_requested'])
        self.status_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def publish_events(self):
        with self.lock:
            events, self.outgoing = self.outgoing, []
        for event in events:
            self.pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def load_pipeline(self, need_person, need_plate, need_face):
        self.model_errors = []
        share = Path(get_package_share_directory('detector_node'))
        if self.pipeline is None:
            self.pipeline = Pipeline(track_ttl=self.get_parameter('track_ttl_sec').value,
                                     plate_confirmations=self.get_parameter('plate_confirmations').value,
                                     plate_ocr_interval=self.get_parameter('plate_ocr_interval_sec').value,
                                     face_capture_interval=self.get_parameter('face_capture_interval_sec').value)
        if need_person and self.pipeline.detector is None:
            try:
                self.pipeline.detector = YoloCpu(share/'models/yolo26n_openvino_model/yolo26n.xml',
                                                self.get_parameter('cpu_threads').value,
                                                self.get_parameter('conf_threshold').value)
            except Exception as error:
                self.model_errors.append('YOLO26n person model: ' + str(error))
        elif not need_person:
            self.pipeline.detector = None
        if need_face and self.pipeline.face is None:
            try:
                self.pipeline.face = YunetCpu(share/'models/face_detection_yunet.onnx',
                                             self.get_parameter('face_conf_threshold').value,
                                             self.get_parameter('face_input_width').value)
            except Exception as error:
                self.model_errors.append('face model: ' + str(error))
        elif not need_face:
            self.pipeline.face = None
        if need_plate and self.pipeline.plate_detector is None:
            try:
                self.pipeline.plate_detector = LpdYunetCpu(share/'models/license_plate_detection_lpd_yunet_2023mar.onnx',
                                                         self.get_parameter('plate_conf_threshold').value,
                                                         self.get_parameter('plate_input_width').value)
            except Exception as error:
                self.model_errors.append('LPD-YuNet plate model: ' + str(error))
        elif not need_plate:
            self.pipeline.plate_detector = None
        if need_plate and self.pipeline.plate is None:
            try:
                self.pipeline.plate = PlateTextCpu(share/'models/plate_recognition_hyperlpr3.onnx',
                                                   self.get_parameter('cpu_threads').value)
            except Exception as error:
                self.model_errors.append('plate text model: ' + str(error))
        elif not need_plate:
            self.pipeline.plate = None
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
            source = {'face_capture': 'host_yunet', 'person': 'host_yolo26n', 'vehicle_plate': 'host_lpd_yunet_ocr'}
            result.update(source=source[result['event_type']],
                          source_instance_id=self.instance,
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
        schedule = ModelSchedule()
        cached_message, cached_frame, bridge = None, None, None
        processed_sequences = {}
        model_names = {0: 'person', -1: 'face', -2: 'plate'}
        try:
            while not self.stop_event.is_set():
                person, plate, face = (self.get_parameter(key).value for key in ('enable_person', 'enable_plate', 'enable_face'))
                if not person and not plate and not face:
                    self.pipeline = None
                    cached_message, cached_frame = None, None
                    processed_sequences.clear()
                    if cap is not None:
                        cap.release()
                        cap = None
                    with self.lock:
                        self.state.update(ready=True, person=False, plate=False, face=False, last_error='')
                        self.latest = None
                    self.stop_event.wait(.1)
                    continue
                enabled = ({0} if person else set()) | ({-1} if face else set()) | ({-2} if plate else set())
                model, due = schedule.next(enabled, self.get_parameter('max_inference_hz').value,
                                           time.monotonic())
                if self.stop_event.wait(max(0., due-time.monotonic())):
                    break
                try:
                    pipeline = self.load_pipeline(person, plate, face)
                    with self.lock:
                        self.state.update(ready=not self.model_errors,
                                          face=face and pipeline.face is not None,
                                          person=person and pipeline.detector is not None,
                                          plate=plate and pipeline.plate is not None and pipeline.plate_detector is not None,
                                          last_error='; '.join(self.model_errors))
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
                            message = self.latest
                            sequence = self.image_sequence
                            frame_age = time.monotonic() - self.last_frame_mono
                        if (message is None or frame_age > 3. or
                                processed_sequences.get(model) == sequence):
                            # Reserve this model's slot without treating a
                            # frozen image as another independent observation.
                            skipped = time.monotonic()
                            schedule.complete(model, skipped, skipped)
                            continue
                        # Different models can share the latest full-resolution
                        # frame; do not deserialize/convert it again per model.
                        if message is not cached_message:
                            if bridge is None:
                                from cv_bridge import CvBridge
                                bridge = CvBridge()
                            cached_frame = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                            cached_message = message
                        frame = cached_frame
                        stamp = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
                        basis = 'ros_image_stamp' if stamp else 'host_image_receipt'
                        stamp = stamp or time.time_ns()
                    start = time.monotonic()
                    self.save_events(pipeline.process(frame, person, plate, face=face,
                                                      run_classes={model}), stamp, basis)
                    if not video:
                        processed_sequences[model] = sequence
                    finished = time.monotonic()
                    schedule.complete(model, start, finished)
                    with self.lock:
                        self.state.update(ready=not self.model_errors,
                                          face=face and pipeline.face is not None,
                                          person=person and pipeline.detector is not None,
                                          plate=plate and pipeline.plate is not None and pipeline.plate_detector is not None,
                                          inference_ms=(finished-start)*1000,
                                          plate_ocr_calls=pipeline.plate_ocr_calls,
                                          plate_ocr_skipped=pipeline.plate_ocr_skipped)
                        self.state['last_error'] = '; '.join(self.model_errors)
                        name = model_names[model]
                        self.state['model_frames'][name] = self.state['model_frames'].get(name, 0) + 1
                        self.state['model_inference_ms'][name] = (finished-start)*1000
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
