"""Independent recorder. No action clients, cmd_vel, patrol or navigation imports."""
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_interfaces.srv import QueryEvents, SetRecordingContext
from .core import PoseHistory, Store, archive, canonical_event, approved_file


class Recorder(Node):
    def __init__(self, **kwargs):
        super().__init__('vision_events', **kwargs)
        project = Path(os.environ.get('ROBOT_PROJECT_ROOT', Path.home() / 'Remote_ctrol'))
        self.declare_parameter('data_dir', str(project / 'data/vision'))
        self.declare_parameter('face_dirs', [str(project / 'captures/hk_faces'),
                                             str(project / 'data/vision/native/hk_faces')])
        self.declare_parameter('max_pose_age_sec', .3)
        self.declare_parameter('map_id', '')
        self.declare_parameter('map_revision', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('upload_enabled', False)
        self.declare_parameter('backend_url', 'http://127.0.0.1:8888')
        self.directory = Path(self.get_parameter('data_dir').value).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.ownership = (self.directory / 'recorder.lock').open('a')
        try:
            fcntl.flock(self.ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another recorder owns this data directory') from None
        self.face_dirs = [Path(p).expanduser().resolve() for p in self.get_parameter('face_dirs').value]
        self.staging = self.directory / 'staging'
        self.staging.mkdir(exist_ok=True)
        self.roots = self.face_dirs + [self.staging]
        self.store = Store(self.directory, self.get_parameter('upload_enabled').value)
        self.pose = PoseHistory(self.get_parameter('max_pose_age_sec').value)
        self.pose.context(self.get_parameter('map_id').value, self.get_parameter('map_revision').value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.work = queue.Queue(maxsize=256)
        self.completed = queue.Queue()
        self.pending = set()
        self.scan = None
        self.stopping = threading.Event()
        self.last_error = ''
        self.errors = self.overflows = self.saved = 0
        self.event_pub = self.create_publisher(String, '/vision/events', 20)
        self.legacy_pub = self.create_publisher(String, '/detection_events', 20)
        self.status_pub = self.create_publisher(String, '/vision/status', 5)
        self.raw_sub = self.create_subscription(String, '/vision/raw_detections', self.on_raw, 100)
        self.face_sub = None
        try:
            from hk_camera.msg import FaceCapture
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.face_sub = self.create_subscription(FaceCapture, '/hk_camera/face_capture', self.on_face, qos)
        except ImportError:
            self.last_error = 'hk_camera messages unavailable; directory recovery and host detections remain available'
        self.query_srv = self.create_service(QueryEvents, '/vision/query_events', self.query)
        self.context_srv = self.create_service(SetRecordingContext, '/vision/set_recording_context', self.set_context)
        self.create_timer(.05, self.sample_pose)
        self.create_timer(.1, self.flush)
        self.create_timer(.25, self.scan_files)
        self.create_timer(1., self.status)
        self.worker = threading.Thread(target=self.record_worker, daemon=True)
        self.worker.start()
        self.uploader = None
        if self.store.upload:
            self.uploader = threading.Thread(target=self.upload_worker, daemon=True)
            self.uploader.start()
        self.get_logger().info('Standalone vision recorder ready; navigation, localization and backend are optional')

    def sample_pose(self):
        try:
            transform = self.buffer.lookup_transform('map', self.get_parameter('base_frame').value, rclpy.time.Time())
            stamp = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec
            # A static map/base chain or old dynamic transform is not live localization.
            if not stamp or abs(self.get_clock().now().nanoseconds - stamp) > 500_000_000:
                return
            q = transform.transform.rotation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            self.pose.add(stamp, transform.transform.translation.x, transform.transform.translation.y, yaw)
        except Exception:
            pass  # Missing localization is a supported stationary-duty mode.

    def set_context(self, request, response):
        if len(request.map_id) > 512 or len(request.map_revision) > 512:
            response.message = 'map label too long'
            return response
        self.pose.context(request.map_id, request.map_revision)
        response.success = True
        response.message = 'Map label set; only fresh TF produces coordinates' if request.map_id else 'Map association cleared; image recording continues'
        return response

    def query(self, request, response):
        try:
            events, cursor, total = self.store.query(request.event_type, request.limit, request.cursor, request.event_id)
            response.success = True
            response.events_json = json.dumps(events, ensure_ascii=False, allow_nan=False)
            response.next_cursor, response.total = cursor, total
        except Exception as error:
            response.message = str(error)
        return response

    def on_face(self, message):
        if message.metadata_path:
            self.load_file(Path(message.metadata_path))

    def on_raw(self, message):
        try:
            raw = json.loads(message.data)
            if raw.get('event_type') in ('person', 'vehicle_plate'):
                self.enqueue(raw)
            elif raw.get('metadata_path'):
                self.load_file(Path(raw['metadata_path']))
            else:
                self.enqueue(raw)
        except Exception as error:
            self.fail(error)

    def load_file(self, path):
        try:
            path = approved_file(path, self.roots)
            raw = json.loads(path.read_text())
            if raw.get('source') == 'hikvision_sdk_face_capture':
                raw = dict(event_type='face_capture', source='hikvision_sdk',
                           source_event_key=str(path), event_time_ns=int(raw['received_ns']),
                           time_basis='host_alarm_receipt', camera_time_raw=raw.get('camera_time_raw'),
                           image_path=raw.get('face_path', ''), scene_path=raw.get('background_path', ''),
                           quality_score=None if raw.get('score') == 255 else raw.get('score'),
                           face_id=raw.get('face_id'), target_id=raw.get('target_id'),
                           label='人脸抓拍', confidence=None)
            elif path.parent != self.staging or raw.get('event_type') not in ('person', 'vehicle_plate'):
                return
            else:
                raw['metadata_path'] = str(path)
            self.enqueue(raw)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.fail(error)

    def enqueue(self, raw):
        event = canonical_event(raw)
        key = event['event_id']
        if key in self.pending or self.store.exists(key):
            if key not in self.pending and raw.get('metadata_path'):
                stored = self.store.query(event_id=key)[0]
                if stored and stored[0].get('media_status') == 'ready':
                    self.cleanup_staging(raw)
            return
        event.update(self.pose.associate(event['event_time_ns']))
        try:
            self.work.put_nowait(event)
            self.pending.add(key)
        except queue.Full:
            self.overflows += 1
            self.last_error = 'Recorder queue full; durable native/staging events will be rescanned'

    def scan_files(self):
        if self.work.qsize() > 128:
            return
        if self.scan is None:
            self.scan = (p for root in self.roots if root.is_dir() for p in root.glob('*.json'))
        for _ in range(40):
            try:
                self.load_file(next(self.scan))
            except StopIteration:
                self.scan = None
                break

    def fail(self, error):
        self.errors += 1
        self.last_error = str(error)

    def cleanup_staging(self, event):
        for value in (event.get('metadata_path'), event.get('native_image_path', event.get('image_path'))):
            if value:
                path = Path(value)
                if path.parent == self.staging:
                    path.unlink(missing_ok=True)

    def record_worker(self):
        while not self.stopping.is_set() or not self.work.empty():
            try:
                event = self.work.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                try:
                    archive(event, self.directory, self.roots)
                except (ValueError, OSError) as error:
                    event.update(media_status='error', media_error=str(error), image_path='')
                inserted = self.store.insert(event)
                if event.get('media_status') == 'ready' and event.get('metadata_path'):
                    self.cleanup_staging(event)
                self.completed.put((event, inserted, None))
            except Exception as error:
                self.completed.put((event, False, str(error)))
            finally:
                self.work.task_done()

    def flush(self):
        for _ in range(100):
            try:
                event, inserted, error = self.completed.get_nowait()
            except queue.Empty:
                return
            self.pending.discard(event['event_id'])
            if error:
                self.fail(error)
            elif inserted:
                self.saved += 1
                if event.get('media_status') != 'ready':
                    self.fail(event.get('media_error', 'event has no image'))
                self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
                legacy = dict(type='detection', event='person_face' if event['event_type']=='face_capture' else event['event_type'],
                              label=event['label'], score=event.get('confidence') or 0.,
                              ts=event['event_time_ns']/1e9, event_id=event['event_id'],
                              image='api/vision/media/'+event['event_id'], metadata=event)
                self.legacy_pub.publish(String(data=json.dumps(legacy, ensure_ascii=False)))

    def status(self):
        total = self.store.query(limit=1)[2]
        free = os.statvfs(self.directory)
        value = dict(component='recorder', ready=True, records=total, saved_this_run=self.saved,
                     queue=self.work.qsize(), errors=self.errors, overflows=self.overflows,
                     last_error=self.last_error, data_dir=str(self.directory),
                     map_id=self.pose.contexts[-1][2], free_bytes=free.f_bavail*free.f_frsize,
                     upload_enabled=self.store.upload)
        self.status_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def upload_worker(self):
        import requests
        url = self.get_parameter('backend_url').value.rstrip('/') + '/api/records/detections'
        while not self.stopping.wait(1.):
            event = self.store.pending()
            if not event:
                continue
            payload = dict(event_id=event['event_id'], ts=event['event_time_ns']/1e9,
                           event='person_face' if event['event_type']=='face_capture' else event['event_type'],
                           label=event['label'], score=event.get('confidence') or 0.,
                           image='api/vision/media/' + event['event_id'], metadata=event)
            try:
                response = requests.post(url, json=payload, timeout=2.)
                if response.status_code == 200:
                    self.store.delivered(event['event_id'])
            except requests.RequestException:
                pass  # outbox remains committed; no live event loss

    def shutdown_worker(self):
        self.stopping.set()
        self.worker.join(timeout=10.)
        if self.uploader:
            self.uploader.join(timeout=3.)
        if not self.worker.is_alive():
            self.store.close()
        self.ownership.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Recorder()
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
