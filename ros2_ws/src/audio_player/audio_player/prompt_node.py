"""Read-only ROS2 status observer for the bundled robot announcement clips."""
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_action_status_default, qos_profile_sensor_data)
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from patrol_interfaces.msg import OperatorHeartbeat, PatrolStatus, ExecutorHeartbeat
from std_msgs.msg import Bool, String
from yunle_msgs.msg import Battery
import tf2_ros

from .notifications import PromptRules, load_config


class StatusPrompts(Node):
    def __init__(self, config_path):
        # A launch remap for the playback node must not rename this second node.
        super().__init__('robot_voice_notifications', use_global_arguments=False)
        self.config = load_config(config_path)
        self.rules = PromptRules(self.config)
        self._pub = self.create_publisher(String, '/audio/notify', 20)
        self._audio, self._audio_time = {}, 0.
        self._estop_time = 0.
        self._call_time = 0.
        self._ready_since = None
        self._tf_was_ready = False
        self._tf_missing_since = None
        self._heartbeat_time = 0.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PatrolStatus, '/patrol/status', self._patrol, latched)
        self.create_subscription(ExecutorHeartbeat, '/patrol/executor_heartbeat', self._heartbeat, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._nav,
                                 qos_profile_action_status_default)
        self.create_subscription(Odometry, self.config.get('odom_topic', '/odom'), self._odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Battery, self.config.get('battery_topic', '/battery_status'),
                                 self._battery,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/emergency_stop', self._estop, 10)
        self.create_subscription(OperatorHeartbeat, '/patrol/operator_heartbeat', self._operator, 10)
        for topic, handler in (
            ('/patrol/events', self.rules.patrol_event),
            ('/robot_gateway/events', self.rules.gateway_event),
            ('/vision/events', self.rules.vision),
            ('/gas_detector/data', self.rules.gas),
            ('/gas_detector/status', lambda d: self.rules.device('gas', d)),
            ('/patrol/guard_status', self.rules.guard),
            ('/audio/intercom_state', self._call),
            ('/audio/status', self._on_audio),
            ('/capture_done', self._capture),
            ('/capture/status', self._capture_status),
            ('/vision/detector_status', self._detector_status),
        ):
            self.create_subscription(String, topic,
                lambda message, callback=handler: self._json(message, callback),
                latched if topic == '/audio/intercom_state' else 20)
        self.create_subscription(String, '/hk_camera/status', self._camera_status, 10)
        self._tf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf, self)
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(.2, self._tick)
        self.get_logger().info('30 fixed voice prompts enabled; status observation only')

    def _send(self, notices):
        for item in notices or []:
            self._pub.publish(String(data=json.dumps(item.__dict__, ensure_ascii=False)))

    def _json(self, message, callback):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict):
                self._send(callback(data))
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.get_logger().debug('Ignoring malformed notification input: '+str(error))

    def _patrol(self, message):
        fields = ('instance_id', 'sequence', 'run_id', 'route_id', 'state', 'loop_index',
                  'step_index', 'current_goal_id', 'active', 'error_code')
        self._send(self.rules.patrol({key: getattr(message, key) for key in fields}))

    def _heartbeat(self, message):
        self._heartbeat_time = time.monotonic()
        self.rules.active = bool(message.active)
        if message.nav_goal_id:
            self.rules.owned_nav.add(message.nav_goal_id)

    def _nav(self, message):
        self._send(self.rules.nav_status([(bytes(item.goal_info.goal_id.uuid).hex(), int(item.status))
                                         for item in message.status_list]))

    def _odom(self, message):
        twist = message.twist.twist
        self.rules.odom(math.hypot(twist.linear.x, twist.linear.y), twist.angular.z)

    def _estop(self, message):
        self._estop_time = time.monotonic()
        self._send(self.rules.on_estop(message.data))

    def _battery(self, message):
        if math.isfinite(message.voltage) and message.voltage > 0:
            self._send(self.rules.battery(float(message.capacity)))

    def _operator(self, message):
        if message.client_id == 'desktop_ui':
            self._send(self.rules.operator(message.enabled))

    def _on_audio(self, data):
        self._audio, self._audio_time = data, time.monotonic()

    def _call(self, data):
        self._call_time = time.monotonic()
        return self.rules.call(data)

    def _capture(self, data):
        # Only explicitly manual requests carry this label; inspection service
        # captures must not add one "photo saved" announcement per view.
        if data.get('ok') and data.get('label') in ('manual', '网页手动抓拍', '手动抓拍'):
            return self.rules.notify('capture_saved', str(data.get('request_id', '')))
        return []

    def _capture_status(self, data):
        previous = self.rules.healthy.get('capture_ready', False)
        if data.get('ready'):
            self.rules.healthy['capture_ready'] = True
        if previous and data.get('ready') is False and data.get('last_error'):
            return self.rules.notify('camera_abnormal')
        return []

    def _detector_status(self, data):
        error = str(data.get('last_error', ''))
        if any(word in error.lower() for word in ('video', 'camera', 'frame', '视频', '相机')):
            return self.rules.device('camera', data)
        # Model loading errors are reported as device errors only when an
        # enabled detector previously worked, not while a model is starting.
        if data.get('ready'):
            self.rules.healthy['detector_ready'] = True
        if error and self.rules.healthy.get('detector_ready'):
            return self.rules.device('detector', data)
        return []

    def _camera_status(self, message):
        raw = message.data.lower()
        if 'error' in raw or 'failed' in raw:
            self._send(self.rules.device('camera', {'error': raw}))
        elif raw in ('streaming_active', 'stream_active', 'stopped', 'closed'):
            self.rules.healthy['camera'] = False

    def _tick(self):
        now = time.monotonic()
        if self._call_time and now-self._call_time > 3:
            self.rules.intercom = False  # Gateway died; never invent a call-ended event.
        # A lost executor must not suppress direct Nav2 observations forever.
        if self._heartbeat_time and now-self._heartbeat_time > 3:
            self.rules.active = False
        self._send(self.rules.tick())
        moving_task = self.rules.active or any(state in (1, 2, 3) for state in self.rules.nav.values())
        tf_ready = False
        try:
            tf = self._tf.lookup_transform('map', self.config.get('base_frame', 'base_link'), Time())
            stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            tf_ready = abs(self.get_clock().now().nanoseconds/1e9-stamp) <= 2
        except Exception:
            pass
        if tf_ready:
            self._tf_was_ready, self._tf_missing_since = True, None
        elif moving_task and self._tf_was_ready:
            if self._tf_missing_since is None:
                self._tf_missing_since = now
            elif now-self._tf_missing_since > 2:
                self._send(self.rules.notify('localization_abnormal'))
        if moving_task and self.rules.odom_time is not None and now-self.rules.odom_time > 2:
            self._send(self.rules.notify('sensor_abnormal'))
        ready = (tf_ready and self._nav_client.server_is_ready()
                 and self.rules.odom_time is not None and now-self.rules.odom_time <= 1
                 and self._estop_time and now-self._estop_time <= 3
                 and self._audio.get('ready') and now-self._audio_time <= 3)
        if ready and not self.rules.ready_sent:
            if self._ready_since is None:
                self._ready_since = now
            elif now-self._ready_since >= 1:
                self._send(self.rules.notify('system_ready', 'startup'))
                self.rules.ready_sent = True
        elif not ready:
            self._ready_since = None
