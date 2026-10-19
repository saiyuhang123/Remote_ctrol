#!/usr/bin/env python3
"""WebSocket gateway for the factory-LAN web MVP.

Ownership rules implemented here:
* one browser controller at a time, server-issued random session token;
* local desktop OperatorHeartbeat has priority and revokes web control;
* the /cmd_vel_teleop publisher is created only while a web owner has
  manual control, and is removed after publishing zero;
* web route/return tasks run through the local robot_patrol action server,
  and are canceled when the browser disconnects or loses the lease;
* arbitrary initial-pose and Nav2 target commands are not implemented.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

import rclpy
import websockets
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from hk_camera.srv import ControlPTZ, GetPTZPose
from nav_msgs.msg import Odometry, Path as NavPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_action_status_default, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
import tf2_ros

from patrol_interfaces.action import ReturnHome, RunRoute
from patrol_interfaces.msg import OperatorHeartbeat, PatrolStatus
from patrol_interfaces.srv import ControlRun

CONFIG_CANDIDATES = [
    Path(os.environ.get("ROBOT_PROJECT_ROOT", str(Path.home() / "Remote_ctrol")))
    / "config" / "config.yaml",
    Path.home() / "Remote_ctrol" / "config" / "config.yaml",
]
TARGET_FRAME = "map"
FALLBACK_FRAME = "odom"
PTZ_COMMANDS = {
    "up": 21, "down": 22, "left": 23, "right": 24,
    "zoom_in": 11, "zoom_out": 12,
}
TASK_STATES = {
    0: "idle", 1: "validating", 2: "dispatching", 3: "navigating",
    4: "settling", 5: "dwelling", 6: "pausing", 7: "paused",
    8: "canceling", 9: "canceled", 10: "succeeded", 11: "failed",
    12: "interrupted", 13: "stopping",
}
GOAL_STATES = {
    GoalStatus.STATUS_UNKNOWN: "unknown", GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing", GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


def _clamp(value, limit):
    return max(-limit, min(limit, float(value)))


def _json_status(raw: str) -> dict:
    try:
        value = json.loads(raw)
        return value if isinstance(value, dict) else {}
    except (TypeError, ValueError):
        return {}


def _safe_ready(client) -> bool:
    try:
        return bool(client.service_is_ready())
    except Exception:  # noqa: BLE001 - ROS context teardown
        return False


def _patrol_status_dict(message: PatrolStatus) -> dict:
    return {
        "instance_id": message.instance_id,
        "sequence": message.sequence,
        "run_id": message.run_id,
        "route_id": message.route_id,
        "map_id": message.map_id,
        "state": int(message.state),
        "state_name": TASK_STATES.get(int(message.state), str(message.state)),
        "loop_index": message.loop_index,
        "total_loops": message.total_loops,
        "step_index": message.step_index,
        "step_count": message.step_count,
        "current_point_name": message.current_point_name,
        "dwell_remaining": message.dwell_remaining,
        "error_code": message.error_code,
        "message": message.message,
        "current_goal_id": message.current_goal_id,
        "active": bool(message.active),
    }


class GatewayNode(Node):
    def __init__(self, cfg):
        super().__init__("robot_gateway")
        control = cfg["control"]
        self._max_linear = float(control["max_linear"])
        self._max_angular = float(control["max_angular"])
        self._cmd_timeout = float(control["cmd_timeout"])
        self._lease_timeout = 1.0
        ros_cfg = cfg.get("ros", {})
        self._base_frame = ros_cfg.get("base_frame", "base_link")
        self._cmd_topic = ros_cfg.get("cmd_vel_topic", "/cmd_vel_teleop")

        # Deliberately lazy. Creating this publisher at startup makes the local
        # motion interlock treat the gateway as a second controller.
        self._cmd_pub = None
        self._pub_estop = self.create_publisher(Bool, "/emergency_stop", 10)
        self._operator_pub = self.create_publisher(
            OperatorHeartbeat, "/patrol/operator_heartbeat", 10)

        self._lock = threading.RLock()
        self._control_lock = threading.RLock()
        self._task_lock = threading.RLock()
        self._data_lock = threading.RLock()
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_time = 0.0
        # None = no fresh /emergency_stop feedback. The gateway never writes a
        # synthetic false at startup; that would clear a local safety latch.
        self._estopped: bool | None = None
        self._estop_time = 0.0
        self._estop_timeout = 1.5
        self._stale_logged = True

        self._owner_session = ""
        self._owner_ws = None
        self._control_last_heartbeat = 0.0
        self._control_mode = "idle"  # idle/manual/route/return
        self._release_scheduled = False
        self._local_operator_enabled = False
        self._local_operator_time = 0.0

        self._task_kind = ""
        self._task_request_id = ""
        self._task_goal_handle = None
        self._task_cancel_pending = False
        self._task_started_at = 0.0
        self._task_final = {}
        self._pending_return = None

        self._odom: Odometry | None = None
        self._scan: LaserScan | None = None
        self._global_path: NavPath | None = None
        self._local_path: NavPath | None = None
        self._nav_statuses: dict[str, str] = {}
        self._nav_status_time = 0.0
        self._patrol_status: dict = {}
        self._detector_status: dict = {}
        self._detector_time = 0.0
        self._face_status: dict = {}
        self._face_time = 0.0
        self._camera_status: dict = {}
        self._camera_time = 0.0
        self._recorder_status: dict = {}
        self._recorder_time = 0.0
        self._gas_status: dict = {}
        self._gas_data: dict = {}
        self._gas_status_time = 0.0
        self._gas_data_time = 0.0
        self._ptz_patrol_status: dict = {}
        self._ptz_patrol_time = 0.0
        self._ptz_pose: dict = {}
        self._ptz_pose_time = 0.0
        self._ptz_jog_active = False
        self._ptz_jog_command = 0
        self._ptz_jog_speed = 3
        self._ptz_jog_until = 0.0
        self._ptz_pending = False
        self._last_ptz_pose_poll = 0.0

        self._ws_clients: set = set()
        self._ws_loop = None

        # ROS action/service clients
        self._route_client = ActionClient(self, RunRoute, "/patrol/run_route")
        self._return_client = ActionClient(self, ReturnHome, "/patrol/return_home")
        self._control_client = self.create_client(ControlRun, "/patrol/control")
        self._params_client = self.create_client(SetParameters, "/detector_node/set_parameters")
        self._ptz_control_client = self.create_client(ControlPTZ, "/hk_camera/control_ptz")
        self._ptz_pose_client = self.create_client(GetPTZPose, "/hk_camera/get_ptz_pose")

        # Telemetry
        self.create_subscription(
            Odometry, ros_cfg.get("odom_topic", "/odom"),
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, ros_cfg.get("scan_topic", "/scan"),
            self._on_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PatrolStatus, "/patrol/status", self._on_patrol_status, latched)
        self.create_subscription(
            NavPath, "/plan", self._on_global_path, 10)
        self.create_subscription(
            NavPath, "/local_plan", self._on_local_path, 10)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status",
            self._on_nav_status, qos_profile_action_status_default)
        for topic, callback in (
            ("/vision/detector_status", self._on_detector_status),
            ("/hk_camera/status", self._on_camera_status),
            ("/vision/status", self._on_recorder_status),
            ("/gas_detector/status", self._on_gas_status),
            ("/gas_detector/data", self._on_gas_data),
            ("/ptz_patrol/status", self._on_ptz_patrol_status),
        ):
            self.create_subscription(String, topic, callback, 10)
        self.create_subscription(
            OperatorHeartbeat, "/patrol/operator_heartbeat",
            self._on_operator_heartbeat, 10)
        self.create_subscription(Bool, "/emergency_stop", self._on_estop, 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.05, self._on_fast_timer)
        self.create_timer(0.1, self._make_telemetry)
        self.create_timer(0.2, self._publish_operator_heartbeat)
        self.create_timer(0.25, self._poll_ptz_pose)
        self.get_logger().info(
            f"gateway ready; limits ±{self._max_linear} m/s / "
            f"±{self._max_angular} rad/s; lazy teleop publisher")

    # ------------------------------------------------------------- basic state
    @staticmethod
    def _yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def _ensure_cmd_pub(self):
        with self._lock:
            if self._cmd_pub is None:
                self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

    def _publish_zero_and_destroy_cmd_pub(self):
        with self._lock:
            publisher = self._cmd_pub
            self._cmd_pub = None
            self._linear = self._angular = 0.0
            self._last_cmd_time = 0.0
        if publisher is not None:
            try:
                publisher.publish(Twist())
            except Exception:  # noqa: BLE001 - teardown race
                pass
            try:
                self.destroy_publisher(publisher)
            except Exception:  # noqa: BLE001 - teardown race
                pass

    def set_cmd(self, linear, angular):
        with self._lock:
            self._linear = _clamp(linear, self._max_linear)
            self._angular = _clamp(angular, self._max_angular)
            self._last_cmd_time = time.monotonic()
            self._stale_logged = False

    def estop(self):
        with self._lock:
            self._linear = self._angular = 0.0
            self._last_cmd_time = 0.0
            self._estopped = True
            self._estop_time = time.monotonic()
            publisher = self._cmd_pub
        if publisher is not None:
            try:
                publisher.publish(Twist())
            except Exception:  # noqa: BLE001
                pass
        # Publish the safety latch once. Do not keep overriding the local
        # desktop's later explicit unlock with a periodic True message.
        try:
            self._pub_estop.publish(Bool(data=True))
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().warn("web emergency stop")

    # ------------------------------------------------------------- telemetry
    def _on_odom(self, message):
        self._odom = message

    def _on_scan(self, message):
        self._scan = message

    def _on_patrol_status(self, message):
        with self._data_lock:
            self._patrol_status = _patrol_status_dict(message)

    def _on_global_path(self, message):
        self._global_path = message

    def _on_local_path(self, message):
        self._local_path = message

    def _on_nav_status(self, message):
        statuses = {}
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            statuses[goal_id] = GOAL_STATES.get(int(item.status), str(item.status))
        with self._data_lock:
            self._nav_statuses = statuses
            self._nav_status_time = time.monotonic()

    def _status_callback(self, attr, stamp_attr, message):
        value = _json_status(message.data)
        setattr(self, attr, value)
        setattr(self, stamp_attr, time.monotonic())

    def _on_detector_status(self, message):
        self._status_callback("_detector_status", "_detector_time", message)

    def _on_face_status(self, message):
        self._status_callback("_face_status", "_face_time", message)

    def _on_camera_status(self, message):
        raw = str(message.data or "")
        with self._data_lock:
            self._camera_status = {
                "raw": raw,
                "online": raw in ("streaming_active", "stream_active") or "stream" in raw.lower(),
            }
            self._camera_time = time.monotonic()

    def _on_recorder_status(self, message):
        self._status_callback("_recorder_status", "_recorder_time", message)

    def _on_gas_status(self, message):
        self._status_callback("_gas_status", "_gas_status_time", message)

    def _on_gas_data(self, message):
        self._status_callback("_gas_data", "_gas_data_time", message)

    def _on_ptz_patrol_status(self, message):
        self._status_callback("_ptz_patrol_status", "_ptz_patrol_time", message)

    @staticmethod
    def _path_points(path: NavPath | None, frame: str | None):
        if path is None or path.header.frame_id != frame:
            return []
        return [[round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)]
                for pose in path.poses]

    def _pose_and_scan(self):
        pose = None
        scan_points = []
        frame = None
        odom = self._odom
        if odom is None:
            return None, [], None, 0.0, 0.0
        for candidate in (TARGET_FRAME, FALLBACK_FRAME):
            try:
                transform = self._tf_buffer.lookup_transform(
                    candidate, self._base_frame, Time())
                frame = candidate
                translation = transform.transform.translation
                pose = {
                    "x": round(translation.x, 3), "y": round(translation.y, 3),
                    "yaw": round(self._yaw_of(transform.transform.rotation), 3),
                }
                break
            except Exception:
                continue
        scan = self._scan
        if pose is not None and scan is not None:
            try:
                transform = self._tf_buffer.lookup_transform(
                    frame, scan.header.frame_id, Time())
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                yaw = self._yaw_of(transform.transform.rotation)
                cos_t, sin_t = math.cos(yaw), math.sin(yaw)
                angle = scan.angle_min
                for index, distance in enumerate(scan.ranges):
                    if (index % 2 == 0 and math.isfinite(float(distance))
                            and scan.range_min < distance < scan.range_max):
                        local_x = distance * math.cos(angle)
                        local_y = distance * math.sin(angle)
                        scan_points.append([
                            round(tx + local_x * cos_t - local_y * sin_t, 3),
                            round(ty + local_x * sin_t + local_y * cos_t, 3)])
                    angle += scan.angle_increment
            except Exception:
                scan_points = []
        twist = odom.twist.twist
        return (pose, scan_points, frame,
                round(twist.linear.x, 3), round(twist.angular.z, 3))

    def _control_snapshot(self):
        with self._control_lock:
            owner = bool(self._owner_session)
            mode = self._control_mode
            age = (time.monotonic() - self._control_last_heartbeat
                   if self._control_last_heartbeat else None)
            local_enabled = (self._local_operator_enabled and
                             time.monotonic() - self._local_operator_time <= 2.0)
        with self._lock:
            estop_state = self._estopped
            estop_age = (time.monotonic() - self._estop_time
                         if self._estop_time else None)
        return {
            "owner_present": owner,
            "mode": mode,
            "lease_age": round(age, 2) if age is not None else None,
            "local_control": bool(local_enabled),
            "estopped": estop_state if estop_age is not None and estop_age <= self._estop_timeout else None,
            "estop_known": estop_state is not None and estop_age is not None and estop_age <= self._estop_timeout,
            "estop_age": round(estop_age, 2) if estop_age is not None else None,
        }

    def _on_estop(self, message: Bool):
        pressed = bool(message.data)
        with self._lock:
            self._estopped = pressed
            self._estop_time = time.monotonic()
            if pressed:
                self._linear = self._angular = 0.0
                self._last_cmd_time = 0.0
        if pressed:
            with self._control_lock:
                owned = bool(self._owner_session)
            if owned:
                self._schedule_release("急停已触发", cancel_task=True)

    def _task_snapshot(self):
        with self._task_lock:
            return {
                "kind": self._task_kind,
                "request_id": self._task_request_id,
                "running": bool(self._task_kind),
                "pending_return": self._pending_return is not None,
                "cancel_pending": self._task_cancel_pending,
                "final": dict(self._task_final),
            }

    def _make_telemetry(self):
        if not self._ws_clients:
            return
        pose, scan, frame, speed, turn = self._pose_and_scan()
        with self._data_lock:
            patrol = dict(self._patrol_status)
            nav_statuses = dict(self._nav_statuses)
            nav_age = time.monotonic() - self._nav_status_time if self._nav_status_time else None
            global_path = self._global_path
            local_path = self._local_path
            gas_status = dict(self._gas_status)
            gas_data = dict(self._gas_data)
            gas_status_age = (time.monotonic() - self._gas_status_time
                              if self._gas_status_time else None)
            gas_data_age = (time.monotonic() - self._gas_data_time
                            if self._gas_data_time else None)
            detector = dict(self._detector_status)
            detector_age = (time.monotonic() - self._detector_time
                            if self._detector_time else None)
            face = dict(enabled=bool(detector.get("face_requested", detector.get("face"))),
                        ready=bool(detector.get("face")), source="host_yunet",
                        last_error=detector.get("last_error", ""))
            face_age = detector_age
            camera = dict(self._camera_status)
            camera_age = (time.monotonic() - self._camera_time
                          if self._camera_time else None)
            recorder = dict(self._recorder_status)
            ptz_patrol = dict(self._ptz_patrol_status)
            ptz_patrol_age = (time.monotonic() - self._ptz_patrol_time
                              if self._ptz_patrol_time else None)
            ptz_pose = dict(self._ptz_pose)
            ptz_pose_age = (time.monotonic() - self._ptz_pose_time
                            if self._ptz_pose_time else None)
            ptz_jog = {
                "active": bool(self._ptz_jog_active),
                "command": self._ptz_jog_command,
            }
        active_nav = [goal for goal, state in nav_statuses.items()
                      if state in ("accepted", "executing", "canceling")]
        payload = json.dumps({
            "type": "telemetry",
            "stamp": time.time(),
            "frame": frame,
            "pose": pose,
            "scan": scan,
            "speed": speed,
            "turn": turn,
            "control": self._control_snapshot(),
            "task": self._task_snapshot(),
            "patrol": patrol,
            "nav": {
                "statuses": nav_statuses,
                "active_goal_count": len(active_nav),
                "active_goal_ids": active_nav,
                "age": round(nav_age, 2) if nav_age is not None else None,
            },
            "paths": {
                "global": self._path_points(global_path, frame),
                "local": self._path_points(local_path, frame),
            },
            "gas": {
                "status": gas_status,
                "data": gas_data,
                "status_age": round(gas_status_age, 2) if gas_status_age is not None else None,
                "data_age": round(gas_data_age, 2) if gas_data_age is not None else None,
            },
            "vision": {
                "camera": camera,
                "camera_age": round(camera_age, 2) if camera_age is not None else None,
                "face": face,
                "face_age": round(face_age, 2) if face_age is not None else None,
                "detector": detector,
                "detector_age": round(detector_age, 2) if detector_age is not None else None,
                "recorder": recorder,
                "face_service_ready": bool(_safe_ready(self._params_client)),
                "detector_service_ready": bool(_safe_ready(self._params_client)),
            },
            "ptz": {
                "patrol": ptz_patrol,
                "patrol_age": round(ptz_patrol_age, 2) if ptz_patrol_age is not None else None,
                "pose": ptz_pose,
                "pose_age": round(ptz_pose_age, 2) if ptz_pose_age is not None else None,
                "jog": ptz_jog,
                "service_ready": bool(_safe_ready(self._ptz_control_client)),
                "manual_locked": bool(ptz_patrol.get("active") or ptz_patrol.get("enabled")),
            },
        }, ensure_ascii=False)
        loop = self._ws_loop
        if loop is not None:
            loop.call_soon_threadsafe(self._broadcast, payload)

    # --------------------------------------------------------------- websocket
    def _broadcast(self, payload: str):
        """Only called on the websocket event-loop thread."""
        for websocket in list(self._ws_clients):
            asyncio.ensure_future(self._safe_send(websocket, payload))

    async def _safe_send(self, websocket, payload: str):
        try:
            await websocket.send(payload)
        except Exception:
            self._ws_clients.discard(websocket)

    def _send_json_threadsafe(self, websocket, payload: dict):
        loop = self._ws_loop
        text = json.dumps(payload, ensure_ascii=False)
        if loop is None:
            return
        loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._safe_send(websocket, text)))

    def _broadcast_threadsafe(self, payload: dict):
        loop = self._ws_loop
        text = json.dumps(payload, ensure_ascii=False)
        if loop is None:
            return
        loop.call_soon_threadsafe(lambda: self._broadcast(text))

    def _command_result(self, websocket, command: str, ok: bool, code: str = "",
                        message: str = "", request_id: str = "", **extra):
        payload = {
            "type": "command_result", "command": command, "ok": bool(ok),
            "code": code, "message": message, "request_id": request_id,
            "stamp": time.time(),
        }
        payload.update(extra)
        self._send_json_threadsafe(websocket, payload)

    def _broadcast_control_state(self):
        self._broadcast_threadsafe({
            "type": "control_state",
            "control": self._control_snapshot(),
            "task": self._task_snapshot(),
            "stamp": time.time(),
        })

    def _owner_valid(self, token: str) -> bool:
        with self._control_lock:
            return bool(token) and token == self._owner_session

    def _lease_valid(self) -> bool:
        with self._control_lock:
            return (bool(self._owner_session) and
                    time.monotonic() - self._control_last_heartbeat <= self._lease_timeout)

    def _estop_check(self) -> tuple[bool, str, str]:
        with self._lock:
            state = self._estopped
            age = (time.monotonic() - self._estop_time
                   if self._estop_time else float("inf"))
        if state is None or age > self._estop_timeout:
            return False, "ESTOP_UNKNOWN", "急停状态未知或已过期，请先在本地确认安全"
        if state:
            return False, "ESTOPPED", "急停保持中，请先在本地解除急停"
        return True, "", ""

    def _local_task_active(self) -> bool:
        """A non-web patrol action is currently running locally."""
        with self._task_lock:
            web_task = bool(self._task_kind)
        with self._data_lock:
            patrol = dict(self._patrol_status)
        return bool(patrol.get("active")) and not web_task

    def _active_nav_goal_count(self) -> int:
        with self._data_lock:
            statuses = dict(self._nav_statuses)
        return sum(1 for state in statuses.values()
                   if state in ("accepted", "executing", "canceling"))

    def _claim_control(self, websocket):
        now = time.monotonic()
        estop_ok, estop_code, estop_message = self._estop_check()
        if not estop_ok:
            return None, estop_code, estop_message
        if self._local_task_active():
            return None, "LOCAL_TASK_ACTIVE", "本地已有路线任务在运行"
        with self._control_lock:
            local_enabled = (self._local_operator_enabled and
                             now - self._local_operator_time <= 2.0)
            if local_enabled:
                return None, "LOCAL_HAS_CONTROL", "本地桌面已启用操作，网页不能抢占"
            if self._owner_session:
                if self._owner_ws is websocket:
                    return self._owner_session, "", "控制权已存在"
                return None, "CONTROL_BUSY", "已有其他网页控制者"
            token = "web_" + uuid.uuid4().hex
            self._owner_session = token
            self._owner_ws = websocket
            self._control_last_heartbeat = now
            self._control_mode = "manual"
            self._release_scheduled = False
            self._task_final = {}
        self._ensure_cmd_pub()
        self.get_logger().info("web control claimed")
        self._broadcast_control_state()
        return token, "", "网页已取得控制权"

    def _teardown_control_locked(self):
        token = self._owner_session
        mode = self._control_mode
        self._owner_session = ""
        self._owner_ws = None
        self._control_last_heartbeat = 0.0
        self._control_mode = "idle"
        self._linear = self._angular = 0.0
        self._last_cmd_time = 0.0
        self._release_scheduled = False
        return token, mode

    def _perform_release(self, reason: str, cancel_task: bool = True,
                         notify_websocket=None):
        with self._control_lock:
            token, mode = self._teardown_control_locked()
            had_owner = bool(token)
        with self._task_lock:
            self._pending_return = None
        self._publish_zero_and_destroy_cmd_pub()
        self._ptz_stop()
        if had_owner and cancel_task:
            self._cancel_task(reason, owner_token=token)
        self._publish_disabled_operator_heartbeat(token)
        if had_owner:
            self.get_logger().warn(f"web control released: {reason}")
        if notify_websocket is not None:
            self._command_result(notify_websocket, "control_release", True,
                                 "RELEASED", reason)
        self._broadcast_control_state()

    def _schedule_release(self, reason: str, cancel_task: bool = True):
        """Release from a ROS/timer thread onto the websocket event loop."""
        loop = self._ws_loop
        if loop is None:
            self._perform_release(reason, cancel_task)
            return
        with self._control_lock:
            if not self._owner_session or self._release_scheduled:
                return
            self._release_scheduled = True
        loop.call_soon_threadsafe(
            lambda: self._perform_release(reason, cancel_task))

    # -------------------------------------------------------- local heartbeat
    def _on_operator_heartbeat(self, message: OperatorHeartbeat):
        if str(message.client_id or "") != "desktop_ui":
            return
        enabled = bool(message.enabled)
        immediate_stop = None
        with self._control_lock:
            self._local_operator_enabled = enabled
            self._local_operator_time = time.monotonic()
            remote_owned = bool(self._owner_session)
            if enabled and remote_owned:
                # Revoke new manual commands immediately; do not wait for the
                # websocket event loop to process the release.
                self._control_mode = "stopping"
                self._linear = self._angular = 0.0
                self._last_cmd_time = 0.0
                immediate_stop = self._cmd_pub
        if immediate_stop is not None:
            try:
                immediate_stop.publish(Twist())
            except Exception:  # noqa: BLE001 - teardown race
                pass
        if enabled and remote_owned:
            self._schedule_release("本地桌面已接管控制权", cancel_task=True)

    def _publish_operator_heartbeat(self):
        with self._control_lock:
            token = self._owner_session
            mode = self._control_mode
        if not token:
            return
        message = OperatorHeartbeat()
        message.session_id = token
        message.client_id = "web_gateway"
        message.enabled = True
        message.stamp = self.get_clock().now().to_msg()
        try:
            self._operator_pub.publish(message)
        except Exception:  # noqa: BLE001 - shutdown teardown race
            pass
        del mode

    def _publish_disabled_operator_heartbeat(self, token: str):
        if not token:
            return
        message = OperatorHeartbeat()
        message.session_id = token
        message.client_id = "web_gateway"
        message.enabled = False
        message.stamp = self.get_clock().now().to_msg()
        try:
            self._operator_pub.publish(message)
        except Exception:  # noqa: BLE001 - shutdown teardown race
            pass

    # --------------------------------------------------------------- task API
    @staticmethod
    def _task_fields(data: dict, route: bool = False) -> dict:
        """Validate before reserving a task or canceling an existing route."""
        fields = {}
        for name in ("request_id", "map_id", *(('route_id',) if route else ())):
            value = data.get(name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} 不能为空")
            fields[name] = value.strip()
        for name in ("map_revision", *(('route_revision', 'loop_count') if route else ())):
            value = data.get(name, 1 if name == "loop_count" else None)
            minimum, maximum = (0, 100000) if name == "loop_count" else (1, 2**64 - 1)
            if type(value) is not int or not minimum <= value <= maximum:
                raise ValueError(f"{name} 必须是 {minimum}～{maximum} 的整数")
            fields[name] = value
        return fields

    def _begin_task(self, kind: str, request_id: str):
        with self._task_lock:
            if self._task_kind:
                return False, "BUSY", "已有网页发起的任务正在执行"
            self._task_kind = kind
            self._task_request_id = request_id
            self._task_goal_handle = None
            self._task_cancel_pending = False
            self._task_started_at = time.monotonic()
            self._task_final = {}
        with self._control_lock:
            if kind == "route":
                self._control_mode = "route"
            elif kind == "return":
                self._control_mode = "return"
        # Automatic navigation is owned by Nav2. The teleop publisher must not
        # remain as a competing manual input while the task runs.
        self._publish_zero_and_destroy_cmd_pub()
        self._broadcast_control_state()
        return True, "", "任务已提交"

    def _finish_task(self, kind: str, request_id: str, success: bool,
                     code: str = "", message: str = ""):
        with self._task_lock:
            if self._task_kind != kind or self._task_request_id != request_id:
                return
            self._task_kind = ""
            self._task_goal_handle = None
            self._task_cancel_pending = False
            self._task_final = {
                "success": bool(success), "code": code, "message": message,
                "finished_at": time.time(),
            }
        with self._control_lock:
            if self._owner_session:
                self._control_mode = "manual"
            else:
                self._control_mode = "idle"
            self._last_cmd_time = 0.0
        if self._owner_session:
            self._ensure_cmd_pub()
        pending = None
        if kind == "route":
            with self._task_lock:
                pending = self._pending_return
                self._pending_return = None
        if pending is not None:
            threading.Thread(
                target=self._launch_return_payload,
                args=(pending["websocket"], pending["data"]),
                daemon=True, name="web-return-after-route").start()
        self._broadcast_threadsafe({
            "type": "task_finished", "kind": kind,
            "request_id": request_id, "success": bool(success),
            "code": code, "message": message, "stamp": time.time(),
            "continuation": "return" if pending is not None else "",
            "task": self._task_snapshot(),
        })
        self._broadcast_control_state()

    def _cancel_task(self, reason: str, owner_token: str = ""):
        with self._task_lock:
            kind = self._task_kind
            handle = self._task_goal_handle
            if kind:
                self._task_cancel_pending = True
        if not kind:
            return
        try:
            self.get_logger().warn(f"cancel web task {kind}: {reason}")
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001 - task may already be terminal
                    pass
            if kind == "route":
                with self._data_lock:
                    run_id = str(self._patrol_status.get("run_id", ""))
                if run_id:
                    self._call_control_cancel(run_id, owner_token)
        except Exception as error:  # noqa: BLE001 - shutdown teardown race
            self.get_logger().warn(f"task cancellation failed: {error}")

    def _call_control_cancel(self, run_id: str, owner_token: str = ""):
        if not self._control_client.service_is_ready():
            return
        request = ControlRun.Request()
        request.run_id = run_id
        request.command = ControlRun.Request.CANCEL
        request.owner_session_id = str(owner_token or self._owner_session)
        request.command_id = "web_cancel_" + uuid.uuid4().hex
        future = self._control_client.call_async(request)
        future.add_done_callback(self._on_cancel_service_done)

    def _on_cancel_service_done(self, future):
        try:
            result = future.result()
            if not result.accepted:
                self.get_logger().warn(f"route cancel service rejected: {result.message}")
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f"route cancel service failed: {error}")

    def _start_route(self, websocket, data: dict):
        request_id = str(data.get("request_id", "")).strip()
        try:
            fields = self._task_fields(data, route=True)
            goal = RunRoute.Goal()
            for name, value in fields.items():
                setattr(goal, name, value)
        except (ValueError, TypeError, AssertionError, OverflowError) as error:
            self._command_result(websocket, "route_start", False,
                                 "INVALID_DATA", str(error), request_id)
            return
        if not self._lease_valid():
            self._command_result(websocket, "route_start", False,
                                 "LEASE_LOST", "网页控制心跳已过期", request_id)
            return
        estop_ok, estop_code, estop_message = self._estop_check()
        if not estop_ok:
            self._command_result(websocket, "route_start", False,
                                 estop_code, estop_message, request_id)
            return
        if self._local_task_active():
            self._command_result(websocket, "route_start", False,
                                 "LOCAL_TASK_ACTIVE", "本地已有任务在运行",
                                 request_id)
            return
        if self._active_nav_goal_count():
            self._command_result(websocket, "route_start", False,
                                 "NAV_GOAL_ACTIVE", "Nav2 已有活动目标，禁止启动路线",
                                 request_id)
            return
        if not self._route_client.wait_for_server(timeout_sec=0.5):
            self._command_result(websocket, "route_start", False,
                                 "NAV_NOT_READY", "路线执行器未就绪", request_id)
            return
        with self._control_lock:
            goal.owner_session_id = self._owner_session
        ok, code, message = self._begin_task("route", request_id)
        if not ok:
            self._command_result(websocket, "route_start", False, code, message, request_id)
            return
        try:
            send_future = self._route_client.send_goal_async(
                goal, feedback_callback=lambda feedback: self._on_task_feedback(
                    "route", request_id, feedback))
            send_future.add_done_callback(
                lambda future: self._on_task_goal_response(
                    "route", request_id, websocket, future))
        except Exception as error:  # ROS submission failed before a handle exists.
            self._finish_task("route", request_id, False, "ACTION_FAILED", str(error))
            self._command_result(websocket, "route_start", False,
                                 "ACTION_FAILED", str(error), request_id)
            return
        self._command_result(websocket, "route_start", True, "PENDING",
                             "路线任务提交中", request_id)

    def _start_return(self, websocket, data: dict):
        request_id = str(data.get("request_id", "")).strip()
        try:
            self._task_fields(data)
        except (ValueError, TypeError) as error:
            self._command_result(websocket, "return_home", False,
                                 "INVALID_DATA", str(error), request_id)
            return
        if not self._lease_valid():
            self._command_result(websocket, "return_home", False,
                                 "LEASE_LOST", "网页控制心跳已过期", request_id)
            return
        estop_ok, estop_code, estop_message = self._estop_check()
        if not estop_ok:
            self._command_result(websocket, "return_home", False,
                                 estop_code, estop_message, request_id)
            return
        with self._task_lock:
            active_kind = self._task_kind
            if active_kind == "return":
                self._command_result(
                    websocket, "return_home", False, "BUSY",
                    "返航任务已在执行", request_id)
                return
            if active_kind == "route":
                if self._pending_return is not None:
                    self._command_result(
                        websocket, "return_home", False, "BUSY",
                        "已在等待当前路线取消后返航", request_id)
                    return
                self._pending_return = {
                    "websocket": websocket,
                    "data": dict(data),
                }
                self._task_cancel_pending = True
        if active_kind == "route":
            self._command_result(
                websocket, "return_home", True, "PENDING",
                "正在取消当前路线，随后开始返航", request_id)
            with self._control_lock:
                owner_token = self._owner_session
            self._cancel_task("网页请求一键返航", owner_token=owner_token)
            return
        self._launch_return_payload(websocket, data)

    def _launch_return_payload(self, websocket, data: dict):
        """Start ReturnHome after the previous web route has terminated."""
        request_id = str(data.get("request_id", "")).strip()
        # A disconnect/local takeover or a new owner invalidates the queued
        # return. Never dispatch it against a session that no longer exists.
        if not self._lease_valid() or self._owner_ws is not websocket:
            self._command_result(websocket, "return_home", False,
                                 "LEASE_LOST", "网页控制心跳已过期", request_id)
            return
        try:
            fields = self._task_fields(data)
            goal = ReturnHome.Goal()
            for name, value in fields.items():
                setattr(goal, name, value)
        except (ValueError, TypeError, AssertionError, OverflowError) as error:
            self._command_result(websocket, "return_home", False,
                                 "INVALID_DATA", str(error), request_id)
            return
        if not self._return_client.wait_for_server(timeout_sec=0.5):
            self._command_result(websocket, "return_home", False,
                                 "NAV_NOT_READY", "返航执行器未就绪", request_id)
            return
        with self._control_lock:
            goal.owner_session_id = self._owner_session
        ok, code, message = self._begin_task("return", request_id)
        if not ok:
            self._command_result(websocket, "return_home", False, code, message, request_id)
            return
        try:
            send_future = self._return_client.send_goal_async(
                goal, feedback_callback=lambda feedback: self._on_task_feedback(
                    "return", request_id, feedback))
            send_future.add_done_callback(
                lambda future: self._on_task_goal_response(
                    "return", request_id, websocket, future))
        except Exception as error:
            self._finish_task("return", request_id, False, "ACTION_FAILED", str(error))
            self._command_result(websocket, "return_home", False,
                                 "ACTION_FAILED", str(error), request_id)

    def _on_task_goal_response(self, kind: str, request_id: str,
                               websocket, future):
        try:
            handle = future.result()
        except Exception as error:  # noqa: BLE001 - action boundary
            self._finish_task(kind, request_id, False, "ACTION_FAILED", str(error))
            self._command_result(websocket, kind + (":start" if kind else ""), False,
                                 "ACTION_FAILED", str(error), request_id)
            return
        if handle is None or not handle.accepted:
            self._finish_task(kind, request_id, False, "REJECTED",
                              "本地任务执行器拒绝任务")
            self._command_result(websocket,
                                 "route_start" if kind == "route" else "return_home",
                                 False, "REJECTED", "本地任务执行器拒绝任务", request_id)
            return
        with self._task_lock:
            if self._task_kind != kind or self._task_request_id != request_id:
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                return
            self._task_goal_handle = handle
            cancel_pending = self._task_cancel_pending
        if cancel_pending:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self._broadcast_threadsafe({
            "type": "task_accepted", "kind": kind,
            "request_id": request_id, "stamp": time.time(),
        })
        self._command_result(
            websocket, "route_start" if kind == "route" else "return_home",
            True, "ACCEPTED", "本地任务执行器已接受任务", request_id)
        handle.get_result_async().add_done_callback(
            lambda result_future: self._on_task_result(
                kind, request_id, result_future))

    def _on_task_feedback(self, kind: str, request_id: str, feedback_message):
        try:
            status_message = feedback_message.feedback.status
            status = _patrol_status_dict(status_message)
        except Exception:  # noqa: BLE001 - feedback interface boundary
            return
        with self._task_lock:
            if self._task_kind != kind or self._task_request_id != request_id:
                return
        self._broadcast_threadsafe({
            "type": "task_status", "kind": kind, "request_id": request_id,
            "status": status, "stamp": time.time(),
        })

    def _on_task_result(self, kind: str, request_id: str, future):
        try:
            wrapped = future.result()
            success = int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
            result = wrapped.result
            code = getattr(result, "error_code", "")
            message = getattr(result, "message", "")
            success = success and bool(getattr(result, "success", success))
        except Exception as error:  # noqa: BLE001 - action boundary
            success, code, message = False, "ACTION_FAILED", str(error)
        self._finish_task(kind, request_id, success, code, message)

    def _route_control(self, websocket, data: dict, command_name: str):
        if not self._lease_valid():
            self._command_result(websocket, command_name, False,
                                 "LEASE_LOST", "网页控制心跳已过期", data.get("request_id", ""))
            return
        with self._task_lock:
            if self._task_kind != "route":
                self._command_result(websocket, command_name, False,
                                     "NO_ACTIVE_TASK", "当前没有网页路线任务",
                                     data.get("request_id", ""))
                return
        with self._data_lock:
            run_id = str(self._patrol_status.get("run_id", ""))
        if not run_id:
            if command_name == "route_cancel":
                with self._task_lock:
                    handle = self._task_goal_handle
                    self._task_cancel_pending = True
                if handle is not None:
                    try:
                        handle.cancel_goal_async()
                    except Exception as error:  # noqa: BLE001
                        self._command_result(
                            websocket, command_name, False, "CANCEL_FAILED",
                            str(error), data.get("request_id", ""))
                        return
                self._command_result(websocket, command_name, True, "",
                                     "任务运行号尚未生成，已取消动作请求",
                                     data.get("request_id", ""))
                return
            self._command_result(websocket, command_name, False,
                                 "NOT_READY", "任务尚未进入可控制状态",
                                 data.get("request_id", ""))
            return
        command = {
            "route_pause": ControlRun.Request.PAUSE,
            "route_resume": ControlRun.Request.RESUME,
            "route_cancel": ControlRun.Request.CANCEL,
        }[command_name]
        if not self._control_client.service_is_ready():
            self._command_result(websocket, command_name, False,
                                 "SERVICE_UNAVAILABLE", "路线控制服务未就绪",
                                 data.get("request_id", ""))
            return
        request = ControlRun.Request()
        request.run_id = run_id
        request.command = command
        with self._control_lock:
            request.owner_session_id = self._owner_session
        request.command_id = "web_" + uuid.uuid4().hex
        future = self._control_client.call_async(request)
        future.add_done_callback(
            lambda done: self._on_route_control_done(
                websocket, command_name, data.get("request_id", ""), done))
        if command_name == "route_cancel":
            with self._task_lock:
                self._task_cancel_pending = True
                handle = self._task_goal_handle
            # The service owns engine cancellation; the action cancel is an
            # independent safety path when the run_id/status update races.
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001 - already terminal
                    pass

    def _on_route_control_done(self, websocket, command_name: str,
                               request_id: str, future):
        try:
            result = future.result()
            self._command_result(
                websocket, command_name, bool(result.accepted),
                "" if result.accepted else "REJECTED", result.message, request_id)
        except Exception as error:  # noqa: BLE001
            self._command_result(websocket, command_name, False,
                                 "SERVICE_FAILED", str(error), request_id)

    def _cancel_return(self, websocket, data: dict):
        if not self._lease_valid():
            self._command_result(websocket, "return_cancel", False,
                                 "LEASE_LOST", "网页控制心跳已过期",
                                 data.get("request_id", ""))
            return
        with self._task_lock:
            pending = self._pending_return
            if pending is not None:
                self._pending_return = None
                request_id = str(pending["data"].get("request_id", ""))
                self._command_result(websocket, "return_cancel", True, "",
                                     "已取消等待中的返航请求", request_id)
                self._broadcast_control_state()
                return
            handle = self._task_goal_handle
            kind = self._task_kind
            self._task_cancel_pending = bool(kind == "return")
            request_id = self._task_request_id
        if kind != "return":
            self._command_result(websocket, "return_cancel", False,
                                 "NO_ACTIVE_TASK", "当前没有网页返航任务", request_id)
            return
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as error:  # noqa: BLE001
                self._command_result(websocket, "return_cancel", False,
                                     "CANCEL_FAILED", str(error), request_id)
                return
        self._command_result(websocket, "return_cancel", True, "", "取消返航请求已发送",
                             request_id)

    # ------------------------------------------------------------- face/vision
    def _call_set_face(self, websocket, data: dict):
        self._call_set_detector_param(websocket, data, "vision_face_capture", "enable_face")

    def _call_set_detector_param(self, websocket, data: dict,
                                 command_name: str, parameter_name: str):
        enabled = bool(data.get("enabled"))
        if not self._params_client.service_is_ready():
            self._command_result(websocket, command_name, False,
                                 "VISION_NOT_RUNNING", "识别节点参数服务未启动",
                                 data.get("request_id", ""))
            return
        request = SetParameters.Request()
        value = ParameterValue()
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = enabled
        request.parameters = [Parameter(name=parameter_name, value=value)]
        future = self._params_client.call_async(request)
        future.add_done_callback(
            lambda done: self._vision_result(
                websocket, command_name, data.get("request_id", ""), done))

    def _vision_result(self, websocket, command_name: str, request_id: str, future):
        try:
            result = future.result()
            if hasattr(result, "results"):
                results = list(result.results)
                ok = bool(results and all(item.successful for item in results))
                message = "；".join(item.reason for item in results if item.reason)
            else:
                ok = bool(result.success)
                message = result.message
            self._command_result(websocket, command_name, ok,
                                 "" if ok else "REJECTED", message, request_id)
        except Exception as error:  # noqa: BLE001
            self._command_result(websocket, command_name, False,
                                 "VISION_FAILED", str(error), request_id)

    # ------------------------------------------------------------------- PTZ
    def _ptz_blocked(self) -> bool:
        with self._data_lock:
            status = dict(self._ptz_patrol_status)
        return bool(status.get("active") or status.get("enabled"))

    def _call_ptz(self, command: int = 0, stop: bool = False,
                  speed: int = 3):
        try:
            if not self._ptz_control_client.service_is_ready():
                return False, "云台控制服务未就绪"
            request = ControlPTZ.Request()
            request.command = int(command)
            request.speed = max(1, min(7, int(speed)))
            request.stop = bool(stop)
            self._ptz_control_client.call_async(request)
            return True, ""
        except Exception as error:  # noqa: BLE001 - shutdown/teardown race
            return False, str(error)

    def _ptz_jog(self, websocket, data: dict):
        action = str(data.get("action", ""))
        command = PTZ_COMMANDS.get(action)
        if command is None:
            self._command_result(websocket, "ptz_jog", False,
                                 "INVALID_DATA", "未知云台动作",
                                 data.get("request_id", ""))
            return
        if self._ptz_blocked():
            self._command_result(websocket, "ptz_jog", False,
                                 "PTZ_AUTO_ACTIVE", "云台自动巡航运行中，禁止手动控制",
                                 data.get("request_id", ""))
            return
        speed = int(data.get("speed", 3) or 3)
        ok, message = self._call_ptz(command, False, speed)
        if not ok:
            self._command_result(websocket, "ptz_jog", False,
                                 "PTZ_UNAVAILABLE", message,
                                 data.get("request_id", ""))
            return
        with self._data_lock:
            self._ptz_jog_active = True
            self._ptz_jog_command = command
            self._ptz_jog_speed = max(1, min(7, speed))
            # Browser hold commands renew this lease every 200ms. A lost release
            # or stuck view cannot make the gateway renew the camera forever.
            self._ptz_jog_until = time.monotonic() + 0.75
        self._command_result(websocket, "ptz_jog", True, "", "云台持续运动",
                             data.get("request_id", ""))
        self._poll_ptz_pose(force=True)

    def _ptz_stop(self, websocket=None, request_id: str = ""):
        with self._data_lock:
            was_active = self._ptz_jog_active
            self._ptz_jog_active = False
            self._ptz_jog_command = 0
            self._ptz_jog_until = 0.0
        ok, message = self._call_ptz(stop=True)
        if websocket is not None:
            self._command_result(websocket, "ptz_stop", ok,
                                 "" if ok else "PTZ_UNAVAILABLE",
                                 message or "云台已停止", request_id)
        return was_active

    def _ptz_read(self, websocket, data: dict):
        self._poll_ptz_pose(force=True)
        self._command_result(websocket, "ptz_read", True, "", "云台状态读取请求已发送",
                             data.get("request_id", ""),
                             ptz=self._ptz_snapshot())

    def _poll_ptz_pose(self, force: bool = False):
        with self._data_lock:
            jog = self._ptz_jog_active
            command = self._ptz_jog_command
            speed = self._ptz_jog_speed
            deadline = self._ptz_jog_until
            pending = self._ptz_pending
            last_poll = self._last_ptz_pose_poll
        now = time.monotonic()
        if jog and now >= deadline:
            self._ptz_stop()
        elif jog and self._ptz_control_client.service_is_ready():
            # Camera PTZ commands are a 500 ms lease. Renew faster than the
            # lease while the browser keeps the button held.
            self._call_ptz(command, False, speed)
        if pending or not self._ptz_pose_client.service_is_ready():
            return
        if not force and now - last_poll < 1.0:
            return
        with self._data_lock:
            self._ptz_pending = True
            self._last_ptz_pose_poll = now
        future = self._ptz_pose_client.call_async(GetPTZPose.Request())
        future.add_done_callback(self._on_ptz_pose_done)

    def _on_ptz_pose_done(self, future):
        with self._data_lock:
            self._ptz_pending = False
        try:
            result = future.result()
            if result.success:
                with self._data_lock:
                    self._ptz_pose = {
                        "pan": result.pan, "tilt": result.tilt, "zoom": result.zoom,
                        "limits_valid": bool(result.limits_valid),
                        "pan_min": result.pan_min, "pan_max": result.pan_max,
                        "tilt_min": result.tilt_min, "tilt_max": result.tilt_max,
                        "zoom_min": result.zoom_min, "zoom_max": result.zoom_max,
                    }
                    self._ptz_pose_time = time.monotonic()
        except Exception:  # noqa: BLE001 - telemetry is best effort
            pass

    def _ptz_snapshot(self):
        with self._data_lock:
            return {
                "patrol": dict(self._ptz_patrol_status),
                "pose": dict(self._ptz_pose),
                "jog": {"active": bool(self._ptz_jog_active),
                        "command": self._ptz_jog_command},
            }

    # ---------------------------------------------------------- command loop
    async def handle_command(self, websocket, data: dict):
        msg_type = str(data.get("type", ""))
        request_id = str(data.get("request_id", "") or "")
        token = str(data.get("session_id", "") or "")

        # E-stop is intentionally always accepted, even from a viewer.
        if msg_type == "estop":
            self.estop()
            self._cancel_task("网页急停")
            self._ptz_stop()
            self._perform_release("网页急停", cancel_task=True,
                                  notify_websocket=websocket)
            return
        if msg_type == "control_claim":
            if self._owner_ws is websocket and self._owner_session:
                self._command_result(websocket, msg_type, True, "ALREADY_OWNER",
                                     "当前网页已拥有控制权", request_id,
                                     session_id=self._owner_session)
                return
            claimed, code, message = self._claim_control(websocket)
            if claimed is None:
                self._command_result(websocket, msg_type, False, code, message,
                                     request_id)
            else:
                self._command_result(websocket, msg_type, True, "", message,
                                     request_id, session_id=claimed)
            return
        if msg_type == "control_heartbeat":
            if not self._owner_valid(token):
                self._command_result(websocket, msg_type, False, "NOT_OWNER",
                                     "控制权已释放或 token 无效", request_id)
                return
            with self._control_lock:
                if self._owner_ws is websocket:
                    self._control_last_heartbeat = time.monotonic()
            return
        if msg_type == "control_release":
            if not self._owner_valid(token) or self._owner_ws is not websocket:
                self._command_result(websocket, msg_type, False, "NOT_OWNER",
                                     "控制权已释放或 token 无效", request_id)
                return
            self._perform_release("网页主动释放控制权", cancel_task=True,
                                  notify_websocket=websocket)
            return
        # Every remaining command requires the current owner token.
        if not self._owner_valid(token):
            self._command_result(websocket, msg_type, False, "NOT_OWNER",
                                 "请先取得网页控制权", request_id)
            return
        with self._control_lock:
            revoked = self._release_scheduled
            if self._owner_ws is websocket:
                self._control_last_heartbeat = time.monotonic()
        if revoked:
            self._command_result(websocket, msg_type, False, "CONTROL_REVOKED",
                                 "网页控制权正在释放", request_id)
            return

        if msg_type == "cmd_vel":
            with self._control_lock:
                mode = self._control_mode
            if mode != "manual":
                self._command_result(websocket, msg_type, False,
                                     "TASK_ACTIVE", "自动任务期间禁止手动遥控", request_id)
                return
            if self._local_task_active() or self._active_nav_goal_count():
                self._command_result(websocket, msg_type, False,
                                     "LOCAL_MOTION_ACTIVE", "本地任务或 Nav2 目标运行中，禁止网页手动遥控",
                                     request_id)
                return
            estop_ok, estop_code, estop_message = self._estop_check()
            if not estop_ok:
                self._command_result(websocket, msg_type, False, estop_code,
                                     estop_message, request_id)
                return
            self._ensure_cmd_pub()
            self.set_cmd(data.get("linear", 0.0), data.get("angular", 0.0))
            return
        if msg_type == "route_start":
            self._start_route(websocket, data)
            return
        if msg_type in ("route_pause", "route_resume", "route_cancel"):
            self._route_control(websocket, data, msg_type)
            return
        if msg_type == "return_home":
            self._start_return(websocket, data)
            return
        if msg_type == "return_cancel":
            self._cancel_return(websocket, data)
            return
        if msg_type in ("vision_face_capture", "vision_person", "vision_plate"):
            if msg_type == "vision_face_capture":
                self._call_set_face(websocket, data)
            elif msg_type == "vision_person":
                self._call_set_detector_param(
                    websocket, data, msg_type, "enable_person")
            else:
                self._call_set_detector_param(
                    websocket, data, msg_type, "enable_plate")
            return
        if msg_type == "ptz_jog":
            self._ptz_jog(websocket, data)
            return
        if msg_type == "ptz_stop":
            self._ptz_stop(websocket, request_id)
            return
        if msg_type == "ptz_read":
            self._ptz_read(websocket, data)
            return
        self._command_result(websocket, msg_type, False, "UNKNOWN_COMMAND",
                             "未知或已废弃的网页命令", request_id)

    # -------------------------------------------------------------- ROS timer
    def _on_fast_timer(self):
        with self._lock:
            stale = (time.monotonic() - self._last_cmd_time) > self._cmd_timeout
            linear, angular = (0.0, 0.0) if stale else (self._linear, self._angular)
            just_stale = stale and not self._stale_logged and self._cmd_pub is not None
            if stale:
                self._stale_logged = True
            publisher = self._cmd_pub
        # /emergency_stop is event-driven: the gateway must never periodically
        # publish a synthetic state over the local operator's safety latch.
        # The lease applies in every mode, including while the manual
        # publisher is absent during an automatic task. Check it before the
        # early return for no publisher.
        now = time.monotonic()
        with self._control_lock:
            expired = (bool(self._owner_session) and
                       now - self._control_last_heartbeat > self._lease_timeout)
        if expired:
            self._schedule_release("网页控制心跳超时", cancel_task=True)
        if publisher is None:
            return
        if just_stale:
            self.get_logger().warn("web manual command timeout; publishing zero")
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        try:
            publisher.publish(message)
        except Exception:  # noqa: BLE001 - publisher may be destroyed on release
            pass

    def _remove_client(self, websocket):
        self._ws_clients.discard(websocket)
        with self._control_lock:
            is_owner = self._owner_ws is websocket
        if is_owner:
            try:
                self._perform_release("浏览器连接断开", cancel_task=True)
            except Exception as error:  # noqa: BLE001 - process teardown
                self.get_logger().warn(f"control cleanup during disconnect failed: {error}")


async def _handle_client(node: GatewayNode, websocket):
    peer = websocket.remote_address
    node._ws_clients.add(websocket)
    node.get_logger().info(f"web client connected: {peer}")
    try:
        await websocket.send(json.dumps({
            "type": "hello", "protocol": 2,
            "message": "WebSocket connected; claim control to send commands",
            "stamp": time.time(),
        }, ensure_ascii=False))
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if not isinstance(data, dict):
                continue
            try:
                await node.handle_command(websocket, data)
            except Exception as error:  # noqa: BLE001 - keep one client alive
                node.get_logger().error(f"web command failed: {error}")
                node._command_result(
                    websocket, str(data.get("type", "")), False,
                    "INTERNAL_ERROR", str(error),
                    str(data.get("request_id", "") or ""))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        try:
            node.get_logger().info(f"web client disconnected: {peer}")
        except Exception:  # noqa: BLE001
            pass
        try:
            node._remove_client(websocket)
        except Exception as error:  # noqa: BLE001 - shutdown teardown
            try:
                node.get_logger().warn(f"web client cleanup failed: {error}")
            except Exception:
                pass


def _run_ws_server(node: GatewayNode, port: int):
    async def _serve():
        node._ws_loop = asyncio.get_running_loop()
        async with websockets.serve(
                lambda websocket, *args: _handle_client(node, websocket),
                "0.0.0.0", port):
            node.get_logger().info(f"WebSocket listening on :{port}")
            await asyncio.Future()
    asyncio.run(_serve())


def main():
    config_path = os.environ.get("ROBOT_CONFIG")
    if not config_path:
        for candidate in CONFIG_CANDIDATES:
            if candidate.exists():
                config_path = str(candidate)
                break
    if not config_path:
        raise FileNotFoundError(
            "config.yaml not found; set ROBOT_CONFIG. candidates: " +
            ", ".join(str(item) for item in CONFIG_CANDIDATES))
    with open(config_path, "r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    rclpy.init()
    node = GatewayNode(cfg)
    threading.Thread(target=_run_ws_server, args=(node, int(cfg["ports"]["ws"])),
                     daemon=True, name="robot-gateway-ws").start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:  # noqa: BLE001 - SIGINT/SIGTERM context teardown
        if rclpy.ok():
            raise error
    finally:
        try:
            node._ptz_stop()
            node._perform_release("网关退出", cancel_task=True)
        except Exception:  # noqa: BLE001 - signal-driven context shutdown
            pass
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
