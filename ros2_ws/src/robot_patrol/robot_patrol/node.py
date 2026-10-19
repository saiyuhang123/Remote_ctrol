"""ROS2 local patrol node: storage services, Nav2 execution and Qt API."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import hashlib
import json
import os
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from patrol_interfaces.action import ReturnHome, RunRoute
from patrol_interfaces.msg import (
    ExecutorHeartbeat, MapBinding, PatrolPoint, PatrolRoute, PatrolStatus, RouteStep, RunRecord,
)
from patrol_interfaces.srv import (
    ControlRun,
    DeletePoint,
    DeleteRoute,
    GetHomePoint,
    GetRuns,
    ListMaps,
    ListPoints,
    ListRoutes,
    RegisterMap,
    SaveCurrentPoint,
    SetActiveMap,
    SetHomePoint,
    UpsertPoint,
    UpsertRoute,
)

from .engine import (
    ControlFlags, PatrolEngine, STATE_CANCELED, STATE_CANCELING, STATE_FAILED,
    STATE_IDLE, STATE_INTERRUPTED, STATE_NAVIGATING, STATE_SETTLING,
    STATE_SUCCEEDED, STATE_VALIDATING,
)
from .map_export import default_web_map_dir, export_web_map
from .map_metadata import expected_occupancy_sha256, read_map_binding
from .models import PatrolError, Point, RouteSnapshot, new_id, now_ns, require_finite
from .nav2_client import Nav2Driver, quaternion_to_yaw
from .store import PatrolStore
from .recorded_routes.component import RoadComponent


def stamp_from_ns(value: int) -> TimeMsg:
    message = TimeMsg()
    if value and value > 0:
        message.sec = int(value // 1_000_000_000)
        message.nanosec = int(value % 1_000_000_000)
    return message


def _copy_time(value):
    return SimpleNamespace(sec=value.sec, nanosec=value.nanosec)


class PatrolNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("robot_patrol", parameter_overrides=parameter_overrides)
        default_database = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share")) / \
            "robot_patrol" / "patrol.sqlite3"
        self.declare_parameter("database_path", str(default_database))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("point_timeout_sec", 300.0)
        self.declare_parameter("cancel_confirm_timeout_sec", 2.0)
        self.declare_parameter("settle_timeout_sec", 10.0)
        self.declare_parameter("settle_linear_threshold", 0.03)
        self.declare_parameter("settle_angular_threshold", 0.05)
        self.declare_parameter("record_point_linear_threshold", 0.08)
        self.declare_parameter("record_point_angular_threshold", 0.10)
        self.declare_parameter("settle_duration_sec", 0.8)
        self.declare_parameter("feedback_max_age_sec", 1.5)
        self.declare_parameter("tf_max_age_sec", 1.5)
        self.declare_parameter("map_feedback_timeout_sec", 2.0)
        self.declare_parameter("require_guard", True)
        self.declare_parameter("guard_timeout_sec", 2.0)
        self.declare_parameter("dwell_resume_position_tolerance_m", 0.5)
        self.declare_parameter("dwell_resume_yaw_tolerance_rad", 0.35)
        self.declare_parameter("min_dispatch_interval_sec", 0.5)
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("executor_heartbeat_period_sec", 0.2)
        self.declare_parameter("teleop_topic", "/cmd_vel_teleop")
        self.declare_parameter("return_cancel_timeout_sec", 15.0)
        self.declare_parameter("web_map_dir", str(default_web_map_dir()))
        self.declare_parameter('enable_recorded_routes', False)
        self.declare_parameter('recorded_min_turning_radius', 0.69)
        self.declare_parameter('recorded_footprint', '[[0.4475,0.28],[0.4475,-0.28],[-0.4475,-0.28],[-0.4475,0.28]]')
        self.declare_parameter('recorded_detour_max_offset', 0.8)
        self.declare_parameter('recorded_detour_max_skip', 4.0)
        self.declare_parameter('recorded_detour_max_length', 10.0)
        self.declare_parameter('recorded_obstacle_wait_timeout', 120.0)

        self._callback_group = ReentrantCallbackGroup()
        self._instance_id = "patrol_" + uuid.uuid4().hex
        database_path = str(self.get_parameter("database_path").value)
        if os.environ.get("ROBOT_PATROL_DB"):
            database_path = os.environ["ROBOT_PATROL_DB"]
        self._store = PatrolStore(database_path)
        self._active_map_id = self._store.get_active_map_id()
        self._active_map_lock = threading.RLock()
        self._run_lock = threading.RLock()
        self._active_engine: PatrolEngine | None = None
        self._active_driver: Nav2Driver | None = None
        self._active_goal_handle = None
        self._active_request_id = ""
        self._active_owner_session_id = ""
        self._return_active = False
        self._return_goal_handle = None
        self._return_driver: Nav2Driver | None = None
        self._return_flags: ControlFlags | None = None
        self._return_cancel_requested = False
        self._return_safety_reason = ""
        self._return_owner_session_id = ""
        self._return_request_id = ""
        self._return_map_id = ""
        self._return_state = STATE_IDLE
        self._return_status: dict = {}
        self._run_reserved = False
        self._pending_cancel = False
        self._safety_interrupt_reason = ""
        self._safety_interrupt_delivered = False
        self._guard_status_message = ""
        self._hazard_driver: Nav2Driver | None = None
        self._last_status: dict = {
            "state": STATE_IDLE, "sequence": 0, "run_id": "", "route_id": "",
            "map_id": "", "loop_index": 0, "total_loops": 0, "step_index": 0,
            "step_count": 0, "current_point_name": "", "dwell_remaining": 0.0,
            "error_code": "", "message": "", "current_goal_id": "", "active": False,
        }
        self._status_lock = threading.RLock()
        self._request_cache: dict[str, Point] = {}
        self._odom_lock = threading.RLock()
        self._latest_odom_msg: Odometry | None = None
        self._latest_odom_time = 0.0
        self._map_lock = threading.RLock()
        self._latest_map: OccupancyGrid | None = None
        self._latest_map_time = 0.0
        self._safety_lock = threading.RLock()
        self._last_estop: bool | None = None
        self._estop_time = 0.0
        self._guard_ready: bool | None = None
        self._guard_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(PatrolStatus, "/patrol/status", status_qos,
                                                 callback_group=self._callback_group)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data,
                                 callback_group=self._callback_group)
        self.create_subscription(Bool, "/emergency_stop", self._on_estop, 10,
                                 callback_group=self._callback_group)
        self.create_subscription(Bool, "/patrol/guard_ready", self._on_guard_ready, 10,
                                 callback_group=self._callback_group)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos,
                                 callback_group=self._callback_group)
        self.create_subscription(
            Twist, str(self.get_parameter("teleop_topic").value), self._on_manual_velocity, 10,
            callback_group=self._callback_group)
        self.create_subscription(
            String, "/patrol/guard_status", self._on_guard_status, 10,
            callback_group=self._callback_group)
        self._executor_heartbeat_pub = self.create_publisher(
            ExecutorHeartbeat, "/patrol/executor_heartbeat", 10)

        self._run_server = ActionServer(
            self, RunRoute, "/patrol/run_route", self._execute_run,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._return_server = ActionServer(
            self, ReturnHome, "/patrol/return_home", self._execute_return_home,
            goal_callback=self._return_goal_callback,
            cancel_callback=self._return_cancel_callback,
            callback_group=self._callback_group,
        )

        def service(type_, name, callback):
            return self.create_service(type_, name, callback, callback_group=self._callback_group)

        service(ListMaps, "/patrol/list_maps", self._on_list_maps)
        service(RegisterMap, "/patrol/register_map", self._on_register_map)
        service(SetActiveMap, "/patrol/set_active_map", self._on_set_active_map)
        service(ListPoints, "/patrol/list_points", self._on_list_points)
        service(UpsertPoint, "/patrol/upsert_point", self._on_upsert_point)
        service(DeletePoint, "/patrol/delete_point", self._on_delete_point)
        service(SaveCurrentPoint, "/patrol/save_current_point", self._on_save_current_point)
        service(ListRoutes, "/patrol/list_routes", self._on_list_routes)
        service(UpsertRoute, "/patrol/upsert_route", self._on_upsert_route)
        service(DeleteRoute, "/patrol/delete_route", self._on_delete_route)
        service(ControlRun, "/patrol/control", self._on_control)
        service(GetRuns, "/patrol/get_runs", self._on_get_runs)
        service(GetHomePoint, "/patrol/get_home_point", self._on_get_home_point)
        service(SetHomePoint, "/patrol/set_home_point", self._on_set_home_point)

        period = float(self.get_parameter("status_period_sec").value)
        self._status_timer = self.create_timer(max(0.2, period), self._publish_last_status,
                                               callback_group=self._callback_group)
        heartbeat_period = max(0.05, float(
            self.get_parameter("executor_heartbeat_period_sec").value))
        self._executor_heartbeat_timer = self.create_timer(
            heartbeat_period, self._publish_executor_heartbeat,
            callback_group=self._callback_group)
        self.get_logger().info(f"robot_patrol started, instance={self._instance_id}")
        self._publish_last_status()
        self._publish_executor_heartbeat()
        self.roads = RoadComponent(self)

    # ------------------------------------------------------------- conversions
    def _to_ros_map(self, entry) -> MapBinding:
        message = MapBinding()
        for name in ("map_id", "name", "backend", "frame_id", "navigation_yaml",
                     "image_path", "localization_file", "resolution", "origin_x",
                     "origin_y", "origin_yaw", "width", "height",
                     "navigation_yaml_sha256", "image_sha256", "localization_sha256",
                     "revision"):
            setattr(message, name, getattr(entry, name))
        message.verified = bool(entry.verified)
        return message

    def _to_ros_point(self, point: Point) -> PatrolPoint:
        message = PatrolPoint()
        message.point_id = point.point_id
        message.map_id = point.map_id
        message.name = point.name
        message.x = float(point.x)
        message.y = float(point.y)
        message.yaw = float(point.yaw)
        message.source = point.source
        message.sampled_at = stamp_from_ns(point.sampled_at_ns)
        message.revision = int(point.revision)
        return message

    def _to_ros_route(self, route) -> PatrolRoute:
        message = PatrolRoute()
        message.route_id = route.route_id
        message.map_id = route.map_id
        message.name = route.name
        message.revision = int(route.revision)
        message.navigation_mode = route.navigation_mode
        message.network_id = route.network_id
        message.network_revision = int(route.network_revision)
        message.steps = []
        for step in route.steps:
            item = RouteStep()
            item.step_id = step.step_id
            item.point_id = step.point_id
            item.point_name = step.point_name
            item.x = float(step.x)
            item.y = float(step.y)
            item.yaw = float(step.yaw)
            item.dwell_seconds = float(step.dwell_seconds)
            message.steps.append(item)
        return message

    def _to_ros_status(self, data: dict | None = None) -> PatrolStatus:
        status = PatrolStatus()
        data = data or self._last_status
        status.instance_id = self._instance_id
        status.sequence = int(data.get("sequence", 0))
        status.run_id = str(data.get("run_id", ""))
        status.route_id = str(data.get("route_id", ""))
        status.map_id = str(data.get("map_id", ""))
        status.state = int(data.get("state", STATE_IDLE))
        status.loop_index = int(data.get("loop_index", 0))
        status.total_loops = int(data.get("total_loops", 0))
        status.step_index = int(data.get("step_index", 0))
        status.step_count = int(data.get("step_count", 0))
        status.current_point_name = str(data.get("current_point_name", ""))
        status.dwell_remaining = float(data.get("dwell_remaining", 0.0))
        status.error_code = str(data.get("error_code", ""))
        status.message = str(data.get("message", ""))
        status.current_goal_id = str(data.get("current_goal_id", ""))
        status.active = bool(data.get("active", False))
        return status

    def _to_ros_run(self, record) -> RunRecord:
        message = RunRecord()
        message.run_id = record.run_id
        message.route_id = record.route_id
        message.map_id = record.map_id
        message.state = int(record.state)
        message.completed_loops = int(record.completed_loops)
        message.completed_steps = int(record.completed_steps)
        message.success = bool(record.success)
        message.error_code = record.error_code
        message.message = record.message
        message.route_revision = int(record.route_revision)
        message.map_revision = int(record.map_revision)
        message.started_at = stamp_from_ns(record.started_at_ns)
        message.finished_at = stamp_from_ns(record.finished_at_ns)
        return message

    # ---------------------------------------------------------------- callbacks
    def _on_odom(self, message: Odometry) -> None:
        with self._odom_lock:
            self._latest_odom_msg = message
            self._latest_odom_time = time.monotonic()

    def latest_odom(self):
        with self._odom_lock:
            return self._latest_odom_msg, self._latest_odom_time

    def _on_estop(self, message: Bool) -> None:
        with self._safety_lock:
            self._last_estop = bool(message.data)
            self._estop_time = time.monotonic()
        if bool(message.data):
            self._request_safety_interrupt("急停保持中，巡航任务已中断")

    def _on_guard_ready(self, message: Bool) -> None:
        with self._safety_lock:
            self._guard_ready = bool(message.data)
            self._guard_time = time.monotonic()
        if not bool(message.data):
            self._request_safety_interrupt("patrol guard 联锁未就绪，巡航任务已中断")

    def _on_guard_status(self, message: String) -> None:
        with self._safety_lock:
            self._guard_status_message = str(message.data)

    def _on_manual_velocity(self, message: Twist) -> None:
        # A manual controller is a higher-priority takeover source. The
        # velocity lock stops the physical command immediately; the patrol
        # state machine must also stop dispatching and terminate the task.
        twist = message.twist if hasattr(message, "twist") else message
        moving = (abs(float(twist.linear.x)) > 1e-6 or
                  abs(float(twist.linear.y)) > 1e-6 or
                  abs(float(twist.linear.z)) > 1e-6 or
                  abs(float(twist.angular.x)) > 1e-6 or
                  abs(float(twist.angular.y)) > 1e-6 or
                  abs(float(twist.angular.z)) > 1e-6)
        if moving:
            self._request_safety_interrupt("检测到手动速度输入，巡航任务已被接管")

    def _on_map(self, message: OccupancyGrid) -> None:
        with self._map_lock:
            self._latest_map = message
            self._latest_map_time = time.monotonic()

    def _publish_status_dict(self, data: dict) -> None:
        with self._status_lock:
            self._last_status = dict(data)
            message = self._to_ros_status(data)
        self._status_pub.publish(message)

    def _request_safety_interrupt(self, reason: str) -> None:
        """Unify ESTOP, guard loss and manual takeover into one stop path."""
        reason = str(reason or "安全联锁触发，巡航任务已中断")
        with self._run_lock:
            engine = self._active_engine
            driver = self._active_driver
            return_active = self._return_active
            return_driver = self._return_driver
            return_flags = self._return_flags
            # Idle manual teleoperation, guard startup and e-stop publication
            # are not task interruptions. Only log/act once a route validation,
            # route engine or return-home task actually owns motion.
            if engine is None and not return_active and not self._run_reserved:
                return
            if engine is not None and not self._safety_interrupt_delivered:
                self._safety_interrupt_delivered = True
                self._safety_interrupt_reason = reason
            if engine is None and self._run_reserved:
                self._pending_cancel = True
                self._safety_interrupt_reason = reason
            if return_active:
                self._return_safety_reason = reason
        self.get_logger().warn(reason)
        if engine is not None:
            engine.request_interrupt(reason)
        if driver is not None:
            driver.cancel_current()
        if return_active:
            if return_flags is not None:
                return_flags.request_cancel(reason)
            if return_driver is not None:
                return_driver.cancel_current()

    def _monitor_active_safety(self) -> None:
        with self._run_lock:
            if self._active_engine is None and not self._return_active:
                return
        now = time.monotonic()
        feedback_timeout = float(self.get_parameter("feedback_max_age_sec").value)
        reason = ""
        with self._safety_lock:
            estop = self._last_estop
            estop_time = self._estop_time
            guard_ready = self._guard_ready
            guard_time = self._guard_time
            guard_detail = self._guard_status_message
        if estop is None or (now - estop_time) > feedback_timeout:
            reason = "急停状态未知或已过期，巡航任务已中断"
        elif estop:
            reason = "急停保持中，巡航任务已中断"
        if bool(self.get_parameter("require_guard").value):
            if guard_ready is not True or (now - guard_time) > float(
                    self.get_parameter("guard_timeout_sec").value):
                try:
                    detail = json.loads(guard_detail).get("reason", "")
                except Exception:  # noqa: BLE001 - status is advisory only
                    detail = ""
                reason = detail or "patrol guard 心跳丢失或联锁未就绪，巡航任务已中断"
        if reason:
            self._request_safety_interrupt(reason)

    def _monitor_hazard(self) -> None:
        # A timed-out/failed child goal can remain alive at Nav2 even after the
        # top-level patrol action returned. Keep the single run slot reserved
        # until the driver has observed that child reach a terminal state.
        with self._run_lock:
            driver = self._hazard_driver
            if driver is not None and driver.stop_safe():
                self._hazard_driver = None
                if self._active_engine is None and not self._return_active:
                    self._run_reserved = False

    def _publish_last_status(self) -> None:
        self._monitor_hazard()
        self._monitor_active_safety()
        with self._status_lock:
            data = dict(self._last_status)
        self._status_pub.publish(self._to_ros_status(data))

    def _publish_executor_heartbeat(self) -> None:
        with self._run_lock:
            engine = self._active_engine
            return_active = self._return_active
            active = engine is not None or return_active or self._run_reserved
            owner_session_id = self._active_owner_session_id
            driver = self._active_driver
            if engine is not None:
                run_id = engine.run_id
                status = engine.status
                state = int(status.get("state", STATE_VALIDATING))
                dispatch_allowed = (not engine.flags.cancel_requested and
                                    not engine.flags.pause_requested)
            elif return_active:
                # While an old route is still canceling, report the route's
                # executor identity above. Once the child is terminated this
                # branch owns the same single executor slot for return-home.
                owner_session_id = self._return_owner_session_id
                run_id = ("return_" + self._return_request_id
                          if self._return_request_id else "return")
                state = int(self._return_state or STATE_VALIDATING)
                dispatch_allowed = False
                driver = self._return_driver
            elif active:
                state = int(STATE_VALIDATING)
                dispatch_allowed = False
                run_id = ""
            else:
                state = int(STATE_IDLE)
                dispatch_allowed = True
                run_id = ""
            hazard = self._hazard_driver
            stop_unconfirmed = (hazard is not None and not hazard.stop_safe()) or bool(
                driver is not None and driver.stop_unconfirmed)
        heartbeat = ExecutorHeartbeat()
        heartbeat.instance_id = self._instance_id
        heartbeat.owner_session_id = owner_session_id
        heartbeat.run_id = run_id
        heartbeat.state = state
        heartbeat.active = bool(active)
        heartbeat.dispatch_allowed = bool(dispatch_allowed)
        heartbeat.stop_unconfirmed = bool(stop_unconfirmed)
        if driver is None:
            driver = self._hazard_driver
        heartbeat.nav_goal_id = driver.current_nav_goal_id if driver is not None else ""
        heartbeat.stamp = self.get_clock().now().to_msg()
        self._executor_heartbeat_pub.publish(heartbeat)

    def _notify_status(self, data: dict) -> None:
        self._publish_status_dict(data)
        goal_handle = self._active_goal_handle
        if goal_handle is not None:
            feedback = RunRoute.Feedback()
            feedback.status = self._to_ros_status(data)
            try:
                goal_handle.publish_feedback(feedback)
            except Exception:  # noqa: BLE001 - action may already be terminal
                pass

    def _publish_return_status(self, *, state: int, message: str = "",
                               error_code: str = "", point_name: str = "待命点",
                               active: bool = True) -> None:
        """Publish one return-home state and feed the active action."""
        with self._run_lock:
            driver = self._return_driver
            data = dict(self._return_status)
            data.update({
                "state": int(state),
                "sequence": int(data.get("sequence", 0)) + 1,
                "run_id": ("return_" + self._return_request_id
                           if self._return_request_id else "return"),
                "route_id": "",
                "map_id": self._return_map_id,
                "loop_index": 1 if active else 0,
                "total_loops": 1,
                "step_index": 1 if active else 0,
                "step_count": 1,
                "current_point_name": point_name if active else "",
                "dwell_remaining": 0.0,
                "error_code": str(error_code or ""),
                "message": str(message or ""),
                "current_goal_id": driver.current_nav_goal_id if driver is not None else "",
                "active": bool(active),
            })
            self._return_state = int(state)
            self._return_status = data
            goal_handle = self._return_goal_handle
        self._publish_status_dict(data)
        if goal_handle is not None:
            feedback = ReturnHome.Feedback()
            feedback.status = self._to_ros_status(data)
            try:
                goal_handle.publish_feedback(feedback)
            except Exception:  # noqa: BLE001 - action may be terminal
                pass

    # --------------------------------------------------------------- map/point
    def _on_list_maps(self, request, response):
        try:
            maps = self._store.list_maps()
            with self._active_map_lock:
                active_id = self._active_map_id
            response.maps = [self._to_ros_map(item) for item in maps]
            if active_id:
                active = next((item for item in maps if item.map_id == active_id), None)
                if active:
                    response.active_map = self._to_ros_map(active)
            response.success = True
            response.message = "ok"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _on_register_map(self, request, response):
        try:
            data = read_map_binding(request.navigation_yaml, request.name,
                                    request.backend, request.localization_file)
            previous = self._store.get_map_by_yaml(data["navigation_yaml"])
            geometry_changed = previous is not None and any((
                previous.width != int(data["width"]),
                previous.height != int(data["height"]),
                abs(previous.resolution - float(data["resolution"])) > 1e-12,
                abs(previous.origin_x - float(data["origin_x"])) > 1e-12,
                abs(previous.origin_y - float(data["origin_y"])) > 1e-12,
                abs(previous.origin_yaw - float(data["origin_yaw"])) > 1e-12,
            ))
            entry = self._store.upsert_map(data, request.map_id)
            # Every registration advances the map revision. Coordinates and
            # web assets from the previous generation are no longer valid, so
            # require an explicit re-activation after registration.
            with self._active_map_lock:
                if self._active_map_id == entry.map_id:
                    self._active_map_id = ""
            response.map = self._to_ros_map(entry)
            response.success = True
            response.message = ("地图几何已变化；旧点位和路线已失效并清除，请重新采点" if geometry_changed
                                else "地图已登记；请显式设为活跃地图")
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _verify_map(self, entry) -> None:
        fresh = read_map_binding(entry.navigation_yaml, entry.name, entry.backend,
                                 entry.localization_file)
        checks = (
            ("navigation_yaml_sha256", fresh["navigation_yaml_sha256"]),
            ("image_sha256", fresh["image_sha256"]),
            ("localization_sha256", fresh["localization_sha256"]),
            ("resolution", fresh["resolution"]),
            ("origin_x", fresh["origin_x"]),
            ("origin_y", fresh["origin_y"]),
            ("origin_yaw", fresh["origin_yaw"]),
            ("width", fresh["width"]),
            ("height", fresh["height"]),
        )
        for name, value in checks:
            if getattr(entry, name) != value:
                raise PatrolError("MAP_MISMATCH", f"地图文件已变化：{name}")

    def _on_set_active_map(self, request, response):
        try:
            if self.roads.busy_recording():
                raise PatrolError('BUSY', '录制草稿活动期间禁止切换地图，请先保存或放弃')
            with self._run_lock:
                if self._active_engine is not None or self._run_reserved:
                    raise PatrolError("BUSY", "巡航任务活动期间禁止切换活跃地图")
            entry = self._store.get_map(request.map_id)
            if request.expected_revision and int(request.expected_revision) != int(entry.revision):
                raise PatrolError("STALE_DATA", "地图版本已变化，请刷新后重新确认")
            self._verify_map(entry)
            self._verify_runtime_map(entry)
            self._store.set_active_map(entry.map_id)
            with self._active_map_lock:
                self._active_map_id = entry.map_id
            self._export_active_web_map(entry)
            response.active_map = self._to_ros_map(entry)
            response.success = True
            response.message = "活跃地图已设置，且已与 Nav2 当前 /map 核对"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _verify_runtime_map(self, entry) -> None:
        with self._map_lock:
            message = self._latest_map
        # A static Nav2 map_server publishes its map once with TRANSIENT_LOCAL
        # durability. It does not periodically republish a static map, so the
        # age of the last received message is not a validity criterion. The
        # message must exist, and its full raster must match the selected map
        # files. A map_server replacement/change publishes a newer message.
        if message is None:
            raise PatrolError("NAV_NOT_READY", "尚未收到 /map；无法确认 Nav2 实际加载地图")
        info = message.info
        actual_yaw = quaternion_to_yaw(info.origin.orientation)
        checks = (
            ("width", int(info.width), int(entry.width)),
            ("height", int(info.height), int(entry.height)),
            ("resolution", float(info.resolution), float(entry.resolution)),
            ("origin_x", float(info.origin.position.x), float(entry.origin_x)),
            ("origin_y", float(info.origin.position.y), float(entry.origin_y)),
            ("origin_yaw", actual_yaw, float(entry.origin_yaw)),
        )
        for name, actual, expected in checks:
            if abs(actual - expected) > 1e-6:
                raise PatrolError("MAP_MISMATCH",
                                  f"Nav2 实际 /map 与活跃地图不一致：{name} actual={actual} expected={expected}")

        expected_count = int(info.width) * int(info.height)
        if len(message.data) != expected_count:
            raise PatrolError(
                "MAP_MISMATCH",
                f"Nav2 /map 栅格长度不一致：actual={len(message.data)} expected={expected_count}")
        try:
            expected_width, expected_height, expected_digest = \
                expected_occupancy_sha256(entry.navigation_yaml)
        except PatrolError as error:
            raise PatrolError("MAP_MISMATCH", f"无法核对地图栅格内容：{error.message}") from error
        if (expected_width, expected_height) != (int(info.width), int(info.height)):
            raise PatrolError("MAP_MISMATCH", "地图文件与 Nav2 /map 尺寸不一致")
        actual_digest = hashlib.sha256(
            bytes(int(value) & 0xFF for value in message.data)).hexdigest()
        if actual_digest != expected_digest:
            raise PatrolError("MAP_MISMATCH", "Nav2 实际 /map 栅格内容与已选地图文件不一致")

    def _require_execution_safety(self) -> None:
        now = time.monotonic()
        feedback_timeout = float(self.get_parameter("feedback_max_age_sec").value)
        with self._safety_lock:
            estop = self._last_estop
            estop_time = self._estop_time
            guard_ready = self._guard_ready
            guard_time = self._guard_time
        if estop is None or (now - estop_time) > feedback_timeout:
            raise PatrolError("INTERRUPTED", "急停状态未知或已过期，禁止启动巡航")
        if estop:
            raise PatrolError("INTERRUPTED", "急停保持中，禁止启动巡航")
        if bool(self.get_parameter("require_guard").value):
            if guard_ready is not True or (now - guard_time) > float(
                    self.get_parameter("guard_timeout_sec").value):
                raise PatrolError("LEASE_LOST", "patrol guard 未就绪或心跳过期，禁止启动巡航")

    def _active_map(self):
        with self._active_map_lock:
            map_id = self._active_map_id
        if not map_id:
            raise PatrolError("MAP_MISMATCH", "尚未设置活跃地图")
        return self._store.get_map(map_id)

    def _on_list_points(self, request, response):
        try:
            if request.map_id:
                points = self._store.list_points(request.map_id)
            else:
                points = self._store.list_points(self._active_map().map_id)
            response.points = [self._to_ros_point(item) for item in points]
            response.success = True
            response.message = "ok"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _on_upsert_point(self, request, response):
        try:
            point = self._store.upsert_point(
                request.point_id, request.map_id, request.name, request.x, request.y,
                request.yaw, "manual", now_ns(), request.expected_revision,
            )
            response.point = self._to_ros_point(point)
            response.success = True
            response.message = "点位已保存"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _on_delete_point(self, request, response):
        try:
            self._store.delete_point(request.point_id, request.expected_revision)
            response.success = True
            response.message = "点位已删除"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value,
                self.get_parameter("base_frame").value,
                Time(),
            )
            stamp = transform.header.stamp
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1_000_000_000.0
            if age < -0.5 or age > float(self.get_parameter("tf_max_age_sec").value):
                raise PatrolError("STALE_DATA", "map→base_link TF 已过期，不能采点或判断停稳")
            translation = transform.transform.translation
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            return (translation.x, translation.y,
                    quaternion_to_yaw(transform.transform.rotation), stamp_ns)
        except PatrolError:
            raise
        except Exception as error:  # noqa: BLE001 - TF is external
            raise PatrolError("NAV_NOT_READY", f"无法获取 map→base_link 位姿：{error}") from error

    def _on_save_current_point(self, request, response):
        try:
            if request.request_id and request.request_id in self._request_cache:
                point = self._request_cache[request.request_id]
                response.point = self._to_ros_point(point)
                response.success = True
                response.message = "重复请求返回已有点位"
                return response
            map_entry = self._active_map()
            if request.map_id and request.map_id != map_entry.map_id:
                raise PatrolError("MAP_MISMATCH", "服务请求地图不是当前活跃地图")
            self._verify_map(map_entry)
            self._verify_runtime_map(map_entry)
            odom, received = self.latest_odom()
            if odom is None or (time.monotonic() - received) > float(
                    self.get_parameter("feedback_max_age_sec").value):
                raise PatrolError("NAV_NOT_READY", "没有新鲜的 /odom 反馈")
            linear = require_finite(odom.twist.twist.linear.x, "里程计线速度")
            angular = require_finite(odom.twist.twist.angular.z, "里程计角速度")
            # Recording tolerates odometry jitter independently of arrival,
            # return-home and guard standstill checks. Read on every request
            # so operators can tune these two parameters without restarting.
            linear_limit = require_finite(
                self.get_parameter("record_point_linear_threshold").value,
                "录点线速度阈值")
            angular_limit = require_finite(
                self.get_parameter("record_point_angular_threshold").value,
                "录点角速度阈值")
            if linear_limit < 0.0 or angular_limit < 0.0:
                raise PatrolError("INVALID_DATA", "录点速度阈值必须大于等于 0")
            if abs(linear) > linear_limit or abs(angular) > angular_limit:
                raise PatrolError(
                    "INVALID_DATA",
                    f"车辆仍在运动，确认停稳后再保存点位；当前 |v|={abs(linear):.3f} m/s，"
                    f"|w|={abs(angular):.3f} rad/s；录点豁免阈值："
                    f"|v|≤{linear_limit:.3f} m/s，|w|≤{angular_limit:.3f} rad/s")
            x, y, yaw, stamp_ns = self._current_pose()
            point = self._store.upsert_point(
                "", map_entry.map_id, request.name, x, y, yaw, "current_pose",
                stamp_ns, 0,
            )
            if request.request_id:
                self._request_cache[request.request_id] = point
                if len(self._request_cache) > 200:
                    self._request_cache.pop(next(iter(self._request_cache)))
            response.point = self._to_ros_point(point)
            response.success = True
            response.message = "当前位姿已保存为点位"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    # ---------------------------------------------------------------- route/run
    def _on_list_routes(self, request, response):
        try:
            if request.map_id:
                routes = self._store.list_routes(request.map_id)
            else:
                routes = self._store.list_routes(self._active_map().map_id)
            response.routes = [self._to_ros_route(item) for item in routes]
            response.success = True
            response.message = "ok"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _on_upsert_route(self, request, response):
        try:
            route = self._store.upsert_route(
                request.route_id, request.map_id, request.name,
                request.steps, request.expected_revision,
                request.navigation_mode, request.network_id, request.network_revision,
            )
            response.route = self._to_ros_route(route)
            response.success = True
            response.message = "路线已保存"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _on_delete_route(self, request, response):
        try:
            self._store.delete_route(request.route_id, request.expected_revision)
            response.success = True
            response.message = "路线已删除"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _export_active_web_map(self, entry) -> None:
        """Best-effort atomic web export after activation.

        Failure does not change the local active map, but stale meta/hash makes
        the web task page refuse execution until the export succeeds.
        """
        try:
            configured_dir = str(self.get_parameter("web_map_dir").value or "").strip()
            export_web_map(
                entry.navigation_yaml,
                configured_dir or os.environ.get("ROBOT_WEB_MAP_DIR") or
                str(default_web_map_dir()),
                map_id=entry.map_id,
                revision=int(entry.revision),
                backend=str(entry.backend),
            )
            self.get_logger().info(
                f"网页地图已自动导出：map_id={entry.map_id} revision={entry.revision}")
        except Exception as error:  # noqa: BLE001 - local navigation remains usable
            self.get_logger().error(f"网页地图自动导出失败，网页将禁止执行任务：{error}")

    def _home_point_response(self, response, map_entry, home) -> None:
        response.success = True
        response.map_revision = int(map_entry.revision)
        response.exists = home is not None
        if home is not None:
            response.x = float(home.x)
            response.y = float(home.y)
            response.yaw = float(home.yaw)
            response.message = "ok"
        else:
            response.x = 0.0
            response.y = 0.0
            response.yaw = 0.0
            response.message = "当前地图尚未设置返航点"

    def _on_get_home_point(self, request, response):
        try:
            map_id = request.map_id
            if not map_id:
                map_id = self._active_map().map_id
            entry = self._store.get_map(map_id)
            home = self._store.get_home_point(map_id)
            self._home_point_response(response, entry, home)
        except PatrolError as error:
            response.success = False
            response.exists = False
            response.message = error.message
        return response

    def _on_set_home_point(self, request, response):
        try:
            map_entry = self._active_map()
            if request.map_id and request.map_id != map_entry.map_id:
                raise PatrolError("MAP_MISMATCH", "只能为当前活跃地图设置返航点")
            if request.expected_map_revision and                     int(request.expected_map_revision) != int(map_entry.revision):
                raise PatrolError("STALE_DATA", "地图版本已变化，请刷新后重试")
            if request.clear:
                self._store.clear_home_point(map_entry.map_id)
                self._home_point_response(response, map_entry, None)
                response.message = "返航点已清除"
                return response
            with self._run_lock:
                if self._active_engine is not None or self._return_active or self._run_reserved:
                    raise PatrolError("BUSY", "任务活动期间不能修改返航点")
            self._verify_map(map_entry)
            odom, received = self.latest_odom()
            if odom is None or (time.monotonic() - received) > float(
                    self.get_parameter("feedback_max_age_sec").value):
                raise PatrolError("NAV_NOT_READY", "没有新鲜的 /odom 反馈")
            if abs(float(odom.twist.twist.linear.x)) > float(
                    self.get_parameter("settle_linear_threshold").value) or \
                    abs(float(odom.twist.twist.angular.z)) > float(
                    self.get_parameter("settle_angular_threshold").value):
                raise PatrolError("INVALID_DATA", "车辆仍在运动，确认停稳后才能保存返航点")
            x, y, yaw, _ = self._current_pose()
            home = self._store.set_home_point(
                map_entry.map_id, int(map_entry.revision), x, y, yaw)
            self._home_point_response(response, map_entry, home)
            response.message = "当前位姿已保存为待命返航点"
        except PatrolError as error:
            response.success = False
            response.exists = False
            response.message = error.message
        return response

    def _on_get_runs(self, request, response):
        try:
            runs = self._store.list_runs(request.map_id, request.route_id, request.limit)
            response.runs = [self._to_ros_run(item) for item in runs]
            response.success = True
            response.message = "ok"
        except PatrolError as error:
            response.success = False
            response.message = error.message
        return response

    def _goal_cancel_requested(self, goal_handle) -> bool:
        try:
            action_canceled = bool(goal_handle.is_cancel_requested)
        except Exception:  # noqa: BLE001 - action server boundary
            action_canceled = False
        with self._run_lock:
            return self._pending_cancel or action_canceled

    def _finish_canceled_goal(self, goal_handle, result, message: str,
                              run_id: str = "") -> RunRoute.Result:
        result.success = False
        result.run_id = run_id
        safety_reason = self._safety_interrupt_reason
        if safety_reason:
            result.final_state = STATE_INTERRUPTED
            result.error_code = "INTERRUPTED"
            result.message = safety_reason
            goal_handle.abort()
        else:
            result.final_state = STATE_CANCELED
            result.error_code = "CANCELED"
            result.message = message
            try:
                goal_handle.canceled()
            except Exception:  # noqa: BLE001 - server may already be canceling
                goal_handle.abort()
        return result

    def _goal_callback(self, goal_request):
        if self.roads.busy_recording():
            return GoalResponse.REJECT
        with self._run_lock:
            hazard = self._hazard_driver
            if hazard is not None:
                if not hazard.stop_safe():
                    return GoalResponse.REJECT
                self._hazard_driver = None
                if self._active_engine is None:
                    self._run_reserved = False
            if self._return_active or self._return_goal_handle is not None:
                return GoalResponse.REJECT
            if self._run_reserved or self._active_engine is not None:
                return GoalResponse.REJECT
            # This is the start of a new action request. Any cancellation
            # intent left over from a previous request must not leak into it;
            # a cancel that arrives from now until _execute_run finishes is
            # preserved in _pending_cancel.
            self._pending_cancel = False
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        with self._run_lock:
            self._pending_cancel = True
            engine = self._active_engine
            driver = self._active_driver
        if engine is not None:
            engine.request_cancel()
        if driver is not None:
            driver.cancel_current()
        return CancelResponse.ACCEPT

    def _execute_run(self, goal_handle):
        result = RunRoute.Result()
        result.run_id = ""
        # Match the road service lock order so a passive recording cannot
        # start between goal acceptance and reserving the execution slot.
        with self.roads.lock, self._run_lock:
            if self.roads.recorder is not None:
                result.error_code, result.message = 'BUSY', '请先保存或放弃当前录制草稿'
                result.final_state = STATE_FAILED
                goal_handle.abort()
                return result
            hazard = self._hazard_driver
            if hazard is not None:
                if not hazard.stop_safe():
                    result.error_code = "CANCEL_UNCONFIRMED"
                    result.message = "上一导航目标尚未确认终止，禁止启动新任务"
                    result.final_state = STATE_FAILED
                    goal_handle.abort()
                    return result
                self._hazard_driver = None
                if self._active_engine is None:
                    self._run_reserved = False
            if self._run_reserved or self._active_engine is not None:
                result.error_code = "BUSY"
                result.message = "已有巡航任务在运行"
                result.final_state = STATE_FAILED
                goal_handle.abort()
                return result
            # Reserve the single execution slot before validation. The slot
            # is not released until either no Nav2 child goal exists or the
            # old child has reached a known terminal state.
            self._run_reserved = True
            self._active_owner_session_id = str(
                goal_handle.request.owner_session_id or "").strip()
            self._safety_interrupt_reason = ""
            self._safety_interrupt_delivered = False
        engine = None
        driver = None
        run_id = ""
        loops = 0
        steps = 0
        try:
            # A cancel can be accepted between action-goal acceptance and this
            # execute callback. _pending_cancel is deliberately not reset at
            # execute start, so that pre-validation cancellation cannot be
            # lost and dispatch a navigation goal afterwards.
            if self._goal_cancel_requested(goal_handle):
                return self._finish_canceled_goal(
                    goal_handle, result, "任务在启动前被取消")

            if not goal_handle.request.request_id:
                raise PatrolError("INVALID_DATA", "request_id 不能为空")
            if not goal_handle.request.owner_session_id:
                raise PatrolError("INVALID_DATA", "owner_session_id 不能为空")
            route = self._store.get_route(goal_handle.request.route_id)
            map_entry = self._store.get_map(goal_handle.request.map_id)
            if route.map_id != map_entry.map_id:
                raise PatrolError("MAP_MISMATCH", "路线与地图不匹配")
            with self._active_map_lock:
                active_id = self._active_map_id
            if active_id != map_entry.map_id:
                raise PatrolError("MAP_MISMATCH", "执行地图不是已确认的活跃地图")
            if goal_handle.request.route_revision and \
                    int(goal_handle.request.route_revision) != int(route.revision):
                raise PatrolError("STALE_DATA", "路线已被修改，请刷新预览后重试")
            if goal_handle.request.map_revision and \
                    int(goal_handle.request.map_revision) != int(map_entry.revision):
                raise PatrolError("STALE_DATA", "地图版本已变化，请重新确认")
            self._require_execution_safety()
            self._verify_map(map_entry)
            self._verify_runtime_map(map_entry)
            if self._goal_cancel_requested(goal_handle):
                return self._finish_canceled_goal(
                    goal_handle, result, "任务在地图校验阶段被取消")
            snapshot = RouteSnapshot(
                route_id=route.route_id, map_id=route.map_id, name=route.name,
                revision=route.revision, steps=route.steps,
                navigation_mode=route.navigation_mode,
            )
            loops = int(goal_handle.request.loop_count)
            if loops < 0:
                raise PatrolError("INVALID_DATA", "loop_count 不能为负数")
            if route.navigation_mode == 'RECORDED_NETWORK':
                snapshot.recorded_plan = self.roads.take_plan(route, goal_handle.request.plan_digest, loops)
            self._store.set_navigation_context(map_entry.map_id, route.navigation_mode,
                                               route.network_id, route.network_revision)
            run_id = new_id("run")
            self._store.create_run(run_id, route.route_id, map_entry.map_id,
                                   route.revision, map_entry.revision,
                                   snapshot.as_dict(), loops, STATE_VALIDATING, now_ns())
            driver_type = Nav2Driver
            driver_extra = {}
            if route.navigation_mode == 'RECORDED_NETWORK':
                from .recorded_routes.driver import RecordedDriver
                driver_type = RecordedDriver
                driver_extra = dict(roads=self.roads, plan=snapshot.recorded_plan, run_id=run_id)
            driver = driver_type(
                self,
                **driver_extra,
                map_frame=self.get_parameter("map_frame").value,
                base_frame=self.get_parameter("base_frame").value,
                point_timeout=float(self.get_parameter("point_timeout_sec").value),
                settle_timeout=float(self.get_parameter("settle_timeout_sec").value),
                settle_linear=float(self.get_parameter("settle_linear_threshold").value),
                settle_angular=float(self.get_parameter("settle_angular_threshold").value),
                settle_duration=float(self.get_parameter("settle_duration_sec").value),
                feedback_max_age=float(self.get_parameter("feedback_max_age_sec").value),
                tf_max_age=float(self.get_parameter("tf_max_age_sec").value),
                resume_position_tolerance=float(
                    self.get_parameter("dwell_resume_position_tolerance_m").value),
                resume_yaw_tolerance=float(
                    self.get_parameter("dwell_resume_yaw_tolerance_rad").value),
                cancel_confirm_timeout=float(
                    self.get_parameter("cancel_confirm_timeout_sec").value),
                callback_group=self._callback_group,
                goal_update_callback=self._publish_executor_heartbeat,
            )
            engine = PatrolEngine(
                self._store, snapshot, run_id, loops,
                goal_handle.request.owner_session_id,
                driver, self._notify_status,
                settle_timeout=float(self.get_parameter("settle_timeout_sec").value),
                point_timeout=float(self.get_parameter("point_timeout_sec").value),
                min_dispatch_interval=float(self.get_parameter("min_dispatch_interval_sec").value),
            )
            with self._run_lock:
                safety_reason = self._safety_interrupt_reason
                canceled_before_start = bool(safety_reason) or self._pending_cancel or bool(
                    getattr(goal_handle, "is_cancel_requested", False))
                if canceled_before_start:
                    if safety_reason:
                        engine.request_interrupt(safety_reason)
                        self._safety_interrupt_delivered = True
                    else:
                        engine.request_cancel()
                else:
                    self._active_engine = engine
                    self._active_driver = driver
                    self._active_goal_handle = goal_handle
                    self._active_request_id = goal_handle.request.request_id
            if canceled_before_start:
                if safety_reason:
                    self._store.finish_run(
                        run_id, STATE_INTERRUPTED, False, 0, 0,
                        "INTERRUPTED", safety_reason, now_ns())
                    engine.mark_interrupted_before_start(safety_reason)
                    return self._finish_canceled_goal(
                        goal_handle, result, safety_reason, run_id)
                self._store.finish_run(run_id, STATE_CANCELED, False, 0, 0,
                                       "CANCELED", "任务在校验阶段被取消", now_ns())
                engine.mark_canceled_before_start("任务在校验阶段被取消")
                return self._finish_canceled_goal(
                    goal_handle, result, "任务在校验阶段被取消", run_id)

            if bool(self.get_parameter("require_guard").value):
                # Publish ownership before the first NavigateToPose goal. The
                # guard uses this active heartbeat to distinguish a legitimate
                # patrol child from a late goal owned by a previous restarted
                # executor.
                self._publish_executor_heartbeat()
                heartbeat_period = max(0.05, float(
                    self.get_parameter("executor_heartbeat_period_sec").value))
                time.sleep(max(0.15, heartbeat_period * 2.0))
            engine_result = engine.run()
            loops = engine_result.completed_loops
            steps = engine_result.completed_steps
            self._store.finish_run(
                run_id, engine_result.final_state, engine_result.success,
                engine_result.completed_loops, engine_result.completed_steps,
                engine_result.error_code, engine_result.message, now_ns(),
            )
            result.success = engine_result.success
            result.final_state = engine_result.final_state
            result.run_id = run_id
            result.completed_loops = engine_result.completed_loops
            result.completed_steps = engine_result.completed_steps
            result.error_code = engine_result.error_code
            result.message = engine_result.message
            if engine_result.success:
                goal_handle.succeed()
            elif engine_result.final_state in (STATE_CANCELED,):
                try:
                    goal_handle.canceled()
                except Exception:  # noqa: BLE001 - server may already be canceling
                    goal_handle.abort()
            else:
                goal_handle.abort()
        except PatrolError as error:
            if run_id:
                self._store.finish_run(run_id, STATE_FAILED, False, loops, steps,
                                       error.code, error.message, now_ns())
            result.success = False
            result.final_state = STATE_FAILED
            result.run_id = run_id
            result.completed_loops = loops
            result.completed_steps = steps
            result.error_code = error.code
            result.message = error.message
            goal_handle.abort()
        except Exception as error:  # noqa: BLE001 - node boundary
            if run_id:
                self._store.finish_run(run_id, STATE_FAILED, False, loops, steps,
                                       "INTERRUPTED", str(error), now_ns())
            result.success = False
            result.final_state = STATE_FAILED
            result.run_id = run_id
            result.completed_loops = loops
            result.completed_steps = steps
            result.error_code = "INTERRUPTED"
            result.message = str(error)
            goal_handle.abort()
        finally:
            with self._run_lock:
                self._active_engine = None
                self._active_driver = None
                self._active_goal_handle = None
                self._active_request_id = ""
                self._active_owner_session_id = ""
                self._safety_interrupt_reason = ""
                self._safety_interrupt_delivered = False
                if driver is not None and not driver.stop_safe():
                    # The top-level action may finish, but the old child goal
                    # is still alive at Nav2. Keep the slot reserved until a
                    # terminal status for that child has been observed.
                    self._hazard_driver = driver
                    self._run_reserved = True
                else:
                    if self._hazard_driver is not None and self._hazard_driver.stop_safe():
                        self._hazard_driver = None
                    if self._hazard_driver is None:
                        self._run_reserved = False
                self._pending_cancel = False
        if driver is not None and driver.stop_safe() and hasattr(driver, 'close'):
            driver.close()
        if engine is not None:
            self._publish_status_dict(engine.status)
        return result

    # ----------------------------------------------------------- return home
    def _return_goal_callback(self, goal_request):
        if self.roads.busy_recording():
            return GoalResponse.REJECT
        with self._run_lock:
            if self._return_active or self._return_goal_handle is not None:
                return GoalResponse.REJECT
            self._return_cancel_requested = False
            self._return_safety_reason = ""
        return GoalResponse.ACCEPT

    def _return_cancel_callback(self, goal_handle):
        with self._run_lock:
            self._return_cancel_requested = True
            flags = self._return_flags
            driver = self._return_driver
        if flags is not None:
            flags.request_cancel("操作员取消返航")
        if driver is not None:
            driver.cancel_current()
        return CancelResponse.ACCEPT

    def _return_cancel_pending(self, goal_handle) -> bool:
        try:
            action_canceled = bool(goal_handle.is_cancel_requested)
        except Exception:  # noqa: BLE001 - action boundary
            action_canceled = False
        with self._run_lock:
            return self._return_cancel_requested or action_canceled

    def _wait_for_return_slot(self, goal_handle, timeout: float) -> str:
        """Cancel any route and wait until its Nav2 child is terminal."""
        deadline = time.monotonic() + max(0.5, float(timeout))
        while time.monotonic() < deadline:
            if self._return_cancel_pending(goal_handle):
                return "CANCELED"
            with self._run_lock:
                if self._return_safety_reason:
                    return "INTERRUPTED"
                engine = self._active_engine
                hazard = self._hazard_driver
                reserved = self._run_reserved
            if engine is None and hazard is None and not reserved:
                with self._run_lock:
                    # Route callbacks use the same flag. Recheck after taking
                    # the lock so a just-accepted route cannot slip in.
                    if self._active_engine is None and self._hazard_driver is None:
                        self._run_reserved = True
                        return "OK"
            time.sleep(0.05)
        return "TIMEOUT"

    def _finish_return_action(self, goal_handle, result, state: int, success: bool,
                              error_code: str, message: str) -> ReturnHome.Result:
        result.success = bool(success)
        result.final_state = int(state)
        result.error_code = str(error_code or "")
        result.message = str(message or "")
        self._publish_return_status(
            state=state, message=message, error_code=error_code, active=False)
        if success:
            goal_handle.succeed()
        elif state in (STATE_CANCELED,):
            try:
                goal_handle.canceled()
            except Exception:  # noqa: BLE001 - already canceling
                goal_handle.abort()
        else:
            goal_handle.abort()
        return result

    def _execute_return_home(self, goal_handle):
        result = ReturnHome.Result()
        result.success = False
        request = goal_handle.request
        driver = None
        flags = None
        with self.roads.lock, self._run_lock:
            if self.roads.recorder is not None:
                result.error_code, result.message = 'BUSY', '请先保存或放弃当前录制草稿'
                result.final_state = STATE_FAILED
                goal_handle.abort()
                return result
            self._return_active = True
            self._return_goal_handle = goal_handle
            self._return_owner_session_id = str(request.owner_session_id or "").strip()
            self._return_request_id = str(request.request_id or "").strip()
            self._return_map_id = str(request.map_id or "").strip()
            self._return_driver = None
            self._return_flags = None
            self._return_state = STATE_VALIDATING
            self._return_status = {
                "state": STATE_VALIDATING, "sequence": 0,
                "run_id": ("return_" + self._return_request_id
                           if self._return_request_id else "return"),
                "route_id": "", "map_id": self._return_map_id,
                "loop_index": 0, "total_loops": 1, "step_index": 0,
                "step_count": 1, "current_point_name": "",
                "dwell_remaining": 0.0, "error_code": "", "message": "",
                "current_goal_id": "", "active": True,
            }
        try:
            if self._return_cancel_pending(goal_handle):
                return self._finish_return_action(
                    goal_handle, result, STATE_CANCELED, False, "CANCELED",
                    "返航在启动前被取消")
            if not self._return_request_id:
                raise PatrolError("INVALID_DATA", "request_id 不能为空")
            if not self._return_owner_session_id:
                raise PatrolError("INVALID_DATA", "owner_session_id 不能为空")
            map_entry = self._active_map()
            if self._return_map_id and self._return_map_id != map_entry.map_id:
                raise PatrolError("MAP_MISMATCH", "返航请求地图不是当前活跃地图")
            self._return_map_id = map_entry.map_id
            if request.map_revision and int(request.map_revision) != int(map_entry.revision):
                raise PatrolError("STALE_DATA", "地图版本已变化，请刷新后重试")
            home = self._store.get_home_point(map_entry.map_id)
            if home is None:
                raise PatrolError("HOME_NOT_FOUND", "当前地图没有有效的待命返航点")
            self._require_execution_safety()
            self._publish_return_status(
                state=STATE_VALIDATING, message="返航校验通过，正在清理当前任务")

            # Return-home preempts a route owned by either local or web. The
            # route's own action result is canceled first; return-home never
            # dispatches while an old child goal is still alive.
            with self._run_lock:
                engine = self._active_engine
                active_driver = self._active_driver
                active_goal = self._active_goal_handle
                route_reserved = self._run_reserved
                if route_reserved or engine is not None:
                    # Route execution has a validation window before its
                    # engine object exists. The route action consults this
                    # flag at each validation boundary, so preemption cannot
                    # race into a first NavigateToPose dispatch.
                    self._pending_cancel = True
            if engine is not None:
                self._publish_return_status(
                    state=STATE_CANCELING, message="正在取消当前路线任务")
                engine.request_cancel()
                if active_driver is not None:
                    active_driver.cancel_current()
            if active_goal is not None:
                try:
                    active_goal.cancel_goal_async()
                except Exception:  # noqa: BLE001 - already terminal
                    pass
            slot = self._wait_for_return_slot(
                goal_handle,
                float(self.get_parameter("return_cancel_timeout_sec").value))
            if slot == "CANCELED":
                return self._finish_return_action(
                    goal_handle, result, STATE_CANCELED, False, "CANCELED",
                    "返航已取消")
            if slot == "INTERRUPTED":
                reason = self._return_safety_reason or "安全联锁触发，返航已中断"
                return self._finish_return_action(
                    goal_handle, result, STATE_INTERRUPTED, False,
                    "INTERRUPTED", reason)
            if slot != "OK":
                return self._finish_return_action(
                    goal_handle, result, STATE_FAILED, False, "CANCEL_UNCONFIRMED",
                    "上一路线目标尚未确认终止，禁止开始返航")

            # Validate again after the old task settled: map files, runtime
            # map and safety feedback may have changed while canceling.
            map_entry = self._active_map()
            if map_entry.map_id != self._return_map_id:
                raise PatrolError("MAP_MISMATCH", "返航期间活跃地图已变化")
            if request.map_revision and int(request.map_revision) != int(map_entry.revision):
                raise PatrolError("STALE_DATA", "返航期间地图版本已变化")
            home = self._store.get_home_point(map_entry.map_id)
            if home is None:
                raise PatrolError("HOME_NOT_FOUND", "当前地图没有有效的待命返航点")
            self._require_execution_safety()
            self._verify_map(map_entry)
            self._verify_runtime_map(map_entry)

            flags = ControlFlags()
            return_plan = self.roads.plan_home(home)
            return_type, return_extra = Nav2Driver, {}
            if return_plan is not None:
                from .recorded_routes.driver import RecordedDriver
                return_type = RecordedDriver
                return_extra = dict(roads=self.roads, plan=return_plan, run_id=self._return_status['run_id'])
            driver = return_type(
                self,
                **return_extra,
                map_frame=self.get_parameter("map_frame").value,
                base_frame=self.get_parameter("base_frame").value,
                point_timeout=float(self.get_parameter("point_timeout_sec").value),
                settle_timeout=float(self.get_parameter("settle_timeout_sec").value),
                settle_linear=float(self.get_parameter("settle_linear_threshold").value),
                settle_angular=float(self.get_parameter("settle_angular_threshold").value),
                settle_duration=float(self.get_parameter("settle_duration_sec").value),
                feedback_max_age=float(self.get_parameter("feedback_max_age_sec").value),
                tf_max_age=float(self.get_parameter("tf_max_age_sec").value),
                resume_position_tolerance=float(
                    self.get_parameter("dwell_resume_position_tolerance_m").value),
                resume_yaw_tolerance=float(
                    self.get_parameter("dwell_resume_yaw_tolerance_rad").value),
                cancel_confirm_timeout=float(
                    self.get_parameter("cancel_confirm_timeout_sec").value),
                callback_group=self._callback_group,
                goal_update_callback=self._publish_executor_heartbeat,
            )
            with self._run_lock:
                self._return_driver = driver
                self._return_flags = flags
            self._publish_return_status(
                state=STATE_NAVIGATING, message="正在返回待命点")
            if bool(self.get_parameter("require_guard").value):
                self._publish_executor_heartbeat()
                time.sleep(max(0.15, float(
                    self.get_parameter("executor_heartbeat_period_sec").value) * 2.0))
            if self._return_cancel_pending(goal_handle):
                flags.request_cancel("操作员取消返航")

            def status_callback(**changes):
                state = int(changes.pop("state", STATE_NAVIGATING))
                message = str(changes.pop("message", "") or "正在返回待命点")
                self._publish_return_status(state=state, message=message)

            nav_result = driver.navigate(home, flags, status_callback)
            if nav_result == "SUCCEEDED":
                self._publish_return_status(
                    state=STATE_SETTLING, message="已到待命点，正在确认停稳")
                settle = driver.settle(home, flags, status_callback)
                if settle[0] == "OK":
                    return self._finish_return_action(
                        goal_handle, result, STATE_SUCCEEDED, True, "",
                        "已返回待命点并确认停稳")
                if settle[0] == "CANCELED":
                    nav_result = "CANCELED"
                else:
                    nav_result = settle[0] or "SETTLE_FAILED"
            if self._return_cancel_pending(goal_handle) or nav_result == "CANCELED":
                return self._finish_return_action(
                    goal_handle, result, STATE_CANCELED, False, "CANCELED",
                    "返航已取消")
            if self._return_safety_reason:
                return self._finish_return_action(
                    goal_handle, result, STATE_INTERRUPTED, False, "INTERRUPTED",
                    self._return_safety_reason)
            return self._finish_return_action(
                goal_handle, result, STATE_FAILED, False,
                nav_result or "NAV_FAILED", f"返回待命点失败：{nav_result}")
        except PatrolError as error:
            return self._finish_return_action(
                goal_handle, result, STATE_FAILED, False, error.code, error.message)
        except Exception as error:  # noqa: BLE001 - node boundary
            return self._finish_return_action(
                goal_handle, result, STATE_FAILED, False, "INTERRUPTED", str(error))
        finally:
            with self._run_lock:
                stop_safe = driver is None or driver.stop_safe()
                self._return_driver = None
                self._return_flags = None
                self._return_goal_handle = None
                self._return_active = False
                self._return_cancel_requested = False
                self._return_safety_reason = ""
                self._return_owner_session_id = ""
                self._return_request_id = ""
                self._return_state = STATE_IDLE
                if not stop_safe:
                    self._hazard_driver = driver
                    self._run_reserved = True
                elif self._hazard_driver is None:
                    self._run_reserved = False
            self._publish_executor_heartbeat()

            if driver is not None and driver.stop_safe() and hasattr(driver, 'close'):
                driver.close()

    def _on_control(self, request, response):
        with self._run_lock:
            engine = self._active_engine
            driver = self._active_driver
            run_id = engine.run_id if engine is not None else ""
            status = engine.status if engine is not None else self._last_status
        try:
            if engine is None or (request.run_id and request.run_id != run_id):
                raise PatrolError("BUSY", "没有匹配的活动巡航任务")
            if request.command == ControlRun.Request.PAUSE:
                engine.request_pause()
                response.message = "暂停请求已受理"
            elif request.command == ControlRun.Request.RESUME:
                # RESUME is a new motion decision: recheck the same safety
                # prerequisites as start. Do not clear pause if they fail.
                self._require_execution_safety()
                active_map = self._active_map()
                self._verify_runtime_map(active_map)
                with self._run_lock:
                    if self._active_engine is not engine:
                        raise PatrolError("BUSY", "巡航任务已结束，无法继续")
                    if not engine.flags.pause_requested:
                        raise PatrolError("INVALID_DATA", "当前任务未处于暂停状态")
                    engine.request_resume()
                response.message = "继续请求已受理"
            elif request.command == ControlRun.Request.CANCEL:
                engine.request_cancel()
                if driver is not None:
                    driver.cancel_current()
                response.message = "取消请求已受理"
            elif request.command == ControlRun.Request.CLAIM:
                response.accepted = False
                response.message = "当前版本尚未开放 CLAIM，请重新连接后在主控制端取消任务"
                response.status = self._to_ros_status(status)
                return response
            else:
                raise PatrolError("INVALID_DATA", "未知控制命令")
            response.accepted = True
            response.status = self._to_ros_status(engine.status)
        except PatrolError as error:
            response.accepted = False
            response.message = error.message
            response.status = self._to_ros_status(status)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        node._store.close()
        rclpy.try_shutdown()
