"""Thin blocking adapter around Nav2 NavigateToPose for the patrol executor."""
from __future__ import annotations

from typing import Any, Callable
import math
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Nav2Driver:
    def __init__(self, node: Node, action_name: str = "/navigate_to_pose",
                 map_frame: str = "map", base_frame: str = "base_link",
                 point_timeout: float = 300.0, settle_timeout: float = 10.0,
                 settle_linear: float = 0.03, settle_angular: float = 0.05,
                 settle_duration: float = 0.8, feedback_max_age: float = 1.5,
                 tf_max_age: float = 1.5, resume_position_tolerance: float = 0.5,
                 resume_yaw_tolerance: float = 0.35, callback_group=None):
        self.node = node
        self.action_name = action_name
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.point_timeout = float(point_timeout)
        self.settle_timeout = float(settle_timeout)
        self.settle_linear = float(settle_linear)
        self.settle_angular = float(settle_angular)
        self.settle_duration = float(settle_duration)
        self.feedback_max_age = float(feedback_max_age)
        self.tf_max_age = float(tf_max_age)
        self.resume_position_tolerance = float(resume_position_tolerance)
        self.resume_yaw_tolerance = float(resume_yaw_tolerance)
        self.client = ActionClient(node, NavigateToPose, action_name, callback_group=callback_group)
        self._current_handle = None
        self._handle_lock = threading.Lock()
        self.stop_unconfirmed = False
        self.last_error = ""
        self.last_goal_id = ""

    def cancel_current(self) -> None:
        with self._handle_lock:
            handle = self._current_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as error:  # noqa: BLE001 - best effort cancel
                self.last_error = str(error)

    def stop_safe(self) -> bool:
        with self._handle_lock:
            return self._current_handle is None and not self.stop_unconfirmed

    def _mark_unconfirmed(self, handle, result_future) -> None:
        """Keep the goal handle until Nav2 reports a terminal state."""
        with self._handle_lock:
            self._current_handle = handle
            self.stop_unconfirmed = True

        def clear(_future):
            with self._handle_lock:
                if self._current_handle is handle:
                    self._current_handle = None
                self.stop_unconfirmed = False

        result_future.add_done_callback(clear)

    def _wait_future(self, future, timeout: float, flags, status_cb,
                     on_timeout: Callable[[], None] | None = None):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        deadline = time.monotonic() + timeout
        while True:
            if event.wait(0.05):
                try:
                    return future.result()
                except Exception as error:  # noqa: BLE001 - navigation boundary
                    self.last_error = str(error)
                    return None
            if time.monotonic() >= deadline:
                if on_timeout:
                    on_timeout()
                self.last_error = "TIMEOUT"
                return None

    def _cancel_late_accept(self, future) -> None:
        """Cancel a goal that is accepted after the caller timed out."""
        try:
            handle = future.result()
            if handle is not None and getattr(handle, "accepted", False):
                result_future = handle.get_result_async()
                self._mark_unconfirmed(handle, result_future)
                handle.cancel_goal_async()
            else:
                with self._handle_lock:
                    self.stop_unconfirmed = False
        except Exception as error:  # noqa: BLE001 - late goal best effort
            self.last_error = str(error)
            with self._handle_lock:
                self.stop_unconfirmed = True

    def navigate(self, step: Any, flags, status_cb: Callable[[dict[str, Any]], None]) -> str:
        if not self.client.wait_for_server(timeout_sec=0.2):
            self.last_error = "NAV_NOT_READY"
            return "NAV_NOT_READY"
        self.last_goal_id = "goal_" + uuid.uuid4().hex
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(step.x)
        pose.pose.position.y = float(step.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(float(step.yaw))
        goal.pose = pose

        def feedback_callback(message):
            status_cb(state=3, current_point_name=step.point_name,
                      current_goal_id=self.last_goal_id,
                      message="导航中")

        send_future = self.client.send_goal_async(goal, feedback_callback=feedback_callback)
        goal_handle = self._wait_future(send_future, min(15.0, self.point_timeout),
                                        flags, status_cb)
        if goal_handle is None:
            # Never leave a late acceptance unsupervised. Keep the node's
            # stop-unconfirmed interlock until the late future is resolved.
            # A pause is not a safe terminal state here: the goal may still be
            # accepted after this timeout, so reporting PAUSED would let the
            # engine resume and dispatch a second child goal.
            with self._handle_lock:
                self.stop_unconfirmed = True
            send_future.add_done_callback(self._cancel_late_accept)
            if flags.cancel_requested or flags.pause_requested:
                self.last_error = "CANCEL_UNCONFIRMED"
                return "CANCEL_UNCONFIRMED"
            return "TIMEOUT"
        if not getattr(goal_handle, "accepted", False):
            self.last_error = "GOAL_REJECTED"
            return "REJECTED"
        unconfirmed = False
        with self._handle_lock:
            self._current_handle = goal_handle
        try:
            if flags.cancel_requested or flags.pause_requested:
                self.last_error = "CANCELED"
                goal_handle.cancel_goal_async()
            result_future = goal_handle.get_result_async()
            result = self._wait_future(result_future, self.point_timeout, flags, status_cb,
                                       on_timeout=self.cancel_current)
            if result is None:
                # The result future did not reach a terminal state. Keep the
                # task failed/cancel-unconfirmed rather than pretending the
                # subgoal stopped; the caller must not dispatch a new point.
                unconfirmed = True
                self._mark_unconfirmed(goal_handle, result_future)
                if flags.cancel_requested or flags.pause_requested:
                    self.last_error = "CANCEL_UNCONFIRMED"
                    return "CANCEL_UNCONFIRMED"
                self.last_error = "TIMEOUT"
                return "TIMEOUT"
            status = int(result.status)
            if status == GoalStatus.STATUS_SUCCEEDED:
                return "SUCCEEDED"
            if status == GoalStatus.STATUS_CANCELED:
                if flags.cancel_requested:
                    return "CANCELED"
                if flags.pause_requested:
                    return "PAUSED"
                return "CANCELED"
            self.last_error = f"NAV_ABORTED({status})"
            return "ABORTED"
        finally:
            if not unconfirmed:
                with self._handle_lock:
                    self._current_handle = None

    def _current_pose(self) -> tuple[float, float, float, int] | None:
        try:
            transform = self.node.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
            stamp = transform.header.stamp
            age = (self.node.get_clock().now() -
                   Time.from_msg(stamp)).nanoseconds / 1_000_000_000.0
            if age < -0.5 or age > self.tf_max_age:
                self.last_error = "STALE_TF"
                return None
            translation = transform.transform.translation
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            return (translation.x, translation.y,
                    quaternion_to_yaw(transform.transform.rotation), stamp_ns)
        except Exception:  # noqa: BLE001 - TF can be missing while localization starts
            return None

    def settle(self, step: Any, flags, status_cb: Callable[[dict[str, Any]], None]):
        deadline = time.monotonic() + self.settle_timeout
        stable_since = None
        while time.monotonic() < deadline:
            if flags.cancel_requested:
                return ("CANCELED", None)
            if flags.pause_requested:
                return ("PAUSED", None)
            odom, received = self.node.latest_odom()
            if odom is None or (time.monotonic() - received) > self.feedback_max_age:
                stable_since = None
                time.sleep(0.1)
                continue
            pose = self._current_pose()
            if pose is None:
                stable_since = None
                time.sleep(0.1)
                continue
            pose = pose[:3]
            linear = abs(float(odom.twist.twist.linear.x))
            angular = abs(float(odom.twist.twist.angular.z))
            if linear <= self.settle_linear and angular <= self.settle_angular:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= self.settle_duration:
                    return ("OK", pose)
            else:
                stable_since = None
            time.sleep(0.1)
        return ("TIMEOUT", None)


    def reconfirm(self, step: Any, flags, status_cb) -> str:
        settle = self.settle(step, flags, status_cb)
        if settle[0] != "OK":
            return settle[0]
        x, y, yaw = settle[1]
        distance = math.hypot(x - float(step.x), y - float(step.y))
        yaw_error = math.atan2(math.sin(float(step.yaw) - yaw),
                               math.cos(float(step.yaw) - yaw))
        if distance <= self.resume_position_tolerance and abs(yaw_error) <= self.resume_yaw_tolerance:
            return "AT_POINT"
        self.last_error = "OFF_POINT"
        return "OFF_POINT"
