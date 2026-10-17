"""Independent patrol dead-man guard and twist_mux interlock writer."""
from __future__ import annotations

import json
import re
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from patrol_interfaces.msg import ExecutorHeartbeat, OperatorHeartbeat
from patrol_interfaces.srv import ResetInterlock

from .guard_logic import GuardDecision, PatrolGuardState


class PatrolGuard(Node):
    """Owns /patrol/stop_lock and publishes guard readiness.

    The process intentionally has no dependency on the patrol executor's
    Python objects or lifecycle. If it exits, twist_mux expires the positive
    timeout on /patrol/stop_lock and blocks all velocity inputs; the chassis
    bridge then sees no fresh command and commands zero speed.
    """

    def __init__(self, parameter_overrides=None):
        super().__init__("patrol_guard", parameter_overrides=parameter_overrides)
        self.declare_parameter("publish_period_sec", 0.05)
        self.declare_parameter("executor_timeout_sec", 1.5)
        self.declare_parameter("operator_timeout_sec", 1.5)
        self.declare_parameter("startup_grace_sec", 1.0)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("stop_linear_threshold", 0.03)
        self.declare_parameter("stop_angular_threshold", 0.05)
        self.declare_parameter("require_operator_heartbeat", True)
        self.declare_parameter("require_mux_interlock", True)
        self.declare_parameter("mux_timeout_sec", 3.0)
        self.declare_parameter("mux_diagnostics_topic", "/diagnostics")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("stale_goal_cancel_timeout_sec", 3.0)
        self.declare_parameter("stale_goal_service_timeout_sec", 2.0)
        self.declare_parameter("startup_nav_goal_quiet_sec", 2.0)
        self.declare_parameter("unexpected_nav_goal_grace_sec", 0.75)

        # Reset verification blocks until Nav2 has accepted a cancel-all and
        # the action status shows no non-terminal goals. Reentrant callbacks
        # let the action-status subscription and cancel response run while the
        # reset service callback is waiting.
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._nav_goal_lock = threading.RLock()
        self._nav_cancel_lock = threading.Lock()
        self._startup_nav_verify_stop = threading.Event()
        self._startup_nav_verify_thread: threading.Thread | None = None
        self._unauthorized_nav_cancel_thread: threading.Thread | None = None
        self._startup_nav_verify_last_error = ""
        self._nav_statuses: dict[str, int] = {}
        self._nav_status_sequence = 0
        self._nav_action_name = str(
            self.get_parameter("nav2_action_name").value).rstrip("/") or "/navigate_to_pose"
        self._stale_goal_cancel_timeout = max(
            0.2, float(self.get_parameter("stale_goal_cancel_timeout_sec").value))
        self._stale_goal_service_timeout = max(
            0.2, float(self.get_parameter("stale_goal_service_timeout_sec").value))
        self._startup_nav_goal_quiet = max(
            0.0, float(self.get_parameter("startup_nav_goal_quiet_sec").value))
        self._startup_nav_probe_response_timeout = max(
            0.2, min(0.75, self._stale_goal_service_timeout))
        self._state = PatrolGuardState(
            executor_timeout_sec=float(
                self.get_parameter("executor_timeout_sec").value),
            operator_timeout_sec=float(
                self.get_parameter("operator_timeout_sec").value),
            startup_grace_sec=float(self.get_parameter("startup_grace_sec").value),
            odom_timeout_sec=float(self.get_parameter("odom_timeout_sec").value),
            stop_linear_threshold=float(
                self.get_parameter("stop_linear_threshold").value),
            stop_angular_threshold=float(
                self.get_parameter("stop_angular_threshold").value),
            require_operator_heartbeat=bool(
                self.get_parameter("require_operator_heartbeat").value),
            require_mux_interlock=bool(
                self.get_parameter("require_mux_interlock").value),
            mux_timeout_sec=float(self.get_parameter("mux_timeout_sec").value),
            unexpected_nav_goal_grace_sec=float(
                self.get_parameter("unexpected_nav_goal_grace_sec").value),
        )
        self._last_log_key: tuple[bool, bool, str] | None = None

        self._lock_pub = self.create_publisher(Bool, "/patrol/stop_lock", 10)
        self._ready_pub = self.create_publisher(Bool, "/patrol/guard_ready", 10)
        self._status_pub = self.create_publisher(String, "/patrol/guard_status", 10)

        self.create_subscription(
            ExecutorHeartbeat, "/patrol/executor_heartbeat", self._on_executor, 10,
            callback_group=self._callback_group)
        self.create_subscription(
            OperatorHeartbeat, "/patrol/operator_heartbeat", self._on_operator, 10,
            callback_group=self._callback_group)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_profile_sensor_data,
            callback_group=self._callback_group)
        self.create_subscription(
            DiagnosticArray, str(self.get_parameter("mux_diagnostics_topic").value),
            self._on_mux_diagnostics, 10, callback_group=self._callback_group)
        self.create_subscription(
            GoalStatusArray, self._nav_action_name + "/_action/status",
            self._on_nav_goal_status, qos_profile_action_status_default,
            callback_group=self._callback_group)
        self._cancel_goal_client = self.create_client(
            CancelGoal, self._nav_action_name + "/_action/cancel_goal",
            callback_group=self._callback_group)
        self._get_result_client = self.create_client(
            NavigateToPose.Impl.GetResultService,
            self._nav_action_name + "/_action/get_result",
            callback_group=self._callback_group)
        self.create_service(
            ResetInterlock, "/patrol/reset_interlock", self._on_reset_interlock,
            callback_group=self._callback_group)

        period = max(0.02, float(self.get_parameter("publish_period_sec").value))
        self.create_timer(period, self._on_timer, callback_group=self._callback_group)
        self._publish_decision(self._evaluate(), time.monotonic(), force=True)
        # A missing action-status message is not evidence that no old goal
        # exists. Perform an explicit cancel-all reconciliation against the
        # Nav2 action server before startup may ever clear the mux lock.
        self._startup_nav_verify_thread = threading.Thread(
            target=self._startup_nav_verify_worker,
            name="patrol-guard-startup-nav-verify", daemon=True)
        self._startup_nav_verify_thread.start()
        self.get_logger().info(
            "patrol_guard started locked; verifying Nav2 goals and waiting for a fresh idle executor heartbeat")

    def _on_executor(self, message: ExecutorHeartbeat) -> None:
        with self._state_lock:
            self._state.update_executor(
                message.instance_id, message.owner_session_id, message.run_id,
                int(message.state), bool(message.active), bool(message.dispatch_allowed),
                time.monotonic(), bool(message.stop_unconfirmed),
                str(message.nav_goal_id))

    def _on_operator(self, message: OperatorHeartbeat) -> None:
        with self._state_lock:
            self._state.update_operator(
                message.session_id, message.client_id, bool(message.enabled),
                time.monotonic())

    def _on_odom(self, message: Odometry) -> None:
        with self._state_lock:
            self._state.update_odom(
                float(message.twist.twist.linear.x),
                float(message.twist.twist.angular.z), time.monotonic())

    def _on_nav_goal_status(self, message: GoalStatusArray) -> None:
        statuses = {}
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            statuses[goal_id] = int(item.status)
        with self._nav_goal_lock:
            self._nav_statuses = statuses
            self._nav_status_sequence += 1
        nonterminal_ids = {
            goal_id for goal_id, status in statuses.items()
            if status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                          GoalStatus.STATUS_CANCELING)
        }
        now = time.monotonic()
        with self._state_lock:
            self._state.update_nav_goal_status(
                len(nonterminal_ids), now, goal_ids=nonterminal_ids)
        # Publish the lock transition immediately on the status callback,
        # before waiting for the next periodic guard timer. The twist_mux lock
        # must already be active while an unauthorized goal is being checked.
        self._publish_decision(self._evaluate(), now)
        self._ensure_unauthorized_nav_goal_cancel()

    def _nonterminal_nav_goals(self) -> dict[str, int]:
        with self._nav_goal_lock:
            statuses = dict(self._nav_statuses)
        return {
            goal_id: status for goal_id, status in statuses.items()
            if status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                          GoalStatus.STATUS_CANCELING)
        }

    @staticmethod
    def _future_result(future, timeout: float, stop_event: threading.Event | None = None):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            if event.wait(min(0.05, remaining)):
                break
            if stop_event is not None and stop_event.is_set():
                return None
        try:
            return future.result()
        except Exception:  # noqa: BLE001 - service boundary, fail closed
            return None

    def _startup_nav_verify_worker(self) -> None:
        """Keep recovery locked through a quiet, repeatedly probed interval.

        cancel-all cannot cancel a goal that has been sent but not yet
        accepted by Nav2. Therefore a single empty cancel response is not
        enough. Keep probing cancel-all while holding the mux lock, cancel any
        late goal that appears, and only pass startup after a full quiet
        interval with empty probes and no non-terminal action status.
        """
        quiet_started: float | None = None
        next_probe = 0.0
        probe_period = max(0.1, min(0.5, self._startup_nav_goal_quiet / 4.0)) \
            if self._startup_nav_goal_quiet > 0.0 else 0.1
        while not self._startup_nav_verify_stop.is_set():
            now = time.monotonic()
            active = self._nonterminal_nav_goals()
            due = quiet_started is None or now >= next_probe or bool(active)
            if due:
                verified, message, goals_seen = \
                    self._cancel_and_verify_nav_goals_with_count(
                        stop_event=self._startup_nav_verify_stop,
                        response_timeout=self._startup_nav_probe_response_timeout)
                next_probe = time.monotonic() + probe_period
                if self._startup_nav_verify_stop.is_set():
                    return
                if not verified:
                    quiet_started = None
                    if message != self._startup_nav_verify_last_error:
                        self._startup_nav_verify_last_error = message
                        self.get_logger().warn(
                            f"patrol guard startup Nav2 goal check pending: {message}")
                    self._startup_nav_verify_stop.wait(0.2)
                    continue
                if quiet_started is None:
                    quiet_started = time.monotonic()
                if active or goals_seen > 0:
                    with self._state_lock:
                        self._state.note_stale_nav_goal_detected(time.monotonic())
                    quiet_started = time.monotonic()
                    self.get_logger().warn(
                        "patrol guard startup detected and cancelled a late Nav2 goal; "
                        "restarting quiet verification")

            if quiet_started is not None and \
                    time.monotonic() - quiet_started >= self._startup_nav_goal_quiet:
                # One last probe immediately before releasing the mux closes
                # the gap between the quiet timer and the readiness decision.
                verified, message, goals_seen = \
                    self._cancel_and_verify_nav_goals_with_count(
                        stop_event=self._startup_nav_verify_stop,
                        response_timeout=self._startup_nav_probe_response_timeout)
                if self._startup_nav_verify_stop.is_set():
                    return
                if verified and goals_seen == 0 and not self._nonterminal_nav_goals():
                    with self._state_lock:
                        self._state.mark_startup_nav_goal_check(True, time.monotonic())
                    self.get_logger().info(
                        f"patrol guard startup Nav2 goal check passed after quiet interval: {message}")
                    self._publish_decision(self._evaluate(), time.monotonic(), force=True)
                    return
                if not verified:
                    quiet_started = None
                    if message != self._startup_nav_verify_last_error:
                        self._startup_nav_verify_last_error = message
                        self.get_logger().warn(
                            f"patrol guard startup Nav2 goal check pending: {message}")
                else:
                    quiet_started = time.monotonic()
            self._startup_nav_verify_stop.wait(0.05)

    def _ensure_unauthorized_nav_goal_cancel(self) -> None:
        with self._state_lock:
            requested = self._state.unauthorized_nav_goal_cancel_requested
        thread = self._unauthorized_nav_cancel_thread
        if not requested or (thread is not None and thread.is_alive()):
            return
        self._unauthorized_nav_cancel_thread = threading.Thread(
            target=self._unauthorized_nav_goal_cancel_worker,
            name="patrol-guard-unauthorized-nav-cancel", daemon=True)
        self._unauthorized_nav_cancel_thread.start()

    def _unauthorized_nav_goal_cancel_worker(self) -> None:
        while not self._startup_nav_verify_stop.is_set():
            verified, message = self._cancel_and_verify_nav_goals(
                stop_event=self._startup_nav_verify_stop)
            if self._startup_nav_verify_stop.is_set():
                return
            if verified:
                self.get_logger().warn(
                    f"patrol guard cancelled an unauthorized Nav2 goal: {message}")
                return
            self.get_logger().warn(
                f"patrol guard unauthorized Nav2 goal cancellation pending: {message}")
            self._startup_nav_verify_stop.wait(0.5)

    def stop_startup_verification(self) -> None:
        self._startup_nav_verify_stop.set()
        for thread in (self._startup_nav_verify_thread,
                       self._unauthorized_nav_cancel_thread):
            if (thread is not None and thread.is_alive() and
                    thread is not threading.current_thread()):
                thread.join(timeout=3.0)

    def destroy_node(self):
        self.stop_startup_verification()
        return super().destroy_node()

    def _query_nav_goal_status(
            self, goal_id_hex: str,
            stop_event: threading.Event | None = None) -> int | None:
        if not self._get_result_client.wait_for_service(
                timeout_sec=self._stale_goal_service_timeout):
            return None
        request = NavigateToPose.Impl.GetResultService.Request()
        try:
            request.goal_id.uuid = list(bytes.fromhex(goal_id_hex))
        except ValueError:
            return None
        try:
            future = self._get_result_client.call_async(request)
        except Exception:  # noqa: BLE001 - probe failed, fail closed
            return None
        response = self._future_result(
            future, self._startup_nav_probe_response_timeout, stop_event=stop_event)
        if response is None:
            return None
        return int(response.status)

    @staticmethod
    def _goal_status_is_terminal(status: int) -> bool:
        return status in (GoalStatus.STATUS_UNKNOWN, GoalStatus.STATUS_SUCCEEDED,
                          GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)

    def _cancel_and_verify_nav_goals(
            self, stop_event: threading.Event | None = None) -> tuple[bool, str]:
        """Cancel every Nav2 goal and require observable termination."""
        verified, message, _goals_seen = \
            self._cancel_and_verify_nav_goals_with_count(stop_event=stop_event)
        return verified, message

    def _cancel_and_verify_nav_goals_with_count(
            self, stop_event: threading.Event | None = None,
            response_timeout: float | None = None) -> tuple[bool, str, int]:
        """Cancel every Nav2 goal and report how many active goals were seen.

        The replacement patrol executor's IDLE heartbeat only says that the
        new process has no in-memory goal handle. It says nothing about a goal
        still owned by Nav2 after the old process died, so the guard performs
        and verifies the cancel itself before permitting a reset.
        """
        if stop_event is None:
            acquired = self._nav_cancel_lock.acquire()
        else:
            while not self._nav_cancel_lock.acquire(timeout=0.05):
                if stop_event.is_set():
                    return False, "guard 正在退出", 0
            acquired = True
        try:
            return self._cancel_and_verify_nav_goals_locked(
                stop_event=stop_event, response_timeout=response_timeout)
        finally:
            if acquired:
                self._nav_cancel_lock.release()

    def _cancel_and_verify_nav_goals_locked(
            self, stop_event: threading.Event | None = None,
            response_timeout: float | None = None) -> tuple[bool, str, int]:
        if stop_event is not None and stop_event.is_set():
            return False, "guard 正在退出", 0
        if not self._cancel_goal_client.wait_for_service(
                timeout_sec=self._stale_goal_service_timeout):
            return False, "无法连接 Nav2 cancel_goal 服务，不能核对旧导航目标", 0
        if stop_event is not None and stop_event.is_set():
            return False, "guard 正在退出", 0

        with self._nav_goal_lock:
            before_sequence = self._nav_status_sequence
        before_active = self._nonterminal_nav_goals()
        request = CancelGoal.Request()
        # ROS2 action semantics: zero UUID and zero timestamp mean cancel all.
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        try:
            future = self._cancel_goal_client.call_async(request)
        except Exception as error:  # noqa: BLE001 - fail closed on client errors
            return False, f"发送 Nav2 全部目标取消请求失败：{error}", 0
        response = self._future_result(
            future,
            response_timeout if response_timeout is not None else max(
                self._stale_goal_service_timeout, self._stale_goal_cancel_timeout),
            stop_event=stop_event)
        if response is None:
            if stop_event is not None and stop_event.is_set():
                return False, "guard 正在退出", 0
            return False, "等待 Nav2 取消旧导航目标响应超时", 0
        if int(response.return_code) != CancelGoal.Response.ERROR_NONE:
            return False, f"Nav2 拒绝取消旧导航目标，返回码 {int(response.return_code)}", 0
        canceling_ids = {
            bytes(item.goal_id.uuid).hex() for item in response.goals_canceling
        }
        goals_seen = max(len(canceling_ids), len(before_active))

        if not canceling_ids:
            if not before_active:
                return True, "Nav2 已确认没有未终止导航目标", 0
            # A goal already in CANCELING may not be returned by cancel-all,
            # and a local status entry may additionally be stale after the
            # goal terminated. Query the authoritative result service for each
            # cached non-terminal goal before declaring the interlock clear.
            for goal_id in before_active:
                if stop_event is not None and stop_event.is_set():
                    return False, "guard 正在退出", 0
                status = self._query_nav_goal_status(goal_id, stop_event=stop_event)
                if status is None or not self._goal_status_is_terminal(status):
                    return (False, "Nav2 未返回空目标列表且无法确认旧目标已终止，联锁保持锁定",
                            len(before_active))
            return True, "Nav2 已确认缓存目标均已终止", 0

        deadline = time.monotonic() + self._stale_goal_cancel_timeout
        while True:
            with self._nav_goal_lock:
                sequence = self._nav_status_sequence
            active = self._nonterminal_nav_goals()
            if sequence > before_sequence and not active:
                return True, "Nav2 已无未终止导航目标", goals_seen
            if stop_event is not None and stop_event.is_set():
                return False, "guard 正在退出", goals_seen
            if time.monotonic() >= deadline:
                if canceling_ids:
                    return (False, "Nav2 已接受取消，但旧导航目标仍未终止，联锁保持锁定",
                            goals_seen)
                return (False, "Nav2 已接受取消，但无法确认旧导航目标的状态更新，联锁保持锁定",
                        goals_seen)
            if stop_event is not None:
                stop_event.wait(0.05)
            else:
                time.sleep(0.05)

    def _on_mux_diagnostics(self, message: DiagnosticArray) -> None:
        # twist_mux publishes one status whose values include the configured
        # lock handlers. Requiring this diagnostic prevents an already-running
        # navigation session with the default mux config from being treated as
        # interlocked merely because a guard process exists.
        for status in message.status:
            for value in status.values:
                if not value.key.startswith("lock locks.patrol_guard"):
                    continue
                if "/patrol/stop_lock" not in value.value:
                    continue
                match = re.search(
                    r"@\s*([0-9.]+)s\s+with\s+priority\s+#([0-9]+)", value.value)
                if not match:
                    continue
                timeout = float(match.group(1))
                priority = int(match.group(2))
                # A patrol lock with timeout 0 cannot detect guard death; a
                # lower priority may not mask all velocity sources.
                if timeout > 0.0 and priority >= 200:
                    with self._state_lock:
                        self._state.update_mux_interlock(time.monotonic())
                    return

    def _evaluate(self) -> GuardDecision:
        with self._state_lock:
            return self._state.evaluate(time.monotonic())

    def _on_timer(self) -> None:
        decision = self._evaluate()
        self._publish_decision(decision, time.monotonic())
        self._ensure_unauthorized_nav_goal_cancel()

    def _publish_decision(self, decision: GuardDecision, now: float,
                          force: bool = False) -> None:
        self._lock_pub.publish(Bool(data=bool(decision.locked)))
        self._ready_pub.publish(Bool(data=bool(decision.ready)))
        payload = {
            "ready": bool(decision.ready),
            "locked": bool(decision.locked),
            "fault_code": decision.fault_code,
            "reason": decision.reason,
            "startup": bool(decision.startup),
            "stamp_ns": time.time_ns(),
        }
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        key = (decision.ready, decision.locked, decision.fault_code)
        if force or key != self._last_log_key:
            self._last_log_key = key
            if decision.ready:
                self.get_logger().info(f"patrol guard ready: {decision.reason}")
            else:
                self.get_logger().warn(
                    f"patrol guard locked [{decision.fault_code}]: {decision.reason}")

    def _on_reset_interlock(self, request, response):
        del request
        # Reset is a safety decision, not merely an acknowledgement of the
        # replacement executor's heartbeat. Always ask Nav2 to cancel all and
        # confirm no non-terminal goal remains before clearing the mux lock.
        nav_verified, nav_message = self._cancel_and_verify_nav_goals()
        with self._state_lock:
            if not nav_verified:
                success, message = False, nav_message
            else:
                now = time.monotonic()
                self._state.mark_startup_nav_goal_check(True, now)
                success, message = self._state.reset(
                    now, stale_nav_goals_verified_clear=True)
            decision = self._state.evaluate(time.monotonic())
        self._publish_decision(decision, time.monotonic(), force=True)
        response.success = bool(success)
        response.message = message
        response.interlocked = bool(decision.locked)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PatrolGuard()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_startup_verification()
        executor.shutdown()
        # Best-effort explicit lock before shutdown. The mux timeout remains
        # the authoritative dead-man behavior if this process is killed.
        try:
            node._lock_pub.publish(Bool(data=True))  # noqa: SLF001
            node._ready_pub.publish(Bool(data=False))  # noqa: SLF001
        except Exception:  # noqa: BLE001 - shutdown boundary
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
