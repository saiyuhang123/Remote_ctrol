import json
import os
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_action_status_default
from std_msgs.msg import Bool, String

from patrol_interfaces.msg import ExecutorHeartbeat, OperatorHeartbeat
from patrol_interfaces.srv import ResetInterlock
from robot_patrol.guard import PatrolGuard


pytestmark = pytest.mark.skipif(
    os.environ.get("ROBOT_PATROL_INTEGRATION") != "1",
    reason="set ROBOT_PATROL_INTEGRATION=1 to run live ROS integration",
)


def wait_future(future, timeout=5.0):
    event = threading.Event()
    future.add_done_callback(lambda _future: event.set())
    assert event.wait(timeout), "future timeout"
    return future.result()


class FakeNavGoals(Node):
    """Minimal real NavigateToPose action endpoint for guard reconciliation."""

    def __init__(self, honor_cancel=True, gate_accept=False):
        super().__init__("fake_guard_nav2")
        self.honor_cancel = bool(honor_cancel)
        self.cancel_calls = 0
        self.canceled_goals = 0
        self.goals = []
        self.release_goals = threading.Event()
        self.goal_request_received = threading.Event()
        self.accept_gate = threading.Event()
        if not gate_accept:
            self.accept_gate.set()
        group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self, NavigateToPose, "/navigate_to_pose", execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel, callback_group=group)

    def goal_callback(self, _goal_request):
        self.goal_request_received.set()
        if not self.accept_gate.wait(timeout=10.0):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel(self, _goal_handle):
        self.cancel_calls += 1
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        self.goals.append(goal_handle)
        while rclpy.ok() and not self.release_goals.is_set():
            if self.honor_cancel and goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.canceled_goals += 1
                return NavigateToPose.Result()
            time.sleep(0.02)
        goal_handle.abort()
        return NavigateToPose.Result()


class FakeLeases(Node):
    def __init__(self, diagnostics_topic="/patrol/test/guard_diagnostics"):
        super().__init__("fake_patrol_leases")
        self.diagnostics_topic = diagnostics_topic
        self.instance_id = "executor-test"
        self.publish_executor = True
        self.publish_operator = False
        self.publish_diagnostics = True
        self.stop_unconfirmed = False
        self.nav_goal_id = ""
        self.executor_active = False
        self.owner = "operator-a"
        self.run_id = ""
        self.state = 0
        self.last_twist = Twist()
        self._executor_pub = self.create_publisher(
            ExecutorHeartbeat, "/patrol/executor_heartbeat", 10)
        self._operator_pub = self.create_publisher(
            OperatorHeartbeat, "/patrol/operator_heartbeat", 10)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10)
        self.create_timer(0.05, self.publish)

    def publish(self):
        if self.publish_executor:
            heartbeat = ExecutorHeartbeat()
            heartbeat.instance_id = self.instance_id
            heartbeat.owner_session_id = self.owner
            heartbeat.run_id = self.run_id if self.executor_active else ""
            heartbeat.state = self.state if self.executor_active else 0
            heartbeat.active = bool(self.executor_active)
            heartbeat.dispatch_allowed = bool(self.executor_active)
            heartbeat.stop_unconfirmed = bool(self.stop_unconfirmed)
            heartbeat.nav_goal_id = str(self.nav_goal_id)
            heartbeat.stamp = self.get_clock().now().to_msg()
            self._executor_pub.publish(heartbeat)
        if self.publish_operator:
            heartbeat = OperatorHeartbeat()
            heartbeat.session_id = self.owner
            heartbeat.client_id = "fake_ui"
            heartbeat.enabled = True
            heartbeat.stamp = self.get_clock().now().to_msg()
            self._operator_pub.publish(heartbeat)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.twist.twist.linear.x = float(self.last_twist.linear.x)
        odom.twist.twist.angular.z = float(self.last_twist.angular.z)
        self._odom_pub.publish(odom)
        if self.publish_diagnostics:
            diagnostics = DiagnosticArray()
            diagnostics.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "twist_mux: Twist mux status"
            status.message = "ok"
            status.values.append(KeyValue(
                key="lock locks.patrol_guard",
                value="free (listening to /patrol/stop_lock @ 0.500000s with priority #255)"))
            diagnostics.status.append(status)
            self._diagnostics_pub.publish(diagnostics)

    def set_active(self, active):
        self.executor_active = bool(active)
        self.run_id = "run-test" if active else ""
        self.state = 3 if active else 0


def test_guard_latches_operator_loss_and_requires_explicit_reset(tmp_path):
    del tmp_path
    rclpy.init()
    executor = None
    spin_thread = None
    guard = None
    leases = None
    nav_server = None
    try:
        diagnostics_topic = f"/patrol/test/p{os.getpid()}_guard_diagnostics"
        guard = PatrolGuard(parameter_overrides=[
            Parameter("publish_period_sec", value=0.05),
            Parameter("executor_timeout_sec", value=0.5),
            Parameter("operator_timeout_sec", value=0.5),
            Parameter("startup_grace_sec", value=0.15),
            Parameter("odom_timeout_sec", value=0.5),
            Parameter("stop_linear_threshold", value=0.03),
            Parameter("stop_angular_threshold", value=0.05),
            Parameter("require_operator_heartbeat", value=True),
            Parameter("mux_timeout_sec", value=0.3),
            Parameter("mux_diagnostics_topic", value=diagnostics_topic),
            Parameter("startup_nav_goal_quiet_sec", value=0.2),
        ])
        leases = FakeLeases(diagnostics_topic)
        nav_server = FakeNavGoals()
        leases.publish_diagnostics = False
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(guard)
        executor.add_node(leases)
        executor.add_node(nav_server)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        ready = {"value": False}
        locked = {"value": True}
        guard_status = {"data": ""}
        guard.create_subscription(Bool, "/patrol/guard_ready",
                                  lambda m: ready.__setitem__("value", bool(m.data)), 10)
        guard.create_subscription(Bool, "/patrol/stop_lock",
                                  lambda m: locked.__setitem__("value", bool(m.data)), 10)
        guard.create_subscription(String, "/patrol/guard_status",
                                  lambda m: guard_status.__setitem__("data", str(m.data)), 10)

        def fault_code():
            try:
                return json.loads(guard_status["data"]).get("fault_code", "")
            except Exception:
                return ""

        def wait_until(predicate, timeout=5.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return True
                time.sleep(0.02)
            return False

        # An executor idle heartbeat without twist_mux patrol-lock diagnostics
        # is not enough to consider the navigation session interlocked.
        time.sleep(0.4)
        assert locked["value"] and not ready["value"]
        leases.publish_diagnostics = True
        assert wait_until(lambda: ready["value"] and not locked["value"], timeout=3.0)

        leases.publish_operator = True
        leases.set_active(True)
        assert wait_until(lambda: ready["value"] and not locked["value"], timeout=2.0)

        leases.publish_operator = False
        assert wait_until(lambda: not ready["value"] and locked["value"], timeout=3.0)
        assert fault_code() == "OPERATOR_LOST"

        leases.publish_operator = True
        time.sleep(0.3)
        assert locked["value"], "operator reconnect must not auto-clear a latched fault"

        leases.set_active(False)
        leases.last_twist.linear.x = 0.0
        leases.last_twist.angular.z = 0.0
        time.sleep(0.3)
        reset = guard.create_client(ResetInterlock, "/patrol/reset_interlock")
        assert reset.wait_for_service(timeout_sec=3.0)
        response = wait_future(reset.call_async(ResetInterlock.Request()))
        assert response.success, response.message
        assert not response.interlocked
        assert wait_until(lambda: ready["value"] and not locked["value"], timeout=2.0)

        # A child goal whose cancellation cannot be confirmed must lock
        # immediately, while the executor is still alive.
        leases.set_active(True)
        assert wait_until(lambda: ready["value"] and not locked["value"], timeout=2.0)
        leases.stop_unconfirmed = True
        assert wait_until(lambda: not ready["value"] and locked["value"], timeout=2.0)
        assert fault_code() == "STOP_UNCONFIRMED"

        response = wait_future(reset.call_async(ResetInterlock.Request()))
        assert not response.success

        # Losing the executor, not just the operator, is also a latched stop.
        leases.stop_unconfirmed = False
        leases.set_active(False)
        time.sleep(0.3)
        response = wait_future(reset.call_async(ResetInterlock.Request()))
        assert response.success, response.message
        leases.set_active(True)
        assert wait_until(lambda: ready["value"] and not locked["value"], timeout=2.0)
        leases.publish_executor = False
        assert wait_until(lambda: not ready["value"] and locked["value"], timeout=3.0)
        assert fault_code() == "EXECUTOR_LOST"
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if guard is not None:
            guard.destroy_node()
        if leases is not None:
            leases.destroy_node()
        if nav_server is not None:
            nav_server.destroy_node()
        rclpy.try_shutdown()


def test_reset_cancels_stale_nav2_goal_before_unlocking(tmp_path):
    del tmp_path
    rclpy.init()
    executor = None
    spin_thread = None
    guard = None
    leases = None
    nav_server = None
    action_client = None
    try:
        diagnostics_topic = f"/patrol/test/p{os.getpid()}_stale_goal_diagnostics"
        guard = PatrolGuard(parameter_overrides=[
            Parameter("publish_period_sec", value=0.05),
            Parameter("executor_timeout_sec", value=0.5),
            Parameter("operator_timeout_sec", value=0.5),
            Parameter("startup_grace_sec", value=0.15),
            Parameter("odom_timeout_sec", value=0.5),
            Parameter("mux_timeout_sec", value=0.5),
            Parameter("mux_diagnostics_topic", value=diagnostics_topic),
            Parameter("stale_goal_cancel_timeout_sec", value=2.0),
            Parameter("stale_goal_service_timeout_sec", value=2.0),
            Parameter("startup_nav_goal_quiet_sec", value=0.2),
        ])
        leases = FakeLeases(diagnostics_topic)
        leases.publish_operator = True
        leases.owner = "operator-a"
        leases.instance_id = "executor-old"
        nav_server = FakeNavGoals()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(guard)
        executor.add_node(leases)
        executor.add_node(nav_server)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        action_client = ActionClient(leases, NavigateToPose, "/navigate_to_pose")
        assert action_client.wait_for_server(timeout_sec=3.0)
        send_future = action_client.send_goal_async(NavigateToPose.Goal())
        goal_handle = wait_future(send_future, timeout=3.0)
        assert goal_handle.accepted
        deadline = time.monotonic() + 3.0
        while not nav_server.goals and time.monotonic() < deadline:
            time.sleep(0.02)
        assert nav_server.goals, "fake Nav2 did not start the old goal"

        leases.set_active(True)
        time.sleep(0.25)
        # The replacement process is genuinely idle, but the old NavigateToPose
        # goal is still executing at Nav2.
        leases.publish_executor = False
        time.sleep(0.8)
        leases.instance_id = "executor-new"
        leases.set_active(False)
        leases.publish_executor = True
        time.sleep(0.35)

        reset = guard.create_client(ResetInterlock, "/patrol/reset_interlock")
        assert reset.wait_for_service(timeout_sec=3.0)
        response = wait_future(reset.call_async(ResetInterlock.Request()), timeout=6.0)
        assert response.success, response.message
        assert not response.interlocked
        assert nav_server.cancel_calls >= 1
        assert nav_server.canceled_goals >= 1
    finally:
        if action_client is not None:
            action_client.destroy()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if guard is not None:
            guard.destroy_node()
        if leases is not None:
            leases.destroy_node()
        if nav_server is not None:
            nav_server.destroy_node()
        rclpy.try_shutdown()


def test_reset_stays_locked_when_stale_nav2_goal_does_not_stop(tmp_path):
    del tmp_path
    rclpy.init()
    executor = None
    spin_thread = None
    guard = None
    leases = None
    nav_server = None
    action_client = None
    try:
        diagnostics_topic = f"/patrol/test/p{os.getpid()}_ignored_cancel_diagnostics"
        guard = PatrolGuard(parameter_overrides=[
            Parameter("publish_period_sec", value=0.05),
            Parameter("executor_timeout_sec", value=0.5),
            Parameter("operator_timeout_sec", value=0.5),
            Parameter("startup_grace_sec", value=0.15),
            Parameter("odom_timeout_sec", value=0.5),
            Parameter("mux_timeout_sec", value=0.5),
            Parameter("mux_diagnostics_topic", value=diagnostics_topic),
            Parameter("stale_goal_cancel_timeout_sec", value=0.5),
            Parameter("stale_goal_service_timeout_sec", value=1.5),
            Parameter("startup_nav_goal_quiet_sec", value=0.2),
        ])
        leases = FakeLeases(diagnostics_topic)
        leases.publish_operator = True
        leases.instance_id = "executor-old"
        nav_server = FakeNavGoals(honor_cancel=False)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(guard)
        executor.add_node(leases)
        executor.add_node(nav_server)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        action_client = ActionClient(leases, NavigateToPose, "/navigate_to_pose")
        assert action_client.wait_for_server(timeout_sec=3.0)
        goal_handle = wait_future(action_client.send_goal_async(NavigateToPose.Goal()), timeout=3.0)
        assert goal_handle.accepted
        deadline = time.monotonic() + 3.0
        while not nav_server.goals and time.monotonic() < deadline:
            time.sleep(0.02)
        assert nav_server.goals

        leases.set_active(True)
        time.sleep(0.25)
        leases.publish_executor = False
        time.sleep(0.8)
        leases.instance_id = "executor-new"
        leases.set_active(False)
        leases.publish_executor = True
        time.sleep(0.35)

        locked = {"value": False}
        guard.create_subscription(
            Bool, "/patrol/stop_lock", lambda msg: locked.__setitem__("value", bool(msg.data)), 10)
        reset = guard.create_client(ResetInterlock, "/patrol/reset_interlock")
        assert reset.wait_for_service(timeout_sec=3.0)
        response = wait_future(reset.call_async(ResetInterlock.Request()), timeout=6.0)
        assert not response.success
        assert ("未终止" in response.message or "状态更新" in response.message or
                "无法确认旧目标" in response.message)
        deadline = time.monotonic() + 2.0
        while not locked["value"] and time.monotonic() < deadline:
            time.sleep(0.02)
        assert locked["value"], "guard must remain locked while the old Nav2 goal executes"
        assert nav_server.cancel_calls >= 1
    finally:
        if nav_server is not None:
            nav_server.release_goals.set()
            time.sleep(0.1)
        if action_client is not None:
            action_client.destroy()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if guard is not None:
            guard.destroy_node()
        if leases is not None:
            leases.destroy_node()
        if nav_server is not None:
            nav_server.destroy_node()
        rclpy.try_shutdown()


def test_guard_does_not_start_without_explicit_nav2_goal_check(tmp_path):
    del tmp_path
    rclpy.init()
    executor = None
    spin_thread = None
    guard = None
    leases = None
    try:
        diagnostics_topic = f"/patrol/test/p{os.getpid()}_missing_nav_status_diag"
        guard = PatrolGuard(parameter_overrides=[
            Parameter("publish_period_sec", value=0.05),
            Parameter("executor_timeout_sec", value=0.5),
            Parameter("startup_grace_sec", value=0.1),
            Parameter("mux_timeout_sec", value=0.5),
            Parameter("mux_diagnostics_topic", value=diagnostics_topic),
            Parameter("stale_goal_cancel_timeout_sec", value=0.4),
            Parameter("stale_goal_service_timeout_sec", value=0.4),
            Parameter("startup_nav_goal_quiet_sec", value=0.2),
        ])
        leases = FakeLeases(diagnostics_topic)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(guard)
        executor.add_node(leases)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        ready = {"value": False}
        locked = {"value": True}
        guard.create_subscription(
            Bool, "/patrol/guard_ready", lambda m: ready.__setitem__("value", bool(m.data)), 10)
        guard.create_subscription(
            Bool, "/patrol/stop_lock", lambda m: locked.__setitem__("value", bool(m.data)), 10)
        # There is deliberately no NavigateToPose action server/status message.
        # The startup grace must expire without releasing the mux.
        time.sleep(0.9)
        assert not ready["value"]
        assert locked["value"]
        with guard._state_lock:  # noqa: SLF001 - test observes safety state
            assert not guard._state.startup_nav_goal_check_passed

        # A delayed action-status message revealing an old executing goal must
        # latch a stale-goal fault, not be ignored because startup grace ended.
        status_pub = leases.create_publisher(
            GoalStatusArray, "/navigate_to_pose/_action/status",
            qos_profile_action_status_default)
        status_array = GoalStatusArray()
        item = GoalStatus()
        item.goal_info.goal_id.uuid = [7] * 16
        item.status = GoalStatus.STATUS_EXECUTING
        status_array.status_list.append(item)
        status_pub.publish(status_array)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with guard._state_lock:  # noqa: SLF001
                if guard._state.fault_code == "STALE_NAV_GOAL":
                    break
            time.sleep(0.02)
        with guard._state_lock:  # noqa: SLF001
            assert guard._state.fault_code == "STALE_NAV_GOAL"
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and (
                not locked["value"] or ready["value"]):
            time.sleep(0.02)
        assert locked["value"] and not ready["value"]
    finally:
        if guard is not None:
            guard.stop_startup_verification()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if guard is not None:
            guard.destroy_node()
        if leases is not None:
            leases.destroy_node()
        rclpy.try_shutdown()


def test_late_accepted_stale_goal_across_guard_restart_is_cancelled(tmp_path):
    del tmp_path
    rclpy.init()
    nav_executor = None
    guard_executor = None
    nav_spin = None
    guard_spin = None
    guard = None
    leases = None
    nav_server = None
    action_client = None
    try:
        diagnostics_topic = f"/patrol/test/p{os.getpid()}_late_accept_diag"
        leases = FakeLeases(diagnostics_topic)
        nav_server = FakeNavGoals(gate_accept=True)
        nav_executor = MultiThreadedExecutor(num_threads=4)
        nav_executor.add_node(leases)
        nav_executor.add_node(nav_server)
        nav_spin = threading.Thread(target=nav_executor.spin, daemon=True)
        nav_spin.start()

        # Queue the old request while Nav2 is still withholding acceptance.
        action_client = ActionClient(leases, NavigateToPose, "/navigate_to_pose")
        assert action_client.wait_for_server(timeout_sec=3.0)
        old_send = action_client.send_goal_async(NavigateToPose.Goal())
        assert nav_server.goal_request_received.wait(timeout=3.0), \
            "fake Nav2 never received the old goal request"

        guard = PatrolGuard(parameter_overrides=[
            Parameter("publish_period_sec", value=0.05),
            Parameter("executor_timeout_sec", value=0.5),
            Parameter("startup_grace_sec", value=0.1),
            Parameter("mux_timeout_sec", value=0.5),
            Parameter("mux_diagnostics_topic", value=diagnostics_topic),
            Parameter("stale_goal_cancel_timeout_sec", value=1.0),
            Parameter("stale_goal_service_timeout_sec", value=1.5),
            Parameter("startup_nav_goal_quiet_sec", value=1.0),
        ])
        guard_executor = MultiThreadedExecutor(num_threads=4)
        guard_executor.add_node(guard)
        guard_spin = threading.Thread(target=guard_executor.spin, daemon=True)
        guard_spin.start()

        ready = {"value": False}
        locked = {"value": True}
        guard.create_subscription(
            Bool, "/patrol/guard_ready", lambda m: ready.__setitem__("value", bool(m.data)), 10)
        guard.create_subscription(
            Bool, "/patrol/stop_lock", lambda m: locked.__setitem__("value", bool(m.data)), 10)

        # The initial cancel-all cannot see the still-unaccepted request. The
        # guard must nevertheless stay in its quiet verification loop.
        time.sleep(0.5)
        with guard._state_lock:  # noqa: SLF001
            assert not guard._state.startup_nav_goal_check_passed
        assert not ready["value"] and locked["value"]

        # Nav2 finally accepts the old request while the guard is still in
        # recovery. The late goal must be seen, cancelled and kept interlocked.
        nav_server.accept_gate.set()
        goal_handle = wait_future(old_send, timeout=3.0)
        assert goal_handle.accepted
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and nav_server.canceled_goals < 1:
            time.sleep(0.02)
        assert nav_server.cancel_calls >= 1
        assert nav_server.canceled_goals >= 1
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with guard._state_lock:  # noqa: SLF001
                fault_code = guard._state.fault_code
            if fault_code == "STALE_NAV_GOAL":
                break
            time.sleep(0.02)
        assert fault_code == "STALE_NAV_GOAL"
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and (
                not locked["value"] or ready["value"]):
            time.sleep(0.02)
        assert locked["value"] and not ready["value"]
    finally:
        if nav_server is not None:
            nav_server.accept_gate.set()
            nav_server.release_goals.set()
            time.sleep(0.1)
        if action_client is not None:
            action_client.destroy()
        if guard is not None:
            guard.stop_startup_verification()
        if guard_executor is not None:
            guard_executor.shutdown()
        if nav_executor is not None:
            nav_executor.shutdown()
        if guard_spin is not None:
            guard_spin.join(timeout=3.0)
        if nav_spin is not None:
            nav_spin.join(timeout=3.0)
        if guard is not None:
            guard.destroy_node()
        if leases is not None:
            leases.destroy_node()
        if nav_server is not None:
            nav_server.destroy_node()
        rclpy.try_shutdown()


def test_late_accepted_stale_goal_after_startup_ready_is_cancelled(tmp_path):
    del tmp_path
    rclpy.init()
    nav_executor = None
    guard_executor = None
    nav_spin = None
    guard_spin = None
    guard = None
    leases = None
    nav_server = None
    action_client = None
    try:
        diagnostics_topic = f"/patrol/test/p{os.getpid()}_late_after_ready_diag"
        leases = FakeLeases(diagnostics_topic)
        nav_server = FakeNavGoals(gate_accept=True)
        nav_executor = MultiThreadedExecutor(num_threads=4)
        nav_executor.add_node(leases)
        nav_executor.add_node(nav_server)
        nav_spin = threading.Thread(target=nav_executor.spin, daemon=True)
        nav_spin.start()

        action_client = ActionClient(leases, NavigateToPose, "/navigate_to_pose")
        assert action_client.wait_for_server(timeout_sec=3.0)
        old_send = action_client.send_goal_async(NavigateToPose.Goal())
        assert nav_server.goal_request_received.wait(timeout=3.0)

        guard = PatrolGuard(parameter_overrides=[
            Parameter("publish_period_sec", value=0.05),
            Parameter("executor_timeout_sec", value=0.5),
            Parameter("startup_grace_sec", value=0.1),
            Parameter("mux_timeout_sec", value=0.5),
            Parameter("mux_diagnostics_topic", value=diagnostics_topic),
            Parameter("stale_goal_cancel_timeout_sec", value=1.0),
            Parameter("stale_goal_service_timeout_sec", value=1.5),
            Parameter("startup_nav_goal_quiet_sec", value=0.3),
            Parameter("unexpected_nav_goal_grace_sec", value=0.4),
        ])
        guard_executor = MultiThreadedExecutor(num_threads=4)
        guard_executor.add_node(guard)
        guard_spin = threading.Thread(target=guard_executor.spin, daemon=True)
        guard_spin.start()

        ready = {"value": False}
        locked = {"value": True}
        guard.create_subscription(
            Bool, "/patrol/guard_ready", lambda m: ready.__setitem__("value", bool(m.data)), 10)
        guard.create_subscription(
            Bool, "/patrol/stop_lock", lambda m: locked.__setitem__("value", bool(m.data)), 10)

        # Let startup verification finish and the guard become ready while the
        # old request is still pending acceptance at Nav2.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with guard._state_lock:  # noqa: SLF001
                checked = guard._state.startup_nav_goal_check_passed
            if checked and ready["value"] and not locked["value"]:
                break
            time.sleep(0.02)
        with guard._state_lock:  # noqa: SLF001
            assert guard._state.startup_nav_goal_check_passed
        assert ready["value"] and not locked["value"]

        # The old pending goal is accepted only after the quiet window has
        # completed and the guard has already released the mux. Continuous
        # ownership monitoring must relock and cancel it.
        nav_server.accept_gate.set()
        goal_handle = wait_future(old_send, timeout=3.0)
        assert goal_handle.accepted
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            with guard._state_lock:  # noqa: SLF001
                fault_code = guard._state.fault_code
            if (fault_code == "UNAUTHORIZED_NAV_GOAL" and
                    nav_server.canceled_goals >= 1 and locked["value"]):
                break
            time.sleep(0.02)
        with guard._state_lock:  # noqa: SLF001
            assert guard._state.fault_code == "UNAUTHORIZED_NAV_GOAL"
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and nav_server.canceled_goals < 1:
            time.sleep(0.02)
        assert nav_server.cancel_calls >= 1
        assert nav_server.canceled_goals >= 1
        assert locked["value"] and not ready["value"]
    finally:
        if nav_server is not None:
            nav_server.accept_gate.set()
            nav_server.release_goals.set()
            time.sleep(0.1)
        if action_client is not None:
            action_client.destroy()
        if guard is not None:
            guard.stop_startup_verification()
        if guard_executor is not None:
            guard_executor.shutdown()
        if nav_executor is not None:
            nav_executor.shutdown()
        if guard_spin is not None:
            guard_spin.join(timeout=3.0)
        if nav_spin is not None:
            nav_spin.join(timeout=3.0)
        if guard is not None:
            guard.destroy_node()
        if leases is not None:
            leases.destroy_node()
        if nav_server is not None:
            nav_server.destroy_node()
        rclpy.try_shutdown()


def test_paused_patrol_does_not_authorize_late_stale_nav_goal(tmp_path):
    del tmp_path
    rclpy.init()
    nav_executor = None
    guard_executor = None
    nav_spin = None
    guard_spin = None
    guard = None
    leases = None
    nav_server = None
    action_client = None
    try:
        diagnostics_topic = f"/patrol/test/p{os.getpid()}_paused_owner_diag"
        leases = FakeLeases(diagnostics_topic)
        nav_server = FakeNavGoals(gate_accept=True)
        nav_executor = MultiThreadedExecutor(num_threads=4)
        nav_executor.add_node(leases)
        nav_executor.add_node(nav_server)
        nav_spin = threading.Thread(target=nav_executor.spin, daemon=True)
        nav_spin.start()

        action_client = ActionClient(leases, NavigateToPose, "/navigate_to_pose")
        assert action_client.wait_for_server(timeout_sec=3.0)
        old_send = action_client.send_goal_async(NavigateToPose.Goal())
        assert nav_server.goal_request_received.wait(timeout=3.0)

        guard = PatrolGuard(parameter_overrides=[
            Parameter("publish_period_sec", value=0.05),
            Parameter("executor_timeout_sec", value=0.5),
            Parameter("operator_timeout_sec", value=0.5),
            Parameter("startup_grace_sec", value=0.1),
            Parameter("mux_timeout_sec", value=0.5),
            Parameter("mux_diagnostics_topic", value=diagnostics_topic),
            Parameter("stale_goal_cancel_timeout_sec", value=1.0),
            Parameter("stale_goal_service_timeout_sec", value=1.5),
            Parameter("startup_nav_goal_quiet_sec", value=0.3),
            Parameter("unexpected_nav_goal_grace_sec", value=0.3),
        ])
        guard_executor = MultiThreadedExecutor(num_threads=4)
        guard_executor.add_node(guard)
        guard_spin = threading.Thread(target=guard_executor.spin, daemon=True)
        guard_spin.start()

        ready = {"value": False}
        locked = {"value": True}
        guard.create_subscription(
            Bool, "/patrol/guard_ready", lambda m: ready.__setitem__("value", bool(m.data)), 10)
        guard.create_subscription(
            Bool, "/patrol/stop_lock", lambda m: locked.__setitem__("value", bool(m.data)), 10)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not ready["value"]:
            time.sleep(0.02)
        assert ready["value"]

        # The new patrol task is active but PAUSED and owns no Nav2 child goal.
        leases.publish_operator = True
        leases.owner = "paused-owner"
        leases.executor_active = True
        leases.run_id = "run-paused"
        leases.state = 7
        leases.nav_goal_id = ""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with guard._state_lock:  # noqa: SLF001
                if guard._state.ready and not guard._state.locked:
                    break
            time.sleep(0.02)
        with guard._state_lock:  # noqa: SLF001
            assert guard._state.ready and not guard._state.locked

        # The old pending goal is accepted now. It is not the paused task's
        # child, so it must lock the mux immediately and then be cancelled.
        nav_server.accept_gate.set()
        goal_handle = wait_future(old_send, timeout=3.0)
        assert goal_handle.accepted
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            with guard._state_lock:  # noqa: SLF001
                fault_code = guard._state.fault_code
            if fault_code == "UNAUTHORIZED_NAV_GOAL" and locked["value"]:
                break
            time.sleep(0.02)
        with guard._state_lock:  # noqa: SLF001
            assert guard._state.fault_code == "UNAUTHORIZED_NAV_GOAL"
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and nav_server.canceled_goals < 1:
            time.sleep(0.02)
        assert nav_server.cancel_calls >= 1
        assert nav_server.canceled_goals >= 1
        assert locked["value"] and not ready["value"]
    finally:
        if nav_server is not None:
            nav_server.accept_gate.set()
            nav_server.release_goals.set()
            time.sleep(0.1)
        if action_client is not None:
            action_client.destroy()
        if guard is not None:
            guard.stop_startup_verification()
        if guard_executor is not None:
            guard_executor.shutdown()
        if nav_executor is not None:
            nav_executor.shutdown()
        if guard_spin is not None:
            guard_spin.join(timeout=3.0)
        if nav_spin is not None:
            nav_spin.join(timeout=3.0)
        if guard is not None:
            guard.destroy_node()
        if leases is not None:
            leases.destroy_node()
        if nav_server is not None:
            nav_server.destroy_node()
        rclpy.try_shutdown()
