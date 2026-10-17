import math
import os
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from patrol_interfaces.action import RunRoute
from patrol_interfaces.msg import PatrolStatus, RouteStep
from patrol_interfaces.srv import (
    ControlRun, GetRuns, ListMaps, RegisterMap, SaveCurrentPoint, SetActiveMap,
    UpsertPoint, UpsertRoute,
)
from robot_patrol.engine import STATE_CANCELED, STATE_DWELLING, STATE_PAUSED
from robot_patrol.map_metadata import read_map_binding
from robot_patrol.node import PatrolNode


pytestmark = pytest.mark.skipif(
    os.environ.get("ROBOT_PATROL_INTEGRATION") != "1",
    reason="set ROBOT_PATROL_INTEGRATION=1 to run live ROS action integration",
)


class FakeNav(Node):
    def __init__(self, delay_accept=0.0, cancel_accept=True,
                 execute_duration=0.4, map_publish_limit=None):
        super().__init__("fake_nav2")
        self.goals = []
        self.cancel_calls = 0
        self.delay_accept = delay_accept
        self.cancel_accept = cancel_accept
        self.execute_duration = float(execute_duration)
        self.map_publish_limit = map_publish_limit
        self.map_publish_count = 0
        self.map_data = None
        self.guard_ready = True
        self.estop = False
        self.wrong_map = False
        self.tf_age_sec = 0.0
        self.server = ActionServer(
            self, NavigateToPose, "/navigate_to_pose", execute_callback=self.execute,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.guard_pub = self.create_publisher(Bool, "/patrol/guard_ready", 10)
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop", 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid, "/map",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_feedback)

    def publish_feedback(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        self.odom_pub.publish(odom)
        guard = Bool(); guard.data = self.guard_ready; self.guard_pub.publish(guard)
        estop = Bool(); estop.data = self.estop; self.estop_pub.publish(estop)
        if self.map_publish_limit is None or self.map_publish_count < self.map_publish_limit:
            grid = OccupancyGrid()
            grid.header.stamp = stamp
            grid.header.frame_id = "map"
            grid.info.resolution = 0.05
            grid.info.width = 20
            grid.info.height = 20
            grid.info.origin.position.x = 99.0 if self.wrong_map else -1.0
            grid.info.origin.position.y = -1.0
            grid.info.origin.orientation.w = 1.0
            grid.data = list(self.map_data) if self.map_data is not None else [0] * (20 * 20)
            self.map_pub.publish(grid)
            self.map_publish_count += 1
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.stamp.sec -= int(self.tf_age_sec)
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)

    def goal_callback(self, _goal_request):
        if self.delay_accept:
            time.sleep(self.delay_accept)
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.cancel_calls += 1
        return (rclpy.action.CancelResponse.ACCEPT if self.cancel_accept
                else rclpy.action.CancelResponse.REJECT)

    def execute(self, goal_handle):
        self.goals.append(goal_handle.request.pose)
        deadline = time.monotonic() + self.execute_duration
        while time.monotonic() < deadline:
            if self.cancel_accept and goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        goal_handle.succeed()
        return NavigateToPose.Result()


def wait_future(future, timeout=10.0):
    event = threading.Event()
    future.add_done_callback(lambda _future: event.set())
    assert event.wait(timeout), "future timeout"
    return future.result()


def make_map(tmp_path, stem="map", image_value=205):
    image = tmp_path / f"{stem}.pgm"
    image.write_bytes(b"P5\n20 20\n255\n" + bytes([image_value]) * 400)
    yaml = tmp_path / f"{stem}.yaml"
    yaml.write_text(
        f"image: {image.name}\nmode: trinary\nresolution: 0.05\n"
        "origin: [-1.0, -1.0, 0.0]\nnegate: 0\n"
        "free_thresh: 0.25\noccupied_thresh: 0.65\n", encoding="utf-8")
    return yaml


def test_real_services_and_nav2_action(tmp_path):
    rclpy.init()
    executor = None
    spin_thread = None
    patrol = None
    fake = None
    try:
        patrol = PatrolNode(parameter_overrides=[
            Parameter("database_path", value=str(tmp_path / "patrol.sqlite3")),
            Parameter("settle_timeout_sec", value=2.0),
            Parameter("settle_duration_sec", value=0.2),
            Parameter("feedback_max_age_sec", value=2.0),
            Parameter("point_timeout_sec", value=10.0),
            Parameter("min_dispatch_interval_sec", value=0.0),
        ])
        fake = FakeNav()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(patrol)
        executor.add_node(fake)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(0.8)

        map_yaml = make_map(tmp_path)
        register = patrol.create_client(RegisterMap, "/patrol/register_map")
        assert register.wait_for_service(timeout_sec=5.0)
        request = RegisterMap.Request()
        request.navigation_yaml = str(map_yaml)
        request.name = "test map"
        request.backend = "amcl"
        response = wait_future(register.call_async(request))
        assert response.success, response.message
        map_id = response.map.map_id
        map_revision = response.map.revision

        active = patrol.create_client(SetActiveMap, "/patrol/set_active_map")
        request = SetActiveMap.Request()
        request.map_id = map_id
        request.expected_revision = map_revision
        fake.wrong_map = True
        time.sleep(0.2)
        response = wait_future(active.call_async(request))
        assert not response.success and "不一致" in response.message, response.message
        fake.wrong_map = False
        time.sleep(0.2)
        response = wait_future(active.call_async(request))
        assert response.success, response.message

        save_point = patrol.create_client(SaveCurrentPoint, "/patrol/save_current_point")
        request = SaveCurrentPoint.Request()
        request.map_id = map_id
        request.name = "stale TF must fail"
        fake.tf_age_sec = 5.0
        time.sleep(1.8)
        response = wait_future(save_point.call_async(request))
        assert not response.success and "TF" in response.message, response.message
        fake.tf_age_sec = 0.0
        time.sleep(0.2)
        request = SaveCurrentPoint.Request()
        request.map_id = map_id
        request.name = "fresh current point"
        response = wait_future(save_point.call_async(request))
        assert response.success, response.message

        add_point = patrol.create_client(UpsertPoint, "/patrol/upsert_point")
        points = []
        for index, (x, y) in enumerate(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))):
            request = UpsertPoint.Request()
            request.map_id = map_id
            request.name = f"P{index}"
            request.x = x
            request.y = y
            response = wait_future(add_point.call_async(request))
            assert response.success, response.message
            points.append(response.point)

        add_route = patrol.create_client(UpsertRoute, "/patrol/upsert_route")
        request = UpsertRoute.Request()
        request.map_id = map_id
        request.name = "route"
        for point in reversed(points):
            step = RouteStep()
            step.point_id = point.point_id
            step.dwell_seconds = 0.0
            request.steps.append(step)
        response = wait_future(add_route.call_async(request))
        assert response.success, response.message
        route = response.route

        action_client = rclpy.action.ActionClient(patrol, RunRoute, "/patrol/run_route")
        assert action_client.wait_for_server(timeout_sec=5.0)
        goal = RunRoute.Goal()
        goal.request_id = "integration-1"
        goal.owner_session_id = "pytest"
        goal.route_id = route.route_id
        goal.map_id = map_id
        goal.route_revision = route.revision
        goal.map_revision = map_revision
        goal.loop_count = 1
        goal_handle = wait_future(action_client.send_goal_async(goal))
        assert goal_handle.accepted
        result = wait_future(goal_handle.get_result_async(), timeout=20.0)
        assert result.result.success, result.result.message
        assert result.result.completed_loops == 1
        assert len(fake.goals) == 3

        runs_service = patrol.create_client(GetRuns, "/patrol/get_runs")
        response = wait_future(runs_service.call_async(GetRuns.Request()))
        assert response.success and response.runs
        assert response.runs[0].success
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if patrol is not None:
            patrol.destroy_node()
        if fake is not None:
            fake.destroy_node()
        rclpy.try_shutdown()


def test_cancel_while_goal_acceptance_is_pending(tmp_path):
    rclpy.init()
    executor = None
    spin_thread = None
    patrol = None
    fake = None
    try:
        patrol = PatrolNode(parameter_overrides=[
            Parameter("database_path", value=str(tmp_path / "patrol.sqlite3")),
            Parameter("settle_timeout_sec", value=1.0),
            Parameter("settle_duration_sec", value=0.1),
            Parameter("feedback_max_age_sec", value=2.0),
            Parameter("tf_max_age_sec", value=2.0),
            Parameter("point_timeout_sec", value=5.0),
            Parameter("dwell_resume_position_tolerance_m", value=0.5),
            Parameter("dwell_resume_yaw_tolerance_rad", value=0.35),
            Parameter("min_dispatch_interval_sec", value=0.0),
        ])
        fake = FakeNav(delay_accept=0.5)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(patrol)
        executor.add_node(fake)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(0.8)

        map_yaml = make_map(tmp_path)
        data = read_map_binding(str(map_yaml), "race map", "amcl")
        mapping = patrol._store.upsert_map(data)
        p1 = patrol._store.upsert_point("", mapping.map_id, "A", 0.0, 0.0, 0.0, "manual", 1)
        p2 = patrol._store.upsert_point("", mapping.map_id, "B", 1.0, 0.0, 0.0, "manual", 2)
        route = patrol._store.upsert_route("", mapping.map_id, "race route", [
            type("S", (), {"point_id": p1.point_id, "dwell_seconds": 0.0})(),
            type("S", (), {"point_id": p2.point_id, "dwell_seconds": 0.0})(),
        ])
        with patrol._active_map_lock:
            patrol._active_map_id = mapping.map_id

        statuses = []
        status_event = threading.Event()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        patrol.create_subscription(
            PatrolStatus, "/patrol/status",
            lambda message: (statuses.append(message), status_event.set()), qos)

        action_client = rclpy.action.ActionClient(patrol, RunRoute, "/patrol/run_route")
        assert action_client.wait_for_server(timeout_sec=5.0)
        goal = RunRoute.Goal()
        goal.request_id = "cancel-race"
        goal.owner_session_id = "pytest"
        goal.route_id = route.route_id
        goal.map_id = mapping.map_id
        goal.route_revision = route.revision
        goal.map_revision = mapping.revision
        goal.loop_count = 1
        goal_handle = wait_future(action_client.send_goal_async(goal))
        assert goal_handle.accepted

        status_event.clear()
        deadline = time.monotonic() + 5.0
        run_id = ""
        while time.monotonic() < deadline and not run_id:
            for message in list(statuses):
                if message.active and message.run_id:
                    run_id = message.run_id
                    break
            if not run_id:
                status_event.wait(0.05)
                status_event.clear()
        assert run_id, "patrol status did not expose an active run_id"

        control = patrol.create_client(ControlRun, "/patrol/control")
        request = ControlRun.Request()
        request.command = ControlRun.Request.CANCEL
        request.run_id = run_id
        request.owner_session_id = "pytest"
        response = wait_future(control.call_async(request))
        assert response.accepted, response.message

        result = wait_future(goal_handle.get_result_async(), timeout=10.0)
        assert result.result.final_state == 9, result.result.message
        assert len(fake.goals) == 1, "a second point was dispatched after cancel"
        assert fake.cancel_calls >= 1, "late-accepted Nav2 goal was not cancelled"
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if patrol is not None:
            patrol.destroy_node()
        if fake is not None:
            fake.destroy_node()
        rclpy.try_shutdown()


class SlowExecutePatrolNode(PatrolNode):
    """Hold the execute callback so a cancel can arrive before validation."""

    def __init__(self, parameter_overrides=None):
        self.execute_entered = threading.Event()
        self.execute_gate = threading.Event()
        super().__init__(parameter_overrides=parameter_overrides)

    def _execute_run(self, goal_handle):
        self.execute_entered.set()
        if not self.execute_gate.wait(10.0):
            raise RuntimeError("test execute gate timed out")
        return super()._execute_run(goal_handle)


def wait_until(predicate, timeout=5.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def direct_route(patrol, tmp_path, stem="map", image_value=205, dwell=0.0,
                 point_name="A"):
    yaml = make_map(tmp_path, stem=stem, image_value=image_value)
    data = read_map_binding(str(yaml), stem, "amcl")
    mapping = patrol._store.upsert_map(data)
    point = patrol._store.upsert_point(
        "", mapping.map_id, point_name, 0.0, 0.0, 0.0, "manual", time.time_ns())
    route = patrol._store.upsert_route(
        "", mapping.map_id, f"{stem} route",
        [SimpleNamespace(point_id=point.point_id, dwell_seconds=dwell)])
    with patrol._active_map_lock:
        patrol._active_map_id = mapping.map_id
    return mapping, route


def make_run_goal(route, mapping, request_id, loop_count=1):
    goal = RunRoute.Goal()
    goal.request_id = request_id
    goal.owner_session_id = "pytest"
    goal.route_id = route.route_id
    goal.map_id = mapping.map_id
    goal.route_revision = route.revision
    goal.map_revision = mapping.revision
    goal.loop_count = loop_count
    return goal


def test_cancel_before_execute_callback_dispatches_nothing(tmp_path):
    rclpy.init()
    executor = None
    spin_thread = None
    patrol = None
    fake = None
    try:
        patrol = SlowExecutePatrolNode(parameter_overrides=[
            Parameter("database_path", value=str(tmp_path / "patrol.sqlite3")),
            Parameter("settle_timeout_sec", value=1.0),
            Parameter("settle_duration_sec", value=0.1),
            Parameter("feedback_max_age_sec", value=2.0),
            Parameter("tf_max_age_sec", value=2.0),
            Parameter("point_timeout_sec", value=5.0),
            Parameter("status_period_sec", value=0.1),
            Parameter("min_dispatch_interval_sec", value=0.0),
        ])
        fake = FakeNav()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(patrol)
        executor.add_node(fake)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(0.8)

        mapping, route = direct_route(patrol, tmp_path)
        action_client = rclpy.action.ActionClient(patrol, RunRoute, "/patrol/run_route")
        assert action_client.wait_for_server(timeout_sec=5.0)
        goal_handle = wait_future(action_client.send_goal_async(
            make_run_goal(route, mapping, "cancel-before-execute")))
        assert goal_handle.accepted
        assert patrol.execute_entered.wait(5.0), "execute callback did not start"

        cancel_future = goal_handle.cancel_goal_async()
        cancel_response = wait_future(cancel_future, timeout=5.0)
        assert cancel_response.goals_canceling, "action server did not accept cancel"
        patrol.execute_gate.set()

        result = wait_future(goal_handle.get_result_async(), timeout=10.0)
        assert result.result.final_state == STATE_CANCELED, result.result.message
        assert result.result.error_code == "CANCELED"
        assert fake.goals == [], "canceled-before-execute run still dispatched Nav2"
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if patrol is not None:
            patrol.destroy_node()
        if fake is not None:
            fake.destroy_node()
        rclpy.try_shutdown()


def test_unconfirmed_timeout_keeps_run_slot_until_child_terminates(tmp_path):
    rclpy.init()
    executor = None
    spin_thread = None
    patrol = None
    fake = None
    try:
        patrol = PatrolNode(parameter_overrides=[
            Parameter("database_path", value=str(tmp_path / "patrol.sqlite3")),
            Parameter("settle_timeout_sec", value=1.0),
            Parameter("settle_duration_sec", value=0.1),
            Parameter("feedback_max_age_sec", value=2.0),
            Parameter("tf_max_age_sec", value=2.0),
            Parameter("point_timeout_sec", value=0.4),
            Parameter("status_period_sec", value=0.1),
            Parameter("min_dispatch_interval_sec", value=0.0),
        ])
        fake = FakeNav(cancel_accept=False, execute_duration=2.0)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(patrol)
        executor.add_node(fake)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(0.8)

        mapping, route = direct_route(patrol, tmp_path)
        action_client = rclpy.action.ActionClient(patrol, RunRoute, "/patrol/run_route")
        assert action_client.wait_for_server(timeout_sec=5.0)

        first_handle = wait_future(action_client.send_goal_async(
            make_run_goal(route, mapping, "timeout-first")))
        assert first_handle.accepted
        first_result = wait_future(first_handle.get_result_async(), timeout=10.0)
        assert first_result.result.final_state == 11, first_result.result.message
        assert first_result.result.error_code == "TIMEOUT"
        assert fake.cancel_calls >= 1, "driver did not request child cancellation"

        # The child goal is still executing and Nav2 rejected cancellation.
        # The node must reject a new top-level goal instead of starting a
        # second child while the first one is still alive.
        second_handle = wait_future(action_client.send_goal_async(
            make_run_goal(route, mapping, "timeout-second")))
        assert not second_handle.accepted, "new run accepted while stop was unconfirmed"

        assert wait_until(
            lambda: not patrol._run_reserved,
            timeout=5.0,
        ), "run slot was not released after the child terminated"

        fake.execute_duration = 0.1
        third_handle = wait_future(action_client.send_goal_async(
            make_run_goal(route, mapping, "timeout-third")))
        assert third_handle.accepted, "run slot did not reopen after child termination"
        third_result = wait_future(third_handle.get_result_async(), timeout=10.0)
        assert third_result.result.success, third_result.result.message
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if patrol is not None:
            patrol.destroy_node()
        if fake is not None:
            fake.destroy_node()
        rclpy.try_shutdown()


def test_static_map_remains_valid_and_raster_content_is_checked(tmp_path):
    rclpy.init()
    executor = None
    spin_thread = None
    patrol = None
    fake = None
    try:
        patrol = PatrolNode(parameter_overrides=[
            Parameter("database_path", value=str(tmp_path / "patrol.sqlite3")),
            Parameter("settle_timeout_sec", value=1.0),
            Parameter("settle_duration_sec", value=0.1),
            Parameter("feedback_max_age_sec", value=2.0),
            Parameter("tf_max_age_sec", value=2.0),
            Parameter("map_feedback_timeout_sec", value=0.2),
            Parameter("status_period_sec", value=0.1),
            Parameter("min_dispatch_interval_sec", value=0.0),
        ])
        fake = FakeNav(map_publish_limit=1)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(patrol)
        executor.add_node(fake)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        map_a_yaml = make_map(tmp_path, stem="free_map", image_value=205)
        map_b_yaml = make_map(tmp_path, stem="occupied_map", image_value=0)
        map_a_data = read_map_binding(str(map_a_yaml), "free map", "amcl")
        map_b_data = read_map_binding(str(map_b_yaml), "occupied map", "amcl")
        map_a = patrol._store.upsert_map(map_a_data)
        map_b = patrol._store.upsert_map(map_b_data)

        # The fake map_server publishes the all-free map exactly once with
        # transient-local durability, then stays silent like a real static map.
        assert wait_until(lambda: fake.map_publish_count == 1, timeout=3.0)
        time.sleep(1.0)
        assert fake.map_publish_count == 1

        active = patrol.create_client(SetActiveMap, "/patrol/set_active_map")
        assert active.wait_for_service(timeout_sec=5.0)
        request = SetActiveMap.Request()
        request.map_id = map_a.map_id
        request.expected_revision = map_a.revision
        response = wait_future(active.call_async(request))
        assert response.success, response.message

        # Same geometry, different raster: this must no longer be accepted
        # based on width/height/origin alone.
        request = SetActiveMap.Request()
        request.map_id = map_b.map_id
        request.expected_revision = map_b.revision
        response = wait_future(active.call_async(request))
        assert not response.success, "a different occupancy raster was accepted"
        assert "栅格内容" in response.message, response.message
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if patrol is not None:
            patrol.destroy_node()
        if fake is not None:
            fake.destroy_node()
        rclpy.try_shutdown()


def test_resume_rechecks_estop_guard_and_map_before_clearing_pause(tmp_path):
    rclpy.init()
    executor = None
    spin_thread = None
    patrol = None
    fake = None
    try:
        patrol = PatrolNode(parameter_overrides=[
            Parameter("database_path", value=str(tmp_path / "patrol.sqlite3")),
            Parameter("settle_timeout_sec", value=1.0),
            Parameter("settle_duration_sec", value=0.1),
            Parameter("feedback_max_age_sec", value=2.0),
            Parameter("tf_max_age_sec", value=2.0),
            Parameter("guard_timeout_sec", value=0.5),
            Parameter("point_timeout_sec", value=5.0),
            Parameter("status_period_sec", value=0.1),
            Parameter("min_dispatch_interval_sec", value=0.0),
        ])
        fake = FakeNav(execute_duration=0.2)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(patrol)
        executor.add_node(fake)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(0.8)

        mapping, route = direct_route(patrol, tmp_path, dwell=2.0)
        statuses = []
        status_condition = threading.Condition()

        def on_status(message):
            with status_condition:
                statuses.append(message)
                status_condition.notify_all()

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        patrol.create_subscription(PatrolStatus, "/patrol/status", on_status, qos)

        def wait_status(predicate, timeout=6.0):
            deadline = time.monotonic() + timeout
            with status_condition:
                while time.monotonic() < deadline:
                    for message in reversed(statuses):
                        if predicate(message):
                            return message
                    status_condition.wait(0.05)
            return None

        action_client = rclpy.action.ActionClient(patrol, RunRoute, "/patrol/run_route")
        assert action_client.wait_for_server(timeout_sec=5.0)
        goal_handle = wait_future(action_client.send_goal_async(
            make_run_goal(route, mapping, "resume-safety")))
        assert goal_handle.accepted
        active_status = wait_status(lambda m: m.active and bool(m.run_id))
        assert active_status is not None
        run_id = active_status.run_id
        assert wait_status(lambda m: m.state == STATE_DWELLING) is not None

        control = patrol.create_client(ControlRun, "/patrol/control")
        assert control.wait_for_service(timeout_sec=5.0)
        request = ControlRun.Request()
        request.command = ControlRun.Request.PAUSE
        request.run_id = run_id
        response = wait_future(control.call_async(request))
        assert response.accepted, response.message
        assert wait_status(lambda m: m.state == STATE_PAUSED) is not None

        # ESTOP held: RESUME must not clear the paused flags.
        fake.estop = True
        time.sleep(0.3)
        request = ControlRun.Request()
        request.command = ControlRun.Request.RESUME
        request.run_id = run_id
        response = wait_future(control.call_async(request))
        assert not response.accepted, "RESUME accepted while emergency stop was held"
        assert "急停" in response.message, response.message
        engine = patrol._active_engine
        assert engine is not None and engine.flags.pause_requested

        # Guard not ready: RESUME must still be denied after ESTOP clears.
        fake.estop = False
        fake.guard_ready = False
        time.sleep(0.3)
        response = wait_future(control.call_async(request))
        assert not response.accepted, "RESUME accepted while guard was not ready"
        assert "guard" in response.message.lower(), response.message
        assert engine.flags.pause_requested

        fake.guard_ready = True
        time.sleep(0.3)
        response = wait_future(control.call_async(request))
        assert response.accepted, response.message
        result = wait_future(goal_handle.get_result_async(), timeout=12.0)
        assert result.result.success, result.result.message
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if patrol is not None:
            patrol.destroy_node()
        if fake is not None:
            fake.destroy_node()
        rclpy.try_shutdown()


def test_pause_during_unconfirmed_acceptance_is_not_resumable(tmp_path):
    rclpy.init()
    executor = None
    spin_thread = None
    patrol = None
    fake = None
    try:
        patrol = PatrolNode(parameter_overrides=[
            Parameter("database_path", value=str(tmp_path / "patrol.sqlite3")),
            Parameter("settle_timeout_sec", value=1.0),
            Parameter("settle_duration_sec", value=0.1),
            Parameter("feedback_max_age_sec", value=2.0),
            Parameter("tf_max_age_sec", value=2.0),
            Parameter("point_timeout_sec", value=0.5),
            Parameter("status_period_sec", value=0.1),
            Parameter("min_dispatch_interval_sec", value=0.0),
        ])
        # Acceptance is slower than point_timeout_sec. The child goal may be
        # accepted only after the driver has already timed out waiting.
        fake = FakeNav(delay_accept=1.5, execute_duration=0.1)
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(patrol)
        executor.add_node(fake)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(0.8)

        mapping, route = direct_route(patrol, tmp_path)
        action_client = rclpy.action.ActionClient(patrol, RunRoute, "/patrol/run_route")
        assert action_client.wait_for_server(timeout_sec=5.0)
        goal_handle = wait_future(action_client.send_goal_async(
            make_run_goal(route, mapping, "pause-unconfirmed-accept")))
        assert goal_handle.accepted

        assert wait_until(
            lambda: patrol._active_engine is not None and
            patrol._active_engine.status.get("state") == 2,
            timeout=3.0,
        ), "never reached DISPATCHING state"
        run_id = patrol._active_engine.run_id

        control = patrol.create_client(ControlRun, "/patrol/control")
        assert control.wait_for_service(timeout_sec=5.0)
        request = ControlRun.Request()
        request.command = ControlRun.Request.PAUSE
        request.run_id = run_id
        response = wait_future(control.call_async(request))
        assert response.accepted, response.message

        # The acceptance timeout occurs with pause intent set. The engine must
        # fail as stop-unconfirmed, not enter a resumable PAUSED state.
        result = wait_future(goal_handle.get_result_async(), timeout=8.0)
        assert result.result.final_state == 11, result.result.message
        assert result.result.error_code == "CANCEL_UNCONFIRMED", result.result.message
        assert patrol._last_status.get("state") != STATE_PAUSED

        # The late-accepted child goal is canceled and reaches terminal state.
        # No resume may have dispatched a second child in the meantime.
        assert wait_until(lambda: not patrol._run_reserved, timeout=5.0)
        assert len(fake.goals) == 1, "resume dispatched a duplicate Nav2 goal"
        assert fake.cancel_calls >= 1

        request = ControlRun.Request()
        request.command = ControlRun.Request.RESUME
        request.run_id = run_id
        response = wait_future(control.call_async(request))
        assert not response.accepted, "finished unconfirmed run accepted RESUME"
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if patrol is not None:
            patrol.destroy_node()
        if fake is not None:
            fake.destroy_node()
        rclpy.try_shutdown()
