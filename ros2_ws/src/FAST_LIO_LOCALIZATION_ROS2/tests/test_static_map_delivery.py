"""Static-map delivery with late/restarted consumers; no physical devices."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
PROJECT = PACKAGE.parents[2]
SCRIPT = PACKAGE/'scripts/global_map_publisher.py'
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    # Isolate this fixture even when collected alongside runtime tests that
    # select another domain. Restore the caller's environment afterward.
    environment = {'ROS_DOMAIN_ID': '201', 'ROS_LOCALHOST_ONLY': '1'}
    previous = {key: os.environ.get(key) for key in environment}
    os.environ.update(environment)
    try:
        rclpy.init(args=[])
        yield
    finally:
        rclpy.try_shutdown()
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


@pytest.fixture
def rig(tmp_path):
    node = rclpy.create_node('static_map_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    processes, logs, extra_nodes = [], [], []
    xs, ys = np.meshgrid(np.arange(7)*.25, np.arange(7)*.25)
    points = np.column_stack((xs.ravel(), ys.ravel(), np.zeros(xs.size))).astype(np.float32)

    def write_map(name, offset=0.):
        expected = points + np.float32(offset)
        path = tmp_path/name
        header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {len(expected)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                  f'POINTS {len(expected)}\nDATA ascii\n')
        path.write_text(header+'\n'.join(' '.join(map(str, row)) for row in expected)+'\n')
        return path, expected

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)

    def start(path, *parameters):
        log_path = tmp_path/f'publisher_{len(processes)}.log'
        stream = log_path.open('w+')
        logs.append(stream)
        arguments = [sys.executable, str(SCRIPT), '--ros-args',
                     '-p', 'map_file_path:='+str(path), '-p', 'interval:=1']
        for parameter in parameters:
            arguments += ['-p', parameter]
        process = subprocess.Popen(arguments, stdout=stream, stderr=subprocess.STDOUT)
        processes.append(process)
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            text = log_path.read_text(errors='replace')
            assert process.poll() is None, text
            if 'Map Publisher Node Initialized' in text:
                return process
            time.sleep(.05)
        raise AssertionError(log_path.read_text(errors='replace'))

    def spin_until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        assert predicate()

    def spin_for(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)

    resources = dict(node=node, executor=executor, write_map=write_map, start=start,
                     stop=stop, spin_until=spin_until, spin_for=spin_for,
                     extra_nodes=extra_nodes)
    try:
        yield resources
    finally:
        for extra in extra_nodes:
            executor.remove_node(extra)
            extra.destroy_node()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown(timeout_sec=1.)
        for process in reversed(processes):
            stop(process)
        for stream in logs:
            stream.close()


def assert_map(message, expected):
    assert message.header.frame_id == 'map'
    assert message.header.stamp.sec == message.header.stamp.nanosec == 0
    assert message.width == len(expected) and message.height == 1
    assert message.point_step == 12 and message.row_step == len(expected)*12
    assert [f.name for f in message.fields] == ['x', 'y', 'z']
    actual = np.frombuffer(message.data, dtype='<f4').reshape(-1, 3)
    order = lambda values: values[np.lexsort((values[:, 2], values[:, 1], values[:, 0]))]
    np.testing.assert_allclose(order(actual), order(expected), atol=1e-6)


def test_one_publish_reaches_late_and_recreated_display_subscriptions(rig):
    path, expected = rig['write_map']('map.pcd')
    rig['start'](path)  # Publisher completes before any subscriber exists.
    received = []
    subscription = rig['node'].create_subscription(PointCloud2, '/global_map', received.append, MAP_QOS)
    rig['spin_until'](lambda: bool(received))
    assert_map(received[0], expected)
    rig['spin_for'](2.5)  # interval=1 must not cause periodic publications.
    assert len(received) == 1
    rig['node'].destroy_subscription(subscription)
    reopened = []
    rig['node'].create_subscription(PointCloud2, '/global_map', reopened.append, MAP_QOS)
    rig['spin_until'](lambda: bool(reopened))
    assert_map(reopened[0], expected)
    rig['spin_for'](1.2)
    assert len(reopened) == 1


def test_localization_loads_cache_when_started_late_and_after_restart(rig):
    path, expected = rig['write_map']('map.pcd')
    rig['start'](path)
    spec = importlib.util.spec_from_file_location('static_map_localization',
                                                PACKAGE/'scripts/global_localization.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for _ in range(2):
        localization = module.GlobalLocalizationNode()
        rig['extra_nodes'].append(localization)
        rig['executor'].add_node(localization)
        rig['spin_until'](lambda: localization.global_map is not None, timeout=15)
        assert len(localization.map_points) == len(expected)
        assert np.isfinite(localization.map_normals).all()
        assert not any(sub.topic_name == '/global_map' for sub in localization.subscriptions)
        rig['executor'].remove_node(localization)
        rig['extra_nodes'].remove(localization)
        localization.destroy_node()


def test_switching_map_publishes_the_new_map_for_existing_and_late_consumers(rig):
    first, expected_first = rig['write_map']('first.pcd')
    second, expected_second = rig['write_map']('second.pcd', offset=10.)
    received = []
    rig['node'].create_subscription(PointCloud2, '/global_map', received.append, MAP_QOS)
    process = rig['start'](first)
    rig['spin_until'](lambda: bool(received))
    assert_map(received[-1], expected_first)
    rig['stop'](process)
    received.clear()
    rig['start'](second)
    rig['spin_until'](lambda: bool(received))
    assert_map(received[-1], expected_second)
    late = []
    rig['node'].create_subscription(PointCloud2, '/global_map', late.append, MAP_QOS)
    rig['spin_until'](lambda: bool(late))
    assert_map(late[-1], expected_second)


def test_explicit_periodic_mode_remains_available(rig):
    path, expected = rig['write_map']('map.pcd')
    rig['start'](path, 'publish_once:=false')
    received = []
    rig['node'].create_subscription(PointCloud2, '/global_map', received.append, MAP_QOS)
    rig['spin_until'](lambda: len(received) >= 3)
    for message in received:
        assert_map(message, expected)


@pytest.mark.parametrize('relative', [
    'desktop_ui/config/rviz/navigation.rviz',
    'desktop_ui/config/rviz/navigation_amcl.rviz',
    'desktop_ui/config/rviz/navigation_fastlio.rviz',
    'ros2_ws/src/FAST_LIO_LOCALIZATION_ROS2/rviz/fastlio_localization.rviz',
])
def test_static_map_displays_request_cached_data(relative):
    config = yaml.safe_load((PROJECT/relative).read_text())

    def topics(value):
        if isinstance(value, dict):
            if value.get('Value') == '/global_map':
                yield value
            for child in value.values():
                yield from topics(child)
        elif isinstance(value, list):
            for child in value:
                yield from topics(child)

    matches = list(topics(config))
    assert len(matches) == 1
    assert matches[0]['Durability Policy'] == 'Transient Local'
    assert matches[0]['Reliability Policy'] == 'Reliable'
