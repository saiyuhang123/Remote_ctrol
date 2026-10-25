"""Explicit isolated ROS tests. Never opens a device or sends a real PTZ command."""
import importlib.util
import json
import os
from pathlib import Path
import struct
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('DEVICE_ROS_TESTS') != '1',
                                reason='requires explicit isolated ROS test opt-in')
if os.environ.get('DEVICE_ROS_TESTS') == '1':
    assert os.environ.get('ROS_DOMAIN_ID') == '230'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool
from gas_detector.node import GasMonitor
from gq_otk9633 import GQOTK9633, frame_with_crc
from hk_camera.srv import ControlPTZ

spec = importlib.util.spec_from_file_location('tested_ptz_patrol',
    Path(__file__).resolve().parents[2] / 'chassis_nav2/scripts/ptz_patrol.py')
ptz_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ptz_module)


def spin_until(executor, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    assert predicate()


@pytest.fixture
def ros():
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node
    yield executor, add
    for node in reversed(nodes):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


def test_real_protocol_snapshot_uses_only_reads_and_preserves_unverified_scaling():
    class Transport:
        timeout = 0
        def __init__(self):
            self.response = b''
            self.requests = []
        def reset_input_buffer(self):
            self.response = b''
        def write(self, request):
            self.requests.append(request)
            address, function, register, count = struct.unpack('>BBHH', request[:-2])
            assert function == 3 and count == 1
            value = {0: 150, 1: 0x0200, 2: 1, 3: 0x0100, 5: 1000, 7: 100, 8: 150}[register]
            self.response = frame_with_crc(struct.pack('>BBBH', address, 3, 2, value))
            return len(request)
        def flush(self):
            pass
        def read(self, count):
            part, self.response = self.response[:count], self.response[count:]
            return part
        def close(self):
            pass
    transport = Transport()
    detector = GQOTK9633(transport=transport)
    data = detector.snapshot()
    assert len(transport.requests) == 7
    assert data['raw_value'] == 150 and data['value_scaled'] == 15
    assert data['unit'] == 'ppm' and data['state'] == 'high_alarm'
    assert data['decimal_scaling_verified'] is False


def test_gas_disabled_start_stop_releases_port_without_navigation(ros):
    executor, add = ros
    devices = []
    class FakeGas:
        def __init__(self, port, **kwargs):
            assert port == '/dev/gq_otk9633'
            self.closed = False
            devices.append(self)
        def snapshot(self):
            return dict(raw_value=150, value_scaled=15, state_code=2, unit='ppm',
                        decimal_scaling_verified=False)
        def close(self):
            self.closed = True
    monitor = add(GasMonitor(device_factory=FakeGas))
    peer = add(rclpy.create_node('gas_test_peer'))
    data, statuses = [], []
    sub = peer.create_subscription(String, '/gas_detector/data', lambda msg: data.append(json.loads(msg.data)), 10)
    status_sub = peer.create_subscription(String, '/gas_detector/status', lambda msg: statuses.append(json.loads(msg.data)), 10)
    switch = peer.create_client(SetBool, '/gas_detector/set_enabled')
    spin_until(executor, switch.service_is_ready)
    assert devices == []
    future = switch.call_async(SetBool.Request(data=True))
    spin_until(executor, lambda: future.done() and data)
    assert data[0]['raw_value'] == 150 and not devices[0].closed
    future = switch.call_async(SetBool.Request(data=False))
    spin_until(executor, lambda: future.done() and devices[0].closed)
    count = len(data)
    spin_until(executor, lambda: statuses and statuses[-1]['state'] == 'stopped')
    assert len(data) == count
    assert sub is not None and status_sub is not None


def test_gas_read_failure_reports_error_without_fake_zero(ros):
    executor, add = ros
    def missing(*args, **kwargs):
        raise OSError('isolated missing gas device')
    monitor = add(GasMonitor(device_factory=missing))
    statuses, samples = [], []
    peer = add(rclpy.create_node('gas_failure_peer'))
    peer.create_subscription(String, '/gas_detector/status', lambda msg: statuses.append(json.loads(msg.data)), 10)
    peer.create_subscription(String, '/gas_detector/data', lambda msg: samples.append(msg.data), 10)
    switch = peer.create_client(SetBool, '/gas_detector/set_enabled')
    spin_until(executor, switch.service_is_ready)
    switch.call_async(SetBool.Request(data=True))
    spin_until(executor, lambda: statuses and statuses[-1]['state'] == 'error')
    assert 'isolated missing' in statuses[-1]['last_error']
    assert samples == []
    switch.call_async(SetBool.Request(data=False))
    spin_until(executor, lambda: statuses[-1]['state'] == 'stopped')


def test_ptz_renew_reverse_stop_failure_and_parameter_validation(ros):
    executor, add = ros
    commands, statuses = [], []
    fail_stop = [False]
    peer = add(rclpy.create_node('camera_ptz_fixture'))
    def control(request, response):
        commands.append((request.command, request.stop))
        response.success = not (request.stop and fail_stop[0])
        response.message = 'isolated PTZ fixture'
        return response
    peer.create_service(ControlPTZ, '/ptz/control', control)
    peer.create_subscription(String, '/ptz_patrol/status', lambda msg: statuses.append(json.loads(msg.data)), 10)
    node = add(ptz_module.PtzPatrol())
    spin_until(executor, lambda: statuses and node._ctrl.service_is_ready())
    assert not commands and not statuses[-1]['active']
    assert node.set_parameters_atomically([Parameter('enabled', value=True)]).successful
    spin_until(executor, lambda: len(commands) >= 2)
    assert commands[0] == (23, False)
    node._last_flip = time.monotonic() - 21
    spin_until(executor, lambda: any(command == (24, False) for command in commands))
    assert not node.set_parameters_atomically([Parameter('pan_speed', value=9)]).successful
    assert not node.set_parameters_atomically([Parameter('sweep_seconds', value=float('nan'))]).successful
    fail_stop[0] = True
    assert node.set_parameters_atomically([Parameter('enabled', value=False)]).successful
    spin_until(executor, lambda: any(stop for _, stop in commands))
    assert node._moving  # A failed stop must not release the UI interlock.
    fail_stop[0] = False
    spin_until(executor, lambda: statuses and not statuses[-1]['active'])
    assert not node._moving
    count = len(commands)
    node._tick()
    executor.spin_once(timeout_sec=0.1)
    assert len(commands) == count


def test_ptz_stop_requested_during_inflight_start_is_not_lost(ros):
    executor, add = ros
    commands = []
    peer = add(rclpy.create_node('ptz_delayed_fixture'))
    def control(request, response):
        commands.append(request.stop)
        response.success = True
        return response
    peer.create_service(ControlPTZ, '/ptz/control', control)
    node = add(ptz_module.PtzPatrol())
    spin_until(executor, node._ctrl.service_is_ready)
    node.set_parameters_atomically([Parameter('enabled', value=True)])
    node._tick()
    assert node._ctrl_pending is not None
    node.set_parameters_atomically([Parameter('enabled', value=False)])
    spin_until(executor, lambda: commands == [False, True] and node._ctrl_pending is None)
    assert not node._moving and not node._stop_due
