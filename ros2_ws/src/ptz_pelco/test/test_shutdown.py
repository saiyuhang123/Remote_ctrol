"""A pending SDK reply must not leave a driver holding the serial port on exit."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading

import pytest


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_stop_while_waiting_for_sdk_stops_head_closes_port_and_exits(stop_signal):
    source = Path(__file__).resolve().parents[1]
    environment = dict(os.environ, ROS_DOMAIN_ID='227', ROS_LOCALHOST_ONLY='1')
    environment['PYTHONPATH'] = str(source) + os.pathsep + environment.get('PYTHONPATH', '')
    script = """
import threading
from rclpy.task import Future
import ptz_pelco.node as driver

class FakeHead:
    def __init__(self, *args):
        self.ser = object()
        self.lock = threading.RLock()
        self.stats = {}
    def open(self):
        return True, 'fake port'
    def read_angles(self):
        return 125., 0.
    def stop_move(self):
        print('HEAD_STOPPED', flush=True)
        return True
    def close(self):
        self.ser = None
        print('PORT_CLOSED', flush=True)

async def pending_sdk(self, client, request):
    print('SDK_PENDING', flush=True)
    return await Future()

driver.PelcoSerial = FakeHead
driver.PtzPelcoNode._sdk_request = pending_sdk
driver.main()
"""
    process = subprocess.Popen([sys.executable, '-u', '-c', script], env=environment,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    ready = threading.Event()
    output = []
    def read_output():
        for line in process.stdout:
            output.append(line)
            if 'SDK_PENDING' in line:
                ready.set()
    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    try:
        assert ready.wait(5.), ''.join(output)
        process.send_signal(stop_signal)
        process.wait(timeout=3.)
        reader.join(1.)
        assert process.returncode == 0, ''.join(output)
        assert 'HEAD_STOPPED' in ''.join(output)
        assert 'PORT_CLOSED' in ''.join(output)
    finally:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=2.)
        reader.join(1.)
