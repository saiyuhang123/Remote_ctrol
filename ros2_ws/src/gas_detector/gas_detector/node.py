"""Read-only ROS adapter. The standalone driver and its CLI remain usable."""
import json
import fcntl
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from std_srvs.srv import SetBool
from gq_otk9633 import GQOTK9633


class GasMonitor(Node):
    def __init__(self, device_factory=GQOTK9633, **kwargs):
        super().__init__('gas_detector', **kwargs)
        for name, default in dict(port='/dev/gq_otk9633', address=1, baudrate=2400,
                                  interval=2.0, enabled=False).items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.port = self.get_parameter('port').value
        self.address = self.get_parameter('address').value
        self.baudrate = self.get_parameter('baudrate').value
        self.interval = self.get_parameter('interval').value
        if not self.port or not 1 <= self.address <= 254 or self.baudrate <= 0:
            raise ValueError('Invalid explicit gas serial port, address or baudrate')
        if not math.isfinite(self.interval) or self.interval < 1.5:
            raise ValueError('interval must be >= 1.5 seconds')
        self._factory = device_factory
        self._enabled = threading.Event()
        self._quit = threading.Event()
        self._lock = threading.Lock()
        self._state = 'stopped'
        self._error = ''
        self._last_sample_ns = 0
        self._samples = 0
        self._connected = False
        self._data = self.create_publisher(String, '/gas_detector/data', 10)
        self._status = self.create_publisher(String, '/gas_detector/status', 10)
        self.create_service(SetBool, '/gas_detector/set_enabled', self._set_enabled)
        self.create_timer(0.5, self._publish_status)
        if self.get_parameter('enabled').value:
            self._enabled.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _set_enabled(self, request, response):
        if request.data:
            self._enabled.set()
        else:
            self._enabled.clear()
        response.success = True
        response.message = ('Acquisition requested' if request.data else
                            'Stop requested; waiting for current read and serial close')
        self._publish_status()
        return response

    def _publish_status(self):
        with self._lock:
            value = dict(component='gas', enabled=self._enabled.is_set(),
                         state=self._state, connected=self._connected, port=self.port,
                         address=self.address, baudrate=self.baudrate,
                         interval=self.interval, last_error=self._error,
                         last_sample_ns=self._last_sample_ns, samples=self._samples)
        self._status.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def _run(self):
        device = None
        while not self._quit.is_set():
            if not self._enabled.is_set():
                if device is not None:
                    try:
                        device.close()
                    except Exception as error:
                        with self._lock:
                            self._error = str(error)
                    device = None
                with self._lock:
                    self._connected = False
                    self._state = 'stopped'
                self._quit.wait(0.1)
                continue
            try:
                if device is None:
                    with self._lock:
                        self._state = 'connecting'
                    # Explicit path prevents probing or sending requests to an IMU.
                    device = self._factory(self.port, address=self.address,
                                           baudrate=self.baudrate, timeout=0.5)
                    with self._lock:
                        self._connected = True
                started = time.monotonic()
                value = device.snapshot()  # Only documented read registers; never writes.
                stamp = self.get_clock().now().nanoseconds
                if self._enabled.is_set() and not self._quit.is_set():
                    value.update(component='gas', sample_time_ns=stamp, port=self.port)
                    self._data.publish(String(data=json.dumps(value, ensure_ascii=False)))
                    with self._lock:
                        self._state, self._error = 'sampling', ''
                        self._last_sample_ns = stamp
                        self._samples += 1
                while (self._enabled.is_set() and not self._quit.is_set() and
                       time.monotonic() - started < self.interval):
                    self._quit.wait(0.05)
            except Exception as error:
                if device is not None:
                    try:
                        device.close()
                    except Exception:
                        pass
                    device = None
                with self._lock:
                    self._connected = False
                    self._state, self._error = 'error', str(error)
                # Bounded reconnect cadence; stop remains responsive.
                until = time.monotonic() + 2.0
                while self._enabled.is_set() and time.monotonic() < until:
                    if self._quit.wait(0.1):
                        break
        if device is not None:
            try:
                device.close()
            except Exception:
                pass

    def destroy_node(self):
        self._enabled.clear()
        self._quit.set()
        self._thread.join(timeout=6.0)
        return super().destroy_node()


def main(args=None):
    lock_path = '/tmp/remote_ctrol_gas_%s_%s.lock' % (os.getuid(), os.environ.get('ROS_DOMAIN_ID', '0'))
    with open(lock_path, 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('gas_detector already running in this ROS domain; reuse the existing node')
            return
        rclpy.init(args=args)
        node = GasMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
