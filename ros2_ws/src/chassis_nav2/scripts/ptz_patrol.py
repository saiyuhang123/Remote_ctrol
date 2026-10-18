#!/usr/bin/env python3
"""Independent, opt-in PTZ sweep. Default disabled; compatible with ros2 param.

Renews the camera driver's 500 ms jog lease. Reports real command acknowledgments
and keeps stop pending until acknowledged; no navigation dependencies.
"""
import fcntl
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from hk_camera.srv import ControlPTZ, GetPTZPose, SetPTZPose

PAN_LEFT, PAN_RIGHT = 23, 24
MIN_FLIP_GAP = 3.0


class PtzPatrol(Node):
    def __init__(self):
        super().__init__('ptz_patrol')
        for key, value in dict(enabled=False, pan_speed=2, sweep_seconds=20.0,
                               pan_min=-1.0, pan_max=-1.0, home_pan=-1.0,
                               home_tilt=-1.0, home_zoom=-1.0).items():
            self.declare_parameter(key, value)
        self._validate({p.name: p.value for p in self.get_parameters([
            'enabled', 'pan_speed', 'sweep_seconds', 'pan_min', 'pan_max',
            'home_pan', 'home_tilt', 'home_zoom'])})
        self.add_on_set_parameters_callback(self._on_parameters)
        self._ctrl = self.create_client(ControlPTZ, '/hk_camera/control_ptz')
        self._pose = self.create_client(GetPTZPose, '/hk_camera/get_ptz_pose')
        self._set = self.create_client(SetPTZPose, '/hk_camera/set_ptz_pose')
        self._status = self.create_publisher(String, '/ptz_patrol/status', 10)
        self._moving = False
        self._direction = PAN_LEFT
        self._last_flip = time.monotonic()
        self._ctrl_pending = None
        self._ctrl_deadline = 0.0
        self._pose_pending = None
        self._pose_deadline = 0.0
        self._pan = None
        self._pan_stamp = 0.0
        self._error = ''
        self._stop_due = False
        self._home_due = False
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._poll_pose)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info('云台巡航就绪，默认关闭；enabled=true 开启')

    @staticmethod
    def _validate(values):
        if type(values['enabled']) is not bool:
            raise ValueError('enabled must be boolean')
        if type(values['pan_speed']) is not int or not 1 <= values['pan_speed'] <= 7:
            raise ValueError('pan_speed must be an integer in 1..7')
        if not math.isfinite(values['sweep_seconds']) or not 1 <= values['sweep_seconds'] <= 600:
            raise ValueError('sweep_seconds must be 1..600 seconds')
        lo, hi = values['pan_min'], values['pan_max']
        if not (lo == hi == -1 or 0 <= lo < hi <= 360):
            raise ValueError('pan bounds must both be -1, or 0 <= min < max <= 360')
        for key in ('home_pan', 'home_tilt', 'home_zoom'):
            value = values[key]
            if not math.isfinite(value) or not (value == -1 or
                    (1 <= value <= 999.9 if key == 'home_zoom' else 0 <= value <= 360)):
                raise ValueError('Invalid ' + key)

    def _on_parameters(self, parameters):
        names = ('enabled', 'pan_speed', 'sweep_seconds', 'pan_min', 'pan_max',
                 'home_pan', 'home_tilt', 'home_zoom')
        values = {name: self.get_parameter(name).value for name in names}
        for parameter in parameters:
            if parameter.name in names:
                values[parameter.name] = parameter.value
        try:
            self._validate(values)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        # Configuration changes are applied with enabled=true by the UI atomically.
        if self.get_parameter('enabled').value and values['enabled'] and any(
                p.name != 'enabled' and p.name in names and
                p.value != self.get_parameter(p.name).value for p in parameters):
            return SetParametersResult(successful=False, reason='Stop PTZ patrol before changing settings')
        if not values['enabled'] and (self._moving or self._ctrl_pending is not None):
            self._stop_due = True
        if values['enabled'] and not self.get_parameter('enabled').value:
            self._last_flip = time.monotonic()
            self._direction = PAN_LEFT
        return SetParametersResult(successful=True)

    def _use_pan_bounds(self):
        return 0 <= self.get_parameter('pan_min').value < self.get_parameter('pan_max').value <= 360

    def _tick(self):
        now = time.monotonic()
        if self._ctrl_pending is not None and now >= self._ctrl_deadline:
            self._ctrl.remove_pending_request(self._ctrl_pending)
            self._ctrl_pending = None
            self._error = 'PTZ command timed out; stop required before retry'
            self._stop_due = True  # The timed-out command may have executed.
        if self._ctrl_pending is not None:
            return
        enabled = self.get_parameter('enabled').value
        if self._stop_due or (not enabled and self._moving):
            if self._ctrl.service_is_ready():
                self._command(True)
            return
        if not enabled:
            self._last_flip = now
            return
        if not self._ctrl.service_is_ready():
            self._error = 'Waiting for camera PTZ service'
            return
        elapsed = now - self._last_flip
        flip = elapsed >= self.get_parameter('sweep_seconds').value
        if not flip and elapsed > MIN_FLIP_GAP and self._use_pan_bounds() and \
                self._pan is not None and now - self._pan_stamp < 2.0:
            flip = not (self.get_parameter('pan_min').value < self._pan <
                        self.get_parameter('pan_max').value)
        if flip:
            # Stop the old direction before starting a different direction.
            self._direction = PAN_RIGHT if self._direction == PAN_LEFT else PAN_LEFT
            self._last_flip = now
        self._command(False)

    def _command(self, stop):
        req = ControlPTZ.Request()
        req.command, req.speed, req.stop = (self._direction,
            self.get_parameter('pan_speed').value, stop)
        future = self._ctrl.call_async(req)
        self._ctrl_pending = future
        self._ctrl_deadline = time.monotonic() + 1.5
        if not stop:
            self._home_due = True
        future.add_done_callback(lambda done: self._on_ctrl_done(done, stop))

    def _on_ctrl_done(self, future, stop):
        if future is not self._ctrl_pending:
            return
        self._ctrl_pending = None
        try:
            result = future.result()
            if not result.success:
                self._error = 'PTZ control failed: ' + result.message
                self._stop_due = True
                return
            self._error = ''
            self._moving = not stop
            if stop:
                self._stop_due = False
                if self._home_due and not self.get_parameter('enabled').value:
                    self._go_home()
                    self._home_due = False
        except Exception as error:
            self._error = str(error)
            self._stop_due = True
        self._publish_status()

    def _poll_pose(self):
        now = time.monotonic()
        if self._pose_pending is not None and now >= self._pose_deadline:
            self._pose.remove_pending_request(self._pose_pending)
            self._pose_pending = None
            self._pan = None
        if not self._use_pan_bounds() or not self._moving or self._pose_pending is not None:
            return
        if self._pose.service_is_ready():
            self._pose_pending = self._pose.call_async(GetPTZPose.Request())
            self._pose_deadline = now + 1.5
            self._pose_pending.add_done_callback(self._on_pose_done)

    def _on_pose_done(self, future):
        if future is not self._pose_pending:
            return
        self._pose_pending = None
        try:
            result = future.result()
            self._pan = result.pan if result.success else None
            self._pan_stamp = time.monotonic()
        except Exception:
            self._pan = None

    def _publish_status(self):
        enabled = self.get_parameter('enabled').value
        active = enabled or self._moving or self._stop_due or self._ctrl_pending is not None
        state = ('stopping' if not enabled and active else 'error' if self._error else
                 'sweeping' if self._moving else 'waiting' if enabled else 'stopped')
        value = dict(component='ptz', enabled=enabled, moving=self._moving,
                     status_time_ns=self.get_clock().now().nanoseconds,
                     active=active, state=state, last_error=self._error,
                     direction='left' if self._direction == PAN_LEFT else 'right',
                     pan=self._pan, pan_speed=self.get_parameter('pan_speed').value,
                     sweep_seconds=self.get_parameter('sweep_seconds').value,
                     pan_min=self.get_parameter('pan_min').value,
                     pan_max=self.get_parameter('pan_max').value)
        self._status.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def _go_home(self):
        pan, tilt, zoom = [self.get_parameter('home_' + name).value for name in ('pan', 'tilt', 'zoom')]
        if not (0 <= pan <= 360 and 0 <= tilt <= 360 and zoom >= 1) or not self._set.service_is_ready():
            return
        req = SetPTZPose.Request()
        req.pan, req.tilt, req.zoom = float(pan), float(tilt), float(zoom)
        # Only an explicit standalone home_* configuration enables this behavior.
        self._set.call_async(req)

    def stop_before_exit(self):
        self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
        deadline = time.monotonic() + 1.0
        while (self._moving or self._stop_due or self._ctrl_pending is not None) and time.monotonic() < deadline:
            self._tick()
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    # Navigation launch and UI can discover at the same time; prevent duplicate writers.
    lock_path = '/tmp/remote_ctrol_ptz_%s_%s.lock' % (os.getuid(), os.environ.get('ROS_DOMAIN_ID', '0'))
    with open(lock_path, 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('ptz_patrol already running in this ROS domain; reuse the existing node')
            return
        rclpy.init()
        node = PtzPatrol()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                node.stop_before_exit()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
