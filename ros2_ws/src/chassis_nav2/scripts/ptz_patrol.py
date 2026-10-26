#!/usr/bin/env python3
"""Independent, opt-in PTZ sweep. Default disabled; compatible with ros2 param.

Renews the RS485 head driver's 500 ms lease. Reports real command acknowledgments
and reverses after the measured horizontal travel reaches the requested angle.
Keeps stop pending until acknowledged; no navigation dependencies.
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
from hk_camera.srv import ControlPTZ, ControlPTZPatrol, GetPTZPose, SetPTZPose
from patrol_interfaces.srv import PtzInspectionControl

PAN_LEFT, PAN_RIGHT = 23, 24
MIN_FLIP_GAP = 3.0
PAN_FEEDBACK_MAX_AGE = 2.0
SETTINGS = ('enabled', 'pan_speed', 'pan_sweep_degrees', 'pan_min', 'pan_max',
            'tilt_enabled', 'tilt_min', 'tilt_max', 'tilt_speed',
            'home_pan', 'home_tilt', 'home_zoom')


class PtzPatrol(Node):
    def __init__(self):
        super().__init__('ptz_patrol')
        for key, value in dict(enabled=False, pan_speed=2, pan_sweep_degrees=360.0,
                               pan_min=-1.0, pan_max=-1.0, home_pan=-1.0,
                               home_tilt=-1.0, home_zoom=-1.0, tilt_enabled=False,
                               tilt_min=-9.0, tilt_max=15.0, tilt_speed=1).items():
            self.declare_parameter(key, value)
        self._validate({p.name: p.value for p in self.get_parameters(SETTINGS)})
        self.add_on_set_parameters_callback(self._on_parameters)
        self._ctrl = self.create_client(ControlPTZ, '/ptz/control')
        self._patrol_ctrl = self.create_client(ControlPTZPatrol, '/ptz/control_patrol')
        self._pose = self.create_client(GetPTZPose, '/ptz/get_angles')
        self._set = self.create_client(SetPTZPose, '/hk_camera/set_ptz_pose')
        self._status = self.create_publisher(String, '/ptz_patrol/status', 10)
        self._moving = False
        self._direction = PAN_LEFT
        self._last_flip = time.monotonic()
        self._ctrl_pending = None
        self._pending_client = None
        self._ctrl_deadline = 0.0
        self._pose_pending = None
        self._pose_deadline = 0.0
        self._pan = None
        self._tilt = None
        self._positioning = False
        self._pan_stamp = 0.0
        self._pan_previous = None
        self._pan_travel = 0.0
        self._pan_motion_stamp = time.monotonic()
        self._pan_motion_anchor = 0.0
        self._completed_sweeps = 0
        self._error = ''
        self._stop_due = False
        self._home_due = False
        self._inspection_owner = ''
        self._inspection_deadline = 0.0
        self._inspection_fault = ''
        self._inspection_releases = {}
        self.create_service(PtzInspectionControl, '/ptz_patrol/inspection_control', self._inspection_control)
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
        if type(values['tilt_enabled']) is not bool:
            raise ValueError('tilt_enabled must be boolean')
        if type(values['tilt_speed']) is not int or not 1 <= values['tilt_speed'] <= 7:
            raise ValueError('tilt_speed must be an integer in 1..7')
        lo, hi = values['tilt_min'], values['tilt_max']
        if not (math.isfinite(lo) and math.isfinite(hi) and -90 <= lo < hi <= 25 and hi - lo >= 2):
            raise ValueError('tilt bounds require -90 <= min < max <= 25 and a span >= 2 degrees')
        if not math.isfinite(values['pan_sweep_degrees']) or not 1 <= values['pan_sweep_degrees'] <= 360:
            raise ValueError('pan_sweep_degrees must be 1..360 degrees')
        lo, hi = values['pan_min'], values['pan_max']
        if not (lo == hi == -1 or 0 <= lo < hi <= 360):
            raise ValueError('pan bounds must both be -1, or 0 <= min < max <= 360')
        for key in ('home_pan', 'home_tilt', 'home_zoom'):
            value = values[key]
            if not math.isfinite(value) or not (value == -1 or
                    (1 <= value <= 999.9 if key == 'home_zoom' else 0 <= value <= 360)):
                raise ValueError('Invalid ' + key)

    def _on_parameters(self, parameters):
        names = SETTINGS
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
            self._reset_pan_progress(invalidate=True)
            self._completed_sweeps = 0
            self._inspection_fault = ''
            self._error = ''
        return SetParametersResult(successful=True)

    def _expire_inspection(self):
        if self._inspection_owner and time.monotonic() >= self._inspection_deadline:
            # A lost executor must never silently restart the sweep into a
            # pending capture. An explicit operator start is required afterward.
            self._home_due = False
            self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
            self._remember_release(self._inspection_owner, False)
            self._inspection_owner = ''
            self._stop_due = True
            self._inspection_fault = '巡检联动失联，云台保持停转，请重新确认后启动'

    def _inspection_stopped(self):
        return not self._moving and not self._stop_due and self._ctrl_pending is None

    def _remember_release(self, owner, resume):
        self._inspection_releases[owner] = resume
        while len(self._inspection_releases)>20:
            self._inspection_releases.pop(next(iter(self._inspection_releases)))

    def _inspection_control(self, request, response):
        self._expire_inspection()
        owner = request.owner_id.strip()
        command = request.command
        if not owner or len(owner)>128 or command not in (1, 2, 3):
            response.message = '巡检联动请求无效'
            return response
        if command in (1, 2) and (not math.isfinite(request.lease_seconds) or not 1 <= request.lease_seconds <= 10):
            response.message = '巡检联动租期无效'
            return response
        if command == PtzInspectionControl.Request.ACQUIRE:
            if owner in self._inspection_releases:
                response.message = '该巡检占用已结束，拒绝迟到请求'
                return response
            if self._inspection_owner and self._inspection_owner != owner:
                response.message = '云台已由其他巡检动作占用'
                return response
            if not self._inspection_owner:
                self._stop_due = True  # Require an actual camera stop ACK.
                self._reset_pan_progress(invalidate=True)
            self._inspection_owner = owner
            self._inspection_deadline = time.monotonic()+request.lease_seconds
            self._inspection_fault = ''
            self._home_due = False
            self._tick()
        elif command == PtzInspectionControl.Request.RENEW:
            if self._inspection_owner != owner:
                response.message = '巡检联动占用已失效'
                return response
            self._inspection_deadline = time.monotonic()+request.lease_seconds
        else:
            if self._inspection_owner != owner:
                if self._inspection_owner:
                    response.message = '云台已由其他巡检动作占用'
                    return response
                previous = self._inspection_releases.get(owner)
                if not request.resume:
                    # A cancel can overtake an ACQUIRE or follow a RELEASE
                    # whose response was lost. Never resurrect the old hold.
                    if previous is True:
                        self._home_due = False
                        self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
                    self._remember_release(owner, False)
                response.success, response.stopped = True, self._inspection_stopped()
                response.sweep_enabled = bool(self.get_parameter('enabled').value)
                response.message = '巡检占用已结束'
                return response
            self._home_due = False
            if not request.resume:
                self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
            self._inspection_owner = ''
            self._remember_release(owner, bool(request.resume))
            self._last_flip = time.monotonic()
            # Inspection can point the head elsewhere. Resume a new sweep from
            # the actual post-inspection angle, excluding that repositioning.
            self._reset_pan_progress(invalidate=True)
        response.success = True
        response.stopped = self._inspection_stopped()
        response.sweep_enabled = bool(self.get_parameter('enabled').value)
        response.message = '巡检停转已确认' if self._inspection_owner and response.stopped else '巡检联动请求已受理'
        self._publish_status()
        return response

    def _use_pan_bounds(self):
        lo, hi = self.get_parameter('pan_min').value, self.get_parameter('pan_max').value
        return 0 <= lo < hi <= 360 and hi - lo < 360

    def _reset_pan_progress(self, invalidate=False):
        self._pan_previous = None
        self._pan_travel = 0.0
        self._pan_motion_stamp = time.monotonic()
        self._pan_motion_anchor = 0.0
        if invalidate:
            self._pan_stamp = 0.0
            if self._pose_pending is not None:
                self._pose.remove_pending_request(self._pose_pending)
                self._pose_pending = None

    def _accept_pan(self, pan):
        if not math.isfinite(pan) or not 0 <= pan <= 360:
            return False
        self._pan, self._pan_stamp = float(pan), time.monotonic()
        if self._positioning or self._inspection_owner or self._stop_due or not self.get_parameter('enabled').value:
            self._pan_previous = None
            return True
        if self._pan_previous is not None:
            # Unwrap 0/360 without counting jitter or a repeated start heading
            # as a completed revolution. Signed deltas cancel reverse drift.
            delta = math.remainder(pan - self._pan_previous, 360.)
            self._pan_travel += delta
            if abs(self._pan_travel - self._pan_motion_anchor) >= .1:
                self._pan_motion_stamp = time.monotonic()
                self._pan_motion_anchor = self._pan_travel
        self._pan_previous = float(pan)
        return True

    def _fail_feedback(self, message):
        self._error = message
        self._home_due = False
        self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
        self._stop_due = True

    def _tick(self):
        self._expire_inspection()
        now = time.monotonic()
        if self._ctrl_pending is not None and now >= self._ctrl_deadline:
            self._pending_client.remove_pending_request(self._ctrl_pending)
            self._ctrl_pending = None
            self._error = 'PTZ command timed out; stop required before retry'
            self._stop_due = True  # The timed-out command may have executed.
            self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
        if self._ctrl_pending is not None:
            return
        enabled = self.get_parameter('enabled').value
        control = self._patrol_ctrl if self.get_parameter('tilt_enabled').value else self._ctrl
        if self._stop_due or (not enabled and self._moving):
            if control.service_is_ready():
                self._command(True)
            return
        if not enabled or self._inspection_owner:
            self._last_flip = now
            return
        if not control.service_is_ready():
            self._error = '等待 485 云台服务，请启动或更新 ptz_pelco 节点'
            return
        if self._pan_stamp <= 0 or now - self._pan_stamp >= PAN_FEEDBACK_MAX_AGE:
            if self._moving:
                self._fail_feedback('485 水平角度反馈超时，已停止巡航')
                self._command(True)
            else:
                # Establish an actual starting angle before any horizontal
                # move, including horizontal-only sweeps without bounds.
                self._poll_pose()
            return
        elapsed = now - self._last_flip
        if self._moving and not self._positioning and now - self._pan_motion_stamp > 3.0:
            self._fail_feedback('485 水平角度持续无变化，已停止巡航')
            self._command(True)
            return
        flip = abs(self._pan_travel) >= self._sweep_target()
        if self._use_pan_bounds():
            flip = elapsed > MIN_FLIP_GAP and not self._positioning and not (
                self.get_parameter('pan_min').value < self._pan < self.get_parameter('pan_max').value)
        if flip and not self._positioning:
            self._direction = PAN_RIGHT if self._direction == PAN_LEFT else PAN_LEFT
            self._last_flip = now
            self._completed_sweeps += 1
            self._reset_pan_progress()
            self._pan_previous = self._pan
        self._command(False)

    def _sweep_target(self):
        if self.get_parameter('pan_min').value == 0 and self.get_parameter('pan_max').value == 360:
            return 360.0
        return self.get_parameter('pan_sweep_degrees').value

    def _command(self, stop):
        patrol_mode = bool(self.get_parameter('tilt_enabled').value)
        if patrol_mode:
            req = ControlPTZPatrol.Request(command=self._direction,
                pan_speed=self.get_parameter('pan_speed').value, stop=stop,
                tilt_enabled=True, tilt_min=self.get_parameter('tilt_min').value,
                tilt_max=self.get_parameter('tilt_max').value,
                tilt_speed=self.get_parameter('tilt_speed').value)
            client = self._patrol_ctrl
        else:
            req = ControlPTZ.Request()
            req.command, req.speed, req.stop = (self._direction,
                self.get_parameter('pan_speed').value, stop)
            client = self._ctrl
        future = client.call_async(req)
        self._pending_client = client
        self._ctrl_pending = future
        self._ctrl_deadline = time.monotonic() + 1.5
        if not stop:
            self._home_due = True
        future.add_done_callback(lambda done: self._on_ctrl_done(done, stop, patrol_mode))

    def _on_ctrl_done(self, future, stop, patrol_mode=False):
        if future is not self._ctrl_pending:
            return
        self._ctrl_pending = None
        try:
            result = future.result()
            if not result.success:
                self._error = 'PTZ control failed: ' + result.message
                self._stop_due = True
                self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
                return
            if not stop and self.get_parameter('enabled').value and not self._stop_due:
                self._error = ''
            self._moving = not stop
            self._positioning = patrol_mode and not stop and result.positioning
            if self._positioning:
                self._last_flip = time.monotonic()
                self._reset_pan_progress()
            if patrol_mode and not stop:
                if not result.pose_valid or not math.isfinite(result.tilt) or not self._accept_pan(result.pan):
                    self._fail_feedback('485 实际角度反馈无效，已停止巡航')
                else:
                    self._tilt = result.tilt
            if stop:
                self._stop_due = False
                if self._home_due and not self.get_parameter('enabled').value and not self._inspection_owner:
                    self._go_home()
                    self._home_due = False
        except Exception as error:
            self._error = str(error)
            self._stop_due = True
            self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
        self._publish_status()

    def _poll_pose(self):
        now = time.monotonic()
        if self._pose_pending is not None and now >= self._pose_deadline:
            self._pose.remove_pending_request(self._pose_pending)
            self._pose_pending = None
            if self.get_parameter('enabled').value and not self._inspection_owner:
                self._fail_feedback('485 水平角度读取超时，已停止巡航')
        if (not self.get_parameter('enabled').value or self._inspection_owner or self._stop_due or
                self._pose_pending is not None or (self.get_parameter('tilt_enabled').value and self._moving)):
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
            if not result.success or not self._accept_pan(result.pan):
                if self.get_parameter('enabled').value and not self._inspection_owner:
                    self._fail_feedback('485 水平角度反馈无效，已停止巡航')
        except Exception:
            if self.get_parameter('enabled').value and not self._inspection_owner:
                self._fail_feedback('485 水平角度读取失败，已停止巡航')

    def _publish_status(self):
        enabled = self.get_parameter('enabled').value
        active = enabled or self._moving or self._stop_due or self._ctrl_pending is not None or bool(self._inspection_owner)
        state = ('inspection_stopping' if self._inspection_owner and not self._inspection_stopped() else
                 'inspection_paused' if self._inspection_owner else
                 'stopping' if not enabled and active else 'error' if self._error or self._inspection_fault else
                 'positioning' if self._positioning else 'sweeping' if self._moving else 'waiting' if enabled else 'stopped')
        value = dict(component='ptz', enabled=enabled, moving=self._moving,
                     status_time_ns=self.get_clock().now().nanoseconds,
                     active=active, state=state, last_error=self._error or self._inspection_fault,
                     inspection_held=bool(self._inspection_owner), inspection_owner=self._inspection_owner,
                     direction='left' if self._direction == PAN_LEFT else 'right',
                     pan=self._pan, pan_speed=self.get_parameter('pan_speed').value,
                     pan_sweep_degrees=self._sweep_target(),
                     pan_progress_degrees=abs(self._pan_travel),
                     completed_sweeps=self._completed_sweeps,
                     sweep_mode='bounds' if self._use_pan_bounds() else 'angle',
                     pan_min=self.get_parameter('pan_min').value,
                     pan_max=self.get_parameter('pan_max').value)
        value.update(control_transport='rs485', tilt=self._tilt,
                     tilt_enabled=self.get_parameter('tilt_enabled').value,
                     tilt_min=self.get_parameter('tilt_min').value,
                     tilt_max=self.get_parameter('tilt_max').value,
                     tilt_speed=self.get_parameter('tilt_speed').value)
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
