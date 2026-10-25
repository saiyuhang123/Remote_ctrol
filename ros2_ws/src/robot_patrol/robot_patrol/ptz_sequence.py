"""Worker-side, feedback-confirmed low-zoom photography. No direct serial I/O."""
import json
import math
from pathlib import Path
import re
import threading
import time

from .models import InspectionResult, PatrolError, new_id, now_ns
from .ptz_shots import device_pan_sign, parse_shots


DEFAULTS = {
    'inspection_shot_arrival_timeout_sec': 30.,
    'inspection_shot_poll_sec': .3,
    'inspection_shot_stable_sec': .6,
    'inspection_shot_angle_tolerance_deg': .3,
    'inspection_shot_zoom_tolerance': .15,
    'inspection_shot_position_tolerance_m': .25,
    'inspection_shot_base_motion_m': .05,
    'inspection_shot_base_motion_deg': 1.,
    'inspection_shot_stop_timeout_sec': 1.5,
}


def path_part(value):
    return re.sub(r'[^A-Za-z0-9_-]', '_', str(value))[:120] or 'unnamed'


class PtzShotSequence:
    def __init__(self, client, callback_group=None, pose_provider=None):
        from hk_camera.srv import ControlPTZ, GetPTZPose, SetPTZPose
        self.client, self.node = client, client.node
        self.Control, self.Get, self.Set = ControlPTZ, GetPTZPose, SetPTZPose
        self.get = self.node.create_client(GetPTZPose, '/hk_camera/get_ptz_pose', callback_group=callback_group)
        self.set = self.node.create_client(SetPTZPose, '/hk_camera/set_ptz_pose', callback_group=callback_group)
        self.stop = self.node.create_client(ControlPTZ, '/hk_camera/control_ptz', callback_group=callback_group)
        self.pose_provider = pose_provider
        self.settings = {}
        for name, default in DEFAULTS.items():
            if not self.node.has_parameter(name):
                self.node.declare_parameter(name, default)
            value = float(self.node.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
            self.settings[name.removeprefix('inspection_shot_')] = value
        self.lock = threading.RLock()
        self.late_sets = set()
        self.late_stop_requests = {}
        self.stop_unconfirmed = False

    def _retain_late_set(self, future):
        # Canceling a ROS future cannot retract a device command. A late set
        # reply must be followed by another stop, even after cancellation.
        with self.lock:
            self.late_sets.add(future)
        future.add_done_callback(self._late_set_done)

    def _late_set_done(self, future):
        try:
            future.result()  # Consume failures too; a partial write still needs stop.
        except Exception:
            pass
        self._request_late_stop(future)

    def _request_late_stop(self, future):
        try:
            with self.lock:
                if future not in self.late_sets or self.late_stop_requests.get(future) is not None:
                    return
                if not self.stop.service_is_ready():
                    return
                stop_future = self.stop.call_async(self.Control.Request(stop=True))
                self.late_stop_requests[future] = stop_future
            def completed(done):
                try:
                    ok = bool(done.result().success)
                except Exception:
                    ok = False
                with self.lock:
                    if self.late_stop_requests.get(future) is done:
                        if ok:
                            self.late_sets.discard(future)
                            self.late_stop_requests.pop(future, None)
                        else:
                            self.late_stop_requests[future] = None
            stop_future.add_done_callback(completed)
        except Exception:
            pass

    def _stop_motion(self):
        """Bounded cleanup independent of the canceled flags or lost lease."""
        self.stop_unconfirmed = True
        deadline = time.monotonic() + self.settings['stop_timeout_sec']
        try:
            if not self.stop.wait_for_service(timeout_sec=min(.1, self.settings['stop_timeout_sec'])):
                return False
            future = self.stop.call_async(self.Control.Request(stop=True))
            event = threading.Event()
            def completed(done):
                try:
                    done.exception()
                except Exception:
                    pass
                event.set()
            future.add_done_callback(completed)
            if not event.wait(max(0., deadline - time.monotonic())):
                return False
            ok = bool(future.result().success)
            self.stop_unconfirmed = not ok
            return ok
        except Exception:
            return False

    def _check_previous_stop(self):
        with self.lock:
            pending = list(self.late_sets)
        for future in pending:
            if future.done():
                self._request_late_stop(future)
        with self.lock:
            if self.late_sets:
                raise PatrolError('INSPECTION_PTZ_PENDING', '上次定位及停止仍在确认，禁止开始新视角')
        if self.stop_unconfirmed:
            if not self._stop_motion():
                raise PatrolError('INSPECTION_STOP_UNCONFIRMED', '上次云台停止尚未确认')

    def _pose(self, baseline=None):
        if self.pose_provider is None:
            raise PatrolError('INSPECTION_POSE_UNAVAILABLE', '没有可用的 map→base_link 位姿来源')
        value = self.pose_provider()  # Production provider checks the TF stamp.
        if value is None or len(value) < 3 or not all(math.isfinite(v) for v in value[:3]):
            raise PatrolError('INSPECTION_POSE_UNAVAILABLE', '定位位姿无效，不回退到车体角度')
        pose = dict(x=float(value[0]), y=float(value[1]), yaw=float(value[2]))
        if baseline is not None:
            distance = math.hypot(pose['x']-baseline['x'], pose['y']-baseline['y'])
            angle = abs(math.degrees(math.remainder(pose['yaw']-baseline['yaw'], 2*math.pi)))
            if distance > self.settings['base_motion_m'] or angle > self.settings['base_motion_deg']:
                raise PatrolError('INSPECTION_BASE_MOVED', '拍摄期间车体位姿变化，已中止本次巡检')
        return pose

    def _get(self, flags, timeout=None):
        response = self.client._call(self.get, self.Get.Request(), timeout or self.client.service_timeout,
                                     '云台实际位置读取', flags)
        if not response.success:
            raise PatrolError('INSPECTION_PTZ_READ_FAILED', response.message or '实际位置读取失败')
        values = dict(pan=float(response.pan), tilt=float(response.tilt), zoom=float(response.zoom))
        if (not all(math.isfinite(v) for v in values.values()) or not 0 <= values['pan'] <= 360 or
                not -90 <= values['tilt'] <= 25 or not 1 <= values['zoom'] <= 999.9):
            raise PatrolError('INSPECTION_PTZ_READ_FAILED', '云台返回无效实际位置')
        self.last_actual = values
        self.last_feedback_received_at_ns = now_ns()
        return values

    def _matches(self, actual, target):
        tolerance = self.settings['angle_tolerance_deg']
        return (abs(math.remainder(actual['pan']-target['pan'], 360.)) <= tolerance and
                abs(actual['tilt']-target['tilt']) <= tolerance and
                abs(actual['zoom']-target['zoom']) <= self.settings['zoom_tolerance'])

    def _wait_arrival(self, target, baseline, deadline, flags):
        stable_since, previous = None, None
        last_error = '尚未到位'
        while time.monotonic() < deadline:
            self.client._check_control(flags)
            self._pose(baseline)
            try:
                actual = self._get(flags, min(self.client.service_timeout, max(.05, deadline-time.monotonic())))
            except PatrolError as error:
                if error.code not in ('INSPECTION_PTZ_READ_FAILED', 'INSPECTION_TIMEOUT', 'INSPECTION_UNAVAILABLE'):
                    raise
                actual, last_error = None, error.message
            if actual is not None and self._matches(actual, target) and (previous is None or self._matches(actual, previous)):
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= self.settings['stable_sec']:
                    return actual
            else:
                stable_since = None
            previous = actual
            if actual is not None:
                last_error = f'实际 {actual}，目标 {target}'
            self.client._wait(min(self.settings['poll_sec'], max(0., deadline-time.monotonic())), flags)
        raise PatrolError('INSPECTION_PTZ_NOT_ARRIVED', '云台到位超时：' + last_error)

    def _set(self, target, flags):
        response = self.client._call(self.set, self.Set.Request(**target), self.client.service_timeout,
                                     '云台绝对定位', flags, on_abandoned=self._retain_late_set)
        if not response.success:
            raise PatrolError('INSPECTION_PTZ_SET_FAILED', response.message or '云台定位失败')

    def run(self, step, run_id, map_id, loop_index, step_index, status_cb=None, flags=None):
        from .inspection_client import InspectionOutcome
        shots = parse_shots(step.shots)
        directory = self.client.image_dir / path_part(run_id) / f'loop{loop_index:02d}' / f'step{step_index:03d}_{path_part(step.point_id)}'
        images = []
        def progress(message):
            if status_cb:
                status_cb(message=message)
        for index, shot in enumerate(shots, 1):
            self.last_actual = None
            self.last_feedback_received_at_ns = 0
            started = now_ns()
            metadata = dict(schema_version=1, shot=shot.as_dict(), run_id=run_id, map_id=map_id,
                            point_id=step.point_id, loop_index=loop_index, step_index=step_index)
            image_path, motion_sent = '', False
            error_code, message = '', ''
            try:
                self.client._check_control(flags)
                self._check_previous_stop()
                mapping = self.client.store.get_map(map_id)
                if mapping.revision != shot.map_revision:
                    raise PatrolError('STALE_DATA', '地图版本已变化，请重新标定视角')
                configured_sign = device_pan_sign(self.node)
                if configured_sign and shot.pan_sign != configured_sign:
                    raise PatrolError('STALE_DATA', '视角与设备方向配置不一致，请清空后重新记录')
                if not self.client.ptz_lease.client.wait_for_service(timeout_sec=.5):
                    raise PatrolError('INSPECTION_PTZ_UNAVAILABLE',
                        '多视角拍照需要云台停转占用接口，请先启动 ptz_patrol.py（默认不旋转）')
                self.client.ptz_lease.acquire(flags)
                baseline = self._pose()
                self.client.sequence_pose_guard = lambda: self._pose(baseline)
                if math.hypot(baseline['x']-shot.x_cal, baseline['y']-shot.y_cal) > self.settings['position_tolerance_m']:
                    raise PatrolError('INSPECTION_OFF_CALIBRATION', '距标定位置超过允许范围，请重新停靠')
                target = dict(pan=shot.target_pan(baseline['yaw']), tilt=float(shot.tilt), zoom=float(shot.zoom))
                metadata.update(robot_pose=baseline, target=target)
                progress(f'视角 {index}/{len(shots)} {shot.name}：定位至 {target["pan"]:.2f}° / {shot.tilt:.2f}° / {shot.zoom:.1f}×')
                actual = self._get(flags)
                deadline = time.monotonic() + self.settings['arrival_timeout_sec']
                # Finish horizontal positioning before changing the other head
                # axis; this avoids relying on simultaneous absolute-axis moves.
                pan_target = dict(pan=target['pan'], tilt=actual['tilt'], zoom=actual['zoom'])
                motion_sent = True
                self._set(pan_target, flags)
                self._wait_arrival(pan_target, baseline, deadline, flags)
                self._set(target, flags)
                actual = self._wait_arrival(target, baseline, deadline, flags)
                self.client._wait(shot.settle_seconds, flags)
                self._pose(baseline)
                actual = self._get(flags)
                if not self._matches(actual, target):
                    raise PatrolError('INSPECTION_PTZ_NOT_ARRIVED', '抓拍前实际位置偏离目标')
                metadata['actual'] = actual
                directory.mkdir(parents=True, exist_ok=True)
                attempt = new_id('attempt')
                for capture_index in range(1, 4):
                    self.client._check_control(flags)
                    self._pose(baseline)
                    image_path = str(directory / f'{index:02d}_{shot.shot_id}_{attempt}_try{capture_index}.jpg')
                    response = self.client._call(self.client.capture,
                        self.client._CapturePicture.Request(quality=0, save_path=image_path),
                        self.client.capture_timeout, '多视角抓拍', flags)
                    if response.success and Path(image_path).is_file() and Path(image_path).stat().st_size:
                        break
                    if capture_index == 3:
                        raise PatrolError('INSPECTION_CAPTURE_FAILED', response.message or '抓拍文件未保存')
                    self.client._wait(.2, flags)
                self._pose(baseline)
                self.client._check_control(flags)
                if not self._matches(self._get(flags), target):
                    raise PatrolError('INSPECTION_PTZ_NOT_ARRIVED', '抓拍期间云台实际位置偏离目标')
                metadata['image_path'] = image_path
                message = f'视角 {shot.name}：拍摄完成（仅存图）'
                images.append(image_path)
            except Exception as error:
                error_code = error.code if isinstance(error, PatrolError) else 'INSPECTION_FAILED'
                message = error.message if isinstance(error, PatrolError) else str(error)
                if motion_sent:
                    metadata['stop_confirmed'] = self._stop_motion()
                    if not metadata['stop_confirmed']:
                        message += '；云台停止尚未确认，禁止恢复执勤'
                        if error_code == 'PAUSED':
                            error_code = 'INSPECTION_STOP_UNCONFIRMED'
                message = f'视角 {shot.name}：{message}'
            self.client.sequence_pose_guard = None
            metadata.update(success=not bool(error_code), error_code=error_code, message=message,
                            image_path=image_path,
                            last_actual=self.last_actual,
                            feedback_received_at_ns=self.last_feedback_received_at_ns,
                            started_at_ns=started, finished_at_ns=now_ns())
            directory.mkdir(parents=True, exist_ok=True)
            manifest = directory / f'{index:02d}_{shot.shot_id}_{new_id("result")}.json'
            manifest.write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
            self.client.store.add_inspection_result(InspectionResult(
                result_id=new_id('inspection'), run_id=run_id, map_id=map_id,
                point_id=step.point_id, point_name=step.point_name, loop_index=loop_index,
                step_index=step_index, started_at_ns=started, finished_at_ns=metadata['finished_at_ns'],
                ptz_preset=0, prompt='', image_path=image_path, text='', model='', success=not bool(error_code),
                error_code=error_code, message=message, shot_id=shot.shot_id, shot_name=shot.name, metadata=metadata))
            if error_code:
                return InspectionOutcome(False, message, image_path=image_path, error_code=error_code)
            progress(message)
        return InspectionOutcome(True, f'已完成 {len(images)} 个低倍视角拍摄',
                                 text=f'已保存 {len(images)} 张照片', image_path=images[-1])
