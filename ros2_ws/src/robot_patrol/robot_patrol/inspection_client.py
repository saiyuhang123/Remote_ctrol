"""Point inspection actions: PTZ move -> capture -> online recognition."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import threading
import time
from typing import Any, Callable

from .models import InspectionResult, PatrolError, new_id, now_ns
from .ptz_inspection import PtzInspectionLease
from .inspection_prompts import DIGITAL_METER_PROMPT


@dataclass
class InspectionOutcome:
    success: bool
    message: str
    text: str = ""
    image_path: str = ""
    model: str = ""
    error_code: str = ""


class InspectionClient:
    """Synchronous facade usable from the patrol engine worker thread.

    ROS service client callbacks are processed by the PatrolNode's
    MultiThreadedExecutor while this worker waits on futures.
    """

    def __init__(self, node, store, callback_group=None, pose_provider=None, motion_pose_provider=None):
        from hk_camera.srv import CapturePicture, PTZPreset
        from ocr_interfaces.srv import RecognizeImage

        self.node = node
        self.store = store
        self._PTZPreset = PTZPreset
        self._CapturePicture = CapturePicture
        self._RecognizeImage = RecognizeImage

        self.preset = node.create_client(
            PTZPreset, str(node.get_parameter("inspection_preset_service").value),
            callback_group=callback_group)
        self.capture = node.create_client(
            CapturePicture, str(node.get_parameter("inspection_capture_service").value),
            callback_group=callback_group)
        self.recognize = node.create_client(
            RecognizeImage, str(node.get_parameter("inspection_recognition_service").value),
            callback_group=callback_group)

        self.image_dir = Path(str(node.get_parameter("inspection_image_dir").value)).expanduser()
        self.service_timeout = float(node.get_parameter("inspection_service_timeout_sec").value)
        self.capture_timeout = float(node.get_parameter("inspection_capture_timeout_sec").value)
        self.recognition_timeout = float(
            node.get_parameter("inspection_recognition_timeout_sec").value)
        self.preset_settle_sec = float(
            node.get_parameter("inspection_preset_settle_sec").value)
        self.default_prompt = str(
            node.get_parameter("inspection_default_prompt").value).strip() or DIGITAL_METER_PROMPT
        self.ptz_lease = PtzInspectionLease(node, self._call, self._check_control, self.service_timeout, callback_group)
        from .ptz_sequence import PtzShotSequence
        self.sequence = PtzShotSequence(self, callback_group, pose_provider, motion_pose_provider)

    def _check_control(self, flags):
        if flags is not None and flags.cancel_requested:
            raise PatrolError("CANCELED", "巡检动作已取消")
        if flags is not None and flags.pause_requested:
            raise PatrolError("PAUSED", "巡检动作已暂停")
        lease = getattr(self, 'ptz_lease', None)
        if lease is not None:
            lease.check()
        pose_guard = getattr(self, 'sequence_pose_guard', None)
        if pose_guard is not None:
            pose_guard()

    def _prompt_for(self, step):
        return str(getattr(step, 'recognition_prompt', '') or '').strip() or self.default_prompt

    def _wait(self, seconds, flags):
        deadline = time.monotonic() + max(0.0, seconds)
        while True:
            self._check_control(flags)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.05, remaining))

    def _call(self, client, request, timeout: float, label: str, flags=None, on_abandoned=None):
        available_until = time.monotonic() + min(3.0, max(0.1, timeout))
        while True:
            self._check_control(flags)
            if client.wait_for_service(timeout_sec=0.05):
                break
            if time.monotonic() >= available_until:
                raise PatrolError("INSPECTION_UNAVAILABLE", f"{label}服务未就绪")
        self._check_control(flags)
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + max(0.1, timeout)
        try:
            while True:
                self._check_control(flags)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise PatrolError("INSPECTION_TIMEOUT", f"{label}超时（{timeout:.1f}s）")
                if completed.wait(min(0.05, remaining)):
                    self._check_control(flags)
                    break
        except PatrolError:
            # ROS cannot retract a device request. Positioning retains a hook
            # that stops again after a late reply; other operations retire the
            # local future and cannot dispatch subsequent capture/recognition.
            if on_abandoned is not None:
                on_abandoned(future)
            else:
                future.cancel()
                if hasattr(client, "remove_pending_request"):
                    client.remove_pending_request(future)
            raise
        error = future.exception()
        if error is not None:
            raise PatrolError("INSPECTION_FAILED", f"{label}失败：{error}")
        return future.result()

    def run(self, step, run_id: str, map_id: str, loop_index: int, step_index: int,
            status_cb: Callable[..., Any] | None = None, flags=None) -> InspectionOutcome:
        """Run one point inspection, persist the result, and return the outcome."""
        if getattr(step, 'shots', None):
            return self.sequence.run(step, run_id, map_id, loop_index, step_index, status_cb, flags)
        started_at = now_ns()
        prompt = self._prompt_for(step)
        image_path = ""
        text = ""
        model = ""
        response = None
        error_code = ""
        message = ""

        def progress(text_: str) -> None:
            if status_cb is not None:
                try:
                    status_cb(message=text_)
                except Exception:
                    pass

        try:
            self._check_control(flags)
            self.image_dir.mkdir(parents=True, exist_ok=True)
            preset_no = int(getattr(step, "ptz_preset", 0))
            if not 1 <= preset_no <= 255:
                raise PatrolError(
                    "INSPECTION_PRESET_INVALID",
                    "巡检点未配置有效的云台预置位编号（1～255）")

            lease = getattr(self, 'ptz_lease', None)
            if lease is not None:
                progress('正在暂停云台自转，等待停止确认')
                lease.acquire(flags)

            progress(f"正在转到云台预置位 {preset_no}")
            preset_request = self._PTZPreset.Request()
            preset_request.action = 1  # GOTO preset; camera firmware does closed-loop positioning.
            preset_request.preset_no = preset_no
            preset_response = self._call(
                self.preset, preset_request, self.service_timeout, "云台预置位跳转", flags)
            if not preset_response.success:
                raise PatrolError("INSPECTION_PRESET_FAILED",
                                  preset_response.message or "云台预置位跳转失败")
            # This camera does not report absolute pan/tilt. Wait a bounded,
            # configurable settle interval before capturing, then verify the
            # image quality through the recognition step.
            self._wait(self.preset_settle_sec, flags)
            progress("预置位等待完成，正在抓拍")

            loop_dir = self.image_dir / run_id
            loop_dir.mkdir(parents=True, exist_ok=True)
            filename = (f"loop{int(loop_index):02d}_step{int(step_index):03d}_"
                        f"{int(time.time() * 1000)}.jpg")
            image_path = str(loop_dir / filename)
            capture_request = self._CapturePicture.Request()
            capture_request.quality = 0
            capture_request.save_path = image_path
            response = self._call(
                self.capture, capture_request, self.capture_timeout, "相机抓拍", flags)
            if not response.success:
                raise PatrolError("INSPECTION_CAPTURE_FAILED",
                                  response.message or "相机抓拍失败")

            progress("抓拍完成，正在使用 DeepSeek 识别")
            vision_request = self._RecognizeImage.Request()
            vision_request.image = response.image
            vision_request.prompt = prompt
            vision = self._call(
                self.recognize, vision_request, self.recognition_timeout, "DeepSeek 识别", flags)
            if not vision.success:
                raise PatrolError("INSPECTION_RECOGNITION_FAILED",
                                  vision.message or "DeepSeek 识别失败")
            text = vision.text.strip()
            model = vision.model
            if not text:
                raise PatrolError("INSPECTION_RECOGNITION_FAILED", "识别结果为空")
            message = f"识别完成：{text[:300]}"
            outcome = InspectionOutcome(
                success=True, message=message, text=text,
                image_path=image_path, model=model)
        except PatrolError as error:
            error_code, message = error.code, error.message
            outcome = InspectionOutcome(
                success=False, message=message, text=text,
                image_path=image_path, model=model, error_code=error_code)
        except Exception as error:  # noqa: BLE001 - external camera/API boundary
            error_code = "INSPECTION_FAILED"
            message = str(error)
            outcome = InspectionOutcome(
                success=False, message=message, text=text,
                image_path=image_path, model=model, error_code=error_code)

        finished_at = now_ns()
        try:
            self.store.add_inspection_result(InspectionResult(
                result_id=new_id("inspection"),
                run_id=run_id, map_id=map_id,
                point_id=str(getattr(step, "point_id", "")),
                point_name=str(getattr(step, "point_name", "")),
                loop_index=int(loop_index), step_index=int(step_index),
                started_at_ns=int(started_at), finished_at_ns=int(finished_at),
                ptz_preset=int(getattr(step, "ptz_preset", 0)),
                prompt=prompt, image_path=image_path, text=text, model=model,
                success=bool(outcome.success), error_code=error_code, message=message,
            ))
        except Exception as error:  # noqa: BLE001 - persistence must not hide the result
            self.node.get_logger().error(f"保存巡检结果失败：{error}")
        if not outcome.success and error_code not in ("CANCELED", "PAUSED"):
            self.node.get_logger().error(
                f"巡检点动作失败：{error_code}: {message}")
        return outcome

    def depart(self, flags=None):
        self.ptz_lease.depart(flags)

    def finish(self, success=False):
        self.ptz_lease.finish(success)
