"""Threaded, synchronous patrol state machine with an injectable Nav2 driver."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable
import threading
import time

from .models import PatrolError, RunRecord, RouteSnapshot, new_id, now_ns
from .store import PatrolStore

# Mirrors patrol_interfaces/msg/PatrolStatus.msg.
STATE_IDLE = 0
STATE_VALIDATING = 1
STATE_DISPATCHING = 2
STATE_NAVIGATING = 3
STATE_SETTLING = 4
STATE_DWELLING = 5
STATE_PAUSING = 6
STATE_PAUSED = 7
STATE_CANCELING = 8
STATE_CANCELED = 9
STATE_SUCCEEDED = 10
STATE_FAILED = 11
STATE_INTERRUPTED = 12
STATE_STOPPING = 13


class ControlFlags:
    """Thread-safe pause/cancel intent shared by services and the worker."""

    def __init__(self):
        self._lock = threading.RLock()
        self._pause = False
        self._cancel = False
        self._wake = threading.Event()

    @property
    def pause_requested(self) -> bool:
        with self._lock:
            return self._pause

    @property
    def cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel

    def request_pause(self) -> None:
        with self._lock:
            self._pause = True
        self._wake.set()

    def request_resume(self) -> None:
        with self._lock:
            self._pause = False
        self._wake.set()

    def request_cancel(self) -> None:
        with self._lock:
            self._cancel = True
            self._pause = False
        self._wake.set()

    def wake(self) -> None:
        self._wake.set()

    def wait(self, timeout: float) -> bool:
        result = self._wake.wait(timeout)
        self._wake.clear()
        return result


@dataclass
class EngineResult:
    success: bool
    final_state: int
    run_id: str
    completed_loops: int
    completed_steps: int
    error_code: str = ""
    message: str = ""


class PatrolEngine:
    """Execute a frozen route snapshot one NavigateToPose goal at a time.

    The navigation driver is injected so the same loop can be tested with a
    fake Action server. The driver must implement:

    navigate(step, flags, status_cb) -> str:
        "SUCCEEDED", "PAUSED", "CANCELED", "REJECTED", "ABORTED", "TIMEOUT"
    settle(step, flags, status_cb) -> ("OK", (x, y, yaw)) or
        ("PAUSED", None), ("CANCELED", None), ("TIMEOUT", None)
    cancel_current() -> None
    """

    def __init__(self, store: PatrolStore, snapshot: RouteSnapshot, run_id: str,
                 loop_count: int, owner_session_id: str, driver: Any,
                 status_cb: Callable[[dict[str, Any]], None],
                 settle_timeout: float = 10.0, point_timeout: float = 300.0,
                 min_dispatch_interval: float = 0.5):
        self.store = store
        self.snapshot = snapshot
        self.run_id = run_id
        self.loop_count = int(loop_count)
        self.owner_session_id = owner_session_id
        self.driver = driver
        self.status_cb = status_cb
        self.settle_timeout = max(0.1, float(settle_timeout))
        self.point_timeout = max(1.0, float(point_timeout))
        self.min_dispatch_interval = max(0.0, float(min_dispatch_interval))
        self.flags = ControlFlags()
        self._status_lock = threading.RLock()
        self._status: dict[str, Any] = {
            "state": STATE_VALIDATING, "sequence": 0, "run_id": run_id,
            "route_id": snapshot.route_id, "map_id": snapshot.map_id,
            "loop_index": 0, "total_loops": loop_count,
            "step_index": 0, "step_count": len(snapshot.steps),
            "current_point_name": "", "dwell_remaining": 0.0,
            "error_code": "", "message": "", "active": True,
        }

    @property
    def status(self) -> dict[str, Any]:
        with self._status_lock:
            return dict(self._status)

    def _set_status(self, **changes: Any) -> None:
        with self._status_lock:
            self._status.update(changes)
            self._status["sequence"] = int(self._status.get("sequence", 0)) + 1
            status = dict(self._status)
        self.status_cb(status)

    def request_pause(self) -> None:
        self.flags.request_pause()
        self.driver.cancel_current()

    def request_resume(self) -> None:
        self.flags.request_resume()

    def request_cancel(self) -> None:
        self.flags.request_cancel()
        self.driver.cancel_current()

    def mark_canceled_before_start(self, message: str = "任务在启动前被取消") -> None:
        """Publish a terminal status for a run canceled before dispatch."""
        self._set_status(state=STATE_CANCELED, active=False,
                         error_code="CANCELED", message=message)

    def _driver_stop_safe(self) -> bool:
        checker = getattr(self.driver, "stop_safe", None)
        if not callable(checker):
            return True
        try:
            return bool(checker())
        except Exception:  # noqa: BLE001 - conservative interlock
            return False

    def _pause_and_wait(self, state: int, **extra: Any) -> str:
        self._set_status(state=state, **extra)
        while self.flags.pause_requested and not self.flags.cancel_requested:
            self.flags.wait(0.1)
        if self.flags.cancel_requested:
            return "CANCELED"
        return "RESUMED"

    def _sleep_interruptible(self, seconds: float) -> str:
        deadline = time.monotonic() + max(0.0, seconds)
        while True:
            if self.flags.cancel_requested:
                return "CANCELED"
            if self.flags.pause_requested:
                return "PAUSED"
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return "OK"
            time.sleep(min(0.1, remaining))

    def _run_dwell(self, step: Any, seconds: float) -> str:
        remaining = max(0.0, float(seconds))
        while remaining > 0.0:
            if self.flags.cancel_requested:
                return "CANCELED"
            if self.flags.pause_requested:
                action = self._pause_and_wait(STATE_PAUSED, dwell_remaining=remaining)
                if action == "CANCELED":
                    return "CANCELED"
                if not hasattr(self.driver, "reconfirm"):
                    continue
                reconfirm = self.driver.reconfirm(step, self.flags, self._set_status)
                if reconfirm == "CANCELED":
                    return "CANCELED"
                if reconfirm != "AT_POINT":
                    self._set_status(message="暂停后当前位置未确认，重新导航当前点")
                    return "REPOSITION"
                continue
            started = time.monotonic()
            self._set_status(state=STATE_DWELLING, dwell_remaining=remaining)
            time.sleep(min(0.1, remaining))
            remaining = max(0.0, remaining - (time.monotonic() - started))
        return "OK"

    def _record_attempt(self, loop_index: int, step_index: int, attempt_index: int,
                        step: Any, result: str, actual: tuple[float, float, float] | None,
                        started_at: int, error_code: str = "") -> None:
        self.store.add_attempt(
            new_id("attempt"), self.run_id, loop_index, step_index, attempt_index, "",
            (step.x, step.y, step.yaw), actual, now_ns(), result, error_code,
            started_at=started_at, finished_at=now_ns(),
        )

    def run(self) -> EngineResult:
        completed_loops = 0
        completed_steps = 0
        loop_index = 0
        try:
            if not self.snapshot.steps:
                raise PatrolError("INVALID_DATA", "路线没有可执行步骤")
            while True:
                loop_index += 1
                self._set_status(state=STATE_VALIDATING, loop_index=loop_index,
                                 completed_loops=completed_loops,
                                 completed_steps=completed_steps)
                for step_index, step in enumerate(self.snapshot.steps, start=1):
                    if self.flags.cancel_requested:
                        return self._canceled(completed_loops, completed_steps)
                    attempt_index = 0
                    while True:
                        if self.flags.cancel_requested:
                            return self._canceled(completed_loops, completed_steps)
                        if self.flags.pause_requested:
                            action = self._pause_and_wait(
                                STATE_PAUSED, loop_index=loop_index, step_index=step_index,
                                current_point_name=step.point_name,
                            )
                            if action == "CANCELED":
                                return self._canceled(completed_loops, completed_steps)
                        if not self._driver_stop_safe():
                            return self._failed(
                                completed_loops, completed_steps, "CANCEL_UNCONFIRMED",
                                "上一导航目标尚未确认终止，禁止继续派发新目标")
                        attempt_index += 1
                        started_at = now_ns()
                        self._set_status(state=STATE_DISPATCHING, loop_index=loop_index,
                                         step_index=step_index, current_point_name=step.point_name,
                                         error_code="", message="")
                        result = self.driver.navigate(step, self.flags, self._set_status)
                        if result == "PAUSED":
                            action = self._pause_and_wait(
                                STATE_PAUSED, loop_index=loop_index,
                                step_index=step_index, current_point_name=step.point_name,
                            )
                            if action == "CANCELED":
                                return self._canceled(completed_loops, completed_steps)
                            if not self._driver_stop_safe():
                                return self._failed(
                                    completed_loops, completed_steps,
                                    "CANCEL_UNCONFIRMED",
                                    "旧导航目标尚未确认终止，禁止恢复后续派发")
                            continue
                        if result == "CANCELED":
                            if self.flags.cancel_requested:
                                return self._canceled(completed_loops, completed_steps)
                            self._record_attempt(loop_index, step_index, attempt_index, step,
                                                 "CANCELED", None, started_at, "INTERRUPTED")
                            return self._failed(completed_loops, completed_steps,
                                                "INTERRUPTED", "导航目标在未请求取消时结束")
                        if result != "SUCCEEDED":
                            self._record_attempt(loop_index, step_index, attempt_index, step,
                                                 result, None, started_at, result)
                            return self._failed(completed_loops, completed_steps, result,
                                                f"导航到点失败：{result}")
                        self._set_status(state=STATE_SETTLING, loop_index=loop_index,
                                         step_index=step_index,
                                         current_point_name=step.point_name)
                        settle = self.driver.settle(step, self.flags, self._set_status)
                        if settle[0] == "PAUSED":
                            action = self._pause_and_wait(
                                STATE_PAUSED, loop_index=loop_index,
                                step_index=step_index, current_point_name=step.point_name,
                            )
                            if action == "CANCELED":
                                return self._canceled(completed_loops, completed_steps)
                            settle = self.driver.settle(step, self.flags, self._set_status)
                        if settle[0] == "CANCELED":
                            if self.flags.cancel_requested:
                                return self._canceled(completed_loops, completed_steps)
                            self._record_attempt(loop_index, step_index, attempt_index, step,
                                                 "CANCELED", None, started_at, "INTERRUPTED")
                            return self._failed(completed_loops, completed_steps,
                                                "INTERRUPTED", "停稳等待被外部取消")
                        if settle[0] != "OK":
                            self._record_attempt(loop_index, step_index, attempt_index, step,
                                                 settle[0], None, started_at, settle[0])
                            return self._failed(completed_loops, completed_steps, settle[0],
                                                "Nav2 到点后未在超时内确认停稳")
                        actual = settle[1]
                        self._record_attempt(loop_index, step_index, attempt_index, step,
                                             "SUCCEEDED", actual, started_at)
                        dwell_result = self._run_dwell(step, step.dwell_seconds)
                        if dwell_result == "CANCELED":
                            return self._canceled(completed_loops, completed_steps)
                        if dwell_result == "REPOSITION":
                            continue
                        completed_steps += 1
                        pause = self._sleep_interruptible(self.min_dispatch_interval)
                        if pause == "CANCELED":
                            return self._canceled(completed_loops, completed_steps)
                        if pause == "PAUSED":
                            action = self._pause_and_wait(STATE_PAUSED)
                            if action == "CANCELED":
                                return self._canceled(completed_loops, completed_steps)
                        break
                completed_loops += 1
                self._set_status(state=STATE_VALIDATING, loop_index=loop_index,
                                 completed_loops=completed_loops,
                                 completed_steps=completed_steps)
                if self.loop_count != 0 and completed_loops >= self.loop_count:
                    self._set_status(state=STATE_SUCCEEDED, active=False,
                                     message=f"完成 {completed_loops} 圈")
                    return EngineResult(True, STATE_SUCCEEDED, self.run_id,
                                        completed_loops, completed_steps)
                if self.flags.cancel_requested:
                    return self._canceled(completed_loops, completed_steps)
        except PatrolError as error:
            return self._failed(completed_loops, completed_steps, error.code, error.message)
        except Exception as error:  # noqa: BLE001 - state machine boundary
            return self._failed(completed_loops, completed_steps, "INTERRUPTED", str(error))

    def _canceled(self, loops: int, steps: int) -> EngineResult:
        self._set_status(state=STATE_CANCELED, active=False, message="任务已取消")
        return EngineResult(False, STATE_CANCELED, self.run_id, loops, steps,
                            "CANCELED", "任务已取消")

    def _failed(self, loops: int, steps: int, code: str, message: str) -> EngineResult:
        self._set_status(state=STATE_FAILED, active=False, error_code=code, message=message)
        return EngineResult(False, STATE_FAILED, self.run_id, loops, steps, code, message)
