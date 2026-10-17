"""Pure patrol interlock state machine.

The ROS wrapper in guard.py owns topics, timers and the reset service. This
module contains no rclpy imports so the safety transitions can be tested with
a deterministic monotonic clock.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable


@dataclass(frozen=True)
class GuardDecision:
    ready: bool
    locked: bool
    fault_code: str
    reason: str
    startup: bool


@dataclass
class _ExecutorSnapshot:
    instance_id: str
    owner_session_id: str
    run_id: str
    state: int
    active: bool
    dispatch_allowed: bool
    stop_unconfirmed: bool
    nav_goal_id: str


@dataclass
class _OperatorSnapshot:
    enabled: bool
    client_id: str


class PatrolGuardState:
    """Latching dead-man interlock for the patrol executor.

    Normal startup begins locked. Once a fresh executor heartbeat reports IDLE
    for ``startup_grace_sec``, the guard becomes ready. While a run is active,
    both the executor and the matching operator heartbeat must remain fresh.
    Any loss latches the lock; only ``reset()`` can clear a latched fault.
    """

    def __init__(self, executor_timeout_sec: float = 1.5,
                 operator_timeout_sec: float = 1.5,
                 startup_grace_sec: float = 1.0,
                 odom_timeout_sec: float = 1.0,
                 stop_linear_threshold: float = 0.03,
                 stop_angular_threshold: float = 0.05,
                 require_operator_heartbeat: bool = True,
                 require_mux_interlock: bool = True,
                 mux_timeout_sec: float = 3.0,
                 unexpected_nav_goal_grace_sec: float = 0.75):
        self.executor_timeout_sec = max(0.1, float(executor_timeout_sec))
        self.operator_timeout_sec = max(0.1, float(operator_timeout_sec))
        self.startup_grace_sec = max(0.0, float(startup_grace_sec))
        self.odom_timeout_sec = max(0.1, float(odom_timeout_sec))
        self.stop_linear_threshold = max(0.0, float(stop_linear_threshold))
        self.stop_angular_threshold = max(0.0, float(stop_angular_threshold))
        self.require_operator_heartbeat = bool(require_operator_heartbeat)
        self.require_mux_interlock = bool(require_mux_interlock)
        self.mux_timeout_sec = max(0.2, float(mux_timeout_sec))
        self.unexpected_nav_goal_grace_sec = max(
            0.1, float(unexpected_nav_goal_grace_sec))

        self._executor: _ExecutorSnapshot | None = None
        self._executor_time = 0.0
        self._operators: Dict[str, tuple[float, _OperatorSnapshot]] = {}
        self._odom_time = 0.0
        self._odom_linear = 0.0
        self._odom_angular = 0.0
        self._mux_time = 0.0

        self._startup_ready = False
        self._startup_idle_since: float | None = None
        self._fault_latched = False
        self._fault_code = "STARTING"
        self._reason = "guard 启动校验中"
        self._active_identity: tuple[str, str] | None = None
        # A replacement executor reporting IDLE is not proof that the old
        # Nav2 goal ended. Keep this requirement latched until the guard has
        # independently observed/cancelled every non-terminal navigation goal.
        self._stale_goal_reconciliation_required = False
        self._observed_active_instance: str | None = None
        self._last_executor_instance: str | None = None
        self._nonterminal_nav_goals = 0
        self._nonterminal_nav_goal_ids: frozenset[str] = frozenset()
        # Startup must not infer "no goal" from a missing/cached status topic.
        # The ROS guard sets this only after an explicit Nav2 cancel-all
        # request has been answered and any returned goals have terminated.
        self._startup_nav_goal_check_passed = False
        # A Nav2 goal accepted while the patrol executor is idle is not an
        # authorized patrol child. Hold the mux briefly while the executor
        # heartbeat catches up, then latch and cancel if ownership never
        # appears.
        self._unexpected_nav_goal_since: float | None = None
        self._unauthorized_nav_goal_cancel_requested = False

        self._locked = True
        self._ready = False

    @property
    def locked(self) -> bool:
        return self._locked

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def fault_latched(self) -> bool:
        return self._fault_latched

    @property
    def fault_code(self) -> str:
        return self._fault_code

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def startup_ready(self) -> bool:
        return self._startup_ready

    @property
    def stale_goal_reconciliation_required(self) -> bool:
        return self._stale_goal_reconciliation_required

    @property
    def nonterminal_nav_goals(self) -> int:
        return self._nonterminal_nav_goals

    @property
    def nonterminal_nav_goal_ids(self) -> frozenset[str]:
        return self._nonterminal_nav_goal_ids

    @property
    def startup_nav_goal_check_passed(self) -> bool:
        return self._startup_nav_goal_check_passed

    @property
    def unauthorized_nav_goal_cancel_requested(self) -> bool:
        return self._unauthorized_nav_goal_cancel_requested

    def mark_startup_nav_goal_check(self, passed: bool, now: float) -> None:
        del now
        self._startup_nav_goal_check_passed = bool(passed)

    def note_stale_nav_goal_detected(self, now: float) -> None:
        """Latch a late/pending goal discovered during startup recovery."""
        del now
        self._stale_goal_reconciliation_required = True
        if not self._fault_latched:
            self._latch(
                "STALE_NAV_GOAL",
                "guard 启动恢复期间检测到迟到或未终止的 Nav2 导航目标")

    def update_executor(self, instance_id: str, owner_session_id: str, run_id: str,
                        state: int, active: bool, dispatch_allowed: bool,
                        now: float, stop_unconfirmed: bool = False,
                        nav_goal_id: str = "") -> None:
        instance = str(instance_id or "")
        snapshot = _ExecutorSnapshot(
            instance, str(owner_session_id or ""), str(run_id or ""),
            int(state), bool(active), bool(dispatch_allowed), bool(stop_unconfirmed),
            str(nav_goal_id or ""),
        )
        if (self._last_executor_instance is not None and
                instance != self._last_executor_instance and
                self._observed_active_instance == self._last_executor_instance):
            # The previous instance was executing a patrol run when it was
            # replaced. An IDLE heartbeat from the replacement cannot prove
            # that the Nav2 child goal died with the old process.
            self._stale_goal_reconciliation_required = True
        self._last_executor_instance = instance
        self._executor = snapshot
        self._executor_time = float(now)
        if self._executor_active(snapshot):
            self._observed_active_instance = instance
            self._startup_idle_since = None

    def update_operator(self, session_id: str, client_id: str, enabled: bool,
                        now: float) -> None:
        session = str(session_id or "")
        if not session:
            return
        self._operators[session] = (
            float(now), _OperatorSnapshot(bool(enabled), str(client_id or "")))

    def update_odom(self, linear: float, angular: float, now: float) -> None:
        linear = float(linear)
        angular = float(angular)
        if not math.isfinite(linear) or not math.isfinite(angular):
            # Invalid feedback must never be treated as a confirmed stop.
            self._odom_time = 0.0
            self._odom_linear = 0.0
            self._odom_angular = 0.0
            return
        self._odom_linear = linear
        self._odom_angular = angular
        self._odom_time = float(now)

    def update_mux_interlock(self, now: float) -> None:
        self._mux_time = float(now)

    def update_nav_goal_status(self, nonterminal_goals: int, now: float,
                               goal_ids: Iterable[str] | None = None) -> None:
        now = float(now)
        self._nonterminal_nav_goals = max(0, int(nonterminal_goals))
        if goal_ids is not None:
            self._nonterminal_nav_goal_ids = frozenset(
                str(goal_id) for goal_id in goal_ids if goal_id)
        elif self._nonterminal_nav_goals == 0:
            self._nonterminal_nav_goal_ids = frozenset()
        if self._nonterminal_nav_goals == 0:
            self._unexpected_nav_goal_since = None
            self._unauthorized_nav_goal_cancel_requested = False
        elif not self._unauthorized_nav_goal_ids(now):
            self._unexpected_nav_goal_since = None
        elif self._unexpected_nav_goal_since is None:
            self._unexpected_nav_goal_since = now

    def evaluate(self, now: float) -> GuardDecision:
        now = float(now)
        if self._fault_latched:
            return self._locked_decision()

        executor_fresh = self._executor is not None and \
            (now - self._executor_time) <= self.executor_timeout_sec
        if not executor_fresh:
            if not self._startup_ready:
                return self._soft_locked("等待有效的巡航执行器心跳")
            return self._latch("EXECUTOR_LOST", "巡航执行器心跳丢失")

        assert self._executor is not None
        active = self._executor_active(self._executor)

        # A stale Nav2 goal while the guard is still validating startup (for
        # example a previous patrol process died but its NavigateToPose goal
        # remained live) is never allowed to release the mux.
        if not self._startup_ready and self._nonterminal_nav_goals > 0:
            return self._latch(
                "STALE_NAV_GOAL",
                "guard 启动时发现 Nav2 仍有未终止的导航目标")

        if self._startup_ready and self._nonterminal_nav_goals > 0:
            unauthorized = self._unauthorized_nav_goal_ids(now)
            if not unauthorized:
                self._unexpected_nav_goal_since = None
            else:
                if self._unexpected_nav_goal_since is None:
                    self._unexpected_nav_goal_since = now
                if (now - self._unexpected_nav_goal_since) < \
                        self.unexpected_nav_goal_grace_sec:
                    return self._ownership_wait_locked(
                        "检测到 Nav2 目标，等待巡航执行器上报匹配的 goal UUID")
                self._unauthorized_nav_goal_cancel_requested = True
                self._stale_goal_reconciliation_required = True
                return self._latch(
                    "UNAUTHORIZED_NAV_GOAL",
                    "Nav2 目标 UUID 与当前巡航子目标所有权不匹配，guard 已锁车并请求取消")
        elif self._nonterminal_nav_goals == 0:
            self._unexpected_nav_goal_since = None
            self._unauthorized_nav_goal_cancel_requested = False

        if self._executor.stop_unconfirmed:
            return self._latch(
                "STOP_UNCONFIRMED",
                "巡航子目标取消后仍未确认终止，guard 已锁车")

        current_instance = self._executor.instance_id
        current_run_id = self._executor.run_id

        # Once a run was observed on an executor instance, a heartbeat from a
        # different instance is never enough to prove the old Nav2 goal ended.
        # This must latch even if the replacement executor reports IDLE.
        if self._active_identity is not None:
            previous_instance, _previous_run_id = self._active_identity
            if current_instance != previous_instance:
                self._stale_goal_reconciliation_required = True
                return self._latch(
                    "EXECUTOR_INSTANCE_CHANGED",
                    "巡航执行器实例在旧任务确认终止前发生变化")

        # The same instance may restart its executor thread while retaining
        # its process identity. That is still a loss of the old goal's owner.
        if (self._observed_active_instance is not None and
                current_instance != self._observed_active_instance):
            self._stale_goal_reconciliation_required = True
            return self._latch(
                "EXECUTOR_INSTANCE_CHANGED",
                "巡航执行器实例在旧任务确认终止前发生变化")

        if not self.require_mux_interlock or (
                self._mux_time > 0.0 and (now - self._mux_time) <= self.mux_timeout_sec):
            mux_ok = True
        else:
            mux_ok = False
        if not mux_ok:
            if not self._startup_ready:
                return self._soft_locked("等待 twist_mux 巡航锁诊断")
            return self._latch(
                "MUX_LOCK_LOST",
                "twist_mux 未报告 /patrol/stop_lock 联锁，guard 保持锁定")

        if active:
            if not self._startup_ready:
                return self._latch(
                    "EXECUTOR_ACTIVE_AT_GUARD_START",
                    "guard 启动时发现巡航执行器仍有活动任务")
            if self._active_identity is None:
                # During validation the executor is active but may not have a
                # run_id yet. A later same-instance heartbeat that fills in the
                # run_id is normal maturation, not a new run.
                self._active_identity = (current_instance, current_run_id)
            else:
                # Same-instance run changes are legitimate back-to-back tasks:
                # the executor owns the child-goal terminal interlock and may
                # have completed the previous task between guard heartbeats.
                self._active_identity = (current_instance, current_run_id)
            if self.require_operator_heartbeat and not self._operator_fresh(
                    self._executor.owner_session_id, now):
                return self._latch(
                    "OPERATOR_LOST",
                    "巡航操作端心跳丢失或与当前任务 owner 不匹配")
            return self._healthy("巡航执行器、操作端与 twist_mux 联锁正常")
        else:
            self._active_identity = None
            if not self._startup_ready:
                if self._startup_idle_since is None:
                    self._startup_idle_since = now
                if not self._startup_nav_goal_check_passed:
                    return self._soft_locked("等待 Nav2 旧导航目标核对结果")
                if (now - self._startup_idle_since) + 1e-9 < self.startup_grace_sec:
                    return self._soft_locked("guard 启动校验中")
                self._startup_ready = True
            return self._healthy("巡航联锁健康")

    def reset(self, now: float, stale_nav_goals_verified_clear: bool = False) -> tuple[bool, str]:
        """Explicit operator reset; never clears while an old run is active.

        ``stale_nav_goals_verified_clear`` must only be set by the ROS guard
        after it has sent a Nav2 cancel-all request and observed that no
        non-terminal navigation goal remains. A replacement executor's IDLE
        heartbeat alone is deliberately insufficient.
        """
        now = float(now)
        executor_fresh = self._executor is not None and \
            (now - self._executor_time) <= self.executor_timeout_sec
        if not executor_fresh:
            return False, "执行器心跳不新鲜，无法确认任务已终止"
        assert self._executor is not None
        if self._executor_active(self._executor):
            return False, "巡航执行器仍有活动任务，不能复位联锁"
        if self._executor.stop_unconfirmed:
            return False, "旧导航目标尚未确认终止，不能复位联锁"
        if ((self._nonterminal_nav_goals > 0 or
             self._stale_goal_reconciliation_required) and
                not stale_nav_goals_verified_clear):
            return False, "尚未终止并核对旧 Nav2 导航目标，不能复位联锁"
        if self.require_mux_interlock and not (
                self._mux_time > 0.0 and (now - self._mux_time) <= self.mux_timeout_sec):
            return False, "twist_mux 巡航锁诊断不新鲜，无法确认联锁可用"
        odom_fresh = self._odom_time > 0.0 and \
            (now - self._odom_time) <= self.odom_timeout_sec
        if not odom_fresh:
            return False, "里程计反馈不新鲜，无法确认车辆已停稳"
        if not math.isfinite(self._odom_linear) or not math.isfinite(self._odom_angular):
            return False, "里程计反馈包含非有限数值，无法确认车辆已停稳"
        if abs(self._odom_linear) > self.stop_linear_threshold or \
                abs(self._odom_angular) > self.stop_angular_threshold:
            return False, "车辆尚未停稳，不能复位联锁"

        self._fault_latched = False
        self._fault_code = ""
        self._reason = "联锁已复位"
        self._startup_ready = True
        self._startup_idle_since = now
        self._active_identity = None
        self._stale_goal_reconciliation_required = False
        self._observed_active_instance = None
        self._nonterminal_nav_goals = 0
        self._nonterminal_nav_goal_ids = frozenset()
        self._startup_nav_goal_check_passed = True
        self._unexpected_nav_goal_since = None
        self._unauthorized_nav_goal_cancel_requested = False
        self._locked = False
        self._ready = True
        return True, "巡航联锁已复位，任务已终止且车辆停稳"

    @staticmethod
    def _executor_active(executor: _ExecutorSnapshot) -> bool:
        return executor.active or bool(executor.run_id) or executor.state != 0

    def _authorized_nav_goal_ids(self, now: float) -> frozenset[str]:
        if self._executor is None:
            return frozenset()
        if (now - self._executor_time) > self.executor_timeout_sec:
            return frozenset()
        if not self._executor_active(self._executor):
            return frozenset()
        if self.require_operator_heartbeat and not self._operator_fresh(
                self._executor.owner_session_id, now):
            return frozenset()
        if self._executor.stop_unconfirmed:
            return frozenset()
        goal_id = self._executor.nav_goal_id
        return frozenset((goal_id,)) if goal_id else frozenset()

    def _unauthorized_nav_goal_ids(self, now: float) -> frozenset[str]:
        if self._nonterminal_nav_goals <= 0:
            return frozenset()
        authorized = self._authorized_nav_goal_ids(now)
        if self._nonterminal_nav_goal_ids:
            return self._nonterminal_nav_goal_ids - authorized
        # A status count without IDs cannot be matched to the executor's
        # concrete Nav2 goal and is therefore never authorized.
        return frozenset(("<unknown>",))

    def _operator_fresh(self, owner_session_id: str, now: float) -> bool:
        if not owner_session_id:
            return False
        item = self._operators.get(owner_session_id)
        if item is None:
            return False
        timestamp, snapshot = item
        return snapshot.enabled and (now - timestamp) <= self.operator_timeout_sec

    def _healthy(self, reason: str) -> GuardDecision:
        self._fault_code = ""
        self._reason = reason
        self._locked = False
        self._ready = True
        return GuardDecision(True, False, "", reason, not self._startup_ready)

    def _ownership_wait_locked(self, reason: str) -> GuardDecision:
        # Keep guard_ready true during the short ownership handshake so the
        # patrol executor does not interrupt its own just-accepted child goal.
        # The twist_mux lock is still active; no velocity can pass.
        code = "NAV_GOAL_OWNERSHIP_PENDING"
        self._fault_code = code
        self._reason = reason
        self._locked = True
        self._ready = True
        return GuardDecision(True, True, code, reason, not self._startup_ready)

    def _soft_locked(self, reason: str) -> GuardDecision:
        self._fault_code = "STARTING"
        self._reason = reason
        self._locked = True
        self._ready = False
        return GuardDecision(False, True, "STARTING", reason, True)

    def _latch(self, code: str, reason: str) -> GuardDecision:
        if (self._active_identity is not None or
                self._observed_active_instance is not None or
                (self._executor is not None and self._executor_active(self._executor)) or
                code in ("STOP_UNCONFIRMED", "EXECUTOR_INSTANCE_CHANGED",
                         "EXECUTOR_ACTIVE_AT_GUARD_START", "STALE_NAV_GOAL",
                         "UNAUTHORIZED_NAV_GOAL")):
            self._stale_goal_reconciliation_required = True
        self._fault_latched = True
        self._fault_code = code
        self._reason = reason
        self._locked = True
        self._ready = False
        return self._locked_decision()

    def _locked_decision(self) -> GuardDecision:
        self._locked = True
        self._ready = False
        return GuardDecision(False, True, self._fault_code, self._reason,
                             not self._startup_ready)
