from robot_patrol.guard_logic import PatrolGuardState


def start_ready(state, now=1.0):
    state.update_mux_interlock(now)
    state.mark_startup_nav_goal_check(True, now)
    state.update_executor("executor", "", "", 0, False, True, now)
    decision = state.evaluate(now + 0.1)
    assert decision.locked
    decision = state.evaluate(now + 0.3)
    assert decision.ready and not decision.locked


def test_guard_starts_locked_until_idle_heartbeat_is_stable():
    guard = PatrolGuardState(startup_grace_sec=0.2)
    decision = guard.evaluate(0.0)
    assert decision.locked and not decision.ready
    guard.update_mux_interlock(1.0)
    guard.update_executor("executor", "", "", 0, False, True, 1.0)
    decision = guard.evaluate(1.1)
    assert decision.locked
    assert not guard.startup_nav_goal_check_passed
    # Missing Nav2 status is not proof that there are no old goals.
    decision = guard.evaluate(1.3)
    assert decision.locked and decision.fault_code == "STARTING"
    guard.mark_startup_nav_goal_check(True, 1.3)
    decision = guard.evaluate(1.3)
    assert decision.ready and not decision.locked
    assert decision.fault_code == ""


def test_late_nav_goal_status_after_missing_startup_check_latches():
    guard = PatrolGuardState(startup_grace_sec=0.1)
    guard.update_mux_interlock(1.0)
    guard.update_executor("executor", "", "", 0, False, True, 1.0)
    # Let the old startup grace expire while no Nav2 status was ever received.
    assert guard.evaluate(1.5).locked
    assert not guard.startup_ready
    # The delayed action status reveals the orphan. It must not be able to
    # release the mux merely because the grace timer had already elapsed.
    guard.update_nav_goal_status(1, 1.6)
    decision = guard.evaluate(1.6)
    assert decision.locked
    assert decision.fault_code == "STALE_NAV_GOAL"


def test_active_run_requires_fresh_executor_and_matching_operator_heartbeat():
    guard = PatrolGuardState(executor_timeout_sec=0.5,
                             operator_timeout_sec=0.5,
                             startup_grace_sec=0.2)
    start_ready(guard)
    guard.update_executor("executor", "session-a", "run-a", 3, True, True, 2.0)
    guard.update_operator("session-a", "desktop", True, 2.0)
    decision = guard.evaluate(2.1)
    assert decision.ready and not decision.locked

    # A fresh hb for a different session cannot keep owner session-a alive.
    guard.update_executor("executor", "session-a", "run-a", 3, True, True, 2.4)
    guard.update_operator("session-b", "other-client", True, 2.4)
    decision = guard.evaluate(2.7)
    assert decision.locked and not decision.ready
    assert decision.fault_code == "OPERATOR_LOST"

    # Reconnect alone does not clear the latched interlock.
    guard.update_operator("session-a", "desktop", True, 2.8)
    assert guard.evaluate(2.9).locked
    assert guard.evaluate(2.9).fault_code == "OPERATOR_LOST"


def test_executor_loss_latches_and_explicit_reset_requires_idle_and_stopped():
    guard = PatrolGuardState(executor_timeout_sec=0.5,
                             operator_timeout_sec=0.5,
                             startup_grace_sec=0.2,
                             odom_timeout_sec=0.5)
    start_ready(guard)
    guard.update_executor("executor", "session-a", "run-a", 3, True, True, 2.0)
    guard.update_operator("session-a", "desktop", True, 2.0)
    guard.update_odom(0.8, 0.2, 2.0)
    assert guard.evaluate(2.1).ready

    decision = guard.evaluate(2.6)
    assert decision.locked
    assert decision.fault_code == "EXECUTOR_LOST"

    # Reset is refused while the last known run is still active.
    guard.update_executor("executor", "session-a", "run-a", 3, True, True, 2.7)
    guard.update_operator("session-a", "desktop", True, 2.7)
    success, _ = guard.reset(2.7)
    assert not success

    # Reset is refused while odometry says the vehicle is moving.
    guard.update_executor("executor", "session-a", "", 0, False, True, 2.8)
    guard.update_odom(0.5, 0.0, 2.8)
    success, _ = guard.reset(2.8)
    assert not success

    guard.update_odom(0.0, 0.0, 2.9)
    success, message = guard.reset(2.9)
    assert not success
    assert "Nav2" in message
    success, message = guard.reset(2.9, stale_nav_goals_verified_clear=True)
    assert success, message
    decision = guard.evaluate(2.9)
    assert decision.ready and not decision.locked


def test_guard_starting_while_executor_active_latches_fault():
    guard = PatrolGuardState(startup_grace_sec=0.2)
    guard.update_mux_interlock(1.0)
    guard.update_executor("executor", "session-a", "run-a", 3, True, True, 1.0)
    decision = guard.evaluate(1.1)
    assert decision.locked
    assert decision.fault_code == "EXECUTOR_ACTIVE_AT_GUARD_START"


def test_same_instance_back_to_back_run_is_allowed():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1)
    start_ready(guard)
    guard.update_executor("executor", "owner", "run-1", 3, True, True, 2.0)
    guard.update_operator("owner", "desktop", True, 2.0)
    assert guard.evaluate(2.1).ready
    # The next run starts before a periodic heartbeat exposed the IDLE gap.
    guard.update_executor("executor", "owner", "run-2", 3, True, True, 2.2)
    guard.update_operator("owner", "desktop", True, 2.2)
    decision = guard.evaluate(2.3)
    assert decision.ready and not decision.locked


def test_replacement_executor_reporting_idle_after_old_run_still_latches():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1)
    start_ready(guard)
    guard.update_executor("executor-old", "owner", "run-1", 3, True, True, 2.0)
    guard.update_operator("owner", "desktop", True, 2.0)
    assert guard.evaluate(2.1).ready
    guard.update_executor("executor-new", "", "", 0, False, True, 2.2)
    decision = guard.evaluate(2.3)
    assert decision.locked
    assert decision.fault_code == "EXECUTOR_INSTANCE_CHANGED"
    assert guard.stale_goal_reconciliation_required

    guard.update_odom(0.0, 0.0, 2.3)
    success, message = guard.reset(2.3)
    assert not success
    assert "Nav2" in message
    success, message = guard.reset(2.3, stale_nav_goals_verified_clear=True)
    assert success, message
    assert decision.ready or guard.evaluate(2.4).ready


def test_stop_unconfirmed_latches_immediately():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1)
    start_ready(guard)
    guard.update_executor("executor", "owner", "run-1", 3, True, False, 2.0)
    guard.update_operator("owner", "desktop", True, 2.0)
    assert guard.evaluate(2.1).ready
    guard.update_executor("executor", "owner", "run-1", 13, True, False, 2.2,
                          stop_unconfirmed=True)
    decision = guard.evaluate(2.3)
    assert decision.locked
    assert decision.fault_code == "STOP_UNCONFIRMED"


def test_missing_mux_diagnostics_keeps_guard_locked():
    guard = PatrolGuardState(startup_grace_sec=0.1)
    guard.update_executor("executor", "", "", 0, False, True, 1.0)
    decision = guard.evaluate(1.3)
    assert decision.locked and not decision.ready
    assert decision.fault_code == "STARTING"


def test_nonterminal_nav_goal_holds_guard_locked_until_reconciled():
    guard = PatrolGuardState(startup_grace_sec=0.1, odom_timeout_sec=1.0)
    guard.update_mux_interlock(1.0)
    guard.update_executor("executor", "", "", 0, False, True, 1.0)
    guard.update_nav_goal_status(1, 1.0)
    decision = guard.evaluate(1.3)
    assert decision.locked
    assert decision.fault_code == "STALE_NAV_GOAL"
    assert guard.stale_goal_reconciliation_required

    guard.update_odom(0.0, 0.0, 1.3)
    success, message = guard.reset(1.3)
    assert not success
    assert "Nav2" in message
    guard.update_nav_goal_status(0, 1.4)
    success, message = guard.reset(1.4, stale_nav_goals_verified_clear=True)
    assert success, message


def test_patrol_owned_nav_goal_is_allowed_after_startup():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1)
    start_ready(guard)
    guard.update_executor("executor", "owner", "run-1", 3, True, True, 2.0,
                          nav_goal_id="goal-a")
    guard.update_operator("owner", "desktop", True, 2.0)
    guard.update_nav_goal_status(1, 2.0, goal_ids={"goal-a"})
    decision = guard.evaluate(2.1)
    assert decision.ready and not decision.locked
    assert not guard.unauthorized_nav_goal_cancel_requested


def test_nav_goal_accepted_while_executor_idle_latches_after_grace():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             startup_grace_sec=0.1,
                             unexpected_nav_goal_grace_sec=0.3)
    start_ready(guard)
    guard.update_nav_goal_status(1, 2.0, goal_ids={"goal-a"})
    # The mux is held immediately while the executor heartbeat gets a chance
    # to prove that this is an authorized patrol child. guard_ready remains
    # true during this short handshake so the executor does not interrupt its
    # own just-accepted goal.
    decision = guard.evaluate(2.1)
    assert decision.locked and decision.ready
    assert decision.fault_code == "NAV_GOAL_OWNERSHIP_PENDING"
    decision = guard.evaluate(2.4)
    assert decision.locked and not decision.ready
    assert decision.fault_code == "UNAUTHORIZED_NAV_GOAL"
    assert guard.unauthorized_nav_goal_cancel_requested
    assert guard.stale_goal_reconciliation_required


def test_non_finite_odom_cannot_authorize_reset():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1,
                             odom_timeout_sec=1.0)
    start_ready(guard)
    guard.update_executor("executor", "owner", "run-1", 3, True, True, 2.0)
    guard.update_operator("owner", "desktop", True, 2.0)
    guard.evaluate(2.1)
    guard.update_executor("executor", "owner", "", 0, False, True, 2.1)
    guard.update_odom(float("nan"), 0.0, 2.1)
    success, message = guard.reset(2.1)
    assert not success
    assert "里程计" in message


def test_validation_heartbeat_maturing_into_run_id_is_not_identity_change():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1)
    start_ready(guard)
    guard.update_executor("executor", "owner", "", 2, True, False, 2.0)
    guard.update_operator("owner", "desktop", True, 2.0)
    assert guard.evaluate(2.1).ready

    # The executor heartbeat gained its run_id after validation completed.
    guard.update_executor("executor", "owner", "run-1", 3, True, True, 2.2)
    guard.update_operator("owner", "desktop", True, 2.2)
    decision = guard.evaluate(2.3)
    assert decision.ready and not decision.locked


def test_paused_run_does_not_authorize_a_late_external_nav_goal():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1,
                             unexpected_nav_goal_grace_sec=0.3)
    start_ready(guard)
    # The patrol task stays active but PAUSED, so it owns no Nav2 child goal.
    guard.update_executor("executor", "owner", "run-paused", 7, True, True, 2.0,
                          nav_goal_id="")
    guard.update_operator("owner", "desktop", True, 2.0)
    guard.update_nav_goal_status(1, 2.0, goal_ids={"late-old-goal"})
    decision = guard.evaluate(2.1)
    assert decision.locked and decision.ready
    assert decision.fault_code == "NAV_GOAL_OWNERSHIP_PENDING"
    decision = guard.evaluate(2.4)
    assert decision.locked and not decision.ready
    assert decision.fault_code == "UNAUTHORIZED_NAV_GOAL"
    assert guard.unauthorized_nav_goal_cancel_requested


def test_extra_nav_goal_does_not_match_owned_goal_uuid():
    guard = PatrolGuardState(executor_timeout_sec=2.0,
                             operator_timeout_sec=1.0,
                             startup_grace_sec=0.1,
                             unexpected_nav_goal_grace_sec=0.3)
    start_ready(guard)
    guard.update_executor("executor", "owner", "run-1", 3, True, True, 2.0,
                          nav_goal_id="goal-owned")
    guard.update_operator("owner", "desktop", True, 2.0)
    guard.update_nav_goal_status(
        2, 2.0, goal_ids={"goal-owned", "goal-late-old"})
    decision = guard.evaluate(2.1)
    assert decision.locked and decision.ready
    decision = guard.evaluate(2.4)
    assert decision.fault_code == "UNAUTHORIZED_NAV_GOAL"
