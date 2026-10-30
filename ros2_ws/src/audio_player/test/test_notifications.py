from pathlib import Path
import json
import queue
import wave

import pytest

from audio_player.notifications import Playback, PlaybackQueue, PromptRules, load_config

ROOT = Path(__file__).resolve().parents[1]


class Clock:
    now = 100.
    def __call__(self):
        return self.now


@pytest.fixture
def rules():
    clock = Clock()
    result = PromptRules(load_config(ROOT/'config/prompts.json'), clock, lambda: 1700000000.)
    result.test_clock = clock
    return result


def keys(notices):
    return [n.key for n in notices if not n.clear]


def status(state, seq, run='run_a', active=True, instance='executor_a', **values):
    return {**dict(instance_id=instance, run_id=run, state=state, sequence=seq,
                   active=active, current_goal_id=''), **values}


def test_latched_completion_does_not_play_on_startup_and_new_run_finishes_once(rules):
    assert not keys(rules.patrol(status(10, 50, active=False)))
    assert not keys(rules.patrol(status(10, 50, active=False)))
    assert not keys(rules.patrol(status(1, 1, run='run_b')))
    assert keys(rules.patrol(status(3, 2, run='run_b'))) == ['patrol_start']
    assert not keys(rules.patrol(status(3, 3, run='run_b')))
    assert keys(rules.patrol(status(10, 4, run='run_b', active=False))) == ['patrol_completed']
    assert not keys(rules.patrol(status(10, 5, run='run_b', active=False)))
    assert not keys(rules.patrol(status(3, 2, run='run_b')))  # Out-of-order feedback.


def test_pause_resume_cancel_and_return_have_distinct_prompts(rules):
    rules.patrol(status(1, 1))
    rules.patrol(status(3, 2))
    paused = rules.patrol(status(7, 3))
    assert any(n.clear and n.scope == 'nav:run_a' for n in paused)
    assert keys(paused) == ['patrol_paused']
    assert keys(rules.patrol(status(3, 4))) == ['patrol_resumed']
    assert keys(rules.patrol(status(9, 5, active=False))) == ['patrol_canceled']
    rules.patrol(status(1, 1, run='return_one'))
    assert keys(rules.patrol(status(3, 2, run='return_one'))) == ['return_start']
    assert keys(rules.patrol(status(10, 3, run='return_one', active=False))) == ['return_arrived']


def test_failures_and_interruption_cannot_play_success(rules):
    rules.patrol(status(1, 1))
    assert keys(rules.patrol(status(11, 2, error_code='OCR_FAILED', active=False))) == ['inspection_failed']
    rules.patrol(status(1, 1, run='run_b'))
    assert keys(rules.patrol(status(12, 2, run='run_b', active=False))) == ['task_interrupted']


def test_point_events_deduplicate_retries_but_allow_next_loop_and_reject_history(rules):
    rules.patrol(status(1, 1))
    event = dict(event='point_arrived', run_id='run_a', loop_index=1, step_index=1,
                 event_time_ns=1700000000*10**9)
    assert keys(rules.patrol_event(event)) == ['point_arrived']
    assert not keys(rules.patrol_event(event))
    assert keys(rules.patrol_event(dict(event, loop_index=2))) == ['point_arrived']
    assert not keys(rules.patrol_event(dict(event, step_index=2, event_time_ns=1699999900*10**9)))
    assert not keys(rules.patrol_event(dict(event, run_id='old_run')))
    assert keys(rules.patrol_event(dict(event, event='inspection_started'))) == ['point_inspection_start']
    assert keys(rules.patrol_event(dict(event, event='inspection_completed'))) == ['point_inspection_done']
    report = rules.patrol_event(dict(event, event='report_ready'))
    assert keys(report) == ['report_ready'] and report[0].scope == 'report:run_a'


def test_person_events_have_freshness_dedup_and_thirty_second_cooldown(rules):
    event = dict(event_type='person', event_id='one', event_time_ns=1700000000*10**9)
    assert keys(rules.vision(event)) == ['person_detected']
    assert not rules.vision(dict(event, event_id='two'))
    rules.test_clock.now += 31
    assert not rules.vision(event)
    assert keys(rules.vision(dict(event, event_id='two'))) == ['person_detected']
    rules.test_clock.now += 31
    assert not rules.vision(dict(event, event_type='face_capture'))
    assert not rules.vision(dict(event, event_id='old', event_time_ns=1699999900*10**9))


def test_invalid_battery_and_threshold_noise_do_not_announce(rules):
    for value in (float('nan'), float('inf'), -1, 101, True, None):
        assert not rules.battery(value)
    assert not rules.battery(80)
    assert keys(rules.battery(20)) == ['battery_low']
    rules.test_clock.now += 150
    assert not rules.battery(21) and not rules.battery(20)
    assert keys(rules.battery(10)) == ['battery_critical']
    assert not rules.battery(11)


def test_gas_alarm_uses_fresh_device_alarm_not_estimated_value(rules):
    assert not rules.gas(dict(state='unknown', value_scaled=999999, sample_time_ns=1700000000*10**9))
    assert not rules.gas(dict(state='high_alarm', sample_time_ns=1699999900*10**9))
    assert keys(rules.gas(dict(state='high_alarm', sample_time_ns=1700000000*10**9))) == ['gas_alarm']
    assert not rules.gas(dict(state='high_alarm', sample_time_ns=1700000000*10**9))


def test_disabled_devices_and_initial_estop_are_not_false_alarms(rules):
    assert not rules.on_estop(True)
    assert keys(rules.on_estop(False)) == ['emergency_released']
    assert keys(rules.on_estop(True)) == ['emergency_stop']
    assert not rules.on_estop(True)
    assert not rules.device('gas', dict(enabled=False, state='error', error='no serial'))
    assert keys(rules.device('gas', dict(enabled=True, state='error', error='lost serial'))) == ['sensor_abnormal']


def test_live_call_suppresses_backlog_and_announces_actual_connection_once(rules):
    assert not rules.call(dict(reserved=False, active=False, call_id=''))
    assert not rules.call(dict(reserved=True, active=False, call_id='one'))
    assert not rules.notify('person_detected', 'person')
    assert keys(rules.call(dict(reserved=True, active=True, call_id='one'))) == ['intercom_started']
    assert not rules.call(dict(reserved=True, active=True, call_id='one'))
    assert keys(rules.call(dict(reserved=False, active=False, call_id='one'))) == ['intercom_ended']
    assert not rules.call(dict(reserved=False, active=False, call_id='one'))


def test_direct_goal_arrival_requires_new_success_and_fresh_standstill(rules):
    assert not rules.nav_status([('old', 4)])
    assert not rules.nav_status([('new', 2)])
    rules.odom(.1, 0)
    assert not rules.nav_status([('new', 4)])
    assert not rules.tick()
    rules.odom(0, 0)
    rules.test_clock.now += .6
    assert keys(rules.tick()) == ['point_arrived']
    assert not rules.tick()
    rules.nav_status([('aborted', 2)])
    assert keys(rules.nav_status([('aborted', 6)])) == ['navigation_failed']


def test_raw_nav_child_success_never_completes_a_patrol_task(rules):
    rules.nav_status([])
    rules.patrol(status(1, 1))
    rules.patrol(status(3, 2, current_goal_id='child'))
    rules.nav_status([('child', 2)])
    assert not rules.nav_status([('child', 4)])
    rules.patrol(status(10, 3, active=False))
    assert not rules.nav_status([('child', 4)])
    assert rules.pending_arrival is None


def test_priority_queue_expires_bounds_evicts_and_clears_only_matching_scope():
    clock = Clock()
    q = PlaybackQueue(2, clock)
    def item(key, priority, scope='run', deadline=110):
        return Playback(key, key, 0, priority, deadline, scope, key, key)
    q.put_nowait(item('normal', 10))
    q.put_nowait(item('other', 10, 'other'))
    q.put_nowait(item('alarm', 100))
    assert q.get_nowait().key == 'alarm'
    q.discard_scope('run')
    assert q.get_nowait().scope == 'other'
    q.put_nowait(item('expired', 100))
    clock.now = 120
    with pytest.raises(queue.Empty):
        q.get_nowait()


def test_all_thirty_assets_are_portable_mono_wav_and_configuration_is_complete():
    config = load_config(ROOT/'config/prompts.json')
    assert len(config['clips']) == 30
    for key, spec in config['clips'].items():
        with wave.open(str(ROOT/'prompts'/spec['file']), 'rb') as audio:
            assert (audio.getnchannels(), audio.getsampwidth(), audio.getframerate()) == (1, 2, 24000)
            assert .5 < audio.getnframes()/audio.getframerate() < 12
    assert config['clips']['emergency_stop']['priority'] > config['clips']['person_detected']['priority']
