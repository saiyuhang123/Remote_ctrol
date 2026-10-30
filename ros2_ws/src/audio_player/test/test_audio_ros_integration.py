"""Real DDS and ffplay decoder, dummy sound output, isolated domain only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from patrol_interfaces.msg import PatrolStatus


pytestmark = pytest.mark.skipif(os.environ.get('ROBOT_AUDIO_INTEGRATION') != '1',
                                reason='explicit isolated audio integration required')
if os.environ.get('ROBOT_AUDIO_INTEGRATION') == '1':
    assert os.environ.get('ROS_DOMAIN_ID') == '219'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'


@pytest.fixture(scope='module')
def system(tmp_path_factory):
    rclpy.init()
    node = rclpy.create_node('notification_test_source')
    states, notices = [], []
    node.create_subscription(String, '/audio/status', lambda m: states.append(json.loads(m.data)), 50)
    node.create_subscription(String, '/audio/notify', lambda m: notices.append(json.loads(m.data)), 50)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {
        'patrol': node.create_publisher(PatrolStatus, '/patrol/status', latched),
        'events': node.create_publisher(String, '/patrol/events', 20),
        'vision': node.create_publisher(String, '/vision/events', 20),
        'call': node.create_publisher(String, '/audio/intercom_state', latched),
        'notify': node.create_publisher(String, '/audio/notify', 20),
    }
    logs, children = [], []
    def start():
        log = (tmp_path_factory.mktemp('audio')/'service.log').open('w+')
        logs.append(log)
        env = dict(os.environ, SDL_AUDIODRIVER='dummy', SDL_VIDEODRIVER='dummy', ROBOT_ENABLE_PROMPTS='1')
        for key in ('ROBOT_PROMPT_CONFIG', 'ROBOT_PROMPT_DIR'):
            env.pop(key, None)
        process = subprocess.Popen([sys.executable, '-m', 'audio_player.tts_node'],
                                    stdout=log, stderr=subprocess.STDOUT, env=env)
        children.append(process)
        return process
    def until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)
            if predicate():
                return
        for log in logs:
            log.flush();log.seek(0)
            print(log.read())
        raise AssertionError('Timed out waiting for audio state: '+repr(states[-3:]))
    def send(name, data):
        pubs[name].publish(String(data=json.dumps(data)) if isinstance(data, dict) else data)
    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=3)
    process = start()
    try:
        until(lambda: bool(states) and all(p.get_subscription_count() for k,p in pubs.items() if k != 'call'))
        yield node, states, notices, send, until, start, stop, process
    finally:
        for process in children:
            stop(process)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


def test_real_patrol_and_person_events_play_the_bundled_files(system):
    node, states, notices, send, until, *_ = system
    send('patrol', PatrolStatus(instance_id='voice_fixture', run_id='run_voice', state=1,
                               sequence=1, active=True))
    # Separate source samples so the executor can observe both transitions.
    for _ in range(8):
        rclpy.spin_once(node, timeout_sec=.02)
    send('patrol', PatrolStatus(instance_id='voice_fixture', run_id='run_voice', state=3,
                               sequence=2, active=True))
    until(lambda: any(s.get('filename') == 'patrol_start.wav' and s.get('state') == 'done' for s in states), 9)
    point = dict(event='point_arrived', instance_id='voice_fixture', run_id='run_voice',
                 loop_index=1, step_index=1, event_time_ns=time.time_ns())
    send('events', point)
    send('events', point)  # Same factual event twice must not replay the clip.
    until(lambda: any(s.get('filename') == 'point_arrived.wav' and s.get('state') == 'done' for s in states))
    send('patrol', PatrolStatus(instance_id='voice_fixture', run_id='run_voice', state=10,
                               sequence=3, active=False))
    until(lambda: any(s.get('filename') == 'patrol_completed.wav' and s.get('state') == 'done' for s in states))
    send('vision', dict(event_type='person', event_id='person_fixture', event_time_ns=time.time_ns()))
    until(lambda: any(s.get('filename') == 'person_detected.wav' and s.get('state') == 'done' for s in states))
    assert not any(s.get('state') == 'error' for s in states)
    assert sum(n.get('key') == 'point_arrived' for n in notices) == 1
    assert sum(s.get('filename') == 'point_arrived.wav' and s.get('state') == 'playing' for s in states) <= 3
    # The observer and player publish no vehicle motion or estop commands.
    for name in ('tts_node', 'robot_voice_notifications'):
        topics = dict(node.get_publisher_names_and_types_by_node(name, '/'))
        assert not {'/cmd_vel', '/cmd_vel_auto', '/cmd_vel_nav', '/emergency_stop'} & topics.keys()


def test_intercom_reservation_blocks_ordinary_prompt_but_allows_connected_cue(system):
    _, states, notices, send, until, *_ = system
    baseline = len(states)
    send('call', dict(reserved=False, active=False, call_id=''))
    send('call', dict(reserved=True, active=False, call_id='test_call'))
    # Observe the reservation first; no arbitrary sleeps against hardware.
    until(lambda: any(s.get('request_id') == 'intercom' for s in states[baseline:]))
    send('notify', dict(key='point_arrived', token='muted_point', scope='muted'))
    send('call', dict(reserved=True, active=True, call_id='test_call'))
    until(lambda: any(s.get('filename') == 'intercom_started.wav' and s.get('state') == 'done' for s in states[baseline:]))
    send('call', dict(reserved=False, active=False, call_id='test_call'))
    until(lambda: any(s.get('filename') == 'intercom_ended.wav' and s.get('state') == 'done' for s in states[baseline:]))
    assert not any(s.get('filename') == 'point_arrived.wav' and s.get('state') == 'playing' for s in states[baseline:])


def test_two_launchers_share_one_player_and_standby_takes_over(system):
    node, states, _, _, until, start, stop, first = system
    second = start()
    try:
        # Give the standby a chance to either take the lock or incorrectly join DDS.
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)
        until(lambda: len(node.get_publishers_info_by_topic('/audio/status')) == 1, 5)
        stop(first)
        assert first.returncode == 0
        baseline = len(states)
        until(lambda: any(s.get('state') == 'idle' for s in states[baseline:]), 10)
        assert second.poll() is None
        assert len(node.get_publishers_info_by_topic('/audio/status')) == 1
    finally:
        stop(second)
        assert second.returncode == 0
