import json
import queue
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import String

from audio_player.notifications import Playback, PlaybackQueue, load_config
from audio_player.tts_node import TtsNode


def player(tmp_path):
    node = object.__new__(TtsNode)
    node._audio_dir = node._prompt_dir = tmp_path.resolve()
    node._prompt_config = load_config(Path(__file__).resolve().parents[1]/'config/prompts.json')
    node._q = PlaybackQueue()
    node._audio_lock = threading.RLock()
    node._generation = 0
    node._current_item = None
    node._current_cancel = threading.Event()
    node._synth_slots = threading.BoundedSemaphore(2)
    node._play_process = None
    node._shutdown = threading.Event()
    node._notice_seen = {}
    node._scope_versions = {}
    node._intercom_busy = False
    node._intercom_time = 0.
    node._audio_state = {'ready': True, 'state': 'idle'}
    node._pub_audio = node._pub_status = NS(publish=lambda m: None)
    node.get_logger = lambda: NS(warn=lambda m: None, error=lambda m: None)
    for name in ('point_arrived', 'emergency_stop', 'intercom_started', 'patrol_completed'):
        (tmp_path/(name+'.wav')).write_bytes(b'fixture')
    return node


def notify(node, key, token='one', scope='run'):
    node._on_notification(String(data=json.dumps(dict(key=key, token=token, scope=scope))))


def test_alarm_interrupts_current_audio_and_plays_before_common_notice(tmp_path):
    node = player(tmp_path)
    stopped = []
    node._play_process = NS(poll=lambda: None, terminate=lambda: stopped.append(True))
    notify(node, 'point_arrived')
    notify(node, 'emergency_stop')
    assert stopped == [True]
    assert node._current_cancel.is_set()
    assert node._q.get_nowait().key == 'emergency_stop'
    assert node._q.get_nowait().key == 'point_arrived'


def test_scope_clear_cancels_only_that_task_and_ignores_duplicate_token(tmp_path):
    node = player(tmp_path)
    notify(node, 'point_arrived')
    notify(node, 'point_arrived')
    notify(node, 'patrol_completed', scope='other')
    assert node._q.qsize() == 2
    node._current_item = node._q.get_nowait()
    node._on_notification(String(data=json.dumps(dict(clear=True, scope='other'))))
    assert node._current_cancel.is_set()
    assert node._q.get_nowait().scope == 'run'


def test_intercom_blocks_files_text_and_notifications_until_released(tmp_path):
    node = player(tmp_path)
    node._on_intercom_state(String(data='{"reserved":true}'))
    notify(node, 'point_arrived')
    node._on_say(String(data='old text'))
    node._on_audio_play(String(data='point_arrived.wav'))
    assert node._q.empty()
    notify(node, 'intercom_started')
    assert node._q.get_nowait().key == 'intercom_started'
    node._on_intercom_state(String(data='{"reserved":false}'))
    notify(node, 'point_arrived')
    assert node._q.get_nowait().key == 'point_arrived'


def test_clear_racing_with_worker_dequeue_cannot_play_a_stale_task(tmp_path, monkeypatch):
    node = player(tmp_path)
    notify(node, 'point_arrived')
    original_get = node._q.get
    fetched, release = threading.Event(), threading.Event()
    def get(timeout=None):
        item = original_get(timeout)
        fetched.set()
        assert release.wait(2)
        return item
    node._q.get = get
    played = []
    node._play = lambda path, generation: played.append(path)
    monkeypatch.setattr('audio_player.tts_node.rclpy.ok', lambda: True)
    worker = threading.Thread(target=node._worker)
    worker.start()
    try:
        assert fetched.wait(1)
        node._on_notification(String(data='{"clear":true,"scope":"run"}'))
        release.set()
        time.sleep(.05)
    finally:
        node._shutdown.set()
        release.set()
        worker.join(1)
    assert not worker.is_alive() and not played


def test_unknown_notification_cannot_supply_an_arbitrary_path(tmp_path):
    node = player(tmp_path)
    notify(node, '../outside')
    assert node._q.empty()


def test_colcon_symlink_installed_asset_is_accepted(tmp_path):
    node = player(tmp_path)
    installed = tmp_path/'install'
    installed.mkdir()
    (installed/'point_arrived.wav').symlink_to(tmp_path/'point_arrived.wav')
    node._prompt_dir = installed
    notify(node, 'point_arrived')
    assert Path(node._q.get_nowait().path).is_file()


def test_slow_online_synthesis_cannot_delay_an_emergency_clip(tmp_path, monkeypatch):
    node = player(tmp_path)
    entered, finish_synth, alarm_played = threading.Event(), threading.Event(), threading.Event()
    def synth(text):
        entered.set()
        assert finish_synth.wait(2)
        return str(tmp_path/'point_arrived.wav')
    node._synth = synth
    node._play = lambda path, generation: alarm_played.set() if path.endswith('emergency_stop.wav') else None
    monkeypatch.setattr('audio_player.tts_node.rclpy.ok', lambda: True)
    node._on_say(String(data='Slow optional online synthesis'))
    worker = threading.Thread(target=node._worker)
    worker.start()
    try:
        assert entered.wait(1)
        notify(node, 'emergency_stop')
        assert alarm_played.wait(.5)
    finally:
        finish_synth.set()
        node._shutdown.set()
        worker.join(1)
    assert not worker.is_alive()
