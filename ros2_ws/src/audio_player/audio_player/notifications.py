"""Deterministic notification rules and a bounded, expiring playback queue.

No ROS or audio I/O lives here. All deadlines use the injected monotonic clock;
event timestamps use wall time only to reject old, replayed device messages.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import threading
import time


def load_config(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    for key, item in value['clips'].items():
        filename = item.get('file', key+'.wav')
        if Path(filename).name != filename or not filename.endswith('.wav'):
            raise ValueError('Invalid notification filename: '+filename)
        for field in ('priority', 'cooldown', 'ttl'):
            if not isinstance(item[field], (int, float)) or not math.isfinite(item[field]) or item[field] < 0:
                raise ValueError('Invalid notification '+field)
    low, critical = value['battery_low'], value['battery_critical']
    if not 0 <= critical < low <= 100:
        raise ValueError('Require 0 <= battery_critical < battery_low <= 100')
    return value


@dataclass
class Notice:
    key: str
    token: str
    scope: str = ''
    clear: bool = False


@dataclass
class Playback:
    path: str
    request_id: str
    generation: int
    priority: int
    deadline: float
    scope: str
    key: str
    token: str
    scope_version: int = 0


@dataclass
class Speech:
    text: str
    generation: int


class PlaybackQueue:
    """Queue-compatible API for legacy TTS/files plus priority notifications."""
    def __init__(self, maxsize=20, clock=time.monotonic):
        self.maxsize, self.clock = maxsize, clock
        self._condition = threading.Condition()
        self._items = []
        self._sequence = 0

    @staticmethod
    def priority(item):
        return item.priority if isinstance(item, Playback) else 20

    def put_nowait(self, item):
        with self._condition:
            self._prune()
            if len(self._items) >= self.maxsize:
                weakest = min(self._items, key=lambda x: (self.priority(x[1]), x[0]))
                if self.priority(item) <= self.priority(weakest[1]):
                    raise queue.Full()
                self._items.remove(weakest)
            self._sequence += 1
            self._items.append((self._sequence, item))
            self._condition.notify()

    def _prune(self):
        now = self.clock()
        self._items = [(seq, item) for seq, item in self._items
                       if not isinstance(item, Playback) or item.deadline > now]

    def get(self, timeout=None):
        with self._condition:
            deadline = None if timeout is None else self.clock()+timeout
            while True:
                self._prune()
                if self._items:
                    chosen = min(self._items, key=lambda x: (-self.priority(x[1]), x[0]))
                    self._items.remove(chosen)
                    return chosen[1]
                if deadline is not None:
                    remaining = deadline-self.clock()
                    if remaining <= 0:
                        raise queue.Empty()
                else:
                    remaining = None
                self._condition.wait(remaining)

    def get_nowait(self):
        return self.get(0)

    def qsize(self):
        with self._condition:
            self._prune()
            return len(self._items)

    def empty(self):
        return self.qsize() == 0

    def discard_scope(self, scope):
        with self._condition:
            self._items = [(seq, item) for seq, item in self._items
                           if not isinstance(item, Playback) or item.scope != scope]


class PromptRules:
    """Translate observations into notices; never send motion/service commands."""
    def __init__(self, config, clock=time.monotonic, wall=time.time):
        self.config, self.clock, self.wall = config, clock, wall
        self.last = {}
        self.seen = OrderedDict()
        self.run = None
        self.run_sequence = -1
        self.started = set()
        self.finished = set()
        self.retired = OrderedDict()
        self.owned_nav = set()
        self.nav = {}
        self.nav_initialized = False
        self.pending_arrival = None
        self.active = False
        self.odom_time = None
        self.still_since = None
        self.estop = None
        self.battery_band = None
        self.intercom = None
        self.call_id = ''
        self.local_enabled = None
        self.ready_sent = False
        self.healthy = {}

    def notify(self, key, token='', scope=''):
        clip = self.config['clips'].get(key)
        if not clip or not clip.get('enabled', True):
            return []
        if self.intercom and key != 'intercom_started':
            return []  # Do not build a backlog during a live conversation.
        identity = (key, str(token)) if token else None
        if identity and identity in self.seen:
            return []
        now = self.clock()
        if now-self.last.get(key, -math.inf) < clip['cooldown']:
            return []
        self.last[key] = now
        if identity:
            self.seen[identity] = None
            while len(self.seen) > 2048:
                self.seen.popitem(last=False)
        return [Notice(key, str(token), scope)]

    @staticmethod
    def clear(scope):
        return [Notice('', '', scope, True)] if scope else []

    @staticmethod
    def recent(data, wall, seconds=10):
        stamp = data.get('event_time_ns', data.get('time_ns', 0))
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
            return False
        return -2 <= wall-stamp/1e9 <= seconds

    def patrol(self, data):
        run = (str(data.get('instance_id', '')), str(data.get('run_id', '')))
        if not run[1] or run in self.retired:
            return []
        state, seq = int(data.get('state', 0)), int(data.get('sequence', 0))
        scope = 'nav:'+run[1]
        fresh = self.run != run
        if not fresh and seq <= self.run_sequence:
            return []
        notices = []
        if fresh:
            notices += self.clear('nav:'+self.run[1] if self.run else '')
            if self.run:
                self.retired[self.run] = None
                while len(self.retired) > 512:
                    self.retired.popitem(last=False)
            # A latched terminal message at startup is history, not a new event.
            if self.run is None and state not in (1, 2):
                self.started.add(run)
                if state in (9, 10, 11, 12):
                    self.finished.add(run)
            self.run, self.run_sequence = run, -1
            self._previous_state = None
        self.run_sequence = seq
        old = self._previous_state
        self._previous_state = state
        self.active = bool(data.get('active', False))
        goal_id = str(data.get('current_goal_id', ''))
        if goal_id:
            self.owned_nav.add(goal_id)
        returning = run[1].startswith('return_') or run[1] == 'return'
        if state == 3 and run not in self.started:
            self.started.add(run)
            notices += self.notify('return_start' if returning else 'patrol_start', run[1], scope)
        if old == 7 and state not in (6, 7, 8, 9, 11, 12, 13):
            notices += self.notify('patrol_resumed', f'{run[1]}:{seq}', scope)
        if state == 7 and old != 7:
            notices += self.clear(scope)
            notices += self.notify('patrol_paused', f'{run[1]}:{seq}', scope)
        if state in (8, 13):
            notices += self.clear(scope)
        if state in (9, 10, 11, 12) and run not in self.finished:
            self.finished.add(run)
            notices += self.clear(scope)
            key = {9: 'patrol_canceled', 10: 'return_arrived' if returning else 'patrol_completed',
                   11: 'navigation_failed', 12: 'task_interrupted'}[state]
            error = str(data.get('error_code', '')).upper()
            if state == 11 and (old == 14 or any(s in error for s in
                    ('INSPECTION', 'OCR', 'CAPTURE', 'PTZ', 'VISION', 'DEEPSEEK', 'RECOGNITION'))):
                key = 'inspection_failed'
            notices += self.notify(key, run[1], scope)
        # Bound run and goal caches without losing the active identity.
        if len(self.started) > 512:
            self.started = {run}
            self.finished.intersection_update({run})
        if len(self.owned_nav) > 1024:
            self.owned_nav = {goal_id} if goal_id else set()
        return notices

    def patrol_event(self, event):
        if not self.recent(event, self.wall()):
            return []
        run = str(event.get('run_id', ''))
        if self.run and run != self.run[1]:
            return []
        key = {'point_arrived': 'point_arrived', 'inspection_started': 'point_inspection_start',
               'inspection_completed': 'point_inspection_done', 'report_ready': 'report_ready'}.get(event.get('event'))
        token = f"{event.get('instance_id', '')}:{run}:{event.get('loop_index', 0)}:{event.get('step_index', 0)}"
        # Report-ready is emitted only after the report transaction has committed.
        return self.notify(key, token, ('report:' if key == 'report_ready' else 'nav:')+run) if key else []

    def vision(self, data):
        if data.get('event_type') != 'person' or not self.recent(data, self.wall()):
            return []
        identifier = str(data.get('event_id', ''))
        return self.notify('person_detected', identifier) if identifier else []

    def on_estop(self, pressed):
        previous, self.estop = self.estop, bool(pressed)
        if previous is None or previous == self.estop:
            return []
        if self.estop:
            self.pending_arrival = None
            return (self.clear('nav:'+self.run[1] if self.run else '')
                    + self.clear('direct_nav') + self.notify('emergency_stop'))
        return self.notify('emergency_released')

    def battery(self, percent):
        if isinstance(percent, bool) or not isinstance(percent, (int, float)) or not math.isfinite(percent) or not 0 <= percent <= 100:
            return []
        band = 2 if percent <= self.config['battery_critical'] else 1 if percent <= self.config['battery_low'] else 0
        # Hysteresis keeps noisy readings at a threshold from repeatedly rearming.
        if self.battery_band == 2 and percent <= self.config['battery_critical']+3:
            band = 2
        elif self.battery_band == 1 and percent <= self.config['battery_low']+3 and band == 0:
            band = 1
        old, self.battery_band = self.battery_band, band
        return self.notify('battery_critical' if band == 2 else 'battery_low') if band and old != band else []

    def gas(self, data):
        stamp = data.get('sample_time_ns', 0)
        if not self.recent({'time_ns': stamp}, self.wall(), 6):
            return []
        alarm = data.get('state') in ('low_alarm', 'high_alarm')
        previous = self.healthy.get('gas_alarm', False)
        self.healthy['gas_alarm'] = alarm
        return self.notify('gas_alarm') if alarm and not previous else []

    def device(self, name, data):
        # Explicitly stopped/disabled devices are not faults.
        if data.get('enabled') is False or data.get('state') in ('stopped', 'disabled', 'idle'):
            self.healthy[name] = False
            return []
        error = bool(data.get('last_error') or data.get('error'))
        if name == 'gas' and data.get('enabled') and data.get('state') == 'error':
            error = True
        previous = self.healthy.get(name, False)
        self.healthy[name] = error
        return self.notify('camera_abnormal' if name == 'camera' else 'sensor_abnormal') if error and not previous else []

    def guard(self, data):
        if not self.active:
            return []
        code = str(data.get('fault_code', '')).upper()
        if any(x in code for x in ('TF', 'POSE', 'LOCALIZATION')):
            return self.notify('localization_abnormal')
        if any(x in code for x in ('ODOM', 'SENSOR', 'IMU', 'LIDAR')):
            return self.notify('sensor_abnormal')
        return []

    def operator(self, enabled):
        previous, self.local_enabled = self.local_enabled, bool(enabled)
        return self.notify('local_control') if previous is not None and not previous and enabled else []

    def gateway_event(self, event):
        if not self.recent(event, self.wall()):
            return []
        key = {'remote_control': 'remote_control', 'remote_disconnected': 'remote_disconnected',
               'capture_saved': 'capture_saved'}.get(event.get('event'))
        return self.notify(key, event.get('event_id', '')) if key else []

    def call(self, data):
        previous = self.intercom
        self.intercom = bool(data.get('reserved'))
        call_id = str(data.get('call_id', ''))
        notices = []
        if previous is None and data.get('active'):
            self.call_id = call_id
            return []  # Observer restarted during a call; do not replay its opening cue.
        if data.get('active') and call_id and call_id != self.call_id:
            self.call_id = call_id
            notices += self.notify('intercom_started', call_id)
        if previous and not self.intercom and self.call_id:
            notices += self.notify('intercom_ended', self.call_id)
        return notices

    def odom(self, linear, angular):
        now = self.clock()
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.still_since = None
            return
        if self.odom_time is None or now-self.odom_time > 1:
            self.still_since = None
        self.odom_time = now
        if abs(linear) <= .02 and abs(angular) <= .02:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None

    def nav_status(self, statuses):
        notices = []
        first = not self.nav_initialized
        self.nav_initialized = True
        for goal, state in statuses:
            previous = self.nav.get(goal)
            self.nav[goal] = state
            if state in (1, 2, 3) and self.pending_arrival and self.pending_arrival[0] != goal:
                self.pending_arrival = None
            if first or previous is None or state == previous:
                continue
            if self.active or goal in self.owned_nav:
                continue
            if state == 4 and previous in (1, 2):
                self.pending_arrival = (goal, self.clock()+5)
            elif state == 6 and previous in (1, 2):
                notices += self.notify('navigation_failed', goal, 'direct_nav')
        if any(state in (1, 2, 3) for _, state in statuses):
            self.pending_arrival = None
        if len(self.nav) > 512:
            self.nav = dict(statuses)
        return notices

    def tick(self):
        if not self.pending_arrival:
            return []
        goal, until = self.pending_arrival
        now = self.clock()
        if self.active or self.estop or now > until:
            self.pending_arrival = None
            return []
        if self.odom_time is not None and now-self.odom_time <= 1 and self.still_since is not None and now-self.still_since >= .5:
            self.pending_arrival = None
            return self.notify('point_arrived', goal, 'direct_nav')
        return []
