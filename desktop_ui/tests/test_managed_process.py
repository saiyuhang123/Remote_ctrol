import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import pytest

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/managed_process.py'


def alive(pid):
    try:
        state = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
        return state != 'Z'
    except (FileNotFoundError, ProcessLookupError):
        return False


def wait_gone(pid, timeout=8):
    deadline = time.monotonic()+timeout
    while alive(pid) and time.monotonic() < deadline:
        time.sleep(.05)
    assert not alive(pid), f'process {pid} is still running'


def wait_ready(process, timeout=12):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if select.select([process.stdout], [], [], .2)[0]:
            line = process.stdout.readline()
            if not line:
                break
            if line.startswith('FAKE_READY '):
                _, parent, child = line.split()
                return int(parent), int(child)
        if process.poll() is not None:
            break
    raise AssertionError('managed process did not start: '+process.stdout.read())


def stop_managed(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    process.wait(timeout=10)


@pytest.fixture
def fake_ros(tmp_path):
    signal_log = tmp_path/'signals.log'
    executable = tmp_path/'ros2'
    executable.write_text(f'''#!{sys.executable}
import os, signal, subprocess, sys, time
child = subprocess.Popen([{sys.executable!r}, '-c',
    'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); time.sleep(30)'])
print(f'FAKE_READY {{os.getpid()}} {{child.pid}}', flush=True)
if os.environ.get('FAKE_PARENT_EXIT'):
    sys.exit(0)
def stop(*_):
    with open({str(signal_log)!r}, 'a') as output:
        output.write('interrupt\\n')
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
time.sleep(30)
''')
    executable.chmod(0o755)
    environment = os.environ.copy()
    environment['PATH'] = str(tmp_path)+os.pathsep+environment['PATH']
    environment['PYTHONUNBUFFERED'] = '1'
    environment['FAKE_SIGNAL_LOG'] = str(signal_log)
    processes = []
    def start(extra_env=None):
        env = environment.copy()
        env.update(extra_env or {})
        process = subprocess.Popen([sys.executable, str(SCRIPT), '--', 'ros2', 'run', 'fake_pkg', 'fake_node'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            env=env, start_new_session=True)
        processes.append(process)
        return process
    yield start, signal_log
    for process in processes:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3)


def test_same_identity_replaces_whole_process_tree(fake_ros):
    start, _ = fake_ros
    first = start()
    first_parent, first_child = wait_ready(first)
    second = start()
    second_parent, second_child = wait_ready(second)
    stop_managed(first)
    wait_gone(first_parent)
    wait_gone(first_child)
    assert alive(second_parent) and alive(second_child)
    stop_managed(second)
    wait_gone(second_parent)
    wait_gone(second_child)


def test_stop_interrupts_once_and_leaves_unrelated_process_alone(fake_ros):
    start, signal_log = fake_ros
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
                                 start_new_session=True)
    managed = start()
    parent, child = wait_ready(managed)
    try:
        stop_managed(managed)
        wait_gone(parent)
        wait_gone(child)
        assert unrelated.poll() is None
        assert signal_log.read_text().splitlines() == ['interrupt']
    finally:
        if unrelated.poll() is None:
            os.killpg(unrelated.pid, signal.SIGKILL)
            unrelated.wait(timeout=3)


def test_parent_exit_also_reaps_remaining_child(fake_ros):
    start, _ = fake_ros
    managed = start({'FAKE_PARENT_EXIT': '1'})
    parent, child = wait_ready(managed)
    code = managed.wait(timeout=10)
    assert code == 0, managed.stdout.read()
    wait_gone(parent)
    wait_gone(child)


def test_two_simultaneous_starts_converge_to_one(fake_ros):
    start, _ = fake_ros
    first, second = start(), start()
    running = []
    for process in (first, second):
        try:
            running.append((process, wait_ready(process)))
        except AssertionError:
            if process.poll() is None:
                stop_managed(process)
    assert len(running) == 1
    process, (parent, child) = running[0]
    stop_managed(process)
    wait_gone(parent)
    wait_gone(child)


def test_spec_parsing_distinguishes_run_launch_and_navigation():
    from desktop_ui.scripts.managed_process import parse_spec
    assert parse_spec(['ros2', 'run', 'hk_camera', 'hk_camera_node']).kind == 'run'
    assert parse_spec(['ros2', 'launch', 'other_pkg', 'other.launch.py']).kind == 'launch'
    navigation = parse_spec(['ros2', 'launch', 'chassis_nav2', 'navigation.launch.py'])
    assert navigation.kind == 'navigation'
    assert navigation.key == 'robot_navigation_session'
    assert parse_spec(['python3', '-c', 'pass']).kind == 'generic'


def test_navigation_graph_conflict_blocks_start(monkeypatch):
    pytest.importorskip('rclpy')
    monkeypatch.setenv('ROS_DOMAIN_ID', '201')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROBOT_DESKTOP_SESSION_NODES', 'planner_server,patrol_guard')
    environment = os.environ.copy()
    command = [sys.executable, '-c',
        'from desktop_ui.scripts.managed_process import navigation_conflicts; navigation_conflicts()']
    empty = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=10)
    assert empty.returncode == 0, empty.stdout + empty.stderr
    child = subprocess.Popen([sys.executable, '-c',
        "import rclpy,time; from rclpy.node import Node; rclpy.init(); node=Node('planner_server'); "
        "print('ready',flush=True); time.sleep(30)"],
        env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert child.stdout.readline().strip() == 'ready'
        conflict = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=10)
        assert conflict.returncode == 1
        assert 'planner_server' in conflict.stdout + conflict.stderr
    finally:
        child.kill()
        child.wait(timeout=5)
