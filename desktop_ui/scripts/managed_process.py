#!/usr/bin/env python3
import fcntl
import hashlib
import os
from dataclasses import dataclass, field
from pathlib import Path
import signal
import subprocess
import sys
import time

NAV_LAUNCH_PACKAGES = {
    'chassis_nav2', 'robot_patrol', 'fast_lio_localization', 'slam_toolbox',
    'cartographer_ros', 'nav2_bringup', 'rslidar_sdk', 'wit_ros2_imu',
}
MANAGED_NAME = 'managed_process.py'


@dataclass(frozen=True)
class Spec:
    key: str
    kind: str
    package: str = ''
    executable: str = ''


@dataclass
class ProcessInfo:
    pid: int
    ppid: int
    pgid: int
    uid: int
    state: str
    args: tuple


@dataclass
class Target:
    pgid: int
    pids: set = field(default_factory=set)
    trusted: bool = False
    leader: int | None = None


def parse_spec(arguments):
    if len(arguments) >= 4 and Path(arguments[0]).name == 'ros2':
        if arguments[1] == 'run':
            package, executable = arguments[2], arguments[3]
            return Spec(f'run:{package}:{executable}', 'run', package, executable)
        if arguments[1] == 'launch':
            package, executable = arguments[2], arguments[3]
            if package == 'chassis_nav2':
                return Spec('robot_navigation_session', 'navigation', package, executable)
            return Spec(f'launch:{package}:{executable}', 'launch', package, executable)
    return Spec('', 'generic')


def sequence(args, expected):
    expected = tuple(expected)
    return any(args[index:index+len(expected)] == expected
               for index in range(len(args)-len(expected)+1))


def managed_arguments(args):
    for index, value in enumerate(args):
        if Path(value).name == MANAGED_NAME and '--' in args[index+1:]:
            split = args.index('--', index+1)
            return args[split+1:]
    return None


def launch_package(args):
    for index, value in enumerate(args[:-1]):
        if Path(value).name == 'ros2' and args[index+1] == 'launch' and index+2 < len(args):
            return args[index+2]
        if Path(value).name == 'launch' and index+1 < len(args):
            return args[index+1]
    return ''


def navigation_node_names():
    return {value for value in os.environ.get('ROBOT_DESKTOP_SESSION_NODES', '').split(',') if value}


def matches(info, spec, script_path, node_names):
    managed = managed_arguments(info.args)
    if managed is not None:
        return parse_spec(managed).key == spec.key
    if spec.kind == 'run':
        if sequence(info.args, ('ros2', 'run', spec.package, spec.executable)):
            return True
        installed = f'/install/{spec.package}/lib/{spec.package}/{spec.executable}'
        return any(installed in value for value in info.args)
    if spec.kind == 'launch':
        return launch_package(info.args) == spec.package and spec.executable in info.args
    if spec.kind != 'navigation':
        return False
    if launch_package(info.args) in NAV_LAUNCH_PACKAGES:
        return True
    for name in node_names:
        if f'__node:={name}' in info.args:
            return True
        if any(Path(value).name in (name, name+'.py') for value in info.args):
            return True
    return False


def read_processes():
    result = {}
    with os.scandir('/proc') as entries:
        for entry in entries:
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                uid = entry.stat().st_uid
                raw = Path(entry.path, 'cmdline').read_bytes()
                stat = Path(entry.path, 'stat').read_text()
                fields = stat[stat.rfind(')')+2:].split()
                args = tuple(value.decode('utf-8', 'replace') for value in raw.split(b'\0') if value)
                result[pid] = ProcessInfo(pid, int(fields[1]), int(fields[2]), uid,
                                          fields[0], args)
            except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError, ValueError):
                continue
    return result


def trusted_launcher(info, spec, script_path, node_names):
    if managed_arguments(info.args) is not None:
        return True
    if 'robot-desktop-session' in info.args:
        return True
    if spec.kind == 'run':
        return sequence(info.args, ('ros2', 'run', spec.package, spec.executable)) and any(
            Path(value).name == 'ros2' for value in info.args)
    if spec.kind in ('launch', 'navigation'):
        return launch_package(info.args) in ({spec.package} | NAV_LAUNCH_PACKAGES) and any(
            Path(value).name in ('ros2', 'launch') for value in info.args)
    return False


def find_targets(spec):
    if not spec.key:
        return []
    own_pid, own_group = os.getpid(), os.getpgid(0)
    script_path = str(Path(__file__).resolve())
    node_names = navigation_node_names()
    processes = read_processes()
    targets = {}
    for info in processes.values():
        if info.uid != os.getuid() or info.pid == own_pid or info.pgid == own_group:
            continue
        if not matches(info, spec, script_path, node_names):
            continue
        target = targets.setdefault(info.pgid, Target(info.pgid))
        target.pids.add(info.pid)
    for target in targets.values():
        leader = processes.get(target.pgid)
        if leader and trusted_launcher(leader, spec, script_path, node_names):
            target.trusted = True
            target.leader = leader.pid
    return list(targets.values())


def alive_targets(targets):
    processes = read_processes()
    remaining = []
    for target in targets:
        if target.trusted:
            if any(info.pgid == target.pgid and info.state != 'Z' for info in processes.values()):
                remaining.append(target)
        elif any(pid in processes and processes[pid].state != 'Z' for pid in target.pids):
            remaining.append(target)
    return remaining


def signal_targets(targets, sig, leaders_only=False):
    processes = read_processes()
    for target in targets:
        if target.trusted:
            if leaders_only:
                leader = processes.get(target.leader)
                if leader and leader.state != 'Z':
                    os.kill(leader.pid, sig)
            else:
                os.kill(-target.pgid, sig)
        else:
            for pid in target.pids:
                if pid in processes and processes[pid].state != 'Z':
                    os.kill(pid, sig)


def wait_targets(targets, timeout):
    deadline = time.monotonic()+timeout
    remaining = targets
    while remaining and time.monotonic() < deadline:
        time.sleep(.1)
        remaining = alive_targets(remaining)
    return remaining


def cleanup_existing(spec):
    targets = find_targets(spec)
    if not targets:
        return
    print(f'[进程监督] {spec.key}：结束 {len(targets)} 组旧实例', flush=True)
    try:
        signal_targets(targets, signal.SIGINT, leaders_only=True)
        remaining = wait_targets(targets, 3.)
        if remaining:
            signal_targets(remaining, signal.SIGTERM)
            remaining = wait_targets(remaining, 2.)
        if remaining:
            signal_targets(remaining, signal.SIGKILL)
            remaining = wait_targets(remaining, 1.)
        if remaining:
            groups = ','.join(str(target.pgid) for target in remaining)
            raise RuntimeError('旧实例未完全退出，进程组：'+groups)
    except PermissionError as error:
        raise RuntimeError('没有权限结束旧实例') from error


def lock_file(path, timeout):
    handle = path.open('a+')
    path.chmod(0o600)
    deadline = time.monotonic()+timeout
    while True:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            return handle
        except BlockingIOError:
            if time.monotonic() >= deadline:
                handle.close()
                raise RuntimeError('等待进程启动锁超时')
            time.sleep(.1)


def navigation_conflicts():
    names = navigation_node_names()
    if not names:
        raise RuntimeError('缺少导航冲突节点配置')
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node(f'managed_process_probe_{os.getpid()}')
    try:
        deadline = time.monotonic()+6.
        clear_since = None
        conflicts = set()
        while time.monotonic() < deadline:
            node_names = {value.rsplit('/', 1)[-1] for value in node.get_node_names()}
            conflicts = node_names & names
            conflicts |= {value for value in node_names if value.startswith('tf_base_link_to_')}
            for topic in ('/cmd_vel', '/odom', '/scan', '/map', '/Odometry', '/cloud_registered',
                          '/patrol/stop_lock', '/patrol/executor_heartbeat'):
                if node.count_publishers(topic):
                    conflicts.add(topic)
            if conflicts:
                clear_since = None
            else:
                if clear_since is None:
                    clear_since = time.monotonic()
                if time.monotonic()-clear_since >= 1.:
                    return
            time.sleep(.15)
        raise RuntimeError('仍有导航冲突：'+', '.join(sorted(conflicts)))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def group_members(exclude=None):
    own_group = os.getpgid(0)
    return [info for info in read_processes().values()
            if info.pgid == own_group and info.state != 'Z' and info.pid not in (exclude or set())]


def signal_members(members, sig):
    for info in members:
        try:
            os.kill(info.pid, sig)
        except ProcessLookupError:
            pass


def supervise(child, stop_requested):
    interrupted_at = None
    terminated = False
    killed = False
    while True:
        code = child.poll()
        if code is None and not stop_requested():
            time.sleep(.1)
            continue
        members = group_members({os.getpid()})
        if code is not None and not members:
            return code
        now = time.monotonic()
        if stop_requested() and interrupted_at is None:
            interrupted_at = now
            try:
                child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        if code is not None and members and not terminated:
            if interrupted_at is None:
                interrupted_at = now
            signal_members(members, signal.SIGTERM)
            terminated = True
        if interrupted_at is not None:
            if not terminated and now-interrupted_at >= 3.:
                signal_members(members, signal.SIGTERM)
                terminated = True
            if not killed and now-interrupted_at >= 6.:
                signal_members(members, signal.SIGKILL)
                killed = True
        time.sleep(.05)


def main(arguments):
    if arguments and arguments[0] == '--':
        arguments = arguments[1:]
    if not arguments:
        print('[进程监督] 缺少启动参数', flush=True)
        return 2
    spec = parse_spec(arguments)
    stop_requested = False
    child = None
    locks = []

    def request_stop(signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    try:
        if spec.key:
            lock_dir = Path(f'/tmp/remote_ctrol_process_{os.getuid()}')
            lock_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
            digest = hashlib.sha256(spec.key.encode()).hexdigest()[:24]
            locks.append(lock_file(lock_dir/(digest+'.startup'), 10.))
            cleanup_existing(spec)
            if stop_requested:
                return 130
            if spec.kind == 'navigation':
                navigation_conflicts()
            if stop_requested:
                return 130
            locks.append(lock_file(lock_dir/(digest+'.instance'), .1))
        child = subprocess.Popen(arguments)
        if len(locks) > 1:
            locks[0].close()
            locks.pop(0)
        code = supervise(child, lambda: stop_requested)
        return code
    except (OSError, RuntimeError) as error:
        print('[进程监督] '+str(error), flush=True)
        if child is not None:
            stop_requested = True
            supervise(child, lambda: True)
        return 1
    finally:
        for handle in locks:
            handle.close()


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
