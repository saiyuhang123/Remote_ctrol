#!/usr/bin/env python3
"""Terminal WASD control for the JD03 vendor driver's /ecu interface."""
import argparse
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import traceback
import tty

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
import yaml
from yunle_msgs.msg import Ecu, VehicleStatus


class KeyboardState:
    def __init__(self, speed=.1, turn_angle=10., steering_sign=-1., key_timeout=.6,
                 four_wheel_steering=False, rear_steering_sign=-1.):
        self.speed, self.turn_angle = speed, turn_angle
        self.steering_sign, self.key_timeout = steering_sign, key_timeout
        self.four_wheel_steering = four_wheel_steering
        self.rear_steering_sign = rear_steering_sign
        self.direction, self.steering, self.gear = 0, 0., Ecu.SHIFT_D
        self.last_key = None
        self.escape = 0

    def stop(self):
        self.direction, self.steering, self.last_key = 0, 0., None

    def key(self, key, now):
        # Consume arrow/function-key sequences; they must not become WASD keys.
        if key == '\x1b':
            self.stop()
            self.escape = 1
            return True
        if self.escape:
            if self.escape == 1 and key in '[O':
                self.escape = 2
            elif self.escape == 1 or '@' <= key <= '~':
                self.escape = 0
            return True
        key = key.lower()
        if key in ('q', '\x03', '\x04'):
            self.stop()
            return False
        if key == ' ':
            self.stop()
        elif key in 'wasdx':
            if key == 'w':
                self.direction = 1
            elif key == 's':
                self.direction = -1
            elif key == 'a':
                self.steering = self.turn_angle
            elif key == 'd':
                self.steering = -self.turn_angle
            else:
                self.steering = 0.
            self.last_key = now
        return True

    def command(self, now, feedback_ready, feedback_speed=0., autodrive=True):
        reason = ''
        if not feedback_ready:
            reason = '等待底盘反馈 / 反馈已中断'
        elif not autodrive:
            reason = '底盘自动驾驶按钮未启用'
        elif self.last_key is None or now - self.last_key >= self.key_timeout:
            reason = '刹车：无按键输入'
        elif self.direction and (Ecu.SHIFT_D if self.direction > 0 else Ecu.SHIFT_R) != self.gear:
            if abs(feedback_speed) > .02:
                reason = '刹车：换向前等待车辆停稳，再按 W/S'
        if reason:
            self.stop()
            return 0., 0., self.gear, True, reason
        if self.direction:
            self.gear = Ecu.SHIFT_D if self.direction > 0 else Ecu.SHIFT_R
        motor = self.speed * 3.6 if self.direction else 0.  # /ecu uses km/h, not m/s.
        steer = self.steering * self.steering_sign
        label = '前进' if self.direction > 0 else '倒车' if self.direction < 0 else '停车转向'
        return motor, steer, self.gear, self.direction == 0, f'{label}：{self.speed if self.direction else 0.:.2f} m/s，轮胎转角（左正）{self.steering:+.1f}°'


class KeyboardNode(Node):
    def __init__(self, state):
        super().__init__('jd03_keyboard')
        self.state = state
        self.feedback_time, self.feedback_speed, self.autodrive = None, 0., False
        self.last_display = None
        self.publisher = self.create_publisher(Ecu, '/ecu', 10)
        self.subscription = self.create_subscription(VehicleStatus, '/vehicle_status', self.feedback, 10)
        self.timer = self.create_timer(.05, self.publish_command)

    def feedback(self, msg):
        if math.isfinite(msg.cur_speed) and math.isfinite(msg.cur_steer):
            self.feedback_time = time.monotonic()
            self.feedback_speed, self.autodrive = msg.cur_speed, msg.is_autodrive

    def publish_values(self, motor, steer, gear, brake):
        msg = Ecu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.motor, msg.steer, msg.shift, msg.brake = motor, steer, gear, brake
        msg.set_torque = False
        msg.rear_wheel_flag = self.state.four_wheel_steering
        msg.rear_steer = self.state.rear_steering_sign * steer if self.state.four_wheel_steering else 0.
        self.publisher.publish(msg)

    def publish_command(self):
        if self.count_publishers(self.publisher.topic_name) > 1:
            self.state.stop()
            self.publish_values(0., 0., self.state.gear, True)
            raise RuntimeError('检测到其他 /ecu 发布者；请停止 Nav2 控制桥或其他控制脚本后再测试')
        now = time.monotonic()
        fresh = self.feedback_time is not None and now - self.feedback_time < .5
        motor, steer, gear, brake, display = self.state.command(
            now, fresh, self.feedback_speed, self.autodrive)
        self.publish_values(motor, steer, gear, brake)
        if display != self.last_display:
            print(display, flush=True)
            self.last_display = display

    def finish(self):
        self.state.stop()
        self.timer.cancel()
        # Repeat the final brake while DDS still runs, before destroying the node.
        for _ in range(5):
            self.publish_values(0., 0., self.state.gear, True)
            time.sleep(.05)


def arguments():
    parser = argparse.ArgumentParser(description='JD03 WASD 键盘测试，直接发布 /ecu')
    parser.add_argument('--speed', type=float, default=.1, help='测试速度，m/s（默认 0.1）')
    parser.add_argument('--turn-angle', type=float, default=10., help='A/D 转向角，度（默认 10）')
    parser.add_argument('--key-timeout', type=float, default=.6, help='无新按键时自动刹车，秒（默认 0.6）')
    parser.add_argument('--hardware-file', default=str(Path(get_package_share_directory('chassis_nav2')) / 'config/hardware.yaml'))
    args = parser.parse_args(remove_ros_args()[1:])
    try:
        hw = yaml.safe_load(Path(args.hardware_file).expanduser().read_text())
        max_speed = float(hw['chassis']['max_speed'])
        max_angle = float(hw['robot_geometry']['max_steering_deg'])
        sign = float(hw['chassis']['command_steering_sign'])
        four_wheel_steering = bool(hw['chassis'].get('four_wheel_steering', False))
        rear_sign = float(hw['chassis'].get('rear_steering_sign', -1.))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        parser.error(f'读取底盘配置失败：{error}')
    if not math.isfinite(max_speed) or max_speed <= 0 or not math.isfinite(max_angle) or not 0 < max_angle <= 27 or sign not in (-1., 1.):
        parser.error('底盘配置中的速度上限、转角上限或转向符号无效')
    if not math.isfinite(args.speed) or not 0 < args.speed <= max_speed:
        parser.error(f'--speed 必须在 (0, {max_speed}] m/s 内')
    if not math.isfinite(args.turn_angle) or not 0 < args.turn_angle <= max_angle:
        parser.error(f'--turn-angle 必须在 (0, {max_angle}] 度内')
    if not math.isfinite(args.key_timeout) or not .1 <= args.key_timeout <= 1.:
        parser.error('--key-timeout 必须在 [0.1, 1.0] 秒内')
    if rear_sign not in (-1., 1.):
        parser.error('rear_steering_sign 必须为 -1 或 1')
    if not sys.stdin.isatty():
        parser.error('请在可接收键盘输入的终端中运行')
    return KeyboardState(args.speed, args.turn_angle, sign, args.key_timeout,
                         four_wheel_steering, rear_sign)


def main():
    state = arguments()
    fd = sys.stdin.fileno()
    saved_terminal = termios.tcgetattr(fd)
    saved_int_handler = signal.getsignal(signal.SIGINT)
    saved_term_handler = signal.getsignal(signal.SIGTERM)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stop_requested = False
    def terminate(signum, frame):
        # Do not raise through a running pybind11/rclpy call on Ctrl+C.
        nonlocal stop_requested
        stop_requested = True
        state.stop()
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    try:
        node = KeyboardNode(state)
        print('W 前进 | S 倒车 | A 左转 | D 右转 | X 回正 | 空格刹车 | Q/Ctrl+C 退出', flush=True)
        print(f'A/D 保持 W/S 选定的行驶方向；停车时 A/D 只转向。无新按键 {state.key_timeout:g} 秒后自动刹车。', flush=True)
        if state.four_wheel_steering:
            print(f'四轮转向已启用（rear_steering_sign={state.rear_steering_sign:g}）：A/D 时后轮反相同角度。', flush=True)
        tty.setcbreak(fd)
        running = True
        while running and not stop_requested and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            if stop_requested:
                break
            if select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 64)
                if not data:
                    break
                for key in data.decode(errors='ignore'):
                    running = state.key(key, time.monotonic())
                    if not running:
                        break
                node.publish_command()
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        print(f'已停止：{error}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if node is not None:
                node.finish()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved_terminal)
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
            signal.signal(signal.SIGINT, saved_int_handler)
            signal.signal(signal.SIGTERM, saved_term_handler)
        print('已发送停车刹车指令。', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
