import os
import subprocess
import threading
import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


pytestmark = pytest.mark.skipif(
    os.environ.get("ROBOT_TWIST_MUX_INTEGRATION") != "1",
    reason="set ROBOT_TWIST_MUX_INTEGRATION=1 to run the real twist_mux test",
)


def test_twist_mux_patrol_lock_deadman_semantics():
    os.environ.setdefault("ROS_DOMAIN_ID", "231")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    executor = None
    spin_thread = None
    node = None
    process = None
    try:
        config = (Path(__file__).resolve().parents[3] / "src" / "chassis_nav2" /
                  "config" / "twist_mux_patrol.yaml")
        assert config.is_file(), config
        process = subprocess.Popen([
            "ros2", "run", "twist_mux", "twist_mux",
            "--ros-args", "--params-file", str(config),
            "-r", "cmd_vel_out:=/twist_mux_test/cmd_vel",
        ], env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        node = Node("twist_mux_interlock_test")
        velocity_pub = node.create_publisher(Twist, "/cmd_vel_auto", 10)
        lock_pub = node.create_publisher(Bool, "/patrol/stop_lock", 10)
        outputs = []

        def on_output(message):
            outputs.append((time.monotonic(), float(message.linear.x)))

        node.create_subscription(Twist, "/twist_mux_test/cmd_vel", on_output, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(1.2)

        def publish_velocity(value=0.2):
            message = Twist()
            message.linear.x = value
            velocity_pub.publish(message)

        def publish_lock(value):
            lock_pub.publish(Bool(data=bool(value)))

        def wait_for_outputs(count, timeout=1.0):
            deadline = time.monotonic() + timeout
            while len(outputs) < count and time.monotonic() < deadline:
                time.sleep(0.02)
            return len(outputs)

        # Before the first guard message, the positive lock timeout must make
        # the mux fail closed.
        publish_velocity()
        time.sleep(0.25)
        assert outputs == []

        # A true lock blocks navigation input.
        publish_lock(True)
        time.sleep(0.1)
        publish_velocity()
        time.sleep(0.25)
        assert outputs == []

        # A fresh false lock releases the interlock and traffic is forwarded.
        publish_lock(False)
        time.sleep(0.1)
        publish_velocity(0.3)
        assert wait_for_outputs(1) >= 1
        assert outputs[-1][1] == pytest.approx(0.3)

        # If the guard dies, the positive timeout re-locks the mux.
        time.sleep(0.7)
        publish_velocity(0.4)
        time.sleep(0.25)
        assert all(value != pytest.approx(0.4) for _, value in outputs)
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if node is not None:
            node.destroy_node()
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
        rclpy.try_shutdown()
