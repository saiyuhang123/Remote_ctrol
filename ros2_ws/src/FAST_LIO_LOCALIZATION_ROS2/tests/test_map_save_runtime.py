"""Exercise real FAST-LIO save/export/stop/reload with synthetic sensors.

Run after building and sourcing the workspace. Uses ROS domain 211 and temporary
maps; starts no physical sensor/chassis drivers or motion control bridge.
"""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = "211"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[4]


def stop(process):
    if process is None or process.poll() is not None:
        return
    # Same first signal as the UI: let the launch process forward it once.
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


class SaveRuntimeTest(unittest.TestCase):
    def test_save_export_stop_and_reload(self):
        with tempfile.TemporaryDirectory(prefix="fastlio-save-runtime-") as directory:
            root = Path(directory)
            mapping_prefix = root / "map"
            # The current bringup reserves /map_save for loop-closure output.
            # This stationary sensor fixture exercises FAST-LIO's raw snapshot;
            # it cannot generate the moving keyframes required by loop closure.
            prefix = root / "map_fastlio_raw"
            log = (root / "runtime.log").open("w+")
            processes = []
            rclpy.init(args=[])
            node = rclpy.create_node("fastlio_map_save_fixture")
            try:
                imu = node.create_publisher(Imu, "/imu/data", 50)
                lidar = node.create_publisher(PointCloud2, "/velodyne_points", 10)
                angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
                radius = np.minimum(4. / np.maximum(np.abs(np.cos(angles)), 1e-6),
                                    3. / np.maximum(np.abs(np.sin(angles)), 1e-6))
                points = np.zeros((720, 16), dtype=[
                    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                    ("intensity", "<f4"), ("time", "<f4"), ("ring", "<u2"), ("pad", "u2")])
                points["x"] = (radius * np.cos(angles))[:, None]
                points["y"] = (radius * np.sin(angles))[:, None]
                points["z"] = np.linspace(-1., 1., 16)[None, :]
                points["intensity"] = 1.
                points["time"] = np.linspace(0., .09, 720)[:, None]
                points["ring"] = np.arange(16)[None, :]
                cloud = PointCloud2()
                cloud.header.frame_id = "velodyne"
                cloud.height, cloud.width = 1, points.size
                cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                                for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12), ("time", 16))]
                cloud.fields.append(PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1))
                cloud.point_step = 24
                cloud.row_step = 24 * cloud.width
                cloud.is_dense = True
                cloud.data = points.tobytes()

                def publish_imu():
                    message = Imu()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = "imu_link"
                    message.linear_acceleration.z = 9.80665
                    message.orientation.w = 1.
                    imu.publish(message)

                def publish_cloud():
                    cloud.header.stamp = node.get_clock().now().to_msg()
                    lidar.publish(cloud)

                timers = [node.create_timer(.01, publish_imu), node.create_timer(.1, publish_cloud)]
                mapping = subprocess.Popen([
                    "ros2", "launch", "chassis_nav2", "mapping.launch.py",
                    "start_sensors:=false", "start_chassis:=false", "enable_control:=false",
                    "rviz:=false", "pcd_map:=" + str(mapping_prefix) + ".pcd"],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                processes.append(mapping)

                def wait_for(condition, seconds=15):
                    deadline = time.monotonic() + seconds
                    while time.monotonic() < deadline:
                        if condition():
                            return
                        self.assertIsNone(mapping.poll(), "Mapping exited early")
                        rclpy.spin_once(node, timeout_sec=.02)
                    self.fail("Timed out waiting for runtime condition")

                received = []
                node.create_subscription(PointCloud2, "/cloud_registered", received.append, 10)
                service = node.create_client(Trigger, "/fastlio/raw_map_save")
                wait_for(lambda: len(received) >= 15 and service.service_is_ready())
                result = service.call_async(Trigger.Request())
                wait_for(result.done)
                self.assertTrue(result.result().success, result.result().message)
                self.assertGreater(Path(str(prefix) + ".pcd").stat().st_size, 0)
                exporter = subprocess.Popen([
                    "python3", "-u", str(ROOT / "desktop_ui/scripts/export_fastlio_map.py"),
                    "--prefix", str(prefix), "--config", str(ROOT / "desktop_ui/config/fastlio_map_export.yaml")],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                processes.append(exporter)
                wait_for(lambda: exporter.poll() is not None, 30)
                self.assertEqual(exporter.returncode, 0)
                paths = [Path(str(prefix) + suffix) for suffix in (".pcd", ".pgm", ".yaml")]
                snapshot = [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths]
                count = len(received)
                # Accumulate more points after the explicit snapshot. Shutdown
                # must keep the PCD that was used to generate its PGM/YAML.
                wait_for(lambda: len(received) >= count + 10)
                for timer in timers:
                    timer.cancel()
                stop(mapping)
                self.assertEqual(snapshot, [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths])

                maps = []
                map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=ReliabilityPolicy.RELIABLE)
                node.create_subscription(PointCloud2, "/global_map", maps.append, map_qos)
                publisher = subprocess.Popen([
                    "ros2", "run", "fast_lio_localization", "global_map_publisher.py", "--ros-args",
                    "-p", "map_file_path:=" + str(prefix) + ".pcd", "-p", "interval:=1"],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                processes.append(publisher)
                deadline = time.monotonic() + 15
                while not maps and time.monotonic() < deadline:
                    self.assertIsNone(publisher.poll(), "Saved PCD could not be loaded")
                    rclpy.spin_once(node, timeout_sec=.05)
                self.assertTrue(maps, "Saved PCD was not republished")
                self.assertGreater(maps[0].width, 0)
                self.assertEqual(maps[0].header.frame_id, "map")
                print(f"Save/export/stop/reload passed: {paths[0].stat().st_size} bytes, "
                      f"{maps[0].width} loaded map points; all three snapshot hashes unchanged", flush=True)
            except BaseException:
                log.flush()
                print((root / "runtime.log").read_text(errors="replace")[-16000:], flush=True)
                raise
            finally:
                for process in reversed(processes):
                    stop(process)
                node.destroy_node()
                rclpy.try_shutdown()
                log.close()


if __name__ == "__main__":
    unittest.main()
