#!/usr/bin/env python3
"""Export a saved FAST-LIO PCD without using the robot's live /map topic."""

import argparse
from contextlib import ExitStack
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import OccupancyGrid
import yaml
from ament_index_python.packages import get_package_prefix


def executable(package, name):
    return str(Path(get_package_prefix(package)) / "lib" / package / name)


def settings(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    values = {key: float(config[key]) for key in
              ("z_min", "z_max", "resolution", "radius", "timeout_sec")}
    neighbors = config["min_neighbors"]
    if (not all(math.isfinite(value) for value in values.values())
            or values["z_min"] >= values["z_max"]
            or any(values[key] <= 0 for key in ("resolution", "radius", "timeout_sec"))
            or isinstance(neighbors, bool) or not isinstance(neighbors, int) or neighbors < 1):
        raise ValueError("切片参数无效，请检查 fastlio_map_export.yaml")
    values["min_neighbors"] = neighbors
    return values


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


def publish_files(stage, prefix, grid):
    """Validate the staged pair; retain the previous pair on ordinary write errors."""
    pgm, metadata_file = stage / "map.pgm", stage / "map.yaml"
    if not pgm.is_file() or pgm.stat().st_size == 0:
        raise RuntimeError("栅格导出未生成有效 PGM")
    with pgm.open("rb") as stream:
        header = []
        while len(header) < 4:
            line = stream.readline()
            if not line:
                raise RuntimeError("PGM 文件头不完整")
            header.extend(line.split(b"#", 1)[0].split())
        if (header != [b"P5", str(grid.info.width).encode(), str(grid.info.height).encode(), b"255"]
                or pgm.stat().st_size - stream.tell() != grid.info.width * grid.info.height):
            raise RuntimeError("PGM 尺寸或数据长度与本次 PCD 投影不符")
    with metadata_file.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if metadata["image"] != "map.pgm" or metadata["mode"] != "trinary":
        raise RuntimeError("Nav2 栅格元数据格式异常")
    # Humble's map saver can serialize origin with only three significant
    # digits. Restore the original grid coordinates instead of shifting the
    # navigation map relative to the PCD by several centimetres.
    metadata["origin"] = [grid.info.origin.position.x, grid.info.origin.position.y, 0.0]
    metadata["resolution"] = float(grid.info.resolution)
    metadata["image"] = prefix.name + ".pgm"
    with metadata_file.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, allow_unicode=True, sort_keys=False)

    # Both files are complete before replacing either existing output. Backups stay
    # on the same filesystem so a failed second replacement can restore the pair.
    outputs = [(pgm, Path(str(prefix) + ".pgm")),
               (metadata_file, Path(str(prefix) + ".yaml"))]
    backups = {}
    for _, target in outputs:
        if target.exists():
            backup = stage / ("previous" + target.suffix)
            shutil.copy2(target, backup)
            backups[target] = backup
    replaced = []
    try:
        for source, target in outputs:
            os.replace(source, target)
            replaced.append(target)
    except BaseException:
        for target in reversed(replaced):
            if target in backups:
                os.replace(backups[target], target)
            else:
                target.unlink(missing_ok=True)
        raise


def export(prefix, config):
    pcd = Path(str(prefix) + ".pcd")
    if not prefix.is_absolute() or not pcd.is_file() or pcd.stat().st_size == 0:
        raise ValueError("未找到有效的绝对路径 PCD 文件：" + str(pcd))
    session = "export_" + uuid.uuid4().hex
    namespace = "/desktop_ui/" + session
    topic = namespace + "/map"
    deadline = time.monotonic() + config["timeout_sec"]
    converter = saver = node = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node(session)
        received = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        subscription = node.create_subscription(OccupancyGrid, topic, received.append, qos)
        with tempfile.TemporaryDirectory(prefix=".fastlio-export-", dir=prefix.parent) as directory, ExitStack() as processes:
            stage = Path(directory)
            params = {"pcd_file": str(pcd), "map_topic_name": topic,
                      "thre_z_min": config["z_min"], "thre_z_max": config["z_max"],
                      "map_resolution": config["resolution"], "thre_radius": config["radius"],
                      "thres_point_count": config["min_neighbors"], "flag_pass_through": False,
                      "odom_to_lidar_odom": [0.0] * 6, "use_sim_time": False}
            params_file = stage / "projection.yaml"
            with params_file.open("w", encoding="utf-8") as stream:
                yaml.safe_dump({"/**": {"ros__parameters": params}}, stream)
            print(f"PCD 已保存，开始二维切片：高度 {config['z_min']}～{config['z_max']} m，"
                  f"分辨率 {config['resolution']} m", flush=True)
            converter = subprocess.Popen([
                executable("pcd2pgm", "pcd2pgm_node"), "--ros-args",
                "--params-file", str(params_file), "-r", "__ns:=" + namespace])
            processes.callback(stop_process, converter)
            while not received:
                if converter.poll() is not None:
                    raise RuntimeError("PCD 切片工具退出，未得到栅格；请检查点云及高度/半径过滤参数")
                if time.monotonic() >= deadline:
                    raise TimeoutError("等待 PCD 切片超时")
                rclpy.spin_once(node, timeout_sec=0.1)
            grid = received[0]
            if (grid.header.frame_id != "map" or grid.info.width == 0 or grid.info.height == 0
                    or len(grid.data) != grid.info.width * grid.info.height):
                raise RuntimeError("PCD 切片输出的栅格无效")
            node.destroy_subscription(subscription)
            print(f"切片完成：{grid.info.width} × {grid.info.height}，保存 PGM/YAML…", flush=True)
            saver = subprocess.Popen([
                executable("nav2_map_server", "map_saver_cli"), "-t", topic,
                "-f", str(stage / "map"), "--fmt", "pgm", "--ros-args",
                "-r", "__ns:=" + namespace,
                "-p", "map_subscribe_transient_local:=true", "-p", "save_map_timeout:=10.0"])
            processes.callback(stop_process, saver)
            while saver.poll() is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError("保存二维栅格超时")
                time.sleep(0.1)
            if saver.returncode != 0:
                raise RuntimeError("Nav2 栅格保存工具失败")
            publish_files(stage, prefix, grid)
            print("PCD、PGM、YAML 已保存：" + str(prefix), flush=True)
    finally:
        stop_process(saver)
        stop_process(converter)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def interrupted(*_):
    raise KeyboardInterrupt()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    try:
        export(args.prefix, settings(args.config))
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(f"二维栅格保存失败：{error or '操作被中断'}；已保存的 PCD 保留。", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
