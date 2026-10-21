"""Integration checks for PTZ -> capture -> vision inspection orchestration."""
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from hk_camera.srv import CapturePicture, PTZPreset
from ocr_interfaces.srv import RecognizeImage
from robot_patrol.inspection_client import InspectionClient
from robot_patrol.store import PatrolStore


def make_store(tmp_path):
    store = PatrolStore(tmp_path / 'patrol.sqlite3')
    mapping = store.upsert_map({
        'name': 'map', 'backend': 'amcl', 'frame_id': 'map',
        'navigation_yaml': '/tmp/map.yaml', 'image_path': '/tmp/map.pgm',
        'localization_file': '', 'resolution': 0.05, 'origin_x': 0.0,
        'origin_y': 0.0, 'origin_yaw': 0.0, 'width': 10, 'height': 10,
        'navigation_yaml_sha256': 'a' * 64, 'image_sha256': 'b' * 64,
        'localization_sha256': '', 'verified': True,
    })
    store.create_run('run1', 'route1', mapping.map_id, 1, mapping.revision,
                     {'steps': []}, 1, 1, 1)
    return store, mapping


def test_inspection_client_moves_camera_captures_and_recognizes(tmp_path):
    rclpy.init()
    server_node = Node('fake_inspection_services')
    client_node = Node('inspection_client_fixture')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)
    executor.add_node(client_node)

    calls = []
    def preset(request, response):
        calls.append(('preset', int(request.action), int(request.preset_no)))
        response.success = True
        response.message = 'goto preset accepted'
        return response

    def capture(request, response):
        calls.append(('capture', request.save_path))
        Path(request.save_path).write_bytes(b'fake jpeg')
        image = Image()
        image.height = 2
        image.width = 2
        image.encoding = 'bgr8'
        image.step = 6
        image.data = bytes([0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255])
        response.success = True
        response.message = 'captured'
        response.image = image
        return response

    def recognize(request, response):
        calls.append(('recognize', request.prompt, len(request.image.data)))
        response.success = True
        response.message = 'OK'
        response.text = '电压 220 V，绿色 RUNNING 指示灯'
        response.model = 'deepseek-flash'
        response.processing_time_ms = 12.0
        return response

    services = [
        server_node.create_service(PTZPreset, '/hk_camera/preset', preset),
        server_node.create_service(CapturePicture, '/hk_camera/capture', capture),
        server_node.create_service(RecognizeImage, '/vision/recognize_image', recognize),
    ]

    params = {
        'inspection_image_dir': str(tmp_path / 'images'),
        'inspection_service_timeout_sec': 2.0,
        'inspection_capture_timeout_sec': 2.0,
        'inspection_recognition_timeout_sec': 5.0,
        'inspection_preset_settle_sec': 0.0,
        'inspection_preset_service': '/hk_camera/preset',
        'inspection_capture_service': '/hk_camera/capture',
        'inspection_recognition_service': '/vision/recognize_image',
        'inspection_default_prompt': '请识别设备状态',
    }
    for name, value in params.items():
        client_node.declare_parameter(name, value)

    store, mapping = make_store(tmp_path)
    inspection = InspectionClient(client_node, store)
    step = SimpleNamespace(
        point_id='p1', point_name='仪表点', inspection_enabled=True,
        ptz_preset=90,
        recognition_prompt='读取电压和指示灯',
    )

    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        outcome = inspection.run(step, 'run1', mapping.map_id, 1, 1)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        client_node.destroy_node()
        server_node.destroy_node()
        rclpy.try_shutdown()

    assert outcome.success, outcome.message
    assert outcome.text == '电压 220 V，绿色 RUNNING 指示灯'
    assert outcome.model == 'deepseek-flash'
    assert ('preset', 1, 90) in calls
    assert any(item[0] == 'capture' for item in calls)
    assert any(item[0] == 'recognize' and item[1] == '读取电压和指示灯' for item in calls)
    assert Path(outcome.image_path).read_bytes() == b'fake jpeg'
    results = store.list_inspection_results(mapping.map_id, 'run1', 10)
    assert len(results) == 1
    assert results[0].success
    assert results[0].text == outcome.text
    assert results[0].image_path == outcome.image_path
    assert results[0].ptz_preset == 90
    store.close()
