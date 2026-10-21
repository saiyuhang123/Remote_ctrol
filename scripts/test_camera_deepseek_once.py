#!/usr/bin/env python3
"""Capture one image from hk_camera and send it to the DeepSeek ROS service.

Usage:
  source /opt/ros/humble/setup.bash
  source ~/Remote_ctrol/ros2_ws/install/setup.bash
  python3 scripts/test_camera_deepseek_once.py \
      --prompt "读取仪表数值并判断指示灯状态"

The script performs no navigation and no PTZ movement. It only calls the
camera capture service and the online recognition service.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from hk_camera.srv import CapturePicture
from ocr_interfaces.srv import RecognizeImage


def wait_call(node, client, request, label, timeout):
    if not client.wait_for_service(timeout_sec=min(15.0, timeout)):
        raise RuntimeError(f'{label} 服务未就绪')
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise TimeoutError(f'{label} 超时（{timeout:.0f}s）')
    error = future.exception()
    if error is not None:
        raise RuntimeError(f'{label} 调用异常：{error}')
    return future.result()


def main() -> int:
    parser = argparse.ArgumentParser(description='海康抓拍 + DeepSeek 在线识别单次测试')
    parser.add_argument('--save-path', default='/tmp/hk_camera_deepseek_test.jpg',
                        help='抓拍图片保存路径')
    parser.add_argument(
        '--prompt',
        default='只按以下5行输出，每行都必须保留，没有识别到的内容写“无”；'
                '不要Markdown、不要编号、不要额外解释、不要描述背景、纸箱、货物或场景，不要推测：\n'
                '设备状态：\n仪表读数：\n指示灯：\n告警文字：\n明显异常：',
        help='发送给 DeepSeek 的识别提示词')
    parser.add_argument('--capture-service', default='/hk_camera/capture')
    parser.add_argument('--vision-service', default='/vision/recognize_image')
    parser.add_argument('--capture-timeout', type=float, default=20.0)
    parser.add_argument('--vision-timeout', type=float, default=150.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('camera_deepseek_one_shot_test')
    try:
        capture = node.create_client(CapturePicture, args.capture_service)
        vision = node.create_client(RecognizeImage, args.vision_service)

        save_path = str(Path(args.save_path).expanduser())
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        cap_request = CapturePicture.Request()
        cap_request.quality = 0
        cap_request.save_path = save_path
        print(f'[1/2] 正在调用海康抓拍服务 {args.capture_service} ...')
        cap_response = wait_call(
            node, capture, cap_request, '海康抓拍', args.capture_timeout)
        if not cap_response.success:
            raise RuntimeError(f'海康抓拍失败：{cap_response.message}')
        image = cap_response.image
        if not image.data:
            raise RuntimeError('海康抓拍返回了空图像')
        print(f'      抓拍成功：{image.width}x{image.height}, encoding={image.encoding}, '
              f'bytes={len(image.data)}')
        print(f'      图片保存：{save_path}')

        req = RecognizeImage.Request()
        req.image = image
        req.prompt = args.prompt
        print(f'[2/2] 正在调用 DeepSeek 识别服务 {args.vision_service} ...')
        started = time.monotonic()
        response = wait_call(node, vision, req, 'DeepSeek 识别', args.vision_timeout)
        elapsed = time.monotonic() - started
        if not response.success:
            raise RuntimeError(f'DeepSeek 识别失败：{response.message}')
        print(f'      识别成功：model={response.model}, '
              f'耗时={response.processing_time_ms:.0f}ms, 总耗时={elapsed:.1f}s')
        print('--- DeepSeek 结果 ---')
        print(response.text)
        print('---------------------')
        return 0
    except Exception as error:  # noqa: BLE001 - command-line test boundary
        print(f'测试失败：{error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
