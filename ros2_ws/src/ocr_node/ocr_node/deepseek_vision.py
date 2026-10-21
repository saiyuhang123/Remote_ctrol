#!/usr/bin/env python3
"""ROS2 service for online image understanding through a DeepSeek vision model.

The API key is loaded from a local YAML/legacy text file outside the repo.
It is never printed, logged, or included in ROS errors.
"""
from __future__ import annotations

import base64
import json
import re
import time
from pathlib import Path
from typing import Any
import urllib.error
import urllib.request

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from ocr_interfaces.srv import RecognizeImage

DEFAULT_PROMPT = (
    '只按以下5行输出，每行都必须保留，没有识别到的内容写“无”；'
    '不要Markdown、不要编号、不要额外解释、不要描述背景、纸箱、货物或场景，不要推测：\n'
    '设备状态：\n'
    '仪表读数：\n'
    '指示灯：\n'
    '告警文字：\n'
    '明显异常：'
)
LEGACY_KEY_RE = re.compile(r'API\s*Key\s*[:：]\s*([A-Za-z0-9_\-]{20,})', re.I)


def load_vision_config(path: str) -> dict[str, Any]:
    """Load new YAML config or the legacy labelled-key file.

    The legacy format is accepted for an in-place upgrade and to avoid breaking
    an existing operator file. Secret values are never returned in errors.
    """
    config_path = Path(path).expanduser()
    try:
        raw = config_path.read_text(encoding='utf-8')
    except OSError as exc:
        raise ValueError(f'无法读取视觉识别配置：{config_path}') from exc
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError:
        data = None
    if isinstance(data, dict):
        api_key = str(data.get('api_key') or data.get('apikey') or '').strip()
        if not api_key:
            raise ValueError(f'视觉识别配置缺少 api_key：{config_path}')
        return {
            'api_key': api_key,
            'base_url': str(data.get('base_url') or 'https://api.deepseek.com').strip(),
            'model': str(data.get('model') or 'deepseek-flash').strip(),
            'timeout_sec': float(data.get('timeout_sec') or 120.0),
            'max_tokens': int(data.get('max_tokens') or 2048),
            'system_prompt': str(data.get('system_prompt') or '').strip(),
        }
    match = LEGACY_KEY_RE.search(raw)
    if not match:
        raise ValueError(f'视觉识别配置缺少 API Key：{config_path}')
    return {
        'api_key': match.group(1),
        'base_url': 'https://api.deepseek.com',
        'model': 'deepseek-flash',
        'timeout_sec': 120.0,
        'max_tokens': 2048,
        'system_prompt': '',
    }


class DeepSeekVisionNode(Node):
    def __init__(self) -> None:
        super().__init__('deepseek_vision')
        self.declare_parameter('config_path', '/home/jkbot/dshapi')
        self.declare_parameter('service_name', '/vision/recognize_image')
        self.declare_parameter('default_prompt', DEFAULT_PROMPT)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('max_image_dimension', 1920)
        config_path = str(self.get_parameter('config_path').value)
        self._config = load_vision_config(config_path)
        self._bridge = CvBridge()
        self._jpeg_quality = max(50, min(95, int(self.get_parameter('jpeg_quality').value)))
        self._max_dimension = max(320, int(self.get_parameter('max_image_dimension').value))
        self._default_prompt = str(self.get_parameter('default_prompt').value or DEFAULT_PROMPT)
        self._group = ReentrantCallbackGroup()
        self._service = self.create_service(
            RecognizeImage,
            str(self.get_parameter('service_name').value),
            self._handle_recognize,
            callback_group=self._group,
        )
        self.get_logger().info(
            f'DeepSeek vision service ready: {self.get_parameter("service_name").value} '
            f'(model={self._config["model"]})')

    def _encode_image(self, msg: Image) -> tuple[bytes, str]:
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge errors include no secret
            raise ValueError(f'图像转换失败：{exc}') from exc
        if image is None or image.size == 0:
            raise ValueError('收到空图像')
        height, width = image.shape[:2]
        longest = max(height, width)
        if longest > self._max_dimension:
            scale = self._max_dimension / float(longest)
            image = cv2.resize(
                image,
                (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            raise ValueError('图像 JPEG 编码失败')
        return encoded.tobytes(), 'image/jpeg'

    @staticmethod
    def _extract_text(payload: dict[str, Any]) -> str:
        choices = payload.get('choices') or []
        if not choices:
            return ''
        message = choices[0].get('message') or {}
        content = message.get('content', '')
        if isinstance(content, str):
            return content.strip()
        if isinstance(content, list):
            parts: list[str] = []
            for item in content:
                if isinstance(item, dict) and item.get('type') in ('text', 'output_text'):
                    if item.get('text'):
                        parts.append(str(item['text']))
            return '\n'.join(parts).strip()
        return str(content).strip()

    def _post_chat(self, image_bytes: bytes, mime: str, prompt: str) -> dict[str, Any]:
        base_url = self._config['base_url'].rstrip('/')
        if not base_url.endswith('/v1'):
            base_url += '/v1'
        url = base_url + '/chat/completions'
        image_b64 = base64.b64encode(image_bytes).decode('ascii')
        system_prompt = self._config.get('system_prompt') or ''
        messages: list[dict[str, Any]] = []
        if system_prompt:
            messages.append({'role': 'system', 'content': system_prompt})
        messages.append({
            'role': 'user',
            'content': [
                {'type': 'text', 'text': prompt},
                {'type': 'image_url', 'image_url': {
                    'url': f'data:{mime};base64,{image_b64}',
                }},
            ],
        })
        body = {
            'model': self._config['model'],
            'messages': messages,
            'temperature': 0.1,
            'max_tokens': int(self._config['max_tokens']),
        }
        request = urllib.request.Request(
            url,
            data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self._config["api_key"]}',
                'Content-Type': 'application/json',
                'Accept': 'application/json',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(
                    request, timeout=float(self._config['timeout_sec'])) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = ''
            try:
                error_payload = json.loads(exc.read().decode('utf-8', 'replace'))
                detail = str(error_payload.get('error', {}).get('message', ''))[:500]
            except Exception:
                pass
            raise RuntimeError(f'DeepSeek HTTP {exc.code}' + (f'：{detail}' if detail else '')) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'DeepSeek 网络请求失败：{exc.reason}') from exc
        except TimeoutError as exc:
            raise RuntimeError('DeepSeek 请求超时') from exc
        if not isinstance(payload, dict):
            raise RuntimeError('DeepSeek 返回格式无效')
        text = self._extract_text(payload)
        if not text:
            raise RuntimeError('DeepSeek 未返回识别文本')
        return payload

    def _handle_recognize(self, request: RecognizeImage.Request,
                          response: RecognizeImage.Response) -> RecognizeImage.Response:
        started = time.monotonic()
        prompt = (request.prompt or '').strip() or self._default_prompt
        try:
            image_bytes, mime = self._encode_image(request.image)
            payload = self._post_chat(image_bytes, mime, prompt)
            response.success = True
            response.text = self._extract_text(payload)
            response.model = str(payload.get('model') or self._config['model'])
            response.message = 'OK'
            response.processing_time_ms = (time.monotonic() - started) * 1000.0
            self.get_logger().info(
                f'vision recognition OK ({response.processing_time_ms:.0f} ms, '
                f'{len(response.text)} chars)')
        except Exception as exc:  # noqa: BLE001 - report external failure to caller
            response.success = False
            response.text = ''
            response.model = str(self._config.get('model') or '')
            response.message = str(exc)[:1000]
            response.processing_time_ms = (time.monotonic() - started) * 1000.0
            self.get_logger().error(f'vision recognition failed：{response.message}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeepSeekVisionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
