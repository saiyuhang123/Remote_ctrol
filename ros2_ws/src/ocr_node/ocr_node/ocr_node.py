#!/usr/bin/env python3
"""
OCR Node — EasyOCR text/number recognition for ROS2.

Two usage modes (can coexist):

 1. Streaming mode   — continuous recognition at a fixed interval.
                       Enable with `enable_streaming: true`.
 2. Service mode     — on-demand recognition via `/ocr/recognize`.
                       Always available; blocks until the latest frame
                       is processed, then returns results.

Subscribes:
  /hk_camera/image_raw (sensor_msgs/Image) — camera frames

Publishes (streaming mode only):
  /ocr/result         (ocr_interfaces/OcrResult)  — recognized detections
  /ocr/annotated_img  (sensor_msgs/Image)         — annotated preview

Service:
  /ocr/recognize      (ocr_interfaces/RecognizeText) — trigger OCR on demand
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image

from ocr_interfaces.msg import OcrDetection, OcrResult
from ocr_interfaces.srv import RecognizeText


class OcrNode(Node):
    """ROS2 node that caches camera frames and runs EasyOCR on demand."""

    def __init__(self):
        super().__init__('ocr_node')

        # ---- Parameters ----
        self.declare_parameter('lang', 'ch')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('processing_interval', 0.5)
        self.declare_parameter('enable_annotated_img', True)
        self.declare_parameter('enable_streaming', False)

        self._lang = self.get_parameter('lang').value
        self._use_gpu = self.get_parameter('use_gpu').value
        self._conf_threshold = self.get_parameter('conf_threshold').value
        self._interval = self.get_parameter('processing_interval').value
        self._publish_annotated = self.get_parameter('enable_annotated_img').value
        self._streaming = self.get_parameter('enable_streaming').value

        # ---- EasyOCR (lazy-init on first use) ----
        self._ocr = None
        self._ocr_lock = threading.Lock()

        self.get_logger().info(
            f'OCR node starting (lang={self._lang}, gpu={self._use_gpu}, '
            f'streaming={self._streaming})'
        )

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- Latest-frame cache (always updated) ----
        self._latest_frame = None          # cv::Mat (BGR8)
        self._latest_header = None         # std_msgs/Header
        self._frame_lock = threading.Lock()

        # ---- Subscriber (always on — cache every frame for service use) ----
        self._sub = self.create_subscription(
            Image,
            '/hk_camera/image_raw',
            self._image_callback,
            10,
        )

        # ---- Streaming-mode publishers (only when enable_streaming) ----
        self._result_pub = None
        self._annotated_pub = None
        if self._streaming:
            self._result_pub = self.create_publisher(OcrResult, '/ocr/result', 10)
            if self._publish_annotated:
                self._annotated_pub = self.create_publisher(Image, '/ocr/annotated_img', 10)

        # ---- Streaming throttle ----
        self._last_time = 0.0
        self._processing = False

        # ---- Service: on-demand recognition ----
        # Use a separate callback group to avoid deadlocks with the subscriber.
        self._srv_group = MutuallyExclusiveCallbackGroup()
        self._recognize_srv = self.create_service(
            RecognizeText,
            '/ocr/recognize',
            self._handle_recognize,
            callback_group=self._srv_group,
        )

        mode_desc = 'streaming + service' if self._streaming else 'service-only'
        self.get_logger().info(f'OCR Node ready [{mode_desc}]')

    # ==================================================================
    #  EasyOCR lifecycle
    # ==================================================================

    def _init_ocr(self):
        """Create the EasyOCR reader (call with lock held)."""
        import easyocr

        # Map PaddleOCR-style lang to EasyOCR language list
        if self._lang == 'ch':
            lang_list = ['ch_sim', 'en']
        else:
            lang_list = [self._lang]

        self._ocr = easyocr.Reader(
            lang_list,
            gpu=self._use_gpu,
        )
        self.get_logger().info(
            f'EasyOCR initialized (lang={self._lang}, gpu={self._use_gpu})'
        )

    def _ensure_ocr(self):
        """Return the EasyOCR reader; lazy-init on first call if needed."""
        if self._ocr is not None:
            return self._ocr
        with self._ocr_lock:
            if self._ocr is None:
                self._init_ocr()
        return self._ocr

    # ==================================================================
    #  Frame caching (always active)
    # ==================================================================

    def _image_callback(self, msg: Image):
        """Always cache the latest frame. Optionally trigger streaming OCR."""
        # Decode and cache the latest frame (always)
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return
        with self._frame_lock:
            self._latest_frame = frame
            self._latest_header = msg.header

        # Streaming mode: throttle and dispatch
        if not self._streaming:
            return
        now = time.time()
        if now - self._last_time < self._interval:
            return
        if self._processing:
            return
        self._last_time = now
        self._processing = True
        t = threading.Thread(target=self._process_streaming_frame, args=(frame, msg.header), daemon=True)
        t.start()

    # ==================================================================
    #  Core OCR inference (shared by streaming and service)
    # ==================================================================

    def _run_ocr(self, cv_image, conf_threshold=None):
        """Run EasyOCR on a single BGR8 image.

        Args:
            cv_image: BGR8 OpenCV Mat.
            conf_threshold: override confidence threshold (None = use default).

        Returns:
            (detections: list[OcrDetection], proc_ms: float, annotated: cv::Mat)
        """
        ocr = self._ensure_ocr()
        threshold = conf_threshold if conf_threshold and conf_threshold > 0.0 else self._conf_threshold

        t0 = time.time()
        raw = ocr.readtext(cv_image)  # EasyOCR: [(bbox, text, confidence), ...]
        proc_ms = (time.time() - t0) * 1000.0

        annotated = cv_image.copy()
        detections = []

        for (box, text, conf) in raw:
            # box = [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
            if conf < threshold:
                continue

            d = OcrDetection()
            d.corners = [float(v) for p in box for v in p]
            d.text = text
            d.confidence = float(conf)
            detections.append(d)

            # Draw on annotated image
            pts = np.array([[int(p[0]), int(p[1])] for p in box], dtype=np.int32)
            hull = cv2.convexHull(pts)
            cv2.polylines(annotated, [hull], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.putText(
                annotated, f'{text} ({conf:.2f})',
                org=(int(box[0][0]), max(int(box[0][1]) - 8, 10)),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.5,
                color=(0, 255, 0),
                thickness=1,
            )

        return detections, proc_ms, annotated

    # ==================================================================
    #  Streaming mode
    # ==================================================================

    def _process_streaming_frame(self, cv_image, header):
        """Run OCR on a streaming frame and publish results."""
        try:
            detections, proc_ms, annotated = self._run_ocr(cv_image)
        except Exception as exc:
            self.get_logger().error(f'EasyOCR streaming error: {exc}')
            self._processing = False
            return

        # Publish structured result
        if self._result_pub is not None:
            result = OcrResult()
            result.header = header
            result.processing_time_ms = float(proc_ms)
            result.detections = detections
            self._result_pub.publish(result)

        # Publish annotated preview
        if self._annotated_pub is not None:
            ann_msg = self._bridge.cv2_to_imgmsg(annotated, 'bgr8')
            ann_msg.header = header
            self._annotated_pub.publish(ann_msg)

        if detections:
            texts = ', '.join(d.text for d in detections[:5])
            if len(detections) > 5:
                texts += '...'
            self.get_logger().info(f'[stream][{proc_ms:.0f}ms] {len(detections)} text(s): {texts}')
        else:
            self.get_logger().debug(f'[stream][{proc_ms:.0f}ms] no text detected')

        self._processing = False

    # ==================================================================
    #  Service mode — /ocr/recognize
    # ==================================================================

    def _handle_recognize(self, request, response):
        """Service callback: grab the latest cached frame and run OCR.

        This blocks the caller until recognition completes (~100-500ms
        depending on hardware).  If no frame has been received yet the
        call returns success=false immediately.
        """
        # Grab the latest frame
        with self._frame_lock:
            if self._latest_frame is None:
                response.success = False
                response.message = 'No frame received yet — is hk_camera running?'
                self.get_logger().warn(response.message)
                return response
            frame = self._latest_frame.copy()
            header = self._latest_header

        # Run OCR (blocks until done)
        try:
            detections, proc_ms, _ = self._run_ocr(
                frame,
                conf_threshold=request.conf_threshold,
            )
        except Exception as exc:
            response.success = False
            response.message = f'OCR inference failed: {exc}'
            self.get_logger().error(response.message)
            return response

        # Fill response
        response.success = True
        response.message = (
            f'OK: {len(detections)} text(s) in {proc_ms:.0f}ms'
        )
        response.detections = detections
        response.processing_time_ms = float(proc_ms)

        if detections:
            texts = ', '.join(d.text for d in detections[:5])
            self.get_logger().info(f'[service][{proc_ms:.0f}ms] {texts}')
        else:
            self.get_logger().info(f'[service][{proc_ms:.0f}ms] no text')

        return response


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OcrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
