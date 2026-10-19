"""Independent CPU YuNet / YOLO26n / LPD-YuNet + plate text recognition."""
import time
from itertools import product

import cv2
import numpy as np


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2-x1) * max(0, y2-y1)
    return intersection / max(1., (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection)


class YoloCpu:
    """YOLO26n COCO person class; deployed export uses the NMS-free head."""
    def __init__(self, model_path, threads=2, confidence=.45):
        import openvino as ov
        self.model = ov.Core().compile_model(str(model_path), 'CPU',
                                            {'INFERENCE_NUM_THREADS': threads, 'NUM_STREAMS': '1'})
        self.output = self.model.output(0)
        self.confidence = confidence

    @staticmethod
    def decode(output, confidence, width, height, ratio, dx, dy):
        prediction = np.asarray(output)[0]
        if prediction.ndim != 2:
            raise ValueError('unsupported YOLO output shape')
        if prediction.shape[-1] == 6:
            # End-to-end: x1, y1, x2, y2, score, class. No second NMS.
            rows = prediction[(prediction[:, 4] >= confidence) & (prediction[:, 5] == 0)]
        elif prediction.shape[0] == 84:
            # Also accept a COCO one-to-many export with explicit NMS.
            raw = prediction.T
            scores = raw[:, 4:]
            mask = (scores.argmax(axis=1) == 0) & (scores[:, 0] >= confidence)
            raw = raw[mask]
            boxes = np.column_stack((raw[:, :2]-raw[:, 2:4]/2, raw[:, 2:4]))
            selected = cv2.dnn.NMSBoxes(boxes.tolist(), raw[:, 4].tolist(), confidence, .45)
            rows = np.asarray([[*boxes[i, :2], *(boxes[i, :2]+boxes[i, 2:]), raw[i, 4], 0]
                               for i in np.asarray(selected).reshape(-1)], dtype=np.float32).reshape(-1, 6)
        else:
            raise ValueError(f'unsupported YOLO output shape: {prediction.shape}')
        result = []
        for x1, y1, x2, y2, score, _cls in rows:
            if not np.isfinite([x1, y1, x2, y2, score]).all():
                continue
            box = [max(0, int((x1-dx)/ratio)), max(0, int((y1-dy)/ratio)),
                   min(width, int((x2-dx)/ratio)), min(height, int((y2-dy)/ratio))]
            if box[2] > box[0] and box[3] > box[1]:
                result.append((0, float(score), box))
        return result

    def __call__(self, image):
        h, w = image.shape[:2]
        ratio = min(640/w, 640/h)
        rw, rh = round(w*ratio), round(h*ratio)
        dx, dy = (640-rw)//2, (640-rh)//2
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[dy:dy+rh, dx:dx+rw] = cv2.resize(image, (rw, rh))
        tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)/255.
        return self.decode(self.model([tensor])[self.output], self.confidence, w, h, ratio, dx, dy)


class YunetCpu:
    """Detect faces, without identity recognition or a face database."""
    def __init__(self, model_path, confidence=.8, input_width=960):
        self.input_width = input_width
        self.model = cv2.FaceDetectorYN.create(
            str(model_path), '', (320, 320), confidence, .3, 5000,
            cv2.dnn.DNN_BACKEND_OPENCV, cv2.dnn.DNN_TARGET_CPU)

    def __call__(self, image):
        h, w = image.shape[:2]
        ratio = min(1., self.input_width / max(w, h))
        rw, rh = max(1, round(w*ratio)), max(1, round(h*ratio))
        resized = cv2.resize(image, (rw, rh)) if ratio < 1. else image
        canvas = cv2.copyMakeBorder(resized, 0, (-rh) % 32, 0, (-rw) % 32,
                                    cv2.BORDER_CONSTANT, value=(0, 0, 0))
        self.model.setInputSize((canvas.shape[1], canvas.shape[0]))
        _status, faces = self.model.detect(canvas)
        result = []
        for row in faces if faces is not None else []:
            if not np.isfinite(row).all():
                continue
            x, y, width, height = row[:4]
            box = [max(0, int(x/ratio)), max(0, int(y/ratio)),
                   min(w, int((x+width)/ratio)), min(h, int((y+height)/ratio))]
            if box[2]-box[0] >= 20 and box[3]-box[1] >= 20:
                result.append((-1, float(row[-1]), box))
        return result


class LpdYunetCpu:
    """Chinese plate quadrilaterals from OpenCV Zoo LPD-YuNet.

    Model contract: opencv/opencv_zoo/models/license_plate_detection_yunet.
    NMS receives xywh boxes, never eight landmark coordinates.
    """
    def __init__(self, model_path, confidence=.8, input_width=640):
        self.model = cv2.dnn.readNet(str(model_path))
        self.model.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.model.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.confidence, self.input_width = confidence, input_width
        self.prior_shape, self.priors = None, None

    def anchors(self, width, height):
        if self.prior_shape != (width, height):
            entries = []
            for stride, sizes in zip((8, 16, 32, 64), ((10, 16, 24), (32, 48), (64, 96), (128, 192, 256))):
                for y, x in product(range(height//stride), range(width//stride)):
                    entries.extend([[(x+.5)*stride/width, (y+.5)*stride/height, size/width, size/height]
                                    for size in sizes])
            self.priors = np.asarray(entries, dtype=np.float32)
            self.prior_shape = (width, height)
        return self.priors

    def __call__(self, image):
        h, w = image.shape[:2]
        ratio = min(1., self.input_width/max(h, w))
        rw, rh = max(1, round(w*ratio)), max(1, round(h*ratio))
        resized = cv2.resize(image, (rw, rh)) if ratio < 1. else image
        canvas = cv2.copyMakeBorder(resized, 0, (-rh) % 64, 0, (-rw) % 64,
                                    cv2.BORDER_CONSTANT, value=(0, 0, 0))
        height, width = canvas.shape[:2]
        self.model.setInput(cv2.dnn.blobFromImage(canvas))
        loc, conf, quality = self.model.forward(['loc', 'conf', 'iou'])
        loc, conf, quality = loc.reshape(-1, 14), conf.reshape(-1, 2), quality.reshape(-1)
        priors = self.anchors(width, height)
        if len(loc) != len(priors):
            raise ValueError('LPD-YuNet anchor/output shape mismatch')
        scores = np.sqrt(np.clip(conf[:, 1], 0, 1) * np.clip(quality, 0, 1))
        keep = (scores >= self.confidence) & np.isfinite(loc).all(axis=1)
        loc, priors, scores = loc[keep], priors[keep], scores[keep]
        corners = np.stack([(priors[:, :2] + loc[:, offset:offset+2]*.1*priors[:, 2:])
                            * [width, height] / ratio for offset in (4, 6, 10, 12)], axis=1)
        boxes, valid = [], []
        for index, points in enumerate(corners):
            points[:, 0] = np.clip(points[:, 0], 0, w-1)
            points[:, 1] = np.clip(points[:, 1], 0, h-1)
            low, high = points.min(axis=0), points.max(axis=0)
            if high[0]-low[0] < 30 or high[1]-low[1] < 10:
                continue
            boxes.append([*low.tolist(), *(high-low).tolist()]); valid.append(index)
        if not boxes:
            return []
        selected = cv2.dnn.NMSBoxes(boxes, [float(scores[i]) for i in valid], self.confidence, .3)
        result = []
        for selected_index in np.asarray(selected).reshape(-1):
            index = valid[int(selected_index)]
            x, y, bw, bh = boxes[int(selected_index)]
            result.append(dict(confidence=float(scores[index]), bbox=[int(x), int(y), int(x+bw), int(y+bh)],
                               corners=corners[index].tolist()))
        return result


def rectify_plate(frame, corners):
    points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    if not np.isfinite(points).all():
        return None
    hull = cv2.convexHull(points).reshape(-1, 2)
    if len(hull) != 4 or abs(cv2.contourArea(hull)) < 20:
        return None
    angles = np.arctan2(hull[:, 1]-hull[:, 1].mean(), hull[:, 0]-hull[:, 0].mean())
    ordered = hull[np.argsort(angles)]
    ordered = np.roll(ordered, -int(ordered.sum(axis=1).argmin()), axis=0)
    width = round(max(np.linalg.norm(ordered[1]-ordered[0]), np.linalg.norm(ordered[2]-ordered[3])))
    height = round(max(np.linalg.norm(ordered[3]-ordered[0]), np.linalg.norm(ordered[2]-ordered[1])))
    if width < 30 or height < 10:
        return None
    target = np.float32([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]])
    return cv2.warpPerspective(frame, cv2.getPerspectiveTransform(ordered, target), (width, height))


class PlateTextCpu:
    """Only HyperLPR3's text model, without its detector/classifier.

    Character order and BGR normalization match HyperLPR3 0.1.3's
    rpv3_mdict_160_r3 model. Rectified single-line plates are normalized to
    its full input aspect ratio, including strongly foreshortened plates.
    """
    characters = ["blank", "'"] + list('0123456789ABCDEFGHJKLM NOPQRSTUVWXYZ'.replace(' ', '')) + list(
        '云京冀吉学宁川挂新晋桂民沪津浙渝港湘琼甘皖粤航苏蒙藏警豫贵赣辽鄂闽陕青鲁黑领使澳')

    def __init__(self, model_path, threads=2):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads, options.inter_op_num_threads = threads, 1
        options.log_severity_level = 3
        self.session = ort.InferenceSession(str(model_path), sess_options=options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]
        self.height, self.width = self.input.shape[2:]

    @classmethod
    def decode(cls, probabilities):
        indices = probabilities.argmax(axis=-1).reshape(-1)
        scores = probabilities.max(axis=-1).reshape(-1)
        chars, confidence = [], []
        previous = None
        for index, score in zip(indices, scores):
            if index != 0 and index != previous and index < len(cls.characters):
                chars.append(cls.characters[int(index)]); confidence.append(float(score))
            previous = index
        return ''.join(chars), float(np.mean(confidence)) if confidence else 0.

    def __call__(self, image):
        if image is None or not image.size:
            return '', 0.
        resized = cv2.resize(image, (self.width, self.height)).astype(np.float32)
        tensor = np.ascontiguousarray((resized.transpose(2, 0, 1)[None]-127.5)/127.5)
        return self.decode(self.session.run(None, {self.input.name: tensor})[0])


class Pipeline:
    """Independent models with anonymous IoU tracks and per-track records."""
    def __init__(self, detector=None, plate=None, track_ttl=2., plate_confirmations=2,
                 face=None, face_capture_interval=2., plate_detector=None):
        self.detector, self.plate, self.plate_detector = detector, plate, plate_detector
        self.face, self.face_capture_interval = face, face_capture_interval
        self.track_ttl, self.plate_confirmations = track_ttl, plate_confirmations
        self.tracks, self.next_id = {}, 0

    def reset(self):
        self.tracks.clear()

    def process(self, frame, person=True, plate=True, now=None, face=False):
        now = time.monotonic() if now is None else now
        enabled_classes = ({0} if person else set()) | ({-1} if face else set()) | ({-2} if plate else set())
        self.tracks = {key: tr for key, tr in self.tracks.items()
                       if now-tr['last'] <= self.track_ttl and tr['cls'] in enabled_classes}
        available = set(self.tracks)
        events, detections = [], []
        if person and self.detector is not None:
            detections.extend((cls, score, box, None) for cls, score, box in self.detector(frame) if cls == 0)
        if face and self.face is not None:
            detections.extend((cls, score, box, None) for cls, score, box in self.face(frame))
        if plate and self.plate_detector is not None:
            detections.extend((-2, item['confidence'], item['bbox'], item['corners'])
                              for item in self.plate_detector(frame))
        for cls, score, box, corners in detections:
            candidates = [(iou(box, self.tracks[key]['box']), key) for key in available
                          if self.tracks[key]['cls'] == cls]
            overlap, key = max(candidates, default=(0., -1))
            if overlap < .25:
                self.next_id += 1; key = self.next_id
                self.tracks[key] = dict(cls=cls, person_reported=False, plates=set(), candidate='', confirmations=0)
            available.discard(key)
            tr = self.tracks[key]; tr.update(box=box, last=now)
            x1, y1, x2, y2 = map(int, box)
            crop = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
            if not crop.size:
                continue
            if cls == -1:
                last = tr.get('face_last')
                if last is not None and now-last < self.face_capture_interval:
                    continue
                tr['face_last'] = now
                events.append(dict(event_type='face_capture', label='人脸抓拍', track_id=key,
                                   confidence=score, bbox=box, image=crop.copy()))
            elif cls == 0:
                if not tr['person_reported'] and crop.shape[0] >= 40:
                    tr['person_reported'] = True
                    events.append(dict(event_type='person', label='行人', track_id=key,
                                       confidence=score, bbox=box, image=crop.copy()))
            elif self.plate is not None:
                rectified = rectify_plate(frame, corners)
                if rectified is None:
                    continue
                text, confidence = self.plate(rectified)
                if len(text) not in (7, 8) or confidence < .7:
                    tr.update(candidate='', confirmations=0)
                    continue
                tr['confirmations'] = tr['confirmations']+1 if tr['candidate'] == text else 1
                tr['candidate'] = text
                if text in tr['plates'] or tr['confirmations'] < self.plate_confirmations:
                    continue
                tr['plates'].add(text)
                events.append(dict(event_type='vehicle_plate', label=text, plate_text=text,
                                   confidence=confidence, plate_detection_confidence=score,
                                   track_id=key, bbox=box, corners=corners, image=rectified))
        return events
