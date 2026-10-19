"""CPU YuNet face crops, shared YOLO and plates; no identity matching or HTTP."""
import time
from collections import Counter
import cv2
import numpy as np


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2-x1) * max(0, y2-y1)
    return intersection / max(1., (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection)


class YoloCpu:
    def __init__(self, model_path, threads=2, confidence=.45):
        import openvino as ov
        self.model = ov.Core().compile_model(str(model_path), 'CPU',
                                            {'INFERENCE_NUM_THREADS': threads, 'NUM_STREAMS': '1'})
        self.output = self.model.output(0)
        self.confidence = confidence

    def __call__(self, image):
        h, w = image.shape[:2]
        ratio = min(640/w, 640/h)
        rw, rh = round(w*ratio), round(h*ratio)
        dx, dy = (640-rw)//2, (640-rh)//2
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[dy:dy+rh, dx:dx+rw] = cv2.resize(image, (rw, rh))
        tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)/255.
        prediction = np.asarray(self.model([tensor])[self.output])[0].T
        classes = prediction[:, 4:].argmax(axis=1)
        scores = prediction[np.arange(len(prediction)), classes+4]
        mask = (scores >= self.confidence) & np.isin(classes, [0, 2, 3, 5, 7])
        prediction, classes, scores = prediction[mask], classes[mask], scores[mask]
        boxes = []
        for row in prediction:
            cx, cy, bw, bh = row[:4]
            x1, y1 = max(0., (cx-bw/2-dx)/ratio), max(0., (cy-bh/2-dy)/ratio)
            x2, y2 = min(w, (cx+bw/2-dx)/ratio), min(h, (cy+bh/2-dy)/ratio)
            boxes.append([int(x1), int(y1), int(x2-x1), int(y2-y1)])
        result = []
        for cls in sorted(set(classes.tolist())):
            indices = [i for i, value in enumerate(classes) if value == cls]
            selected = cv2.dnn.NMSBoxes([boxes[i] for i in indices], [float(scores[i]) for i in indices],
                                        self.confidence, .45)
            for j in np.asarray(selected).reshape(-1):
                k = indices[int(j)]
                x, y, bw, bh = boxes[k]
                if bw > 0 and bh > 0:
                    result.append((cls, float(scores[k]), [x, y, x+bw, y+bh]))
        return result


class YunetCpu:
    """Detect faces in the video frame without running identity recognition."""
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
        # Pad to stride boundaries without stretching the source geometry.
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


class Pipeline:
    """One shared detector, lightweight IoU tracks, per-track records."""
    def __init__(self, detector=None, plate=None, track_ttl=2., plate_confirmations=2,
                 face=None, face_capture_interval=2.):
        self.detector, self.plate = detector, plate
        self.face, self.face_capture_interval = face, face_capture_interval
        self.track_ttl, self.plate_confirmations = track_ttl, plate_confirmations
        self.tracks, self.next_id = {}, 0

    def reset(self):
        self.tracks.clear()

    def process(self, frame, person=True, plate=True, now=None, face=False):
        if not person and not plate and not face:
            return []
        now = time.monotonic() if now is None else now
        self.tracks = {key: tr for key, tr in self.tracks.items() if now-tr['last'] <= self.track_ttl}
        available = set(self.tracks)
        events = []
        detections = []
        if (person or plate) and self.detector is not None:
            detections.extend(self.detector(frame))
        if face and self.face is not None:
            detections.extend(self.face(frame))
        if not face:
            self.tracks = {key: tr for key, tr in self.tracks.items() if tr['cls'] != -1}
            available.intersection_update(self.tracks)
        for cls, score, box in detections:
            if (cls == -1 and not face) or (cls == 0 and not person) or (cls > 0 and not plate):
                continue
            candidates = [(iou(box, self.tracks[key]['box']), key) for key in available
                          if self.tracks[key]['cls'] == cls]
            overlap, key = max(candidates, default=(0., -1))
            if overlap < .25:
                self.next_id += 1
                key = self.next_id
                self.tracks[key] = dict(cls=cls, person_reported=False, plates=set(), votes=Counter())
            available.discard(key)
            tr = self.tracks[key]
            tr.update(box=box, last=now)
            x1, y1, x2, y2 = map(int, box)
            crop = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
            if crop.size == 0:
                continue
            if cls == -1:
                last = tr.get('face_last')
                if last is not None and now-last < self.face_capture_interval:
                    continue
                tr['face_last'] = now
                events.append(dict(event_type='face_capture', label='人脸抓拍', track_id=key,
                                   confidence=score, bbox=box, image=crop.copy()))
            elif cls == 0:
                if tr['person_reported'] or crop.shape[0] < 40:
                    continue
                tr['person_reported'] = True
                events.append(dict(event_type='person', label='行人', track_id=key,
                                   confidence=score, bbox=box, image=crop.copy()))
            elif self.plate is not None and crop.shape[1] >= 90:
                seen = set()
                for result in self.plate(crop):
                    text, confidence = str(result[0]), float(result[1])
                    if not text or confidence < .7 or text in tr['plates'] or text in seen:
                        continue
                    seen.add(text)
                    tr['votes'][text] += 1
                    if tr['votes'][text] < self.plate_confirmations:
                        continue
                    tr['plates'].add(text)
                    events.append(dict(event_type='vehicle_plate', label=text, plate_text=text,
                                       confidence=confidence, vehicle_confidence=score,
                                       track_id=key, bbox=box, image=crop.copy()))
        return events
