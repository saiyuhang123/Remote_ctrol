"""CPU YOLO + plates. No Torch, face model, identity database or HTTP."""
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


class Pipeline:
    """One shared detector, lightweight IoU tracks, per-track records."""
    def __init__(self, detector, plate=None, track_ttl=2., plate_confirmations=2):
        self.detector, self.plate = detector, plate
        self.track_ttl, self.plate_confirmations = track_ttl, plate_confirmations
        self.tracks, self.next_id = {}, 0

    def reset(self):
        self.tracks.clear()

    def process(self, frame, person=True, plate=True, now=None):
        if not person and not plate:
            return []
        now = time.monotonic() if now is None else now
        self.tracks = {key: tr for key, tr in self.tracks.items() if now-tr['last'] <= self.track_ttl}
        available = set(self.tracks)
        events = []
        for cls, score, box in self.detector(frame):
            if (cls == 0 and not person) or (cls != 0 and not plate):
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
            if cls == 0:
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
