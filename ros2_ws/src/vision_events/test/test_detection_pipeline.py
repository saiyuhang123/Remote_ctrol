import numpy as np
from pathlib import Path
from detector_node.detection_pipeline import Pipeline, YunetCpu


def test_people_do_not_wait_for_faces_and_reentry_creates_new_record():
    frame = np.zeros((300, 500, 3), np.uint8)
    pipeline = Pipeline(lambda image: [(0, .92, [10, 10, 80, 250])])
    first = pipeline.process(frame, plate=False, now=1.)
    assert len(first) == 1 and first[0]['confidence'] == .92
    assert first[0]['event_type'] == 'person'
    assert pipeline.process(frame, plate=False, now=1.2) == []
    assert len(pipeline.process(frame, plate=False, now=5.)) == 1


def test_one_yolo_pass_plate_confirmations_and_feature_switches():
    calls = []
    frame = np.zeros((300, 500, 3), np.uint8)
    def detect(image):
        calls.append(1)
        return [(0, .9, [5, 10, 60, 240]), (2, .8, [100, 20, 450, 270])]
    pipeline = Pipeline(detect, plate=lambda crop: [['鲁A12345', .95, 0], ['鲁A12345', .94, 0]])
    assert pipeline.process(frame, False, False) == [] and calls == []
    first = pipeline.process(frame, now=1.)
    assert [r['event_type'] for r in first] == ['person']
    second = pipeline.process(frame, now=1.2)
    assert second[0]['plate_text'] == '鲁A12345' and len(calls) == 2
    assert pipeline.process(frame, now=1.4) == []
    assert len(pipeline.process(frame, person=False, now=5.)) == 0
    later = pipeline.process(frame, person=False, now=5.2)
    assert later[0]['event_type'] == 'vehicle_plate'


def test_face_only_crops_repeat_without_identity_or_yolo():
    frame = np.full((120, 160, 3), 80, np.uint8)
    calls = []
    def faces(image):
        calls.append(1)
        return [(-1, .94, [20, 25, 70, 85])]
    pipeline = Pipeline(face=faces, face_capture_interval=2.)
    first = pipeline.process(frame, person=False, plate=False, face=True, now=1.)
    assert len(first) == 1 and first[0]['event_type'] == 'face_capture'
    assert first[0]['image'].shape == (60, 50, 3)
    assert not any(key in first[0] for key in ('name', 'identity', 'registered'))
    assert pipeline.process(frame, False, False, now=1.5, face=True) == []
    assert len(pipeline.process(frame, False, False, now=3., face=True)) == 1
    # Disabling face work while other features run clears its capture tracks.
    pipeline.process(frame, person=True, plate=False, face=False, now=3.1)
    assert len(pipeline.process(frame, False, False, now=3.2, face=True)) == 1
    assert len(calls) == 4


def test_all_three_features_share_one_yolo_and_face_is_independent():
    frame = np.zeros((300, 500, 3), np.uint8)
    calls = []
    def detect(image):
        calls.append(1)
        return [(0, .9, [5, 10, 60, 240]), (2, .8, [100, 20, 450, 270])]
    pipeline = Pipeline(detect, plate=lambda crop: [['鲁A12345', .95, 0]],
                        face=lambda image: [(-1, .96, [10, 20, 50, 70])])
    first = pipeline.process(frame, face=True, now=1.)
    assert {event['event_type'] for event in first} == {'face_capture', 'person'}
    second = pipeline.process(frame, face=True, now=1.2)
    assert [event['event_type'] for event in second] == ['vehicle_plate']
    assert len(calls) == 2


def test_bundled_yunet_model_accepts_full_hd_and_small_frames():
    model = Path(__file__).resolve().parents[2] / 'detector_node/models/face_detection_yunet.onnx'
    face = YunetCpu(model)
    assert face(np.zeros((1080, 1920, 3), np.uint8)) == []
    assert face(np.zeros((70, 100, 3), np.uint8)) == []
