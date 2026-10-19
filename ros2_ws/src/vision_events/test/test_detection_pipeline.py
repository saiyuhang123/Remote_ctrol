import numpy as np
from pathlib import Path
from detector_node.detection_pipeline import Pipeline, YunetCpu, YoloCpu, LpdYunetCpu, PlateTextCpu, rectify_plate


def plate_detections(_frame):
    return [dict(confidence=.95, bbox=[100, 80, 450, 170],
                 corners=[[100, 80], [450, 80], [450, 170], [100, 170]])]


def test_people_do_not_wait_for_faces_and_reentry_creates_new_record():
    frame = np.zeros((300, 500, 3), np.uint8)
    pipeline = Pipeline(lambda image: [(0, .92, [10, 10, 80, 250])])
    first = pipeline.process(frame, plate=False, now=1.)
    assert len(first) == 1 and first[0]['confidence'] == .92
    assert first[0]['event_type'] == 'person'
    assert pipeline.process(frame, plate=False, now=1.2) == []
    assert len(pipeline.process(frame, plate=False, now=5.)) == 1


def test_person_and_plate_models_are_independent_and_plate_needs_confirmation():
    calls = []
    frame = np.zeros((300, 500, 3), np.uint8)
    def detect(image):
        calls.append(1)
        return [(0, .9, [5, 10, 60, 240])]
    pipeline = Pipeline(detect, plate=lambda crop: ('鲁A12345', .95), plate_detector=plate_detections)
    assert pipeline.process(frame, False, False) == [] and calls == []
    first = pipeline.process(frame, now=1.)
    assert [r['event_type'] for r in first] == ['person']
    second = pipeline.process(frame, now=1.2)
    assert second[0]['plate_text'] == '鲁A12345' and len(calls) == 2
    assert second[0]['image'].shape[:2] == (90, 350)
    assert pipeline.process(frame, now=1.4) == []
    assert len(pipeline.process(frame, person=False, now=5.)) == 0
    later = pipeline.process(frame, person=False, now=5.2)
    assert later[0]['event_type'] == 'vehicle_plate'
    assert len(calls) == 3  # Plate-only work never runs YOLO.


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


def test_all_three_features_each_run_their_selected_model():
    frame = np.zeros((300, 500, 3), np.uint8)
    calls = []
    def detect(image):
        calls.append(1)
        return [(0, .9, [5, 10, 60, 240])]
    pipeline = Pipeline(detect, plate=lambda crop: ('鲁A12345', .95), plate_detector=plate_detections,
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


def test_yolo26_end_to_end_coordinates_person_filter_and_empty_frame():
    output = np.asarray([[[100, 200, 200, 300, .9, 0],
                          [0, 0, 640, 640, .95, 2],
                          [100, 200, 200, 300, .2, 0]]], dtype=np.float32)
    found = YoloCpu.decode(output, .45, 1280, 720, .5, 0, 140)
    assert len(found) == 1 and found[0][2] == [200, 120, 400, 320]
    assert YoloCpu.decode(np.zeros((1, 300, 6), np.float32), .45, 640, 640, 1, 0, 0) == []


def test_plate_text_ctc_repeated_characters_and_perspective_crop():
    text = '粤A00123'
    indices = [0]
    for char in text:
        index = PlateTextCpu.characters.index(char)
        indices.extend([index, index, 0])
    probabilities = np.zeros((1, len(indices), len(PlateTextCpu.characters)), np.float32)
    probabilities[0, np.arange(len(indices)), indices] = .95
    result, score = PlateTextCpu.decode(probabilities)
    assert result == text and score > .94
    frame = np.full((200, 500, 3), 80, np.uint8)
    crop = rectify_plate(frame, [[60, 80], [400, 65], [410, 160], [50, 150]])
    assert crop is not None and crop.shape[1] > crop.shape[0]
    assert rectify_plate(frame, [[0, 0]]*4) is None


def test_bundled_plate_models_work_without_yolo_or_face():
    root = Path(__file__).resolve().parents[2] / 'detector_node/models'
    detector = LpdYunetCpu(root/'license_plate_detection_lpd_yunet_2023mar.onnx')
    assert detector(np.zeros((1080, 1920, 3), np.uint8)) == []
    reader = PlateTextCpu(root/'plate_recognition_hyperlpr3.onnx')
    assert reader.input.shape == [1, 3, 48, 160]
    assert len(reader.characters) == 77


def test_incomplete_plate_text_is_not_saved_or_counted_as_confirmation():
    readings = iter(['鲁A1234', '鲁A1234', '鲁A12345', '鲁A12345'])
    pipeline = Pipeline(plate=lambda image: (next(readings), .99), plate_detector=plate_detections)
    frame = np.zeros((300, 500, 3), np.uint8)
    for now in (1., 1.2, 1.4):
        assert pipeline.process(frame, person=False, now=now) == []
    events = pipeline.process(frame, person=False, now=1.6)
    assert len(events) == 1 and events[0]['plate_text'] == '鲁A12345'
