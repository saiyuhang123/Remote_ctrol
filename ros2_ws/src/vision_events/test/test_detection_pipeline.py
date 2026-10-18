import numpy as np
from detector_node.detection_pipeline import Pipeline


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
