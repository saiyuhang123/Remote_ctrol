"""OCR scheduling and event behavior without a camera or inference models."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detector_node.detection_pipeline import Pipeline


class Plates:
    def __init__(self):
        self.visible = True
        self.offset = 0

    def __call__(self, frame):
        if not self.visible:
            return []
        x = 20 + self.offset
        return [dict(confidence=.99, bbox=[x, 20, x+100, 50],
                     corners=[[x, 20], [x+100, 20], [x+100, 50], [x, 50]])]


class OCR:
    def __init__(self):
        self.calls = 0
        self.text = '粤B12345'
        self.confidence = .99

    def __call__(self, image):
        self.calls += 1
        return self.text, self.confidence


@pytest.fixture
def scene():
    plates, ocr = Plates(), OCR()
    pipeline = Pipeline(plate_detector=plates, plate=ocr, plate_ocr_interval=2.)
    frame = np.zeros((100, 300, 3), dtype=np.uint8)

    def process(now):
        return pipeline.process(frame, person=False, plate=True, now=now)

    return pipeline, plates, ocr, process


def confirm(scene):
    pipeline, plates, ocr, process = scene
    assert process(0.) == []
    events = process(.25)
    assert len(events) == 1 and events[0]['plate_text'] == ocr.text
    assert ocr.calls == 2


def test_stable_confirmed_plate_is_rechecked_without_duplicate_event(scene):
    pipeline, plates, ocr, process = scene
    confirm(scene)
    for now in [.5, .75, 1., 1.25, 1.5, 1.75, 2.]:
        assert process(now) == []
    assert ocr.calls == 2
    assert process(2.25) == []
    assert ocr.calls == 3
    assert pipeline.plate_ocr_calls == 3
    assert pipeline.plate_ocr_skipped == 7


def test_changed_text_requires_fast_confirmation_again(scene):
    pipeline, plates, ocr, process = scene
    confirm(scene)
    for now in [.5, 1., 1.5, 2.]:
        process(now)
    ocr.text = '粤B54321'
    assert process(2.25) == []
    events = process(2.5)
    assert len(events) == 1 and events[0]['plate_text'] == ocr.text
    assert ocr.calls == 4


def test_unreadable_plate_retries_without_waiting_for_cooldown(scene):
    pipeline, plates, ocr, process = scene
    confirm(scene)
    for now in [.5, 1., 1.5, 2.]:
        process(now)
    ocr.confidence = .3
    assert process(2.25) == []
    ocr.confidence = .99
    process(2.5)
    process(2.75)
    assert ocr.calls == 5


def test_missed_detection_forces_immediate_recheck(scene):
    pipeline, plates, ocr, process = scene
    confirm(scene)
    plates.visible = False
    process(.5)
    plates.visible = True
    process(.75)
    assert ocr.calls == 3


def test_large_box_change_forces_immediate_recheck(scene):
    pipeline, plates, ocr, process = scene
    confirm(scene)
    plates.offset = 30  # IoU .54: same track, but no longer stable.
    process(.5)
    assert ocr.calls == 3


@pytest.mark.parametrize('reset', [False, True])
def test_expired_or_reset_tracks_require_confirmation_again(scene, reset):
    pipeline, plates, ocr, process = scene
    confirm(scene)
    if reset:
        pipeline.reset()
    assert process(3.) == []
    assert len(process(3.25)) == 1
    assert ocr.calls == 4


def test_zero_interval_preserves_every_frame_ocr(scene):
    pipeline, plates, ocr, process = scene
    pipeline.plate_ocr_interval = 0.
    for now in [0., .25, .5, .75, 1.]:
        process(now)
    assert ocr.calls == 5
    assert pipeline.plate_ocr_skipped == 0


@pytest.mark.parametrize('interval', [-1., 61., float('nan'), float('inf')])
def test_invalid_interval_is_rejected(interval):
    with pytest.raises(ValueError):
        Pipeline(plate_ocr_interval=interval)


def test_other_detection_events_keep_their_existing_cadence():
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    person = lambda _: [(0, .99, [0, 0, 50, 100])]
    face = lambda _: [(-1, .99, [0, 0, 30, 30])]
    pipeline = Pipeline(detector=person, face=face)
    initial = pipeline.process(frame, person=True, plate=False, face=True, now=0.)
    assert {event['event_type'] for event in initial} == {'person', 'face_capture'}
    assert pipeline.process(frame, person=True, plate=False, face=True, now=.25) == []
    later = pipeline.process(frame, person=True, plate=False, face=True, now=2.)
    assert [event['event_type'] for event in later] == ['face_capture']
