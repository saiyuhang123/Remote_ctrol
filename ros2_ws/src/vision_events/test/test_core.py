import json
from pathlib import Path
import time
import cv2
import numpy as np
import pytest
from vision_events.core import Store, PoseHistory, archive, canonical_event


def event(key='one', at=1_000_000_000):
    return canonical_event(dict(event_type='face_capture', source='hikvision_sdk',
                                source_event_key=key, event_time_ns=at, face_id=7))


def test_all_captures_are_kept_but_message_replay_is_idempotent(tmp_path):
    store = Store(tmp_path, upload=True)
    for i in range(20):
        item = event(str(i))
        assert store.insert(item)
        assert not store.insert(item)
    assert store.query()[2] == 20
    assert store.pending() is not None
    first, cursor, _ = store.query(limit=7)
    second, _, _ = store.query(limit=7, cursor=cursor)
    assert len(first) == len(second) == 7
    assert not ({v['event_id'] for v in first} & {v['event_id'] for v in second})
    item = store.pending()
    store.delivered(item['event_id'])
    store.close()
    reopened = Store(tmp_path, upload=True)
    assert reopened.query()[2] == 20
    pending = []
    while reopened.pending():
        current = reopened.pending()
        pending.append(current['event_id'])
        reopened.delivered(current['event_id'])
    assert len(pending) == 19
    reopened.close()


def test_upload_can_be_enabled_after_local_only_recording(tmp_path):
    store = Store(tmp_path)
    store.insert(event())
    store.close()
    store = Store(tmp_path, upload=True)
    assert store.pending()['event_id'] == event()['event_id']
    store.close()


def test_pose_uses_event_time_and_never_new_map_or_stale_coordinates():
    history = PoseHistory()
    assert not history.associate(10**9)['pose_valid']
    history.context('A', at_ns=10**9)
    history.add(2*10**9, 1., 2., .3)
    history.add(3*10**9, 4., 5., .6)
    old = history.associate(2*10**9)
    assert old['pose_valid'] and old['robot_x'] == 1.
    assert not history.associate(4*10**9)['pose_valid']
    history.context('B', at_ns=4*10**9)
    history.add(5*10**9, 100., 100., 0.)
    assert history.associate(2*10**9)['pose_status'] == 'context_changed'
    assert history.associate(2*10**9)['robot_x'] is None
    assert history.associate(5*10**9)['map_id'] == 'B'


def test_media_is_durable_and_paths_are_constrained(tmp_path):
    native = tmp_path/'native'; native.mkdir()
    image = native/'sample.jpg'
    assert cv2.imwrite(str(image), np.full((60, 60, 3), 127, np.uint8))
    value = event()
    value['image_path'] = str(image)
    original = image.read_bytes()
    result = archive(value, tmp_path, [native])
    assert Path(result['image_path']).read_bytes() == original
    image.unlink()
    assert cv2.imread(result['image_path']).shape == (60, 60, 3)
    result['image_path'] = '/etc/passwd'
    with pytest.raises(ValueError, match='outside'):
        archive(result, tmp_path, [native])


def test_invalid_payload_and_cursor_are_rejected(tmp_path):
    with pytest.raises(ValueError):
        canonical_event(dict(event_type='identity', source_event_key='x', event_time_ns=123))
    store = Store(tmp_path)
    with pytest.raises((ValueError, TypeError)):
        store.query(cursor='bad')
    store.close()
