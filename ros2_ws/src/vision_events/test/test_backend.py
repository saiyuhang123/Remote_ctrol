"""The current read-only web must neither upload nor expose local vision files.

Old api_records/api_vision write routes were removed during web consolidation.
Local persistence/idempotence is exercised in test_core and standalone ROS tests.
"""
from fastapi.testclient import TestClient


def test_local_archive_does_not_require_or_reenable_legacy_web_uploads(tmp_path, monkeypatch):
    from backend.main import app
    from vision_events.core import Store, canonical_event
    monkeypatch.setenv('VISION_DATA_DIR', str(tmp_path))
    image = tmp_path/'private.jpg'; image.write_bytes(b'private-local-media')
    store = Store(tmp_path)
    try:
        event = canonical_event(dict(event_type='face_capture', source='host_yunet',
                                     source_event_key='one', event_time_ns=1, image_path=str(image)))
        store.insert(event)
        with TestClient(app) as client:
            assert client.post('/api/records/detections', json=event).status_code in (404, 405)
            assert client.get('/api/vision/media/'+event['event_id']).status_code == 404
        assert store.query()[2] == 1 and image.read_bytes() == b'private-local-media'
    finally:
        store.close()
