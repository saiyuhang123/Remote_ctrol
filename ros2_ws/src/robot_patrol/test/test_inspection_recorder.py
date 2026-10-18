import json
import threading
from types import SimpleNamespace

from std_msgs.msg import String

from robot_patrol.inspection_recorder import InspectionRecorder


class FakeStore:
    def __init__(self):
        self.rows = []

    def upsert_inspection_summary(self, run_id, date, map_id, route_id,
                                  faces, persons, plates):
        self.rows.append({
            "run_id": run_id, "date": date, "map_id": map_id,
            "route_id": route_id, "faces": faces, "persons": persons,
            "plates": plates,
        })


def _recorder():
    node = object.__new__(InspectionRecorder)
    node._lock = threading.RLock()
    node._run_id = ""
    node._map_id = ""
    node._route_id = ""
    node._date = ""
    node._faces = 0
    node._persons = 0
    node._plates = {}
    node._store = FakeStore()
    return node


def test_inspection_recorder_only_counts_during_active_route():
    node = _recorder()
    node._on_event(String(data=json.dumps({"event_type": "person"})))
    assert node._store.rows == []

    node._on_status(SimpleNamespace(
        active=True, run_id="run_1", route_id="route_1", map_id="map_1"))
    assert node._store.rows[0]["faces"] == 0
    node._on_event(String(data=json.dumps({"event_type": "face_capture"})))
    node._on_event(String(data=json.dumps({"event_type": "person"})))
    node._on_event(String(data=json.dumps({
        "event_type": "vehicle_plate", "label": "京A12345"})))
    node._on_event(String(data=json.dumps({
        "event_type": "vehicle_plate", "label": "京A12345"})))
    row = node._store.rows[-1]
    assert row["faces"] == 1
    assert row["persons"] == 1
    assert row["plates"] == [{"plate": "京A12345", "count": 2}]

    # Return-home status uses the same topic but has no route id.
    node._on_status(SimpleNamespace(
        active=True, run_id="return_123", route_id="", map_id="map_1"))
    assert node._run_id == ""
    node._on_event(String(data=json.dumps({"event_type": "person"})))
    assert node._store.rows[-1]["persons"] == 1
