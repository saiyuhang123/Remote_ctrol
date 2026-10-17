from types import SimpleNamespace
import threading
import time

from robot_patrol.engine import PatrolEngine, STATE_CANCELED, STATE_FAILED, STATE_PAUSED
from robot_patrol.models import RouteSnapshot
from robot_patrol.store import PatrolStore


class FakeDriver:
    def __init__(self, pause_once=False):
        self.calls = []
        self.pause_once = pause_once

    def cancel_current(self):
        pass

    def navigate(self, step, flags, status_cb):
        self.calls.append(step.point_id)
        if self.pause_once:
            self.pause_once = False
            flags.request_pause()
            return "PAUSED"
        return "SUCCEEDED"

    def settle(self, step, flags, status_cb):
        return ("OK", (step.x, step.y, step.yaw))

    def reconfirm(self, step, flags, status_cb):
        return "AT_POINT"


def snapshot(points, dwell=0.0):
    return RouteSnapshot(
        route_id="route", map_id="map", name="route", revision=1,
        steps=[SimpleNamespace(point_id=p, point_name=p, x=float(i), y=0.0, yaw=0.0,
                               dwell_seconds=dwell, order=i)
               for i, p in enumerate(points)],
    )


def make_store(tmp_path):
    store = PatrolStore(tmp_path / "patrol.sqlite3")
    mapping = store.upsert_map({
        "name": "map", "backend": "amcl", "frame_id": "map",
        "navigation_yaml": "/tmp/map.yaml", "image_path": "/tmp/map.pgm",
        "localization_file": "", "resolution": 0.05, "origin_x": 0.0,
        "origin_y": 0.0, "origin_yaw": 0.0, "width": 10, "height": 10,
        "navigation_yaml_sha256": "a" * 64, "image_sha256": "b" * 64,
        "localization_sha256": "", "verified": True,
    })
    return store, mapping


def test_three_points_two_loops_sequence(tmp_path):
    store, mapping = make_store(tmp_path)
    driver = FakeDriver()
    statuses = []
    run_id = "run1"
    store.create_run(run_id, "route", mapping.map_id, 1, mapping.revision,
                     snapshot(["A", "B", "C"]).as_dict(), 2, 1, 1)
    engine = PatrolEngine(store, snapshot(["A", "B", "C"]), run_id, 2, "owner",
                          driver, statuses.append, settle_timeout=1.0, point_timeout=1.0,
                          min_dispatch_interval=0.0)
    result = engine.run()
    assert result.success
    assert driver.calls == ["A", "B", "C", "A", "B", "C"]
    assert result.completed_loops == 2
    assert result.completed_steps == 6
    assert statuses[-1]["state"] == 10
    store.close()


def test_pause_while_navigation_redispatches_same_point(tmp_path):
    store, mapping = make_store(tmp_path)
    driver = FakeDriver(pause_once=True)
    run_id = "run1"
    store.create_run(run_id, "route", mapping.map_id, 1, mapping.revision,
                     snapshot(["A", "B"]).as_dict(), 1, 1, 1)
    engine = PatrolEngine(store, snapshot(["A", "B"]), run_id, 1, "owner",
                          driver, lambda status: None, settle_timeout=1.0,
                          point_timeout=1.0, min_dispatch_interval=0.0)
    threading.Timer(0.2, engine.request_resume).start()
    result = engine.run()
    assert result.success
    assert driver.calls == ["A", "A", "B"]
    store.close()


def test_pause_during_dwell_reconfirms_and_repositions(tmp_path):
    class OffPointDriver(FakeDriver):
        def __init__(self):
            super().__init__()
            self.reconfirms = 0

        def reconfirm(self, step, flags, status_cb):
            self.reconfirms += 1
            return "OFF_POINT" if self.reconfirms == 1 else "AT_POINT"

    store, mapping = make_store(tmp_path)
    driver = OffPointDriver()
    route = snapshot(["A"], dwell=0.3)
    run_id = "run1"
    store.create_run(run_id, "route", mapping.map_id, 1, mapping.revision,
                     route.as_dict(), 1, 1, 1)
    dwelling = threading.Event()
    def status_cb(status):
        if status.get("state") == 5:
            dwelling.set()
    engine = PatrolEngine(store, route, run_id, 1, "owner", driver,
                          status_cb, settle_timeout=1.0,
                          point_timeout=1.0, min_dispatch_interval=0.0)
    def pause_resume():
        assert dwelling.wait(2.0)
        engine.request_pause()
        time.sleep(0.1)
        engine.request_resume()
    threading.Thread(target=pause_resume, daemon=True).start()
    result = engine.run()
    assert result.success
    assert driver.reconfirms == 1
    assert driver.calls == ["A", "A"]


def test_cancel_before_dispatch_does_not_send_goal(tmp_path):
    store, mapping = make_store(tmp_path)
    driver = FakeDriver()
    run_id = "run1"
    store.create_run(run_id, "route", mapping.map_id, 1, mapping.revision,
                     snapshot(["A", "B"]).as_dict(), 0, 1, 1)
    engine = PatrolEngine(store, snapshot(["A", "B"]), run_id, 0, "owner",
                          driver, lambda status: None, settle_timeout=1.0,
                          point_timeout=1.0, min_dispatch_interval=0.0)
    engine.request_cancel()
    result = engine.run()
    assert not result.success
    assert result.final_state == STATE_CANCELED
    assert driver.calls == []
    store.close()


def test_unsafe_pause_cannot_redispatch_after_resume(tmp_path):
    class UnsafePauseDriver(FakeDriver):
        def __init__(self):
            super().__init__()
            self.safe = True

        def stop_safe(self):
            return self.safe

        def navigate(self, step, flags, status_cb):
            self.calls.append(step.point_id)
            # Simulate the acceptance-timeout path: this navigation operation
            # reports PAUSED while its old child goal is not actually stopped.
            self.safe = False
            flags.request_pause()
            return "PAUSED"

    store, mapping = make_store(tmp_path)
    driver = UnsafePauseDriver()
    route = snapshot(["A"])
    run_id = "run-unsafe-pause"
    store.create_run(run_id, "route", mapping.map_id, 1, mapping.revision,
                     route.as_dict(), 1, 1, 1)
    paused = threading.Event()

    def status_cb(status):
        if status.get("state") == STATE_PAUSED:
            paused.set()

    engine = PatrolEngine(store, route, run_id, 1, "owner", driver,
                          status_cb, settle_timeout=1.0,
                          point_timeout=1.0, min_dispatch_interval=0.0)

    def resume_after_pause():
        assert paused.wait(2.0)
        engine.request_resume()

    threading.Thread(target=resume_after_pause, daemon=True).start()
    result = engine.run()
    assert not result.success
    assert result.final_state == STATE_FAILED
    assert result.error_code == "CANCEL_UNCONFIRMED"
    assert driver.calls == ["A"], "a second Nav2 goal was dispatched after unsafe resume"
    store.close()
