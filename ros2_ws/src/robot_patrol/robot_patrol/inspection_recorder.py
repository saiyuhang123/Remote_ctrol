"""Minimal inspection-summary writer.

It is deliberately not a general event log. It follows only active local
route runs and records the inspection date plus counts of captured faces,
detected people and deduplicated plates. Gas, PTZ, control, disconnect and
service-health events never enter this database.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from patrol_interfaces.msg import PatrolStatus
from .store import PatrolStore


class InspectionRecorder(Node):
    def __init__(self, **kwargs):
        super().__init__("inspection_recorder", **kwargs)
        default_database = (Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
                            / "robot_patrol" / "patrol.sqlite3")
        self.declare_parameter("database_path", str(default_database))
        database_path = os.environ.get("ROBOT_PATROL_DB") or str(
            self.get_parameter("database_path").value)
        self._store = PatrolStore(database_path)
        self._lock = threading.RLock()
        self._run_id = ""
        self._map_id = ""
        self._route_id = ""
        self._date = ""
        self._faces = 0
        self._persons = 0
        self._plates: dict[str, int] = {}

        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PatrolStatus, "/patrol/status", self._on_status, status_qos)
        self.create_subscription(String, "/vision/events", self._on_event, 50)
        self.get_logger().info(
            f"inspection_recorder ready; database={database_path}")

    def _save(self) -> None:
        with self._lock:
            if not self._run_id:
                return
            plates = [{"plate": plate, "count": count}
                      for plate, count in self._plates.items()]
            self._store.upsert_inspection_summary(
                self._run_id, self._date, self._map_id, self._route_id,
                self._faces, self._persons, plates,
            )

    def _begin(self, run_id: str, map_id: str, route_id: str) -> None:
        with self._lock:
            if self._run_id == run_id:
                self._map_id = map_id or self._map_id
                self._route_id = route_id or self._route_id
                return
            self._run_id = run_id
            self._map_id = map_id
            self._route_id = route_id
            self._date = time.strftime("%Y-%m-%d", time.localtime())
            self._faces = 0
            self._persons = 0
            self._plates = {}
        self._save()

    def _clear(self) -> None:
        with self._lock:
            run_id = self._run_id
            if run_id:
                # Persist the last event counts before the action terminal
                # status closes the run. No separate completion row is needed.
                self._save()
            self._run_id = ""
            self._map_id = ""
            self._route_id = ""
            self._faces = 0
            self._persons = 0
            self._plates = {}

    def _on_status(self, message: PatrolStatus) -> None:
        run_id = str(message.run_id or "")
        route_id = str(message.route_id or "")
        # Return-home uses the same status topic but deliberately has no route
        # id; it is not an inspection route and must not create a summary.
        if bool(message.active) and run_id and route_id and not run_id.startswith("return_"):
            self._begin(run_id, str(message.map_id or ""), route_id)
        elif not bool(message.active) or not route_id:
            self._clear()

    def _on_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
            event_type = str(event.get("event_type", ""))
        except (TypeError, ValueError):
            return
        with self._lock:
            if not self._run_id:
                return
            if event_type == "face_capture":
                self._faces += 1
            elif event_type == "person":
                self._persons += 1
            elif event_type == "vehicle_plate":
                plate = str(event.get("label") or event.get("plate") or
                            event.get("plate_text") or "").strip()
                if plate:
                    self._plates[plate] = self._plates.get(plate, 0) + 1
                else:
                    return
            else:
                return
        self._save()

    def destroy_node(self):
        self._clear()
        self._store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InspectionRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
