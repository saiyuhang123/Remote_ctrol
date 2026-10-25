"""Pure data models and validation for local patrol data."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable
import math
import time
import uuid


class PatrolError(Exception):
    """A typed error that can be mapped to a service/action response."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex}"


def now_ns() -> int:
    return time.time_ns()


def require_finite(value: float, field_name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise PatrolError("INVALID_DATA", f"{field_name} 必须是有限数值")
    return value


def validate_name(name: str, label: str = "名称") -> str:
    value = (name or "").strip()
    if not value:
        raise PatrolError("INVALID_DATA", f"{label}不能为空")
    if len(value) > 120:
        raise PatrolError("INVALID_DATA", f"{label}过长")
    return value


@dataclass
class MapBinding:
    map_id: str
    name: str
    backend: str
    frame_id: str
    navigation_yaml: str
    image_path: str
    localization_file: str
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    navigation_yaml_sha256: str
    image_sha256: str
    localization_sha256: str
    verified: bool
    revision: int
    created_at_ns: int = 0
    updated_at_ns: int = 0


@dataclass
class Point:
    point_id: str
    map_id: str
    name: str
    x: float
    y: float
    yaw: float
    source: str
    sampled_at_ns: int
    revision: int
    created_at_ns: int = 0
    updated_at_ns: int = 0
    inspection_enabled: bool = False
    ptz_preset: int = 0
    recognition_prompt: str = ""
    shots: list = field(default_factory=list)


@dataclass
class RouteStep:
    point_id: str
    dwell_seconds: float
    order: int = 0
    step_id: str = ""
    point_name: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    inspection_enabled: bool = False
    ptz_preset: int = 0
    recognition_prompt: str = ""
    shots: list = field(default_factory=list)


@dataclass
class Route:
    route_id: str
    map_id: str
    name: str
    steps: list[RouteStep] = field(default_factory=list)
    revision: int = 1
    created_at_ns: int = 0
    updated_at_ns: int = 0
    navigation_mode: str = "FREE_NAV"
    network_id: str = ""
    network_revision: int = 0


@dataclass
class RunRecord:
    run_id: str
    route_id: str
    map_id: str
    state: int
    completed_loops: int
    completed_steps: int
    success: bool
    error_code: str
    message: str
    route_revision: int
    map_revision: int
    started_at_ns: int
    finished_at_ns: int


@dataclass
class RouteSnapshot:
    route_id: str
    map_id: str
    name: str
    revision: int
    steps: list[RouteStep]
    navigation_mode: str = "FREE_NAV"
    recorded_plan: dict = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "route_id": self.route_id,
            "map_id": self.map_id,
            "name": self.name,
            "revision": self.revision,
            "navigation_mode": self.navigation_mode,
            "recorded_plan": self.recorded_plan,
            "steps": [
                {
                    "point_id": step.point_id,
                    "point_name": step.point_name,
                    "x": step.x,
                    "y": step.y,
                    "yaw": step.yaw,
                    "dwell_seconds": step.dwell_seconds,
                    "order": step.order,
                    "inspection_enabled": bool(getattr(step, "inspection_enabled", False)),
                    "ptz_preset": int(getattr(step, "ptz_preset", 0)),
                    "recognition_prompt": str(getattr(step, "recognition_prompt", "") or ""),
                    "shots": [shot.as_dict() for shot in getattr(step, "shots", [])],
                }
                for step in self.steps
            ],
        }



@dataclass
class HomePoint:
    map_id: str
    map_revision: int
    x: float
    y: float
    yaw: float
    updated_at_ns: int = 0


@dataclass
class InspectionSummary:
    run_id: str
    inspection_date: str
    map_id: str
    route_id: str
    face_capture_count: int = 0
    person_count: int = 0
    plates: list[dict[str, Any]] = field(default_factory=list)
    updated_at_ns: int = 0

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "inspection_date": self.inspection_date,
            "map_id": self.map_id,
            "route_id": self.route_id,
            "face_capture_count": self.face_capture_count,
            "person_count": self.person_count,
            "plates": list(self.plates),
            "updated_at": self.updated_at_ns / 1_000_000_000.0,
        }

@dataclass
class InspectionResult:
    result_id: str
    run_id: str
    map_id: str
    point_id: str
    point_name: str
    loop_index: int
    step_index: int
    started_at_ns: int
    finished_at_ns: int
    ptz_preset: int
    prompt: str
    image_path: str
    text: str
    model: str
    success: bool
    error_code: str = ""
    message: str = ""
    shot_id: str = ""
    shot_name: str = ""
    metadata: dict = field(default_factory=dict)


def validate_inspection_settings(enabled: bool, preset: int,
                                   prompt: str, has_shots: bool = False) -> tuple[bool, int, str]:
    enabled = bool(enabled)
    try:
        preset = int(preset)
    except (TypeError, ValueError) as error:
        raise PatrolError("INVALID_DATA", "云台预置位编号必须是整数") from error
    if has_shots and not 0 <= preset <= 255:
        raise PatrolError("INVALID_DATA", "云台预置位编号必须在 0～255 之间")
    if enabled and not has_shots and not (1 <= preset <= 255):
        raise PatrolError("INVALID_DATA", "启用巡检动作时必须选择 1～255 的云台预置位")
    return enabled, preset, str(prompt or "")[:2000]


def validate_point_values(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    return (require_finite(x, "x"), require_finite(y, "y"), require_finite(yaw, "yaw"))


def validate_dwell(value: float) -> float:
    value = require_finite(value, "dwell_seconds")
    if value < 0.0 or value > 86400.0:
        raise PatrolError("INVALID_DATA", "停留时间必须在 0～86400 秒之间")
    return value


def validate_route_steps(steps: Iterable[Any]) -> list[tuple[str, float]]:
    result: list[tuple[str, float]] = []
    for step in steps:
        point_id = getattr(step, "point_id", "")
        if not point_id:
            raise PatrolError("INVALID_DATA", "路线步骤缺少点位 ID")
        result.append((point_id, validate_dwell(getattr(step, "dwell_seconds", 0.0))))
    if not result:
        raise PatrolError("INVALID_DATA", "路线至少需要一个步骤")
    if len(result) > 500:
        raise PatrolError("INVALID_DATA", "路线步骤不能超过 500 个")
    return result


def validate_loop_count(value: int) -> int:
    if value < 0:
        raise PatrolError("INVALID_DATA", "循环圈数不能为负数；0 表示持续循环")
    if value > 100000:
        raise PatrolError("INVALID_DATA", "循环圈数过大")
    return int(value)
