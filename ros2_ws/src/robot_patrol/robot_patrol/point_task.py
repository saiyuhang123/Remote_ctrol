"""One-off targets reuse patrol snapshots without editing points or routes."""
import math

from .models import PatrolError, Route, RouteStep, new_id, require_finite, validate_inspection_settings


def point_route(store, request, mapping):
    if request.point_id:
        point = store.get_point(request.point_id)
        if point.map_id != mapping.map_id:
            raise PatrolError('MAP_MISMATCH', '巡检点不属于当前地图')
        if not request.point_revision or request.point_revision != point.revision:
            raise PatrolError('STALE_DATA', '巡检点版本已变化，请重新选择')
        step = RouteStep(point_id=point.point_id, dwell_seconds=0., point_name=point.name,
                         x=point.x, y=point.y, yaw=point.yaw,
                         inspection_enabled=point.inspection_enabled, ptz_preset=point.ptz_preset,
                         recognition_prompt=point.recognition_prompt, shots=list(point.shots))
    else:
        enabled, preset, prompt = validate_inspection_settings(
            request.inspection_enabled, request.ptz_preset, request.recognition_prompt)
        x, y, yaw = (require_finite(getattr(request, key), key) for key in ('x', 'y', 'yaw'))
        step = RouteStep(point_id=new_id('target'), dwell_seconds=0.,
                         point_name=f'地图点 ({x:.2f}, {y:.2f})', x=x, y=y,
                         yaw=math.atan2(math.sin(yaw), math.cos(yaw)),
                         inspection_enabled=enabled, ptz_preset=preset, recognition_prompt=prompt)
    dx, dy = step.x-mapping.origin_x, step.y-mapping.origin_y
    c, s = math.cos(mapping.origin_yaw), math.sin(mapping.origin_yaw)
    lx, ly = c*dx+s*dy, -s*dx+c*dy
    if not (0 <= lx < mapping.width*mapping.resolution and
            0 <= ly < mapping.height*mapping.resolution):
        raise PatrolError('INVALID_DATA', '目标点超出当前地图范围')
    return Route(route_id=new_id('adhoc'), map_id=mapping.map_id,
                 name='一键到达 · '+step.point_name, steps=[step], revision=1)
