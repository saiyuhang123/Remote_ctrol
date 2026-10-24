"""CSV compatibility, ordered arc-length tracking and swept-body collision checks.

The ROS 1 chain was global_planner -> local_planner -> control/purePursuitNode.
This fixed-route port keeps recorded geometry and ordered Pure Pursuit. Obstacles
stop the vehicle; they never authorize departure from the recorded route.
"""
from __future__ import annotations
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    speed: float | None = None


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def quaternion_yaw(values):
    qx, qy, qz, qw = values
    norm = math.sqrt(sum(x*x for x in values))
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('四元数无效')
    qx, qy, qz, qw = (x/norm for x in values)
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))


def load_csv(filename, legacy_spacing=.08):
    """Read ROS 1 x,y,z,qx,qy,qz,qw and new header-based CSVs."""
    points, inferred, header, legacy = [], [], None, True
    with Path(filename).expanduser().open(newline='', encoding='utf-8-sig') as stream:
        for line, row in enumerate(csv.reader(stream), 1):
            if not row or all(not x.strip() for x in row) or row[0].lstrip().startswith('#'):
                continue
            try:
                if not points and header is None and row[0].strip().lower() == 'x':
                    header = [key.strip().lower() for key in row]
                    legacy = False
                    if len(set(header)) != len(header) or not {'x', 'y'} <= set(header):
                        raise ValueError('CSV 表头无效')
                    continue
                values = [float(value.strip()) for value in row]
                if not all(math.isfinite(x) for x in values):
                    raise ValueError('坐标/速度包含 NaN 或 Inf')
                yaw_missing = False
                if header is not None:
                    if len(values) != len(header):
                        raise ValueError('列数与表头不同')
                    data = dict(zip(header, values))
                    if {'qx', 'qy', 'qz', 'qw'} <= data.keys():
                        yaw = quaternion_yaw([data[k] for k in ('qx', 'qy', 'qz', 'qw')])
                    elif 'yaw' in data:
                        yaw = data['yaw']
                    else:
                        yaw, yaw_missing = 0., True
                    point = Point(data['x'], data['y'], data.get('z', 0.), yaw, data.get('speed'))
                elif len(values) in (7, 8):
                    point = Point(*values[:3], quaternion_yaw(values[3:7]), values[7] if len(values) == 8 else None)
                    legacy = legacy and len(values) == 7
                elif len(values) in (2, 3):
                    legacy = False
                    point = Point(values[0], values[1], values[2] if len(values) == 3 else 0.)
                    yaw_missing = True
                else:
                    raise ValueError('支持 2/3 列坐标、旧 7 列位姿或带表头的新 CSV')
                if points and distance(points[-1], point) < .001:
                    # ROS 1 rostopic recordings contain many identical stationary samples.
                    continue
                points.append(point)
                inferred.append(yaw_missing)
                if len(points) > 200000:
                    raise ValueError('路线超过 200000 点，请分段')
            except (ValueError, KeyError) as exc:
                raise ValueError(f'CSV 第 {line} 行：{exc}') from exc
    if len(points) < 2:
        raise ValueError('路线至少需要两个不同位置的点')
    if legacy and len(points) > 2 and legacy_spacing > 0:
        # ROS 1 rostopic recordings contain millimetre/centimetre localization
        # jitter even while stationary. Thin only legacy files without signed
        # speed metadata; native recordings retain every short motion boundary.
        sampled = [points[0]]
        for point in points[1:]:
            if distance(sampled[-1], point) >= legacy_spacing:
                sampled.append(point)
        endpoint = points[-1]
        tail = distance(sampled[-1], endpoint)
        if .001 <= tail < legacy_spacing/2 and len(sampled) > 1:
            sampled[-1] = endpoint
        elif tail >= .001:
            sampled.append(endpoint)
        if len(sampled) < 2:
            raise ValueError('旧录制数据仅有静止抖动，没有有效路线')
        return sampled
    for i, missing in enumerate(inferred):
        if missing:
            a, b = (points[i], points[i+1]) if i+1 < len(points) else (points[i-1], points[i])
            p = points[i]
            points[i] = Point(p.x, p.y, p.z, math.atan2(b.y-a.y, b.x-a.x), p.speed)
    return points


@dataclass
class Block:
    points: list[Point]
    direction: int

    def __post_init__(self):
        self.arc = [0.]
        for a, b in zip(self.points, self.points[1:]):
            self.arc.append(self.arc[-1] + distance(a, b))
        self.length = self.arc[-1]

    def at(self, arc):
        arc = min(self.length, max(0., arc))
        i = min(len(self.points)-2, max(0, bisect.bisect_right(self.arc, arc)-1))
        a, b = self.points[i:i+2]
        ratio = (arc-self.arc[i])/(self.arc[i+1]-self.arc[i])
        return Point(a.x+ratio*(b.x-a.x), a.y+ratio*(b.y-a.y), a.z+ratio*(b.z-a.z),
                     a.yaw+ratio*wrap(b.yaw-a.yaw), a.speed)

    def project(self, pose, low, high):
        """Restrict projection to nearby ordered segments, including at crossings."""
        first = max(0, bisect.bisect_right(self.arc, low)-1)
        last = min(len(self.points)-2, bisect.bisect_right(self.arc, high)-1)
        best = (math.inf, low)
        for i in range(first, last+1):
            a, b = self.points[i:i+2]
            dx, dy = b.x-a.x, b.y-a.y
            length = self.arc[i+1]-self.arc[i]
            ratio = ((pose.x-a.x)*dx+(pose.y-a.y)*dy)/(length*length)
            ratio = min(min(1., (high-self.arc[i])/length),
                        max(max(0., (low-self.arc[i])/length), ratio))
            arc = self.arc[i]+ratio*length
            error = math.hypot(pose.x-a.x-ratio*dx, pose.y-a.y-ratio*dy)
            best = min(best, (error, arc))
        return best


def motion_blocks(points, allow_reverse=False, max_segment=1.0):
    blocks, begin, previous = [], 0, None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = distance(a, b)
        if length < .001 or length > max_segment:
            raise ValueError(f'第 {i+1} 段长度 {length:.3f}m 无效，存在重复点或录制断点')
        tangent = math.atan2(b.y-a.y, b.x-a.x)
        projection = math.cos(wrap(tangent-a.yaw))
        if abs(projection) < .5:
            raise ValueError(f'第 {i+1} 段位移与车头朝向不符')
        direction = 1 if projection > 0 else -1
        if a.speed is not None and abs(a.speed) > .03 and a.speed*direction < 0:
            raise ValueError(f'第 {i+1} 段录制速度与位移方向不符')
        if direction < 0 and not allow_reverse:
            raise ValueError('路线含倒车段；需 allow_reverse:=true 且雷达覆盖后方才能执行')
        if previous is not None and direction != previous:
            blocks.append(Block(points[begin:i+1], previous))
            begin = i
        previous = direction
    blocks.append(Block(points[begin:], previous))
    return blocks


class Tracker:
    def __init__(self, blocks, radius, max_cross_track=.45, min_lookahead=.45,
                 max_lookahead=.8, lookahead_time=1., goal_tolerance=.1,
                 goal_yaw_tolerance=.35):
        self.blocks, self.radius = blocks, radius
        self.max_cross_track, self.min_lookahead = max_cross_track, min_lookahead
        self.max_lookahead, self.lookahead_time = max_lookahead, lookahead_time
        self.goal_tolerance, self.goal_yaw_tolerance = goal_tolerance, goal_yaw_tolerance
        self.index, self.progress, self.target = 0, 0., blocks[0].points[0]

    @property
    def block(self):
        return self.blocks[self.index]

    def advance(self):
        if self.index+1 == len(self.blocks):
            return False
        self.index += 1
        self.progress = 0.
        return True

    def command(self, pose, measured_speed, speed_limit, deceleration, dt=.05):
        block = self.block
        error, arc = block.project(pose, max(0., self.progress-.05),
                                   min(block.length, self.progress+max(.3, abs(measured_speed)*dt*2+.1)))
        if error > self.max_cross_track:
            raise ValueError(f'偏离录制路线 {error:.2f}m，请暂停后检查定位或人工接回原路线')
        self.progress = max(self.progress, arc)
        remaining = block.length-self.progress
        end = block.points[-1]
        # Short blocks use a proportionate tolerance; a closed loop cannot finish at its start.
        tolerance = min(self.goal_tolerance, block.length/8.)
        endpoint_distance = distance(pose, end)
        if remaining <= tolerance and endpoint_distance <= tolerance:
            if abs(wrap(end.yaw-pose.yaw)) > self.goal_yaw_tolerance:
                raise ValueError('已接近端点但朝向不满足，停止；底盘不能原地转向')
            return 0., 0., True
        reference = block.at(self.progress)
        if abs(wrap(reference.yaw-pose.yaw)) > 1.0:
            raise ValueError('车头朝向偏离录制路线超过 1rad')
        lookahead = min(self.max_lookahead, max(self.min_lookahead,
                                              self.min_lookahead+abs(measured_speed)*self.lookahead_time))
        self.target = block.at(self.progress+lookahead)
        dx, dy = self.target.x-pose.x, self.target.y-pose.y
        local_x = math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy
        local_y = -math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
        if local_x*block.direction < -.02:
            raise ValueError('前视点已位于允许行驶方向后方，停止以防越过端点')
        curvature = 2*local_y/max(dx*dx+dy*dy, 1e-6)
        curvature = max(-1/self.radius, min(1/self.radius, curvature))
        # Keep recorded speeds when present, bounded by this vehicle's limits.
        speed = speed_limit
        if reference.speed is not None and abs(reference.speed) > .03:
            speed = min(speed, abs(reference.speed))
        speed = min(speed, math.sqrt(2*deceleration*max(remaining, endpoint_distance, 0.)),
                    math.sqrt(.35/max(abs(curvature), 1e-6)))
        velocity = block.direction*speed
        return velocity, velocity*curvature, False


def point_in_padded_polygon(x, y, polygon, padding):
    inside = False
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm = dx*dx+dy*dy
        ratio = min(1., max(0., ((x-a[0])*dx+(y-a[1])*dy)/norm)) if norm else 0.
        if math.hypot(x-a[0]-ratio*dx, y-a[1]-ratio*dy) <= padding:
            return True
        if (a[1] > y) != (b[1] > y) and x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:
            inside = not inside
    return inside


def swept_collision(points, polygon, velocity, curvature, deceleration, margin):
    """Check full hardware footprint along the commanded arc through a full stop."""
    horizon = velocity*velocity/(2*deceleration)+abs(velocity)*.5+.15
    direction = 1 if velocity >= 0 else -1
    for i in range(math.ceil(horizon/.04)+1):
        travel = direction*min(horizon, i*.04)
        yaw = curvature*travel
        x = math.sin(yaw)/curvature if abs(curvature) > 1e-6 else travel
        y = (1-math.cos(yaw))/curvature if abs(curvature) > 1e-6 else 0.
        c, s = math.cos(yaw), math.sin(yaw)
        for px, py in points:
            if point_in_padded_polygon(c*(px-x)+s*(py-y), -s*(px-x)+c*(py-y), polygon, margin):
                return True
    return False
