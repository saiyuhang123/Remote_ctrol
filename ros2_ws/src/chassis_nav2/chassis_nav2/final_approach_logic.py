"""Dual-Ackermann final precise approach logic, without ROS dependencies.

Nav2 brings the chassis near the goal with a coarse tolerance; this module
closes the last meters. Every tick it computes the shortest Reeds-Shepp path
from the current pose to the goal pose (curvature-feasible, reverse segments
included) and executes the first segment — re-planning every tick turns the
open-loop path into a feedback policy. If no RS word applies (rare), it falls
back to curvature-limited pursuit of an overshoot point past the goal.
"""
import math

from chassis_nav2.reeds_shepp import best_path


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FinalApproachLogic:
    RUNNING, DONE, FAILED = 'running', 'done', 'failed'

    def __init__(self, speed=.12, max_curvature=1.45, position_tolerance=.03,
                 yaw_tolerance=.05, settle_time=.5, timeout=120., overshoot=.5):
        self.speed = speed
        self.max_curvature = max_curvature
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.settle_time = settle_time
        self.timeout = timeout
        self.overshoot = overshoot
        self.reset()

    def reset(self):
        self.elapsed = 0.
        self.settled = 0.
        self.path_length = None  # 当前 RS 路径剩余全长（m），供状态播报

    def update(self, dx, dy, dyaw, dt):
        """One control tick; dx/dy/dyaw is the goal in base_link; returns
        (linear_x, angular_z, status)."""
        self.elapsed += dt
        if self.elapsed > self.timeout:
            return 0., 0., self.FAILED
        if (math.hypot(dx, dy) < self.position_tolerance
                and abs(dyaw) < self.yaw_tolerance):
            self.path_length = 0.
            self.settled += dt
            if self.settled >= self.settle_time:
                return 0., 0., self.DONE
            return 0., 0., self.RUNNING
        self.settled = 0.
        path = best_path(dx, dy, dyaw, self.max_curvature)
        if path is None:
            self.path_length = None
            return self._approach(dx, dy, dyaw)
        lengths, ctypes = path
        self.path_length = sum(abs(l) for l in lengths) / self.max_curvature
        # 跳过退化（长度≈0）的首段，执行第一个有效段
        for length, ctype in zip(lengths, ctypes):
            if abs(length) > 1e-3:
                direction = 1. if length > 0 else -1.
                kappa = {'L': self.max_curvature, 'R': -self.max_curvature,
                         'S': 0.}[ctype]
                v = self.speed * direction
                return v, v * kappa, self.RUNNING
        return 0., 0., self.RUNNING

    def _approach(self, dx, dy, dyaw):
        # 兜底：瞄准目标沿其朝向再过冲一段的点，让车沿目标朝向"穿过"目标。
        # 过冲距离自适应：侧向偏差越大，过冲点越远，直到所需曲率可行（留 20% 余量）
        d = self.overshoot
        while True:
            bx = dx + d * math.cos(dyaw)
            by = dy + d * math.sin(dyaw)
            denom = bx * bx + by * by
            kappa = 2 * by / denom if denom > 1e-6 else 0.
            if abs(kappa) <= .8 * self.max_curvature or d >= 2.:
                break
            d += .1
        # 前进/倒车按目标点的前后决定；目标明显在身后才倒车
        v = self.speed if dx > -.05 else -self.speed
        kappa = max(-self.max_curvature, min(self.max_curvature, kappa))
        return v, v * kappa, self.RUNNING
