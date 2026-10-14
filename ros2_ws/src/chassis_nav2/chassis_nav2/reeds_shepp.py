"""Reeds-Shepp path words in the base frame (unit turning radius space).

Adapted from the PythonRobotics reference implementation
(PathPlanning/ReedsSheppPath, MIT License, (c) Atsushi Sakai and co-authors).

Only path generation is kept: the final-approach controller re-plans every
tick and executes the first segment, so interpolation and plotting are
dropped. Lengths are signed (negative = reverse) and normalized by curvature.
"""
import math


def _mod2pi(x):
    v = math.fmod(x, 2 * math.pi)
    if v < -math.pi:
        v += 2 * math.pi
    elif v > math.pi:
        v -= 2 * math.pi
    return v


def _polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _lsl(x, y, phi):
    u, t = _polar(x - math.sin(phi), y - 1. + math.cos(phi))
    if 0. <= t <= math.pi:
        v = _mod2pi(phi - t)
        if 0. <= v <= math.pi:
            return True, [t, u, v], ['L', 'S', 'L']
    return False, [], []


def _lsr(x, y, phi):
    u1, t1 = _polar(x + math.sin(phi), y - 1. - math.cos(phi))
    u1 = u1 ** 2
    if u1 >= 4.:
        u = math.sqrt(u1 - 4.)
        theta = math.atan2(2., u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        if t >= 0. and v >= 0.:
            return True, [t, u, v], ['L', 'S', 'R']
    return False, [], []


def _lxl_xl(x, y, phi):
    zeta = x - math.sin(phi)
    eeta = y - 1. + math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 <= 4.:
        A = math.acos(.25 * u1)
        t = _mod2pi(A + theta + math.pi / 2)
        u = _mod2pi(math.pi - 2 * A)
        v = _mod2pi(phi - t - u)
        return True, [t, -u, v], ['L', 'R', 'L']
    return False, [], []


def _lxl(x, y, phi):
    zeta = x - math.sin(phi)
    eeta = y - 1. + math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 <= 4.:
        A = math.acos(.25 * u1)
        t = _mod2pi(A + theta + math.pi / 2)
        u = _mod2pi(math.pi - 2 * A)
        v = _mod2pi(-phi + t + u)
        return True, [t, -u, -v], ['L', 'R', 'L']
    return False, [], []


def _lrxl(x, y, phi):
    zeta = x - math.sin(phi)
    eeta = y - 1. + math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 <= 4.:
        u = math.acos(1 - u1 ** 2 * .125)
        A = math.asin(2 * math.sin(u) / u1)
        t = _mod2pi(-A + theta + math.pi / 2)
        v = _mod2pi(t - u - phi)
        return True, [t, u, -v], ['L', 'R', 'L']
    return False, [], []


def _lrxlr(x, y, phi):
    zeta = x + math.sin(phi)
    eeta = y - 1. - math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    # Solutions with 2 < u1 <= 4 are sub-optimal per the paper; u1 > 4 has none.
    if u1 <= 2:
        A = math.acos((u1 + 2) * .25)
        t = _mod2pi(theta + A + math.pi / 2)
        u = _mod2pi(A)
        v = _mod2pi(phi - t + 2 * u)
        if t >= 0. and u >= 0. and v >= 0.:
            return True, [t, u, -u, -v], ['L', 'R', 'L', 'R']
    return False, [], []


def _lxlxr(x, y, phi):
    zeta = x + math.sin(phi)
    eeta = y - 1. - math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    u2 = (20 - u1 ** 2) / 16
    if 0 <= u2 <= 1:
        u = math.acos(u2)
        A = math.asin(2 * math.sin(u) / u1)
        t = _mod2pi(theta + A + math.pi / 2)
        v = _mod2pi(t - phi)
        if t >= 0. and v >= 0.:
            return True, [t, -u, -u, v], ['L', 'R', 'L', 'R']
    return False, [], []


def _lxr90sl(x, y, phi):
    zeta = x - math.sin(phi)
    eeta = y - 1. + math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 >= 2.:
        u = math.sqrt(u1 ** 2 - 4) - 2
        A = math.atan2(2, math.sqrt(u1 ** 2 - 4))
        t = _mod2pi(theta + A + math.pi / 2)
        v = _mod2pi(t - phi + math.pi / 2)
        if t >= 0. and v >= 0.:
            return True, [t, -math.pi / 2, -u, -v], ['L', 'R', 'S', 'L']
    return False, [], []


def _lsr90xl(x, y, phi):
    zeta = x - math.sin(phi)
    eeta = y - 1. + math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 >= 2.:
        u = math.sqrt(u1 ** 2 - 4) - 2
        A = math.atan2(math.sqrt(u1 ** 2 - 4), 2)
        t = _mod2pi(theta - A + math.pi / 2)
        v = _mod2pi(t - phi - math.pi / 2)
        if t >= 0. and v >= 0.:
            return True, [t, u, math.pi / 2, -v], ['L', 'S', 'R', 'L']
    return False, [], []


def _lxr90sr(x, y, phi):
    zeta = x + math.sin(phi)
    eeta = y - 1. - math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 >= 2.:
        t = _mod2pi(theta + math.pi / 2)
        u = u1 - 2
        v = _mod2pi(phi - t - math.pi / 2)
        if t >= 0. and v >= 0.:
            return True, [t, -math.pi / 2, -u, -v], ['L', 'R', 'S', 'R']
    return False, [], []


def _lsl90xr(x, y, phi):
    zeta = x + math.sin(phi)
    eeta = y - 1. - math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 >= 2.:
        t = _mod2pi(theta)
        u = u1 - 2
        v = _mod2pi(phi - t - math.pi / 2)
        if t >= 0. and v >= 0.:
            return True, [t, u, math.pi / 2, -v], ['L', 'S', 'L', 'R']
    return False, [], []


def _lxr90sl90xr(x, y, phi):
    zeta = x + math.sin(phi)
    eeta = y - 1. - math.cos(phi)
    u1, theta = _polar(zeta, eeta)
    if u1 >= 4.:
        u = math.sqrt(u1 ** 2 - 4) - 4
        A = math.atan2(2, math.sqrt(u1 ** 2 - 4))
        t = _mod2pi(theta + A + math.pi / 2)
        v = _mod2pi(t - phi)
        if t >= 0. and v >= 0.:
            return True, [t, -math.pi / 2, -u, -math.pi / 2, v], ['L', 'R', 'S', 'L', 'R']
    return False, [], []


_WORDS = [_lsl, _lsr, _lxl_xl, _lxl, _lrxl, _lrxlr, _lxlxr,
          _lxr90sl, _lxr90sr, _lsr90xl, _lsl90xr, _lxr90sl90xr]


def _reflect(ctypes):
    return [{'L': 'R', 'R': 'L'}.get(c, 'S') for c in ctypes]


def generate_paths(dx, dy, dyaw, max_curvature):
    """Goal (dx, dy, dyaw) in the base frame; returns [(signed lengths, ctypes)]."""
    x, y, phi = dx * max_curvature, dy * max_curvature, dyaw
    paths = []
    for word in _WORDS:
        ok, lengths, ctypes = word(x, y, phi)
        if ok:
            paths.append((lengths, ctypes))
        ok, lengths, ctypes = word(-x, y, -phi)
        if ok:
            paths.append(([-l for l in lengths], ctypes))
        ok, lengths, ctypes = word(x, -y, -phi)
        if ok:
            paths.append((lengths, _reflect(ctypes)))
        ok, lengths, ctypes = word(-x, -y, phi)
        if ok:
            paths.append(([-l for l in lengths], _reflect(ctypes)))
    return paths


def best_path(dx, dy, dyaw, max_curvature):
    """Shortest feasible path as (lengths, ctypes), or None if no word applies."""
    paths = generate_paths(dx, dy, dyaw, max_curvature)
    if not paths:
        return None
    return min(paths, key=lambda p: sum(abs(l) for l in p[0]))
