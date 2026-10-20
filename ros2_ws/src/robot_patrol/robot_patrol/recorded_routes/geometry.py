"""Ordered vehicle poses; yaw is body heading even while reversing."""
from __future__ import annotations

import math
import statistics
from ..models import PatrolError

# Direction is judged over at least this much displacement. Shorter steps are
# within the localization/frame-correction noise floor, so they cannot reliably
# distinguish body-axis travel from a small lateral pose correction.
DIRECTION_BASELINE = 0.08


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def distance(a, b):
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def pose(value):
    result = {k: float(value[k]) for k in ('x', 'y', 'yaw')}
    if not all(math.isfinite(v) for v in result.values()):
        raise PatrolError('INVALID_DATA', '道路坐标必须为有限数值')
    result['yaw'] = angle(result['yaw'])
    return result


def length(samples):
    return sum(distance(a, b) for a, b in zip(samples, samples[1:]))


def project(samples, point, begin=0, end=None):
    """Project to ordered segments, returning (distance, index, fraction, pose)."""
    best = None
    end = min(len(samples) - 1, end if end is not None else len(samples) - 1)
    for i in range(max(0, begin), end):
        a, b = samples[i:i+2]
        dx, dy = b['x'] - a['x'], b['y'] - a['y']
        den = dx*dx + dy*dy
        t = max(0.0, min(1.0, ((point['x']-a['x'])*dx +
                                (point['y']-a['y'])*dy)/den)) if den > 1e-12 else 0.0
        q = {'x': a['x'] + t*dx, 'y': a['y'] + t*dy,
             'yaw': angle(a['yaw'] + t*angle(b['yaw'] - a['yaw']))}
        item = (distance(q, point), i, t, q)
        if best is None or item[0] < best[0]:
            best = item
    if best is None:
        raise PatrolError('INVALID_DATA', '道路片段少于两个有效点')
    return best


def _settled_stop_tail(raw):
    """Remove a bounded localization overshoot at a confirmed terminal stop.

    Keep the measured, settled endpoint. A pose rebound after braking is not
    a recorded reverse manoeuvre. Reconcile only the last 15 cm, along an
    almost straight approach, with both stationary feedback and a settled
    endpoint. Every original interval is validated before reaching here.
    """
    if abs(raw[-1]['v']) > 0.03:
        return raw
    stop = len(raw)-1
    while stop > 0 and abs(raw[stop-1]['v']) <= 0.03:
        stop -= 1
    end = raw[-1]
    if stop == 0 or end['t']-raw[stop]['t'] < 0.8:
        return raw
    gear = 1 if raw[stop-1]['v'] > 0 else -1
    tail = raw[stop:]
    if any(distance(p, end) > 0.15 or abs(angle(p['yaw']-end['yaw'])) > 0.10
           for p in tail):
        return raw

    # Require the final pose to have actually settled, rather than treating
    # any low-speed movement as a stop. Old drafts can have sparse samples.
    stable = len(raw)-1
    while stable > stop and distance(raw[stable-1], end) <= 0.03 and \
            abs(angle(raw[stable-1]['yaw']-end['yaw'])) <= 0.05:
        stable -= 1
    if end['t']-raw[stable]['t'] < 0.8:
        return raw

    # Preserve slow real reversals, including those below the stop threshold.
    reverse_travel = sum(max(0.0, -gear*(a['v']+b['v'])/2)*(b['t']-a['t'])
                         for a, b in zip(tail, tail[1:]))
    if reverse_travel >= 0.025:
        return raw
    c, s = math.cos(end['yaw']), math.sin(end['yaw'])

    def along(p):
        return gear*((p['x']-end['x'])*c+(p['y']-end['y'])*s)

    # A lateral shift alone is not a braking overshoot. Its original invalid
    # motion must remain visible to the usual direction/curvature checks.
    if max(along(p) for p in tail) < 0.035:
        return raw
    for i in range(stop-1, -1, -1):
        p = raw[i]
        ds = distance(p, end)
        if ds > 0.15 or abs(angle(p['yaw']-end['yaw'])) > 0.10 or gear*p['v'] < -0.01:
            break
        if ds < DIRECTION_BASELINE or along(p) > -0.9*ds:
            continue
        # The settled endpoint must lie within 6 cm of the observed approach;
        # do not turn a localization jump into a newly invented road.
        if project(raw[i:stop+1], end)[0] <= 0.06:
            return raw[:i+1]+[end]
    return raw


def _recording_poses(raw):
    """Debounce recorded poses, keeping endpoints and confirmed gear cusps.

    LIO velocities near zero can change sign at every scan. Those samples
    remain in the raw draft, but must not create millimetre reverse legs.
    Pose-only imported geometry keeps the original, exact processing path.
    """
    if not all('t' in p and 'v' in p for p in raw):
        return raw
    if any(not math.isfinite(float(p[key])) for p in raw for key in ('t', 'v')):
        raise PatrolError('INVALID_DATA', '录制时间或速度必须为有限数值')
    if any(b['t'] <= a['t'] for a, b in zip(raw, raw[1:])):
        raise PatrolError('INVALID_DATA', '录制时间不连续，请检查定位或补录')
    raw = _settled_stop_tail(raw)

    # Integrate displacement along the BODY axis, including while reversing.
    # Four centimetres is a direction decision baseline, not a goal tolerance.
    resolution = 0.04
    travel = [0.0]
    for a, b in zip(raw, raw[1:]):
        yaw = a['yaw'] + 0.5 * angle(b['yaw'] - a['yaw'])
        travel.append(travel[-1] + (b['x']-a['x'])*math.cos(yaw) +
                      (b['y']-a['y'])*math.sin(yaw))
    phases = []
    start = extreme = direction = 0
    for i in range(1, len(raw)):
        if not direction:
            if abs(travel[i]-travel[start]) >= resolution:
                direction = 1 if travel[i] > travel[start] else -1
                extreme = i
        elif direction*(travel[i]-travel[extreme]) >= 0:
            extreme = i
        elif direction*(travel[i]-travel[extreme]) <= -resolution:
            phases.append([start, extreme, direction])
            start, direction, extreme = extreme, -direction, i
    if direction:
        phases.append([start, len(raw)-1, direction])

    def stationary_drift(begin, end, gear):
        # Suppress only bounded, almost fixed-heading, low-speed residence.
        # Short manoeuvres and sustained low-speed travel are retained.
        if raw[end]['t']-raw[begin]['t'] < 0.5:
            return False
        if any(distance(raw[begin], p) > 0.08 or
               abs(angle(p['yaw']-raw[begin]['yaw'])) > 0.10
               for p in raw[begin:end+1]):
            return False
        if statistics.median(abs(p['v']) for p in raw[begin:end+1]) > 0.03:
            return False
        # A short real reversal can follow a long stop. Look for a local
        # interval with both geometric travel and agreeing measured velocity,
        # rather than averaging that reversal into the preceding residence.
        moving = [0]
        velocity = [0.0]
        for p in raw[begin:end+1]:
            signed = gear*p['v']
            moving.append(moving[-1] + (signed > 0.005))
            velocity.append(velocity[-1] + signed)
        for j in range(begin+1, end+1):
            # Recording runs at 10Hz. Bound work even for recovered data with
            # pathological sub-microsecond timestamps.
            for i in range(j-1, max(begin-1, j-33), -1):
                if raw[j]['t']-raw[i]['t'] > 8.0:
                    break
                if gear*(travel[j]-travel[i]) < 0.035:
                    continue
                count = j-i
                lo, hi = i-begin+1, j-begin+1
                if (moving[hi]-moving[lo]) >= 0.7*count and \
                        (velocity[hi]-velocity[lo]) >= 0.0075*count:
                    return False
        return True

    kept = []
    for begin, end, gear in phases:
        if stationary_drift(begin, end, gear):
            if kept:
                kept[-1][1] = end
            continue
        if not kept:
            begin = 0
        if kept and kept[-1][2] == gear:
            kept[-1][1] = end
        else:
            kept.append([begin, end, gear])
    if not kept:
        if any(distance(raw[0], p) >= 0.08 for p in raw):
            raise PatrolError('INVALID_DATA', '位移方向与车体朝向不一致，请检查定位或补录')
        raise PatrolError('INVALID_DATA', '道路有效位移不足，停车抖动不能作为道路；请实际行驶后再分段')

    clean = []
    for begin, end, gear in kept:
        # Check the original poses over a meaningful displacement baseline,
        # separately for each gear. A baseline across a true cusp could reject
        # a valid short forward/reverse parking manoeuvre as sideways motion.
        anchor = begin
        for i in range(begin+1, end+1):
            a, b = raw[anchor], raw[i]
            ds = distance(a, b)
            if ds < DIRECTION_BASELINE:
                continue
            yaw = a['yaw'] + 0.5 * angle(b['yaw']-a['yaw'])
            along = (b['x']-a['x'])*math.cos(yaw) + (b['y']-a['y'])*math.sin(yaw)
            if gear*along < ds*0.6:
                lateral = math.sqrt(max(0.0, ds*ds-along*along))
                raise PatrolError('INVALID_DATA',
                                  f'位移方向与车体朝向不一致（位移{ds*100:.1f}cm、横移{lateral*100:.1f}cm），请检查定位后补录')
            anchor = i
        indices = [begin]
        limit = gear*(travel[end]-travel[begin])
        for i in range(begin+1, end):
            if gear*(travel[i]-travel[begin]) > limit + 1e-9:
                continue
            if gear*(travel[i]-travel[indices[-1]]) >= resolution:
                indices.append(i)
        if indices[-1] != end:
            # Absorb an uncertain short terminal tail into the preceding
            # baseline, while retaining the exact measured endpoint pose.
            if len(indices) > 1 and distance(raw[indices[-1]], raw[end]) < resolution:
                indices.pop()
            indices.append(end)
        clean.extend(raw[i] for i in indices if not clean or raw[i] is not clean[-1])
    return clean


def motion_blocks(samples, spacing=0.05):
    """Keep every cusp. Never interpolate heading across a gear change."""
    if not 2 <= len(samples) <= 200000:
        raise PatrolError('INVALID_DATA', '道路需包含 2～200000 个采样点')
    raw = [dict(item, **pose(item)) for item in samples]
    # Validate every original interval before filtering so a jump or in-place
    # turn cannot be concealed by denoising.
    for a, b in zip(raw, raw[1:]):
        ds = distance(a, b)
        if ds < 0.003:
            if abs(angle(b['yaw'] - a['yaw'])) > 0.15:
                raise PatrolError('INVALID_DATA', '停车时朝向跳变或原地旋转，请补录')
        if ds > 0.6:
            raise PatrolError('INVALID_DATA', '道路采样间断超过 0.6m，禁止直线补齐')
    raw = _recording_poses(raw)
    blocks = []
    for a, b in zip(raw, raw[1:]):
        ds = distance(a, b)
        if ds < 0.003:
            continue
        along = (b['x']-a['x'])*math.cos(a['yaw']) + (b['y']-a['y'])*math.sin(a['yaw'])
        # Do not reject a road on a single sub-baseline step. A 5 cm pose
        # correction at a gear cusp was exactly the width of the localization
        # noise floor and blocked an otherwise valid recording; direction is
        # still checked over DIRECTION_BASELINE windows in _recording_poses.
        if ds >= DIRECTION_BASELINE and abs(along) < ds * 0.6:
            lateral = math.sqrt(max(0.0, ds*ds-along*along))
            raise PatrolError('INVALID_DATA',
                              f'位移方向与车体朝向不一致（位移{ds*100:.1f}cm、横移{lateral*100:.1f}cm），请检查定位后补录')
        direction = 1 if along > 0 else -1
        if not blocks or blocks[-1]['direction'] != direction:
            blocks.append({'direction': direction, 'samples': [pose(a)], 'barriers': []})
        block = blocks[-1]
        n = max(1, math.ceil(ds / spacing))
        for k in range(1, n + 1):
            t = k/n
            block['samples'].append({'x': a['x']+t*(b['x']-a['x']),
                                     'y': a['y']+t*(b['y']-a['y']),
                                     'yaw': angle(a['yaw']+t*angle(b['yaw']-a['yaw']))})
    if not blocks or sum(length(b['samples']) for b in blocks) < 0.10:
        raise PatrolError('INVALID_DATA', '道路长度不足 0.10m')
    return blocks


def validate_blocks(blocks, radius):
    for block in blocks:
        samples = block['samples']
        # Anchor curvature baselines in the positive BODY direction. The same
        # geometry must have the same verdict when traversed in reverse;
        # otherwise a shifted 25cm grouping changes its noise sensitivity.
        if block['direction'] < 0:
            samples = list(reversed(samples))
        # Use an arc-length baseline to avoid amplifying centimetre pose noise.
        # 0.25 m is long enough that a one-frame yaw blip at a segment start
        # does not look like a 0.6 m turn, while sustained over-curvature still
        # exceeds 1/radius over the same distance.
        j = 0
        for i in range(1, len(samples)):
            ds = length(samples[j:i+1])
            if ds >= 0.25 or i == len(samples)-1:
                if ds > 0.03 and abs(angle(samples[i]['yaw']-samples[j]['yaw'])) / ds > 1/radius + 0.15:
                    raise PatrolError('KINEMATIC_INVALID', '轨迹曲率超过车辆约束，请补录更宽的转弯')
                j = i


def join_edges(edges):
    """Join equal-gear roads continuously, retaining non-skippable junctions."""
    result = []
    for occurrence, edge in enumerate(edges):
        for source in edge['blocks']:
            block = dict(source, samples=[dict(p) for p in source['samples']],
                         barriers=list(source.get('barriers', [])), speed=float(edge.get('speed', 0.2)))
            block['edges'] = [{'edge_id': edge['id'], 'occurrence': occurrence}]
            if result and result[-1]['direction'] == block['direction']:
                prev = result[-1]
                if distance(prev['samples'][-1], block['samples'][0]) > 0.08 or \
                        abs(angle(prev['samples'][-1]['yaw']-block['samples'][0]['yaw'])) > 0.26:
                    raise PatrolError('KINEMATIC_INVALID', '道路连接位姿不连续')
                prev['barriers'].append(len(prev['samples'])-1)
                offset = len(prev['samples'])-1
                prev['barriers'].extend(offset+i for i in block['barriers'])
                prev['samples'].extend(block['samples'][1:])
                prev['edges'].extend(block['edges'])
                prev['speed'] = min(prev['speed'], block['speed'])
            else:
                result.append(block)
    return result
