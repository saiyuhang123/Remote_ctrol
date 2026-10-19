"""Ordered vehicle poses; yaw is body heading even while reversing."""
from __future__ import annotations

import math
from ..models import PatrolError


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


def motion_blocks(samples, spacing=0.05):
    """Keep every cusp. Never interpolate heading across a gear change."""
    if not 2 <= len(samples) <= 200000:
        raise PatrolError('INVALID_DATA', '道路需包含 2～200000 个采样点')
    raw = [dict(item, **pose(item)) for item in samples]
    blocks = []
    for a, b in zip(raw, raw[1:]):
        ds = distance(a, b)
        if ds < 0.003:
            if abs(angle(b['yaw'] - a['yaw'])) > 0.15:
                raise PatrolError('INVALID_DATA', '停车时朝向跳变或原地旋转，请补录')
            continue
        if ds > 0.6:
            raise PatrolError('INVALID_DATA', '道路采样间断超过 0.6m，禁止直线补齐')
        along = (b['x']-a['x'])*math.cos(a['yaw']) + (b['y']-a['y'])*math.sin(a['yaw'])
        if abs(along) < ds * 0.6:
            raise PatrolError('INVALID_DATA', '位移方向与车体朝向不一致，请检查定位或补录')
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
    if not blocks or length(raw) < 0.10:
        raise PatrolError('INVALID_DATA', '道路长度不足 0.10m')
    return blocks


def validate_blocks(blocks, radius):
    for block in blocks:
        samples = block['samples']
        # Use an arc-length baseline to avoid amplifying centimetre pose noise.
        j = 0
        for i in range(1, len(samples)):
            ds = length(samples[j:i+1])
            if ds >= 0.15 or i == len(samples)-1:
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
