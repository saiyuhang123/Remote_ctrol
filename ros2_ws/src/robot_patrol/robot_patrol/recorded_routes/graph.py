"""Directed, heading-aware recorded roads. No spatial auto-connections."""
from __future__ import annotations

import copy
import hashlib
import heapq
import json
import math

from .geometry import (DIRECTION_BASELINE, angle, distance, pose, length, project,
                       join_edges, validate_blocks)
from ..models import PatrolError


def digest(value):
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                    separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def validate_network(doc, radius=0.69):
    if not math.isfinite(radius) or radius<=0:
        raise PatrolError('INVALID_DATA','底盘最小转弯半径无效')
    if doc.get('schema_version') != 1:
        raise PatrolError('INVALID_DATA', '不支持的路网文件版本')
    nodes, edges = doc.get('nodes', {}), doc.get('edges', {})
    if not isinstance(nodes, dict) or not isinstance(edges, dict) or len(nodes) > 3000 or len(edges) > 10000:
        raise PatrolError('INVALID_DATA', '路网规模或格式无效')
    if sum(len(b.get('samples', [])) for edge in edges.values() for b in edge.get('blocks', [])) > 200000:
        raise PatrolError('INVALID_DATA', '路网采样总数超过 200000，请拆分路网')
    for node_id, node in nodes.items():
        if not node_id or node.get('id') != node_id:
            raise PatrolError('INVALID_DATA', '道路节点 ID 无效')
        pose(node)
    for edge_id, edge in edges.items():
        if edge.get('id') != edge_id or edge.get('from') not in nodes or edge.get('to') not in nodes:
            raise PatrolError('INVALID_DATA', '道路引用了不存在的节点')
        if not math.isfinite(float(edge.get('speed', 0))) or not 0.03 <= edge['speed'] <= 0.5:
            raise PatrolError('INVALID_DATA', '道路限速必须为 0.03～0.50m/s')
        blocks = edge.get('blocks', [])
        if not blocks or len(blocks) > 500:
            raise PatrolError('INVALID_DATA', '道路运动段为空或过多')
        for block in blocks:
            if block.get('direction') not in (-1, 1) or not 2 <= len(block.get('samples', [])) <= 200000:
                raise PatrolError('INVALID_DATA', '道路方向或采样数量无效')
            for p in block['samples']:
                pose(p)
            for a, b in zip(block['samples'], block['samples'][1:]):
                ds = distance(a, b)
                if ds > 0.20:
                    raise PatrolError('INVALID_DATA', '道路采样过稀，请重新处理')
                along = (b['x']-a['x'])*math.cos(a['yaw']) + (b['y']-a['y'])*math.sin(a['yaw'])
                # A sub-baseline step (for example a 5 cm pose correction at a
                # gear cusp) cannot establish a contradictory road direction.
                if ds >= DIRECTION_BASELINE and along*block['direction'] < ds*0.6:
                    raise PatrolError('INVALID_DATA', '道路方向与车体朝向矛盾')
        validate_blocks(blocks, radius)
        chain = [blocks[0]['samples'][0]] + [b['samples'][-1] for b in blocks]
        for previous, following in zip(blocks, blocks[1:]):
            if distance(previous['samples'][-1], following['samples'][0]) > 0.02 or \
                    abs(angle(previous['samples'][-1]['yaw']-following['samples'][0]['yaw'])) > 0.10:
                raise PatrolError('KINEMATIC_INVALID', '换向点位姿不连续')
        for actual, node in ((chain[0], nodes[edge['from']]), (chain[-1], nodes[edge['to']])):
            if distance(actual, node) > 0.04 or abs(angle(actual['yaw']-node['yaw'])) > 0.12:
                raise PatrolError('KINEMATIC_INVALID', '道路端点未与所选节点对齐，请补录连接段')
    for point_id, values in doc.get('bindings', {}).items():
        if not point_id or not isinstance(values, list) or not 1 <= len(values) <= 8 or any(v not in nodes for v in values):
            raise PatrolError('INVALID_DATA', '巡检点道路绑定无效')
    polygon = doc.get('allowed_polygon', [])
    if polygon and (len(polygon) < 3 or len(polygon) > 1000 or
                    any(len(p) != 2 or not all(math.isfinite(float(v)) for v in p) for p in polygon)):
        raise PatrolError('INVALID_DATA', '允许通行区域多边形格式无效')
    return doc


class NetworkPlanner:
    def __init__(self, document):
        self.doc = copy.deepcopy(document)
        self.nodes, self.edges = self.doc['nodes'], self.doc['edges']
        self.adj = {key: [] for key in self.nodes}
        self._cache = {}
        for edge_id, edge in self.edges.items():
            if edge.get('enabled', True):
                cost = sum(length(b['samples']) for b in edge['blocks'])
                self.adj[edge['from']].append((cost, edge['to'], edge_id))

    def shortest(self, start, target):
        key = (start, target)
        if key in self._cache:
            return self._cache[key]
        queue, best = [(0.0, start, [])], {start: 0.0}
        while queue:
            cost, node, route = heapq.heappop(queue)
            if cost > best[node] + 1e-9:
                continue
            if node == target:
                self._cache[key] = (cost, route)
                return cost, route
            for add, end, edge_id in self.adj.get(node, []):
                new_cost = cost + add
                if new_cost < best.get(end, math.inf):
                    best[end] = new_cost
                    heapq.heappush(queue, (new_cost, end, route + [edge_id]))
        return math.inf, []

    def start_candidates(self, current):
        candidates = []
        for node_id, node in self.nodes.items():
            if distance(current, node) <= 0.12 and abs(angle(current['yaw']-node['yaw'])) <= 0.26:
                candidates.append((node_id, 0.0, []))
        # Mid-road entry may only trim the beginning of one *unambiguous* block.
        # Overlapping visits/cusps require manual entry at a named node.
        if candidates:
            nearest = min(distance(current, self.nodes[item[0]]) for item in candidates)
            return [item for item in candidates if distance(current, self.nodes[item[0]]) <= nearest+0.02]
        projections = []
        for edge in self.edges.values():
            if not edge.get('enabled', True) or len(edge['blocks']) != 1:
                continue
            points = edge['blocks'][0]['samples']
            d, i, t, q = project(points, current)
            if d <= 0.12 and abs(angle(current['yaw']-q['yaw'])) <= 0.26:
                tail = points[i+1:]
                if len(tail) < 1:
                    continue
                trimmed = copy.deepcopy(edge)
                trimmed['blocks'][0]['samples'] = [q] + tail
                projections.append((d, edge['to'], length([q]+tail), trimmed))
        if projections:
            # Two explicitly recorded directions on the same road are valid
            # alternatives for a new task. An ambiguous revisit *within one
            # edge* is not: its traversal occurrence cannot be guessed.
            valid = []
            for _, end, cost, edge in projections:
                original = self.edges[edge['id']]['blocks'][0]['samples']
                _, nearest, _, _ = project(original,current)
                ambiguous = any(distance(point,current)<0.10 and abs(i-nearest)>6 and
                                abs(angle(point['yaw']-current['yaw']))<0.26
                                for i,point in enumerate(original))
                if not ambiguous:
                    valid.append((end,cost,[edge]))
            if valid:
                return valid
        raise PatrolError('OFF_ROAD', '当前位置未在可确定进度的录制道路上，请手动驶入节点并对齐')

    def _bindings(self, step):
        nodes = self.doc.get('bindings', {}).get(step.point_id, [])
        valid = [n for n in nodes if n in self.nodes and
                 distance(self.nodes[n], {'x': step.x, 'y': step.y}) <= 0.08]
        if not valid:
            raise PatrolError('POINT_NOT_BOUND', f'巡检点“{step.point_name}”未绑定有效道路节点，请补录或重新绑定')
        return valid

    def cycle(self, node):
        best = (math.inf, [])
        for cost, end, edge_id in self.adj.get(node, []):
            remaining, ids = self.shortest(end, node)
            if cost + remaining < best[0]:
                best = cost+remaining, [edge_id]+ids
        return best

    def plan(self, steps, current, loops=1, single_stop_loop=True):
        if not steps:
            raise PatrolError('INVALID_DATA', '巡检路线为空')
        starts = self.start_candidates(current)
        bindings = [self._bindings(step) for step in steps]
        overall = None
        # Enumerate first arrival state so a repeated cycle closes to that same
        # body-heading state. DP then keeps all feasible states at each stop.
        for first in bindings[0]:
            entries = []
            for source, prefix_cost, prefix in starts:
                cost, ids = self.shortest(source, first)
                if math.isfinite(cost):
                    entries.append((prefix_cost+cost, prefix, ids))
            if not entries:
                continue
            entry = min(entries, key=lambda item: item[0])
            cycle_ids = []
            cycle_cost = 0.0
            needs_cycle = len(steps) == 1 and single_stop_loop and (loops != 1 or entry[0] < 0.02)
            if needs_cycle:
                cycle_cost, cycle_ids = self.cycle(first)
                if not math.isfinite(cycle_cost):
                    continue
            states = {first: (entry[0], [first], [entry[1]+[self.edges[e] for e in entry[2]]])}
            if cycle_ids:
                states[first] = (entry[0]+cycle_cost, [first],
                                 [entry[1]+[self.edges[e] for e in entry[2]+cycle_ids]])
            for index, targets in enumerate(bindings[1:], 1):
                next_states = {}
                for source, (base, nodes, paths) in states.items():
                    for target in targets:
                        add, ids = self.shortest(source, target)
                        if math.isfinite(add) and base+add < next_states.get(target, (math.inf,))[0]:
                            next_states[target] = (base+add, nodes+[target], paths+[[self.edges[e] for e in ids]])
                states = next_states
                if not states:
                    break
            for last, state in states.items():
                back_cost, back_ids = self.shortest(last, first)
                if needs_cycle:
                    back_cost, back_ids = cycle_cost, cycle_ids
                if loops != 1 and not math.isfinite(back_cost):
                    continue
                score = state[0] + (back_cost if loops != 1 else 0.0)
                if overall is None or score < overall[0]:
                    overall = (score, state, back_ids if loops != 1 else [])
        if overall is None:
            order = ' → '.join(s.point_name for s in steps)
            raise PatrolError('ROAD_UNREACHABLE', f'{order} 缺少方向/朝向可行的录制连接（循环需末点返回首点），请补录；不会自由规划')
        _, (_, nodes, paths), back_ids = overall
        legs = [self.leg(path, node, step.point_id) for path, node, step in zip(paths, nodes, steps)]
        closure = self.leg([self.edges[e] for e in back_ids], nodes[0], steps[0].point_id) if loops != 1 else None
        result = {'schema_version': 1, 'legs': legs, 'closure': closure,
                  'loop_count': loops, 'initial_pose': pose(current),
                  'network_sha256': digest(self.doc), 'point_order': [s.point_id for s in steps],
                  'allowed_polygon': self.doc.get('allowed_polygon', [])}
        if sum(len(b['samples']) for leg in legs+([closure] if closure else []) for b in leg['blocks']) > 200000:
            raise PatrolError('INVALID_DATA', '任务路径过长，请分为多条巡检路线')
        result['digest'] = digest(result)
        return result

    def leg(self, edges, target_node, point_id):
        blocks=join_edges(edges)
        validate_blocks(blocks,float(self.doc.get('vehicle',{}).get('minimum_turning_radius',.69)))
        return {'blocks': blocks, 'target': pose(self.nodes[target_node]),
                'target_node': target_node, 'point_id': point_id,
                'edge_ids': [e['id'] for e in edges]}
