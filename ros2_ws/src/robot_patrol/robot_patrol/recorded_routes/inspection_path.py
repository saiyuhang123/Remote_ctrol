"""Ordered teaching paths, automatic point creation and exact reverse travel."""
from __future__ import annotations

import copy

from ..models import PatrolError, new_id
from .geometry import angle, distance, pose
from .graph import NetworkPlanner, digest
from .entry import aligned, connector_blocks
from .geometry import length


def path_edges(document):
    """Use recording order, or adopt one unbranched legacy chain/cycle."""
    meta = document.get('inspection_path', {})
    edges = document['edges']
    ids = meta.get('edge_ids')
    if ids is None:
        original = {k: e for k, e in edges.items() if not e.get('reverse_of')}
        incoming, outgoing = {}, {}
        for key, edge in original.items():
            if edge['from'] in outgoing or edge['to'] in incoming:
                raise PatrolError('INVALID_DATA', '这些道路存在分支，请分别保存为巡检路径，或使用高级路网编排')
            outgoing[edge['from']] = key
            incoming[edge['to']] = key
        starts = [n for n in outgoing if n not in incoming]
        if not original:
            return []
        if len(starts) > 1:
            raise PatrolError('INVALID_DATA', '道路没有连续连接，请分别保存路径或补录连接')
        node = starts[0] if starts else next(iter(original.values()))['from']
        ids = []
        while node in outgoing and outgoing[node] not in ids:
            key = outgoing[node]
            ids.append(key)
            node = original[key]['to']
        if len(ids) != len(original):
            raise PatrolError('INVALID_DATA', '道路没有组成一条连续路径，请检查连接')
    result = []
    for key in ids:
        edge = edges.get(key)
        if not edge or not edge.get('enabled', True):
            raise PatrolError('ROAD_UNREACHABLE', '巡检路径中的道路不存在或已停用')
        if result and result[-1]['to'] != edge['from']:
            raise PatrolError('INVALID_DATA', '巡检路径分段没有连续连接')
        result.append(edge)
    return result


def path_nodes(document):
    edges = path_edges(document)
    return [edges[0]['from']] + [e['to'] for e in edges] if edges else []


def prepare_path(document, name, make_reverse=False):
    """Return new point rows; existing bindings/point identities are stable."""
    meta = document.setdefault('inspection_path', {'edge_ids': [e['id'] for e in path_edges(document)]})
    meta.setdefault('route_id', new_id('route'))
    points = meta.setdefault('point_ids', {})
    nodes = path_nodes(document)
    meta['closed'] = bool(nodes and nodes[0] == nodes[-1])
    created = []
    for index, key in enumerate(dict.fromkeys(nodes)):
        if key not in points:
            points[key] = new_id('point')
            created.append(dict(pose(document['nodes'][key]), point_id=points[key],
                                name=f'{name[:95]} · 点{index+1}'))
        document.setdefault('bindings', {})[points[key]] = [key]
        document['nodes'][key]['name'] = f'{name[:95]} · 点{index+1}'
    if make_reverse and not meta['closed']:
        reverse_ids = []
        for source in reversed(path_edges(document)):
            key = 'return_' + source['id']
            blocks = []
            for block in reversed(source['blocks']):
                n = len(block['samples'])
                blocks.append(dict(direction=-block['direction'],
                                   samples=list(reversed(copy.deepcopy(block['samples']))),
                                   barriers=sorted(n-1-i for i in block.get('barriers', []))))
            document['edges'][key] = dict(id=key, name=source['name']+'（原路返程）',
                                         **{'from': source['to'], 'to': source['from']},
                                         blocks=blocks, raw_samples=[], reverse_of=source['id'],
                                         speed=min(.10, source['speed']), enabled=True)
            reverse_ids.append(key)
        meta['reverse_edge_ids'] = reverse_ids
    return created


def route_point_ids(document):
    meta = document['inspection_path']
    nodes = path_nodes(document)
    if meta['closed']:
        nodes = nodes[1:]
    return [meta['point_ids'][n] for n in nodes]


def is_path_route(document, route):
    meta = document.get('inspection_path', {})
    return (meta.get('route_id') == route.route_id and
            [s.point_id for s in route.steps] == route_point_ids(document))


def choose_entry(document, current, compute, check_collision):
    """Try both ends, without ever skipping part of the recorded inspection."""
    nodes = path_nodes(document)
    candidates = [0] if document['inspection_path']['closed'] else [0, len(nodes)-1]
    ready = [i for i in candidates if aligned(current, document['nodes'][nodes[i]])]
    if ready:
        return min(ready, key=lambda i: distance(current, document['nodes'][nodes[i]])), []
    feasible, failures = [], []
    radius = float(document['vehicle']['minimum_turning_radius'])
    for index in sorted(candidates, key=lambda i: distance(current, document['nodes'][nodes[i]])):
        target = document['nodes'][nodes[index]]
        try:
            blocks = connector_blocks(compute(current, target), current, target, radius)
            check_collision(blocks)
            # Compare actual manoeuvre lengths, including reverse travel.
            cost = sum(length(b['samples']) for b in blocks)
            feasible.append((cost, index, blocks))
        except PatrolError as error:
            failures.append(f"{target.get('name', '端点')}：{error.message}")
    if not feasible:
        raise PatrolError('ENTRY_UNREACHABLE', '无法自动接入巡检路径；请检查定位、障碍和转弯空间。'+'；'.join(failures))
    _, index, blocks = min(feasible, key=lambda item: item[0])
    return index, blocks


def plan_path(document, steps, current, loops, entry=None):
    """Two bounded pass templates allow arbitrary repeat counts without expansion."""
    planner = NetworkPlanner(document)
    meta = document['inspection_path']
    edges, nodes = path_edges(document), path_nodes(document)
    if not edges:
        raise PatrolError('INVALID_DATA', '巡检路径没有有效行驶记录')
    candidates = [0] if meta['closed'] else [0, len(nodes)-1]
    if entry is None:
        ready = [i for i in candidates if aligned(current, document['nodes'][nodes[i]])]
        if not ready:
            raise PatrolError('OFF_ROAD', '需要自动接入规划，请在导航就绪后重新开始')
        start, entry_blocks = min(ready, key=lambda i: distance(current, document['nodes'][nodes[i]])), []
    else:
        start, entry_blocks = entry
        if start not in candidates or (not entry_blocks and not aligned(current, document['nodes'][nodes[start]])):
            raise PatrolError('INVALID_DATA', '自动接入端点无效')
    passes, indices = [], []
    if meta['closed']:
        passes = [[planner.leg([edge], edge['to'], step.point_id) for edge, step in zip(edges, steps)]]
        indices = [list(range(len(steps)))]
    else:
        reverse = [document['edges'].get(key) for key in meta.get('reverse_edge_ids', [])]
        if len(reverse) != len(edges) or any(not e or not e.get('enabled', True) for e in reverse):
            raise PatrolError('ROAD_UNREACHABLE', '原路返程记录缺失或停用，请重新保存巡检路径')
        for source, back in zip(reversed(edges), reverse):
            if (back.get('reverse_of') != source['id'] or back['from'] != source['to'] or
                    back['to'] != source['from'] or len(back['blocks']) != len(source['blocks']) or
                    any(b['direction'] != -a['direction'] or b['samples'] != list(reversed(a['samples']))
                        for a,b in zip(reversed(source['blocks']),back['blocks']))):
                raise PatrolError('INVALID_DATA', '原路返程轨迹与录制路径不一致，请重新保存路径')
        forward_indices = list(range(len(nodes)))
        backward_indices = list(reversed(forward_indices))
        for order, paths in ((forward_indices, edges), (backward_indices, reverse)):
            legs = [planner.leg([], nodes[order[0]], steps[order[0]].point_id)]
            legs.extend(planner.leg([edge], nodes[index], steps[index].point_id)
                        for edge, index in zip(paths, order[1:]))
            passes.append(legs)
            indices.append(order)
        if start:
            passes.reverse()
            indices.reverse()
    if sum(len(b['samples']) for legs in passes for leg in legs for b in leg['blocks']) > 200000:
        raise PatrolError('INVALID_DATA', '任务路径过长，请拆分巡检路径')
    initial = copy.deepcopy(passes[0])
    if entry_blocks:
        # Do not merge this seam: stop and confirm heading before entering
        # the immutable recorded road. The connector runs only on pass 1.
        initial[0]['blocks'] = copy.deepcopy(entry_blocks)+initial[0]['blocks']
    if sum(len(b['samples']) for leg in initial for b in leg['blocks']) > 200000:
        raise PatrolError('INVALID_DATA', '接入及巡检轨迹过长，请将车辆移近路径或拆分任务')
    return dict(schema_version=1, legs=initial, initial_legs=initial, closure=None, pass_legs=passes,
                pass_step_indices=indices, traversal='CLOSED' if meta['closed'] else 'SHUTTLE',
                loop_count=loops, initial_pose=pose(current), network_sha256=digest(document),
                point_order=[steps[i].point_id for i in indices[0]],
                entry=dict(required=bool(entry_blocks), target_node=nodes[start],
                           target_name=document['nodes'][nodes[start]].get('name', '端点'),
                           length=sum(length(b['samples']) for b in entry_blocks),
                           direction_changes=max(0, len(entry_blocks)-1)),
                allowed_polygon=document.get('allowed_polygon', []))


def loop_legs(plan, loop):
    if loop == 1 and plan.get('initial_legs'):
        return plan['initial_legs']
    passes = plan.get('pass_legs')
    return passes[(loop-1) % len(passes)] if passes else plan['legs']


def plan_leg(plan, loop, index):
    if plan.get('pass_legs'):
        return loop_legs(plan, loop)[index]
    return plan['closure'] if loop > 1 and index == 0 else plan['legs'][index]
