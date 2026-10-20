"""Stationary, Ackermann-feasible entry planning. Never sends motion goals."""
from __future__ import annotations

import math
import time
import uuid

from ..models import PatrolError
from .geometry import angle, distance, length, pose, validate_blocks


def aligned(current, target):
    return distance(current, target) <= .08 and abs(angle(current['yaw']-target['yaw'])) <= .12


def connector_blocks(samples, start, target, radius):
    """Keep the planner's body headings and every forward/reverse cusp.

    Densification subdivides the returned curve only. It never draws a free
    connection to an endpoint or interpolates over a missing motion segment.
    """
    if not 2 <= len(samples) <= 20000:
        raise PatrolError('ENTRY_UNREACHABLE', '接入规划没有返回有效轨迹')
    points = [pose(p) for p in samples]
    if distance(points[0], start) > .04 or abs(angle(points[0]['yaw']-start['yaw'])) > .08:
        raise PatrolError('ENTRY_UNREACHABLE', '接入轨迹没有与当前位姿对齐')
    if distance(points[-1], target) > .04 or abs(angle(points[-1]['yaw']-target['yaw'])) > .08:
        raise PatrolError('ENTRY_UNREACHABLE', '接入轨迹没有到达端点的位置和车头方向')
    blocks = []
    for a, b in zip(points, points[1:]):
        ds = distance(a, b)
        turn = angle(b['yaw']-a['yaw'])
        if ds < 1e-5:
            if abs(turn) > .01:
                raise PatrolError('ENTRY_UNREACHABLE', '接入轨迹包含阿克曼底盘不能执行的原地旋转')
            continue
        if ds > .6 or abs(turn)/ds > 1/radius+.15:
            raise PatrolError('ENTRY_UNREACHABLE', '接入轨迹采样间断或转弯半径不足')
        yaw = a['yaw']+.5*turn
        along = (b['x']-a['x'])*math.cos(yaw)+(b['y']-a['y'])*math.sin(yaw)
        if abs(along) < .90*ds:
            raise PatrolError('ENTRY_UNREACHABLE', '接入轨迹位移与车头方向不一致')
        gear = 1 if along > 0 else -1
        if not blocks or blocks[-1]['direction'] != gear:
            blocks.append(dict(direction=gear, samples=[a], barriers=[], edges=[],
                               speed=.20 if gear > 0 else .12, entry=True))
        block = blocks[-1]
        n = max(1, math.ceil(ds/.04))
        for i in range(1, n+1):
            t = i/n
            block['samples'].append(dict(x=a['x']+t*(b['x']-a['x']),
                                         y=a['y']+t*(b['y']-a['y']), yaw=angle(a['yaw']+t*turn)))
    if not blocks or len(blocks) > 100 or sum(length(b['samples']) for b in blocks) > 100:
        raise PatrolError('ENTRY_UNREACHABLE', '接入轨迹为空或过长，请将车辆移近巡检路径')
    validate_blocks(blocks, radius)
    blocks[-1]['entry_target'] = pose(target)
    return blocks


class EntryPlanner:
    """Own executor prevents a planning service from starving patrol heartbeats."""
    def __init__(self, parent):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from nav2_msgs.action import ComputePathToPose
        self.action_type = ComputePathToPose
        self.node = rclpy.create_node('recorded_entry_'+uuid.uuid4().hex[:10],
                                      context=parent.context, use_global_arguments=False,
                                      parameter_overrides=[Parameter('use_sim_time', value=parent.get_parameter('use_sim_time').value)],
                                      namespace=parent.get_namespace())
        self.executor = SingleThreadedExecutor(context=parent.context)
        self.executor.add_node(self.node)
        self.client = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')

    def close(self):
        self.client.destroy()
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()

    def _wait(self, future, seconds):
        deadline = time.monotonic()+seconds
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.05)
        if not future.done():
            raise PatrolError('ENTRY_UNREACHABLE', '自动接入规划超时，请检查规划器或将车辆移近路径')
        return future.result()

    def compute(self, start, target):
        from geometry_msgs.msg import PoseStamped
        from ..nav2_client import yaw_to_quaternion
        if not self.client.wait_for_server(timeout_sec=1.):
            raise PatrolError('NAV2_UNAVAILABLE', '自动接入规划器未就绪，请启用固定路线并重新启动导航')
        def stamped(value):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.node.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = float(value['x']), float(value['y'])
            p.pose.orientation = yaw_to_quaternion(value['yaw'])
            return p
        goal = self.action_type.Goal()
        goal.start, goal.goal, goal.use_start = stamped(start), stamped(target), True
        goal.planner_id = 'RecordedEntry'
        accepted = self.client.send_goal_async(goal)
        # A late acceptance is computation only, but still cancel it on timeout.
        try:
            handle = self._wait(accepted, 2.)
        except PatrolError:
            accepted.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result() and f.result().accepted else None)
            raise
        if not handle or not handle.accepted:
            raise PatrolError('ENTRY_UNREACHABLE', '自动接入规划请求被拒绝')
        try:
            result = self._wait(handle.get_result_async(), 4.5)
        except PatrolError:
            handle.cancel_goal_async()
            raise
        if result.status != 4 or result.result.path.header.frame_id != 'map':
            raise PatrolError('ENTRY_UNREACHABLE', '当前端点没有可行的接入轨迹（请检查障碍物和转弯空间）')
        points = []
        for p in result.result.path.poses:
            q = p.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            points.append(dict(x=p.pose.position.x, y=p.pose.position.y, yaw=yaw))
        return points
