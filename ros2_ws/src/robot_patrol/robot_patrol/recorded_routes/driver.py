"""Fixed-road child navigation using an immutable, per-attempt BT ticket."""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID
from nav2_msgs.action import FollowPath

from ..models import PatrolError
from ..nav2_client import Nav2Driver, yaw_to_quaternion
from .geometry import distance, angle, pose, project, length
from .inspection_path import plan_leg


def file_signature(mapping):
    result = []
    for name in ('navigation_yaml', 'image_path', 'localization_file'):
        path = getattr(mapping, name)
        if path:
            stat = Path(path).stat()
            result.append([path, stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns])
    return result


class RecordedDriver(Nav2Driver):
    def __init__(self, *args, roads, plan, run_id, **kwargs):
        super().__init__(*args, **kwargs)
        self.roads, self.plan, self.run_id = roads, copy.deepcopy(plan), run_id
        self._ticket = ''
        self._attempts = {}
        self._nav_key = None
        self._last_completed = None
        self._pending_follow_unknown = False
        self._follow_active = set()
        self._follow_received = 0.0
        self._follow_sub = self.node.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                                                         self._follow_status, 10,
                                                         callback_group=self.node._callback_group)
        self._follow_cancel = self.node.create_client(CancelGoal, '/follow_path/_action/cancel_goal',
                                                        callback_group=self.node._callback_group)
        self._follow_result = self.node.create_client(FollowPath.Impl.GetResultService,'/follow_path/_action/get_result',
                                                       callback_group=self.node._callback_group)
        runtime = Path(os.environ.get('XDG_RUNTIME_DIR', tempfile.gettempdir())) / f'robot_recorded_routes_{os.getuid()}'
        runtime.mkdir(mode=0o700, parents=True, exist_ok=True)
        if runtime.is_symlink() or runtime.stat().st_uid != os.getuid():
            raise PatrolError('STORE_ERROR', '固定路线运行目录不安全')
        os.chmod(runtime, 0o700)
        self.runtime = runtime
        self._xml_files = []

    def _follow_status(self, message):
        self._follow_received = time.monotonic()
        self._follow_active = {bytes(s.goal_info.goal_id.uuid).hex() for s in message.status_list if s.status in (1, 2, 3)}

    def _key(self, step):
        engine = self.node._active_engine
        if engine:
            status = engine.status
            return int(status['loop_index']), int(status['step_index'])-1
        return 1, 0

    def prepare_goal(self, step, goal, flags):
        loop, index = self._key(step)
        key = (loop, index)
        leg = copy.deepcopy(plan_leg(self.plan, loop, index))
        previous = self._attempts.get(key)
        if previous and previous.get('completed'):
            raise PatrolError('OFF_ROAD', '固定路线到点后发生偏移，禁止自动自由重定位，请重新预览任务')
        current = self.roads.current()
        if previous:
            leg = copy.deepcopy(previous.get('leg', leg))
            leg['resume'] = previous.get('progress', {})
        elif leg['blocks']:
            leg = self._continue_recorded_tail(leg, current)
            start = leg['blocks'][0]['samples'][0]
            if distance(current, start) > 0.15 or abs(angle(current['yaw']-start['yaw'])) > 0.30:
                if leg['blocks'][0].get('entry'):
                    raise PatrolError('OFF_ROAD', '车辆已离开自动接入的规划起始位姿，请停车后重新开始')
                raise PatrolError('OFF_ROAD', '车辆未处于下一录制路段的接入位姿，请补录连接或重新预览')
        nav_uuid = uuid.uuid4()
        ticket_id = 'leg_' + uuid.uuid4().hex
        mapping = self.node._active_map()
        value = dict(leg, ticket_id=ticket_id, nav_uuid=nav_uuid.hex, run_id=self.run_id,
                     map_id=mapping.map_id, map_revision=mapping.revision,
                     file_signature=file_signature(mapping), map_message_id=id(self.node._latest_map),
                     minimum_turning_radius=self.roads.radius,
                     vehicle=self.roads.vehicle_stamp(),
                     footprint=json.loads(self.node.get_parameter('recorded_footprint').value),
                     allowed_polygon=self.plan.get('allowed_polygon', []),
                     max_offset=float(self.node.get_parameter('recorded_detour_max_offset').value),
                     max_skip=float(self.node.get_parameter('recorded_detour_max_skip').value),
                     max_length=float(self.node.get_parameter('recorded_detour_max_length').value),
                     wait_timeout=float(self.node.get_parameter('recorded_obstacle_wait_timeout').value),
                     resume=leg.get('resume', {}))
        with self.roads.lock:
            self.roads.tickets[ticket_id] = value
        root = ET.Element('root', main_tree_to_execute='RecordedRoute')
        tree = ET.SubElement(root, 'BehaviorTree', ID='RecordedRoute')
        ET.SubElement(tree, 'FollowRecordedLeg', ticket_id=ticket_id)
        target = self.runtime / f'{ticket_id}.xml'
        temporary = target.with_suffix('.tmp')
        with temporary.open('wb') as output:
            ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
        os.chmod(temporary, 0o600)
        temporary.replace(target)
        self._xml_files.append(target)
        goal.behavior_tree = str(target)
        # The road's body heading is the arrival state; business point yaw is
        # not an instruction to turn on the spot.
        goal.pose.pose.position.x = leg['target']['x']
        goal.pose.pose.position.y = leg['target']['y']
        goal.pose.pose.orientation = yaw_to_quaternion(leg['target']['yaw'])
        self._ticket, self._nav_key = ticket_id, key
        return {'goal_uuid': UUID(uuid=list(nav_uuid.bytes))}

    def navigate(self, step, flags, status_cb):
        # Empty leg means a repeated point / already at the first node. It is
        # a real business step, but must not send a zero-length Nav2 goal.
        loop, index = self._key(step)
        leg = plan_leg(self.plan, loop, index)
        if not leg['blocks']:
            current = self.roads.current()
            return 'SUCCEEDED' if distance(current, leg['target']) <= 0.20 and abs(angle(current['yaw']-leg['target']['yaw'])) < 0.30 else 'OFF_ROAD'
        try:
            result = super().navigate(step, flags, status_cb)
        except PatrolError as error:
            self.last_error = error.message
            return error.code
        finally:
            # Parent action results and progress use different DDS writers.
            # Allow the final halt report to arrive before retiring a ticket.
            with self.roads.lock:
                pending=self.roads.tickets.get(self._ticket,{}).get('progress',{}).get('follow_dispatch_pending',False)
            if pending:
                time.sleep(.20)
            with self.roads.lock:
                value = self.roads.tickets.pop(self._ticket, None)
                if value:
                    self._attempts[self._nav_key] = {'progress': value.get('progress', {}), 'completed': False, 'leg': value}
                    if value.get('progress',{}).get('follow_dispatch_pending',False):
                        self._pending_follow_unknown = True
                        self.last_error='内部 FollowPath 接受未确认；请停止并重启导航会话，不能仅凭空目标列表放行'
        stopped = self._confirm_internal_stop()
        if not stopped:
            self.stop_unconfirmed = True
            self._notify_goal_changed()
            return 'CANCEL_UNCONFIRMED'
        for path in self._xml_files:
            path.unlink(missing_ok=True)
        self._xml_files.clear()
        if result == 'SUCCEEDED':
            progress = self._attempts.get(self._nav_key, {}).get('progress', {})
            if progress.get('state') != 'COMPLETED':
                self.last_error = '固定路线未确认完整进度，拒绝将普通导航成功当作完成'
                return 'ROAD_NOT_COMPLETED'
            self._attempts[self._nav_key]['completed'] = True
            self._last_completed = self._attempts[self._nav_key]
        while len(self._attempts)>2:
            self._attempts.pop(next(iter(self._attempts)))
        return result

    def _continue_recorded_tail(self, leg, current):
        """A 20cm business stop must not erase the remaining road before a
        turn/reversal. Carry that exact tail into the next leg, never draw a
        free connector from the actual stop pose to the next target.
        """
        previous = self._last_completed
        if not previous or not previous['leg']['blocks']:
            return leg
        old = previous['leg']['blocks'][-1]
        points = old['samples']
        index = min(int(previous['progress'].get('reference_index', 0)), len(points)-2)
        error, index, fraction, q = project(points,current,max(0,index-3))
        if error>0.12 or abs(angle(current['yaw']-q['yaw']))>0.30:
            raise PatrolError('OFF_ROAD', '到点停车后已离开已验证道路，禁止生成自由连接')
        remaining = [q]+points[index+1:]
        if length(remaining)<0.025:
            return leg
        tail=copy.deepcopy(old)
        tail['samples']=remaining
        tail['barriers']=[max(0,int(i)-index) for i in old.get('barriers',[]) if int(i)>index]
        first=leg['blocks'][0]
        if distance(tail['samples'][-1],first['samples'][0])>0.04 or abs(angle(tail['samples'][-1]['yaw']-first['samples'][0]['yaw']))>0.12:
            raise PatrolError('KINEMATIC_INVALID', '上一停靠状态无法接入下一道路，需补录连接')
        if tail['direction']==first['direction']:
            offset=len(remaining)-1
            first['samples']=remaining+first['samples'][1:]
            first['barriers']=tail['barriers']+[offset]+[offset+int(i) for i in first.get('barriers',[])]
            first['edges']=tail.get('edges',[])+first.get('edges',[])
            first['speed']=min(tail.get('speed',.2),first.get('speed',.2))
        else:
            leg['blocks'].insert(0,tail)
        return leg

    def _confirm_internal_stop(self):
        if self._pending_follow_unknown:
            return False
        if not self._follow_cancel.wait_for_service(timeout_sec=0.2):
            return False
        future = self._follow_cancel.call_async(CancelGoal.Request())
        progress=self._attempts.get(self._nav_key,{}).get('progress',{})
        child_id=progress.get('follow_goal_id','')
        result_future=None
        if child_id:
            if not self._follow_result.wait_for_service(timeout_sec=.2):
                return False
            request=FollowPath.Impl.GetResultService.Request(goal_id=UUID(uuid=list(bytes.fromhex(child_id))))
            result_future=self._follow_result.call_async(request)
        deadline = time.monotonic()+self.cancel_confirm_timeout
        while time.monotonic() < deadline:
            if result_future is not None:
                if result_future.done():
                    try:return result_future.result().status in (4,5,6)
                    except Exception:return False
                time.sleep(.03)
                continue
            if future.done():
                try:
                    response = future.result()
                    if not response.goals_canceling and not self._follow_active:
                        return True
                except Exception:
                    return False
            time.sleep(0.03)
        return False

    def stop_safe(self):
        if self._pending_follow_unknown:
            return False
        if self.stop_unconfirmed and self._current_handle is None and self._follow_received > 0 and \
                time.monotonic()-self._follow_received < 1.5 and not self._follow_active:
            self.stop_unconfirmed = False
        return super().stop_safe()

    def reconfirm(self, step, flags, status_cb):
        result = super().reconfirm(step, flags, status_cb)
        if result == 'OFF_POINT':
            return 'OFF_POINT'
        return result

    def close(self):
        with self.roads.lock:
            self.roads.tickets.pop(self._ticket, None)
        for path in self._xml_files:
            path.unlink(missing_ok=True)
        self.node.destroy_subscription(self._follow_sub)
        self.node.destroy_client(self._follow_cancel)
        self.node.destroy_client(self._follow_result)
