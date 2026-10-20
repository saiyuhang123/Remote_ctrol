"""Local road-library ROS adapter, owned by the existing patrol node."""
from __future__ import annotations

import copy
import json
import math
import threading
import time
from types import SimpleNamespace

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as RosPoint
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from patrol_interfaces.srv import RoadCommand
from patrol_interfaces.msg import OperatorHeartbeat

from ..models import PatrolError, new_id, validate_name
from .geometry import pose, distance, angle, length
from .graph import NetworkPlanner, validate_network, digest
from .recorder import Recorder
from .inspection_path import path_edges, path_nodes, prepare_path, is_path_route, plan_path, choose_entry
from .entry import EntryPlanner


class RoadComponent:
    def __init__(self, node):
        self.node, self.store = node, node._store
        self.lock = threading.RLock()
        self.planning_lock = threading.Lock()
        self.recorder = None
        self.draft_id = ''
        self.owner = ''
        self.anchor = None
        self.path_mode = False
        self.stationary = False
        self.stop_since = 0.0
        self.stop_pose = None
        self.stop_map = None
        self.moving_ticks = 0
        self.network_id = ''
        self.plans = {}
        self.tickets = {}
        self.progress = {}
        self.command_cache = {}
        self.last_draft_save = 0.0
        self.last_record = ''
        self.operator_seen = {}
        self.status_pub = node.create_publisher(String, '/patrol/recording/status', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.markers = node.create_publisher(MarkerArray, '/patrol/road_network_preview', qos)
        self.plan_markers = node.create_publisher(MarkerArray, '/patrol/recorded_plan_preview', qos)
        self.detour_markers = node.create_publisher(MarkerArray, '/patrol/recorded_detour_preview', qos)
        self.last_detour_preview = 0.0
        self.service = node.create_service(RoadCommand, '/patrol/road_command', self.command,
                                          callback_group=node._callback_group)
        self.timer = node.create_timer(0.1, self.tick, callback_group=node._callback_group)
        self.progress_sub = node.create_subscription(String, '/patrol/recorded_progress', self.on_progress,
                                                      10, callback_group=node._callback_group)
        self.operator_sub=node.create_subscription(OperatorHeartbeat,'/patrol/operator_heartbeat',self.on_operator,
                                                    10,callback_group=node._callback_group)

    def on_operator(self,message):
        with self.lock:
            self.operator_seen[message.session_id]=(time.monotonic(),bool(message.enabled))
            # Passive draft ownership, independent of the autonomous guard.
            for key,value in list(self.operator_seen.items()):
                if time.monotonic()-value[0]>10:
                    self.operator_seen.pop(key,None)

    @property
    def enabled(self):
        return bool(self.node.get_parameter('enable_recorded_routes').value)

    def busy_recording(self):
        with self.lock:
            return self.recorder is not None

    def _map(self):
        mapping = self.node._active_map()
        self.node._verify_map(mapping)
        self.node._verify_runtime_map(mapping)
        return mapping

    @staticmethod
    def map_stamp(mapping):
        return {'resolution': mapping.resolution, 'origin_x': mapping.origin_x,
                'origin_y': mapping.origin_y, 'origin_yaw': mapping.origin_yaw,
                'width': mapping.width, 'height': mapping.height,
                'localization_sha256': mapping.localization_sha256,
                'image_sha256': mapping.image_sha256,
                'navigation_yaml_sha256': mapping.navigation_yaml_sha256}

    def _network(self, network_id):
        entry = self.store.get_network(network_id)
        mapping = self.node._active_map()
        if entry['map_id'] != mapping.map_id or entry['map_revision'] != mapping.revision:
            raise PatrolError('MAP_MISMATCH', '路网地图或版本不是当前活跃地图，请重新验证或补录')
        return entry

    def vehicle_stamp(self):
        return {'minimum_turning_radius':self.radius,
                'footprint':json.loads(self.node.get_parameter('recorded_footprint').value)}

    def _require_vehicle(self,document):
        if document.get('vehicle') != self.vehicle_stamp():
            raise PatrolError('STALE_DATA','底盘转弯半径或车身尺寸已变化，请重新校验发布路网并更新任务版本')

    @staticmethod
    def anchored_pose(anchor, x, y, yaw):
        c, s = math.cos(anchor['yaw']), math.sin(anchor['yaw'])
        return {'x': anchor['x'] + c*x - s*y,
                'y': anchor['y'] + s*x + c*y,
                'yaw': angle(anchor['yaw'] + yaw)}

    def _anchor_pose(self):
        """Continuous pose used inside a segment.

        Prefer LIO base odometry even in wheel_imu mode: /odom there is the
        EKF output and its scale/steering error accumulated 25 cm over a 7.5
        m segment, while the ICP correction refers to the LIO pose. Fall back
        to odom->base_link only when /odom_lio is unavailable or stale.
        """
        latest = getattr(self.node, 'latest_lio_odom', None)
        if latest is not None:
            message, received = latest()
            if message is not None and time.monotonic()-received <= 1.5:
                p, q = message.pose.pose.position, message.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                stamp = message.header.stamp
                return p.x, p.y, angle(yaw), int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
        return self.node._current_odom_pose()

    def _capture_anchor(self, target):
        """Freeze the continuous frame at a segment start.

        ICP corrects map->odom about once per second. Recomposing every pose
        with the live transform turns a 0.7 deg yaw correction at 8 m into a
        10 cm step inside the recorded road. Anchoring the segment to its
        start node keeps the shape continuous; the next segment re-anchors to
        the saved end node, so the network still joins exactly.
        """
        if not hasattr(self.node, '_current_odom_pose') and \
                not hasattr(self.node, 'latest_lio_odom'):
            # Minimal test doubles (and any legacy node) keep map poses.
            self.anchor = None
            return None
        ox, oy, oyaw, _ = self._anchor_pose()
        yaw = angle(target['yaw'] - oyaw)
        c, s = math.cos(yaw), math.sin(yaw)
        self.anchor = {'x': target['x'] - (c*ox - s*oy),
                       'y': target['y'] - (s*ox + c*oy),
                       'yaw': yaw}
        return self.anchor

    def _close_anchor_gap(self, r):
        """Pull a segment endpoint back to the live map with a smooth ramp.

        The segment shape follows continuous odometry; the ICP map correction
        is deliberately not applied as an in-segment step. At the split the
        live map pose and the anchored end can therefore differ by the drift
        accumulated since the segment start. Distribute that difference over
        the segment (smoothstep, so both ends keep their curvature) instead
        of leaving a map offset or reintroducing a kink. A difference too
        large to be drift must be re-localized, not silently bent.
        """
        if self.anchor is None or not r.samples or len(r.samples) < 2:
            return
        try:
            mx, my, myaw, _ = self.node._current_pose()
        except PatrolError:
            return
        end = r.samples[-1]
        dx, dy = mx-end['x'], my-end['y']
        dyaw = angle(myaw-end['yaw'])
        gap = math.hypot(dx, dy)
        if gap > 0.15 or abs(dyaw) > 0.08:
            raise PatrolError(
                'INVALID_DATA',
                f'分段末端定位偏差 {gap*100:.0f}cm/{abs(math.degrees(dyaw)):.1f}°，请重新定位后再分段')
        n = len(r.samples)
        for i, p in enumerate(r.samples):
            f = i/(n-1)
            f = f*f*(3-2*f)
            p['x'] += f*dx
            p['y'] += f*dy
            p['yaw'] = angle(p['yaw'] + f*dyaw)

    def current(self):
        if self.anchor is None:
            x, y, yaw, stamp = self.node._current_pose()
        else:
            ox, oy, oyaw, stamp = self._anchor_pose()
            value = self.anchored_pose(self.anchor, ox, oy, oyaw)
            x, y, yaw = value['x'], value['y'], value['yaw']
        odom, received = self.node.latest_odom()
        if odom is None or time.monotonic()-received > 1.5:
            raise PatrolError('STALE_DATA', '没有新鲜的里程计反馈')
        return dict(x=x, y=y, yaw=yaw, t=stamp/1e9, v=float(odom.twist.twist.linear.x))

    def _idle(self):
        with self.node._run_lock:
            if self.node._run_reserved or self.node._active_engine or self.node._return_active:
                raise PatrolError('BUSY', '请先结束自主任务')
        if self.node._hazard_driver and not self.node._hazard_driver.stop_safe():
            raise PatrolError('CANCEL_UNCONFIRMED', '旧导航尚未确认停止')

    def command(self, request, response):
        try:
            if len(request.payload_json.encode()) > 32*1024*1024:
                raise PatrolError('INVALID_DATA', '请求内容超过 32MB')
            payload = json.loads(request.payload_json or '{}')
            if not isinstance(payload, dict):
                raise PatrolError('INVALID_DATA', '请求必须为对象')
            # Planning can take several seconds. Keep heartbeat, recording
            # status and stop/ticket services independent of that computation.
            if request.operation.upper() == 'PLAN':
                data = self.dispatch('PLAN', payload)
                response.success, response.message = True, 'ok'
                response.data_json = json.dumps(data, ensure_ascii=False, allow_nan=False)
                return response
            with self.lock:
                op = request.operation.upper()
                key = (op, str(payload.get('owner_session_id','')), str(payload.get('command_id','')))
                cacheable = bool(key[2]) and op not in ('STATUS','LIST','PLAN','LOAD_LEG','CHECK_LEG','EVENTS','SHOW_NETWORK','EXPORT')
                signature = digest(payload)
                if cacheable and key in self.command_cache:
                    old_signature, data = self.command_cache[key]
                    if signature != old_signature:
                        raise PatrolError('INVALID_DATA','同一命令 ID 的内容发生变化')
                else:
                    data = self.dispatch(op, payload)
                    if cacheable:
                        self.command_cache[key] = (signature, copy.deepcopy(data))
                        while len(self.command_cache)>20:
                            self.command_cache.pop(next(iter(self.command_cache)))
            response.success, response.message = True, 'ok'
            response.data_json = json.dumps(data, ensure_ascii=False, allow_nan=False)
        except PatrolError as error:
            response.success, response.error_code, response.message = False, error.code, error.message
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            response.success, response.error_code, response.message = False, 'INVALID_DATA', str(error)
        except Exception as error:
            self.node.get_logger().error(f'Road service: {error}')
            response.success, response.error_code, response.message = False, 'STORE_ERROR', str(error)
        return response

    def dispatch(self, op, p):
        if op == 'STATUS':
            r = self.recorder
            live = r.samples[::max(1,math.ceil(len(r.samples)/1000))] if r else []
            if r and r.samples and live[-1] is not r.samples[-1]:
                live=live+[r.samples[-1]]
            return {'enabled': self.enabled, 'state': r.state if r else 'IDLE',
                    'draft_id': self.draft_id, 'samples': len(r.samples) if r else 0,
                    'length': length(r.samples) if r else 0, 'error': r.error if r else '',
                    'network_id': self.network_id, 'last_record': self.last_record,
                    'progress': self.progress,'live_samples':[pose(p) for p in live]}
        if op == 'LIST':
            mapping = self.node._active_map()
            return {'networks': self.store.list_networks(mapping.map_id),
                    'drafts': self.store.list_road_drafts(mapping.map_id)}
        if op == 'EVENTS':
            return {'events': self.store.road_events(str(p['run_id']))}
        if op == 'SHOW_NETWORK':
            net = self._network(p['network_id'])
            self.publish_preview(net['document'])
            return {}
        if op in ('LOAD_LEG', 'CHECK_LEG'):
            return self.check_ticket(p['ticket_id'], include_path=op == 'LOAD_LEG')
        if not self.enabled:
            raise PatrolError('DISABLED', '当前导航会话未启用固定路线，请在启动时启用固定路线模块')
        if op == 'PLAN':
            return self.preview(p['route_id'], int(p.get('loop_count', 1)))
        self._idle()
        mapping = self._map()
        if op == 'START_PATH':
            if self.recorder:
                raise PatrolError('BUSY', '请先结束当前录制')
            self.node._require_execution_safety()
            owner = str(p.get('owner_session_id', '')).strip()
            seen, authorized = self.operator_seen.get(owner, (0, False))
            if not owner or not authorized or time.monotonic()-seen > 3:
                raise PatrolError('LEASE_LOST', '请启用操作，并确认操作端心跳正常')
            name = validate_name(p.get('name', ''), '巡检路径名称')
            current = self.current()
            doc = dict(schema_version=1, nodes={}, edges={}, bindings={}, allowed_polygon=[],
                       map_stamp=self.map_stamp(mapping), vehicle=self.vehicle_stamp(),
                       inspection_path=dict(edge_ids=[], point_ids={}, route_id=new_id('route'), closed=False))
            net = self.store.save_network(mapping.map_id, name, doc)
            self._capture_anchor(current)
            self.recorder = Recorder(mapping.map_id, mapping.revision, name=name)
            # The first sample must use the same continuous time base as the
            # rest of the segment; the pre-anchor map pose has a different TF
            # stamp and could make the second sample's dt negative.
            self.recorder.add(self.current())
            self.owner, self.network_id, self.draft_id = owner, net['network_id'], new_id('draft')
            self.path_mode = True
            self._reset_stop()
            self._save_draft()
            return self.dispatch('STATUS', {})
        if op in ('MARK_PATH_POINT', 'SAVE_PATH'):
            return self._save_path(p, complete=op == 'SAVE_PATH')
        if op == 'CREATE':
            doc = {'schema_version': 1, 'nodes': {}, 'edges': {}, 'bindings': {},
                   'map_stamp': self.map_stamp(mapping), 'allowed_polygon': [],'vehicle':self.vehicle_stamp()}
            return self.store.save_network(mapping.map_id, p['name'], doc)
        if op == 'START_RECORD':
            if self.recorder:
                raise PatrolError('BUSY', '请先保存或放弃当前录制')
            self.node._require_execution_safety()
            requested_owner=str(p.get('owner_session_id','')).strip()
            seen,authorized=self.operator_seen.get(requested_owner,(0,False))
            if not requested_owner or not authorized or time.monotonic()-seen>3:
                raise PatrolError('LEASE_LOST','请先启用操作，并确认录制操作端心跳正常')
            net = self._network(p['network_id'])
            current = self.current()
            start_node = str(p.get('start_node', ''))
            target = net['document']['nodes'][start_node] if start_node else current
            if start_node:
                self._aligned(current, target)
            self.owner = str(p.get('owner_session_id', '')).strip()
            if not self.owner:
                raise PatrolError('INVALID_DATA', '录制操作端会话不能为空')
            self._capture_anchor(target)
            self.recorder = Recorder(mapping.map_id, mapping.revision, start_node,
                                     name=str(p.get('name', '道路')))
            self.recorder.add(self.current())
            self.network_id, self.draft_id = net['network_id'], new_id('draft')
            self.path_mode = False
            self._reset_stop()
            self._save_draft()
            return self.dispatch('STATUS', {})
        if op in ('RECOVER_DRAFT', 'RECOVER_PATH'):
            if self.recorder:
                seen,enabled=self.operator_seen.get(self.owner,(0,False))
                if p.get('draft_id') != self.draft_id or self.recorder.state == 'RECORDING' or \
                        (self.owner != str(p.get('owner_session_id','')) and enabled and time.monotonic()-seen<=3):
                    raise PatrolError('BUSY', '先处理当前草稿；其他在线操作端仍持有录制时禁止恢复')
            draft = next((item for item in self.store.list_road_drafts(mapping.map_id) if item['draft_id'] == p['draft_id']), None)
            if not draft or int(draft['payload']['map_revision']) != mapping.revision:
                raise PatrolError('MAP_MISMATCH', '草稿不存在或地图版本已变化')
            old = draft['payload']
            self._network(old['network_id'])
            r = Recorder(mapping.map_id, mapping.revision, old.get('start_node', ''), name=old['name'])
            r.samples = old['samples']
            if old.get('state')=='INVALID':
                r.state, r.error = 'INVALID', old['error']
            elif op == 'RECOVER_PATH':
                r.state = 'PAUSED'
                r.pause_pose = old.get('pause_pose') or pose(r.samples[-1])
            else:
                r.finish(self.radius)
            self.recorder, self.network_id, self.draft_id = r, old['network_id'], draft['draft_id']
            self.owner = str(p.get('owner_session_id', ''))
            if not self.owner:
                self.recorder = None
                self.anchor = None
                raise PatrolError('INVALID_DATA', '恢复操作端会话不能为空')
            # Anchor to the live map pose, not to the draft endpoint: the
            # subsequent resume check must still detect a robot that was moved
            # while the backend was restarting.
            self.anchor = None
            self._capture_anchor(self.current())
            self.path_mode = op == 'RECOVER_PATH'
            self._reset_stop()
            self._save_draft()
            result = self.dispatch('STATUS', {})
            if r.state == 'REVIEW':
                result['review_blocks'] = r.finish(self.radius)
            return result
        if op in ('PAUSE_RECORD', 'RESUME_RECORD', 'FINISH_RECORD', 'SAVE_RECORD', 'DISCARD_RECORD', 'MARK_NODE'):
            r = self.recorder
            if not r or self.owner != str(p.get('owner_session_id', '')):
                raise PatrolError('NOT_OWNER', '没有本会话的活动录制')
            if op == 'PAUSE_RECORD':
                self._reset_stop()
                r.pause(self.current())
            elif op == 'RESUME_RECORD':
                self._reset_stop()
                r.resume(self.current())
            elif op == 'DISCARD_RECORD':
                r.state = 'DISCARDED'
                self._save_draft()
                self.recorder = None
                self.anchor = None
                self.path_mode = False
                self._reset_stop()
            elif op == 'FINISH_RECORD':
                r.add(self.current())
                self._close_anchor_gap(r)
                blocks = r.finish(self.radius)
                self.anchor = None
                self.path_mode = False
                self._reset_stop()
                self._save_draft()
                self._publish_paths([(r.name,b['samples'],b['direction']) for b in blocks],self.plan_markers)
                return dict(self.dispatch('STATUS', {}), review_blocks=blocks)
            else:
                result = self._save_record(p)
                if op == 'MARK_NODE':
                    end = result['edge']['to']
                    target = self._network(self.network_id)['document']['nodes'][end]
                    self._capture_anchor(target)
                    self.recorder = Recorder(mapping.map_id, mapping.revision, end,
                                             name=str(p.get('next_name', '后续道路')))
                    self.recorder.add(self.current())
                    self.draft_id = new_id('draft')
                    self.path_mode = False
                    self._reset_stop()
                    self._save_draft()
                else:
                    self.anchor = None
                    self.path_mode = False
                    self._reset_stop()
                return result
            self._save_draft()
            return self.dispatch('STATUS', {})
        if self.recorder:
            raise PatrolError('BUSY', '请先保存或放弃录制草稿再修改路网')
        if op == 'IMPORT':
            exported = p['export']
            if exported.get('schema_version')!=1 or digest(exported['document'])!=exported.get('sha256') or \
                    exported['document'].get('map_stamp')!=exported['map_stamp']:
                raise PatrolError('INVALID_DATA','导入文件版本或内容摘要不一致')
            if exported['map_stamp'] != self.map_stamp(mapping):
                raise PatrolError('MAP_MISMATCH', '导入地图摘要/坐标与当前地图不一致')
            doc = exported['document']
            self._check_bindings(doc)
            validate_network(doc, self.radius)
            return self.store.save_network(mapping.map_id, exported['name']+'（导入）', doc)
        net = self.store.get_network(p['network_id'])
        if net['map_id'] != mapping.map_id:
            raise PatrolError('MAP_MISMATCH', '路网不属于当前地图')
        if op == 'EXPORT':
            return dict(schema_version=1, name=net['name'], document=net['document'],
                        map_stamp=net['document']['map_stamp'], sha256=digest(net['document']))
        if int(p.get('expected_revision', -1)) != net['revision']:
            raise PatrolError('STALE_DATA', '路网已变化，请刷新后重试')
        if op == 'DELETE':
            self.store.delete_network(net['network_id'], net['revision'])
            self.publish_preview({})
            return {}
        doc = copy.deepcopy(net['document'])
        if op == 'BIND_POINT':
            point = self.store.get_point(p['point_id'])
            if point.map_id != mapping.map_id:
                raise PatrolError('MAP_MISMATCH', '点位地图不一致')
            ids = list(p['node_ids'])
            if not ids or any(distance(doc['nodes'][key], vars(point)) > 0.08 for key in ids):
                raise PatrolError('OFF_ROAD', '点位距离节点超过 8cm，请录制停靠支路或重新采点')
            doc['bindings'][point.point_id] = ids
        elif op == 'UNBIND_POINT':
            point_id = str(p['point_id'])
            remaining = [key for key in doc['bindings'].get(point_id,[]) if key != str(p['node_id'])]
            if remaining:
                doc['bindings'][point_id] = remaining
            else:
                doc['bindings'].pop(point_id,None)
        elif op == 'UPDATE_EDGE':
            edge = doc['edges'][p['edge_id']]
            edge['enabled'] = bool(p.get('enabled', True))
            edge['speed'] = float(p.get('speed', edge['speed']))
        elif op == 'SET_REGION':
            doc['allowed_polygon'] = p['polygon']
        elif op in ('PUBLISH', 'REVALIDATE'):
            old = dict(doc['map_stamp'])
            new = self.map_stamp(mapping)
            for key in ('image_sha256', 'navigation_yaml_sha256'):
                old.pop(key, None)
                new.pop(key, None)
            if old != new:
                raise PatrolError('MAP_MISMATCH', '定位坐标基准或地图几何已变化，需重新录制，不能直接重绑定')
            self._check_bindings(doc)
            if not doc['edges']:
                raise PatrolError('INVALID_DATA', '路网没有道路')
            validate_network(doc, self.radius)
            self._check_map_collision(doc)
            doc['map_stamp'] = self.map_stamp(mapping)
            doc['vehicle'] = self.vehicle_stamp()
        else:
            raise PatrolError('INVALID_DATA', '不支持的道路操作')
        validate_network(doc, self.radius)
        result = self.store.save_network(mapping.map_id, net['name'], doc, net['network_id'],
                                         net['revision'], published=op == 'PUBLISH')
        self.publish_preview(doc)
        return result

    @property
    def radius(self):
        return float(self.node.get_parameter('recorded_min_turning_radius').value)

    @staticmethod
    def _aligned(current, target):
        if distance(current, target) > 0.04 or abs(angle(current['yaw']-target['yaw'])) > 0.12:
            raise PatrolError('OFF_ROAD', '道路端点与节点不一致（需位置 4cm、朝向约 7° 内），请补录连接')

    def _save_path(self, p, complete):
        r = self.recorder
        if r and self.owner != str(p.get('owner_session_id', '')):
            raise PatrolError('NOT_OWNER', '没有本会话的活动录制')
        if not complete and (not r or r.state != 'RECORDING'):
            raise PatrolError('INVALID_STATE', '请先开始或继续录制，再记录点位')
        net = self._network(p.get('network_id', self.network_id))
        if r and net['network_id'] != self.network_id:
            raise PatrolError('MAP_MISMATCH', '当前草稿属于另一条巡检路径')
        if int(p.get('expected_revision', -1)) != net['revision']:
            raise PatrolError('STALE_DATA', '路径版本已变化，请刷新后重试')
        doc = copy.deepcopy(net['document'])
        original_edges = path_edges(doc)
        if 'inspection_path' not in doc:
            doc['inspection_path'] = dict(edge_ids=[e['id'] for e in original_edges], point_ids={},
                                          route_id=new_id('route'), closed=False)
        current = None
        if r:
            if r.state not in ('RECORDING', 'PAUSED', 'REVIEW'):
                raise PatrolError('INVALID_STATE', r.error or '当前录制无效，请检查定位后补录')
            if r.state == 'RECORDING':
                current = self.current()
                r.add(current)
                if r.samples and current['t'] > r.samples[-1]['t']:
                    r.samples.append(current)
                self._close_anchor_gap(r)
            stayed = bool(r.samples and all(distance(r.samples[0], q) < .04 and
                                           abs(angle(r.samples[0]['yaw']-q['yaw'])) < .10 for q in r.samples))
            if not complete and stayed:
                return dict(network=net, point_count=len(doc['inspection_path']['point_ids']),
                            unchanged=True, state='RECORDING')
            # A split saves the preceding edge immediately. Finishing while
            # still at that node closes the session, without saving noise.
            empty_tail = bool(complete and original_edges and r.samples and
                              r.start_node == original_edges[-1]['to'] and
                              stayed)
            if stayed and not original_edges:
                raise PatrolError('INVALID_DATA', '尚未录到有效道路，请先驾驶一段距离再保存')
            if not empty_tail:
                blocks = r.finish(self.radius)
                start = r.start_node
                if original_edges and start != original_edges[-1]['to']:
                    raise PatrolError('INVALID_DATA', '新分段没有接在路径末端，请从末端继续录制')
                if not start:
                    start = new_id('port')
                    doc['nodes'][start] = dict(pose(blocks[0]['samples'][0]), id=start, name='起点')
                self._aligned(blocks[0]['samples'][0], doc['nodes'][start])
                end = new_id('port')
                first = original_edges[0]['from'] if original_edges else start
                end_pose = blocks[-1]['samples'][-1]
                if (distance(end_pose, doc['nodes'][first]) <= .04 and
                        abs(angle(end_pose['yaw']-doc['nodes'][first]['yaw'])) <= .12):
                    end = first
                else:
                    doc['nodes'][end] = dict(pose(end_pose), id=end, name='终点')
                key = new_id('road')
                doc['edges'][key] = dict(id=key, name=f'{net["name"][:90]} · 段{len(original_edges)+1}',
                                        **{'from': start, 'to': end}, blocks=blocks,
                                        raw_samples=copy.deepcopy(r.samples), speed=.20, enabled=True)
                doc['inspection_path']['edge_ids'].append(key)
        if not path_edges(doc):
            raise PatrolError('INVALID_DATA', '尚未录到有效道路，请先驾驶一段距离')
        created = prepare_path(doc, net['name'], make_reverse=complete)
        if complete:
            # Same publication checks as advanced road management; only the
            # manual binding/publish steps disappear from the user workflow.
            old, new = dict(doc['map_stamp']), self.map_stamp(self.node._active_map())
            for key in ('image_sha256', 'navigation_yaml_sha256'):
                old.pop(key, None)
                new.pop(key, None)
            if old != new:
                raise PatrolError('MAP_MISMATCH', '录制地图的坐标基准已变化，请重新录制')
            doc['map_stamp'] = self.map_stamp(self.node._active_map())
            doc['vehicle'] = self.vehicle_stamp()
            validate_network(doc, self.radius)
            self._check_map_collision(doc)
        draft = None
        if r:
            payload = dict(vars(r), state='SAVED', network_id=net['network_id'])
            draft = dict(draft_id=self.draft_id, payload=payload)
        saved = self.store.save_path_bundle(net, doc, created, publish=complete, draft=draft)
        if r:
            r.state = 'SAVED'
        self.network_id = saved['network_id']
        self.recorder = None
        if not complete:
            end = path_nodes(doc)[-1]
            self._capture_anchor(doc['nodes'][end])
            self.recorder = Recorder(r.map_id, r.map_revision, start_node=end, name=r.name)
            self.recorder.add(self.current())
            self.draft_id = new_id('draft')
            self.path_mode = True
            self._reset_stop()
            self._save_draft()
        else:
            self.anchor = None
            self.path_mode = False
            self._reset_stop()
        self.publish_preview(doc)
        return dict(network=saved, route_id=doc['inspection_path']['route_id'] if complete else '',
                    point_count=len(doc['inspection_path']['point_ids']),
                    closed=doc['inspection_path']['closed'], state='IDLE' if complete else 'RECORDING')

    def _save_record(self, p):
        r = self.recorder
        if r.state == 'RECORDING':
            r.add(self.current())
            self._close_anchor_gap(r)
        blocks = r.finish(self.radius)
        net = self._network(self.network_id)
        if int(p.get('expected_revision', -1)) != net['revision']:
            raise PatrolError('STALE_DATA', '路网版本变化，草稿保留，请刷新后保存')
        doc = copy.deepcopy(net['document'])
        start, end = r.start_node, str(p.get('end_node', ''))
        for role, value, key in [('起点', blocks[0]['samples'][0], start),
                                  ('终点', blocks[-1]['samples'][-1], end)]:
            if key == '__start__':
                key = start
            if key:
                self._aligned(value, doc['nodes'][key])
            else:
                key = new_id('port')
                doc['nodes'][key] = dict(pose(value), id=key, name=r.name+' '+role)
            if role == '起点':
                start = key
            else:
                end = key
        edge = {'id': new_id('road'), 'name': r.name, 'from': start, 'to': end,
                'blocks': blocks, 'raw_samples': r.samples, 'speed': 0.2, 'enabled': True}
        doc['edges'][edge['id']] = edge
        validate_network(doc, self.radius)
        result = self.store.save_network(net['map_id'], net['name'], doc, net['network_id'], net['revision'])
        r.state = 'SAVED'
        self._save_draft()
        self.last_record = edge['id']
        self.recorder = None
        self.publish_preview(doc)
        return {'network': result, 'edge': edge}

    def _save_draft(self):
        r = self.recorder
        if r:
            self.store.save_road_draft(self.draft_id, r.map_id, dict(vars(r), network_id=self.network_id))
        self.last_draft_save = time.monotonic()

    def _check_bindings(self, doc):
        for point_id, nodes in doc.get('bindings', {}).items():
            point = self.store.get_point(point_id)
            if point.map_id != self.node._active_map().map_id or any(distance(vars(point), doc['nodes'][n]) > 0.08 for n in nodes):
                raise PatrolError('POINT_NOT_BOUND', f'巡检点 {point.name} 绑定失效，请重新绑定')

    def _check_map_collision(self, doc):
        grid = self.node._latest_map
        if grid is None:
            raise PatrolError('STALE_DATA', '没有当前地图用于检查道路')
        info = grid.info
        oyaw = 2*math.atan2(info.origin.orientation.z, info.origin.orientation.w)
        footprint = json.loads(self.node.get_parameter('recorded_footprint').value)
        # Rasterise the complete vehicle rectangle/polygon interior, including
        # edges, at half-cell spacing. Unknown/outside cells are not traversable.
        from .region import inside_polygon
        step = info.resolution/2
        xs, ys = [p[0] for p in footprint], [p[1] for p in footprint]
        offsets = []
        x = min(xs)
        while x <= max(xs)+1e-8:
            y = min(ys)
            while y <= max(ys)+1e-8:
                if inside_polygon(x, y, footprint):
                    offsets.append((x, y))
                y += step
            x += step
        offsets += [tuple(p) for p in footprint]
        for edge in doc['edges'].values():
            if not edge.get('enabled', True):
                continue
            for block in edge['blocks']:
                for p in block['samples']:
                    c, s = math.cos(p['yaw']), math.sin(p['yaw'])
                    for ox, oy in offsets:
                        x, y = p['x']+c*ox-s*oy, p['y']+s*ox+c*oy
                        if doc.get('allowed_polygon') and not inside_polygon(x, y, doc['allowed_polygon']):
                            raise PatrolError('ROAD_COLLISION', f"道路 {edge['name']} 车身越出允许区域")
                        dx, dy = x-info.origin.position.x, y-info.origin.position.y
                        ix = math.floor((math.cos(oyaw)*dx+math.sin(oyaw)*dy)/info.resolution)
                        iy = math.floor((-math.sin(oyaw)*dx+math.cos(oyaw)*dy)/info.resolution)
                        if not (0 <= ix < info.width and 0 <= iy < info.height) or not 0 <= grid.data[iy*info.width+ix] < 65:
                            raise PatrolError('ROAD_COLLISION', f"道路 {edge['name']} 的车身扫掠与障碍/未知区域相交")

    def preview(self, route_id, loops):
        planning_lock = getattr(self, 'planning_lock', None)
        if planning_lock and not planning_lock.acquire(blocking=False):
            raise PatrolError('BUSY', '正在规划自动接入，请稍候再试')
        try:
            return self._preview(route_id, loops)
        finally:
            if planning_lock:
                planning_lock.release()

    def _path_entry(self, document, current):
        planner = None
        def compute(start, target):
            nonlocal planner
            if planner is None:
                planner = EntryPlanner(self.node)
            return planner.compute(start, target)
        def collision(blocks):
            self._check_map_collision(dict(allowed_polygon=document.get('allowed_polygon', []),
                edges={'entry':dict(name='自动接入', blocks=blocks)}))
        try:
            return choose_entry(document, current, compute, collision)
        finally:
            if planner:
                planner.close()

    def _preview(self, route_id, loops):
        if not 0 <= loops <= 100000:
            raise PatrolError('INVALID_DATA', '循环次数无效')
        with self.lock:
            self._idle()
            if self.recorder:
                raise PatrolError('BUSY', '请先保存或放弃录制草稿')
            mapping = self._map()
            route = self.store.get_route(route_id)
            if route.navigation_mode != 'RECORDED_NETWORK' or route.map_id != mapping.map_id:
                raise PatrolError('UNSUPPORTED_MODE', '请选择当前地图的固定路线')
            net = self._network(route.network_id)
            if not net['published'] or net['revision'] != route.network_revision:
                raise PatrolError('STALE_DATA', '路网未发布或已修改，请更新并保存巡检路线')
            self._check_bindings(net['document'])
            self._require_vehicle(net['document'])
            validate_network(net['document'],self.radius)
            current = self.current()
        if is_path_route(net['document'], route):
            entry = self._path_entry(net['document'], current)
            plan = plan_path(net['document'], route.steps, current, loops, entry=entry)
        else:
            plan = NetworkPlanner(net['document']).plan(route.steps, current, loops)
        plan.update(route_id=route_id, route_revision=route.revision,
                    network_id=net['network_id'], network_revision=net['revision'],
                    map_id=mapping.map_id, map_revision=mapping.revision,
                    map_stamp=self.map_stamp(mapping))
        plan.pop('digest', None)
        plan['digest'] = digest(plan)
        with self.lock:
            self._idle()
            latest = self._map()
            network = self._network(route.network_id)
            if (not self.enabled or self.recorder or latest.map_id != mapping.map_id or
                    latest.revision != mapping.revision or self.map_stamp(latest) != self.map_stamp(mapping) or
                    network['revision'] != net['revision'] or not network['published'] or
                    self.store.get_route(route_id).revision != route.revision):
                raise PatrolError('STALE_DATA', '规划期间地图、道路或任务已变化，请重新开始')
            self._require_vehicle(network['document'])
            now = self.current()
            if distance(now, current) > .10 or abs(angle(now['yaw']-current['yaw'])) > .20:
                raise PatrolError('OFF_ROAD', '规划期间车辆位姿已变化，请停车后重新开始')
            self.plans[plan['digest']] = copy.deepcopy(plan)
            while len(self.plans) > 20:
                self.plans.pop(next(iter(self.plans)))
            self.publish_plan(plan)
        return plan

    def take_plan(self, route, requested_digest, loops):
        with self.lock:
            if not self.enabled or self.recorder:
                raise PatrolError('DISABLED', '固定路线未启用或尚有录制草稿')
            plan = copy.deepcopy(self.plans.get(requested_digest))
            if not plan or plan['route_id'] != route.route_id or plan['route_revision'] != route.revision or plan['loop_count'] != loops:
                raise PatrolError('STALE_DATA', '请先预览当前固定路线，再按已确认的计划启动')
            net = self._network(route.network_id)
            self._require_vehicle(net['document'])
            self._check_bindings(net['document'])
            if not net['published'] or net['revision'] != plan['network_revision'] or digest(net['document']) != plan['network_sha256']:
                raise PatrolError('STALE_DATA', '路网已修改，请重新保存路线并预览')
            current = self.current()
            if distance(current, plan['initial_pose']) > 0.10 or abs(angle(current['yaw']-plan['initial_pose']['yaw'])) > 0.20:
                raise PatrolError('OFF_ROAD', '车辆已离开预览时的起始位姿，请重新预览')
            return plan

    def plan_home(self, home):
        context = self.store.navigation_context(self.node._active_map().map_id)
        if context['mode'] != 'RECORDED_NETWORK':
            return None
        if not self.enabled:
            raise PatrolError('DISABLED', '上次任务为固定路线，当前会话未启用固定路线，禁止自由返航')
        net = self._network(context['network_id'])
        if not net['published'] or net['revision'] != context['network_revision']:
            raise PatrolError('STALE_DATA', '返航使用的固定路网已变化，请更新并重新确认固定路线')
        self._require_vehicle(net['document'])
        validate_network(net['document'],self.radius)
        doc = copy.deepcopy(net['document'])
        ports = [key for key,n in doc['nodes'].items() if distance(n, vars(home)) <= 0.08 and
                 abs(angle(n['yaw']-home.yaw)) <= 0.26]
        if not ports:
            raise PatrolError('HOME_NOT_BOUND', '待命点没有可用的录制道路停靠节点，请补录返航支路')
        doc['bindings']['__home__'] = ports
        step = SimpleNamespace(point_id='__home__', point_name='待命返航点', x=home.x, y=home.y)
        plan = NetworkPlanner(doc).plan([step], self.current(), single_stop_loop=False)
        plan.update(network_id=net['network_id'],network_revision=net['revision'])
        self.publish_plan(plan)
        return plan

    def check_ticket(self, ticket_id, include_path=False):
        value = self.tickets.get(ticket_id)
        driver = self.node._active_driver or self.node._return_driver
        if not value or driver is None or driver.current_nav_goal_id != value['nav_uuid']:
            raise PatrolError('STALE_TICKET', '路径票据未获当前导航目标授权或已失效')
        engine = self.node._active_engine
        if engine and (engine.flags.cancel_requested or engine.flags.pause_requested):
            raise PatrolError('CANCELED', '任务正在停止')
        if self.node._return_flags and self.node._return_flags.cancel_requested:
            raise PatrolError('CANCELED', '返航正在停止')
        self.node._require_execution_safety()
        if value['vehicle'] != self.vehicle_stamp():
            raise PatrolError('STALE_DATA','运行中底盘几何参数变化，已停止固定路线')
        mapping = self.node._active_map()
        if mapping.map_id != value['map_id'] or mapping.revision != value['map_revision']:
            raise PatrolError('MAP_MISMATCH', '运行地图已变化')
        from .driver import file_signature
        if file_signature(mapping) != value['file_signature']:
            raise PatrolError('MAP_MISMATCH', '运行地图文件已变化')
        if id(self.node._latest_map) != value['map_message_id']:
            self.node._verify_runtime_map(mapping)
            value['map_message_id'] = id(self.node._latest_map)
        self.current()
        return copy.deepcopy(value) if include_path else {'valid': True, 'ticket_id': ticket_id}

    def on_progress(self, message):
        try:
            value = json.loads(message.data)
            ticket = value.get('ticket_id', '')
            with self.lock:
                if ticket not in self.tickets:
                    return
                value['received_at'] = time.monotonic()
                changed = self.progress.get('state') != value.get('state')
                self.progress = value
                self.tickets[ticket]['progress'] = value
                if changed:
                    self.store.road_event(self.tickets[ticket]['run_id'], value)
                if changed or (value.get('detour_points') and time.monotonic()-self.last_detour_preview > 0.5):
                    points = value.get('detour_points', [])
                    self._publish_paths([('局部绕行',points,1)] if points else [],self.detour_markers)
                    self.last_detour_preview = time.monotonic()
        except (ValueError, TypeError):
            return

    def _reset_stop(self):
        self.stationary = False
        self.stop_since = 0.0
        self.stop_pose = None
        self.stop_map = None
        self.moving_ticks = 0

    def _safe_map_pose(self):
        try:
            x, y, yaw, _ = self.node._current_pose()
            return dict(x=x, y=y, yaw=yaw)
        except PatrolError:
            return None

    def _freeze_at_stop(self, recorder, current):
        """Hold the pose reached when the wheels stop.

        The pose estimate keeps drifting while the robot is stationary; adding
        those samples would turn a stop into a sideways road segment. Record
        the stop pose once, then keep the sample clock alive with identical
        held samples so the next moving sample does not look like a 10 s
        sampling gap.
        """
        if not self.stationary:
            self.stationary = True
            self.stop_since = time.monotonic()
            self.stop_pose = dict(current)
            self.stop_map = self._safe_map_pose()
            recorder.add(current)
            return
        latest_map = self._safe_map_pose()
        if latest_map is not None and abs(current['v']) < 0.05:
            self.stop_map = latest_map
        if current['t'] - recorder.samples[-1]['t'] >= 0.5:
            recorder.add(dict(self.stop_pose, t=current['t'], v=0.0))

    def _stop_split_needed(self, recorder, current):
        if not self.path_mode or not self.stationary:
            return False
        if time.monotonic()-self.stop_since < 0.8:
            return False
        if length(recorder.samples) < 0.5:
            return False
        end = recorder.samples[-1]
        # Compare the departure pose with the held stop pose, not the map
        # pose: while parked, ICP keeps the map pose at the stop while LIO
        # drifts away by up to 12 cm. That drift is what would become a
        # sideways step when motion resumes.
        gap = distance(end, current)
        heading = abs(angle(current['yaw']-end['yaw']))
        return gap > 0.05 or heading > 0.02

    def _auto_stop_split(self):
        recorder = self.recorder
        if not recorder or recorder.state != 'RECORDING' or not self.path_mode:
            return False
        try:
            network = self._network(self.network_id)
            self._save_path(dict(owner_session_id=self.owner,
                                 network_id=self.network_id,
                                 expected_revision=network['revision']), complete=False)
            self.node.get_logger().info('自动分段：停车漂移已转为节点')
            return True
        except PatrolError as error:
            self.node.get_logger().warn(f'自动分段失败，保留当前录制：{error.message}')
            return False
        finally:
            self._reset_stop()

    def tick(self):
        with self.lock:
            r = self.recorder
            if r and r.state == 'RECORDING':
                try:
                    if self.node._active_map().map_id != r.map_id or self.node._active_map().revision != r.map_revision:
                        raise PatrolError('MAP_MISMATCH', '录制时地图已变化')
                    current=self.current()
                    seen,enabled=self.operator_seen.get(self.owner,(0,False))
                    if not enabled or time.monotonic()-seen>3:
                        self._reset_stop()
                        r.pause(current)
                        r.error='操作端离线或操作已禁用，录制已暂停；可恢复草稿预览后保存'
                        self._save_draft()
                    elif abs(current['v']) < 0.05:
                        self.moving_ticks = 0
                        self._freeze_at_stop(r, current)
                    else:
                        if self.stationary and self.moving_ticks < 2:
                            # Ignore a one- or two-tick wheel/IMU sign blip at
                            # the end of braking; it is still the same stop.
                            self.moving_ticks += 1
                            self._freeze_at_stop(r, current)
                        else:
                            split = self._stop_split_needed(r, current) if self.stationary else False
                            if split:
                                self._auto_stop_split()
                            else:
                                self._reset_stop()
                            # A successful auto split has replaced self.recorder;
                            # the old recorder is SAVED and must not receive this
                            # moving sample.
                            if r.state == 'RECORDING':
                                r.add(current)
                except PatrolError as error:
                    r.state, r.error = 'INVALID', error.message
                if time.monotonic()-self.last_draft_save > 1:
                    self._save_draft()
            self.status_pub.publish(String(data=json.dumps(self.dispatch('STATUS', {}), ensure_ascii=False)))

    def publish_preview(self, doc):
        paths = [(edge['name'], block['samples'], block['direction'])
                 for edge in doc.get('edges', {}).values() if edge.get('enabled', True)
                 for block in edge['blocks']]
        self._publish_paths(paths, self.markers)

    def publish_plan(self, plan):
        legs = plan['legs'] + ([plan['closure']] if plan.get('closure') else [])
        self._publish_paths([(f'leg {i+1}', b['samples'], b['direction'])
                             for i, leg in enumerate(legs) for b in leg['blocks']], self.plan_markers)

    def _publish_paths(self, paths, publisher):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for index, (name, points, direction) in enumerate(paths):
            line = Marker()
            line.header.frame_id = str(self.node.get_parameter('map_frame').value)
            line.header.stamp = self.node.get_clock().now().to_msg()
            line.ns, line.id, line.type, line.action = 'recorded_roads', index*3, Marker.LINE_STRIP, Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.035
            line.color.a, line.color.g = 0.95, 0.75
            line.color.r, line.color.b = (0.9, 0.2) if direction < 0 else (0.1, 0.95)
            line.points = [RosPoint(x=float(p['x']), y=float(p['y']), z=0.06) for p in points]
            array.markers.append(line)
            if len(points) >= 2:
                arrow = copy.deepcopy(line)
                arrow.id, arrow.type = index*3+1, Marker.ARROW
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.12, 0.16
                k = min(len(points)-1, max(1, len(points)//2))
                a, b = points[max(0, k-4)], points[k]
                arrow.points = [RosPoint(x=float(p['x']), y=float(p['y']), z=0.10) for p in (a, b)]
                array.markers.append(arrow)
        publisher.publish(array)
