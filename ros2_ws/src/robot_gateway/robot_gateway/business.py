"""Owned web editing and previews through the authoritative patrol services."""
import asyncio
import json
import math

from patrol_interfaces.msg import RouteStep
from patrol_interfaces.srv import (DeletePoint, DeleteRoute, ListMaps, ListPoints,
                                   ListRoutes, RoadCommand, UpsertPoint, UpsertRoute)
from rosidl_runtime_py.convert import message_to_ordereddict


async def call(client, request, timeout=12):
    if not client.service_is_ready():
        raise ValueError('巡检服务尚未启动')
    loop = asyncio.get_running_loop()
    result = loop.create_future()
    future = client.call_async(request)
    def complete(done):
        def deliver():
            if result.done():
                return
            try:
                result.set_result(done.result())
            except Exception as error:
                result.set_exception(error)
        try:
            loop.call_soon_threadsafe(deliver)
        except RuntimeError:
            pass  # The browser event loop may already be closed during teardown.
    future.add_done_callback(complete)
    try:
        return await asyncio.wait_for(result, timeout)
    finally:
        if not future.done():
            future.cancel()
            client.remove_pending_request(future)


def integer(value, label, minimum=0, maximum=2**64-1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(label+'无效')
    return value


def text(value, label, maximum=128, empty=False):
    if not isinstance(value, str) or len(value) > maximum or (not empty and not value.strip()):
        raise ValueError(label+'无效')
    return value.strip()


class WebBusiness:
    commands = ('point_save', 'point_delete', 'route_save', 'route_delete', 'route_preview', 'networks_list')

    def __init__(self, node):
        self.node = node
        self.busy = False
        self.maps = node.create_client(ListMaps, '/patrol/list_maps')
        self.points = node.create_client(ListPoints, '/patrol/list_points')
        self.routes = node.create_client(ListRoutes, '/patrol/list_routes')
        self.road = node.create_client(RoadCommand, '/patrol/road_command')
        self.clients = {key: node.create_client(kind, '/patrol/'+service) for key, kind, service in (
            ('point_save', UpsertPoint, 'upsert_point'), ('point_delete', DeletePoint, 'delete_point'),
            ('route_save', UpsertRoute, 'upsert_route'), ('route_delete', DeleteRoute, 'delete_route'))}
        self.plans = {}

    def authorized(self, websocket, token):
        return self.node._owner_ws is websocket and self.node._owner_valid(token) and self.node._lease_valid()

    async def handle(self, websocket, data):
        command, request_id = data['type'], str(data.get('request_id', ''))
        def reply(ok, code, message, **extra):
            self.node._command_result(websocket, command, ok, code, message, request_id, **extra)
        if self.busy:
            reply(False, 'BUSY', '正在处理资料或预览，请稍候')
            return
        self.busy = True
        try:
            token = data.get('session_id', '')
            if not self.authorized(websocket, token):
                raise ValueError('控制权已释放，请重新取得')
            if (self.node._task_snapshot()['running'] or self.node._local_task_active()
                    or self.node._active_nav_goal_count()):
                raise ValueError('请先结束活动任务再编辑或预览')
            map_id = text(data.get('map_id'), '地图编号')
            revision = integer(data.get('map_revision'), '地图版本', 1)
            response = await call(self.maps, ListMaps.Request())
            mapping = response.active_map
            if not response.success or mapping.map_id != map_id or mapping.revision != revision:
                raise ValueError('活跃地图或版本已变化，请刷新资料')
            request = None
            if command in ('point_save', 'point_delete', 'route_save', 'route_delete'):
                point = command.startswith('point_')
                kind = UpsertPoint if command == 'point_save' else DeletePoint if point else UpsertRoute if command == 'route_save' else DeleteRoute
                request = kind.Request()
                key = 'point_id' if point else 'route_id'
                identifier = text(data.get(key, ''), '资料编号', empty=command.endswith('_save'))
                setattr(request, key, identifier)
                request.expected_revision = integer(data.get('expected_revision', 0), '资料版本', 1 if identifier else 0)
                if not identifier and request.expected_revision:
                    raise ValueError('新资料版本应为 0')
                if identifier:
                    listing = (ListPoints.Request() if point else ListRoutes.Request())
                    listing.map_id = map_id
                    records = await call(self.points if point else self.routes, listing)
                    if not records.success or not any(getattr(item, key) == identifier for item in (records.points if point else records.routes)):
                        raise ValueError('资料不属于当前地图或已被删除')
                if command.endswith('_save'):
                    request.map_id, request.name = map_id, text(data.get('name'), '名称')
                    if point:
                        for key in ('x', 'y', 'yaw'):
                            value = data.get(key)
                            if type(value) not in (int, float) or not math.isfinite(value):
                                raise ValueError(key+'必须为有限数值')
                            setattr(request, key, float(value))
                        dx, dy = request.x-mapping.origin_x, request.y-mapping.origin_y
                        c, s = math.cos(mapping.origin_yaw), math.sin(mapping.origin_yaw)
                        if not (0 <= c*dx+s*dy < mapping.width*mapping.resolution and
                                0 <= -s*dx+c*dy < mapping.height*mapping.resolution):
                            raise ValueError('点位超出地图范围')
                        request.yaw = math.atan2(math.sin(request.yaw), math.cos(request.yaw))
                        enabled = data.get('inspection_enabled', False)
                        if type(enabled) is not bool:
                            raise ValueError('巡检开关无效')
                        request.inspection_enabled = enabled
                        request.ptz_preset = integer(data.get('ptz_preset', 0), '云台预置位', 1 if enabled else 0, 255)
                        request.recognition_prompt = text(data.get('recognition_prompt', ''), '识别要求', 2000, empty=True)
                    else:
                        request.navigation_mode = data.get('navigation_mode', 'FREE_NAV')
                        if request.navigation_mode not in ('FREE_NAV', 'RECORDED_NETWORK'):
                            raise ValueError('路线类型无效')
                        request.network_id = text(data.get('network_id', ''), '路网编号', empty=request.navigation_mode == 'FREE_NAV')
                        request.network_revision = integer(data.get('network_revision', 0), '路网版本', 1 if request.navigation_mode == 'RECORDED_NETWORK' else 0)
                        steps = data.get('steps')
                        if not isinstance(steps, list) or not 1 <= len(steps) <= 500:
                            raise ValueError('路线需要 1～500 个步骤')
                        for item in steps:
                            if not isinstance(item, dict):
                                raise ValueError('路线步骤无效')
                            step = RouteStep()
                            step.point_id = text(item.get('point_id'), '点位编号')
                            dwell = item.get('dwell_seconds', 0)
                            if type(dwell) not in (int, float) or not math.isfinite(dwell) or not 0 <= dwell <= 86400:
                                raise ValueError('停留时间无效')
                            step.dwell_seconds = float(dwell)
                            request.steps.append(step)
            if not self.authorized(websocket, token):
                raise ValueError('处理期间控制权已变化')
            if request is not None:
                result = await call(self.clients[command], request)
                if not result.success:
                    reply(False, 'REJECTED', result.message)
                    return
                self.plans.clear()
                reply(True, 'SAVED', result.message, data=message_to_ordereddict(result))
                return
            request = RoadCommand.Request()
            request.operation = 'PLAN' if command == 'route_preview' else 'LIST'
            payload = dict(owner_session_id=token, command_id=request_id)
            if command == 'route_preview':
                payload.update(route_id=text(data.get('route_id'), '路线编号'),
                               loop_count=integer(data.get('loop_count'), '圈数', 0, 100000))
            request.payload_json = json.dumps(payload)
            result = await call(self.road, request, timeout=40)
            if not result.success:
                reply(False, result.error_code, result.message)
                return
            value = json.loads(result.data_json)
            if command == 'route_preview':
                if not self.authorized(websocket, token):
                    raise ValueError('预览期间控制权已变化')
                self.plans[value['digest']] = (token, value['route_id'], value['route_revision'], value['map_id'], value['map_revision'], value['loop_count'])
                while len(self.plans) > 20:
                    self.plans.pop(next(iter(self.plans)))
                # Preserve all motion blocks and endpoints while bounding the
                # browser drawing payload. The executor retains the full plan.
                legs = list(value.get('initial_legs') or value.get('legs', []))
                if value['loop_count'] != 1 and len(value.get('pass_legs', [])) > 1:
                    legs += value['pass_legs'][1]
                if value.get('closure'):
                    legs += [value['closure']]
                blocks = [block for leg in legs for block in leg.get('blocks', [])]
                stride = max(1, math.ceil(sum(len(block.get('samples', [])) for block in blocks)/6000))
                value = {key: value[key] for key in ('digest', 'route_id', 'route_revision', 'map_id', 'map_revision', 'loop_count')}
                value['blocks'] = [dict(direction=b['direction'], samples=b['samples'][::stride]+([b['samples'][-1]] if b['samples'] else [])) for b in blocks]
            else:
                value = {'networks': [{key: item[key] for key in ('network_id', 'map_id', 'map_revision', 'name', 'revision', 'published')}
                                      for item in value.get('networks', [])]}
            reply(True, 'READY', '固定路线预览完成' if command == 'route_preview' else '路网已读取', data=value)
        except asyncio.TimeoutError:
            reply(False, 'TIMEOUT', '服务响应超时；保存可能已完成，请刷新资料核对后重试')
        except (ValueError, TypeError, KeyError, AssertionError, OverflowError) as error:
            reply(False, 'INVALID_DATA', str(error))
        except Exception as error:
            reply(False, 'SERVICE_ERROR', str(error))
        finally:
            self.busy = False
