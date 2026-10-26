"""Web device lifecycle requests relayed to the existing Qt controllers."""
import asyncio
import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, SetParametersAtomically
from std_srvs.srv import SetBool, Trigger

from .business import call


class DesktopRelay:
    commands = ('ptz_prepare', 'vision_face_capture', 'vision_person',
                'vision_plate', 'estop_release')
    features = {'vision_face_capture': 'face', 'vision_person': 'person', 'vision_plate': 'plate'}

    def __init__(self, node):
        self.node = node
        self.camera = node.create_client(Trigger, '/desktop/remote/ensure_camera')
        self.unlock = node.create_client(SetParametersAtomically, '/desktop/remote/release_estop')
        self.vision = {name: node.create_client(SetBool, '/desktop/remote/vision_'+feature)
                       for name, feature in self.features.items()}
        self.pending = {}
        self.last_error = ''
        self.unlock_pending = False

    def snapshot(self):
        with self.node._data_lock:
            pending, error = dict(self.pending), self.last_error
        return dict(camera_available=self.camera.service_is_ready(),
                    unlock_available=self.unlock.service_is_ready(),
                    vision_available=all(c.service_is_ready() for c in self.vision.values()),
                    pending=pending, last_error=error)

    def authorized(self, websocket, data, unlock=False):
        node = self.node
        with node._control_lock:
            if node._local_operator_enabled and time.monotonic()-node._local_operator_time <= 2.0:
                raise ValueError('本地操作仍启用，请先在 Qt 关闭“启用操作”')
            if unlock and not node._owner_session:
                return
            if (node._owner_ws is not websocket or not node._owner_valid(data.get('session_id', ''))
                    or not node._lease_valid()):
                raise ValueError('请先取得当前网页控制权')

    def idle(self):
        node = self.node
        if (node._task_snapshot()['running'] or node._local_task_active()
                or node._active_nav_goal_count()):
            raise ValueError('请先结束活动导航或巡检任务并等待车辆停稳')

    async def handle(self, websocket, data):
        command, request_id = data['type'], str(data.get('request_id', ''))
        node = self.node
        def reply(ok, code, message):
            node._command_result(websocket, command, ok, code, message, request_id)
        with node._data_lock:
            if command in self.pending:
                reply(False, 'BUSY', '此操作正在处理，请等待状态反馈')
                return
            self.pending[command] = '正在请求本地服务'
            self.last_error = ''
        try:
            self.authorized(websocket, data, unlock=command == 'estop_release')
            if command == 'estop_release':
                self.idle()
                if not self.unlock.service_is_ready():
                    raise ValueError('本地解除急停服务未就绪，请更新并重新打开 Qt 客户端')
                with node._control_lock:
                    if self.unlock_pending:
                        raise ValueError('正在解除急停，请稍候')
                    self.unlock_pending = True
                try:
                    issued = time.time_ns()
                    # Remove this gateway's manual publisher before the local
                    # controller checks for conflicting velocity publishers.
                    node._publish_zero_and_destroy_cmd_pub()
                    await asyncio.sleep(.2)
                    self.authorized(websocket, data, unlock=True)
                    self.idle()
                    request = SetParametersAtomically.Request()
                    request.parameters = [Parameter(name=name, value=ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER, integer_value=value))
                        for name, value in (('issued_at_ns', issued), ('deadline_ns', issued+1000000000))]
                    result = await call(self.unlock, request, timeout=3)
                    if not result.result.successful:
                        raise ValueError(result.result.reason or '本地拒绝解除急停')
                    until = time.monotonic()+2
                    while time.monotonic() < until:
                        self.authorized(websocket, data, unlock=True)
                        if node._estop_check()[0]:
                            reply(True, 'READY', '急停已解除；车辆保持静止，原任务不会自动恢复')
                            return
                        await asyncio.sleep(.1)
                    raise ValueError('本地已处理请求，但未收到新鲜的解除反馈，请查看 Qt 状态后再试')
                finally:
                    with node._control_lock:
                        self.unlock_pending = False
            elif command == 'ptz_prepare':
                if not self.camera.service_is_ready():
                    raise ValueError('本地设备中继未就绪，请更新并重新打开 Qt 客户端')
                result = await call(self.camera, Trigger.Request(), timeout=5)
                if not result.success:
                    raise ValueError(result.message)
                reply(True, 'PENDING', result.message)
                await self.wait_ready(websocket, data, lambda: (
                    node._ptz_control_client.service_is_ready() and node._ptz_pose_client.service_is_ready()
                    and node._ptz_driver_fresh()), timeout=30)
                reply(True, 'READY', '本地云台服务已就绪：RS485 控制移动，网口控制光学变倍')
            else:
                enabled = data.get('enabled')
                if type(enabled) is not bool:
                    raise ValueError('检测开关必须为布尔值')
                feature = self.features[command]
                client = self.vision[command]
                if client.service_is_ready():
                    request = SetBool.Request(); request.data = enabled
                    result = await call(client, request, timeout=5)
                    if not result.success:
                        raise ValueError(result.message)
                    message = result.message
                elif node._params_client.service_is_ready():
                    # Already running external detectors remain controllable;
                    # process startup still belongs exclusively to Qt.
                    request = SetParameters.Request()
                    request.parameters = [Parameter(name='enable_'+feature, value=ParameterValue(
                        type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
                    result = await call(node._params_client, request, timeout=5)
                    if not result.results or not all(r.successful for r in result.results):
                        raise ValueError('；'.join(r.reason for r in result.results) or '检测节点拒绝开关请求')
                    message = '检测开关已提交，等待实际状态反馈'
                elif not enabled:
                    reply(True, 'READY', '检测节点未运行，该功能已关闭')
                    return
                else:
                    raise ValueError('本地视觉启动服务未就绪，请更新并重新打开 Qt 客户端')
                reply(True, 'PENDING', message)
                def ready():
                    with node._data_lock:
                        status = dict(node._detector_status)
                        fresh = bool(node._detector_time and time.monotonic()-node._detector_time <= 3)
                    requested = status.get(feature+'_requested', status.get(feature, False))
                    return (fresh and bool(requested) == enabled and
                            (not enabled or (bool(status.get(feature)) and status.get('input_ready') is True)))
                await self.wait_ready(websocket, data, ready, timeout=35)
                reply(True, 'READY', '检测已开启，正在使用本地相机与记录服务' if enabled else '检测已关闭')
            with node._data_lock:
                self.last_error = ''
        except asyncio.TimeoutError:
            message = '本地服务响应超时，请查看 Qt 日志和实际状态后再试'
            with node._data_lock: self.last_error = message
            reply(False, 'TIMEOUT', message)
        except (ValueError, TypeError, KeyError) as error:
            with node._data_lock: self.last_error = str(error)
            reply(False, 'REJECTED', str(error))
        except Exception as error:
            with node._data_lock: self.last_error = str(error)
            reply(False, 'LOCAL_SERVICE_ERROR', str(error))
        finally:
            with node._data_lock: self.pending.pop(command, None)

    async def wait_ready(self, websocket, data, predicate, timeout):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            self.authorized(websocket, data)
            if predicate():
                return
            await asyncio.sleep(.15)
        raise ValueError('本地组件未在期限内就绪，请查看 Qt 日志、相机连接、串口或模型状态')
