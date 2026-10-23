"""Keep optional PTZ sweeping paused until the robot leaves an inspection point."""
import threading
import time

from patrol_interfaces.srv import PtzInspectionControl
from .models import PatrolError, new_id


class PtzInspectionLease:
    def __init__(self, node, call, check_control, timeout, callback_group=None):
        self.node, self.call, self.check_control, self.timeout = node, call, check_control, timeout
        self.client = node.create_client(PtzInspectionControl, '/ptz_patrol/inspection_control', callback_group=callback_group)
        self.lock = threading.RLock()
        self.owner, self.error = '', ''
        self.ready = self.releasing = False
        self.pending, self.deadline = None, 0.
        self.timer = node.create_timer(.4, self.renew, callback_group=callback_group)

    @staticmethod
    def request(command, owner, resume=False):
        request = PtzInspectionControl.Request()
        request.command, request.owner_id = command, owner
        request.lease_seconds, request.resume = 3., resume
        return request

    def check(self):
        with self.lock:
            if self.owner and self.error and not self.releasing:
                raise PatrolError('INSPECTION_PTZ_LOST', self.error)

    def acquire(self, flags):
        self.check_control(flags)
        # No sweeping node is required for standalone camera inspections. If an
        # older sweeping node is present, refuse to compete with its commands.
        if not self.client.wait_for_service(timeout_sec=.1):
            if 'ptz_patrol' not in self.node.get_node_names():
                return
            deadline = time.monotonic()+min(3., self.timeout)
            while not self.client.wait_for_service(timeout_sec=.05):
                self.check_control(flags)
                if time.monotonic() >= deadline:
                    raise PatrolError('INSPECTION_PTZ_UNAVAILABLE', '请更新并重启云台巡航节点，等待巡检停转接口就绪')
        with self.lock:
            if not self.owner:
                self.owner, self.error = new_id('ptz_inspection'), ''
                self.ready = self.releasing = False
            owner = self.owner
        deadline = time.monotonic()+max(1., self.timeout)
        while True:
            response = self.call(self.client, self.request(1, owner), self.timeout, '巡检云台停转', flags)
            if not response.success:
                raise PatrolError('INSPECTION_PTZ_FAILED', response.message)
            if response.stopped:
                with self.lock:
                    self.ready = True
                return
            if time.monotonic() >= deadline:
                raise PatrolError('INSPECTION_PTZ_TIMEOUT', '未收到云台停止确认，已停止后续抓拍')
            self.check_control(flags)
            time.sleep(.05)

    def renew(self):
        with self.lock:
            if not self.owner or self.releasing or self.error:
                return
            if self.pending is not None:
                if time.monotonic() < self.deadline:
                    return
                future, self.pending = self.pending, None
                self.client.remove_pending_request(future)
                future.cancel()
                self.error = '巡检云台停转续约超时，云台将保持停转'
                return
            if not self.client.service_is_ready():
                self.error = '巡检云台停转服务失联'
                return
            owner = self.owner
            try:
                future = self.client.call_async(self.request(2, owner))
                self.pending, self.deadline = future, time.monotonic()+1.2
                future.add_done_callback(lambda done: self.renew_done(done, owner))
            except Exception as error:
                self.error = str(error)

    def renew_done(self, future, owner):
        with self.lock:
            if self.owner != owner or self.pending is not future or self.releasing:
                return
            self.pending = None
            try:
                result = future.result()
                if not result.success or (self.ready and not result.stopped):
                    self.error = result.message or '巡检停转占用已失效'
            except Exception as error:
                self.error = str(error)

    def depart(self, flags=None):
        with self.lock:
            owner = self.owner
            if not owner:
                return
            self.releasing = True
            pending, self.pending = self.pending, None
        if pending is not None:
            self.client.remove_pending_request(pending)
            pending.cancel()
        try:
            response = self.call(self.client, self.request(3, owner, True), self.timeout, '离点恢复云台自转', flags)
            if not response.success:
                raise PatrolError('INSPECTION_PTZ_FAILED', response.message)
            with self.lock:
                if self.owner == owner:
                    self.owner, self.error = '', ''
                    self.ready = False
        finally:
            with self.lock:
                self.releasing = False

    def finish(self, success=False):
        # Terminal cleanup never waits on a camera service, so recognition
        # cancellation and one-click return remain prompt. The lease fails
        # closed if this final request cannot be delivered.
        with self.lock:
            owner, self.owner = self.owner, ''
            self.ready = self.releasing = False
            pending, self.pending = self.pending, None
        if pending is not None:
            self.client.remove_pending_request(pending)
            pending.cancel()
        if not owner:
            return
        try:
            future = self.client.call_async(self.request(3, owner, bool(success)))
            future.add_done_callback(self.finish_done)
        except Exception as error:
            self.node.get_logger().error('结束巡检云台联动失败：'+str(error))

    def finish_done(self, future):
        try:
            result = future.result()
            if not result.success:
                self.node.get_logger().warn('结束巡检云台联动：'+result.message)
        except Exception as error:
            self.node.get_logger().warn('结束巡检云台联动：'+str(error))
