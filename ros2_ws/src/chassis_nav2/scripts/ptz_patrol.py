#!/usr/bin/env python3
"""导航期间驱动海康相机云台左右巡航（多角度巡检展示）。

监听 /navigate_to_pose 动作状态：有目标在执行时，通过 /hk_camera/control_ptz
以固定速度左右扫掠；导航结束（或节点退出后租约 500ms 不续）云台自停。
hk_camera 的 control_ptz 是 500ms 租约制，本节点 5Hz 续租，节点死掉云台自动停。

换向策略（两种同时生效，先到先换）：
- 时间兜底：单侧扫满 sweep_seconds 秒换向，不需要任何角度反馈；
- 角度边界：设置 pan_min/pan_max 后按 /hk_camera/get_ptz_pose 的实际角度换向，
  到达边界且距上次换向超过 3s 才触发。不假设"左/右指令使角度增/减"的方向约定，
  约定反了也会在另一侧边界自我纠正。

相机节点没启动时只周期性告警，不影响导航；运行中可 ros2 param set /ptz_patrol enabled false 关停。
"""
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node

from hk_camera.srv import ControlPTZ, GetPTZPose, SetPTZPose

PAN_LEFT = 23    # 海康 SDK 指令：左
PAN_RIGHT = 24   # 海康 SDK 指令：右
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
MIN_FLIP_GAP = 3.0   # 角度边界换向的最小间隔，防边界抖动


class PtzPatrol(Node):
    def __init__(self):
        super().__init__('ptz_patrol')
        self.declare_parameter('enabled', True)         # 总开关
        self.declare_parameter('pan_speed', 2)          # 海康速度档 1~7
        self.declare_parameter('sweep_seconds', 20.0)   # 单侧扫掠时长（时间兜底换向）
        self.declare_parameter('pan_min', -1.0)         # >=0 且 < pan_max 时启用角度边界换向（度）
        self.declare_parameter('pan_max', -1.0)
        self.declare_parameter('home_pan', -1.0)        # 三个 home_* 都设了，导航结束后回该姿态
        self.declare_parameter('home_tilt', -1.0)
        self.declare_parameter('home_zoom', -1.0)
        self.declare_parameter('status_topic', '/navigate_to_pose/_action/status')

        self._ctrl = self.create_client(ControlPTZ, '/hk_camera/control_ptz')
        self._pose = self.create_client(GetPTZPose, '/hk_camera/get_ptz_pose')
        self._set = self.create_client(SetPTZPose, '/hk_camera/set_ptz_pose')
        self.create_subscription(GoalStatusArray, self.get_parameter('status_topic').value,
                                 self._on_status, 10)

        self._active = False                    # 动作服务器报告有目标在执行
        self._last_status = None                # 最近一次状态消息时间
        self._moving = False                    # 我们认为云台正在转
        self._direction = PAN_LEFT
        self._last_flip = self.get_clock().now()  # 上次换向时刻
        self._ctrl_pending = False
        self._pose_pending = False
        self._pan = None                        # 最近一次云台水平角（仅角度模式轮询）
        self.create_timer(0.2, self._tick)      # 5Hz 续租/换向
        self.create_timer(1.0, self._poll_pose)
        self.get_logger().info('ptz_patrol 就绪：导航执行中云台左右巡航')

    def _on_status(self, msg):
        self._last_status = self.get_clock().now()
        self._active = any(s.status in ACTIVE for s in msg.status_list)

    def _navigating(self):
        # 状态话题由动作服务器周期发布；超过 5s 无消息视为导航栈已退出
        return (self._active and self._last_status is not None and
                (self.get_clock().now() - self._last_status).nanoseconds < 5e9)

    def _use_pan_bounds(self):
        pmin = self.get_parameter('pan_min').value
        pmax = self.get_parameter('pan_max').value
        return 0 <= pmin < pmax <= 360

    def _tick(self):
        if not self.get_parameter('enabled').value or not self._navigating():
            if self._moving:
                self._send_stop()
            return
        if not self._ctrl.service_is_ready():
            self.get_logger().warn('等待 /hk_camera/control_ptz（相机节点未启动？）',
                                   throttle_duration_sec=10.0)
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_flip).nanoseconds * 1e-9
        flip = elapsed >= self.get_parameter('sweep_seconds').value
        if not flip and elapsed > MIN_FLIP_GAP and \
                self._use_pan_bounds() and self._pan is not None:
            pmin = self.get_parameter('pan_min').value
            pmax = self.get_parameter('pan_max').value
            if self._pan <= pmin or self._pan >= pmax:
                flip = True
        if flip:
            self._direction = PAN_RIGHT if self._direction == PAN_LEFT else PAN_LEFT
            self._last_flip = now
        if not self._ctrl_pending:
            self._send_start()

    def _send_start(self):
        req = ControlPTZ.Request()
        req.command = self._direction
        req.speed = max(1, min(7, int(self.get_parameter('pan_speed').value)))
        req.stop = False
        self._ctrl_pending = True
        future = self._ctrl.call_async(req)
        future.add_done_callback(self._on_ctrl_done)
        self._moving = True

    def _send_stop(self):
        req = ControlPTZ.Request()
        req.stop = True
        self._ctrl_pending = True
        future = self._ctrl.call_async(req)
        future.add_done_callback(self._on_ctrl_done)
        self._moving = False
        self._go_home()

    def _on_ctrl_done(self, future):
        self._ctrl_pending = False
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(f'云台控制失败：{res.message}',
                                       throttle_duration_sec=10.0)
        except Exception as exc:
            self.get_logger().warn(f'云台控制调用异常：{exc}', throttle_duration_sec=10.0)

    def _poll_pose(self):
        if not self._use_pan_bounds() or not self._moving or self._pose_pending:
            return
        if not self._pose.service_is_ready():
            return
        self._pose_pending = True
        future = self._pose.call_async(GetPTZPose.Request())
        future.add_done_callback(self._on_pose_done)

    def _on_pose_done(self, future):
        self._pose_pending = False
        try:
            res = future.result()
            self._pan = res.pan if res.success else None
        except Exception:
            self._pan = None

    def _go_home(self):
        pan = self.get_parameter('home_pan').value
        tilt = self.get_parameter('home_tilt').value
        zoom = self.get_parameter('home_zoom').value
        if not (0 <= pan <= 360 and 0 <= tilt <= 360 and zoom >= 1):
            return
        if not self._set.service_is_ready():
            return
        req = SetPTZPose.Request()
        req.pan, req.tilt, req.zoom = float(pan), float(tilt), float(zoom)
        self._set.call_async(req)   # 尽力而为，回位失败不影响导航
        self.get_logger().info(f'导航结束，云台回位 pan={pan} tilt={tilt} zoom={zoom}')


def main():
    rclpy.init()
    node = PtzPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
