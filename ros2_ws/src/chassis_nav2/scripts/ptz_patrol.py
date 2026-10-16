#!/usr/bin/env python3
"""海康相机云台左右巡航（多角度巡检展示），与导航完全解耦、手动开关。

开启后通过 /hk_camera/control_ptz 以固定速度左右扫掠；关闭或节点退出
（租约 500ms 不续）云台自停。hk_camera 的 control_ptz 是 500ms 租约制，
本节点 5Hz 续租，节点死掉云台自动停。

开关：ros2 param set /ptz_patrol enabled true|false（默认 false，不转）。
节点随 Nav2 三套方案自动拉起（默认静默空转），也可单独 ros2 run chassis_nav2 ptz_patrol.py。

换向策略（两种同时生效，先到先换）：
- 时间兜底：单侧扫满 sweep_seconds 秒换向，不需要任何角度反馈；
- 角度边界：设置 pan_min/pan_max 后按 /hk_camera/get_ptz_pose 的实际角度换向，
  到达边界且距上次换向超过 3s 才触发。不假设"左/右指令使角度增/减"的方向约定，
  约定反了也会在另一侧边界自我纠正。

俯仰（tilt_enabled，默认开）：pattern 决定运动方式——
- diagonal（默认）：斜向指令四段绕圈，水平垂直同时动。俯仰幅度受控：
  每次开启巡航时以当前俯仰角为中心，只在其 ±tilt_range 度（默认 5°）内动，
  到边界即换向（"高低低高"小幅转圈）；设备不回读角度时退化为按 tilt_seconds
  定时换向——此时巡航以开启那一刻的实际位置为中心上下交替，
  先把相机手动调到想要的高度再开启即可；上下角速度不一致导致一边偏高/偏低时，
  用 tilt_up_seconds / tilt_down_seconds 分别压缩该方向的时长。
  回读值冻结不变（设备不上报真实角度）也自动退化为定时换向；
- raster：水平扫到头 → 俯仰 tilt_seconds 秒 → 反向水平扫 → 反向俯仰，上下交替。
  云台控制是单指令通道时才需要的降级方案（老固件不支持斜向指令时用）。

相机节点没启动时只周期性告警。
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from hk_camera.srv import ControlPTZ, GetPTZPose, SetPTZPose

PAN_LEFT = 23    # 海康 SDK 指令：左
PAN_RIGHT = 24   # 海康 SDK 指令：右
TILT_UP = 21     # 海康 SDK 指令：上
TILT_DOWN = 22   # 海康 SDK 指令：下
DIAGONAL_LEG = {   # (俯仰方向, 水平方向) -> 海康 SDK 斜向指令
    (TILT_UP, PAN_LEFT): 25, (TILT_UP, PAN_RIGHT): 26,
    (TILT_DOWN, PAN_RIGHT): 28, (TILT_DOWN, PAN_LEFT): 27,
}
MIN_FLIP_GAP = 3.0   # 角度边界换向的最小间隔，防边界抖动


class PtzPatrol(Node):
    def __init__(self):
        super().__init__('ptz_patrol')
        self.declare_parameter('enabled', False)        # 总开关，默认不转，手动开启
        self.declare_parameter('pattern', 'diagonal')   # diagonal 斜向绕圈；raster 水平扫+换向俯仰
        self.declare_parameter('pan_speed', 2)          # 海康速度档 1~7
        self.declare_parameter('sweep_seconds', 20.0)   # 单侧扫掠时长（时间兜底换向）
        self.declare_parameter('pan_min', -1.0)         # >=0 且 < pan_max 时启用角度边界换向（度）
        self.declare_parameter('pan_max', -1.0)
        self.declare_parameter('tilt_enabled', True)    # 水平换向时顺带俯仰一段（扫描式，见模块说明）
        self.declare_parameter('tilt_speed', 2)         # 俯仰速度档 1~7
        self.declare_parameter('tilt_seconds', 4.0)     # 无角度反馈时的俯仰换向时长；raster 每次俯仰时长
        self.declare_parameter('tilt_up_seconds', -1.0)   # >0 时覆盖上抬时长（往上偏高就调小它）
        self.declare_parameter('tilt_down_seconds', -1.0) # >0 时覆盖下压时长（调反了说明设备约定相反，两值互换）
        self.declare_parameter('tilt_range', 5.0)       # diagonal 俯仰半幅（度），以开启时角度为中心
        self.declare_parameter('home_pan', -1.0)        # 三个 home_* 都设了，关停后回该姿态
        self.declare_parameter('home_tilt', -1.0)
        self.declare_parameter('home_zoom', -1.0)

        self._ctrl = self.create_client(ControlPTZ, '/hk_camera/control_ptz')
        self._pose = self.create_client(GetPTZPose, '/hk_camera/get_ptz_pose')
        self._set = self.create_client(SetPTZPose, '/hk_camera/set_ptz_pose')

        self._moving = False                    # 我们认为云台正在转
        self._direction = PAN_LEFT
        self._leg = 0                           # diagonal 模式的段序号
        self._tilt_dir = TILT_UP
        self._tilt_until = None                 # 非 None 表示正在俯仰段（时刻）
        self._last_flip = self.get_clock().now()  # 上次水平换向时刻
        self._tilt_last_flip = self.get_clock().now()  # 上次俯仰换向时刻
        self._was_enabled = False               # 用于开启瞬间锚定俯仰中心
        self._tilt = None                       # 最近一次云台俯仰角
        self._tilt_center = None                # 开启时锚定的俯仰中心
        self._tilt_varies = False               # 回读值是否真的随运动变化（防冻结）
        self._ctrl_pending = False
        self._pose_pending = False
        self._pan = None                        # 最近一次云台水平角（仅角度模式轮询）
        self.create_timer(0.2, self._tick)      # 5Hz 续租/换向
        self.create_timer(1.0, self._poll_pose)
        self.get_logger().info('ptz_patrol 就绪：ros2 param set /ptz_patrol enabled true 开始巡航')

    def _use_pan_bounds(self):
        pmin = self.get_parameter('pan_min').value
        pmax = self.get_parameter('pan_max').value
        return 0 <= pmin < pmax <= 360

    def _dir_tilt_seconds(self):
        # 当前俯仰方向的换向时长；上下可不等（云台受重力影响两边角速度常不一致）
        up = self.get_parameter('tilt_up_seconds').value
        down = self.get_parameter('tilt_down_seconds').value
        base = self.get_parameter('tilt_seconds').value
        if self._tilt_dir == TILT_UP:
            return up if up > 0 else base
        return down if down > 0 else base

    def _tick(self):
        if not self.get_parameter('enabled').value:
            self._was_enabled = False
            if self._moving:
                self._send_stop()
            return
        if not self._ctrl.service_is_ready():
            self.get_logger().warn('等待 /hk_camera/control_ptz（相机节点未启动？）',
                                   throttle_duration_sec=10.0)
            return
        now = self.get_clock().now()
        if not self._was_enabled:
            # 每次开启重新锚定俯仰中心（首个角度回读到位后生效）
            self._was_enabled = True
            self._tilt_center = None
            self._tilt_varies = False
            self._last_flip = self._tilt_last_flip = now
        if self.get_parameter('pattern').value == 'diagonal':
            if (now - self._last_flip).nanoseconds * 1e-9 >= \
                    self.get_parameter('sweep_seconds').value:
                self._direction = PAN_RIGHT if self._direction == PAN_LEFT else PAN_LEFT
                self._last_flip = now
            tilt_flip = False
            tilt_gap = (now - self._tilt_last_flip).nanoseconds * 1e-9
            if self._tilt is not None and self._tilt_center is not None and self._tilt_varies:
                rng = self.get_parameter('tilt_range').value
                # 不假设"上/下指令使角度增/减"的约定，越界即换向，反了也能自我纠正
                if tilt_gap > MIN_FLIP_GAP and \
                        (self._tilt >= self._tilt_center + rng or
                         self._tilt <= self._tilt_center - rng):
                    tilt_flip = True
            elif tilt_gap >= self._dir_tilt_seconds():
                tilt_flip = True
            if tilt_flip:
                self._tilt_dir = TILT_DOWN if self._tilt_dir == TILT_UP else TILT_UP
                self._tilt_last_flip = now
            if not self._ctrl_pending:
                self._send_start(DIAGONAL_LEG[(self._tilt_dir, self._direction)],
                                 self.get_parameter('pan_speed').value)
            return
        if self._tilt_until is not None:
            # 俯仰段：时间到就水平换向、交替俯仰方向，回到水平扫
            if now >= self._tilt_until:
                self._tilt_until = None
                self._tilt_dir = TILT_DOWN if self._tilt_dir == TILT_UP else TILT_UP
                self._direction = PAN_RIGHT if self._direction == PAN_LEFT else PAN_LEFT
                self._last_flip = now
        else:
            elapsed = (now - self._last_flip).nanoseconds * 1e-9
            flip = elapsed >= self.get_parameter('sweep_seconds').value
            if not flip and elapsed > MIN_FLIP_GAP and \
                    self._use_pan_bounds() and self._pan is not None:
                pmin = self.get_parameter('pan_min').value
                pmax = self.get_parameter('pan_max').value
                if self._pan <= pmin or self._pan >= pmax:
                    flip = True
            if flip:
                if self.get_parameter('tilt_enabled').value and \
                        self.get_parameter('tilt_seconds').value > 0:
                    self._tilt_until = now + Duration(
                        seconds=self.get_parameter('tilt_seconds').value)
                else:
                    self._direction = PAN_RIGHT if self._direction == PAN_LEFT else PAN_LEFT
                    self._last_flip = now
        if not self._ctrl_pending:
            if self._tilt_until is not None:
                self._send_start(self._tilt_dir, self.get_parameter('tilt_speed').value)
            else:
                self._send_start(self._direction, self.get_parameter('pan_speed').value)

    def _send_start(self, command, speed):
        req = ControlPTZ.Request()
        req.command = command
        req.speed = max(1, min(7, int(speed)))
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
        if not self._moving or self._pose_pending:
            return
        if not (self._use_pan_bounds() or
                self.get_parameter('pattern').value == 'diagonal'):
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
        except Exception:
            res = None
        if res is None or not res.success:
            self._pan = self._tilt = None
            return
        self._pan = res.pan
        self._tilt = res.tilt
        if self.get_parameter('pattern').value != 'diagonal':
            return
        if self._tilt_center is None:
            self._tilt_center = res.tilt
            self.get_logger().info(
                f'俯仰中心已锚定 {res.tilt:.1f}°，巡航幅度 ±{self.get_parameter("tilt_range").value}°')
        elif abs(res.tilt - self._tilt_center) > 0.2:
            self._tilt_varies = True

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
        self._set.call_async(req)   # 尽力而为，回位失败不影响其他功能
        self.get_logger().info(f'巡航停止，云台回位 pan={pan} tilt={tilt} zoom={zoom}')


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
