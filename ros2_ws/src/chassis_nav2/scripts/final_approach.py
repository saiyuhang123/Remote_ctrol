#!/usr/bin/env python3
"""RViz 目标唯一入口：先派 Nav2 粗导（大容差），到位后精细收敛到 cm/度级。"""
import math
import traceback
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import tf2_ros
from chassis_nav2.final_approach_logic import FinalApproachLogic


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FinalApproach(Node):
    def __init__(self):
        super().__init__('final_approach')
        defaults = {'map_frame': 'map', 'base_frame': 'base_link',
                    'goal_topic': '/goal_pose', 'cmd_topic': '/cmd_vel_final',
                    'approach_speed': .12, 'max_curvature': .91,
                    'position_tolerance': .03, 'yaw_tolerance': .05,
                    'settle_time': .5, 'fine_timeout': 120., 'approach_overshoot': .5}
        for name, value in defaults.items():
            setattr(self, name, self.declare_parameter(name, value).value)
        if not 0 < self.approach_speed <= .3 or self.max_curvature <= 0:
            raise ValueError('approach_speed/max_curvature 配置无效')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, QoSProfile(depth=10))
        self.status_pub = self.create_publisher(
            String, '/final_approach/status', QoSProfile(depth=10))
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal, 10)
        self.state = 'idle'  # idle | navigating | fine
        self.goal_map = None  # (x, y, yaw) in map frame
        self.logic = None
        self.tf_lost_since = None
        self.last_progress = None
        self.create_timer(.05, self.tick)
        self.announce(
            f'就绪：{self.goal_topic} -> Nav2 粗导 -> 精细收敛 '
            f'(±{self.position_tolerance:.2f}m / ±{math.degrees(self.yaw_tolerance):.0f}°)')

    def announce(self, text, warn=False):
        (self.get_logger().warning if warn else self.get_logger().info)(text)
        self.status_pub.publish(String(data=text))

    def on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warning(f'目标坐标系 {msg.header.frame_id} 不是 {self.map_frame}，忽略')
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        yaw = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.goal_map = (x, y, yaw)
        self.logic = None
        self.state = 'navigating'
        if not self.nav_client.wait_for_server(timeout_sec=2.):
            self.announce('Nav2 navigate_to_pose action 不可用', warn=True)
            self.state = 'idle'
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.announce(f'新目标 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)，派给 Nav2 粗导')
        self.nav_client.send_goal_async(goal).add_done_callback(self._on_nav_accepted)

    def _on_nav_accepted(self, future):
        handle = future.result()
        if self.state != 'navigating':
            return
        if not handle.accepted:
            self.announce('Nav2 拒绝了目标', warn=True)
            self.state = 'idle'
            return
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        if self.state != 'navigating':
            return
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.announce('Nav2 粗导完成，开始精细收敛')
            self.logic = FinalApproachLogic(
                speed=self.approach_speed, max_curvature=self.max_curvature,
                position_tolerance=self.position_tolerance, yaw_tolerance=self.yaw_tolerance,
                settle_time=self.settle_time, timeout=self.fine_timeout,
                overshoot=self.approach_overshoot)
            self.last_progress = None
            self.state = 'fine'
        else:
            self.announce(f'Nav2 粗导失败（status={future.result().status}），不接管', warn=True)
            self.state = 'idle'

    def tick(self):
        if self.state != 'fine' or self.goal_map is None:
            return
        try:
            self._fine_tick()
        except Exception:
            self.get_logger().error(f'精细收敛异常，已停车：\n{traceback.format_exc()}')
            self.publish(0., 0.)
            self.announce('精细收敛内部异常，已停车（详见日志）', warn=True)
            self.state = 'idle'

    def _fine_tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            if self.tf_lost_since is None:
                self.tf_lost_since = self.get_clock().now()
                self.get_logger().warning(f'精细收敛：map->base_link 不可用（{error}），原地等待')
            if (self.get_clock().now() - self.tf_lost_since).nanoseconds > 1e9:
                self.publish(0., 0.)
                self.state = 'idle'
                self.announce('定位丢失超过 1s，精细收敛中止', warn=True)
            return
        self.tf_lost_since = None
        cx, cy = tf.transform.translation.x, tf.transform.translation.y
        cyaw = 2 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
        gx, gy, gyaw = self.goal_map
        ex, ey = gx - cx, gy - cy
        dx = math.cos(cyaw) * ex + math.sin(cyaw) * ey
        dy = -math.sin(cyaw) * ex + math.cos(cyaw) * ey
        dyaw = wrap(gyaw - cyaw)
        v, w, status = self.logic.update(dx, dy, dyaw, .05)
        self.publish(v, w)
        now = self.get_clock().now().nanoseconds
        if self.last_progress is None or now - self.last_progress > 2e9:
            self.last_progress = now
            remaining = (f'，RS 路径 {self.logic.path_length:.2f}m'
                         if self.logic.path_length else '')
            self.announce(
                f'精细收敛中：剩 {math.hypot(dx, dy)*100:.1f}cm / {math.degrees(abs(dyaw)):.1f}°{remaining}')
        if status == FinalApproachLogic.DONE:
            self.announce(
                f'精细收敛完成：剩余误差 {math.hypot(dx, dy)*100:.1f}cm / {math.degrees(abs(dyaw)):.1f}°')
            self.state = 'idle'
        elif status == FinalApproachLogic.FAILED:
            self.announce(
                f'精细收敛失败（超时）：剩余 {math.hypot(dx, dy)*100:.1f}cm / '
                f'{math.degrees(abs(dyaw)):.1f}°', warn=True)
            self.state = 'idle'

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FinalApproach()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
