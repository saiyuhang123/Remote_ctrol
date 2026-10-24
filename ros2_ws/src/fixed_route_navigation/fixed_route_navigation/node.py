"""Passive route recording and explicitly started fixed-route navigation."""
from __future__ import annotations
import csv
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .route import Point, Tracker, distance, load_csv, motion_blocks, quaternion_yaw, swept_collision, wrap


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class FixedRoute(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('fixed_route', parameter_overrides=parameter_overrides)
        defaults = {
            'route_file': '', 'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
            'pose_topic': '/current_pose', 'odom_topic': '/odom', 'scan_topic': '/scan',
            'cmd_topic': '/cmd_vel_auto', 'max_speed': .3, 'max_reverse_speed': .15,
            'allow_reverse': False, 'min_turning_radius': .69, 'acceleration': .25, 'deceleration': .4,
            'min_lookahead': .45, 'max_lookahead': .8, 'lookahead_time': 1., 'goal_tolerance': .1,
            'goal_yaw_tolerance': .35, 'start_tolerance': .3, 'start_yaw_tolerance': .5,
            'max_cross_track': .45, 'max_segment_length': 1., 'pose_timeout': .5,
            'scan_timeout': .5, 'health_timeout': .5, 'obstacle_margin': .12,
            'obstacle_clear_time': .5, 'stop_settle_time': .3, 'record_spacing': .05,
            'record_angle': .05,
            'legacy_spacing': .08,
            'footprint': '[[0.4475,0.28],[0.4475,-0.28],[-0.4475,-0.28],[-0.4475,0.28]]',
        }
        for name, value in defaults.items():
            setattr(self, name, self.declare_parameter(name, value).value)
        for key, value in defaults.items():
            if isinstance(value, float) and (not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0):
                raise ValueError(f'{key} 必须为有限正数')
        if self.max_lookahead < self.min_lookahead:
            raise ValueError('max_lookahead 必须不小于 min_lookahead')
        self.polygon = json.loads(self.footprint)
        if (not isinstance(self.polygon, list) or len(self.polygon) < 3 or
                any(len(p) != 2 or not all(math.isfinite(x) for x in p) for p in self.polygon)):
            raise ValueError('footprint 必须是至少三个有限 x/y 顶点的 JSON')
        if not self.route_file.strip():
            raise ValueError('需要 route_file:=/绝对路径/route.csv')
        self.filename = Path(self.route_file).expanduser().resolve()
        self.state, self.reason = 'IDLE', '等待录制或启动路线'
        self.pose, self.pose_msg, self.odom, self.scan = None, None, None, None
        self.pose_receipt = self.odom_receipt = self.scan_receipt = self.health_receipt = -math.inf
        self.healthy, self.estopped = False, False
        self.last_teleop = -math.inf
        self.tracker, self.points = None, []
        self.last_velocity, self.last_tick = 0., time.monotonic()
        self.last_ros_time = self.get_clock().now().nanoseconds*1e-9
        self.clear_since, self.settle_since = None, None
        self.stop_until = -math.inf
        self.record_stream, self.record_writer, self.record_last = None, None, None
        self.record_points = 0
        self.record_previous_pose = None
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, '/fixed_route/status', latched)
        self.path_pub = self.create_publisher(PathMsg, '/fixed_route/path', latched)
        self.target_pub = self.create_publisher(PoseStamped, '/fixed_route/target', 1)
        self.reached_pub = self.create_publisher(Bool, '/fixed_route/reached', latched)
        self.create_subscription(PoseStamped, self.pose_topic, self.receive_pose, 10)
        self.create_subscription(Odometry, self.odom_topic, self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.receive_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/localization/healthy', self.receive_health, 1)
        self.create_subscription(Bool, '/emergency_stop', self.receive_estop, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.receive_teleop, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.receive_initial_pose, 10)
        for name in ('start', 'pause', 'resume', 'stop', 'reload', 'start_recording', 'stop_recording'):
            self.create_service(Trigger, '/fixed_route/'+name, self.service(name))
        self.create_timer(.05, self.tick)
        self.create_timer(.2, self.publish_status)
        self.publish_status()
        self.get_logger().info(f'固定路线就绪（等待服务启动）：{self.filename}')

    def service(self, name):
        def call(_request, response):
            try:
                response.message = getattr(self, name)()
                response.success = True
            except (ValueError, OSError) as exc:
                response.success, response.message = False, str(exc)
            self.publish_status()
            return response
        return call

    def receive_pose(self, msg):
        try:
            p, q = msg.pose.position, msg.pose.orientation
            if msg.header.frame_id != self.map_frame or not all(math.isfinite(x) for x in (p.x, p.y, p.z)):
                raise ValueError('定位坐标或 frame_id 无效')
            pose = Point(p.x, p.y, p.z, quaternion_yaw([q.x, q.y, q.z, q.w]))
            if self.pose_msg is not None:
                dt = stamp_seconds(msg.header.stamp)-stamp_seconds(self.pose_msg.header.stamp)
                if dt <= 0:
                    return
                if distance(pose, self.pose) > .20+2*self.max_speed*dt or abs(wrap(pose.yaw-self.pose.yaw)) > .4+2*self.max_speed/self.min_turning_radius*dt:
                    if self.state == 'RECORDING':
                        self.invalidate_record('录制期间定位跳变，保留 .partial，请重新录制')
                    elif self.state in ('FOLLOWING', 'SETTLING'):
                        self.halt('PAUSED', '定位跳变，请核对位置后 resume')
            self.pose, self.pose_msg, self.pose_receipt = pose, msg, time.monotonic()
        except ValueError as exc:
            self.pose = self.pose_msg = None
            self.reason = str(exc)

    def receive_initial_pose(self, _msg):
        # Also stops execution if an old healthy heartbeat is still in flight.
        self.healthy, self.pose, self.pose_msg = False, None, None
        if self.state == 'RECORDING':
            self.invalidate_record('录制期间重设初始位姿，请重新录制')
        elif self.state in ('FOLLOWING', 'SETTLING'):
            self.halt('PAUSED', '已重设初始位姿，等待重新定位后 resume')

    def receive_odom(self, msg):
        values = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        if msg.header.frame_id == self.odom_frame and msg.child_frame_id == self.base_frame and all(math.isfinite(x) for x in values):
            self.odom, self.odom_receipt = msg, time.monotonic()
        else:
            self.odom = None

    def receive_scan(self, msg):
        self.scan, self.scan_receipt = msg, time.monotonic()

    def receive_health(self, msg):
        self.healthy, self.health_receipt = msg.data, time.monotonic()

    def receive_estop(self, msg):
        self.estopped = msg.data
        if msg.data and self.state in ('FOLLOWING', 'SETTLING'):
            self.halt('PAUSED', '急停已触发，解除后需 resume')

    def receive_teleop(self, msg):
        if abs(msg.linear.x) > .001 or abs(msg.angular.z) > .001:
            self.last_teleop = time.monotonic()
            if self.state in ('FOLLOWING', 'SETTLING'):
                self.halt('PAUSED', '手动控制已接管，返回路线后需 resume')

    def fresh(self, msg, receipt, timeout):
        if msg is None or time.monotonic()-receipt > timeout:
            return False
        age = self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp)
        return -.1 <= age <= timeout

    def ready(self, scan=True):
        if self.estopped:
            raise ValueError('急停未解除')
        if not self.healthy or time.monotonic()-self.health_receipt > self.health_timeout:
            raise ValueError('NDT 定位尚未收敛或已失效')
        if not self.fresh(self.pose_msg, self.pose_receipt, self.pose_timeout):
            raise ValueError('定位位姿过期或时钟不一致')
        if not self.fresh(self.odom, self.odom_receipt, self.pose_timeout):
            raise ValueError('里程计过期或时钟不一致')
        if scan and not self.fresh(self.scan, self.scan_receipt, self.scan_timeout):
            raise ValueError('雷达扫描过期或时钟不一致')

    def make_tracker(self, points):
        return Tracker(motion_blocks(points, self.allow_reverse, self.max_segment_length),
                       self.min_turning_radius, self.max_cross_track, self.min_lookahead,
                       self.max_lookahead, self.lookahead_time, self.goal_tolerance, self.goal_yaw_tolerance)

    def reload(self):
        if self.state in ('RECORDING', 'FOLLOWING', 'SETTLING', 'PAUSED'):
            raise ValueError('请先 stop，再加载路线')
        points = load_csv(self.filename, legacy_spacing=self.legacy_spacing)
        tracker = self.make_tracker(points)
        self.points, self.tracker = points, tracker
        self.publish_path()
        return f'已加载 {len(points)} 个点、{len(tracker.blocks)} 个运动段'

    def start(self):
        if self.state in ('RECORDING', 'FOLLOWING', 'SETTLING', 'PAUSED'):
            raise ValueError('当前任务未结束；暂停任务请 resume，或先 stop')
        self.ready()
        if time.monotonic()-self.last_teleop < .6 or abs(self.odom.twist.twist.linear.x) > .03:
            raise ValueError('请先停止手动行驶并停稳')
        self.reload()
        first = self.points[0]
        tolerance = min(self.start_tolerance, self.tracker.block.length/4.)
        if distance(first, self.pose) > tolerance or abs(wrap(first.yaw-self.pose.yaw)) > self.start_yaw_tolerance:
            raise ValueError('请手动将车辆移到录制起点附近并对齐车头，再 start')
        # Validate scan TF/coverage before accepting an execution request.
        self.obstacle_points(self.tracker.block.direction)
        self.state, self.reason = 'FOLLOWING', '沿录制路线行驶'
        self.last_velocity, self.clear_since, self.settle_since = 0., None, None
        msg = Bool(); msg.data = False; self.reached_pub.publish(msg)
        return '固定路线已启动'

    def halt(self, state, reason):
        self.state, self.reason, self.last_velocity = state, reason, 0.
        self.stop_until = time.monotonic()+.6
        self.settle_since = self.clear_since = None
        self.cmd_pub.publish(Twist())

    def pause(self):
        if self.state not in ('FOLLOWING', 'SETTLING'):
            raise ValueError('当前没有执行中的路线')
        self.halt('PAUSED', '用户暂停')
        return '已暂停'

    def resume(self):
        if self.state != 'PAUSED' or self.tracker is None:
            raise ValueError('当前没有暂停的路线')
        self.ready()
        if time.monotonic()-self.last_teleop < .6 or abs(self.odom.twist.twist.linear.x) > .03:
            raise ValueError('请先停止手动行驶并停稳')
        self.tracker.command(self.pose, 0., self.max_speed, self.deceleration)
        self.obstacle_points(self.tracker.block.direction)
        self.state, self.reason, self.last_velocity = 'FOLLOWING', '继续原路线', 0.
        return '已继续'

    def stop(self):
        if self.state == 'RECORDING':
            self.invalidate_record('用户取消录制；.partial 不作为可执行路线')
        self.halt('IDLE', '任务已停止')
        return '已停止'

    def start_recording(self):
        if self.state in ('FOLLOWING', 'SETTLING', 'PAUSED', 'RECORDING'):
            raise ValueError('请先停止当前任务')
        self.ready(scan=False)
        if self.filename.exists():
            raise ValueError(f'路线已存在，不会覆盖：{self.filename}；请选择新的 route_file')
        self.filename.parent.mkdir(parents=True, exist_ok=True)
        partial = self.filename.with_suffix(self.filename.suffix+'.partial')
        self.record_stream = partial.open('x', newline='', encoding='utf-8')
        self.record_writer = csv.writer(self.record_stream)
        self.record_writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'speed'])
        self.record_last, self.record_previous_pose, self.record_points = None, None, 0
        self.state, self.reason = 'RECORDING', '录制中，可使用现有遥控驾驶'
        self.record_sample(force=True)
        return f'开始录制：{self.filename}'

    def record_sample(self, force=False):
        self.ready(scan=False)
        current = self.pose
        if self.record_previous_pose is not None:
            old, old_stamp = self.record_previous_pose
            gap = stamp_seconds(self.pose_msg.header.stamp)-old_stamp
            if gap < 0. or gap > self.pose_timeout or distance(old, current) > .2+2*self.max_speed*max(gap, 0.):
                raise ValueError('录制位姿中断或跳变，请重新录制')
        self.record_previous_pose = (current, stamp_seconds(self.pose_msg.header.stamp))
        if self.record_last and distance(current, self.record_last) < .001:
            return
        if not force and self.record_last and distance(current, self.record_last) < self.record_spacing and abs(wrap(current.yaw-self.record_last.yaw)) < self.record_angle:
            return
        if self.record_points >= 200000:
            raise ValueError('录制超过 200000 点，请分段')
        speed = self.odom.twist.twist.linear.x
        self.record_writer.writerow([current.x, current.y, current.z, 0., 0., math.sin(current.yaw/2), math.cos(current.yaw/2), speed])
        self.record_stream.flush()
        self.record_last, self.record_points = current, self.record_points+1

    def invalidate_record(self, reason):
        if self.record_stream:
            self.record_stream.close()
        self.record_stream = self.record_writer = None
        self.state, self.reason = 'RECORD_INVALID', reason

    def stop_recording(self):
        if self.state != 'RECORDING':
            raise ValueError('当前不在录制中')
        try:
            self.record_sample(force=True)
            os.fsync(self.record_stream.fileno())
            self.record_stream.close()
            self.record_stream = self.record_writer = None
            partial = self.filename.with_suffix(self.filename.suffix+'.partial')
            points = load_csv(partial)
            self.make_tracker(points)
            # Exclusive final publication; even another process cannot overwrite an existing route.
            os.link(partial, self.filename)
            partial.unlink()
            self.state, self.reason = 'IDLE', f'已保存 {len(points)} 点'
            self.reload()
            return f'已保存 {len(points)} 点：{self.filename}'
        except (ValueError, OSError) as exc:
            self.invalidate_record(str(exc))
            raise

    def obstacle_points(self, direction):
        scan = self.scan
        if (scan is None or not scan.header.frame_id or not scan.ranges or
                not all(math.isfinite(x) for x in (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)) or
                scan.angle_increment <= 0 or scan.range_min < 0 or scan.range_max <= scan.range_min):
            raise ValueError('LaserScan 格式无效')
        # Motion-compensate from scan time to the current odometry sample time.
        try:
            transform = self.tf_buffer.lookup_transform_full(
                self.base_frame, Time.from_msg(self.odom.header.stamp),
                scan.header.frame_id, Time.from_msg(scan.header.stamp), self.odom_frame)
            t, q = transform.transform.translation, transform.transform.rotation
            yaw = quaternion_yaw([q.x, q.y, q.z, q.w])
        except Exception as exc:
            raise ValueError(f'扫描坐标变换尚未就绪：{exc}') from exc
        center = wrap((0. if direction > 0 else math.pi)-yaw)
        upper = scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment
        def observed(angle):
            return any(scan.angle_min-.01 <= angle+k*2*math.pi <= upper+.01 for k in (-1, 0, 1))
        if not all(observed(center+offset) for offset in (-.65, -.3, 0., .3, .65)):
            raise ValueError('扫描未覆盖行驶方向；当前 315° 扫描不支持后方盲区内倒车')
        points, valid = [], 0
        c, s = math.cos(yaw), math.sin(yaw)
        for i, radius in enumerate(scan.ranges):
            angle = scan.angle_min+i*scan.angle_increment
            observed_range = radius == math.inf or (math.isfinite(radius) and scan.range_min <= radius <= scan.range_max)
            if observed_range:
                valid += 1
            elif abs(wrap(angle-center)) <= .65:
                raise ValueError('行驶方向存在无效扫描量测')
            if not math.isfinite(radius) or not scan.range_min <= radius <= scan.range_max:
                continue
            x, y = radius*math.cos(angle), radius*math.sin(angle)
            points.append((c*x-s*y+t.x, s*x+c*y+t.y))
        if valid < len(scan.ranges)*.5:
            raise ValueError('扫描中无效量测过多')
        return points

    def tick(self):
        current_time, steady = self.get_clock().now().nanoseconds*1e-9, time.monotonic()
        dt = min(.1, max(.001, steady-self.last_tick))
        self.last_tick = steady
        if current_time < self.last_ros_time-.05:
            if self.state == 'RECORDING':
                self.invalidate_record('ROS 时钟回跳，请重新录制')
            elif self.state in ('FOLLOWING', 'SETTLING'):
                self.halt('PAUSED', 'ROS 时钟回跳，请重新定位')
            self.healthy, self.pose, self.pose_msg = False, None, None
        self.last_ros_time = current_time
        if self.state == 'RECORDING':
            try:
                self.record_sample()
            except (ValueError, OSError) as exc:
                self.invalidate_record(str(exc))
            return
        if self.state not in ('FOLLOWING', 'SETTLING'):
            if steady < self.stop_until:
                self.cmd_pub.publish(Twist())
            return
        try:
            self.ready()
            measured = self.odom.twist.twist.linear.x
            if self.state == 'SETTLING':
                self.cmd_pub.publish(Twist())
                self.settle_since = (self.settle_since or steady) if abs(measured) <= .03 else None
                if self.settle_since is not None and steady-self.settle_since >= self.stop_settle_time:
                    if self.tracker.advance():
                        self.state, self.reason, self.last_velocity = 'FOLLOWING', '停车换向后继续', 0.
                        self.settle_since = None
                    else:
                        self.halt('COMPLETED', '已走完全部录制路线')
                        msg = Bool(); msg.data = True; self.reached_pub.publish(msg)
                return
            direction = self.tracker.block.direction
            limit = self.max_speed if direction > 0 else min(self.max_speed, self.max_reverse_speed)
            velocity, angular, reached = self.tracker.command(self.pose, measured, limit, self.deceleration, dt)
            if reached:
                self.halt('SETTLING', '端点停车确认中')
                return
            curvature = angular/velocity if abs(velocity) > 1e-6 else 0.
            points = self.obstacle_points(direction)
            # Include actual measured speed in the stopping-distance envelope.
            check_speed = direction*max(abs(velocity), abs(measured), abs(self.last_velocity))
            if swept_collision(points, self.polygon, check_speed, curvature, self.deceleration, self.obstacle_margin):
                self.reason, self.clear_since, self.last_velocity = '前方障碍，停车等待原路线恢复', None, 0.
                self.cmd_pub.publish(Twist())
                return
            if self.clear_since is None:
                self.clear_since = steady
            if steady-self.clear_since < self.obstacle_clear_time:
                self.cmd_pub.publish(Twist())
                return
            self.reason = '沿录制路线行驶'
            step = (self.acceleration if abs(velocity) > abs(self.last_velocity) else self.deceleration)*dt
            velocity = self.last_velocity+max(-step, min(step, velocity-self.last_velocity))
            self.last_velocity = velocity
            command = Twist(); command.linear.x, command.angular.z = velocity, velocity*curvature
            self.cmd_pub.publish(command)
            target = PoseStamped(); target.header = self.pose_msg.header
            target.pose.position.x, target.pose.position.y = self.tracker.target.x, self.tracker.target.y
            target.pose.orientation.w = 1.; self.target_pub.publish(target)
        except ValueError as exc:
            self.halt('PAUSED', str(exc))

    def publish_path(self):
        path = PathMsg(); path.header.frame_id = self.map_frame; path.header.stamp = self.get_clock().now().to_msg()
        for p in self.points:
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x, p.y, p.z
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(p.yaw/2), math.cos(p.yaw/2)
            path.poses.append(pose)
        self.path_pub.publish(path)

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state, 'reason': self.reason, 'route_file': str(self.filename),
            'recorded_points': self.record_points,
            'block': self.tracker.index if self.tracker else 0,
            'progress_m': self.tracker.progress if self.tracker else 0.,
            'total_blocks': len(self.tracker.blocks) if self.tracker else 0,
        }, ensure_ascii=False)
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FixedRoute()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.record_stream:
                node.record_stream.close()
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
