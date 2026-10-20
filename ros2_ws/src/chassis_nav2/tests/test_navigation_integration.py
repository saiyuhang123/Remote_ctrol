"""Offline regression checks; run after building, without connecting hardware."""
import importlib.util
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from PIL import Image
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from chassis_nav2.geometry import transform, inverse, quaternion, ackermann_command, ackermann_command_4ws
from chassis_nav2.final_approach_logic import FinalApproachLogic
from chassis_nav2.wheel_odometry import WheelOdometry
from chassis_nav2 import bringup

SRC = Path(__file__).resolve().parents[2]
ROOT = SRC.parents[1]


def test_recorded_routes_are_opt_in_and_have_independent_plugins():
    from chassis_nav2.recorded_routes import validate_options,nav2_overlay
    assert not validate_options({})
    for args in ({'enable_recorded_routes':'true'}, {'enable_recorded_routes':'true','enable_patrol':'false'}):
        with __import__('pytest').raises(ValueError):validate_options(args)
    assert validate_options({'enable_recorded_routes':'true','enable_patrol':'true'})
    hw=yaml.safe_load((SRC/'chassis_nav2/config/hardware.yaml').read_text())
    for name in ('nav2.yaml','nav2_2d_localization.yaml','nav2_carto_localization.yaml'):
        path=SRC/'chassis_nav2/config'/name
        original=yaml.safe_load(path.read_text())
        assert original['controller_server']['ros__parameters']['controller_plugins']==['FollowPath']
        controller=nav2_overlay('controller_server',path,hw)
        assert controller['controller_plugins']==['FollowPath','RecordedRoute','RecordedEntryRoute']
        assert controller['RecordedRoute.allow_reversing'] and not controller['RecordedRoute.use_rotate_to_heading']
        planners=nav2_overlay('planner_server',path,hw)
        assert planners['planner_plugins']==['GridBased','RecordedDetour','RecordedEntry']
        assert planners['RecordedEntry.plugin']=='recorded_route_nav2::RecordedEntryPlanner'
        assert planners['RecordedDetour.motion_model_for_search']=='DUBIN'
        assert planners['RecordedEntry.minimum_turning_radius']>=.69
        assert controller['recorded_entry_checker.xy_goal_tolerance']==.03
        assert controller['recorded_entry_checker.yaw_goal_tolerance']==.08
        assert controller['recorded_entry_checker.controller_id']=='RecordedEntryRoute'
        assert controller['RecordedEntryRoute.max_lookahead_dist']<controller['RecordedRoute.max_lookahead_dist']
        assert controller['RecordedEntryRoute.desired_linear_vel']==.20
        assert controller['RecordedEntryRoute.min_approach_linear_velocity']==.05
        assert controller['RecordedEntryRoute.approach_velocity_scaling_dist']==.12
        assert controller['RecordedEntryRoute.regulated_linear_scaling_min_radius']==planners['RecordedEntry.minimum_turning_radius']
        progress=original['controller_server']['ros__parameters']['progress_checker_plugin']
        assert controller[progress+'.plugin']=='recorded_route_nav2::EntryProgressChecker'


def test_desktop_recorded_markers_preserve_all_navigation_displays():
    for name in ('navigation','navigation_fastlio','navigation_amcl'):
        document=yaml.safe_load((ROOT/'desktop_ui/config/rviz'/f'{name}.rviz').read_text())
        manager=document['Visualization Manager'];assert manager['Enabled']
        displays={display['Name']:display for display in manager['Displays']}
        assert displays['栅格地图']['Enabled']
        assert displays['录制路网']['Topic']['Durability Policy']=='Transient Local'
        assert displays['固定路线计划']['Enabled'] and displays['局部绕行']['Enabled']
        if name=='navigation_fastlio':assert displays['全局点云地图']['Enabled']


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(matrix, ns=1000000000):
    msg = Odometry()
    msg.header.stamp = Time(sec=ns//1000000000, nanosec=ns % 1000000000)
    msg.header.frame_id, msg.child_frame_id = 'lio_odom', 'imu_link'
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    p.x, p.y, p.z = map(float, matrix[:3, 3])
    q.x, q.y, q.z, q.w = quaternion(matrix)
    return msg


class Capture:
    def __init__(self):
        self.messages = []
    def publish(self, msg):
        self.messages.append(msg)
    def sendTransform(self, msg):
        self.messages.append(msg)


class IntegrationTests(unittest.TestCase):
    def hardware(self):
        hw = yaml.safe_load((SRC/'chassis_nav2/config/hardware.yaml').read_text())
        hw['sensor_calibration'].update(confirmed=True,
            base_from_lidar={'translation': [.4, -.1, .7], 'rpy': [0., 0., .2]},
            base_from_imu={'translation': [.1, .2, .3], 'rpy': [.1, -.1, .3]})
        hw['robot_geometry'].update(confirmed=True, wheel_base=.5, max_steering_deg=27.,
                                    footprint=[[.5, .36], [.5, -.36], [-.5, -.36], [-.5, .36]])
        hw['chassis']['confirmed'] = True
        hw['wheel_odometry'].update(confirmed=True, rear_axle_translation=[-.25, 0.])
        return hw

    def test_new_mounting_requires_calibration_and_composes_correctly(self):
        fresh = yaml.safe_load((SRC/'chassis_nav2/config/hardware.yaml').read_text())
        fresh['sensor_calibration']['confirmed'] = False
        with self.assertRaises(ValueError):
            bringup.calibrations(fresh)
        base_lidar, base_imu = bringup.calibrations(self.hardware())
        imu_lidar = inverse(base_imu) @ base_lidar
        point = np.array([2., -1., .4, 1.])
        np.testing.assert_allclose(base_imu @ imu_lidar @ point, base_lidar @ point, atol=1e-12)
        np.testing.assert_allclose(base_imu @ inverse(base_imu), np.eye(4), atol=1e-12)
        for angle in [math.pi, -math.pi, 1.5]:
            m = transform([0., 0., 0.], [angle, 0., 0.])
            np.testing.assert_allclose(np.linalg.norm(quaternion(m)), 1., atol=1e-12)

    def test_jd03_units_reverse_and_no_in_place_rotation(self):
        motor, steer, gear, brake = ackermann_command(.2, .1, .5, 27., .3)
        self.assertAlmostEqual(motor, .72)
        self.assertLess(steer, 0.)
        self.assertEqual((gear, brake), (1, False))
        reverse = ackermann_command(-.2, .1, .5, 27., .3)
        self.assertGreater(reverse[1], 0.)
        self.assertEqual(reverse[2], 3)
        self.assertEqual(ackermann_command(0., .5, .5, 27., .3), (0., 0., 1, True))
        self.assertTrue(ackermann_command(float('nan'), 0., .5, 27., .3)[3])
        limited = ackermann_command(2., 10., .5, 27., .3)
        self.assertAlmostEqual(limited[0], 1.08)
        self.assertEqual(abs(limited[1]), 27.)

    def test_jd03_four_wheel_steering_command(self):
        motor, front, rear, gear, brake = ackermann_command_4ws(.2, .1, .5, 27., .3)
        self.assertAlmostEqual(motor, .72)
        # Counter-phase rear steering: half the front angle, rear mirrored.
        self.assertAlmostEqual(front, -math.degrees(math.atan(.125)))
        self.assertAlmostEqual(rear, -front)
        self.assertEqual((gear, brake), (1, False))
        reverse = ackermann_command_4ws(-.2, .1, .5, 27., .3)
        self.assertEqual(reverse[3], 3)
        self.assertEqual(ackermann_command_4ws(0., .5, .5, 27., .3), (0., 0., 0., 1, True))
        self.assertTrue(ackermann_command_4ws(float('nan'), 0., .5, 27., .3)[4])
        limited = ackermann_command_4ws(2., 10., .5, 27., .3)
        self.assertEqual(abs(limited[1]), 27.)
        self.assertEqual(abs(limited[2]), 27.)
        crab = ackermann_command_4ws(.2, .1, .5, 27., .3, rear_steering_sign=1.)
        self.assertAlmostEqual(crab[2], crab[1])

    def test_lio_body_offset_velocity_and_duplicate_stamp(self):
        cls = load(SRC/'chassis_nav2/scripts/lio_odometry.py').LioOdometry
        node = cls.__new__(cls)
        base_body = transform([.2, -.1, .3], [0., 0., .2])
        node.body_from_base, node.previous = inverse(base_body), None
        node.odom_frame, node.comparison = 'odom', None
        node.publisher, node.tf = Capture(), Capture()
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        base0 = transform([1., 2., .1], [0., 0., .4])
        node.receive(pose(base0 @ base_body))
        out = node.publisher.messages[-1]
        np.testing.assert_allclose([out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z], base0[:3, 3])
        node.receive(pose(base0 @ base_body))
        self.assertEqual(len(node.publisher.messages), 1)
        base1 = base0.copy()
        base1[:3, 3] += base0[:3, :3] @ [.02, 0., 0.]
        node.receive(pose(base1 @ base_body, 1100000000))
        self.assertAlmostEqual(node.publisher.messages[-1].twist.twist.linear.x, .2)
        self.assertEqual(node.tf.messages[-1].child_frame_id, 'base_link')

    def test_jd03_stop_is_immediate_and_latched(self):
        cls = load(SRC/'chassis_nav2/scripts/jd03_bridge.py').JD03Bridge
        node = cls.__new__(cls)
        node.wheel_base, node.max_steering_deg, node.max_speed = .5, 27., .3
        node.command_steering_sign, node.cmd_timeout, node.estopped = -1., .5, False
        node.four_wheel_steering, node.rear_steering_sign = False, -1.
        node.steering_rate_dps, node.last_steer, node.last_rear = 45., 0., 0.
        node.ecu_pub, node.feedback_time = Capture(), time.monotonic()
        node.command = time.monotonic(), .2, .1
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        node.send()
        self.assertFalse(node.ecu_pub.messages[-1].brake)
        node.emergency_stop(Bool(data=True))
        self.assertTrue(node.ecu_pub.messages[-1].brake)
        node.receive(Twist())
        node.send()
        self.assertTrue(node.ecu_pub.messages[-1].brake)
        node.estopped = False
        node.command, node.feedback_time = (time.monotonic()-2, .2, 0.), time.monotonic()
        node.send()
        self.assertTrue(node.ecu_pub.messages[-1].brake)

    def test_jd03_steering_slew_rate_limits_swing(self):
        # 低速时转角对 w 抖动敏感；速率限制后舵机只能平滑逼近，不能来回抽
        cls = load(SRC/'chassis_nav2/scripts/jd03_bridge.py').JD03Bridge
        node = cls.__new__(cls)
        node.wheel_base, node.max_steering_deg, node.max_speed = .5, 27., .5
        node.command_steering_sign, node.cmd_timeout, node.estopped = -1., .5, False
        node.four_wheel_steering, node.rear_steering_sign = False, -1.
        node.steering_rate_dps, node.last_steer, node.last_rear = 45., 0., 0.
        node.ecu_pub, node.feedback_time = Capture(), time.monotonic()
        node.command = time.monotonic(), .05, 1.0  # 低速大转角：期望打满 27°
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        node.send()
        first = node.ecu_pub.messages[-1].steer
        node.send()
        second = node.ecu_pub.messages[-1].steer
        # 每周期(0.05s)最多转 45*0.05=2.25°，两次发送增量都应被钳在 2.25 内
        self.assertLessEqual(abs(first), 2.25 + 1e-9)
        self.assertLessEqual(abs(second - first), 2.25 + 1e-9)
        # 反向指令同样被限速，而不是瞬时翻满
        node.command = time.monotonic(), .05, -1.0
        node.send()
        third = node.ecu_pub.messages[-1].steer
        self.assertLessEqual(abs(third - second), 2.25 + 1e-9)

    def test_wheel_odometry_straight_reverse_and_exact_turn(self):
        straight = WheelOdometry(.5, [0., 0.])
        reverse = WheelOdometry(.5, [0., 0.])
        turning = WheelOdometry(.5, [0., 0.])
        for i in range(6):
            a = straight.update(1+i*.2, 1., 0., 1)
            b = reverse.update(1+i*.2, 1., 0., 3)
            c = turning.update(1+i*.2, 1., math.atan(.5), 1)
        np.testing.assert_allclose(a[0], [1., 0., 0.], atol=1e-12)
        np.testing.assert_allclose(b[0], [-1., 0., 0.], atol=1e-12)
        np.testing.assert_allclose(c[0], [math.sin(1.), 1-math.cos(1.), 1.], atol=1e-12)
        self.assertGreater(c[2][0, 0], 0.)
        np.testing.assert_allclose(c[2], c[2].T, atol=1e-12)
        self.assertGreaterEqual(np.linalg.eigvalsh(c[2]).min(), -1e-12)

    def test_wheel_odometry_base_lever_arm_and_lateral_velocity(self):
        rear = np.array([-.25, .1])
        model = WheelOdometry(.5, rear)
        first = model.update(1., 1., math.atan(.5), 1)
        np.testing.assert_allclose(first[0], [0., 0., 0.], atol=1e-12)
        for i in range(1, 6):
            sample = model.update(1+i*.2, 1., math.atan(.5), 1)
        rotation = transform([0., 0., 0.], [0., 0., 1.])[:2, :2]
        wanted = rear + [math.sin(1.), 1-math.cos(1.)] - rotation @ rear
        np.testing.assert_allclose(sample[0][:2], wanted, atol=1e-12)
        np.testing.assert_allclose(sample[1], [1.1, .25, 1.], atol=1e-12)

    def test_wheel_odometry_four_wheel_steering_doubles_yaw_rate(self):
        model = WheelOdometry(.5, [0., 0.], four_wheel_steering=True)
        for i in range(6):
            sample = model.update(1+i*.2, 1., math.atan(.5), 1)
        # Yaw rate doubles and the zero-slip reference moves to mid-wheelbase.
        yaw = 2.
        wanted = [.25*(1-math.cos(yaw))+.5*math.sin(yaw),
                  .5*(1-math.cos(yaw))-.25*math.sin(yaw), yaw]
        np.testing.assert_allclose(sample[0], wanted, atol=1e-12)
        np.testing.assert_allclose(sample[1], [1., -.5, 2.], atol=1e-12)
        measured = WheelOdometry(.5, [0., 0.], four_wheel_steering=True, four_wheel_yaw_gain=1.)
        for i in range(6):
            sample = measured.update(1+i*.2, 1., math.atan(.5), 1)
        np.testing.assert_allclose(sample[0][2], 1., atol=1e-12)

    def test_reeds_shepp_paths_reach_goal_exactly(self):
        # 对每条生成的路径词做运动学积分，终点必须与目标位姿一致
        from chassis_nav2.reeds_shepp import generate_paths
        import random
        random.seed(7)
        cases = [(0., .4, 0.), (-.5, 0., 0.), (0., 2.01, -2.77), (.3, -.2, 1.2)]
        cases += [(random.uniform(-2, 2), random.uniform(-2, 2),
                   random.uniform(-math.pi, math.pi)) for _ in range(40)]
        for dx, dy, dyaw in cases:
            paths = generate_paths(dx, dy, dyaw, 1.45)
            self.assertTrue(paths, f'无路径：{(dx, dy, dyaw)}')
            for lengths, ctypes in paths:
                x = y = yaw = 0.
                for length, ctype in zip(lengths, ctypes):
                    dist = length / 1.45
                    if ctype == 'S':
                        x += dist * math.cos(yaw)
                        y += dist * math.sin(yaw)
                    else:
                        kappa = 1.45 if ctype == 'L' else -1.45
                        turn = kappa * dist
                        x += (math.sin(yaw + turn) - math.sin(yaw)) / kappa
                        y += (math.cos(yaw) - math.cos(yaw + turn)) / kappa
                        yaw += turn
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                self.assertAlmostEqual(x, dx, places=6, msg=f'{ctypes}{lengths}')
                self.assertAlmostEqual(y, dy, places=6, msg=f'{ctypes}{lengths}')
                self.assertAlmostEqual(yaw, math.atan2(math.sin(dyaw), math.cos(dyaw)),
                                       places=6, msg=f'{ctypes}{lengths}')

    def test_final_approach_rs_first_segment_and_done(self):
        logic = FinalApproachLogic(speed=.1, max_curvature=1.45, settle_time=.2)
        v, w, _ = logic.update(1., 0., 0., .05)
        self.assertGreater(v, 0.)      # 正前方：直行
        self.assertAlmostEqual(w, 0.)
        v, w, _ = logic.update(-.5, 0., 0., .05)
        self.assertLess(v, 0.)         # 正后方：倒车
        self.assertAlmostEqual(w, 0.)
        v, w, _ = logic.update(0., .4, 0., .05)
        self.assertLessEqual(abs(w / v), 1.45)  # 侧移：首段曲率必可行
        self.assertGreater(abs(w), 0.)
        self.assertIsNotNone(logic.path_length)
        for _ in range(3):
            v, w, status = logic.update(.01, 0., .01, .05)
        self.assertEqual((v, w), (0., 0.))
        self.assertEqual(status, FinalApproachLogic.RUNNING)
        _, _, status = logic.update(.01, 0., .01, .05)
        self.assertEqual(status, FinalApproachLogic.DONE)
        timeout = FinalApproachLogic(timeout=.2)
        _, _, status = timeout.update(1., 0., 0., .3)
        self.assertEqual(status, FinalApproachLogic.FAILED)

    def test_final_approach_fallback_pursuit_respects_curvature(self):
        logic = FinalApproachLogic(speed=.1, max_curvature=.9)
        v, w, _ = logic._approach(0., .23, .18)
        self.assertLessEqual(abs(w / v), .8 * .9 + 1e-9)  # 自适应过冲压曲率到 80% 内
        self.assertGreater(v, 0.)
        v, w, _ = logic._approach(-.5, 0., 0.)
        self.assertLess(v, 0.)

    def test_wheel_odometry_drops_bad_samples_and_does_not_invent_missing_motion(self):
        model = WheelOdometry(.5, [0., 0.])
        self.assertIsNone(model.update(1., 1., 0., 2))  # Unknown coasting direction.
        model.update(1., 1., 0., 1)
        sample = model.update(1.1, 1., 0., 2)
        self.assertAlmostEqual(sample[1][0], 1.)  # Neutral can coast.
        for t, v, a, gear in [(1.1, 1., 0., 1), (1., 1., 0., 1),
                              (1.2, float('nan'), 0., 1), (1.2, 1., math.pi/2, 1),
                              (1.2, 1., 0., 0)]:
            self.assertIsNone(model.update(t, v, a, gear))
        gap = model.update(3., 1., 0., 1)
        np.testing.assert_allclose(gap[0], sample[0], atol=1e-12)
        self.assertTrue(gap[-1])
        self.assertGreater(gap[2][0, 0], sample[2][0, 0])

    def test_wheel_odometry_yaw_rate_variance_never_collapses_to_zero(self):
        # 停车且方向盘回正时，模型推算的角速度方差曾退化为精确的 0，
        # 零方差测量会使下游 EKF 协方差奇异(NaN)，必须有正下限。
        model = WheelOdometry(.5, [0., 0.])
        model.update(1., 1., 0., 1)
        sample = model.update(1.1, 0., 0., 1)
        self.assertAlmostEqual(sample[1][2], 0.)
        self.assertGreaterEqual(sample[3][2, 2], .01**2)
        custom = WheelOdometry(.5, [0., 0.], min_yaw_rate_stddev=.05)
        custom.update(1., 1., 0., 1)
        sample = custom.update(1.1, 0., 0., 1)
        self.assertGreaterEqual(sample[3][2, 2], .05**2)
        moving = WheelOdometry(.5, [0., 0.])
        moving.update(1., 1., math.atan(.5), 1)
        sample = moving.update(1.1, 1., math.atan(.5), 1)
        self.assertGreater(sample[3][2, 2], .01**2)

    def test_wheel_ros_message_uses_measured_speed_even_when_braking(self):
        from yunle_msgs.msg import VehicleStatus
        cls = load(SRC/'chassis_nav2/scripts/jd03_odometry.py').JD03Odometry
        node = cls.__new__(cls)
        node.stamp_source, node.feedback_steering_sign = 'header', -1.
        node.speed_scale, node.steering_scale, node.steering_bias_deg = 1., 1., 0.
        node.odom_frame, node.base_frame = 'odom', 'base_link'
        node.publisher, node.comparison, node.tf = Capture(), Capture(), None
        node.integrator = WheelOdometry(.5, [0., 0.])
        node.get_logger = lambda: SimpleNamespace(warning=lambda *a: None)
        feedback = VehicleStatus(cur_speed=.2, cur_steer=0., shift_level=1, p_status=True)
        feedback.header.stamp = Time(sec=1)
        node.receive(feedback)
        feedback.header.stamp = Time(sec=1, nanosec=100000000)
        node.receive(feedback)
        output = node.publisher.messages[-1]
        self.assertAlmostEqual(output.pose.pose.position.x, .02, places=6)
        self.assertAlmostEqual(output.twist.twist.linear.x, .2, places=6)
        self.assertEqual(output.header.frame_id, 'odom')
        self.assertEqual(node.comparison.messages[-1].header.frame_id, 'wheel_odom')
        node.receive(feedback)
        self.assertEqual(len(node.publisher.messages), 2)
        self.assertEqual(output.pose.covariance[14], 1e3)

    def test_standalone_wheel_entry_needs_no_sensor_calibration_and_defaults_to_no_tf(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'hardware.yaml'
            hw = self.hardware()
            hw['sensor_calibration']['confirmed'] = False
            hw['robot_geometry']['confirmed'] = False
            hw['robot_geometry']['footprint'] = None
            path.write_text(yaml.safe_dump(hw))
            args = {'hardware_file':str(path), 'start_chassis':'false', 'use_as_odom':'false'}
            with patch.object(bringup, 'Node', side_effect=lambda **kwargs: kwargs):
                nodes = bringup.configure_chassis_odometry(SimpleNamespace(launch_configurations=args))
                self.assertEqual(len(nodes), 1)
                params = nodes[0]['parameters'][0]
                self.assertEqual(params['odom_topic'], '/odom_wheel')
                self.assertIs(params['publish_tf'], False)
                args['use_as_odom'] = 'true'
                params = bringup.configure_chassis_odometry(SimpleNamespace(launch_configurations=args))[0]['parameters'][0]
                self.assertEqual(params['odom_topic'], '/odom')
                self.assertIs(params['publish_tf'], True)
                hw['wheel_odometry']['confirmed'] = False
                path.write_text(yaml.safe_dump(hw))
                with self.assertRaises(ValueError):
                    bringup.configure_chassis_odometry(SimpleNamespace(launch_configurations=args))

    def test_gateway_estop_gate_and_cancels_late_owned_task_acceptance(self):
        # The gateway now routes through robot_patrol instead of the removed
        # send_nav_goal API. Test its current ownership/cancellation boundary.
        cls = load(SRC/'robot_gateway/robot_gateway/gateway_node.py').GatewayNode
        node = cls.__new__(cls)
        node._lock, node._estopped = threading.Lock(), True
        node._estop_time, node._estop_timeout = time.monotonic(), 2.0
        self.assertEqual(node._estop_check()[1], 'ESTOPPED')
        node._estopped = None
        self.assertEqual(node._estop_check()[1], 'ESTOP_UNKNOWN')
        node._estopped = False
        self.assertTrue(node._estop_check()[0])
        node._task_lock = threading.Lock()
        node._task_kind, node._task_request_id = '', ''
        canceled = []
        handle = SimpleNamespace(accepted=True, cancel_goal_async=lambda: canceled.append(True))
        node._on_task_goal_response('route', 'old-request', None, SimpleNamespace(result=lambda: handle))
        self.assertEqual(canceled, [True])

    def test_fusion_waits_for_icp_and_corrects_imu_offset(self):
        cls = load(SRC/'FAST_LIO_LOCALIZATION_ROS2/scripts/transform_fusion.py').TransformFusionNode
        node = cls.__new__(cls)
        node.lock, node.tf_broadcaster, node.pub_localization = threading.Lock(), Capture(), Capture()
        node.base_from_body = transform([.2, 0., .4], [0., 0., .1])
        lio_body = transform([1., 2., 0.], [0., 0., .3])
        node.cur_odom_fastlio, node.cur_map_to_odom_fl = pose(lio_body), None
        node.timer_callback()
        self.assertFalse(node.tf_broadcaster.messages)
        map_lio = transform([3., -2., .1], [0., 0., -.2])
        ekf_base = transform([.4, -.3, .2], [0., 0., .1])
        tf = SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(
            x=.4, y=-.3, z=.2), rotation=SimpleNamespace(**dict(zip(('x','y','z','w'), quaternion(ekf_base))))))
        node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: tf)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        node.broadcast_lio_frame = True
        node.odom_source = 'wheel_imu'
        node.cur_map_to_odom_fl = pose(map_lio)
        node.timer_callback()
        wanted = map_lio @ lio_body @ inverse(node.base_from_body) @ inverse(ekf_base)
        actual = node.tf_to_mat(node.tf_broadcaster.messages[0])
        np.testing.assert_allclose(actual, wanted, atol=1e-12)
        self.assertEqual(node.tf_broadcaster.messages[1].child_frame_id, 'lio_odom')
        self.assertEqual(node.pub_localization.messages[-1].child_frame_id, 'base_link')

    def test_lio_fusion_never_injects_historical_tf_motion(self):
        cls = load(SRC/'FAST_LIO_LOCALIZATION_ROS2/scripts/transform_fusion.py').TransformFusionNode
        node = cls.__new__(cls)
        node.lock, node.tf_broadcaster, node.pub_localization = threading.Lock(), Capture(), Capture()
        node.odom_source, node.broadcast_lio_frame = 'lio', False
        node.base_from_body = transform([.32, 0., .22], [0., 0., .08])
        map_lio = transform([.3, -.2, 0.], [0., 0., .85])
        node.cur_map_to_odom_fl = pose(map_lio)
        node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: self.fail('LIO fusion must not query delayed odometry TF'))
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        # Acceleration, sharp steering, braking and residence, with a
        # realistic nonzero IMU offset. ICP is fixed for the whole replay.
        for i, (x,y,yaw) in enumerate([(0.,0.,0.),(.06,0.,0.),(.12,-.01,-.12),
                                     (.16,-.04,-.30),(.18,-.06,-.35),(.18,-.06,-.35)]):
            body = transform([x,y,0.], [0.,0.,yaw])
            node.cur_odom_fastlio = pose(body, 1000000000+i*100000000)
            node.timer_callback()
            actual_map_odom = node.tf_to_mat(node.tf_broadcaster.messages[-1])
            np.testing.assert_allclose(actual_map_odom,map_lio,atol=1e-12)
            np.testing.assert_allclose(actual_map_odom @ body @ inverse(node.base_from_body),
                                       node.pose_to_mat(node.pub_localization.messages[-1]),atol=1e-12)

    def test_wheel_fusion_without_matching_tf_does_not_invent_lio_origin(self):
        cls = load(SRC/'FAST_LIO_LOCALIZATION_ROS2/scripts/transform_fusion.py').TransformFusionNode
        node = cls.__new__(cls)
        node.lock,node.tf_broadcaster,node.pub_localization=threading.Lock(),Capture(),Capture()
        node.base_from_body=np.eye(4);node.odom_source='wheel';node._last_tf_warn=0.
        node.cur_odom_fastlio=node.cur_map_to_odom_fl=pose(np.eye(4))
        def missing(*args):raise RuntimeError('no wheel TF')
        node.tf_buffer=SimpleNamespace(lookup_transform=missing)
        node.get_logger=lambda:SimpleNamespace(warn=lambda _:None)
        node.timer_callback()
        self.assertFalse(node.tf_broadcaster.messages)

    def test_initial_pose_is_base_pose_not_lio_world_origin(self):
        cls = load(SRC/'FAST_LIO_LOCALIZATION_ROS2/scripts/global_localization.py').GlobalLocalizationNode
        node = cls.__new__(cls)
        node.global_map, node.cur_scan_odom, node.pending_initial_pose = None, None, None
        node.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
        node.initialized, node.freq_localization = True, 1.
        node.base_from_body = transform([.3, -.1, .4], [0., 0., .2])
        node.initial_base_z = -.4
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.position.x, initial.pose.pose.position.y = 10., -2.
        initial.pose.pose.orientation.w = 1.
        received = []
        node.global_localization = lambda guess: received.append(guess) or True
        node.cb_init_pose(initial)
        self.assertFalse(received)
        node.global_map = object()
        current = transform([2., 1., .1], [0., 0., .6])
        node.cur_scan_odom = pose(current)
        node._try_initial_pose()
        base_pose = transform([10., -2., -.4], [0., 0., 0.])
        np.testing.assert_allclose(received[0] @ current @ inverse(node.base_from_body), base_pose, atol=1e-12)

    def test_web_map_preserves_image_geometry(self):
        exporter = load(ROOT/'scripts/export_web_map.py')
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            Image.new('L', (3, 2), 205).save(p/'actual.pgm')
            cfg = {'image':'actual.pgm', 'resolution':.07, 'origin':[-2., 3., 0.]}
            (p/'map.yaml').write_text(yaml.safe_dump(cfg))
            result = exporter.export_web_map(p/'map.yaml', p/'web')
            self.assertEqual((result['width'], result['height']), (3, 2))
            self.assertEqual(result['origin'], cfg['origin'])
            self.assertEqual(result['resolution'], .07)
            with Image.open(p/'web/active.png') as img:
                self.assertEqual(img.size, (3, 2))
            cfg['origin'][2] = .1
            (p/'map.yaml').write_text(yaml.safe_dump(cfg))
            rotated = exporter.export_web_map(p/'map.yaml', p/'web')
            self.assertEqual(rotated['origin'], cfg['origin'])

    def test_imu_requires_new_samples_and_keeps_reception_time(self):
        module = load(SRC/'wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py')
        cls = module.imuDriverNode
        node = cls.__new__(cls)
        node.dataLock = threading.Lock()
        node.accData, node.gyroData = np.array([0., 0., 2048.]), np.zeros(3)
        node.angleData, node.magData = np.zeros(3), np.zeros(3)
        node.protocol, node.frame_id = module.protocolType.RS485_HIGH, 'imu_link'
        node.imuPublisher, node.imuRpyPublisher = Capture(), Capture()
        node._last_acc_update = node._last_gyro_update = node._last_angle_update = 0.
        node._last_published_gyro, node._gyro_stamp = 0., None
        with patch.object(module.time, 'monotonic', return_value=10.):
            node.publishImu()
            self.assertFalse(node.imuPublisher.messages)
            node._last_acc_update, node._last_gyro_update = 9.99, 9.98
            node._gyro_stamp = Time(sec=123, nanosec=456)
            node.publishImu()
            actual = node.imuPublisher.messages[-1]
            self.assertEqual(actual.header.stamp, node._gyro_stamp)
            self.assertAlmostEqual(actual.linear_acceleration.z, 9.80665)
            self.assertEqual(actual.orientation_covariance[0], -1.)
            node.publishImu()
            self.assertEqual(len(node.imuPublisher.messages), 1)
            node._last_acc_update, node._last_gyro_update = 9., 9.99
            node.publishImu()
            self.assertEqual(len(node.imuPublisher.messages), 1)

    def test_nav2_uses_measured_footprint_and_turning_radius(self):
        from launch import LaunchContext
        with patch.object(bringup, 'share', return_value=SRC/'chassis_nav2'), patch.object(
                bringup, 'Node', side_effect=lambda **kwargs: kwargs):
            hw = self.hardware()
            nodes = bringup.navigation_nodes('/tmp/test-only-map.yaml', hw,
                                             str(SRC/'chassis_nav2/config/nav2.yaml'))
            rewritten = nodes[0]['parameters'][0].perform(LaunchContext())
            params = yaml.safe_load(Path(rewritten).read_text())
            try:
                radius = params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
                measured = hw['chassis'].get('min_turning_radius')
                expected = float(measured) if measured is not None else .5/math.tan(math.radians(27.))
                self.assertAlmostEqual(radius, expected)
                costmap = params['local_costmap']['local_costmap']['ros__parameters']
                self.assertEqual(__import__('json').loads(costmap['footprint']), hw['robot_geometry']['footprint'])
                self.assertEqual(costmap['robot_base_frame'], 'base_link')
                self.assertIs(costmap['use_sim_time'], False)
                bt = Path(params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
                self.assertTrue(bt.is_file())
            finally:
                Path(rewritten).unlink()

    def test_bringup_has_one_tf_authority_and_no_2d_or_amcl(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p/'hardware.yaml').write_text(yaml.safe_dump(self.hardware()))
            (p/'site.pcd').write_text('test-only PCD placeholder; no runtime nodes start')
            (p/'site.yaml').write_text('image: site.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
            args = {'hardware_file':str(p/'hardware.yaml'), 'mode':'localization',
                    'start_sensors':'true', 'start_chassis':'false', 'enable_control':'false',
                    'odom_source':'lio', 'rviz':'false', 'pcd_map':str(p/'site.pcd'),
                    'lidar_config':str(SRC/'rslidar_sdk-v1.5.19/config/config.yaml'),
                    'nav2_params':str(SRC/'chassis_nav2/config/nav2.yaml'), 'map':str(p/'site.yaml')}
            paths = {'chassis_nav2':SRC/'chassis_nav2', 'chassis_description':SRC/'chassis_description',
                     'fast_lio_localization':SRC/'FAST_LIO_LOCALIZATION_ROS2'}
            def node(**kwargs):
                return kwargs
            with patch.object(bringup, 'share', side_effect=lambda name: paths[name]), patch.object(bringup, 'Node', side_effect=node):
                nodes = bringup.configure(SimpleNamespace(launch_configurations=args))
                packages = [n['package'] for n in nodes]
                self.assertNotIn('hi_ros2', packages)
                self.assertNotIn('nav2_amcl', packages)
                self.assertNotIn('robot_localization', packages)
                self.assertEqual(packages.count('rslidar_sdk'), 1)
                self.assertEqual(sum(n.get('executable') == 'lio_odometry.py' for n in nodes), 1)
                fusion=next(n for n in nodes if n.get('executable')=='transform_fusion.py')
                self.assertEqual(fusion['parameters'][1]['odom_source'],'lio')
                args['odom_source'] = 'wheel_imu'
                nodes = bringup.configure(SimpleNamespace(launch_configurations=args))
                self.assertEqual(sum(n['package'] == 'robot_localization' for n in nodes), 1)
                lio = next(n for n in nodes if n.get('executable') == 'lio_odometry.py')
                self.assertIs(lio['parameters'][0]['publish_tf'], False)
                self.assertEqual(lio['parameters'][0]['odom_topic'], '/odom_lio')
                fusion=next(n for n in nodes if n.get('executable')=='transform_fusion.py')
                self.assertEqual(fusion['parameters'][1]['odom_source'],'wheel_imu')
                args['odom_source'] = 'wheel'
                nodes = bringup.configure(SimpleNamespace(launch_configurations=args))
                self.assertFalse(any(n['package'] == 'robot_localization' for n in nodes))
                wheel = next(n for n in nodes if n.get('executable') == 'jd03_odometry.py')
                self.assertIs(wheel['parameters'][0]['publish_tf'], True)
                self.assertEqual(wheel['parameters'][0]['odom_topic'], '/odom')
                args['odom_source'], args['compare_wheel_odometry'] = 'lio', 'true'
                nodes = bringup.configure(SimpleNamespace(launch_configurations=args))
                wheel = next(n for n in nodes if n.get('executable') == 'jd03_odometry.py')
                self.assertIs(wheel['parameters'][0]['publish_tf'], False)
                self.assertEqual(wheel['parameters'][0]['odom_topic'], '/odom_wheel')
                args['odom_source'], args['mode'] = 'lio', 'navigation'
                nodes = bringup.configure(SimpleNamespace(launch_configurations=args))
                self.assertFalse(any(n['package'] == 'nav2_amcl' for n in nodes))
                smoother = next(n for n in nodes if n.get('executable') == 'velocity_smoother')
                self.assertIn(('cmd_vel_smoothed', '/cmd_vel_auto'), smoother['remappings'])
                args['enable_patrol'] = 'true'
                with patch.object(bringup, 'patrol_launch', return_value='PATROL_COMPANIONS') as companions:
                    interlocked = bringup.configure(SimpleNamespace(launch_configurations=args))
                    self.assertIn('PATROL_COMPANIONS', interlocked)
                    companions.assert_called_once_with()
                args['enable_patrol'] = 'false'
                (p/'site.pcd').write_bytes(b'')
                with self.assertRaisesRegex(ValueError, 'PCD 地图为空'):
                    bringup.configure(SimpleNamespace(launch_configurations=args))

    def test_slam_map_uses_amcl_without_serialized_posegraph(self):
        from chassis_nav2 import bringup_2d
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'hardware.yaml').write_text(yaml.safe_dump(self.hardware()))
            (root/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
            args = {'hardware_file': str(root/'hardware.yaml'), 'map': str(root/'map.yaml'),
                    'map_file': str(root/'no_posegraph_required'), 'start_sensors': 'false',
                    'start_chassis': 'false', 'enable_control': 'false', 'rviz': 'false',
                    'nav2_params': str(SRC/'chassis_nav2/config/nav2_2d_localization.yaml')}
            paths = {'chassis_nav2': SRC/'chassis_nav2', 'chassis_description': SRC/'chassis_description'}
            with patch.object(bringup, 'share', side_effect=lambda name: paths[name]), \
                    patch.object(bringup, 'Node', side_effect=lambda **kwargs: kwargs), \
                    patch.object(bringup_2d, 'Node', side_effect=lambda **kwargs: kwargs), \
                    patch.dict('os.environ', {'ROBOT_PROJECT_ROOT': str(root)}):
                nodes = bringup_2d.assemble(args, mapping=False, backend='slam_toolbox')
                self.assertEqual(sum(n['package'] == 'nav2_amcl' for n in nodes), 1)
                self.assertFalse(any(n['package'] in ('slam_toolbox', 'cartographer_ros',
                                                     'fast_lio_localization') for n in nodes))
                self.assertEqual(sum(n['package'] == 'robot_localization' for n in nodes), 1)
                amcl = next(n for n in nodes if n['package'] == 'nav2_amcl')
                params = yaml.safe_load(Path(amcl['parameters'][0]).read_text())['amcl']['ros__parameters']
                self.assertEqual((params['global_frame_id'], params['odom_frame_id'], params['base_frame_id']),
                                 ('map', 'odom', 'base_link'))
                self.assertIs(params['set_initial_pose'], False)
                self.assertIs(params['always_reset_initial_pose'], True)
                manager = next(n for n in nodes if n.get('name') == 'lifecycle_manager_localization')
                self.assertEqual(manager['parameters'][0]['node_names'], ['map_server', 'amcl'])
                mapping = bringup_2d.assemble(args, mapping=True, backend='slam_toolbox')
                self.assertEqual(sum(n.get('executable') == 'async_slam_toolbox_node' for n in mapping), 1)
                self.assertFalse(any(n['package'] in ('nav2_amcl', 'nav2_map_server') for n in mapping))
                args['enable_patrol'] = 'true'
                with patch.object(bringup, 'patrol_launch', return_value='PATROL_COMPANIONS') as companions:
                    interlocked = bringup_2d.assemble(args, mapping=False, backend='slam_toolbox')
                    self.assertIn('PATROL_COMPANIONS', interlocked)
                    companions.assert_called_once_with()
                args['enable_patrol'] = 'false'

    def test_cartographer_localization_keeps_its_own_state_and_lifecycle(self):
        from chassis_nav2 import bringup_2d
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'hardware.yaml').write_text(yaml.safe_dump(self.hardware()))
            (root/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
            (root/'map.pbstream').write_bytes(b'test-only state placeholder; no runtime nodes start')
            args = {'hardware_file': str(root/'hardware.yaml'), 'map': str(root/'map.yaml'),
                    'map_file': str(root/'map'), 'start_sensors': 'false', 'start_chassis': 'false',
                    'enable_control': 'false', 'rviz': 'false',
                    'nav2_params': str(SRC/'chassis_nav2/config/nav2_carto_localization.yaml')}
            paths = {'chassis_nav2': SRC/'chassis_nav2', 'chassis_description': SRC/'chassis_description'}
            with patch.object(bringup, 'share', side_effect=lambda name: paths[name]), \
                    patch.object(bringup, 'Node', side_effect=lambda **kwargs: kwargs), \
                    patch.object(bringup_2d, 'Node', side_effect=lambda **kwargs: kwargs):
                nodes = bringup_2d.assemble(args, mapping=False, backend='cartographer')
            self.assertFalse(any(n['package'] in ('nav2_amcl', 'slam_toolbox') for n in nodes))
            carto = next(n for n in nodes if n.get('executable') == 'cartographer_node')
            self.assertIn(str(root/'map.pbstream'), carto['arguments'])
            manager = next(n for n in nodes if n.get('name') == 'lifecycle_manager_map')
            self.assertEqual(manager['parameters'][0]['node_names'], ['map_server'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
