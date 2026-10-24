import math
import pytest
from fixed_route_navigation.route import (
    Point, Block, Tracker, load_csv, motion_blocks, swept_collision, wrap,
)


def test_legacy_csv_normalizes_quaternions_and_removes_stationary_duplicates(tmp_path):
    path = tmp_path/'lane_0.csv'
    path.write_text('0,0,-1.1,0,0,0,2\n0,0,-1.1,0,0,0,2\n.1,0,-1.1,0,0,0,2\n')
    points = load_csv(path)
    assert len(points) == 2
    assert points[0].yaw == 0 and points[0].z == -1.1


def test_header_csv_and_recorded_speed_limit(tmp_path):
    path = tmp_path/'route.csv'
    path.write_text('x,y,z,qx,qy,qz,qw,speed\n0,0,0,0,0,0,1,.12\n1,0,0,0,0,0,1,.12\n')
    tracker = Tracker(motion_blocks(load_csv(path)), .69)
    velocity, angular, reached = tracker.command(Point(0, 0), 0., .3, .4)
    assert velocity == .12 and angular == 0. and not reached


def test_coordinate_only_csv_infers_heading(tmp_path):
    path = tmp_path/'route.csv'
    path.write_text('x,y\n0,0\n0,.2\n0,.4\n')
    points = load_csv(path)
    assert all(p.yaw == pytest.approx(math.pi/2) for p in points)


def test_legacy_stationary_jitter_is_thinned_without_moving_the_endpoints(tmp_path):
    path = tmp_path/'legacy.csv'
    rows = [(0, 0), (-.005, .003), (.004, -.006), (.1, 0), (.104, .004), (.2, 0), (.205, .002)]
    path.write_text(''.join(f'{x},{y},0,0,0,0,1\n' for x, y in rows))
    points = load_csv(path)
    assert [(p.x, p.y) for p in points] == [(0., 0.), (.1, 0.), (.205, .002)]
    assert len(motion_blocks(points)) == 1


def test_native_signed_csv_preserves_short_reverse_boundary(tmp_path):
    path = tmp_path/'native.csv'
    path.write_text('x,y,z,qx,qy,qz,qw,speed\n0,0,0,0,0,0,1,.1\n.04,0,0,0,0,0,1,-.1\n0,0,0,0,0,0,1,0\n')
    points = load_csv(path)
    assert len(points) == 3
    blocks = motion_blocks(points, allow_reverse=True)
    assert [block.direction for block in blocks] == [1, -1]
    assert blocks[0].length == pytest.approx(.04)


@pytest.mark.parametrize('data', [
    '0,0,0,0,0,0,0\n.1,0,0,0,0,0,1\n',
    '0,0\nnan,1\n', '0,0\nbad,data\n', '0,0\n0,0\n',
    'x,y,speed\n0,0,.1\n1,0\n', '0,0,0,0\n1,1,1,1\n',
])
def test_invalid_csv_is_rejected_instead_of_silently_skipping_rows(tmp_path, data):
    path = tmp_path/'invalid.csv'
    path.write_text(data)
    with pytest.raises(ValueError):
        load_csv(path)


def test_forward_reverse_blocks_share_a_stop_boundary():
    points = [Point(0, 0), Point(.5, 0), Point(1, 0), Point(.5, 0), Point(0, 0)]
    with pytest.raises(ValueError, match='倒车'):
        motion_blocks(points)
    blocks = motion_blocks(points, allow_reverse=True)
    assert [b.direction for b in blocks] == [1, -1]
    assert blocks[0].points[-1] == blocks[1].points[0]
    tracker = Tracker(blocks, .69)
    assert tracker.advance() and tracker.block.direction == -1
    assert not tracker.advance()


def test_recording_gaps_and_sideways_motion_are_rejected():
    with pytest.raises(ValueError, match='断点'):
        motion_blocks([Point(0, 0), Point(5, 0)])
    with pytest.raises(ValueError, match='朝向'):
        motion_blocks([Point(0, 0), Point(0, .5)])


def test_projection_respects_segment_bounds_at_corner():
    block = Block([Point(0, 0), Point(1, 0), Point(1, 1, yaw=math.pi/2)], 1)
    error, arc = block.project(Point(.5, .4), 0., .6)
    assert error == pytest.approx(.4) and arc == pytest.approx(.5)


def test_closed_loop_does_not_finish_or_jump_to_final_segment_at_start():
    points = [Point(2*math.cos(t), 2*math.sin(t), yaw=t+math.pi/2)
              for t in [i*2*math.pi/160 for i in range(161)]]
    tracker = Tracker(motion_blocks(points), .69)
    _, _, reached = tracker.command(points[0], 0., .3, .4)
    assert not reached and tracker.progress < .001


def test_circle_is_followed_in_order_until_the_endpoint():
    points = [Point(2*math.sin(t), 2*(1-math.cos(t)), yaw=t)
              for t in [i*math.pi/160 for i in range(161)]]
    tracker, pose = Tracker(motion_blocks(points), .69), points[0]
    speed, previous = 0., 0.
    for _ in range(2000):
        speed, angular, reached = tracker.command(pose, speed, .3, .4)
        assert tracker.progress >= previous
        previous = tracker.progress
        assert abs(angular) <= abs(speed)/.69+1e-9
        if reached:
            break
        pose = Point(pose.x+speed*math.cos(pose.yaw)*.05,
                     pose.y+speed*math.sin(pose.yaw)*.05, yaw=pose.yaw+angular*.05)
    else:
        pytest.fail('Pure Pursuit did not reach the recorded endpoint')
    assert math.hypot(pose.x-points[-1].x, pose.y-points[-1].y) <= .1


def test_short_leg_cannot_be_swallowed_by_goal_tolerance():
    block = Block([Point(0, 0), Point(.04, 0)], 1)
    tracker = Tracker([block], .69)
    assert not tracker.command(Point(0, 0), 0., .3, .4)[2]
    assert tracker.command(Point(.039, 0), 0., .3, .4)[2]


def test_off_route_and_wrong_heading_stop_instead_of_in_place_rotation():
    for pose in (Point(0, 1), Point(0, 0, yaw=math.pi)):
        tracker = Tracker(motion_blocks([Point(0, 0), Point(1, 0)]), .69)
        with pytest.raises(ValueError):
            tracker.command(pose, 0., .3, .4)


def test_reverse_pursuit_keeps_explicit_direction_and_physical_curvature():
    block = Block([Point(0, 0), Point(-.5, 0), Point(-1, .1)], -1)
    tracker = Tracker([block], .69)
    velocity, angular, reached = tracker.command(Point(0, -.1), 0., .15, .4)
    assert velocity < 0 and angular < 0 and not reached
    assert abs(angular) <= abs(velocity)/.69


def test_full_footprint_collision_and_braking_distance():
    polygon = [[.4475, .28], [.4475, -.28], [-.4475, -.28], [-.4475, .28]]
    assert swept_collision([(.2, .3)], polygon, .3, 0., .4, .03)
    assert swept_collision([(.8, 0.)], polygon, .3, 0., .4, .12)
    assert swept_collision([(-.8, 0.)], polygon, -.3, 0., .4, .12)
    assert not swept_collision([(1.5, 0.), (0., 1.)], polygon, .3, 0., .4, .12)


def test_turning_collision_uses_swept_polygon_instead_of_front_sector():
    polygon = [[.45, .28], [.45, -.28], [-.45, -.28], [-.45, .28]]
    assert swept_collision([(.65, .5)], polygon, .5, 1/.69, .4, .08)
    assert not swept_collision([(.65, .5)], polygon, .5, 0., .4, .08)
