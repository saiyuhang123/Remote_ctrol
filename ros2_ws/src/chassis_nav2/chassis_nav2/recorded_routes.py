"""Opt-in Nav2 overlay. Ordinary navigation never loads the new libraries."""
import json
import math
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def hardware_radius(hw):
    measured=hw['chassis'].get('min_turning_radius')
    if measured is not None:
        return float(measured)
    geometry=hw['robot_geometry']
    halves=2.0 if hw['chassis'].get('four_wheel_steering',False) else 1.0
    return float(geometry['wheel_base'])/(halves*math.tan(math.radians(float(geometry['max_steering_deg']))))


def validate_options(args, mapping=False):
    enabled = args.get('enable_recorded_routes', 'false').lower() == 'true'
    if enabled and (mapping or args.get('enable_patrol', 'false').lower() != 'true'):
        raise ValueError('固定路线需要定位导航模式及 enable_patrol:=true')
    return enabled


def nav2_overlay(executable, source_file, hw):
    # Resolve the package now, so a half-installed opt-in session fails before
    # launching any nodes. The default session does not need this package.
    get_package_share_directory('recorded_route_nav2')
    original = yaml.safe_load(Path(source_file).read_text())
    if executable == 'bt_navigator':
        libs = list(original['bt_navigator']['ros__parameters']['plugin_lib_names'])
        if 'recorded_route_bt' not in libs:
            libs.append('recorded_route_bt')
        return {'plugin_lib_names': libs}
    if executable == 'planner_server':
        # Independent forward-only candidate planner. Ordinary GridBased
        # retains its motion model and parameters for free navigation.
        names=original['planner_server']['ros__parameters']['planner_plugins']
        radius=hardware_radius(hw)
        return {'planner_plugins':list(dict.fromkeys(names+['RecordedDetour'])),
                'RecordedDetour.plugin':'nav2_smac_planner/SmacPlannerHybrid',
                'RecordedDetour.minimum_turning_radius':radius,
                'RecordedDetour.motion_model_for_search':'DUBIN',
                'RecordedDetour.angle_quantization_bins':72,
                'RecordedDetour.allow_unknown':False,
                'RecordedDetour.tolerance':0.05,
                'RecordedDetour.max_planning_time':3.0,
                'RecordedDetour.cost_penalty':2.0,
                'RecordedDetour.analytic_expansion_max_length':1.5,
                'RecordedDetour.smooth_path':False}
    if executable != 'controller_server':
        return {}
    cfg = original['controller_server']['ros__parameters']
    speed = min(0.2, float(hw['chassis']['max_speed']))
    result = {'controller_plugins': list(dict.fromkeys(cfg['controller_plugins']+['RecordedRoute'])),
              'goal_checker_plugins': list(dict.fromkeys(cfg['goal_checker_plugins']+['recorded_goal_checker','recorded_stop_checker']))}
    parameters = {
        'plugin': 'recorded_route_nav2::RecordedController', 'desired_linear_vel': speed,
        'strict_deviation': 0.20, 'max_heading_error': 0.55,
        'use_rotate_to_heading': False, 'allow_reversing': True,
        'lookahead_dist': 0.35, 'use_velocity_scaled_lookahead_dist': True,
        'lookahead_time': 1.5, 'min_lookahead_dist': 0.15, 'max_lookahead_dist': 0.50,
        'max_robot_pose_search_dist': 0.20, 'use_collision_detection': True,
        'regulated_linear_scaling_min_speed': 0.04, 'transform_tolerance': 0.3,
        'use_regulated_linear_velocity_scaling': True,
        'regulated_linear_scaling_min_radius': 1.5,
    }
    result.update({'RecordedRoute.'+key: value for key,value in parameters.items()})
    for name,tolerance in [('recorded_goal_checker',0.20),('recorded_stop_checker',0.08)]:
        result.update({name+'.plugin': 'recorded_route_nav2::RecordedGoalChecker',
                       name+'.controller_id': 'RecordedRoute', name+'.xy_goal_tolerance': tolerance})
    return result


def clearance_rewrites():
    # A smooth obstacle potential keeps candidate paths away from walls.
    # Only opt-in road sessions use this; full-body collision checks still
    # decide whether a candidate can be executed.
    return {'inflation_radius':'0.55','cost_scaling_factor':'3.0'}
