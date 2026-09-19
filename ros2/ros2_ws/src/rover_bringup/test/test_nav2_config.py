import os
import yaml
import pytest

def find_nav2_params():
    candidates = [
        os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml'),
        '/ros2_ws/src/rover_bringup/config/nav2_params.yaml',
        r'c:\Users\Ron\electronic_projects\yahboom-encoder\ros2\ros2_ws\src\rover_bringup\config\nav2_params.yaml'
    ]
    for p in candidates:
        if os.path.exists(p):
            return os.path.abspath(p)
    raise FileNotFoundError("Could not find nav2_params.yaml")

def test_goal_checker_and_rotatetogoal_tolerances():
    path = find_nav2_params()
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    ctrl = cfg['controller_server']['ros__parameters']
    ggc = ctrl['general_goal_checker']
    fp = ctrl['FollowPath']

    # 1. Arrival tolerances
    assert ggc['xy_goal_tolerance'] <= 0.05, f"general_goal_checker xy_goal_tolerance too loose: {ggc['xy_goal_tolerance']}"
    assert ggc['yaw_goal_tolerance'] <= 0.088, f"general_goal_checker yaw_goal_tolerance too loose: {ggc['yaw_goal_tolerance']}"

    # 2. DWB parameters
    assert fp['xy_goal_tolerance'] <= 0.05, f"FollowPath xy_goal_tolerance too loose: {fp['xy_goal_tolerance']}"
    assert fp['yaw_goal_tolerance'] <= 0.088, f"FollowPath yaw_goal_tolerance too loose: {fp['yaw_goal_tolerance']}"

    # 3. RotateToGoal critic alignment - prevents 25cm early cutoff deadlock
    assert 'RotateToGoal.xy_goal_tolerance' in fp, "RotateToGoal.xy_goal_tolerance MUST be explicitly set to prevent 0.25m default"
    assert fp['RotateToGoal.xy_goal_tolerance'] == fp['xy_goal_tolerance'], \
        f"RotateToGoal.xy_goal_tolerance ({fp['RotateToGoal.xy_goal_tolerance']}) must match FollowPath.xy_goal_tolerance ({fp['xy_goal_tolerance']})"

    assert 'RotateToGoal.trans_stopped_velocity' in fp, "RotateToGoal.trans_stopped_velocity MUST be explicitly set"
    assert fp['RotateToGoal.trans_stopped_velocity'] <= 0.05, \
        f"RotateToGoal.trans_stopped_velocity too high: {fp['RotateToGoal.trans_stopped_velocity']}"

def test_controller_speed_and_progress_checker():
    path = find_nav2_params()
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    ctrl = cfg['controller_server']['ros__parameters']
    fp = ctrl['FollowPath']
    pc = ctrl['progress_checker']

    assert fp['max_vel_x'] <= 0.15, f"max_vel_x exceeds 0.15: {fp['max_vel_x']}"
    assert fp['max_vel_theta'] <= 0.50, f"max_vel_theta exceeds 0.50: {fp['max_vel_theta']}"
    assert fp['rotate_to_heading_angular_vel'] <= 0.50, f"rotate_to_heading_angular_vel exceeds 0.50: {fp['rotate_to_heading_angular_vel']}"

    # Progress checker must not prematurely abort micro-maneuvers near the goal
    assert pc['required_movement_radius'] <= 0.15, f"required_movement_radius too strict: {pc['required_movement_radius']}"
    assert pc['movement_time_allowance'] >= 10.0, f"movement_time_allowance too short: {pc['movement_time_allowance']}"

def test_costmap_and_collision_monitor_invariants():
    path = find_nav2_params()
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    # Global costmap inflation radius
    gcm = cfg['global_costmap']['global_costmap']['ros__parameters']
    assert gcm['inflation_layer']['inflation_radius'] <= 0.35, "global inflation radius too large"
    assert gcm['inflation_layer']['cost_scaling_factor'] >= 3.0, "global cost scaling factor too shallow"

    # Velocity smoother clamp
    vs = cfg['velocity_smoother']['ros__parameters']
    assert vs['max_velocity'][0] <= 0.15
    assert vs['max_velocity'][2] <= 0.50

    # Collision monitor polygons
    cm = cfg['collision_monitor']['ros__parameters']
    assert "PolygonStop" in cm['polygons']
    assert "PolygonSlow" in cm['polygons']