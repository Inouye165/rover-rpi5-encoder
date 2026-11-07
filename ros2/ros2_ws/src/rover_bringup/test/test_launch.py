import os
import sys
import importlib.util
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node


def test_launch_description():
    launch_path = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'foundation.launch.py'
    )
    assert os.path.exists(launch_path)

    # Load foundation.launch.py dynamically
    spec = importlib.util.spec_from_file_location("foundation.launch", launch_path)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)

    # Generate LaunchDescription
    ld = launch_module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    # Phase 3 + Foxglove + cmd_vel + IMU static TF + ToF bridge & 2 ToF static TFs + IMU bridge + nav bridge: 12 entities
    entities = ld.entities
    assert len(entities) == 12, f"Expected 12 entities in launch, found {len(entities)}"

    nodes = [e for e in entities if isinstance(e, Node)]
    node_executables = {e.node_executable for e in nodes}

    assert 'rover_system_health' in node_executables, "rover_system_health must be in launch"
    assert 'rover_lidar_bridge' in node_executables, "rover_lidar_bridge must be in launch"
    assert 'rover_tof_bridge' in node_executables, "rover_tof_bridge must be in launch"
    assert 'rover_encoder_odometry' in node_executables, "rover_encoder_odometry must be in launch"
    assert 'rover_cmd_vel_bridge' in node_executables, "rover_cmd_vel_bridge must be in launch"
    assert 'rover_imu_bridge' in node_executables, "rover_imu_bridge must be in launch"
    assert 'rover_nav_bridge' in node_executables, "rover_nav_bridge must be in launch"
    assert 'static_transform_publisher' in node_executables, "static_transform_publisher must be in launch"
    assert 'foxglove_bridge' in node_executables, "foxglove_bridge must be in launch"

    # Static TF nodes check (laser_frame, imu_link, tof_front_left_link, tof_front_right_link)
    tf_nodes = [n for n in nodes if n.node_executable == 'static_transform_publisher']
    assert len(tf_nodes) == 4, f"Expected 4 static_transform_publisher nodes, found {len(tf_nodes)}"
    for n in tf_nodes:
        assert n.node_package == 'tf2_ros'

    def parse_node_args(node):
        raw_args = getattr(node, '_Node__arguments', []) or getattr(node, 'node_arguments', [])
        parsed = []
        for a in raw_args:
            if isinstance(a, (list, tuple)) and len(a) > 0 and hasattr(a[0], 'text'):
                parsed.append(a[0].text)
            elif hasattr(a, 'text'):
                parsed.append(a.text)
            else:
                parsed.append(str(a))
        return parsed

    # Check ToF static TF arguments (125 mm lateral separation, centered symmetrically at y = ±0.0625 m)
    left_tof_node = next(n for n in tf_nodes if (getattr(n, '_Node__node_name', '') or getattr(n, 'name', '')) == 'base_link_to_tof_front_left_publisher')
    right_tof_node = next(n for n in tf_nodes if (getattr(n, '_Node__node_name', '') or getattr(n, 'name', '')) == 'base_link_to_tof_front_right_publisher')

    left_args = parse_node_args(left_tof_node)
    right_args = parse_node_args(right_tof_node)

    assert '--y' in left_args
    left_y_idx = left_args.index('--y') + 1
    assert float(left_args[left_y_idx]) == 0.0625, f"Front left ToF y should be 0.0625 m, got {left_args[left_y_idx]}"

    assert '--y' in right_args
    right_y_idx = right_args.index('--y') + 1
    assert float(right_args[right_y_idx]) == -0.0625, f"Front right ToF y should be -0.0625 m, got {right_args[right_y_idx]}"

    # Verify 125 mm separation and symmetry
    sep_m = float(left_args[left_y_idx]) - float(right_args[right_y_idx])
    assert abs(sep_m - 0.125) < 1e-6, f"Expected 125 mm (0.125 m) lateral separation, got {sep_m}"

    # Verify forward position x=0.14 m and height z=0.08 m preserved
    left_x_idx = left_args.index('--x') + 1
    assert float(left_args[left_x_idx]) == 0.14
    left_z_idx = left_args.index('--z') + 1
    assert float(left_args[left_z_idx]) == 0.08

    # Verify outward angles ±8 deg (±0.13962634 rad)
    left_yaw_idx = left_args.index('--yaw') + 1
    assert abs(float(left_args[left_yaw_idx]) - 0.13962634) < 1e-6
    right_yaw_idx = right_args.index('--yaw') + 1
    assert abs(float(right_args[right_yaw_idx]) - (-0.13962634)) < 1e-6

    # Foxglove bridge safety & read-only parameters check
    foxglove_node = [n for n in nodes if n.node_executable == 'foxglove_bridge'][0]
    assert foxglove_node.node_package == 'foxglove_bridge'
    node_params = getattr(foxglove_node, 'node_parameters', None) or getattr(foxglove_node, '_Node__parameters', None)
    if node_params and isinstance(node_params, (list, tuple)) and len(node_params) > 0:
        raw_params = node_params[0]
        if isinstance(raw_params, dict):
            # Keys in ROS 2 launch Node parameters are TextSubstitution tuple objects
            param_str_keys = {str(k[0].text if isinstance(k, tuple) and len(k) > 0 and hasattr(k[0], 'text') else k): v for k, v in raw_params.items()}
            assert param_str_keys.get('port') == 8765
            caps = param_str_keys.get('capabilities', [])
            cap_texts = [c.text if hasattr(c, 'text') else str(c) for c in (caps[0] if isinstance(caps, tuple) else caps)]
            assert 'clientPublish' not in cap_texts, "clientPublish must be disabled for read-only visualization safety"

    # Ensure no forbidden navigation or motor packages are launched
    forbidden_packages = [
        'nav2_bringup', 'slam_toolbox', 'navigation2', 'nav2_planner',
        'nav2_controller', 'amcl'
    ]
    for n in nodes:
        assert n.node_package not in forbidden_packages
