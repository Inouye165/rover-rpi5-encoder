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

    # Phase 3 + Foxglove: 5 entities — rover_system_health, rover_lidar_bridge, rover_encoder_odometry, static_transform_publisher, foxglove_bridge
    entities = ld.entities
    assert len(entities) == 5, f"Expected 5 entities in launch, found {len(entities)}"

    nodes = [e for e in entities if isinstance(e, Node)]
    node_executables = {e.node_executable for e in nodes}

    assert 'rover_system_health' in node_executables, "rover_system_health must be in launch"
    assert 'rover_lidar_bridge' in node_executables, "rover_lidar_bridge must be in launch"
    assert 'rover_encoder_odometry' in node_executables, "rover_encoder_odometry must be in launch"
    assert 'static_transform_publisher' in node_executables, "static_transform_publisher must be in launch"
    assert 'foxglove_bridge' in node_executables, "foxglove_bridge must be in launch"

    # Static TF node check
    tf_node = [n for n in nodes if n.node_executable == 'static_transform_publisher'][0]
    assert tf_node.node_package == 'tf2_ros'

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
