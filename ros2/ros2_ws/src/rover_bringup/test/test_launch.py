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

    # Phase 3: 4 entities — rover_system_health, rover_lidar_bridge, rover_encoder_odometry, static_transform_publisher
    entities = ld.entities
    assert len(entities) == 4, f"Expected 4 entities in Phase 3 launch, found {len(entities)}"

    nodes = [e for e in entities if isinstance(e, Node)]
    node_executables = {e.node_executable for e in nodes}

    assert 'rover_system_health' in node_executables, "rover_system_health must be in launch"
    assert 'rover_lidar_bridge' in node_executables, "rover_lidar_bridge must be in launch"
    assert 'rover_encoder_odometry' in node_executables, "rover_encoder_odometry must be in launch"
    assert 'static_transform_publisher' in node_executables, "static_transform_publisher must be in launch"

    # Static TF node check
    tf_node = [n for n in nodes if n.node_executable == 'static_transform_publisher'][0]
    assert tf_node.node_package == 'tf2_ros'

    # Ensure no forbidden navigation or motor packages are launched
    forbidden_packages = [
        'nav2_bringup', 'slam_toolbox', 'navigation2', 'nav2_planner',
        'nav2_controller', 'amcl'
    ]
    for n in nodes:
        assert n.node_package not in forbidden_packages
