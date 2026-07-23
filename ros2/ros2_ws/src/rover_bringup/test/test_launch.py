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

    # Phase 2: exactly 2 nodes — rover_system_health + rover_lidar_bridge
    entities = ld.entities
    assert len(entities) == 2, f"Expected 2 nodes, found {len(entities)}"

    node_executables = {e.node_executable for e in entities if isinstance(e, Node)}
    assert 'rover_system_health' in node_executables, \
        "rover_system_health must be in launch"
    assert 'rover_lidar_bridge' in node_executables, \
        "rover_lidar_bridge must be in launch"

    # All entities must be rover_bringup nodes
    for entity in entities:
        assert isinstance(entity, Node)
        assert entity.node_package == 'rover_bringup'

    # Ensure no forbidden navigation or motor packages are defined
    forbidden_packages = [
        'nav2_bringup', 'slam_toolbox', 'navigation2', 'nav2_planner',
        'nav2_controller', 'amcl', 'robot_state_publisher'
    ]
    for entity in entities:
        if isinstance(entity, Node):
            assert entity.node_package not in forbidden_packages
