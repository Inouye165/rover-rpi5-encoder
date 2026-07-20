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
    
    # Verify we only launch rover_system_health and no navigation/SLAM tools
    entities = ld.entities
    assert len(entities) == 1
    
    node = entities[0]
    assert isinstance(node, Node)
    assert node.node_package == 'rover_bringup'
    
    assert node.node_executable == 'rover_system_health'
    
    # Ensure no forbidden navigation or motor packages are defined
    forbidden_packages = [
        'nav2_bringup', 'slam_toolbox', 'navigation2', 'nav2_planner',
        'nav2_controller', 'amcl', 'robot_state_publisher'
    ]
    for entity in entities:
        if isinstance(entity, Node):
            assert entity.node_package not in forbidden_packages
