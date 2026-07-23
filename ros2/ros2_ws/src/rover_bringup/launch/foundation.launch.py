from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Phase 2 launch: rover_system_health + rover_lidar_bridge.

    Both nodes read their sidecar URLs from the ROS parameters that fall back
    to the ROVER_LIDAR_URL / ROVER_SERVER_URL environment variables already
    injected by compose.yaml.  No /dev mounts or motor commands are involved.
    """
    return LaunchDescription([
        Node(
            package='rover_bringup',
            executable='rover_system_health',
            name='rover_system_health',
            output='screen',
        ),
        Node(
            package='rover_bringup',
            executable='rover_lidar_bridge',
            name='rover_lidar_bridge',
            output='screen',
        ),
    ])
