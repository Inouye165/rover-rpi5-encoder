from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_bringup',
            executable='rover_system_health',
            name='rover_system_health',
            output='screen'
        )
    ])
