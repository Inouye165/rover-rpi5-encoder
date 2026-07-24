from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Phase 3 launch:
      - rover_system_health
      - rover_lidar_bridge
      - rover_encoder_odometry
      - static TF publisher (base_link -> laser_frame)

    All bridge nodes read sidecar URLs from ROS parameters or environment
    variables (ROVER_LIDAR_URL / ROVER_SERVER_URL) injected by compose.yaml.
    No /dev mounts or motor commands are involved.
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
        Node(
            package='rover_bringup',
            executable='rover_encoder_odometry',
            name='rover_encoder_odometry',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_frame_publisher',
            output='screen',
            arguments=[
                '--x', '0.0127',
                '--y', '0.034925',
                '--z', '0.08',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame',
            ],
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'capabilities': ['connectionGraph', 'assets'],
                'topic_whitelist': ['.*'],
            }],
        ),
    ])
