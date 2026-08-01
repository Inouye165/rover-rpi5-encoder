import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_bringup')
    default_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to SLAM Toolbox configuration YAML file'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        slam_node
    ])
