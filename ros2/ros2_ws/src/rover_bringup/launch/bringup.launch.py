import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_bringup')

    default_map = os.path.join(
        '/ros2_ws',
        'maps',
        'house_slam_2026-08-23_final.yaml'
    )
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map YAML file for localization'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to Nav2 configuration YAML file'
    )

    # 1. Foundation launch (sensors, odometry, IMU, TF, Foxglove bridge)
    foundation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'foundation.launch.py')
        )
    )

    # 2. Navigation / Localization launch (Map Server, AMCL, Costmaps, Nav2 stack)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        foundation_launch,
        navigation_launch,
    ])
