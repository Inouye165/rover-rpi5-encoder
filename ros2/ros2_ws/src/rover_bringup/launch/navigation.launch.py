import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to Nav2 configuration YAML file'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join('/ros2_ws', 'maps', 'house_map.yaml'),
        description='Full path to map YAML file for localization'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        map_arg,
        nav2_bringup_launch
    ])
