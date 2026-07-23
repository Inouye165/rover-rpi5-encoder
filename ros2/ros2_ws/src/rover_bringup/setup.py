from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ron',
    maintainer_email='ron@todo.todo',
    description='ROS 2 bringup and health diagnostics for Yahboom Pi 5 Rover',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_system_health = rover_bringup.rover_system_health:main',
            'rover_lidar_bridge = rover_bringup.rover_lidar_bridge:main',
            'rover_encoder_odometry = rover_bringup.rover_encoder_odometry:main',
        ],
    },
)
