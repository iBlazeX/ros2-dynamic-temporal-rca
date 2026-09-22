import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rca_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lynx',
    maintainer_email='lynx@todo.todo',
    description='Gazebo Harmonic simulation backend for the ROS 2 RCA experiments',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = rca_gazebo.gz_drivers:lidar_main',
            'camera_node = rca_gazebo.gz_drivers:camera_main',
            'imu_node = rca_gazebo.gz_drivers:imu_main',
            'odometry_node = rca_gazebo.gz_drivers:odometry_main',
            'sim_world = rca_gazebo.gz_drivers:world_main',
            'scenario_runner = rca_gazebo.scenario_runner:main',
        ],
    },
)
