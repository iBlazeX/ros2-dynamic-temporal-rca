"""Simulation + the existing diagnostic monitor (unchanged)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = get_package_share_directory('rca_sim')
    diag = get_package_share_directory('diagnostic_monitor')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('respawn_delay', default_value='4.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'sim.launch.py')),
            launch_arguments={'seed': LaunchConfiguration('seed'),
                              'camera_enabled': LaunchConfiguration('camera_enabled'),
                              'respawn_delay': LaunchConfiguration('respawn_delay')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(diag, 'launch', 'monitor.launch.py')),
            launch_arguments={'simulator': 'rca_sim'}.items()),
    ])
