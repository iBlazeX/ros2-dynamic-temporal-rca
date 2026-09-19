"""Launch file for the 4-node RCA pipeline."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rca_test_system')
    params_file = os.path.join(pkg_share, 'config', 'rca_params.yaml')

    return LaunchDescription([
        Node(
            package='rca_test_system',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rca_test_system',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rca_test_system',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rca_test_system',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
