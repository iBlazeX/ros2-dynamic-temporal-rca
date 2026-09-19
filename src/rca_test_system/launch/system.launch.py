"""Full system launch file: pipeline nodes + diagnostic monitor."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rca = get_package_share_directory('rca_test_system')
    pkg_diag = get_package_share_directory('diagnostic_monitor')

    pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rca, 'launch', 'pipeline.launch.py')
        )
    )

    monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diag, 'launch', 'monitor.launch.py')
        )
    )

    return LaunchDescription([
        pipeline_launch,
        monitor_launch,
    ])
