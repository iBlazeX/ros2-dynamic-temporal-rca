"""Launch file for the Diagnostic Monitor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('db_path', default_value='events.db', description='Path to SQLite database'),
        DeclareLaunchArgument('warmup_sec', default_value='4.0', description='Warmup grace period in seconds'),
        DeclareLaunchArgument('z_threshold', default_value='3.0', description='Statistical z-score threshold'),
        DeclareLaunchArgument('w_d', default_value='0.30', description='Dependency weight wD'),
        DeclareLaunchArgument('w_t', default_value='0.30', description='Temporal weight wT'),
        DeclareLaunchArgument('w_s', default_value='0.25', description='Symptom coverage weight wS'),
        DeclareLaunchArgument('w_a', default_value='0.15', description='Anomaly severity weight wA'),

        Node(
            package='diagnostic_monitor',
            executable='monitor_node',
            name='diagnostic_monitor',
            output='screen',
            parameters=[{
                'db_path': LaunchConfiguration('db_path'),
                'warmup_sec': LaunchConfiguration('warmup_sec'),
                'z_threshold': LaunchConfiguration('z_threshold'),
                'w_d': LaunchConfiguration('w_d'),
                'w_t': LaunchConfiguration('w_t'),
                'w_s': LaunchConfiguration('w_s'),
                'w_a': LaunchConfiguration('w_a'),
            }],
        ),
    ])
