"""Gazebo backend + the existing diagnostic monitor (unchanged RCA core).

The monitor is given two deployment settings and nothing else:
  simulator:=gazebo                      presentation label on /rca/dashboard_state
  excluded_topic_prefixes:=['/gz/']      raw simulator transport is not application data flow

It is never told the topology, the scenario, the injected fault or its target.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz = get_package_share_directory('rca_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='rca_arena.sdf'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('respawn_delay', default_value='4.0'),
        DeclareLaunchArgument('db_path', default_value=''),
        DeclareLaunchArgument(
            'monitor_delay_sec', default_value='60.0',
            description='Seconds to wait before attaching the diagnostic monitor. Gazebo needs '
                        'tens of seconds before localization converges and the robot starts '
                        'driving; a monitor attached earlier would learn the start-up safety '
                        'stop (empty path, zero command) as its nominal baseline.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gazebo_sim.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui'),
                              'seed': LaunchConfiguration('seed'),
                              'camera_enabled': LaunchConfiguration('camera_enabled'),
                              'respawn_delay': LaunchConfiguration('respawn_delay')}.items()),
        OpaqueFunction(function=_monitor),
    ])


def _monitor(context, *args, **kwargs):
    delay = float(LaunchConfiguration('monitor_delay_sec').perform(context))
    node = Node(
        package='diagnostic_monitor', executable='monitor_node', name='diagnostic_monitor',
        output='screen',
        parameters=[{
            'db_path': LaunchConfiguration('db_path'),
            'simulator': 'gazebo',
            'excluded_topic_prefixes': ['/gz/'],
        }],
    )
    return [TimerAction(period=delay, actions=[node])] if delay > 0 else [node]
