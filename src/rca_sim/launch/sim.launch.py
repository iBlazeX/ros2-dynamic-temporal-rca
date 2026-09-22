"""Simulation only: world + sensors process and the four processing nodes.

Processing nodes respawn automatically (respawn_delay) so node-failure scenarios
also exercise recovery / reappearance in the ROS graph.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    seed = int(LaunchConfiguration('seed').perform(context))
    camera = LaunchConfiguration('camera_enabled').perform(context).lower() in ('true', '1', 'yes')
    # launch (Jazzy) requires respawn_delay as a plain float, not a substitution
    respawn_delay = float(LaunchConfiguration('respawn_delay').perform(context))

    def proc(name):
        return Node(package='rca_sim', executable=name, name=name, output='screen',
                    respawn=True, respawn_delay=respawn_delay, parameters=[{'seed': seed}])

    return [
        Node(package='rca_sim', executable='sim_world', name='sim_world', output='screen',
             parameters=[{'seed': seed, 'camera_enabled': camera}]),
        proc('perception_node'),
        proc('localization_node'),
        proc('planning_node'),
        proc('control_node'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='7', description='Deterministic world / noise seed'),
        DeclareLaunchArgument('camera_enabled', default_value='true',
                              description='Start the camera node at launch (false = camera appears later)'),
        DeclareLaunchArgument('respawn_delay', default_value='4.0',
                              description='Seconds before a crashed/stopped processing node is restarted'),
        OpaqueFunction(function=_nodes),
    ])
