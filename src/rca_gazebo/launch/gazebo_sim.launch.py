"""Gazebo backend: gz sim + ros_gz bridge + sensor drivers + the reused rca_sim pipeline.

    gz sim (rca_arena.sdf)
        -> ros_gz_bridge parameter_bridge  (everything into the /gz/... namespace)
        -> lidar_node / camera_node / imu_node / odometry_node   (rca_gazebo drivers)
        -> perception_node -> localization_node -> planning_node -> control_node
                                                            (rca_sim, reused verbatim)
        -> sim_world (actuation sink) -> /gz/cmd_vel -> bridge -> Gazebo DiffDrive

The processing nodes respawn automatically so node-failure scenarios exercise
recovery and reappearance in the ROS graph, exactly as in the rca_sim backend.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('rca_gazebo')
    world = LaunchConfiguration('world').perform(context)
    world_path = world if os.path.isabs(world) else os.path.join(pkg, 'worlds', world)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    seed = int(LaunchConfiguration('seed').perform(context))
    respawn_delay = float(LaunchConfiguration('respawn_delay').perform(context))
    camera = LaunchConfiguration('camera_enabled').perform(context).lower() in ('true', '1', 'yes')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')

    gz_args = ['gz', 'sim', '-r', '-v', '2']
    if not gui:
        gz_args += ['-s', '--headless-rendering']
    gz_args += [world_path]

    env = dict(os.environ)
    env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')   # WSL2 / headless: software OpenGL for gz sensors

    def driver(exe, name, enabled=True):
        return Node(package='rca_gazebo', executable=exe, name=name, output='screen',
                    respawn=True, respawn_delay=respawn_delay,
                    parameters=[{'seed': seed}]) if enabled else None

    def proc(exe, name):
        return Node(package='rca_sim', executable=exe, name=name, output='screen',
                    respawn=True, respawn_delay=respawn_delay, parameters=[{'seed': seed}])

    actions = [
        ExecuteProcess(cmd=gz_args, output='screen', additional_env=env, name='gz_sim'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             output='screen', parameters=[{'config_file': bridge_cfg}]),
        driver('lidar_node', 'lidar_node'),
        driver('imu_node', 'imu_node'),
        driver('odometry_node', 'odometry_node'),
        Node(package='rca_gazebo', executable='sim_world', name='sim_world', output='screen',
             parameters=[{'seed': seed}]),
        proc('perception_node', 'perception_node'),
        proc('localization_node', 'localization_node'),
        proc('planning_node', 'planning_node'),
        proc('control_node', 'control_node'),
    ]
    if camera:
        actions.insert(3, driver('camera_node', 'camera_node'))
    return [a for a in actions if a is not None]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='rca_arena.sdf',
                              description='World file (name inside rca_gazebo/worlds or absolute path)'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='true starts the Gazebo GUI; false is server + headless rendering'),
        DeclareLaunchArgument('seed', default_value='7',
                              description='Deterministic seed for the sensor-noise RNG of the drivers'),
        DeclareLaunchArgument('camera_enabled', default_value='true',
                              description='Start camera_node at launch (false = the node appears later)'),
        DeclareLaunchArgument('respawn_delay', default_value='4.0',
                              description='Seconds before a crashed/stopped node is restarted'),
        OpaqueFunction(function=_setup),
    ])
