import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rca_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lynx',
    maintainer_email='lynx@todo.todo',
    description='Lightweight kinematic robot simulation with runtime degradation for RCA experiments',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_world = rca_sim.sim_world_node:main',
            'perception_node = rca_sim.perception_node:main',
            'localization_node = rca_sim.localization_node:main',
            'planning_node = rca_sim.planning_node:main',
            'control_node = rca_sim.control_node:main',
            'scenario_runner = rca_sim.scenario_runner:main',
        ],
    },
)
