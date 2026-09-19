import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rca_test_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
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
    description='Robotic pipeline test system and fault injection for dynamic temporal RCA research',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = rca_test_system.sensor_node:main',
            'perception_node = rca_test_system.perception_node:main',
            'localization_node = rca_test_system.localization_node:main',
            'navigation_node = rca_test_system.navigation_node:main',
            'fault_injector = rca_test_system.fault_injector:main',
            'experiment_controller = rca_test_system.experiment_controller:main',
        ],
    },
)
