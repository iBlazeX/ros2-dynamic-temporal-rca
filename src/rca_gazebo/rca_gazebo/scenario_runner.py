"""Gazebo scenario runner (evaluation layer).

Identical evaluation machinery to the rca_sim runner - same ExperimentController,
same RCAEvaluator, same baselines, same ablations, same event store - pointed at
the Gazebo scenario file. The diagnostic monitor and the RCA engine are not
changed or even aware of which backend produced the data.

    ros2 launch rca_gazebo gazebo_system.launch.py     # terminal 1
    ros2 run rca_gazebo scenario_runner --repetitions 3 # terminal 2

Results go to runs/gazebo/<timestamp>/results.json and carry "simulator":
"gazebo" so Gazebo numbers are never silently pooled with rca_sim numbers.
"""

import os

from ament_index_python.packages import get_package_share_directory

from rca_sim import scenario_runner as sim_runner


def default_scenarios_path() -> str:
    return os.path.join(get_package_share_directory('rca_gazebo'), 'config', 'scenarios.yaml')


def main(args=None):
    sim_runner.main(
        argv_defaults={
            'scenarios_file': default_scenarios_path(),
            'out_dir': os.path.join('runs', 'gazebo'),
            'simulator': 'gazebo',
        }
    )


if __name__ == '__main__':
    main()
