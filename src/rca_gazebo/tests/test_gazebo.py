"""Offline tests for the Gazebo backend.

Runtime tests (a live gz sim, real ROS topics, a real fault changing runtime
behaviour, graph discovery, the TUI) are executed by scripts/gazebo_sim.sh and
scripts/gazebo_stages.sh and recorded under runs/gazebo/; they are not unit
tests because they need a running simulator.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WORLD = os.path.join(PKG, 'worlds', 'rca_arena.sdf')
BRIDGE = os.path.join(PKG, 'config', 'bridge.yaml')
SCENARIOS = os.path.join(PKG, 'config', 'scenarios.yaml')


def _scenarios():
    with open(SCENARIOS) as f:
        return yaml.safe_load(f)


# ------------------------------------------------------------------- world

def test_world_is_valid_sdf_with_the_required_robot_and_sensors():
    root = ET.parse(WORLD).getroot()
    assert root.tag == 'sdf'
    world = root.find('world')
    assert world is not None and world.get('name') == 'rca_arena'
    models = {m.get('name') for m in world.findall('model')}
    assert 'rca_bot' in models and 'ground_plane' in models
    assert sum(1 for m in models if m.startswith('wall_')) == 4, 'arena must be bounded'
    assert sum(1 for m in models if m.startswith('obstacle_')) >= 8, 'arena must contain obstacles'

    bot = next(m for m in world.findall('model') if m.get('name') == 'rca_bot')
    sensors = {s.get('name'): s for s in bot.iter('sensor')}
    assert set(sensors) == {'lidar', 'camera', 'imu'}
    assert sensors['lidar'].get('type') == 'gpu_lidar'
    assert sensors['camera'].get('type') == 'camera'
    assert sensors['imu'].get('type') == 'imu'
    # documented sensor rates
    rates = {n: float(s.findtext('update_rate')) for n, s in sensors.items()}
    assert rates == {'lidar': 10.0, 'camera': 5.0, 'imu': 20.0}


def test_world_has_a_differential_drive_with_odometry_at_20hz():
    world = ET.parse(WORLD).getroot().find('world')
    bot = next(m for m in world.findall('model') if m.get('name') == 'rca_bot')
    plugins = [p for p in bot.findall('plugin') if 'DiffDrive' in (p.get('name') or '')]
    assert len(plugins) == 1
    dd = plugins[0]
    assert dd.findtext('left_joint') == 'left_wheel_joint'
    assert dd.findtext('right_joint') == 'right_wheel_joint'
    assert float(dd.findtext('odom_publish_frequency')) == 20.0
    assert dd.findtext('topic') == '/model/rca_bot/cmd_vel'
    assert dd.findtext('odom_topic') == '/model/rca_bot/odometry'
    joints = {j.get('name') for j in bot.findall('joint')}
    assert {'left_wheel_joint', 'right_wheel_joint'} <= joints


def test_world_is_deterministic_and_matches_the_rca_sim_arena():
    """The Gazebo arena is generated from rca_sim's arena, so the two backends
    share the same scene geometry (conceptual parity, not identical physics)."""
    pytest.importorskip('rca_sim')
    from rca_sim.world import World
    w = World.default_arena(7)
    world = ET.parse(WORLD).getroot().find('world')
    obstacles = [m for m in world.findall('model') if (m.get('name') or '').startswith('obstacle_')]
    assert len(obstacles) == len(w.obstacles)
    for m, ob in zip(obstacles, w.obstacles):
        x, y = (float(v) for v in m.findtext('pose').split()[:2])
        assert abs(x - ob.x) < 1e-3 and abs(y - ob.y) < 1e-3


# ------------------------------------------------------------------ bridge

def test_every_bridged_topic_is_documented_and_namespaced():
    with open(BRIDGE) as f:
        entries = yaml.safe_load(f)
    assert entries, 'bridge configuration must not be empty'
    for e in entries:
        assert e['ros_topic_name'].startswith('/gz/'), \
            'raw simulator transport must stay inside the excluded /gz namespace'
        assert e['direction'] in ('GZ_TO_ROS', 'ROS_TO_GZ', 'BIDIRECTIONAL')
        assert e['ros_type_name'].count('/') == 2 and e['gz_type_name'].startswith('gz.msgs.')
    ros_topics = {e['ros_topic_name'] for e in entries}
    assert {'/gz/scan', '/gz/image', '/gz/imu', '/gz/odom', '/gz/cmd_vel'} <= ros_topics
    cmd = next(e for e in entries if e['ros_topic_name'] == '/gz/cmd_vel')
    assert cmd['direction'] == 'ROS_TO_GZ'


def test_raw_bridge_namespace_is_excluded_from_monitoring_but_app_topics_are_not():
    from diagnostic_monitor.graph_engine import RosGraphDiscoverer
    try:
        RosGraphDiscoverer.configure(topic_prefixes=['/gz/'], node_names=[])
        assert RosGraphDiscoverer.should_ignore_topic('/gz/scan')
        assert not RosGraphDiscoverer.should_ignore_topic('/sim/lidar/scan')
        assert not RosGraphDiscoverer.should_ignore_topic('/perception/obstacles')
        assert not RosGraphDiscoverer.should_ignore_node('lidar_node')
    finally:
        RosGraphDiscoverer.configure()


# --------------------------------------------------------------- scenarios

def test_scenarios_declare_the_required_fault_families():
    cfg = _scenarios()
    types = {f['type'] for sc in cfg['scenarios'] for f in sc['faults']}
    required = {'latency', 'dropout', 'processing_delay', 'localization_failure',
                'navigation_failure', 'control_failure', 'crash', 'stop', 'loss',
                'stale', 'noise', 'rate', 'topic_change'}
    assert required <= types, f'missing fault mechanisms: {sorted(required - types)}'
    assert len(cfg['scenarios']) >= 16


def test_scenario_ground_truth_is_well_formed_and_self_consistent():
    cfg = _scenarios()
    keys = [sc['key'] for sc in cfg['scenarios']]
    assert len(keys) == len(set(keys)), 'scenario keys must be unique'
    graph_nodes = set(cfg['environment']['static_design_graph']['nodes'])
    for sc in cfg['scenarios']:
        gt = sc['ground_truth']
        for f in sc['faults']:
            assert f['target'] in graph_nodes, f"{sc['key']}: unknown target {f['target']}"
        if gt.get('expect_no_diagnosis'):
            assert gt.get('root') is None and not gt.get('accepted_roots')
            continue
        assert gt['root'] in gt['accepted_roots']
        assert set(gt['accepted_roots']) <= {f['target'] for f in sc['faults']}, \
            f"{sc['key']}: accepted roots must be injected targets"
        assert gt['root'] in gt['observable_chain']
        assert set(gt['observable_chain']) <= set(gt['physical_chain']), \
            f"{sc['key']}: the observable chain must be a subset of the physical chain"
        assert set(gt['physical_chain']) <= graph_nodes


def test_static_design_graph_is_only_used_by_the_baselines():
    """The static graph lives in the evaluation config, not in the monitor."""
    cfg = _scenarios()
    edges = cfg['environment']['static_design_graph']['edges']
    assert {(e['from'], e['to']) for e in edges} >= {
        ('lidar_node', 'perception_node'), ('camera_node', 'perception_node'),
        ('perception_node', 'localization_node'), ('localization_node', 'planning_node'),
        ('planning_node', 'control_node'), ('control_node', 'sim_world')}


def test_ground_truth_never_reaches_the_monitor_or_the_tui():
    """No component on the diagnostic side may read the Gazebo scenario file or
    any ground-truth field."""
    import diagnostic_monitor
    diag_dir = os.path.dirname(os.path.abspath(diagnostic_monitor.__file__))
    forbidden = ('scenarios.yaml', 'ground_truth', 'accepted_roots', 'observable_chain',
                 'physical_chain', 'expect_no_diagnosis')
    offenders = []
    for base in (diag_dir, os.path.join(os.path.dirname(os.path.dirname(PKG)), 'diagnostic_tui', 'src')):
        if not os.path.isdir(base):
            continue
        for root, _, files in os.walk(base):
            for fn in files:
                if not fn.endswith(('.py', '.cpp', '.hpp')):
                    continue
                path = os.path.join(root, fn)
                # evaluator.py is the evaluation layer and legitimately names the concepts
                if os.path.basename(path) in ('evaluator.py',):
                    continue
                text = open(path, errors='ignore').read()
                for word in forbidden:
                    if word in text:
                        offenders.append(f'{path}: {word}')
    assert not offenders, offenders


# ------------------------------------------------------------------ launch

def test_launch_files_are_importable_and_declare_their_arguments():
    import importlib.util
    for name, expected in (
        ('gazebo_sim.launch.py', {'world', 'gui', 'seed', 'camera_enabled', 'respawn_delay'}),
        ('gazebo_system.launch.py', {'world', 'gui', 'seed', 'camera_enabled', 'respawn_delay', 'db_path'}),
    ):
        path = os.path.join(PKG, 'launch', name)
        spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
        declared = {a.name for a in ld.entities if hasattr(a, 'name') and hasattr(a, 'default_value')}
        assert expected <= declared, f'{name}: missing {expected - declared}'


def test_monitor_launch_is_told_the_backend_and_nothing_about_the_scenario():
    text = open(os.path.join(PKG, 'launch', 'gazebo_system.launch.py')).read()
    assert "'simulator': 'gazebo'" in text
    assert "'excluded_topic_prefixes': ['/gz/']" in text
    for word in ('ground_truth', 'root_cause', 'accepted_roots', 'scenarios.yaml'):
        assert word not in text
