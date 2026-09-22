"""Unit tests for the rca_sim simulation primitives and scenario definitions."""

import json
import math
import os
import random
import re
import time

import pytest
import yaml

from rca_sim.degradation import ALL_FAULTS, CpuPressure, DegradableLink, FaultState, busy_work
from rca_sim.world import Obstacle, World, waypoint_loop, wrap_angle

HERE = os.path.dirname(os.path.abspath(__file__))
SCENARIOS = os.path.join(HERE, '..', 'config', 'scenarios.yaml')
MONITOR_SRC = os.path.join(HERE, '..', '..', 'diagnostic_monitor', 'diagnostic_monitor')
TUI_SRC = os.path.join(HERE, '..', '..', 'diagnostic_tui')


# ------------------------------------------------------------------ world

def test_world_is_deterministic_and_bounded():
    a, b = World.default_arena(seed=3), World.default_arena(seed=3)
    assert [(o.x, o.y, o.r) for o in a.obstacles] == [(o.x, o.y, o.r) for o in b.obstacles]
    assert len(a.obstacles) == 12 and all(math.hypot(o.x, o.y) > 2.5 for o in a.obstacles)
    for _ in range(500):
        a.step(0.02, 0.8, 0.0)
    assert abs(a.robot.x) <= a.width / 2 and abs(a.robot.y) <= a.height / 2
    assert not a.collides(a.robot.x, a.robot.y)


def test_ray_cast_hits_walls_and_obstacles():
    w = World(obstacles=[])
    w.robot.x = w.robot.y = 0.0
    assert abs(w.ray_cast(0.0, 50.0) - 10.0) < 1e-9            # east wall at 10 m
    assert abs(w.ray_cast(math.pi / 2, 50.0) - 10.0) < 1e-9
    w.obstacles.append(Obstacle(4.0, 0.0, 1.0))
    assert abs(w.ray_cast(0.0, 50.0) - 3.0) < 1e-9             # circle edge at 3 m
    assert w.ray_cast(0.0, 2.0) == 2.0                           # clipped to max range
    scan = w.lidar_scan(90, math.pi, 15.0)
    assert len(scan) == 90 and min(scan) > 0 and max(scan) <= 15.0
    assert w.camera_detections(math.radians(70), 7.0) == [(4.0, 0.0)]


def test_velocity_limits_and_angle_wrap():
    w = World(obstacles=[])
    for _ in range(100):
        w.step(0.02, 10.0, 10.0)
    assert w.robot.v <= w.max_v + 1e-9 and w.robot.w <= w.max_w + 1e-9
    assert -math.pi <= w.robot.theta <= math.pi
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9 or abs(wrap_angle(3 * math.pi) + math.pi) < 1e-9
    assert len(waypoint_loop(5.0, 8)) == 8


# ------------------------------------------------------------ degradation

def test_fault_state_parse_apply_expire():
    f = FaultState('lidar_node')
    assert f.parse('not json') is None
    assert f.parse(json.dumps({'target_node': 'camera_node', 'fault_type': 'noise'})) is None
    cmd = f.parse(json.dumps({'target_node': 'lidar_node', 'fault_type': 'noise', 'param': 5, 'duration': 2}))
    assert f.apply(cmd, now=100.0) == 'noise'
    assert f.is_active('noise') and f.param('noise') == 5.0 and not f.is_active('bias')
    assert f.expire(101.0) == [] and f.expire(102.5) == ['noise'] and not f.is_active('noise')
    f.apply(f.parse(json.dumps({'target_node': 'all', 'fault_type': 'stale', 'param': 0, 'duration': 1})), 0.0)
    assert f.param('stale', 1.0) == 1.0                          # param 0 -> default
    assert f.apply({'fault_type': 'clear'}, 0.0) == 'clear' and f.types() == []


def test_degradable_link_delay_loss_rate_are_real():
    sent = []
    faults = FaultState('n')
    link = DegradableLink(sent.append, faults, random.Random(1))
    link.publish('a', now=10.0)
    assert link.drain(10.0) == 1 and sent == ['a']
    faults.apply({'fault_type': 'latency', 'param': 0.5, 'duration': 100}, 10.0)
    link.publish('b', now=10.0)
    assert link.drain(10.4) == 0 and link.pending() == 1         # not before its dispatch time
    assert link.drain(10.5) == 1 and sent[-1] == 'b'
    faults.apply({'fault_type': 'clear'}, 10.0)
    faults.apply({'fault_type': 'loss', 'param': 1.0, 'duration': 100}, 10.0)
    link.publish('c', now=11.0)
    assert link.pending() == 0 and link.dropped == 1
    faults.apply({'fault_type': 'clear'}, 10.0)
    faults.apply({'fault_type': 'rate', 'param': 0.25, 'duration': 100}, 10.0)
    for i in range(40):
        link.publish(i, now=12.0)
    assert link.pending() == 10                                   # quarter rate keeps every 4th
    faults.apply({'fault_type': 'clear'}, 10.0)
    faults.apply({'fault_type': 'jitter', 'param': 0.2, 'duration': 100}, 10.0)
    link.clear()
    for i in range(20):
        link.publish(i, now=20.0)
    assert link.drain(20.0) < 20 and link.drain(25.0) + 0 >= 0    # jittered messages arrive later


def test_busy_work_and_cpu_pressure_consume_real_time():
    t0 = time.perf_counter()
    busy_work(30.0)
    assert time.perf_counter() - t0 >= 0.028
    cp = CpuPressure()
    cp.start(1)
    assert cp.running
    cp.stop()
    assert not cp.running


# ---------------------------------------------------------- scenarios

def _load():
    with open(SCENARIOS) as f:
        return yaml.safe_load(f)


def test_scenario_file_is_well_formed():
    cfg = _load()
    keys = [s['key'] for s in cfg['scenarios']]
    assert len(keys) == len(set(keys)) and len(keys) >= 16
    groups = {s['group'] for s in cfg['scenarios']}
    assert {'single', 'communication', 'workload', 'dynamic_graph', 'compound', 'ambiguous'} <= groups
    assert sum(1 for s in cfg['scenarios'] if s['group'] in ('compound', 'ambiguous')) >= 3
    known_nodes = set(cfg['environment']['static_design_graph']['nodes'])
    for s in cfg['scenarios']:
        gt = s['ground_truth']
        for ft in s['faults']:
            assert ft['type'] in ALL_FAULTS, (s['key'], ft['type'])
            assert ft['target'] in known_nodes, (s['key'], ft['target'])
        if gt.get('expect_no_diagnosis'):
            assert gt['root'] is None and gt['accepted_roots'] == []
        else:
            assert gt['root'] in gt['accepted_roots'] and gt['root'] in known_nodes
            assert gt['observable_chain'][0] == gt['root']
            assert set(gt['observable_chain']) <= set(gt['physical_chain']), s['key']
            injected = {ft['target'] for ft in s['faults']}
            assert set(gt['accepted_roots']) == injected
    assert {d['key'] for d in cfg['demos']} <= set(keys) and len(cfg['demos']) == 3


def test_ground_truth_never_reaches_monitor_or_tui():
    """The scenario file and its ground-truth vocabulary must not be referenced by
    the diagnostic monitor runtime code or the TUI."""
    terms = ('scenarios.yaml', 'accepted_roots', 'observable_chain', 'physical_chain', 'rca_sim')
    for root in (MONITOR_SRC, TUI_SRC):
        for dirpath, _, files in os.walk(root):
            if 'test' in dirpath or '_deps' in dirpath or 'build' in dirpath:
                continue
            for fn in files:
                if fn.endswith(('.py', '.cpp', '.hpp')) and fn != 'evaluator.py':
                    txt = open(os.path.join(dirpath, fn), encoding='utf-8', errors='ignore').read()
                    for t in terms:
                        assert t not in txt, f'{fn} references {t}'


def test_rank_of_any_and_loader_defaults():
    from rca_sim.scenario_runner import load_scenarios, rank_of_any
    assert rank_of_any(['a', 'b', 'c'], ['c', 'b']) == 2
    assert rank_of_any(['a'], ['z']) == -1
    assert rank_of_any(['a'], []) == -1
    cfg = load_scenarios(SCENARIOS)
    sc = {s['key']: s for s in cfg['scenarios']}
    assert sc['lidar_noise']['faults'][0]['duration'] == cfg['defaults']['fault_duration']
    assert sc['perception_crash']['observation_sec'] == 10.0
    assert sc['planning_topic_change']['ground_truth']['expect_no_diagnosis'] is True
