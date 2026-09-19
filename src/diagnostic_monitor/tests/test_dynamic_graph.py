"""Unit tests for DependencyGraph and RosGraphDiscoverer filtering."""

import pytest
from diagnostic_monitor.graph_engine import DependencyGraph, RosGraphDiscoverer


def test_graph_construction_and_traversal():
    g = DependencyGraph()
    # Pipeline: sensor -> perception -> localization -> navigation
    g.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    g.add_edge('perception_node', 'localization_node', '/perception/obstacles')
    g.add_edge('localization_node', 'navigation_node', '/localization/pose')

    assert 'sensor_node' in g.nodes
    assert 'navigation_node' in g.nodes
    assert len(g.nodes) == 4

    # Successors / Predecessors
    assert g.successors('sensor_node') == {'perception_node'}
    assert g.predecessors('navigation_node') == {'localization_node'}

    # Descendants
    assert g.descendants('sensor_node') == {'perception_node', 'localization_node', 'navigation_node'}
    assert g.descendants('localization_node') == {'navigation_node'}
    assert g.descendants('navigation_node') == set()

    # Ancestors
    assert g.ancestors('navigation_node') == {'localization_node', 'perception_node', 'sensor_node'}
    assert g.ancestors('sensor_node') == set()


def test_graph_paths_and_cycle_safety():
    g = DependencyGraph()
    g.add_edge('A', 'B', '/topic1')
    g.add_edge('B', 'C', '/topic2')
    g.add_edge('C', 'A', '/feedback')  # Cycle

    paths = g.find_all_paths('A', 'C')
    assert len(paths) == 1
    assert paths[0] == ['A', 'B', 'C']


def test_node_removal_on_crash():
    g = DependencyGraph()
    g.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    g.add_edge('perception_node', 'localization_node', '/perception/obstacles')

    assert 'perception_node' in g.nodes
    g.remove_node('perception_node')
    assert 'perception_node' not in g.nodes
    assert g.successors('sensor_node') == set()
    assert g.predecessors('localization_node') == set()


def test_discoverer_topic_and_node_filtering():
    # Verify strict ground-truth isolation: fault commands & internals are ignored
    assert RosGraphDiscoverer.should_ignore_topic('/rosout')
    assert RosGraphDiscoverer.should_ignore_topic('/parameter_events')
    assert RosGraphDiscoverer.should_ignore_topic('/rca/fault_command')
    assert RosGraphDiscoverer.should_ignore_topic('/rca/eval_results')
    assert RosGraphDiscoverer.should_ignore_topic('/rca/diagnosis_report')

    # Valid application topics must NOT be ignored
    assert not RosGraphDiscoverer.should_ignore_topic('/sensor/scan')
    assert not RosGraphDiscoverer.should_ignore_topic('/perception/obstacles')
    assert not RosGraphDiscoverer.should_ignore_topic('/localization/pose')
    assert not RosGraphDiscoverer.should_ignore_topic('/navigation/cmd_vel')

    # Node filtering
    assert RosGraphDiscoverer.should_ignore_node('diagnostic_monitor')
    assert RosGraphDiscoverer.should_ignore_node('fault_injector')
    assert RosGraphDiscoverer.should_ignore_node('experiment_controller')
    assert RosGraphDiscoverer.should_ignore_node('_ros2cli_daemon')
    assert not RosGraphDiscoverer.should_ignore_node('sensor_node')
