"""Unit tests for RCA scoring engine, mathematical formulation, and ablation."""

import pytest
from diagnostic_monitor.anomaly_detector import Anomaly
from diagnostic_monitor.graph_engine import DependencyGraph
from diagnostic_monitor.rca_engine import RCAEngine


@pytest.fixture
def test_graph():
    g = DependencyGraph()
    g.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    g.add_edge('perception_node', 'localization_node', '/perception/obstacles')
    g.add_edge('localization_node', 'navigation_node', '/localization/pose')
    return g


def test_sensor_fault_cascade_diagnosis(test_graph):
    # Cascading failure: sensor fails first, then perception, localization, navigation
    t0 = 100.0
    anomalies = [
        Anomaly(t0 + 0.0, 'sensor_node', '/sensor/scan', 'latency', 0.45, 0.1, 0.02, 17.5, 1.0, 'high latency'),
        Anomaly(t0 + 0.3, 'perception_node', '/perception/obstacles', 'inter_arrival', 0.42, 0.1, 0.02, 16.0, 0.9, 'delayed'),
        Anomaly(t0 + 0.6, 'localization_node', '/localization/pose', 'message_timeout', 0.60, 0.1, 0.02, 25.0, 0.9, 'timeout'),
        Anomaly(t0 + 0.9, 'navigation_node', '/navigation/cmd_vel', 'emergency_stop', 0.0, 0.5, 0.05, 10.0, 0.8, 'stop'),
    ]

    rca = RCAEngine(w_d=0.30, w_t=0.30, w_s=0.25, w_a=0.15, time_window_sec=5.0)
    diag = rca.diagnose(anomalies, test_graph, current_time=t0 + 1.5)

    assert diag.root_cause == 'sensor_node'
    assert diag.confidence > 0.85

    # Check that sensor_node achieves highest score across all terms
    sensor_score = next(c for c in diag.candidates if c.node == 'sensor_node')
    assert sensor_score.d_score == 1.0  # reaches 3 downstream, 0 upstream
    assert sensor_score.t_score == 1.0  # earliest failure
    assert sensor_score.s_score == 1.0  # covers all 4 anomalous nodes


def test_localization_fault_diagnosis(test_graph):
    # Fault originating at localization_node: sensor and perception are healthy!
    t0 = 200.0
    anomalies = [
        Anomaly(t0 + 0.0, 'localization_node', '/localization/pose', 'drift', 55.0, 0.0, 1.0, 55.0, 1.0, 'sudden jump'),
        Anomaly(t0 + 0.4, 'navigation_node', '/navigation/cmd_vel', 'emergency_stop', 0.0, 0.5, 0.05, 10.0, 0.8, 'stop'),
    ]

    rca = RCAEngine(w_d=0.30, w_t=0.30, w_s=0.25, w_a=0.15, time_window_sec=5.0)
    diag = rca.diagnose(anomalies, test_graph, current_time=t0 + 1.0)

    # Must diagnose localization_node as root cause, NOT sensor_node
    assert diag.root_cause == 'localization_node'

    loc_score = next(c for c in diag.candidates if c.node == 'localization_node')
    assert loc_score.d_score == 1.0  # reached navigation_node, 0 upstream anomalous parents
    assert loc_score.t_score == 1.0
    assert loc_score.s_score == 1.0  # covers localization + navigation

    # Sensor node is a candidate via ancestor graph, but has no anomalies, so score is much lower
    sensor_score = next(c for c in diag.candidates if c.node == 'sensor_node')
    assert sensor_score.total_score < loc_score.total_score


def test_symptom_coverage_formula(test_graph):
    t0 = 300.0
    anomalies = [
        Anomaly(t0, 'perception_node', '/perception/obstacles', 'err', 1.0, 0.0, 0.1, 10.0, 0.8, 'err'),
        Anomaly(t0 + 0.1, 'localization_node', '/localization/pose', 'err', 1.0, 0.0, 0.1, 10.0, 0.8, 'err'),
        Anomaly(t0 + 0.2, 'navigation_node', '/navigation/cmd_vel', 'err', 1.0, 0.0, 0.1, 10.0, 0.8, 'err'),
    ]

    rca = RCAEngine(w_d=0.25, w_t=0.25, w_s=0.25, w_a=0.25)
    diag = rca.diagnose(anomalies, test_graph, current_time=t0 + 1.0)

    # perception reaches perception, localization, navigation -> 3 / 3 = 1.0
    p_score = next(c for c in diag.candidates if c.node == 'perception_node')
    assert p_score.s_score == 1.0

    # localization reaches localization, navigation -> 2 / 3 = 0.6667
    l_score = next(c for c in diag.candidates if c.node == 'localization_node')
    assert pytest.approx(l_score.s_score, rel=1e-2) == 2.0 / 3.0

    # navigation reaches navigation -> 1 / 3 = 0.3333
    n_score = next(c for c in diag.candidates if c.node == 'navigation_node')
    assert pytest.approx(n_score.s_score, rel=1e-2) == 1.0 / 3.0


def test_ablation_support(test_graph):
    t0 = 400.0
    anomalies = [
        Anomaly(t0 + 0.5, 'sensor_node', '/sensor/scan', 'err', 1.0, 0.0, 0.1, 10.0, 0.5, 'err'),
        Anomaly(t0 + 0.0, 'navigation_node', '/navigation/cmd_vel', 'err', 1.0, 0.0, 0.1, 10.0, 1.0, 'err'),
    ]

    # Ablation 1: Only Temporal (wD=0, wS=0, wA=0, wT=1)
    rca_t_only = RCAEngine(w_d=0.0, w_t=1.0, w_s=0.0, w_a=0.0)
    diag_t = rca_t_only.diagnose(anomalies, test_graph, current_time=t0 + 1.0)
    # Under temporal only, navigation_node failed first at t0+0.0
    assert diag_t.root_cause == 'navigation_node'

    # Ablation 2: Only Dependency & Symptom (wT=0, wA=0, wD=0.5, wS=0.5)
    rca_topo_only = RCAEngine(w_d=0.5, w_t=0.0, w_s=0.5, w_a=0.0)
    diag_topo = rca_topo_only.diagnose(anomalies, test_graph, current_time=t0 + 1.0)
    # Under topological only, sensor_node is upstream of navigation_node
    assert diag_topo.root_cause == 'sensor_node'


def test_insufficient_evidence_handling(test_graph):
    rca = RCAEngine()
    diag = rca.diagnose([], test_graph, current_time=100.0)
    assert diag.root_cause is None
    assert diag.confidence == 0.0
    assert len(diag.candidates) == 0
