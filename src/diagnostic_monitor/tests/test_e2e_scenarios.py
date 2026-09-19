"""End-to-end integration test scenarios for dynamic RCA."""

import time
import pytest
from diagnostic_monitor.anomaly_detector import AnomalyDetector
from diagnostic_monitor.evaluator import RCAEvaluator
from diagnostic_monitor.explainer import Explainer
from diagnostic_monitor.graph_engine import DependencyGraph
from diagnostic_monitor.rca_engine import RCAEngine


def test_scenario_sensor_latency_cascade():
    """Scenario 1: Sensor latency cascades to perception, localization, navigation."""
    # 1. Dynamically constructed graph
    graph = DependencyGraph()
    graph.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    graph.add_edge('perception_node', 'localization_node', '/perception/obstacles')
    graph.add_edge('localization_node', 'navigation_node', '/localization/pose')

    # 2. Ingest telemetry through AnomalyDetector
    detector = AnomalyDetector(warmup_duration_sec=0.1, z_threshold=3.0, confirm_samples=1)
    detector.start_time = 100.0

    # Train baselines
    t = 100.0
    for i in range(25):
        t += 0.1
        detector.process_metric(t, 'sensor_node', '/sensor/scan', 'latency', 0.05)
        detector.process_metric(t, 'perception_node', '/perception/obstacles', 'inter_arrival', 0.1)
        detector.process_metric(t, 'localization_node', '/localization/pose', 'inter_arrival', 0.1)
        detector.process_metric(t, 'navigation_node', '/navigation/cmd_vel', 'inter_arrival', 0.1)

    # Inject real delay on sensor_node at t=105.0s
    anomalies = []
    # Sensor node exhibits high latency
    a1 = detector.process_metric(105.0, 'sensor_node', '/sensor/scan', 'latency', 0.45)
    if a1:
        anomalies.append(a1)

    # Perception node callback triggered late -> inter-arrival spike at t=105.3s
    a2 = detector.process_metric(105.3, 'perception_node', '/perception/obstacles', 'inter_arrival', 0.45)
    if a2:
        anomalies.append(a2)

    # Localization node starves -> timeout at t=105.6s
    a3 = detector.process_metric(105.6, 'localization_node', '/localization/pose', 'inter_arrival', 0.50)
    if a3:
        anomalies.append(a3)

    # Navigation emergency stop at t=105.9s
    a4 = detector.process_metric(105.9, 'navigation_node', '/navigation/cmd_vel', 'inter_arrival', 0.50)
    if a4:
        anomalies.append(a4)

    assert len(anomalies) >= 3

    # 3. RCA diagnosis
    rca = RCAEngine(w_d=0.30, w_t=0.30, w_s=0.25, w_a=0.15)
    diag = rca.diagnose(anomalies, graph, current_time=106.0)

    assert diag.root_cause == 'sensor_node'
    assert diag.confidence > 0.80

    # Verify propagation chain
    chain, chain_nodes = Explainer.build_propagation_chain(diag.root_cause, graph, anomalies)
    assert len(chain) >= 3
    assert 'sensor_node' in chain[0]
    assert chain_nodes[0] == 'sensor_node' and len(chain_nodes) >= 3

    # Verify evaluation metrics
    eval_res = RCAEvaluator.evaluate_single(diag, ground_truth_node='sensor_node', fault_injection_time=105.0)
    assert eval_res['top1_correct'] is True
    assert eval_res['reciprocal_rank'] == 1.0


def test_scenario_localization_fault():
    """Scenario 2: Direct localization failure. Sensor and perception must remain nominal."""
    graph = DependencyGraph()
    graph.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    graph.add_edge('perception_node', 'localization_node', '/perception/obstacles')
    graph.add_edge('localization_node', 'navigation_node', '/localization/pose')

    detector = AnomalyDetector(warmup_duration_sec=0.1, z_threshold=3.0, confirm_samples=1)
    detector.start_time = 200.0

    # Train baselines
    t = 200.0
    for i in range(25):
        t += 0.1
        detector.process_metric(t, 'sensor_node', '/sensor/scan', 'latency', 0.05)
        detector.process_metric(t, 'perception_node', '/perception/obstacles', 'inter_arrival', 0.1)
        detector.process_metric(t, 'localization_node', '/localization/pose', 'z_pos', 0.0)
        detector.process_metric(t, 'navigation_node', '/navigation/cmd_vel', 'inter_arrival', 0.1)

    anomalies = []
    # Direct fault at localization_node: z_pos jumps to 99.0
    a1 = detector.process_metric(205.0, 'localization_node', '/localization/pose', 'z_pos', 99.0)
    if a1:
        anomalies.append(a1)

    # Cascades to navigation at 205.4s
    a2 = detector.process_metric(205.4, 'navigation_node', '/navigation/cmd_vel', 'inter_arrival', 0.50)
    if a2:
        anomalies.append(a2)

    assert len(anomalies) == 2

    rca = RCAEngine(w_d=0.30, w_t=0.30, w_s=0.25, w_a=0.15)
    diag = rca.diagnose(anomalies, graph, current_time=206.0)

    # Must accurately pinpoint localization_node, NOT sensor_node
    assert diag.root_cause == 'localization_node'
    eval_res = RCAEvaluator.evaluate_single(diag, ground_truth_node='localization_node', fault_injection_time=205.0)
    assert eval_res['top1_correct'] is True
