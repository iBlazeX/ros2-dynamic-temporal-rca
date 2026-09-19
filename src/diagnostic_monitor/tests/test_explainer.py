"""Unit tests for Explainer and RCAEvaluator."""

import pytest
from diagnostic_monitor.anomaly_detector import Anomaly
from diagnostic_monitor.evaluator import RCAEvaluator
from diagnostic_monitor.explainer import Explainer
from diagnostic_monitor.graph_engine import DependencyGraph
from diagnostic_monitor.rca_engine import RCAEngine


def test_explainer_propagation_chain():
    g = DependencyGraph()
    g.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    g.add_edge('perception_node', 'localization_node', '/perception/obstacles')

    t0 = 100.0
    anomalies = [
        Anomaly(t0, 'sensor_node', '/sensor/scan', 'latency', 0.4, 0.1, 0.02, 15.0, 1.0, 'late scan'),
        Anomaly(t0 + 0.2, 'perception_node', '/perception/obstacles', 'inter_arrival', 0.5, 0.1, 0.02, 20.0, 0.9, 'drop'),
    ]

    rca = RCAEngine()
    diag = rca.diagnose(anomalies, g, current_time=t0 + 1.0)
    explanation = Explainer.explain(diag, g, anomalies)

    assert 'ROOT CAUSE DIAGNOSIS: sensor_node' in explanation
    assert 'Propagation Chain' in explanation
    assert 'sensor_node' in explanation
    assert 'perception_node' in explanation
    assert 'Score Breakdown' in explanation


def test_evaluator_metrics():
    g = DependencyGraph()
    g.add_edge('A', 'B', '/topic')
    anomalies = [
        Anomaly(10.0, 'A', '/topic', 'err', 1.0, 0.0, 0.1, 10.0, 1.0, 'err'),
        Anomaly(10.5, 'B', '/topic', 'err', 1.0, 0.0, 0.1, 10.0, 0.8, 'err'),
    ]
    rca = RCAEngine()
    diag = rca.diagnose(anomalies, g, current_time=11.0)

    # Correct evaluation
    eval_res = RCAEvaluator.evaluate_single(
        diagnosis=diag,
        ground_truth_node='A',
        fault_injection_time=9.5,
        fault_type='latency'
    )
    assert eval_res['top1_correct'] is True
    assert eval_res['top3_correct'] is True
    assert eval_res['ground_truth_rank'] == 1
    assert eval_res['reciprocal_rank'] == 1.0
    assert eval_res['detection_latency_sec'] == 1.5

    # Incorrect evaluation
    eval_wrong = RCAEvaluator.evaluate_single(
        diagnosis=diag,
        ground_truth_node='B',
        fault_injection_time=9.5,
        fault_type='latency'
    )
    assert eval_wrong['top1_correct'] is False
    assert eval_wrong['top3_correct'] is True
    assert eval_wrong['ground_truth_rank'] == 2
    assert eval_wrong['reciprocal_rank'] == 0.5
