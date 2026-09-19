"""Tests for baseline methods, ablation and the extended evaluator."""

from diagnostic_monitor.anomaly_detector import Anomaly
from diagnostic_monitor.baselines import ABLATION_CONFIGS, BASELINE_NAMES, run_ablations, run_all_baselines
from diagnostic_monitor.evaluator import RCAEvaluator
from diagnostic_monitor.explainer import Explainer
from diagnostic_monitor.graph_engine import DependencyGraph
from diagnostic_monitor.rca_engine import RCAEngine


def _chain():
    g = DependencyGraph()
    g.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    g.add_edge('perception_node', 'localization_node', '/perception/obstacles')
    g.add_edge('localization_node', 'navigation_node', '/localization/pose')
    return g


def test_baselines_disagree_when_evidence_is_misleading():
    """Navigation shows the earliest and most severe symptom, but sensor explains everything."""
    g = _chain()
    t0 = 100.0
    anoms = [
        Anomaly(t0 + 0.30, 'sensor_node', '/sensor/scan', 'latency', 0.4, 0, 0.01, 10, 0.6, ''),
        Anomaly(t0 + 0.35, 'perception_node', '/perception/obstacles', 'latency', 0.4, 0, 0.01, 10, 0.6, ''),
        Anomaly(t0 + 0.40, 'localization_node', '/localization/pose', 'latency', 0.4, 0, 0.01, 10, 0.6, ''),
        Anomaly(t0 + 0.00, 'navigation_node', '/navigation/cmd_vel', 'message_timeout', 0.5, 0.1, 0.02, 5, 1.0, ''),
    ]
    res = run_all_baselines(anoms, g, g, current_time=t0 + 1.0)
    assert set(res) == set(BASELINE_NAMES)
    assert res['independent'][0] == 'navigation_node'      # most severe
    assert res['temporal_only'][0] == 'navigation_node'    # earliest
    # temporal-only must rank by physical onset, not by record time
    late_record = [Anomaly(t0 + 5.0, 'sensor_node', '/sensor/scan', 'message_timeout', 1, 0, 1, 5, 0.9, '',
                           onset_time=t0 - 1.0)] + anoms[1:]
    assert run_all_baselines(late_record, g, g, current_time=t0 + 6.0)['temporal_only'][0] == 'sensor_node'
    assert res['proposed'][0] == 'sensor_node'             # explains all symptoms


def test_static_graph_baseline_is_blind_to_runtime_topology():
    """Static baselines use the design-time chain; if the runtime graph differs
    (here: localization consumes the sensor directly), only the dynamic method adapts."""
    static = _chain()
    dynamic = DependencyGraph()
    dynamic.add_edge('sensor_node', 'localization_node', '/sensor/scan')
    dynamic.add_edge('localization_node', 'navigation_node', '/localization/pose')
    dynamic.add_node('perception_node')
    t0 = 10.0
    anoms = [
        Anomaly(t0 + 0.0, 'localization_node', '/localization/pose', 'z_pos', 99, 0, 0.01, 100, 1.0, ''),
        Anomaly(t0 + 0.1, 'navigation_node', '/navigation/cmd_vel', 'linear_vel', 0, 0.5, 0.01, 50, 1.0, ''),
    ]
    res = run_all_baselines(anoms, dynamic, static, current_time=t0 + 1.0)
    assert res['proposed'][0] == 'localization_node'
    # static+temporal: perception_node is an (incorrect) ancestor candidate on the static graph
    assert 'perception_node' in res['static_dep_temp']
    assert 'perception_node' not in res['proposed']


def test_ablation_configs_disable_terms():
    g = _chain()
    t0 = 0.0
    anoms = [
        Anomaly(t0 + 0.5, 'sensor_node', '/sensor/scan', 'err', 1, 0, 0.1, 10, 0.5, ''),
        Anomaly(t0 + 0.0, 'navigation_node', '/navigation/cmd_vel', 'err', 1, 0, 0.1, 10, 1.0, ''),
    ]
    ab = run_ablations(anoms, g, current_time=t0 + 1.0, time_window_sec=5.0)
    assert set(ab) == set(ABLATION_CONFIGS)
    assert ab['full'][0] == 'sensor_node'
    for cfg, w in ABLATION_CONFIGS.items():
        assert sum(w.values()) > 0


def test_evaluator_extended_metrics():
    g = _chain()
    t0 = 100.0
    anoms = [
        Anomaly(t0 + 0.2, 'sensor_node', '/sensor/scan', 'latency', 0.4, 0, 0.01, 10, 1.0, ''),
        Anomaly(t0 + 0.3, 'perception_node', '/perception/obstacles', 'latency', 0.4, 0, 0.01, 10, 1.0, ''),
    ]
    diag = RCAEngine().diagnose(anoms, g, t0 + 1.0)
    Explainer.explain(diag, g, anoms)
    assert diag.propagation_chain == ['sensor_node', 'perception_node']

    r = RCAEvaluator.evaluate_single(
        diag, 'sensor_node', fault_injection_time=t0, fault_type='latency', k=2,
        expected_chain=['sensor_node', 'perception_node', 'localization_node'],
        first_anomaly_time=t0 + 0.2, first_diagnosis_time=t0 + 0.5, first_correct_diagnosis_time=t0 + 0.5,
        diagnoses_after_injection=['perception_node', 'sensor_node', 'sensor_node', 'sensor_node'],
    )
    assert r['top1_correct'] and r['top2_correct'] and r['ground_truth_rank'] == 1
    assert r['detection_latency_sec'] == 0.2
    assert r['diagnosis_latency_sec'] == 0.5
    assert r['false_diagnosis_rate'] == 0.25
    assert abs(r['chain_accuracy'] - 2 / 3) < 1e-3  # stored rounded to 4 dp
    assert r['chain_order_correct'] is False  # localization missing

    second = dict(r, top1_correct=False, top2_correct=False, reciprocal_rank=0.5, false_diagnosis_rate=1.0)
    agg = RCAEvaluator.aggregate_metrics([r, second], k=2)
    assert agg['top1_accuracy'] == 0.5 and agg['top2_accuracy'] == 0.5
    assert agg['false_diagnosis_rate'] == 0.625  # (1 + 4) / 8
    assert agg['avg_detection_latency_sec'] == 0.2


def test_rank_of_missing():
    assert RCAEvaluator.rank_of(['a', 'b'], 'c') == -1
    assert RCAEvaluator.rank_of(['a', 'b'], 'b') == 2


def test_physical_vs_observable_chain_are_distinguished():
    """Observable chain drives chain_accuracy; physically affected but unobserved
    nodes are reported as uncovered and never credited."""
    m = RCAEvaluator.chain_metrics(
        predicted_chain=['sensor_node', 'perception_node'],
        expected_chain=['sensor_node', 'perception_node'],
        physical_chain=['sensor_node', 'perception_node', 'localization_node'],
    )
    assert m['chain_accuracy'] == 1.0 and m['chain_order_correct'] is True
    assert abs(m['physical_chain_coverage'] - 2 / 3) < 1e-3
    assert m['physically_affected_unobserved'] == ['localization_node']
    # Without a separate physical chain the two coincide
    m2 = RCAEvaluator.chain_metrics(['a', 'b'], ['a', 'b'])
    assert m2['physical_chain'] == ['a', 'b'] and m2['physical_chain_coverage'] == 1.0
    assert m2['physically_affected_unobserved'] == []


def test_nominal_metrics_and_aggregate_include_nominal_fields():
    windows = [
        {'name': 'warmup', 'duration_sec': 15.0, 'anomalies': 0, 'diagnoses': 0},
        {'name': 'nominal_soak', 'duration_sec': 45.0, 'anomalies': 2, 'diagnoses': 1},
    ]
    nm = RCAEvaluator.nominal_metrics(windows)
    assert nm['nominal_fault_free_sec'] == 60.0
    assert nm['nominal_anomalies'] == 2 and nm['nominal_diagnoses'] == 1
    assert nm['nominal_false_alarms'] == 1 and nm['nominal_false_alarms_per_min'] == 1.0
    agg = RCAEvaluator.aggregate_metrics([], k=3, nominal_windows=windows)
    for key in nm:
        assert agg[key] == nm[key]
    assert RCAEvaluator.aggregate_metrics([], k=3)['nominal_false_alarms'] == 0
    assert RCAEvaluator.aggregate_metrics([], k=3)['nominal_false_alarms_per_min'] is None
