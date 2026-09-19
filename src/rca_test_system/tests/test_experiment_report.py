"""Guards against drift between the printed benchmark summary and the saved JSON."""

import json
import re

import pytest
import rclpy

from rca_test_system.experiment_controller import SCENARIOS, ExperimentController


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _fake_result(key, gt, pred, chain, expected, physical=None):
    from diagnostic_monitor.evaluator import RCAEvaluator
    from diagnostic_monitor.rca_engine import CandidateScore, DiagnosisResult
    diag = DiagnosisResult(timestamp=101.0, root_cause=pred, confidence=0.9,
                           candidates=[CandidateScore(pred, 0.9, 1, 1, 1, 1, 100.5)],
                           anomalous_nodes=set(chain), weights={}, propagation_chain=chain)
    r = RCAEvaluator.evaluate_single(diag, gt, fault_injection_time=100.0, expected_chain=expected,
                                     physical_chain=physical, first_anomaly_time=100.4,
                                     first_diagnosis_time=100.9, first_correct_diagnosis_time=100.9,
                                     diagnoses_after_injection=[pred, pred])
    r.update({'scenario': key, 'key': key, 'nominal_false_alarms': 0, 'nominal_window_sec': 1.0})
    return r


def test_printed_and_saved_aggregates_are_identical(ros_context, tmp_path, capsys):
    ctl = ExperimentController(db_path=str(tmp_path / 'e.db'))
    try:
        ctl.eval_results = [
            _fake_result('sensor_latency', 'sensor_node', 'sensor_node',
                         ['sensor_node', 'perception_node'], ['sensor_node', 'perception_node']),
            _fake_result('sensor_degradation', 'sensor_node', 'sensor_node',
                         ['sensor_node', 'perception_node'], ['sensor_node', 'perception_node'],
                         physical=['sensor_node', 'perception_node', 'localization_node']),
        ]
        ctl.nominal_windows = [
            {'name': 'warmup', 't_start': 0.0, 't_end': 15.0, 'duration_sec': 15.0, 'anomalies': 0, 'diagnoses': 0},
            {'name': 'nominal_soak', 't_start': 15.0, 't_end': 45.0, 'duration_sec': 30.0, 'anomalies': 1, 'diagnoses': 1},
        ]
        printed_agg = ctl.print_summary()
        out = capsys.readouterr().out
        # The JSON block printed by print_summary must parse and equal what save() writes
        block = re.search(r'\{\n.*?\n\}', out, re.S).group(0)
        parsed_printed = json.loads(block)
        ctl.save(str(tmp_path / 'r.json'))
        saved = json.load(open(tmp_path / 'r.json'))
        assert parsed_printed == saved['aggregate'] == printed_agg
        # nominal fields must be present and consistent with the recorded windows
        assert saved['aggregate']['nominal_false_alarms'] == 1
        assert saved['aggregate']['nominal_fault_free_sec'] == 45.0
        assert saved['aggregate']['nominal_anomalies'] == 1
        assert saved['nominal_windows'] == ctl.nominal_windows
        # physical vs observable chain reporting
        assert saved['aggregate']['avg_chain_accuracy'] == 1.0
        assert saved['aggregate']['avg_physical_chain_coverage'] < 1.0
        assert saved['aggregate']['scenarios_with_unobserved_physical_nodes'] == ['sensor_degradation']
        assert 'PHYSICALLY AFFECTED BUT UNOBSERVED' in out
    finally:
        ctl.store.close()
        ctl.destroy_node()


def test_record_nominal_window_counts_from_store(ros_context, tmp_path):
    ctl = ExperimentController(db_path=str(tmp_path / 'e.db'))
    try:
        ctl.store.insert_anomaly(10.0, 'n', '/t', 'm', 1, 0, 1, 3, 0.5, 'x')
        ctl.store.insert_diagnosis(10.5, 'n', 0.9, 0.9, {}, [], 'e')
        w = ctl.record_nominal_window('soak', 9.0, 12.0)
        assert w['duration_sec'] == 3.0 and w['anomalies'] == 1 and w['diagnoses'] == 1
        assert w['anomalous_nodes'] == ['n'] and w['diagnosed_roots'] == ['n']
        outside = ctl.record_nominal_window('later', 20.0, 25.0)
        assert outside['anomalies'] == 0 and outside['diagnoses'] == 0
        assert ctl.compute_aggregate()['nominal_false_alarms'] == 1
    finally:
        ctl.store.close()
        ctl.destroy_node()


def test_scenarios_declare_physical_chain_only_where_it_differs():
    sc = {s['key']: s for s in SCENARIOS}
    assert sc['sensor_degradation']['physical_chain'] == ['sensor_node', 'perception_node', 'localization_node']
    assert sc['sensor_degradation']['expected_chain'] == ['sensor_node', 'perception_node']
    for k, s in sc.items():
        if 'physical_chain' in s:
            assert set(s['expected_chain']) <= set(s['physical_chain'])
