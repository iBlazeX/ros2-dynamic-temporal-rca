"""Tests for the reporting contract: explicit denominators, cohorts, latency spread.

These cover the reporting defects fixed in the Gazebo/reporting pass:
  * the proposed method and the baselines must share one method-independent cohort
  * every rate must carry its numerator and denominator
  * "post-injection false-diagnosis rate" and "nominal false-alarm rate" are
    different quantities and must never collapse into one name
  * detection/diagnosis latency must be reported as a distribution, not a mean
  * chain accuracy and physical-chain coverage must be defined and separate
"""

import json
import math
import os

import pytest

from diagnostic_monitor.baselines import BASELINE_NAMES
from diagnostic_monitor.evaluator import RCAEvaluator, latency_stats


# ------------------------------------------------------------ latency spread

def test_latency_stats_reports_a_distribution_not_only_the_mean():
    st = latency_stats([0.1, 0.2, 0.3, 0.4, 0.5])
    assert st['n'] == 5
    assert st['mean'] == 0.3 and st['min'] == 0.1 and st['max'] == 0.5
    assert st['median'] == 0.3 and st['p25'] == 0.2 and st['p75'] == 0.4
    assert abs(st['std'] - math.sqrt(0.02)) < 1e-3   # std is stored rounded to 3 dp


def test_latency_stats_handles_empty_and_single_samples():
    empty = latency_stats([])
    assert empty['n'] == 0 and empty['mean'] is None and empty['std'] is None
    one = latency_stats([0.7])
    assert one['n'] == 1 and one['mean'] == one['min'] == one['max'] == one['median'] == 0.7
    assert one['std'] == 0.0
    assert latency_stats([0.5, None, 0.5])['n'] == 2      # None values are ignored


def test_aggregate_exposes_latency_distributions():
    runs = [
        {'top1_correct': True, 'top3_correct': True, 'reciprocal_rank': 1.0,
         'detection_latency_sec': 0.2, 'diagnosis_latency_sec': 0.6,
         'num_diagnoses_after_injection': 2, 'num_false_diagnoses_after_injection': 0},
        {'top1_correct': False, 'top3_correct': True, 'reciprocal_rank': 0.5,
         'detection_latency_sec': 1.0, 'diagnosis_latency_sec': 1.4,
         'num_diagnoses_after_injection': 2, 'num_false_diagnoses_after_injection': 2},
    ]
    agg = RCAEvaluator.aggregate_metrics(runs)
    assert agg['detection_latency_stats'] == {
        'n': 2, 'mean': 0.6, 'min': 0.2, 'max': 1.0, 'std': 0.4,
        'median': 0.6, 'p25': 0.4, 'p75': 0.8}
    assert agg['diagnosis_latency_stats']['max'] == 1.4
    assert agg['avg_detection_latency_sec'] == agg['detection_latency_stats']['mean']


# --------------------------------------------------------------- denominators

def test_every_headline_rate_carries_its_numerator_and_denominator():
    runs = [
        {'top1_correct': True, 'top3_correct': True, 'reciprocal_rank': 1.0,
         'num_diagnoses_after_injection': 3, 'num_false_diagnoses_after_injection': 1,
         'chain_accuracy': 1.0, 'chain_order_correct': True, 'physical_chain_coverage': 0.5},
        {'top1_correct': False, 'top3_correct': False, 'reciprocal_rank': 0.0,
         'num_diagnoses_after_injection': 1, 'num_false_diagnoses_after_injection': 1,
         'chain_accuracy': 0.0, 'chain_order_correct': False, 'physical_chain_coverage': 0.0},
    ]
    agg = RCAEvaluator.aggregate_metrics(runs)
    assert agg['top1_correct'] == 1 and agg['total_experiments'] == 2
    assert agg['top1_accuracy'] == 0.5
    assert agg['post_injection_false_diagnoses'] == 2
    assert agg['post_injection_diagnoses'] == 4
    assert agg['post_injection_false_diagnosis_rate'] == 0.5
    assert agg['false_diagnosis_rate'] == agg['post_injection_false_diagnosis_rate']
    assert agg['chain_accuracy_n'] == 2 and agg['physical_chain_coverage_n'] == 2
    assert agg['chain_order_correct_count'] == 1 and agg['chain_order_n'] == 2


def test_nominal_false_alarms_are_a_different_quantity_from_post_injection_fdr():
    windows = [{'name': 'soak', 'duration_sec': 120.0, 'anomalies': 3, 'diagnoses': 1},
               {'name': 'pre', 'duration_sec': 60.0, 'anomalies': 0, 'diagnoses': 0}]
    runs = [{'top1_correct': True, 'top3_correct': True, 'reciprocal_rank': 1.0,
             'num_diagnoses_after_injection': 4, 'num_false_diagnoses_after_injection': 0}]
    agg = RCAEvaluator.aggregate_metrics(runs, nominal_windows=windows)
    # post-injection: 0 wrong out of 4 diagnoses
    assert agg['post_injection_false_diagnoses'] == 0 and agg['post_injection_diagnoses'] == 4
    assert agg['post_injection_false_diagnosis_rate'] == 0.0
    # nominal: 1 false alarm in 180 fault-free seconds over 2 windows
    assert agg['nominal_false_alarms'] == 1
    assert agg['nominal_fault_free_windows'] == 2
    assert agg['nominal_fault_free_sec'] == 180.0
    assert agg['nominal_false_alarm_rate_per_min'] == round(1 / 3.0, 4)
    assert agg['nominal_false_alarm_rate_per_min'] != agg['post_injection_false_diagnosis_rate']


def test_empty_aggregate_still_exposes_the_documented_keys():
    agg = RCAEvaluator.aggregate_metrics([])
    for key in ('total_experiments', 'top1_accuracy', 'post_injection_diagnoses',
                'post_injection_false_diagnoses', 'post_injection_false_diagnosis_rate',
                'detection_latency_stats', 'diagnosis_latency_stats',
                'nominal_fault_free_windows', 'nominal_false_alarms'):
        assert key in agg


# -------------------------------------------------------------- chain metrics

def test_chain_accuracy_and_physical_coverage_have_distinct_documented_formulas():
    m = RCAEvaluator.chain_metrics(
        predicted_chain=['a', 'b'],
        expected_chain=['a', 'b', 'c'],            # observable
        physical_chain=['a', 'b', 'c', 'd'])       # physical superset
    # chain accuracy = |pred & obs| / |pred | obs| = 2/3
    assert m['chain_accuracy_numerator'] == 2 and m['chain_accuracy_denominator'] == 3
    assert abs(m['chain_accuracy'] - 2 / 3) < 1e-3
    # physical coverage = |pred & phys| / |phys| = 2/4
    assert m['physical_chain_numerator'] == 2 and m['physical_chain_denominator'] == 4
    assert m['physical_chain_coverage'] == 0.5
    # unobserved physical nodes are listed, never credited
    assert m['physically_affected_unobserved'] == ['c', 'd']


def test_unobserved_physical_nodes_never_increase_chain_accuracy():
    observed_only = RCAEvaluator.chain_metrics(['a'], ['a'], ['a', 'b', 'c'])
    assert observed_only['chain_accuracy'] == 1.0          # perfect on what is observable
    assert observed_only['physical_chain_coverage'] == round(1 / 3, 4)
    assert observed_only['physically_affected_unobserved'] == ['b', 'c']


# --------------------------------------------------------------- cohorts (97 vs 99)

def _run(key, top1, offline, reason=None):
    r = {'key': key, 'top1_correct': top1, 'top3_correct': top1, 'reciprocal_rank': 1.0 if top1 else 0.0,
         'evaluated_offline': offline, 'num_diagnoses_after_injection': 1,
         'num_false_diagnoses_after_injection': 0 if top1 else 1}
    if reason:
        r['exclusion_reason'] = reason
    return r


def _baseline_row(key, hits):
    row = {'key': key}
    for m in BASELINE_NAMES:
        top1 = m in hits
        row[m] = {'ranking': [], 'rank': 1 if top1 else -1, 'top1': top1}
    return row


def test_cohort_reports_all_runs_and_the_common_subset_with_matching_denominators():
    runs = [_run('a', True, True), _run('b', False, True),
            _run('graph_change', True, False, 'scenario declares no ground-truth root cause')]
    baselines = [_baseline_row('a', set(BASELINE_NAMES)), _baseline_row('b', set())]
    cohort = RCAEvaluator.comparison_cohort(runs, baselines, BASELINE_NAMES)

    assert cohort['all_runs'] == {'n': 3, 'proposed_top1_correct': 2,
                                  'proposed_top1_accuracy': round(2 / 3, 4)}
    sub = cohort['common_evidence_subset']
    assert sub['n'] == 2 and sub['proposed_top1_correct'] == 1 and sub['proposed_top1_accuracy'] == 0.5
    for m in BASELINE_NAMES:
        assert sub['baselines'][m]['n'] == 2, 'baselines must use the proposed method denominator'
    assert cohort['denominators_match'] is True
    ex = cohort['excluded_from_common_subset']
    assert ex['n'] == 1 and ex['keys'] == ['graph_change']
    assert sum(ex['reasons'].values()) == 1


def test_a_run_the_proposed_method_failed_to_diagnose_stays_in_the_cohort():
    """The exclusion criterion must be method-independent: a missed diagnosis is a
    miss, not a reason to drop the run from the comparison."""
    runs = [_run('a', True, True), _run('missed', False, True)]
    baselines = [_baseline_row('a', {'proposed'}), _baseline_row('missed', set(BASELINE_NAMES))]
    cohort = RCAEvaluator.comparison_cohort(runs, baselines, BASELINE_NAMES)
    assert cohort['common_evidence_subset']['n'] == 2
    assert cohort['excluded_from_common_subset']['n'] == 0
    # a baseline is allowed to beat the proposed method and must be reported as such
    assert cohort['common_evidence_subset']['baselines']['independent']['top1_correct'] == 1
    assert cohort['common_evidence_subset']['proposed_top1_correct'] == 1


def test_cohort_detects_mismatched_denominators():
    runs = [_run('a', True, True), _run('b', True, True)]
    cohort = RCAEvaluator.comparison_cohort(runs, [_baseline_row('a', set())], BASELINE_NAMES)
    assert cohort['denominators_match'] is False


def test_cohort_handles_an_empty_benchmark():
    cohort = RCAEvaluator.comparison_cohort([], [], BASELINE_NAMES)
    assert cohort['all_runs']['n'] == 0
    assert cohort['common_evidence_subset']['proposed_top1_accuracy'] is None
    assert cohort['denominators_match'] is True


# ----------------------------------------------------- artifacts on disk agree

def _workspace_root():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(os.path.dirname(here)))


def _artifacts():
    import glob
    root = _workspace_root()
    found = sorted(glob.glob(os.path.join(root, 'runs', '**', 'results*.json'), recursive=True))
    # only the artifacts produced by the current reporting contract carry "cohorts"
    keep = []
    for p in found:
        try:
            with open(p) as f:
                doc = json.load(f)
        except (OSError, ValueError):
            continue
        if isinstance(doc, dict) and 'cohorts' in doc and 'results' in doc:
            keep.append(p)
    return keep


ARTIFACTS = _artifacts()


@pytest.mark.skipif(not ARTIFACTS, reason='no benchmark artifacts in this checkout')
@pytest.mark.parametrize('path', ARTIFACTS, ids=lambda p: os.path.relpath(p, _workspace_root()))
def test_saved_benchmark_artifacts_are_internally_consistent(path):
    """Every saved aggregate and cohort must equal a recomputation from the
    per-run results stored in the same file (log == JSON == recomputation)."""
    with open(path) as f:
        doc = json.load(f)
    agg = RCAEvaluator.aggregate_metrics(doc['results'], nominal_windows=doc.get('nominal_windows') or [])
    for key in ('total_experiments', 'top1_correct', 'top1_accuracy', 'top3_accuracy',
                'mean_reciprocal_rank', 'post_injection_diagnoses', 'post_injection_false_diagnoses',
                'post_injection_false_diagnosis_rate', 'nominal_false_alarms',
                'nominal_fault_free_sec', 'nominal_fault_free_windows'):
        assert doc['aggregate'][key] == agg[key], f'{key} in {path}'
    cohort = RCAEvaluator.comparison_cohort(doc['results'], doc['baselines'], BASELINE_NAMES)
    assert doc['cohorts']['all_runs'] == cohort['all_runs']
    assert doc['cohorts']['common_evidence_subset']['n'] == cohort['common_evidence_subset']['n']
    assert doc['cohorts']['common_evidence_subset']['baselines'] == \
        cohort['common_evidence_subset']['baselines']
    assert doc['cohorts']['denominators_match'] is True, \
        'the proposed method and the baselines must share one denominator'
    assert doc.get('simulator') in ('rca_sim', 'gazebo'), 'artifacts must name their backend'


@pytest.mark.skipif(not ARTIFACTS, reason='no benchmark artifacts in this checkout')
@pytest.mark.parametrize('path', ARTIFACTS, ids=lambda p: os.path.relpath(p, _workspace_root()))
def test_saved_artifacts_contain_no_ground_truth_leak_into_diagnoses(path):
    """The recorded diagnoses/explanations come from the monitor, which has no
    ground truth; they must never contain the evaluation labels."""
    with open(path) as f:
        doc = json.load(f)
    for r in doc['results']:
        text = str(r.get('explanation', ''))
        for word in ('ground_truth', 'accepted_roots', 'observable_chain', 'physical_chain',
                     'injected', 'scenario'):
            assert word not in text, f'{word} leaked into an explanation in {path}'
