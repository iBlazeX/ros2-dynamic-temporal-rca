"""Evaluation metrics for RCA research benchmarks.

Ground truth is passed exclusively to this evaluation layer and is NEVER
accessible by the diagnostic engine or monitor.

METRIC DEFINITIONS (numerator / denominator are always reported explicitly)
--------------------------------------------------------------------------
Per experiment (one scenario x one repetition):

- top1_correct            : predicted root cause == ground-truth root.
- topk_correct            : ground-truth root appears in the first k candidates.
- ground_truth_rank       : 1-based position of the truth in the ranking, -1 if absent.
- reciprocal_rank         : 1/rank, 0 if absent.
- detection_latency_sec   : (first anomaly recorded after injection) - t_inject.
- diagnosis_latency_sec   : (first diagnosis of any root after injection) - t_inject.
- correct_diagnosis_latency_sec : (first diagnosis naming the true root) - t_inject.
- false_diagnosis_rate    : per-run POST-INJECTION false-diagnosis rate
                            = #diagnoses after injection whose root != truth
                            / #diagnoses after injection.
                            This is NOT a nominal false-alarm rate; see below.

Propagation-chain metrics (see chain_metrics for the exact sets):

- chain_accuracy          : |predicted & observable| / |predicted | observable|
                            (Jaccard over NODE SETS, unweighted, per run).
- chain_order_correct     : the expected observable nodes appear in the predicted
                            chain in the expected relative order AND all of them
                            are present.
- physical_chain_coverage : |predicted & physical| / |physical|
                            (recall over the PHYSICALLY affected node set).
                            Physically affected nodes that the monitor cannot
                            observe are listed in physically_affected_unobserved
                            and are never credited to the prediction.

Aggregates (aggregate_metrics) are unweighted means over runs for latency and
chain metrics (each run counts once, regardless of how many diagnoses it
produced), and POOLED counts for diagnosis-level rates:

- post_injection_diagnoses       : denominator, total diagnoses after injection
                                   summed over all runs.
- post_injection_false_diagnoses : numerator, of those, how many named a node
                                   other than the ground-truth root.
- post_injection_false_diagnosis_rate = numerator / denominator.

- nominal_fault_free_windows     : number of fault-free observation windows.
- nominal_fault_free_sec         : total fault-free wall-clock time observed.
- nominal_anomalies              : anomalies recorded while no fault was active.
- nominal_false_alarms           : diagnoses issued while no fault was active
                                   (any diagnosis in a fault-free window is a
                                   false alarm by definition).
- nominal_false_alarm_rate_per_min = nominal_false_alarms / (sec / 60).

"post-injection false-diagnosis rate" and "nominal false-alarm rate" are two
different quantities and are never both called "FDR".
"""

import math
from typing import Any, Dict, List, Optional, Sequence
from .rca_engine import DiagnosisResult


def latency_stats(values: Sequence[float]) -> Dict[str, Any]:
    """Distribution summary (not just the mean) for a latency sample."""
    vals = sorted(float(v) for v in values if v is not None)
    n = len(vals)
    if n == 0:
        return {'n': 0, 'mean': None, 'min': None, 'max': None, 'std': None,
                'median': None, 'p25': None, 'p75': None}

    def _q(p: float) -> float:
        if n == 1:
            return vals[0]
        pos = p * (n - 1)
        lo = int(math.floor(pos))
        hi = min(lo + 1, n - 1)
        return vals[lo] + (pos - lo) * (vals[hi] - vals[lo])

    mean = sum(vals) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in vals) / n) if n > 1 else 0.0
    return {
        'n': n,
        'mean': round(mean, 3), 'min': round(vals[0], 3), 'max': round(vals[-1], 3),
        'std': round(std, 3), 'median': round(_q(0.5), 3),
        'p25': round(_q(0.25), 3), 'p75': round(_q(0.75), 3),
    }


class RCAEvaluator:
    """Evaluates diagnosis predictions against injected ground truth."""

    @staticmethod
    def rank_of(ranking: Sequence[str], ground_truth_node: str) -> int:
        return (list(ranking).index(ground_truth_node) + 1) if ground_truth_node in ranking else -1

    @staticmethod
    def chain_metrics(
        predicted_chain: Sequence[str],
        expected_chain: Sequence[str],
        physical_chain: Optional[Sequence[str]] = None,
    ) -> Dict[str, Any]:
        """Scores the predicted propagation chain.

        `expected_chain` is the OBSERVABLE affected set: the nodes whose failure
        the monitor's metrics can actually see. chain_accuracy is the Jaccard
        similarity of the predicted node set with it:
            |predicted & observable| / |predicted | observable|
        (unweighted, per run; 1.0 when both sets are empty).

        `physical_chain` is the PHYSICALLY affected set, which may contain nodes
        whose symptom is invisible to the monitor. physical_chain_coverage is
            |predicted & physical| / |physical|
        and `physically_affected_unobserved` lists the physical nodes missing
        from the prediction - the prediction is never credited with those.
        """
        pred = list(predicted_chain or [])
        exp = list(expected_chain or [])
        ps, es = set(pred), set(exp)
        union = ps | es
        jaccard = (len(ps & es) / len(union)) if union else 1.0
        # Order check: the relative order of expected nodes present in the prediction must match
        present = [n for n in exp if n in ps]
        pred_order = [n for n in pred if n in es]
        order_ok = present == pred_order
        out = {
            'chain_accuracy': round(jaccard, 4),
            'chain_accuracy_numerator': len(ps & es),
            'chain_accuracy_denominator': len(union),
            'chain_order_correct': bool(order_ok and len(present) == len(exp)),
            'predicted_chain': pred,
            'expected_chain': exp,
        }
        phys = list(physical_chain) if physical_chain is not None else exp
        phs = set(phys)
        out['physical_chain'] = phys
        out['physical_chain_coverage'] = round(len(ps & phs) / len(phs), 4) if phs else 1.0
        out['physical_chain_numerator'] = len(ps & phs)
        out['physical_chain_denominator'] = len(phs)
        out['physically_affected_unobserved'] = [n for n in phys if n not in ps]
        return out

    @classmethod
    def evaluate_single(
        cls,
        diagnosis: DiagnosisResult,
        ground_truth_node: str,
        fault_injection_time: Optional[float] = None,
        fault_type: str = '',
        k: int = 3,
        expected_chain: Optional[Sequence[str]] = None,
        physical_chain: Optional[Sequence[str]] = None,
        first_anomaly_time: Optional[float] = None,
        first_diagnosis_time: Optional[float] = None,
        first_correct_diagnosis_time: Optional[float] = None,
        diagnoses_after_injection: Optional[Sequence[str]] = None,
    ) -> Dict[str, Any]:
        """Evaluates a single diagnosis instance against ground truth."""
        candidate_names = [c.node for c in diagnosis.candidates]
        rank = cls.rank_of(candidate_names, ground_truth_node)

        top1_correct = (diagnosis.root_cause == ground_truth_node)
        topk_correct = 0 < rank <= k
        reciprocal_rank = (1.0 / rank) if rank > 0 else 0.0

        def _lat(t):
            if fault_injection_time is None or t is None:
                return None
            return round(max(0.0, t - fault_injection_time), 3)

        # Legacy detection latency (diagnosis timestamp - injection) kept for reference
        legacy_latency = _lat(diagnosis.timestamp)

        fdr = None
        n_after = 0
        n_wrong = 0
        if diagnoses_after_injection is not None:
            n_after = len(diagnoses_after_injection)
            if n_after:
                n_wrong = sum(1 for r in diagnoses_after_injection if r != ground_truth_node)
                fdr = round(n_wrong / n_after, 4)

        res = {
            'ground_truth': ground_truth_node,
            'predicted_root_cause': diagnosis.root_cause,
            'confidence': round(diagnosis.confidence, 4),
            'fault_type': fault_type,
            'top1_correct': bool(top1_correct),
            f'top{k}_correct': bool(topk_correct),
            'top3_correct': bool(0 < rank <= 3),
            'ground_truth_rank': rank,
            'reciprocal_rank': round(reciprocal_rank, 4),
            'detection_latency_sec': _lat(first_anomaly_time) if first_anomaly_time is not None else legacy_latency,
            'diagnosis_latency_sec': _lat(first_diagnosis_time),
            'correct_diagnosis_latency_sec': _lat(first_correct_diagnosis_time),
            'false_diagnosis_rate': fdr,
            'num_diagnoses_after_injection': n_after,
            'num_false_diagnoses_after_injection': n_wrong,
            'ranking': candidate_names,
        }
        if expected_chain is not None:
            res.update(cls.chain_metrics(diagnosis.propagation_chain, expected_chain, physical_chain))
        return res

    @staticmethod
    def nominal_metrics(
        windows: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Summarises fault-free (nominal) observation windows.

        Each window is {'name', 'duration_sec', 'anomalies', 'diagnoses'} measured
        by the evaluation layer from the monitor's event store while no fault was
        active. A diagnosis issued in a fault-free window is a false alarm.

        Reported as an explicit numerator/denominator pair:
            nominal_false_alarms (numerator, count of diagnoses)
            nominal_fault_free_sec / nominal_fault_free_windows (denominators)
        """
        total_sec = sum(float(w.get('duration_sec', 0.0)) for w in windows)
        anomalies = sum(int(w.get('anomalies', 0)) for w in windows)
        diagnoses = sum(int(w.get('diagnoses', 0)) for w in windows)
        rate = round(diagnoses / (total_sec / 60.0), 4) if total_sec > 0 else None
        return {
            'nominal_fault_free_windows': len(windows),
            'nominal_fault_free_sec': round(total_sec, 3),
            'nominal_anomalies': anomalies,
            'nominal_diagnoses': diagnoses,
            'nominal_false_alarms': diagnoses,
            'nominal_false_alarm_rate_per_min': rate,
            # Backwards-compatible alias (same quantity, older artifact key).
            'nominal_false_alarms_per_min': rate,
        }

    @staticmethod
    def comparison_cohort(
        eval_results: List[Dict[str, Any]],
        baseline_results: List[Dict[str, Any]],
        baseline_names: Sequence[str],
    ) -> Dict[str, Any]:
        """Reports the proposed method and the baselines on explicit denominators.

        ALL RUNS       - every executed run, including runs the proposed method
                         failed to diagnose.
        COMMON-EVIDENCE SUBSET - the runs that were also scored offline by the
                         baselines on the identical recorded evidence.

        The inclusion criterion for the common-evidence subset is defined BEFORE
        the comparison and is method-independent: a run is included iff the
        scenario declares a ground-truth root cause (i.e. it is not an
        `expect_no_diagnosis` graph-change scenario). Runs in which the proposed
        method produced NO diagnosis are still included and count as a miss for
        every method, so the denominators of the proposed method and of the
        baselines are identical.
        """
        n_all = len(eval_results)
        included = [r for r in eval_results if r.get('evaluated_offline')]
        excluded = [r for r in eval_results if not r.get('evaluated_offline')]
        reasons: Dict[str, int] = {}
        for r in excluded:
            key = str(r.get('exclusion_reason', 'not_scored_offline'))
            reasons[key] = reasons.get(key, 0) + 1

        def _rate(hits, n):
            return round(hits / n, 4) if n else None

        all_top1 = sum(1 for r in eval_results if r.get('top1_correct'))
        sub_top1 = sum(1 for r in included if r.get('top1_correct'))
        n_sub = len(included)

        baselines: Dict[str, Any] = {}
        nb = len(baseline_results)
        for m in baseline_names:
            hits = sum(1 for b in baseline_results if b.get(m, {}).get('top1'))
            mrr = (sum((1.0 / b[m]['rank']) if b.get(m, {}).get('rank', -1) > 0 else 0.0
                       for b in baseline_results) / nb) if nb else None
            baselines[m] = {'top1_correct': hits, 'n': nb, 'top1_accuracy': _rate(hits, nb),
                            'mean_reciprocal_rank': round(mrr, 4) if mrr is not None else None}

        return {
            'all_runs': {
                'n': n_all,
                'proposed_top1_correct': all_top1,
                'proposed_top1_accuracy': _rate(all_top1, n_all),
            },
            'common_evidence_subset': {
                'n': n_sub,
                'proposed_top1_correct': sub_top1,
                'proposed_top1_accuracy': _rate(sub_top1, n_sub),
                'baselines': baselines,
                'inclusion_criterion': (
                    'scenario declares a ground-truth root cause; defined before the comparison '
                    'and independent of which method succeeded'),
            },
            'excluded_from_common_subset': {
                'n': len(excluded),
                'reasons': reasons,
                'keys': sorted({str(r.get('key', r.get('scenario', '?'))) for r in excluded}),
            },
            'denominators_match': bool(n_sub == nb),
        }

    @classmethod
    def aggregate_metrics(
        cls,
        eval_results: List[Dict[str, Any]],
        k: int = 3,
        nominal_windows: Optional[List[Dict[str, Any]]] = None,
    ) -> Dict[str, Any]:
        """Computes the aggregate evaluation summary across multiple experiments.

        This is the single source of truth for the aggregate: anything that
        prints or saves an aggregate must call this function with the same inputs.
        Every rate is accompanied by its numerator and denominator.
        """
        n = len(eval_results)
        nominal = cls.nominal_metrics(nominal_windows or [])
        if n == 0:
            out = {
                'total_experiments': 0, 'top1_correct': 0, 'top1_accuracy': 0.0,
                f'top{k}_correct': 0, f'top{k}_accuracy': 0.0,
                'top3_correct': 0, 'top3_accuracy': 0.0, 'mean_reciprocal_rank': 0.0,
                'avg_detection_latency_sec': 0.0, 'avg_diagnosis_latency_sec': 0.0,
                'avg_correct_diagnosis_latency_sec': 0.0,
                'detection_latency_stats': latency_stats([]),
                'diagnosis_latency_stats': latency_stats([]),
                'correct_diagnosis_latency_stats': latency_stats([]),
                'post_injection_diagnoses': 0, 'post_injection_false_diagnoses': 0,
                'post_injection_false_diagnosis_rate': None, 'false_diagnosis_rate': None,
                'avg_chain_accuracy': 0.0, 'chain_order_accuracy': 0.0,
                'avg_physical_chain_coverage': 0.0,
            }
            out.update(nominal)
            return out

        def _vals(key):
            return [r[key] for r in eval_results if r.get(key) is not None]

        def _avg(key):
            vals = _vals(key)
            return round(sum(vals) / len(vals), 3) if vals else None

        top1 = sum(1 for r in eval_results if r.get('top1_correct'))
        topk = sum(1 for r in eval_results if r.get(f'top{k}_correct'))
        top3 = sum(1 for r in eval_results if r.get('top3_correct'))
        mrr = sum(r.get('reciprocal_rank', 0.0) for r in eval_results)

        # Pooled post-injection false-diagnosis rate across every diagnosis issued
        # after injection (diagnosis-level, NOT run-level).
        tot_diag = sum(r.get('num_diagnoses_after_injection', 0) or 0 for r in eval_results)
        tot_wrong = 0
        for r in eval_results:
            n_after = r.get('num_diagnoses_after_injection') or 0
            if r.get('num_false_diagnoses_after_injection') is not None:
                tot_wrong += int(r['num_false_diagnoses_after_injection'])
            else:
                tot_wrong += round((r.get('false_diagnosis_rate') or 0.0) * n_after)
        pooled_fdr = round(tot_wrong / tot_diag, 4) if tot_diag else None
        chain_ok = [r for r in eval_results if 'chain_order_correct' in r]
        n_chain_order = len(chain_ok)
        chain_order_hits = sum(1 for r in chain_ok if r['chain_order_correct'])

        out = {
            'total_experiments': n,
            'top1_correct': top1,
            'top1_accuracy': round(top1 / n, 4),
            f'top{k}_correct': topk,
            f'top{k}_accuracy': round(topk / n, 4),
            'top3_correct': top3,
            'top3_accuracy': round(top3 / n, 4),
            'mean_reciprocal_rank': round(mrr / n, 4),
            'avg_detection_latency_sec': _avg('detection_latency_sec'),
            'avg_diagnosis_latency_sec': _avg('diagnosis_latency_sec'),
            'avg_correct_diagnosis_latency_sec': _avg('correct_diagnosis_latency_sec'),
            'detection_latency_stats': latency_stats(_vals('detection_latency_sec')),
            'diagnosis_latency_stats': latency_stats(_vals('diagnosis_latency_sec')),
            'correct_diagnosis_latency_stats': latency_stats(_vals('correct_diagnosis_latency_sec')),
            # explicit numerator / denominator / rate for the post-injection metric
            'post_injection_diagnoses': tot_diag,
            'post_injection_false_diagnoses': tot_wrong,
            'post_injection_false_diagnosis_rate': pooled_fdr,
            # Backwards-compatible alias of post_injection_false_diagnosis_rate.
            'false_diagnosis_rate': pooled_fdr,
            'num_diagnoses_after_injection': tot_diag,
            'avg_chain_accuracy': _avg('chain_accuracy'),
            'chain_accuracy_n': len(_vals('chain_accuracy')),
            'chain_order_correct_count': chain_order_hits,
            'chain_order_n': n_chain_order,
            'chain_order_accuracy': (round(chain_order_hits / n_chain_order, 4) if n_chain_order else None),
            'avg_physical_chain_coverage': _avg('physical_chain_coverage'),
            'physical_chain_coverage_n': len(_vals('physical_chain_coverage')),
            'scenarios_with_unobserved_physical_nodes': sorted({
                r.get('key', r.get('scenario', '?')) for r in eval_results
                if r.get('physically_affected_unobserved')
            }),
        }
        out.update(nominal)
        return out
