"""Evaluation metrics for RCA research benchmarks.

Ground truth is passed exclusively to this evaluation layer and is NEVER
accessible by the diagnostic engine or monitor.

Metrics per experiment:
- top1_correct / topk_correct / ground_truth_rank / reciprocal_rank
- detection_latency_sec : first anomaly recorded after fault injection - t_inject
- diagnosis_latency_sec : first diagnosis (any) after injection - t_inject
- correct_diagnosis_latency_sec : first diagnosis naming the true root - t_inject
- false_diagnosis_rate  : fraction of diagnoses issued after injection whose
                          root cause != ground truth
- chain_accuracy        : Jaccard similarity between the predicted propagation
                          chain node set and the expected affected node set
- chain_order_correct   : predicted chain respects expected ordering
"""

from typing import Any, Dict, List, Optional, Sequence
from .rca_engine import DiagnosisResult


class RCAEvaluator:
    """Evaluates diagnosis predictions against injected ground truth."""

    @staticmethod
    def rank_of(ranking: Sequence[str], ground_truth_node: str) -> int:
        return (list(ranking).index(ground_truth_node) + 1) if ground_truth_node in ranking else -1

    @staticmethod
    def chain_metrics(predicted_chain: Sequence[str], expected_chain: Sequence[str]) -> Dict[str, Any]:
        pred = list(predicted_chain or [])
        exp = list(expected_chain or [])
        ps, es = set(pred), set(exp)
        union = ps | es
        jaccard = (len(ps & es) / len(union)) if union else 1.0
        # Order check: the relative order of expected nodes present in the prediction must match
        present = [n for n in exp if n in ps]
        pred_order = [n for n in pred if n in es]
        order_ok = present == pred_order
        return {
            'chain_accuracy': round(jaccard, 4),
            'chain_order_correct': bool(order_ok and len(present) == len(exp)),
            'predicted_chain': pred,
            'expected_chain': exp,
        }

    @classmethod
    def evaluate_single(
        cls,
        diagnosis: DiagnosisResult,
        ground_truth_node: str,
        fault_injection_time: Optional[float] = None,
        fault_type: str = '',
        k: int = 3,
        expected_chain: Optional[Sequence[str]] = None,
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
        if diagnoses_after_injection is not None:
            n_after = len(diagnoses_after_injection)
            if n_after:
                wrong = sum(1 for r in diagnoses_after_injection if r != ground_truth_node)
                fdr = round(wrong / n_after, 4)

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
            'ranking': candidate_names,
        }
        if expected_chain is not None:
            res.update(cls.chain_metrics(diagnosis.propagation_chain, expected_chain))
        return res

    @classmethod
    def aggregate_metrics(cls, eval_results: List[Dict[str, Any]], k: int = 3) -> Dict[str, Any]:
        """Computes aggregate evaluation summary across multiple experiments."""
        n = len(eval_results)
        if n == 0:
            return {
                'total_experiments': 0, 'top1_accuracy': 0.0, f'top{k}_accuracy': 0.0,
                'top3_accuracy': 0.0, 'mean_reciprocal_rank': 0.0,
                'avg_detection_latency_sec': 0.0, 'avg_diagnosis_latency_sec': 0.0,
                'avg_correct_diagnosis_latency_sec': 0.0, 'false_diagnosis_rate': 0.0,
                'avg_chain_accuracy': 0.0, 'chain_order_accuracy': 0.0,
            }

        def _avg(key):
            vals = [r[key] for r in eval_results if r.get(key) is not None]
            return round(sum(vals) / len(vals), 3) if vals else None

        top1 = sum(1 for r in eval_results if r.get('top1_correct'))
        topk = sum(1 for r in eval_results if r.get(f'top{k}_correct'))
        top3 = sum(1 for r in eval_results if r.get('top3_correct'))
        mrr = sum(r.get('reciprocal_rank', 0.0) for r in eval_results)

        # Pooled false diagnosis rate across all diagnoses issued after injection
        tot_diag = sum(r.get('num_diagnoses_after_injection', 0) or 0 for r in eval_results)
        tot_wrong = sum(
            round((r.get('false_diagnosis_rate') or 0.0) * (r.get('num_diagnoses_after_injection') or 0))
            for r in eval_results
        )
        chain_ok = [r for r in eval_results if 'chain_order_correct' in r]

        return {
            'total_experiments': n,
            'top1_accuracy': round(top1 / n, 4),
            f'top{k}_accuracy': round(topk / n, 4),
            'top3_accuracy': round(top3 / n, 4),
            'mean_reciprocal_rank': round(mrr / n, 4),
            'avg_detection_latency_sec': _avg('detection_latency_sec'),
            'avg_diagnosis_latency_sec': _avg('diagnosis_latency_sec'),
            'avg_correct_diagnosis_latency_sec': _avg('correct_diagnosis_latency_sec'),
            'false_diagnosis_rate': round(tot_wrong / tot_diag, 4) if tot_diag else None,
            'avg_chain_accuracy': _avg('chain_accuracy'),
            'chain_order_accuracy': (round(sum(1 for r in chain_ok if r['chain_order_correct']) / len(chain_ok), 4)
                                     if chain_ok else None),
        }
