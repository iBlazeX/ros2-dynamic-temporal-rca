"""Root-Cause Analysis (RCA) engine combining graph dependencies, temporal order,

symptom coverage, and anomaly severity.

Formula:
    R(C) = wD * D(C) + wT * T(C) + wS * S(C) + wA * A(C)

Fully configurable weights and ablation support.
Strictly isolated from ground truth - operates only on runtime observations.
"""

from typing import Any, Dict, List, Optional, Set
from .anomaly_detector import Anomaly
from .graph_engine import DependencyGraph


class CandidateScore:
    """Detailed score breakdown for a candidate root cause node."""

    def __init__(
        self,
        node: str,
        total_score: float,
        d_score: float,
        t_score: float,
        s_score: float,
        a_score: float,
        first_anomaly_time: Optional[float] = None,
        direct_anomaly_count: int = 0,
        reachable_anomalies: Optional[Set[str]] = None,
        upstream_anomalies: Optional[Set[str]] = None,
    ):
        self.node = node
        self.total_score = total_score
        self.d_score = d_score
        self.t_score = t_score
        self.s_score = s_score
        self.a_score = a_score
        self.first_anomaly_time = first_anomaly_time
        self.direct_anomaly_count = direct_anomaly_count
        self.reachable_anomalies = reachable_anomalies or set()
        self.upstream_anomalies = upstream_anomalies or set()

    def to_dict(self) -> dict:
        return {
            'node': self.node,
            'total_score': round(self.total_score, 4),
            'd_score': round(self.d_score, 4),
            't_score': round(self.t_score, 4),
            's_score': round(self.s_score, 4),
            'a_score': round(self.a_score, 4),
            'first_anomaly_time': self.first_anomaly_time,
            'direct_anomaly_count': self.direct_anomaly_count,
            'reachable_anomalies': sorted(list(self.reachable_anomalies)),
            'upstream_anomalies': sorted(list(self.upstream_anomalies)),
        }


class DiagnosisResult:
    """Result of an RCA diagnosis execution."""

    def __init__(
        self,
        timestamp: float,
        root_cause: Optional[str],
        confidence: float,
        candidates: List[CandidateScore],
        anomalous_nodes: Set[str],
        weights: Dict[str, float],
        explanation: str = '',
        propagation_chain: Optional[List[str]] = None,
    ):
        self.timestamp = timestamp
        self.root_cause = root_cause
        self.confidence = confidence
        self.candidates = candidates
        self.anomalous_nodes = anomalous_nodes
        self.weights = weights
        self.explanation = explanation
        # Ordered node list of the evidence-backed propagation chain (root first)
        self.propagation_chain: List[str] = propagation_chain or []

    def to_dict(self) -> dict:
        return {
            'timestamp': self.timestamp,
            'root_cause': self.root_cause,
            'confidence': round(self.confidence, 4),
            'weights': self.weights,
            'anomalous_nodes': sorted(list(self.anomalous_nodes)),
            'candidates': [c.to_dict() for c in self.candidates],
            'propagation_chain': list(self.propagation_chain),
            'explanation': self.explanation,
        }


class RCAEngine:
    """Computes dynamic dependency-aware temporal root-cause analysis."""

    def __init__(
        self,
        w_d: float = 0.30,
        w_t: float = 0.30,
        w_s: float = 0.25,
        w_a: float = 0.15,
        time_window_sec: float = 5.0,
    ):
        self.w_d = float(w_d)
        self.w_t = float(w_t)
        self.w_s = float(w_s)
        self.w_a = float(w_a)
        self.time_window_sec = float(time_window_sec)

    def set_weights(self, w_d: float, w_t: float, w_s: float, w_a: float):
        """Allows dynamic weight reconfiguration and ablation (setting any weight to 0.0)."""
        self.w_d = float(w_d)
        self.w_t = float(w_t)
        self.w_s = float(w_s)
        self.w_a = float(w_a)

    def diagnose(
        self,
        anomalies: List[Anomaly],
        graph: DependencyGraph,
        current_time: float,
    ) -> DiagnosisResult:
        """Executes root-cause analysis on observed anomalies and dynamically discovered graph."""
        weights_dict = {
            'wD': self.w_d,
            'wT': self.w_t,
            'wS': self.w_s,
            'wA': self.w_a,
        }

        # Filter anomalies to the analysis window
        window_start = current_time - self.time_window_sec
        recent_anomalies = [a for a in anomalies if a.timestamp >= window_start]

        if not recent_anomalies:
            return DiagnosisResult(
                timestamp=current_time,
                root_cause=None,
                confidence=0.0,
                candidates=[],
                anomalous_nodes=set(),
                weights=weights_dict,
                explanation='No anomalies detected in the current observation window. System operating nominally.',
            )

        # Map anomalies by node
        node_anomalies: Dict[str, List[Anomaly]] = {}
        for a in recent_anomalies:
            node_anomalies.setdefault(a.node, []).append(a)

        anomalous_nodes = set(node_anomalies.keys())
        num_anomalies = len(anomalous_nodes)

        # 1. Candidate Generation: anomalous nodes + all their upstream ancestors
        candidate_nodes = set(anomalous_nodes)
        for anom_node in anomalous_nodes:
            candidate_nodes.update(graph.ancestors(anom_node))

        if not candidate_nodes:
            return DiagnosisResult(
                timestamp=current_time,
                root_cause=None,
                confidence=0.0,
                candidates=[],
                anomalous_nodes=anomalous_nodes,
                weights=weights_dict,
                explanation='Insufficient graph connectivity to form candidates.',
            )

        # Earliest anomaly onset across all nodes. onset = physical start of the
        # deviation; the record timestamp is only used for the sliding window above.
        earliest_time = min(
            min(a.onset for a in anoms) for anoms in node_anomalies.values()
        )

        candidate_scores: List[CandidateScore] = []

        total_weight = self.w_d + self.w_t + self.w_s + self.w_a

        for c in candidate_nodes:
            c_desc = graph.descendants(c)
            c_anc = graph.ancestors(c)

            reachable_anoms = c_desc & anomalous_nodes
            upstream_anoms = c_anc & anomalous_nodes

            # --- Dependency Score D(C) ---
            # Interpretable: rewards reaching anomalous components, penalizes having upstream anomalous parents
            if num_anomalies <= 1:
                d_score = 1.0 if c in anomalous_nodes else 0.5
            else:
                max_possible_downstream = num_anomalies - (1 if c in anomalous_nodes else 0)
                downstream_ratio = len(reachable_anoms) / max(1, max_possible_downstream)
                upstream_penalty = len(upstream_anoms) / num_anomalies
                d_score = downstream_ratio * (1.0 - upstream_penalty)
            d_score = max(0.0, min(1.0, d_score))

            # --- Temporal Score T(C) ---
            if c in node_anomalies:
                t_first_c = min(a.onset for a in node_anomalies[c])
                dt = max(0.0, t_first_c - earliest_time)
                # Linear decay over window
                t_score = max(0.0, 1.0 - (dt / max(1.0, self.time_window_sec)))
            else:
                t_first_c = None
                # Penalize unobserved candidate that is only a graph ancestor
                t_score = 0.05

            # --- Symptom Coverage S(C) ---
            # User exact formula: |(Descendants(C) ∪ {C}) ∩ AnomalousNodes| / |AnomalousNodes|
            covered_nodes = (c_desc | {c}) & anomalous_nodes
            s_score = len(covered_nodes) / max(1, len(anomalous_nodes))
            s_score = max(0.0, min(1.0, s_score))

            # --- Anomaly Severity A(C) ---
            if c in node_anomalies:
                a_score = max(a.severity for a in node_anomalies[c])
            else:
                a_score = 0.0
            a_score = max(0.0, min(1.0, a_score))

            # --- Combined Score R(C) ---
            weighted_sum = (
                self.w_d * d_score +
                self.w_t * t_score +
                self.w_s * s_score +
                self.w_a * a_score
            )
            total_score = weighted_sum / total_weight if total_weight > 0 else 0.0

            candidate_scores.append(
                CandidateScore(
                    node=c,
                    total_score=total_score,
                    d_score=d_score,
                    t_score=t_score,
                    s_score=s_score,
                    a_score=a_score,
                    first_anomaly_time=t_first_c,
                    direct_anomaly_count=len(node_anomalies.get(c, [])),
                    reachable_anomalies=reachable_anoms,
                    upstream_anomalies=upstream_anoms,
                )
            )

        # Sort candidates descending by total score, then by earliest anomaly,
        # then by node name (fully deterministic ordering).
        candidate_scores.sort(
            key=lambda x: (-round(x.total_score, 9), x.first_anomaly_time or 9e18, x.node)
        )

        top_candidate = candidate_scores[0] if candidate_scores else None
        root_cause = top_candidate.node if top_candidate else None
        confidence = top_candidate.total_score if top_candidate else 0.0

        return DiagnosisResult(
            timestamp=current_time,
            root_cause=root_cause,
            confidence=confidence,
            candidates=candidate_scores,
            anomalous_nodes=anomalous_nodes,
            weights=weights_dict,
        )
