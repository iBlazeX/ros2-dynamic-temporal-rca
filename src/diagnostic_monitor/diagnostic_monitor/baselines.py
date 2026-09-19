"""Baseline root-cause ranking methods for comparison against the proposed approach.

All methods consume exactly the same evidence (the list of observed anomalies)
and, where applicable, a dependency graph. None of them has access to ground
truth. They return an ordered candidate ranking (best first).

    1. independent      - independent component diagnostics: rank nodes purely by
                          their own anomaly severity (no graph, no time).
    2. temporal_only    - rank by earliest first-anomaly time only.
    3. static_dep_anom  - static (design-time) dependency graph + anomaly severity
                          (no temporal, no symptom-coverage term).
    4. static_dep_temp  - static dependency graph + temporal ordering
                          (no severity, no symptom-coverage term).
    5. proposed         - dynamic discovered graph + temporal + symptom coverage
                          + anomaly severity  (full R(C) model).

Methods 3-5 reuse RCAEngine so that the *only* differences are the graph source
and the enabled scoring terms, which keeps the comparison controlled.
"""

from typing import Dict, List, Optional
from .anomaly_detector import Anomaly
from .graph_engine import DependencyGraph
from .rca_engine import DiagnosisResult, RCAEngine


BASELINE_NAMES = [
    'independent',
    'temporal_only',
    'static_dep_anom',
    'static_dep_temp',
    'proposed',
]


def _group(anomalies: List[Anomaly]) -> Dict[str, List[Anomaly]]:
    out: Dict[str, List[Anomaly]] = {}
    for a in anomalies:
        out.setdefault(a.node, []).append(a)
    return out


def rank_independent(anomalies: List[Anomaly]) -> List[str]:
    """Independent component diagnostics: each node judged in isolation by severity."""
    groups = _group(anomalies)
    scored = [(max(a.severity for a in anoms), len(anoms), node) for node, anoms in groups.items()]
    scored.sort(key=lambda x: (-x[0], -x[1], x[2]))
    return [n for _, _, n in scored]


def rank_temporal_only(anomalies: List[Anomaly]) -> List[str]:
    """Temporal-only: earliest first anomaly wins."""
    groups = _group(anomalies)
    scored = [(min(a.onset for a in anoms), node) for node, anoms in groups.items()]
    scored.sort()
    return [n for _, n in scored]


def rank_with_engine(
    anomalies: List[Anomaly],
    graph: DependencyGraph,
    current_time: float,
    w_d: float, w_t: float, w_s: float, w_a: float,
    time_window_sec: float = 6.0,
) -> DiagnosisResult:
    engine = RCAEngine(w_d=w_d, w_t=w_t, w_s=w_s, w_a=w_a, time_window_sec=time_window_sec)
    return engine.diagnose(anomalies, graph, current_time)


def run_all_baselines(
    anomalies: List[Anomaly],
    dynamic_graph: DependencyGraph,
    static_graph: Optional[DependencyGraph],
    current_time: float,
    weights: Optional[Dict[str, float]] = None,
    time_window_sec: float = 6.0,
) -> Dict[str, List[str]]:
    """Runs every baseline on the same evidence. Returns {method: ranked node list}."""
    w = weights or {'wD': 0.30, 'wT': 0.30, 'wS': 0.25, 'wA': 0.15}
    static_graph = static_graph if static_graph is not None else DependencyGraph()

    results: Dict[str, List[str]] = {}
    results['independent'] = rank_independent(anomalies)
    results['temporal_only'] = rank_temporal_only(anomalies)

    d3 = rank_with_engine(anomalies, static_graph, current_time,
                          w_d=w['wD'], w_t=0.0, w_s=0.0, w_a=w['wA'], time_window_sec=time_window_sec)
    results['static_dep_anom'] = [c.node for c in d3.candidates]

    d4 = rank_with_engine(anomalies, static_graph, current_time,
                          w_d=w['wD'], w_t=w['wT'], w_s=0.0, w_a=0.0, time_window_sec=time_window_sec)
    results['static_dep_temp'] = [c.node for c in d4.candidates]

    d5 = rank_with_engine(anomalies, dynamic_graph, current_time,
                          w_d=w['wD'], w_t=w['wT'], w_s=w['wS'], w_a=w['wA'], time_window_sec=time_window_sec)
    results['proposed'] = [c.node for c in d5.candidates]
    return results


ABLATION_CONFIGS = {
    'full':   {'wD': 0.30, 'wT': 0.30, 'wS': 0.25, 'wA': 0.15},
    'no_D':   {'wD': 0.00, 'wT': 0.30, 'wS': 0.25, 'wA': 0.15},
    'no_T':   {'wD': 0.30, 'wT': 0.00, 'wS': 0.25, 'wA': 0.15},
    'no_S':   {'wD': 0.30, 'wT': 0.30, 'wS': 0.00, 'wA': 0.15},
    'no_A':   {'wD': 0.30, 'wT': 0.30, 'wS': 0.25, 'wA': 0.00},
}


def run_ablations(
    anomalies: List[Anomaly],
    graph: DependencyGraph,
    current_time: float,
    time_window_sec: float = 6.0,
) -> Dict[str, List[str]]:
    """Re-ranks the same evidence with each scoring term disabled in turn."""
    out: Dict[str, List[str]] = {}
    for name, w in ABLATION_CONFIGS.items():
        d = rank_with_engine(anomalies, graph, current_time,
                             w_d=w['wD'], w_t=w['wT'], w_s=w['wS'], w_a=w['wA'],
                             time_window_sec=time_window_sec)
        out[name] = [c.node for c in d.candidates]
    return out
