"""Evidence-based explanation generator and propagation chain builder.

Reconstructs the directed failure propagation chain from the dynamically
discovered graph and formats human-readable evidence summaries explaining why
the top candidate was chosen and why alternatives were ranked lower.

Temporal ordering is presented as *supporting evidence* for a probable root
cause, never as proof of causality.
"""

from typing import Dict, List, Tuple
from .anomaly_detector import Anomaly
from .graph_engine import DependencyGraph
from .rca_engine import DiagnosisResult


class Explainer:
    """Generates evidence-backed explanations and propagation chains."""

    @staticmethod
    def build_propagation_chain(
        root_cause: str,
        graph: DependencyGraph,
        anomalies: List[Anomaly],
    ) -> Tuple[List[str], List[str]]:
        """Finds the ordered failure propagation from root cause through reachable anomalous nodes.

        Returns (chain_lines, chain_nodes): human-readable lines and the ordered
        node list (root first, then downstream anomalous nodes in BFS order).
        """
        node_anoms: Dict[str, List[Anomaly]] = {}
        for a in anomalies:
            node_anoms.setdefault(a.node, []).append(a)

        chain_nodes = [root_cause]
        if root_cause not in node_anoms:
            # Candidate has no direct anomaly (e.g. silent upstream fault)
            chain = [f'{root_cause} (graph ancestor: no direct anomaly observed)']
        else:
            first_a = min(node_anoms[root_cause], key=lambda x: x.onset)
            chain = [f'{root_cause} (t={first_a.onset:.2f}s: {first_a.metric}, sev={first_a.severity:.2f})']

        # Traverse downstream anomalous nodes layer by layer
        visited = {root_cause}
        current_layer = [root_cause]
        depth = 1
        while current_layer:
            next_layer = []
            for u in current_layer:
                for v in sorted(graph.successors(u)):
                    if v in node_anoms and v not in visited:
                        visited.add(v)
                        next_layer.append(v)
                        chain_nodes.append(v)
                        first_v = min(node_anoms[v], key=lambda x: x.onset)
                        t_u = (min(node_anoms[u], key=lambda x: x.onset).onset
                               if u in node_anoms else first_v.onset)
                        dt = first_v.onset - t_u
                        edge_topics = graph.edge_topics.get((u, v), set())
                        topic_str = f' via {", ".join(sorted(edge_topics))}' if edge_topics else ''
                        indent = '  ' * depth
                        chain.append(
                            f'{indent}└─→ {v}{topic_str} (t={first_v.onset:.2f}s, '
                            f'{"+" if dt >= 0 else ""}{dt:.2f}s after {u}: {first_v.metric})'
                        )
            current_layer = next_layer
            depth += 1

        return chain, chain_nodes

    @classmethod
    def explain(
        cls,
        result: DiagnosisResult,
        graph: DependencyGraph,
        anomalies: List[Anomaly],
    ) -> str:
        """Generates a complete narrative explanation of the diagnosis.

        Side effect: fills `result.propagation_chain` with the ordered node list.
        """
        if not result.root_cause or not result.candidates:
            return result.explanation or 'No active anomalies detected.'

        top = result.candidates[0]
        n_anom = max(1, len(result.anomalous_nodes))
        lines = []
        lines.append('=' * 60)
        lines.append(f'ROOT CAUSE DIAGNOSIS: {top.node} (Confidence: {top.total_score:.3f})')
        lines.append('=' * 60)

        # 1. Score breakdown
        w = result.weights
        lines.append('Score Breakdown  R(C) = wD*D + wT*T + wS*S + wA*A:')
        lines.append(
            f'  R({top.node}) = {w["wD"]}·D({top.d_score:.2f}) + '
            f'{w["wT"]}·T({top.t_score:.2f}) + '
            f'{w["wS"]}·S({top.s_score:.2f}) + '
            f'{w["wA"]}·A({top.a_score:.2f}) = {top.total_score:.3f}'
        )
        lines.append(
            f'  - Dependency D: {top.d_score:.2f} (reaches {len(top.reachable_anomalies)} downstream '
            f'anomalous node(s); {len(top.upstream_anomalies)} anomalous upstream node(s))'
        )
        if top.first_anomaly_time is not None:
            lines.append(
                f'  - Temporal   T: {top.t_score:.2f} (first anomaly at t={top.first_anomaly_time:.2f}s; '
                f'earliest in window = supporting evidence, not proof of causality)'
            )
        else:
            lines.append(f'  - Temporal   T: {top.t_score:.2f} (no direct anomaly observed on this node)')
        lines.append(
            f'  - Symptom    S: {top.s_score:.2f} (explains {len((top.reachable_anomalies | {top.node}) & result.anomalous_nodes)}'
            f'/{n_anom} anomalous nodes)'
        )
        lines.append(
            f'  - Severity   A: {top.a_score:.2f} (max normalised severity on node, '
            f'{top.direct_anomaly_count} direct anomaly event(s))'
        )
        lines.append('')

        # 2. Propagation chain
        chain, chain_nodes = cls.build_propagation_chain(top.node, graph, anomalies)
        result.propagation_chain = chain_nodes
        lines.append('Propagation Chain (evidence-ordered, via discovered graph edges):')
        for step in chain:
            lines.append(f'  {step}')
        unexplained = sorted(result.anomalous_nodes - set(chain_nodes))
        if unexplained:
            lines.append(f'  (anomalous but not reachable from {top.node}: {", ".join(unexplained)})')
        lines.append('')

        # 3. Candidate ranking & why alternatives ranked lower
        if len(result.candidates) > 1:
            lines.append('Candidate Ranking & Evidence Comparison:')
            for i, cand in enumerate(result.candidates[1:4], start=2):
                diff = top.total_score - cand.total_score
                reasons = []
                if cand.t_score < top.t_score:
                    if cand.first_anomaly_time is not None and top.first_anomaly_time is not None:
                        dt = cand.first_anomaly_time - top.first_anomaly_time
                        reasons.append(f'first anomaly {dt:.2f}s after {top.node}')
                    elif cand.first_anomaly_time is None:
                        reasons.append('no direct anomaly observed')
                if cand.d_score < top.d_score:
                    if cand.node in top.reachable_anomalies:
                        reasons.append(f'downstream of {top.node} in discovered graph')
                    elif top.node in cand.upstream_anomalies or cand.upstream_anomalies:
                        reasons.append('has anomalous upstream dependency')
                    else:
                        reasons.append('does not reach observed symptoms via dependency edges')
                if cand.s_score < top.s_score:
                    reasons.append(f'lower symptom coverage ({cand.s_score*100:.0f}% vs {top.s_score*100:.0f}%)')
                if cand.a_score < top.a_score and not reasons:
                    reasons.append('lower anomaly severity')

                reason_str = f' [{"; ".join(reasons)}]' if reasons else ''
                lines.append(
                    f'  #{i} {cand.node:<20} R={cand.total_score:.3f} (Δ -{diff:.3f}){reason_str}'
                )

        lines.append('=' * 60)
        return '\n'.join(lines)
