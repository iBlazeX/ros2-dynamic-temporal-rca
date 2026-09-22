"""Live dashboard state: the JSON document published on ``/rca/dashboard_state``.

The diagnostic monitor is the single source of truth. This module turns the
monitor's *current* runtime view (discovered graph, anomalies in the RCA window,
latest diagnosis, bounded event timeline) into a stable, documented JSON schema
that a visualisation layer (the C++ TUI) can render without re-implementing any
RCA logic.

The state contains NO ground truth: the monitor never has any.

Schema (version 1)
------------------
{
  "schema_version": 1,
  "timestamp": <float, unix seconds when this state was built>,
  "simulator": <str, which environment produced the data: "rca_sim" | "gazebo" |
                "rca_test_system" | "unknown">.  Presentation metadata only; it
                is a launch-time label, never evaluation ground truth,
  "monitor_uptime_sec": <float>,
  "system_status": "WAITING" | "NORMAL" | "DEGRADED" | "RECOVERING",
  "counts": {"nodes", "edges", "topics", "anomalies_in_window", "anomalous_nodes",
             "diagnoses_total", "anomalies_total"},
  "rca_window_sec": <float>,
  "nodes": [{"name", "status": "NORMAL"|"ANOMALOUS"|"ROOT_CAUSE"|"MISSING",
             "anomaly_count", "max_severity", "last_metric", "last_onset",
             "in_ros_graph": <bool>, "liveness": "ALIVE"|"STARVED"|"UNKNOWN"}],
  "edges": [{"from", "to", "topics": [..]}],
  "anomalies": [{"node", "type", "timestamp", "onset_time", "severity", "value",
                 "description"}],                       # most recent first, bounded
  "latest_diagnosis": null | {"timestamp", "root_cause", "confidence",
                 "scores": {"dependency", "temporal", "symptom", "severity"},
                 "weights": {"wD","wT","wS","wA"},
                 "candidates": [{"node","total","d","t","s","a"}],   # ranked, bounded
                 "propagation_chain": [..], "anomalous_nodes": [..],
                 "explanation": <str, evidence text produced by the monitor>},
  "timeline": [{"t", "node", "event", "severity", "message"}]   # oldest first, bounded
}

ROS GRAPH MEMBERSHIP vs RUNTIME LIVENESS (they are not the same thing)
---------------------------------------------------------------------
``in_ros_graph`` is DDS/ROS *graph membership*: whether the node was present in
the last ``get_node_names_and_namespaces()`` discovery sweep. A crashed process
does NOT leave the graph immediately - discovery only forgets it once the DDS
participant lease expires (measured on this system at roughly 15-25 s, see
README). Graph membership is therefore a late and unreliable crash detector.

``liveness`` is *runtime liveness* derived from observed data flow: STARVED
means the monitor stopped receiving messages the node used to publish, within
its own learned inter-arrival timeout (a few hundred ms to a couple of seconds).
This is what actually detects a crash quickly; the graph sweep confirms it
later. The two fields are published separately so a viewer can see the
difference instead of being told a crashed node vanished instantly.

``system_status`` semantics (no causal claim, purely observational):
  WAITING    – graph not discovered yet / detector still warming up
  NORMAL     – no anomaly in the RCA window, no active diagnosis
  DEGRADED   – anomalies in the window and fresh evidence (< recovering_after_sec old)
  RECOVERING – anomalies still in the window but no fresh evidence; the window is draining
"""

from collections import deque
from typing import Any, Deque, Dict, Iterable, List, Optional

from .anomaly_detector import Anomaly
from .graph_engine import DependencyGraph
from .rca_engine import DiagnosisResult

SCHEMA_VERSION = 1
MAX_ANOMALIES = 40
MAX_TIMELINE = 60
MAX_CANDIDATES = 6

# Evidence types that are informational rather than a fault symptom on their own
# (e.g. reordered delivery while a delayed queue drains). The TUI renders them dimmer.
INFORMATIONAL_TYPES = ('out_of_order',)


class EventTimeline:
    """Bounded, oldest-first list of notable runtime events for the dashboard."""

    def __init__(self, maxlen: int = MAX_TIMELINE):
        self._events: Deque[Dict[str, Any]] = deque(maxlen=maxlen)

    def add(self, t: float, node: str, event: str, severity: float = 0.0, message: str = ''):
        # One timeline entry per (node, event, onset): a persisting anomaly is confirmed
        # on every sample and re-emitted timeouts/crashes share their onset, but the
        # timeline lists *when things started*, not every observation.
        for e in self._events:
            if e['node'] == node and e['event'] == event and abs(e['t'] - t) < 0.005:
                e['severity'] = max(e['severity'], float(severity))
                return
        self._events.append({
            't': float(t), 'node': node, 'event': event,
            'severity': float(max(0.0, min(1.0, severity))), 'message': message[:160],
        })

    def add_anomaly(self, a: Anomaly):
        # Use the physical onset for ordering on the timeline, keep record time in the message
        self.add(a.onset, a.node, a.metric.split(':')[-1], a.severity,
                 f'{a.description[:100]} (recorded t={a.timestamp:.2f})')

    def items(self) -> List[Dict[str, Any]]:
        return list(self._events)

    def __len__(self):
        return len(self._events)


def node_liveness(name: str, missing: Iterable[str], anomalous: Iterable[str], in_graph: bool) -> str:
    """Runtime liveness from observed data flow (NOT ROS graph membership)."""
    if name in missing:
        return 'STARVED'
    if name in anomalous or in_graph:
        return 'ALIVE'
    return 'UNKNOWN'


def node_status(name: str, anomalous: Iterable[str], root_cause: Optional[str], missing: Iterable[str]) -> str:
    if name in missing:
        return 'MISSING'
    if root_cause is not None and name == root_cause:
        return 'ROOT_CAUSE'
    if name in anomalous:
        return 'ANOMALOUS'
    return 'NORMAL'


def system_status(
    now: float,
    graph_ready: bool,
    warmed_up: bool,
    recent: List[Anomaly],
    recovering_after_sec: float = 2.0,
) -> str:
    if not graph_ready or not warmed_up:
        return 'WAITING'
    if not recent:
        return 'NORMAL'
    last_record = max(a.timestamp for a in recent)
    return 'DEGRADED' if now - last_record <= recovering_after_sec else 'RECOVERING'


def diagnosis_to_dict(diag: Optional[DiagnosisResult]) -> Optional[Dict[str, Any]]:
    if diag is None or not diag.root_cause or not diag.candidates:
        return None
    top = diag.candidates[0]
    return {
        'timestamp': diag.timestamp,
        'root_cause': diag.root_cause,
        'confidence': round(float(diag.confidence), 4),
        'scores': {
            'dependency': round(top.d_score, 4),
            'temporal': round(top.t_score, 4),
            'symptom': round(top.s_score, 4),
            'severity': round(top.a_score, 4),
        },
        'weights': dict(diag.weights),
        'candidates': [
            {'node': c.node, 'total': round(c.total_score, 4), 'd': round(c.d_score, 4),
             't': round(c.t_score, 4), 's': round(c.s_score, 4), 'a': round(c.a_score, 4)}
            for c in diag.candidates[:MAX_CANDIDATES]
        ],
        'propagation_chain': list(diag.propagation_chain),
        'anomalous_nodes': sorted(diag.anomalous_nodes),
        'explanation': diag.explanation or '',
    }


def build_dashboard_state(
    now: float,
    graph: DependencyGraph,
    recent_anomalies: List[Anomaly],
    diagnosis: Optional[DiagnosisResult],
    timeline: EventTimeline,
    *,
    missing_nodes: Iterable[str] = (),
    monitored_topics: int = 0,
    diagnoses_total: int = 0,
    anomalies_total: int = 0,
    monitor_start_time: float = 0.0,
    warmed_up: bool = True,
    rca_window_sec: float = 6.0,
    simulator: str = 'unknown',
    ros_graph_nodes: Optional[Iterable[str]] = None,
) -> Dict[str, Any]:
    """Builds the schema-version-1 dashboard document from the monitor's live view."""
    gd = graph.to_dict()
    missing = set(missing_nodes)
    diag = diagnosis_to_dict(diagnosis)
    root = diag['root_cause'] if diag else None
    anomalous = {a.node for a in recent_anomalies}

    per_node: Dict[str, Dict[str, Any]] = {}
    for a in sorted(recent_anomalies, key=lambda x: x.timestamp):
        d = per_node.setdefault(a.node, {'anomaly_count': 0, 'max_severity': 0.0,
                                         'last_metric': '', 'last_onset': None})
        d['anomaly_count'] += 1
        d['max_severity'] = max(d['max_severity'], float(a.severity))
        d['last_metric'] = a.metric.split(':')[-1]
        d['last_onset'] = a.onset

    all_nodes = sorted(set(gd['nodes']) | anomalous | missing)
    # ROS graph membership: what the last discovery sweep actually saw. Falls back
    # to the (merged) graph's node set when the caller does not track it separately.
    in_graph = set(ros_graph_nodes) if ros_graph_nodes is not None else set(gd['nodes'])
    nodes = []
    for n in all_nodes:
        info = per_node.get(n, {'anomaly_count': 0, 'max_severity': 0.0, 'last_metric': '', 'last_onset': None})
        nodes.append({'name': n, 'status': node_status(n, anomalous, root, missing),
                      'in_ros_graph': n in in_graph,
                      'liveness': node_liveness(n, missing, anomalous, n in in_graph), **info})

    recent_sorted = sorted(recent_anomalies, key=lambda x: (x.timestamp, x.node), reverse=True)[:MAX_ANOMALIES]
    anomalies = [{
        'node': a.node, 'type': a.metric.split(':')[-1], 'timestamp': a.timestamp,
        'onset_time': a.onset_time, 'severity': round(float(a.severity), 4),
        'value': round(float(a.value), 4), 'description': a.description[:160],
        'informational': a.metric in INFORMATIONAL_TYPES,
    } for a in recent_sorted]

    return {
        'schema_version': SCHEMA_VERSION,
        'timestamp': now,
        'simulator': str(simulator or 'unknown'),
        'monitor_uptime_sec': round(max(0.0, now - monitor_start_time), 1) if monitor_start_time else 0.0,
        'system_status': system_status(now, bool(gd['nodes']), warmed_up, recent_anomalies),
        'counts': {
            'nodes': len(gd['nodes']),
            'edges': len(gd['edges']),
            'topics': int(monitored_topics),
            'anomalies_in_window': len(recent_anomalies),
            'anomalous_nodes': len(anomalous),
            'diagnoses_total': int(diagnoses_total),
            'anomalies_total': int(anomalies_total),
        },
        'rca_window_sec': float(rca_window_sec),
        'nodes': nodes,
        'edges': [{'from': e['from'], 'to': e['to'], 'topics': list(e['topics'])} for e in gd['edges']],
        'anomalies': anomalies,
        'latest_diagnosis': diag,
        'timeline': timeline.items(),
    }
