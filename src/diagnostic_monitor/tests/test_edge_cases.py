"""Edge-case tests: startup noise, single-sample jitter, zero std, timeouts ordering,
duplicate / out-of-order events, crashed nodes, insufficient evidence."""

import random

from diagnostic_monitor.anomaly_detector import Anomaly, AnomalyDetector, MetricBaseline
from diagnostic_monitor.event_store import EventStore
from diagnostic_monitor.explainer import Explainer
from diagnostic_monitor.graph_engine import DependencyGraph
from diagnostic_monitor.rca_engine import RCAEngine


def _chain_graph():
    g = DependencyGraph()
    g.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    g.add_edge('perception_node', 'localization_node', '/perception/obstacles')
    g.add_edge('localization_node', 'navigation_node', '/localization/pose')
    return g


def _trained_detector(confirm=2):
    det = AnomalyDetector(warmup_duration_sec=0.1, z_threshold=3.0, confirm_samples=confirm)
    det.start_time = 0.0
    t = 10.0
    for i in range(30):
        t += 0.1
        det.process_metric(t, 'sensor_node', '/sensor/scan', '/sensor/scan:inter_arrival',
                           0.1 + (0.003 if i % 2 else -0.003))
    return det, t


def test_single_sample_jitter_is_not_an_anomaly():
    """A lone 0.4s hiccup (WSL scheduling jitter) must NOT be reported."""
    det, t = _trained_detector(confirm=2)
    a = det.process_metric(t + 0.4, 'sensor_node', '/sensor/scan', '/sensor/scan:inter_arrival', 0.4)
    assert a is None
    # Back to normal -> streak resets, still nothing
    a = det.process_metric(t + 0.5, 'sensor_node', '/sensor/scan', '/sensor/scan:inter_arrival', 0.1)
    assert a is None
    # Baseline was not contaminated by the outlier
    assert abs(det.baselines[('sensor_node', '/sensor/scan:inter_arrival')].mean - 0.1) < 0.01


def test_persistent_deviation_is_confirmed():
    det, t = _trained_detector(confirm=2)
    assert det.process_metric(t + 0.4, 'sensor_node', '/sensor/scan', '/sensor/scan:inter_arrival', 0.4) is None
    a = det.process_metric(t + 0.8, 'sensor_node', '/sensor/scan', '/sensor/scan:inter_arrival', 0.4)
    assert a is not None and a.node == 'sensor_node' and 'confirmed x2' in a.description
    # onset is the first sample of the streak, record time the confirming sample
    assert a.onset == t + 0.4 and a.timestamp == t + 0.8


def test_zero_std_dev_still_detects_with_floor():
    b = MetricBaseline(warmup_samples=5, epsilon_std=1e-4)
    for _ in range(20):
        b.update(0.0)
    assert b.std == 1e-4  # floor, never 0 -> no division by zero
    det = AnomalyDetector(warmup_duration_sec=0.1, confirm_samples=1)
    det.start_time = 0.0
    for i in range(30):
        det.process_metric(10 + i * 0.1, 'localization_node', '/localization/pose',
                           '/localization/pose:z_pos', 0.0)
    a = det.process_metric(14.0, 'localization_node', '/localization/pose', '/localization/pose:z_pos', 99.9)
    assert a is not None and a.severity == 1.0


def test_startup_noise_rejected_by_time_and_sample_warmup():
    det = AnomalyDetector(warmup_duration_sec=5.0, confirm_samples=1)
    now = det.start_time
    assert det.process_metric(now + 1.0, 'n', '/t', '/t:latency', 100.0) is None
    # After the time warmup but with too few samples: still learning
    assert det.process_metric(now + 6.0, 'n', '/t', '/t:latency', 100.0) is None


def test_timeout_onset_timestamps_preserve_ordering():
    det = AnomalyDetector(warmup_duration_sec=0.1, timeout_multiplier=3.0)
    det.start_time = 0.0
    base = 10.0
    streams = [('sensor_node', '/sensor/scan'), ('perception_node', '/perception/obstacles'),
               ('localization_node', '/localization/pose')]
    for i in range(30):
        for j, (n, tp) in enumerate(streams):
            det.process_metric(base + i * 0.1 + j * 0.002, n, tp, f'{tp}:inter_arrival', 0.1)
    # everything goes silent; the check happens 3s later, in one tick
    timeouts = det.check_timeouts(base + 6.0)
    assert {a.node for a in timeouts} == {'sensor_node', 'perception_node', 'localization_node'}
    by_node = {a.node: a.onset for a in timeouts}
    assert by_node['sensor_node'] < by_node['perception_node'] < by_node['localization_node']
    # onset = last seen + threshold; record time = check time (keeps evidence in the window)
    assert all(a.onset < base + 6.0 and a.timestamp == base + 6.0 for a in timeouts)


def test_missing_node_is_re_reported_with_original_onset():
    det = AnomalyDetector(warmup_duration_sec=0.1, crash_realert_sec=2.0)
    det.start_time = 0.0
    det.process_metric(10.0, 'perception_node', '/perception/obstacles', 'm', 1.0)
    det.process_metric(10.0, 'sensor_node', '/sensor/scan', 'm', 1.0)
    first = det.check_node_crashes({'sensor_node'}, 31.0)      # noticed 21s later (DDS lease)
    assert len(first) == 1 and first[0].onset == 10.0 and first[0].timestamp == 31.0
    assert det.check_node_crashes({'sensor_node'}, 32.0) == []  # rate limited
    again = det.check_node_crashes({'sensor_node'}, 33.5)
    assert len(again) == 1 and again[0].onset == 10.0 and again[0].timestamp == 33.5
    assert det.check_node_crashes({'sensor_node', 'perception_node'}, 40.0) == []  # restarted
    assert 'perception_node' not in det.missing_nodes


def test_duplicate_and_out_of_order_headers():
    det = AnomalyDetector(warmup_duration_sec=0.1)
    is_dup, anom = det.check_header_order(1.0, 'sensor_node', '/sensor/scan', 100.0)
    assert not is_dup and anom is None
    is_dup, anom = det.check_header_order(1.1, 'sensor_node', '/sensor/scan', 100.0)
    assert is_dup and anom is None and det.duplicate_count == 1
    is_dup, anom = det.check_header_order(1.2, 'sensor_node', '/sensor/scan', 99.0)
    assert not is_dup and anom is not None and anom.metric == 'out_of_order'


def test_rca_is_invariant_to_duplicate_and_shuffled_events():
    g = _chain_graph()
    t0 = 100.0
    anoms = [
        Anomaly(t0 + 0.0, 'sensor_node', '/sensor/scan', 'latency', 0.4, 0.0, 0.01, 40.0, 1.0, ''),
        Anomaly(t0 + 0.1, 'perception_node', '/perception/obstacles', 'latency', 0.4, 0.0, 0.01, 40.0, 1.0, ''),
        Anomaly(t0 + 0.2, 'localization_node', '/localization/pose', 'latency', 0.4, 0.0, 0.01, 40.0, 1.0, ''),
    ]
    rca = RCAEngine()
    ref = rca.diagnose(anoms, g, t0 + 1.0)
    dup = anoms + anoms + anoms
    random.Random(7).shuffle(dup)
    out = rca.diagnose(dup, g, t0 + 1.0)
    assert [c.node for c in out.candidates] == [c.node for c in ref.candidates]
    assert [round(c.total_score, 6) for c in out.candidates] == \
        [round(c.total_score, 6) for c in ref.candidates]


def test_crashed_node_keeps_last_known_edges_for_attribution():
    """After perception crashes, the live graph has no perception edges. Retaining its
    last-known edges lets the RCA credit it with downstream starvation."""
    before = _chain_graph()
    live = DependencyGraph()
    live.add_node('sensor_node')
    live.add_node('localization_node')
    live.add_node('navigation_node')
    live.add_edge('localization_node', 'navigation_node', '/localization/pose')
    merged = DependencyGraph.from_dict(live.to_dict())
    merged.retain_edges_of(before, {'perception_node'})
    assert merged.successors('perception_node') == {'localization_node'}
    assert merged.predecessors('perception_node') == {'sensor_node'}

    t0 = 50.0
    anoms = [
        Anomaly(t0 + 0.0, 'perception_node', '', 'node_crash', 1.0, 0.0, 0.01, 100.0, 1.0, 'crash'),
        Anomaly(t0 + 0.45, 'localization_node', '/localization/pose', 'message_timeout',
                0.5, 0.1, 0.02, 5.0, 0.8, ''),
        Anomaly(t0 + 0.46, 'navigation_node', '/navigation/cmd_vel', 'message_timeout',
                0.5, 0.1, 0.02, 5.0, 0.8, ''),
    ]
    diag = RCAEngine().diagnose(anoms, merged, t0 + 2.0)
    assert diag.root_cause == 'perception_node'
    # Without retention the crashed node cannot explain the symptoms
    diag_live = RCAEngine().diagnose(anoms, live, t0 + 2.0)
    p = next(c for c in diag_live.candidates if c.node == 'perception_node')
    assert p.s_score < 1.0


def test_insufficient_evidence_and_unknown_nodes():
    g = _chain_graph()
    assert RCAEngine().diagnose([], g, 1.0).root_cause is None
    # Anomaly on a node that is not in the discovered graph at all
    anoms = [Anomaly(1.0, 'ghost_node', '/x', 'latency', 1.0, 0.0, 0.01, 10.0, 0.5, '')]
    d = RCAEngine().diagnose(anoms, g, 2.0)
    assert d.root_cause == 'ghost_node'
    lines, nodes = Explainer.build_propagation_chain('ghost_node', g, anoms)
    assert nodes == ['ghost_node']


def test_event_store_graph_snapshots_and_range_queries(tmp_path):
    store = EventStore(str(tmp_path / 'e.db'))
    g = _chain_graph().to_dict()
    store.insert_graph_snapshot(10.0, g)
    store.insert_anomaly(11.0, 'a', '/t', 'm', 1, 0, 1, 3, 0.5, 'x')
    store.insert_anomaly(12.0, 'b', '/t', 'm', 1, 0, 1, 3, 0.5, 'x')
    store.insert_diagnosis(12.5, 'a', 0.9, 0.9, {'propagation_chain': ['a', 'b']}, [], 'e')
    assert store.get_latest_graph()['graph'] == g
    assert store.get_latest_graph(before_time=9.0) is None
    assert [a['node'] for a in store.get_anomalies_between(11.5, 13.0)] == ['b']
    assert store.get_diagnoses_between(12.0, 13.0)[0]['breakdown']['propagation_chain'] == ['a', 'b']
    assert store.counts()['graph_snapshots'] == 1
    store.close()
