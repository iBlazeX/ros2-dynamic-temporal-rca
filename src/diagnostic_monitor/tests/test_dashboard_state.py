"""Dashboard state generation (pure) and live publication from the monitor node."""

import json
import time

import pytest

from diagnostic_monitor.anomaly_detector import Anomaly
from diagnostic_monitor.dashboard_state import (
    EventTimeline, MAX_ANOMALIES, MAX_TIMELINE, SCHEMA_VERSION, build_dashboard_state, system_status,
)
from diagnostic_monitor.explainer import Explainer
from diagnostic_monitor.graph_engine import DependencyGraph
from diagnostic_monitor.rca_engine import RCAEngine

GT_TERMS = ('ground_truth', 'target_node', 'fault_type', 'expected_chain', 'physical_chain',
            'fault_command', 'scenario', 'injected')


def _chain():
    g = DependencyGraph()
    g.add_edge('sensor_node', 'perception_node', '/sensor/scan')
    g.add_edge('perception_node', 'localization_node', '/perception/obstacles')
    g.add_edge('localization_node', 'navigation_node', '/localization/pose')
    return g


def _anoms(t0):
    return [
        Anomaly(t0 + 0.3, 'sensor_node', '/sensor/scan', '/sensor/scan:latency', 0.4, 0, 0.01, 40, 1.0, 'late', onset_time=t0 + 0.2),
        Anomaly(t0 + 0.4, 'perception_node', '/perception/obstacles', '/perception/obstacles:latency', 0.4, 0, 0.01, 40, 0.9, 'late', onset_time=t0 + 0.25),
        Anomaly(t0 + 0.5, 'sensor_node', '/sensor/scan', 'out_of_order', -0.1, 0, 1, 5, 0.4, 'reordered'),
    ]


def test_empty_state_is_waiting_and_valid():
    tl = EventTimeline()
    st = build_dashboard_state(100.0, DependencyGraph(), [], None, tl, warmed_up=False)
    assert st['schema_version'] == SCHEMA_VERSION
    assert st['system_status'] == 'WAITING'
    assert st['nodes'] == [] and st['edges'] == [] and st['anomalies'] == []
    assert st['latest_diagnosis'] is None and st['timeline'] == []
    json.dumps(st)  # serialisable


def test_normal_state_from_discovered_graph():
    g = _chain()
    st = build_dashboard_state(100.0, g, [], None, EventTimeline(), monitored_topics=4)
    assert st['system_status'] == 'NORMAL'
    assert [n['name'] for n in st['nodes']] == sorted(g.nodes)
    assert all(n['status'] == 'NORMAL' for n in st['nodes'])
    assert {(e['from'], e['to']) for e in st['edges']} == {
        ('sensor_node', 'perception_node'), ('perception_node', 'localization_node'),
        ('localization_node', 'navigation_node')}
    assert st['counts'] == {'nodes': 4, 'edges': 3, 'topics': 4, 'anomalies_in_window': 0,
                            'anomalous_nodes': 0, 'diagnoses_total': 0, 'anomalies_total': 0}


def test_degraded_state_with_diagnosis_and_no_ground_truth():
    g = _chain(); t0 = 100.0
    anoms = _anoms(t0)
    diag = RCAEngine().diagnose(anoms, g, t0 + 1.0)
    diag.explanation = Explainer.explain(diag, g, anoms)
    tl = EventTimeline()
    for a in anoms:
        tl.add_anomaly(a)
    st = build_dashboard_state(t0 + 1.0, g, anoms, diag, tl, diagnoses_total=3, anomalies_total=3)
    assert st['system_status'] == 'DEGRADED'
    by = {n['name']: n for n in st['nodes']}
    assert by['sensor_node']['status'] == 'ROOT_CAUSE' and by['sensor_node']['anomaly_count'] == 2
    assert by['perception_node']['status'] == 'ANOMALOUS'
    assert by['navigation_node']['status'] == 'NORMAL'
    d = st['latest_diagnosis']
    assert d['root_cause'] == 'sensor_node' and 0 < d['confidence'] <= 1
    assert set(d['scores']) == {'dependency', 'temporal', 'symptom', 'severity'}
    assert d['propagation_chain'] == ['sensor_node', 'perception_node']
    assert 'ROOT CAUSE DIAGNOSIS' in d['explanation']
    assert d['candidates'][0]['node'] == 'sensor_node'
    # anomalies most recent first, informational flag on out_of_order
    assert st['anomalies'][0]['type'] == 'out_of_order' and st['anomalies'][0]['informational'] is True
    assert st['anomalies'][1]['onset_time'] == t0 + 0.25
    # timeline ordered by onset, oldest first
    assert [e['event'] for e in st['timeline']] == ['latency', 'latency', 'out_of_order']
    blob = json.dumps(st).lower()
    assert not any(term in blob for term in GT_TERMS)


def test_recovering_and_missing_nodes():
    g = _chain(); t0 = 100.0
    old = [Anomaly(t0, 'perception_node', '', 'node_crash', 1, 0, 0.01, 100, 1.0, 'crash', onset_time=t0 - 1)]
    st = build_dashboard_state(t0 + 5.0, g, old, None, EventTimeline(), missing_nodes=['perception_node'])
    assert st['system_status'] == 'RECOVERING'
    assert {n['name']: n['status'] for n in st['nodes']}['perception_node'] == 'MISSING'
    assert system_status(t0 + 1.0, True, True, old) == 'DEGRADED'
    assert system_status(t0, False, True, []) == 'WAITING'


def test_bounds_are_enforced():
    tl = EventTimeline()
    for i in range(MAX_TIMELINE + 50):
        tl.add(float(i), 'n', 'e', 0.1, 'm')
    assert len(tl) == MAX_TIMELINE
    tl.add(1.0, 'n', 'x'); tl.add(1.0, 'n', 'x')          # exact repeat collapsed
    assert sum(1 for e in tl.items() if e['event'] == 'x') == 1
    # a persisting anomaly confirmed on every sample shares its onset -> one timeline entry
    tl2 = EventTimeline()
    for i in range(50):
        tl2.add_anomaly(Anomaly(100 + i * 0.1, 'sensor_node', '/t', '/t:latency', 0.4, 0, 1, 5, 0.9, 'd',
                                onset_time=99.9))
        tl2.add_anomaly(Anomaly(100 + i * 0.1, 'perception_node', '/t', '/t:latency', 0.4, 0, 1, 5, 0.9, 'd',
                                onset_time=99.95))
    assert [(e['node'], e['t']) for e in tl2.items()] == [('sensor_node', 99.9), ('perception_node', 99.95)]
    anoms = [Anomaly(100 + i * 0.01, 'sensor_node', '/t', '/t:latency', 1, 0, 1, 5, 0.5, 'd')
             for i in range(MAX_ANOMALIES + 30)]
    st = build_dashboard_state(200.0, _chain(), anoms, None, tl)
    assert len(st['anomalies']) == MAX_ANOMALIES
    assert st['counts']['anomalies_in_window'] == MAX_ANOMALIES + 30
    assert len(json.dumps(st)) < 64 * 1024


# --------------------------------------------------------------- live publication

@pytest.fixture(scope='module')
def ros_context():
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


def test_monitor_publishes_latched_dashboard_state(ros_context, tmp_path, monkeypatch):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from diagnostic_monitor import db_path as dbp
    from diagnostic_monitor.monitor_node import DiagnosticMonitorNode

    monkeypatch.setenv(dbp.ENV_VAR, str(tmp_path / 'events.db'))
    mon = DiagnosticMonitorNode()
    try:
        assert mon.event_store.db_path == str(tmp_path / 'events.db')     # resolver applied
        # publish once (initially dirty)
        mon._publish_dashboard_if_due(time.time())
        got = []
        sub_node = Node('dash_test_sub')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub_node.create_subscription(String, '/rca/dashboard_state', lambda m: got.append(m.data), qos)
        # A real monitor may be running on this machine and publishing the same topic;
        # only accept the state produced by the monitor created in this test (young uptime).
        deadline = time.time() + 5.0
        mine = None
        while mine is None and time.time() < deadline:      # late joiner must receive latched state
            rclpy.spin_once(sub_node, timeout_sec=0.1)
            for raw in got:
                st = json.loads(raw)
                if st.get('monitor_uptime_sec', 1e9) < 30.0:
                    mine = (raw, st)
        assert mine is not None, 'no latched dashboard state received from the test monitor'
        raw, st = mine
        assert st['schema_version'] == SCHEMA_VERSION
        assert st['system_status'] in ('WAITING', 'NORMAL')
        assert not any(term in raw.lower() for term in GT_TERMS)
        # rate limiting: an immediate second call must not publish again
        n_before = mon.last_dashboard_pub
        mon.dashboard_dirty = True
        mon._publish_dashboard_if_due(n_before + 0.1)
        assert mon.last_dashboard_pub == n_before
        # heartbeat: publishes even when clean after heartbeat interval
        mon._publish_dashboard_if_due(n_before + mon.dashboard_heartbeat_sec + 0.01)
        assert mon.last_dashboard_pub > n_before
        sub_node.destroy_node()
    finally:
        mon.event_store.close()
        mon.destroy_node()


def test_monitor_drops_samples_with_unresolvable_publisher(ros_context, tmp_path, monkeypatch):
    """Startup race: a message may arrive before its publisher is resolvable. It must be
    dropped, never attributed to a pseudo-node that later looks like a crashed node."""
    from std_msgs.msg import String
    from diagnostic_monitor import db_path as dbp
    from diagnostic_monitor.monitor_node import DiagnosticMonitorNode
    monkeypatch.setenv(dbp.ENV_VAR, str(tmp_path / 'events.db'))
    mon = DiagnosticMonitorNode()
    try:
        assert mon._publisher_of('/ghost/topic') is None
        mon._handle_topic_message('/ghost/topic', String(data='x'))
        assert 'unknown_publisher' not in mon.detector.active_nodes
        assert '/ghost/topic' not in mon.last_recv_times
        mon.event_store.flush()
        assert mon.event_store.counts()['telemetry_events'] == 0
        # once the graph knows the publisher, samples are attributed correctly
        mon.graph.add_edge('src_node', 'dst_node', '/ghost/topic')
        assert mon._publisher_of('/ghost/topic') == 'src_node'
    finally:
        mon.event_store.close()
        mon.destroy_node()
