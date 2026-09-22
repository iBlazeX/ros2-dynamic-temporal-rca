#!/usr/bin/env python3
"""Runtime dynamic-graph validation against a live backend (rca_sim or Gazebo).

Drives real runtime changes - a node stopping, crashing, respawning, and a node
switching which topic it subscribes to - and records what the diagnostic
monitor's *discovered* graph looks like before, during and after each change.
Nothing about the topology is declared: every snapshot below is read back from
the monitor's SQLite event store, which the monitor wrote from its own ROS 2
graph queries.

It also measures how long a crashed process keeps its DDS graph membership,
which is the number quoted in the README as the DDS liveliness lease.

    python3 scripts/graph_probe.py --backend gazebo --out runs/gazebo/dynamic_graph.json
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import textwrap
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from diagnostic_monitor.db_path import resolve_db_path
from diagnostic_monitor.event_store import EventStore

FAULT_TOPIC = '/rca/fault_command'


class Injector(Node):
    def __init__(self):
        super().__init__('graph_probe_injector')
        self.pub = self.create_publisher(String, FAULT_TOPIC, 10)

    def send(self, target, fault_type, param=0.0, duration=10.0):
        msg = String()
        msg.data = json.dumps({'target_node': target, 'fault_type': fault_type,
                               'param': param, 'duration': duration, 'timestamp': time.time()})
        for _ in range(4):
            self.pub.publish(msg)
            time.sleep(0.05)
        print(f'    injected {fault_type} on {target}')


def graph_now(store, label):
    snap = store.get_latest_graph()
    g = snap['graph'] if snap else {'nodes': [], 'edges': []}
    out = {'label': label, 't': time.time(),
           'nodes': sorted(g['nodes']),
           'edges': sorted([e['from'], e['to']] for e in g['edges'])}
    print(f'  [{label}] {len(out["nodes"])} nodes, {len(out["edges"])} edges')
    return out


def ros_graph_members(node):
    """Node names currently visible in the ROS 2 graph (DDS participants)."""
    return {n for n, _ in node.get_node_names_and_namespaces()}


AUX_NODE_SRC = textwrap.dedent("""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class Aux(Node):
        def __init__(self):
            super().__init__('aux_probe_node')
            self.pub = self.create_publisher(String, '/aux/probe', 10)
            self.create_subscription(LaserScan, '/sim/lidar/scan', self.cb, 10)
            self.create_timer(0.2, self.tick)
        def cb(self, msg):
            pass
        def tick(self):
            m = String(); m.data = 'alive'; self.pub.publish(m)

    rclpy.init()
    n = Aux()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
""")


def start_aux():
    """Starts a real extra ROS node that joins the application data flow."""
    return subprocess.Popen([sys.executable, '-c', AUX_NODE_SRC],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


LEASE_NODE_SRC = textwrap.dedent("""
    import rclpy, time
    from std_msgs.msg import String
    rclpy.init()
    n = rclpy.create_node('dds_lease_probe')
    p = n.create_publisher(String, '/aux/lease_probe', 10)
    t = n.create_timer(0.2, lambda: p.publish(String(data='x')))
    rclpy.spin(n)
""")


def measure_dds_lease(node, timeout=90.0):
    """SIGKILLs a standalone ROS node and times how long its name survives in the
    ROS 2 graph. This is the DDS participant liveliness lease: the delay before
    graph membership reflects a process that died without unregistering."""
    proc = subprocess.Popen([sys.executable, '-c', LEASE_NODE_SRC],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time()
    seen = False
    while time.time() - t0 < 25.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if 'dds_lease_probe' in ros_graph_members(node):
            seen = True
            break
    if not seen:
        proc.kill()
        return {'error': 'lease probe never appeared in the ROS graph'}
    proc.send_signal(signal.SIGKILL)
    t_kill = time.time()
    gone = None
    while time.time() - t_kill < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if 'dds_lease_probe' not in ros_graph_members(node):
            gone = time.time() - t_kill
            break
    proc.wait(timeout=5)
    return {'killed_with': 'SIGKILL (no clean DDS unregister)',
            'left_ros_graph_after_sec': round(gone, 2) if gone is not None else None,
            'timeout_sec': timeout,
            'rmw': os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp (default)')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', required=True, choices=['rca_sim', 'gazebo'])
    ap.add_argument('--out', required=True)
    ap.add_argument('--db', default=None)
    ap.add_argument('--lease-timeout', type=float, default=60.0,
                    help='how long to wait for a crashed node to leave the ROS graph')
    a = ap.parse_args()

    rclpy.init()
    inj = Injector()
    store = EventStore(a.db or resolve_db_path())
    result = {'backend': a.backend, 'simulator': a.backend, 'generated_at': time.time(),
              'snapshots': [], 'checks': {}}

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(inj, timeout_sec=0.1)

    print('1. baseline graph')
    inj.send('all', 'clear')
    spin(8)
    base = graph_now(store, 'baseline')
    result['snapshots'].append(base)
    base_nodes, base_edges = set(base['nodes']), {tuple(e) for e in base['edges']}

    # ---------------------------------------------------- node stops and respawns
    print('2. a node stops and comes back: ROS graph membership sampled directly')
    inj.send('camera_node', 'restart', param=6.0, duration=1.0)
    t_stop = time.time()
    gone_after = None
    back_after = None
    while time.time() - t_stop < 40.0:
        rclpy.spin_once(inj, timeout_sec=0.2)
        members = ros_graph_members(inj)
        if gone_after is None and 'camera_node' not in members:
            gone_after = time.time() - t_stop
        elif gone_after is not None and 'camera_node' in members:
            back_after = time.time() - t_stop
            break
    print(f'    camera_node left the ROS graph after {gone_after}s, returned after {back_after}s')
    stopped = graph_now(store, 'after_camera_stop')
    result['snapshots'].append(stopped)
    spin(15)
    restarted = graph_now(store, 'after_camera_restart')
    result['snapshots'].append(restarted)
    result['checks']['node_stop_and_restart'] = {
        'left_ros_graph_after_sec': round(gone_after, 2) if gone_after is not None else None,
        'returned_to_ros_graph_after_sec': round(back_after, 2) if back_after is not None else None,
        'monitor_graph_after_restart_has_camera_node':
            'camera_node' in set(restarted['nodes']),
        'note': 'a node that exits cleanly unregisters from DDS immediately, so unlike a SIGKILL '
                'it leaves the ROS graph at once; the monitor keeps its last-known edges for the '
                'RCA window so the starvation it caused can still be attributed to it',
    }

    # ------------------------------------------------------------- process crash
    print('3. process crash -> runtime starvation first, DDS graph membership later')
    members_before = ros_graph_members(inj)
    t_crash = time.time()
    inj.send('perception_node', 'crash', duration=1.0)
    lease_sec = None
    while time.time() - t_crash < a.lease_timeout:
        rclpy.spin_once(inj, timeout_sec=0.2)
        if 'perception_node' not in ros_graph_members(inj):
            lease_sec = time.time() - t_crash
            break
    first_anom = store.get_anomalies_between(t_crash, time.time() + 1)
    starvation_sec = (first_anom[0]['timestamp'] - t_crash) if first_anom else None
    print(f'    first anomaly after crash: {starvation_sec}s; '
          f'left the ROS graph after: {lease_sec}s')
    result['checks']['crash_detection'] = {
        'first_anomaly_sec': round(starvation_sec, 3) if starvation_sec is not None else None,
        'left_ros_graph_after_sec': round(lease_sec, 2) if lease_sec is not None else None,
        'ros_graph_timeout_sec': a.lease_timeout,
        'node_was_in_graph_before': 'perception_node' in members_before,
        'note': 'runtime starvation detects the crash; DDS graph membership only confirms it later',
    }
    result['snapshots'].append(graph_now(store, 'after_perception_crash'))
    spin(20)
    result['snapshots'].append(graph_now(store, 'after_perception_respawn'))

    # -------------------------------------------------- topic endpoint change
    print('4. runtime topic change -> the planner re-subscribes to another pose source')
    inj.send('planning_node', 'topic_change', param=1.0, duration=12.0)
    spin(10)
    changed = graph_now(store, 'after_planning_topic_change')
    result['snapshots'].append(changed)
    inj.send('all', 'clear')
    spin(15)
    result['snapshots'].append(graph_now(store, 'after_clear'))

    # ------------------------------------------------------------------ checks
    changed_edges = {tuple(e) for e in changed['edges']}
    final = next(s for s in result['snapshots'] if s['label'] == 'after_clear')
    result['checks']['node_disappeared_on_stop'] = {
        'camera_node_in_baseline': 'camera_node' in base_nodes,
        'camera_node_in_monitor_graph_while_down': 'camera_node' in set(stopped['nodes']),
        'camera_node_after_restart': 'camera_node' in set(restarted['nodes']),
    }
    result['checks']['topic_endpoint_change'] = {
        'baseline_has_localization_to_planning': ('localization_node', 'planning_node') in base_edges,
        'changed_has_localization_to_planning': ('localization_node', 'planning_node') in changed_edges,
        'changed_has_odometry_to_planning': ('odometry_node', 'planning_node') in changed_edges,
        'edges_differ_from_baseline': changed_edges != base_edges,
    }
    result['checks']['graph_recovered'] = {
        'final_nodes_equal_baseline': set(final['nodes']) == base_nodes,
        'final_edges_equal_baseline': {tuple(e) for e in final['edges']} == base_edges,
    }
    # ------------------------------------------- node appearance / disappearance
    print('5. a new node joins the running system, then leaves it cleanly')
    before = set(graph_now(store, 'before_aux_node')['nodes'])
    aux = start_aux()
    spin(12)
    with_aux = graph_now(store, 'with_aux_node')
    result['snapshots'].append(with_aux)
    aux.send_signal(signal.SIGINT)
    try:
        aux.wait(timeout=10)
    except subprocess.TimeoutExpired:
        aux.kill()
    spin(14)
    without_aux = graph_now(store, 'after_aux_node_left')
    result['snapshots'].append(without_aux)
    result['checks']['node_appearance'] = {
        'aux_in_graph_before': 'aux_probe_node' in before,
        'aux_in_graph_while_running': 'aux_probe_node' in set(with_aux['nodes']),
        'aux_edge_lidar_to_aux': ['lidar_node', 'aux_probe_node'] in with_aux['edges'],
        'aux_in_graph_after_clean_exit': 'aux_probe_node' in set(without_aux['nodes']),
    }

    # ------------------------------------------------- DDS participant lease
    print('6. DDS participant lease: how long a SIGKILLed node keeps graph membership')
    result['checks']['dds_participant_lease'] = measure_dds_lease(inj)
    print('   ', result['checks']['dds_participant_lease'])

    result['checks']['snapshots_recorded'] = store.counts().get('graph_snapshots', 0)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(result, open(a.out, 'w'), indent=2)
    print('\n' + json.dumps(result['checks'], indent=2))
    print(f'written to {a.out}')

    store.close()
    inj.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
