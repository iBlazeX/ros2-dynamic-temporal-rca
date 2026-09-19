"""Main Diagnostic Monitor ROS 2 node.

Performs dynamic graph discovery, non-invasive runtime topic telemetry collection,
baseline statistical anomaly detection, SQLite storage, and multi-metric RCA.

The monitor has no knowledge of fault injection or ground truth: it only
observes application topics discovered at runtime through the ROS 2 graph API.
"""

import importlib
import json
import math
import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .anomaly_detector import Anomaly, AnomalyDetector
from .event_store import EventStore
from .explainer import Explainer
from .graph_engine import DependencyGraph, RosGraphDiscoverer
from .rca_engine import RCAEngine


class DiagnosticMonitorNode(Node):

    def __init__(self):
        super().__init__('diagnostic_monitor')

        # Parameters
        self.declare_parameter('db_path', 'events.db')
        self.declare_parameter('warmup_sec', 4.0)
        self.declare_parameter('z_threshold', 3.0)
        self.declare_parameter('timeout_multiplier', 3.0)
        self.declare_parameter('confirm_samples', 2)
        self.declare_parameter('rca_window_sec', 6.0)
        self.declare_parameter('w_d', 0.30)
        self.declare_parameter('w_t', 0.30)
        self.declare_parameter('w_s', 0.25)
        self.declare_parameter('w_a', 0.15)

        db_path = self.get_parameter('db_path').value
        warmup_sec = float(self.get_parameter('warmup_sec').value)
        z_threshold = float(self.get_parameter('z_threshold').value)
        timeout_mult = float(self.get_parameter('timeout_multiplier').value)
        confirm_samples = int(self.get_parameter('confirm_samples').value)
        self.rca_window_sec = float(self.get_parameter('rca_window_sec').value)

        w_d = float(self.get_parameter('w_d').value)
        w_t = float(self.get_parameter('w_t').value)
        w_s = float(self.get_parameter('w_s').value)
        w_a = float(self.get_parameter('w_a').value)

        # Core Engines
        self.event_store = EventStore(db_path)
        self.detector = AnomalyDetector(
            warmup_duration_sec=warmup_sec,
            z_threshold=z_threshold,
            timeout_multiplier=timeout_mult,
            confirm_samples=confirm_samples,
        )
        self.rca_engine = RCAEngine(
            w_d=w_d, w_t=w_t, w_s=w_s, w_a=w_a,
            time_window_sec=self.rca_window_sec,
        )
        self.graph = DependencyGraph()
        self.last_stored_graph: Dict[str, Any] = {}
        # Last-known structure of nodes that vanished, kept for the RCA window
        self.vanished_nodes: Dict[str, float] = {}   # node -> time vanished
        self.last_live_graph = DependencyGraph()

        # Dynamic topic subscriptions
        self.subscriptions_map: Dict[str, Any] = {}
        self.last_recv_times: Dict[str, float] = {}
        self.topic_publisher_cache: Dict[str, str] = {}

        # Publisher for diagnosis reports
        self.diagnosis_pub = self.create_publisher(String, '/rca/diagnosis_report', 10)

        # Timers
        self.graph_timer = self.create_timer(1.0, self._update_graph)
        self.monitor_timer = self.create_timer(0.5, self._check_timeouts_and_diagnose)

        self.last_reported_root = None
        self.get_logger().info(
            f'DiagnosticMonitorNode started. DB: {db_path}, Weights: (wD={w_d}, wT={w_t}, wS={w_s}, wA={w_a}), '
            f'z>{z_threshold} confirmed x{confirm_samples}, warmup {warmup_sec}s'
        )

    # ------------------------------------------------------------------ helpers

    def _get_msg_class(self, type_str: str):
        """Dynamically resolves ROS 2 message type from string e.g. 'sensor_msgs/msg/LaserScan'."""
        try:
            parts = type_str.split('/')
            if len(parts) == 3:
                pkg, _, msg_name = parts
                mod = importlib.import_module(f'{pkg}.msg')
                return getattr(mod, msg_name)
        except Exception:
            pass
        return None

    # ------------------------------------------------------------- graph timer

    def _update_graph(self):
        """Periodically discovers live computational graph without hardcoding."""
        now_time = time.time()
        live_graph = RosGraphDiscoverer.discover(self)

        # Crash detection: nodes that were active and are now gone
        crash_anoms = self.detector.check_node_crashes(live_graph.nodes, now_time)
        for a in crash_anoms:
            self._record_anomaly(a)
            if a.node not in self.vanished_nodes:
                self.get_logger().fatal(f'CRASH DETECTED: {a.description}')
            self.vanished_nodes[a.node] = now_time

        # Keep last-known structure while the detector still reports the node missing
        for n in list(self.vanished_nodes):
            if n not in self.detector.missing_nodes:
                self.vanished_nodes.pop(n)

        # Effective graph = live graph + last-known edges of recently vanished nodes.
        # Without this, a crashed node loses all its edges the instant it dies and
        # could never be credited with the downstream starvation it caused.
        merged = DependencyGraph()
        for n in live_graph.nodes:
            merged.add_node(n)
        for (u, v), topics in live_graph.edge_topics.items():
            for t in topics:
                merged.add_edge(u, v, t)
        if self.vanished_nodes:
            merged.retain_edges_of(self.last_live_graph, set(self.vanished_nodes))

        if live_graph.nodes:
            self.last_live_graph = DependencyGraph.from_dict(live_graph.to_dict())
            if self.vanished_nodes:
                self.last_live_graph.retain_edges_of(self.graph, set(self.vanished_nodes))
        self.graph = merged

        # Persist the discovered graph whenever it changes
        gd = self.graph.to_dict()
        if gd != self.last_stored_graph and gd['nodes']:
            self.event_store.insert_graph_snapshot(now_time, gd)
            self.last_stored_graph = gd
            edges = ', '.join(f"{e['from']}->{e['to']}" for e in gd['edges'])
            self.get_logger().info(f'Discovered graph: nodes={gd["nodes"]} edges=[{edges}]')

        # Discover topics and dynamically subscribe to monitor them
        topic_types = self.get_topic_names_and_types()
        for topic_name, types in topic_types:
            if RosGraphDiscoverer.should_ignore_topic(topic_name):
                continue
            if topic_name in self.subscriptions_map:
                continue

            msg_cls = self._get_msg_class(types[0]) if types else None
            if msg_cls is not None:
                try:
                    sub = self.create_subscription(
                        msg_cls,
                        topic_name,
                        lambda msg, t_name=topic_name: self._handle_topic_message(t_name, msg),
                        10
                    )
                    self.subscriptions_map[topic_name] = sub
                    self.get_logger().info(f'Dynamically subscribed to monitor topic: {topic_name} [{types[0]}]')
                except Exception as e:
                    self.get_logger().error(f'Failed to dynamically subscribe to {topic_name}: {e}')

    # ------------------------------------------------------------ telemetry

    def _publisher_of(self, topic_name: str) -> str:
        pub_nodes = [
            u for (u, v), topics in self.graph.edge_topics.items()
            if topic_name in topics
        ]
        if not pub_nodes:
            try:
                for p in self.get_publishers_info_by_topic(topic_name):
                    if not RosGraphDiscoverer.should_ignore_node(p.node_name):
                        pub_nodes.append(p.node_name)
            except Exception:
                pass
        if pub_nodes:
            self.topic_publisher_cache[topic_name] = sorted(pub_nodes)[0]
        return self.topic_publisher_cache.get(topic_name, 'unknown_publisher')

    def _metric(self, now_time, node_name, topic_name, name, value):
        self.event_store.insert_telemetry(now_time, node_name, topic_name, name, value)
        anom = self.detector.process_metric(now_time, node_name, topic_name, f'{topic_name}:{name}', value)
        if anom:
            self._record_anomaly(anom)

    def _handle_topic_message(self, topic_name: str, msg: Any):
        """Processes received telemetry from monitored topics."""
        now_time = time.time()
        node_name = self._publisher_of(topic_name)

        header_stamp_sec = None
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            stamp = msg.header.stamp
            header_stamp_sec = stamp.sec + (stamp.nanosec * 1e-9)

        # 0. Duplicate / out-of-order guard on header stamps
        if header_stamp_sec is not None and header_stamp_sec > 0:
            is_dup, order_anom = self.detector.check_header_order(now_time, node_name, topic_name, header_stamp_sec)
            if order_anom is not None:
                self._record_anomaly(order_anom)
            if is_dup:
                # Exact duplicate stamp: keep liveness, skip statistics
                self.detector.last_seen_times[(node_name, topic_name)] = now_time
                self.last_recv_times[topic_name] = now_time
                return

        # 1. Inter-arrival time metric
        last_time = self.last_recv_times.get(topic_name)
        self.last_recv_times[topic_name] = now_time
        if last_time is not None:
            self._metric(now_time, node_name, topic_name, 'inter_arrival', now_time - last_time)
        else:
            self.detector.last_seen_times[(node_name, topic_name)] = now_time

        # 2. End-to-end latency metric: now - source acquisition stamp (if header present)
        if header_stamp_sec is not None and header_stamp_sec > 0:
            self._metric(now_time, node_name, topic_name, 'latency', now_time - header_stamp_sec)

        # 3. Payload-specific validity metrics (duck-typed on message fields)
        if hasattr(msg, 'ranges'):                                   # LaserScan-like
            nan_count = sum(1 for r in msg.ranges if math.isnan(r) or math.isinf(r))
            self._metric(now_time, node_name, topic_name, 'nan_ratio', nan_count / max(1, len(msg.ranges)))
        elif hasattr(msg, 'poses'):                                  # PoseArray-like
            self._metric(now_time, node_name, topic_name, 'obstacle_count', float(len(msg.poses)))
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):  # PoseStamped-like
            self._metric(now_time, node_name, topic_name, 'z_pos', float(msg.pose.position.z))
        elif hasattr(msg, 'twist'):                                  # TwistStamped-like
            self._metric(now_time, node_name, topic_name, 'linear_vel', float(msg.twist.linear.x))
        elif hasattr(msg, 'linear'):                                 # Twist-like
            self._metric(now_time, node_name, topic_name, 'linear_vel', float(msg.linear.x))

    def _record_anomaly(self, anom: Anomaly):
        self.event_store.insert_anomaly(
            timestamp=anom.timestamp, node=anom.node, topic=anom.topic,
            metric=anom.metric, value=anom.value, baseline_mean=anom.baseline_mean,
            baseline_std=anom.baseline_std, z_score=anom.z_score,
            severity=anom.severity, description=anom.description,
            onset_time=anom.onset_time,
        )
        self.get_logger().warn(
            f'ANOMALY on [{anom.node}] {anom.metric}: {anom.description} (sev={anom.severity:.2f})'
        )

    # ------------------------------------------------------------ diagnosis

    def _check_timeouts_and_diagnose(self):
        """Checks for starvation timeouts and computes RCA diagnosis."""
        now_time = time.time()
        self.event_store.flush()

        for a in self.detector.check_timeouts(now_time):
            self._record_anomaly(a)

        recent_anoms: List[Anomaly] = [
            Anomaly.from_dict(d)
            for d in self.event_store.get_recent_anomalies(self.rca_window_sec, now_time)
        ]

        diagnosis = self.rca_engine.diagnose(recent_anoms, self.graph, now_time)
        explanation = Explainer.explain(diagnosis, self.graph, recent_anoms)
        diagnosis.explanation = explanation

        if diagnosis.root_cause:
            top = diagnosis.candidates[0]
            breakdown = top.to_dict()
            breakdown['propagation_chain'] = list(diagnosis.propagation_chain)
            breakdown['anomalous_nodes'] = sorted(diagnosis.anomalous_nodes)
            breakdown['graph'] = self.graph.to_dict()
            self.event_store.insert_diagnosis(
                timestamp=diagnosis.timestamp,
                root_cause=diagnosis.root_cause,
                confidence=diagnosis.confidence,
                score=top.total_score,
                breakdown=breakdown,
                rank_list=[c.to_dict() for c in diagnosis.candidates],
                explanation=explanation,
            )

            report_msg = String()
            report_msg.data = json.dumps(diagnosis.to_dict())
            self.diagnosis_pub.publish(report_msg)

            if diagnosis.root_cause != self.last_reported_root:
                self.last_reported_root = diagnosis.root_cause
                self.get_logger().info(f'\n{explanation}')
        else:
            if self.last_reported_root is not None:
                self.get_logger().info('Anomaly window cleared. System operating nominally.')
            self.last_reported_root = None


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.event_store.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
