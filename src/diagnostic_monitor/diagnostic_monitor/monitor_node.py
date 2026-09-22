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
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .anomaly_detector import Anomaly, AnomalyDetector
from .dashboard_state import EventTimeline, build_dashboard_state
from .db_path import ensure_parent_dir, resolve_db_path
from .event_store import EventStore
from .explainer import Explainer
from .graph_engine import DependencyGraph, RosGraphDiscoverer
from .rca_engine import RCAEngine


class DiagnosticMonitorNode(Node):

    def __init__(self):
        super().__init__('diagnostic_monitor')

        # Parameters
        # '' = auto: resolved by diagnostic_monitor.db_path (RCA_DB_PATH, then <workspace>/events.db)
        self.declare_parameter('db_path', '')
        self.declare_parameter('warmup_sec', 4.0)
        self.declare_parameter('z_threshold', 3.0)
        self.declare_parameter('timeout_multiplier', 3.0)
        self.declare_parameter('confirm_samples', 2)
        self.declare_parameter('rca_window_sec', 6.0)
        self.declare_parameter('w_d', 0.30)
        self.declare_parameter('w_t', 0.30)
        self.declare_parameter('w_s', 0.25)
        self.declare_parameter('w_a', 0.15)
        # Presentation-only label for the environment that produced the data
        # ("rca_sim", "gazebo", "rca_test_system"). It is NOT evaluation ground
        # truth: it names the backend, never the injected fault or its target.
        self.declare_parameter('simulator', 'unknown')
        # Deployment-specific monitoring exclusions (raw simulator transport etc.)
        self.declare_parameter('excluded_topic_prefixes', [''])
        self.declare_parameter('excluded_node_names', [''])
        self.declare_parameter('dashboard_heartbeat_sec', 1.0)
        self.declare_parameter('dashboard_min_interval_sec', 0.5)

        db_path = ensure_parent_dir(resolve_db_path(ros_param=self.get_parameter('db_path').value))
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
        self.simulator_label = str(self.get_parameter('simulator').value or 'unknown')
        extra_topics, extra_nodes = RosGraphDiscoverer.configure(
            topic_prefixes=list(self.get_parameter('excluded_topic_prefixes').value or []),
            node_names=list(self.get_parameter('excluded_node_names').value or []),
        )
        if extra_topics or extra_nodes:
            self.get_logger().info(
                f'Deployment exclusions: topics={list(extra_topics)} nodes={list(extra_nodes)}')
        # Node names present in the last ROS graph discovery sweep (membership),
        # as opposed to the runtime liveness the anomaly detector tracks.
        self.live_graph_nodes = set()
        self.last_stored_graph: Dict[str, Any] = {}
        # Last-known structure of nodes that vanished, kept for the RCA window
        self.vanished_nodes: Dict[str, float] = {}   # node -> time vanished
        self.last_live_graph = DependencyGraph()

        # Dynamic topic subscriptions
        self.subscriptions_map: Dict[str, Any] = {}
        self.last_recv_times: Dict[str, float] = {}
        self.topic_publisher_cache: Dict[str, str] = {}

        # Publisher for diagnosis reports (unchanged semantics)
        self.diagnosis_pub = self.create_publisher(String, '/rca/diagnosis_report', 10)

        # Live dashboard state for the TUI: latched (transient local) so a TUI that
        # starts later immediately receives the latest state; depth 1, reliable.
        dash_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dashboard_pub = self.create_publisher(String, '/rca/dashboard_state', dash_qos)
        self.dashboard_heartbeat_sec = float(self.get_parameter('dashboard_heartbeat_sec').value)
        self.dashboard_min_interval_sec = float(self.get_parameter('dashboard_min_interval_sec').value)
        self.timeline = EventTimeline()
        self.start_time = time.time()
        self.dashboard_dirty = True
        self.last_dashboard_pub = 0.0
        self.diagnoses_total = 0
        self.anomalies_total = 0
        self.last_recent_anoms: List[Anomaly] = []
        self.last_diagnosis = None

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
        # ROS graph membership as of this sweep, kept separate from runtime
        # liveness: a crashed process lingers here until its DDS lease expires.
        self.live_graph_nodes = set(live_graph.nodes)

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
            self.timeline.add(now_time, 'graph', 'graph_discovered', 0.0,
                              f'{len(gd["nodes"])} nodes, {len(gd["edges"])} edges')
            self.dashboard_dirty = True

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

    def _publisher_of(self, topic_name: str) -> Optional[str]:
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
        # None while the publisher cannot be resolved yet (startup race): the sample
        # is dropped rather than attributed to a pseudo-node.
        return self.topic_publisher_cache.get(topic_name)

    def _metric(self, now_time, node_name, topic_name, name, value):
        self.event_store.insert_telemetry(now_time, node_name, topic_name, name, value)
        anom = self.detector.process_metric(now_time, node_name, topic_name, f'{topic_name}:{name}', value)
        if anom:
            self._record_anomaly(anom)

    def _handle_topic_message(self, topic_name: str, msg: Any):
        """Processes received telemetry from monitored topics."""
        now_time = time.time()
        node_name = self._publisher_of(topic_name)
        if node_name is None:
            return

        header_stamp_sec = None
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            stamp = msg.header.stamp
            header_stamp_sec = stamp.sec + (stamp.nanosec * 1e-9)

        # 0. Duplicate / out-of-order guard on header stamps
        if header_stamp_sec is not None and header_stamp_sec > 0:
            is_dup, order_anom = self.detector.check_header_order(now_time, node_name, topic_name, header_stamp_sec)
            if order_anom is not None:
                self._record_anomaly(order_anom)
            # validity indicator: a repeated header stamp is a re-published (stale) measurement
            self._metric(now_time, node_name, topic_name, 'stale_repeat', 1.0 if is_dup else 0.0)
            if is_dup:
                # Exact duplicate stamp: keep liveness, skip rate/latency statistics
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
        elif hasattr(msg, 'poses') and type(msg).__name__ == 'Path':  # nav_msgs/Path-like
            # validity indicator: an empty path means "no plan" (safety stop upstream)
            self._metric(now_time, node_name, topic_name, 'path_empty', 1.0 if not msg.poses else 0.0)
        elif hasattr(msg, 'poses'):                                  # PoseArray-like
            # validity indicator rather than scene content: the number of detected
            # objects legitimately varies while a robot moves, an empty set does not
            self._metric(now_time, node_name, topic_name, 'obstacle_empty', 1.0 if not msg.poses else 0.0)
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):  # PoseStamped-like
            self._metric(now_time, node_name, topic_name, 'z_pos', float(msg.pose.position.z))
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'covariance'):  # PoseWithCovariance(Stamped)-like
            cov = msg.pose.covariance
            if len(cov) >= 8:
                self._metric(now_time, node_name, topic_name, 'pose_uncertainty',
                             math.sqrt(max(0.0, float(cov[0]) + float(cov[7]))))
        elif hasattr(msg, 'twist') or hasattr(msg, 'linear'):        # TwistStamped / Twist-like
            tw = msg.twist if hasattr(msg, 'twist') else msg
            v, w = float(tw.linear.x), float(tw.angular.z)
            self._metric(now_time, node_name, topic_name, 'linear_vel', v)
            # validity indicator: an exactly-zero command is a commanded stop
            self._metric(now_time, node_name, topic_name, 'cmd_zero',
                         1.0 if (abs(v) < 1e-6 and abs(w) < 1e-6) else 0.0)

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
        self.anomalies_total += 1
        self.timeline.add_anomaly(anom)
        self.dashboard_dirty = True

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
            self.diagnoses_total += 1

            if diagnosis.root_cause != self.last_reported_root:
                self.last_reported_root = diagnosis.root_cause
                self.get_logger().info(f'\n{explanation}')
                self.timeline.add(now_time, diagnosis.root_cause, 'diagnosis', diagnosis.confidence,
                                  f'probable root cause {diagnosis.root_cause} '
                                  f'(R={diagnosis.confidence:.3f}, chain {" -> ".join(diagnosis.propagation_chain)})')
                self.dashboard_dirty = True
        else:
            if self.last_reported_root is not None:
                self.get_logger().info('Anomaly window cleared. System operating nominally.')
                self.timeline.add(now_time, 'system', 'window_cleared', 0.0,
                                  'no anomalies in RCA window; operating nominally')
                self.dashboard_dirty = True
            self.last_reported_root = None

        self.last_recent_anoms = recent_anoms
        self.last_diagnosis = diagnosis if diagnosis.root_cause else None
        self._publish_dashboard_if_due(now_time)

    # ------------------------------------------------------------ dashboard

    def build_dashboard(self, now_time: float) -> Dict[str, Any]:
        """Current live state as the schema-v1 document (no ground truth exists here)."""
        warmed_up = (now_time - self.detector.start_time) >= self.detector.warmup_duration_sec
        return build_dashboard_state(
            now_time, self.graph, self.last_recent_anoms, self.last_diagnosis, self.timeline,
            missing_nodes=self.detector.missing_nodes.keys(),
            monitored_topics=len(self.subscriptions_map),
            diagnoses_total=self.diagnoses_total, anomalies_total=self.anomalies_total,
            monitor_start_time=self.start_time, warmed_up=warmed_up,
            rca_window_sec=self.rca_window_sec,
            simulator=self.simulator_label,
            ros_graph_nodes=self.live_graph_nodes,
        )

    def _publish_dashboard_if_due(self, now_time: float):
        """Publishes on change (rate-limited) or as a low-rate heartbeat."""
        since = now_time - self.last_dashboard_pub
        if (self.dashboard_dirty and since >= self.dashboard_min_interval_sec) or \
                since >= self.dashboard_heartbeat_sec:
            msg = String()
            msg.data = json.dumps(self.build_dashboard(now_time))
            self.dashboard_pub.publish(msg)
            self.last_dashboard_pub = now_time
            self.dashboard_dirty = False


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
