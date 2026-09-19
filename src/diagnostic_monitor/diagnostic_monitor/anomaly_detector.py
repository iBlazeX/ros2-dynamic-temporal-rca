"""Statistical baseline anomaly detector with robust edge-case handling.

Maintains running mean and standard deviation baselines for telemetry metrics.
Handles zero std-dev, startup noise, missing data / dropouts, duplicate/out-of-order
events, and node crashes.

Design notes (research prototype, deterministic and interpretable):
- A z-score anomaly is only *emitted* after `confirm_samples` consecutive
  out-of-band observations. A single late/odd sample (OS scheduling jitter,
  WSL2 stalls) is therefore never reported as an anomaly on its own.
- Anomalous samples never update the baseline, so a fault does not
  contaminate the "normal" model and detection persists for its duration.
- Every anomaly carries two times: `timestamp` (when the evidence was
  recorded; used for the sliding analysis window) and `onset_time` (when the
  deviation physically started; used for temporal ordering). For timeouts the
  onset is last-message-time + threshold, for crashes it is the last moment the
  node was seen alive, for statistical anomalies it is the first sample of the
  confirmed outlier streak. Ongoing conditions (starvation, a node still
  missing) are re-emitted periodically so their evidence stays in the window.
"""

import math
import time
from typing import Dict, List, Optional, Set, Tuple


class MetricBaseline:
    """Tracks running statistics for a single metric stream with variance floors."""

    def __init__(self, warmup_samples: int = 25, epsilon_std: float = 0.025, min_abs_dev: float = 0.10):
        self.warmup_samples = warmup_samples
        self.epsilon_std = epsilon_std
        self.min_abs_dev = min_abs_dev
        self.count = 0
        self.sum = 0.0
        self.sum_sq = 0.0
        self.mean = 0.0
        self.std = 0.0
        self.is_warmed_up = False
        # Consecutive out-of-band observations (confirmation counter) and when the streak began
        self.outlier_streak = 0
        self.streak_start: Optional[float] = None

    def update(self, value: float):
        self.count += 1
        self.sum += value
        self.sum_sq += value * value

        if self.count >= 2:
            self.mean = self.sum / self.count
            variance = (self.sum_sq - self.count * (self.mean ** 2)) / (self.count - 1)
            variance = max(0.0, variance)
            self.std = math.sqrt(variance)
        else:
            self.mean = value
            self.std = self.epsilon_std

        # Handle zero std-dev: ensure minimum variance floor to absorb OS thread scheduling jitter
        if self.std < self.epsilon_std:
            self.std = self.epsilon_std

        if self.count >= self.warmup_samples:
            self.is_warmed_up = True


class Anomaly:
    """Represents a detected anomaly event."""

    def __init__(
        self,
        timestamp: float,
        node: str,
        topic: str,
        metric: str,
        value: float,
        baseline_mean: float,
        baseline_std: float,
        z_score: float,
        severity: float,
        description: str,
        onset_time: Optional[float] = None,
    ):
        self.timestamp = timestamp
        self.onset_time = onset_time
        self.node = node
        self.topic = topic
        self.metric = metric
        self.value = value
        self.baseline_mean = baseline_mean
        self.baseline_std = baseline_std
        self.z_score = z_score
        self.severity = min(1.0, max(0.0, severity))
        self.description = description

    @property
    def onset(self) -> float:
        """Physical onset time of the deviation (falls back to record time)."""
        return self.onset_time if self.onset_time is not None else self.timestamp

    def to_dict(self) -> dict:
        return {
            'timestamp': self.timestamp,
            'onset_time': self.onset_time,
            'node': self.node,
            'topic': self.topic,
            'metric': self.metric,
            'value': self.value,
            'baseline_mean': self.baseline_mean,
            'baseline_std': self.baseline_std,
            'z_score': self.z_score,
            'severity': self.severity,
            'description': self.description,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'Anomaly':
        return cls(
            timestamp=d['timestamp'], node=d['node'], topic=d['topic'], metric=d['metric'],
            value=d['value'], baseline_mean=d['baseline_mean'], baseline_std=d['baseline_std'],
            z_score=d['z_score'], severity=d['severity'], description=d['description'],
            onset_time=d.get('onset_time'),
        )


class AnomalyDetector:
    """Statistical anomaly detector with startup grace period and timeout monitors."""

    def __init__(
        self,
        warmup_duration_sec: float = 4.0,
        z_threshold: float = 3.0,
        timeout_multiplier: float = 3.0,
        epsilon_std: float = 1e-4,
        confirm_samples: int = 2,
        timeout_floor_sec: float = 0.45,
        crash_realert_sec: float = 2.0,
    ):
        self.warmup_duration_sec = warmup_duration_sec
        self.z_threshold = z_threshold
        self.timeout_multiplier = timeout_multiplier
        self.epsilon_std = epsilon_std
        self.confirm_samples = max(1, int(confirm_samples))
        self.timeout_floor_sec = timeout_floor_sec
        self.crash_realert_sec = crash_realert_sec

        self.start_time = time.time()
        self.baselines: Dict[Tuple[str, str], MetricBaseline] = {}  # (node, metric) -> MetricBaseline
        self.last_seen_times: Dict[Tuple[str, str], float] = {}     # (node, topic) -> last_time
        self.last_timeout_alert: Dict[Tuple[str, str], float] = {}  # (node, topic) -> last_alert_time
        self.last_timestamps: Dict[Tuple[str, str], float] = {}    # (node, topic) -> last_header_time
        self.active_nodes: Set[str] = set()
        self.missing_nodes: Dict[str, Tuple[float, float]] = {}    # node -> (last_seen_alive, last_alert)
        self.duplicate_count = 0

    def check_header_order(
        self, timestamp: float, node: str, topic: str, header_timestamp: float
    ) -> Tuple[bool, Optional[Anomaly]]:
        """Checks header timestamps for duplicates / out-of-order delivery.

        Returns (is_duplicate, anomaly). Duplicates are counted and ignored
        (the caller should skip metric updates for them). Out-of-order headers
        produce a low-severity evidence anomaly.
        """
        key_hdr = (node, topic)
        last_hdr = self.last_timestamps.get(key_hdr)
        if last_hdr is not None:
            if header_timestamp == last_hdr:
                self.duplicate_count += 1
                return True, None
            if header_timestamp < last_hdr:
                self.last_timestamps[key_hdr] = header_timestamp
                return False, Anomaly(
                    timestamp=timestamp, node=node, topic=topic, metric='out_of_order',
                    value=header_timestamp - last_hdr, baseline_mean=0.0, baseline_std=1.0,
                    z_score=5.0, severity=0.4,
                    description=f'Out of order header timestamp ({header_timestamp:.3f} < {last_hdr:.3f})',
                )
        self.last_timestamps[key_hdr] = header_timestamp
        return False, None

    def _min_abs_dev_for(self, metric: str) -> float:
        # Physical minimum deviations: guards against flagging pure scheduling jitter
        if 'inter_arrival' in metric:
            return 0.18
        if 'latency' in metric:
            return 0.20
        if 'nan_ratio' in metric:
            return 0.20
        if 'z_pos' in metric:
            return 2.0
        if 'obstacle_count' in metric:
            return 1.0
        if 'linear_vel' in metric:
            return 0.25
        return 0.15

    def process_metric(
        self,
        timestamp: float,
        node: str,
        topic: str,
        metric: str,
        value: float,
        header_timestamp: Optional[float] = None,
    ) -> Optional[Anomaly]:
        """Ingests a telemetry sample and checks for statistical anomalies."""
        self.active_nodes.add(node)
        self.last_seen_times[(node, topic)] = timestamp

        # Optional legacy path: header order check inline (monitor calls check_header_order directly)
        if header_timestamp is not None:
            is_dup, order_anom = self.check_header_order(timestamp, node, topic, header_timestamp)
            if is_dup:
                return None
            if order_anom is not None:
                return order_anom

        key = (node, metric)
        if key not in self.baselines:
            self.baselines[key] = MetricBaseline(
                epsilon_std=self.epsilon_std, min_abs_dev=self._min_abs_dev_for(metric)
            )
        baseline = self.baselines[key]

        # Startup noise rejection: warm-up phase
        in_warmup = (timestamp - self.start_time < self.warmup_duration_sec) or (not baseline.is_warmed_up)
        if in_warmup:
            baseline.update(value)
            return None

        # Calculate z-score
        mean = baseline.mean
        std = max(baseline.std, self.epsilon_std)
        abs_dev = abs(value - mean)
        z = abs_dev / std

        # Anomaly condition: requires both statistical z-score AND physical absolute deviation
        # to ensure normal OS thread/scheduling jitter is not falsely flagged.
        if z > self.z_threshold and abs_dev >= baseline.min_abs_dev:
            baseline.outlier_streak += 1
            if baseline.outlier_streak == 1:
                baseline.streak_start = timestamp
            if baseline.outlier_streak < self.confirm_samples:
                # Not yet confirmed: single outliers are not evidence.
                return None
            severity = min(1.0, z / (self.z_threshold * 3.0))
            return Anomaly(
                timestamp=timestamp,
                node=node,
                topic=topic,
                metric=metric,
                value=value,
                baseline_mean=mean,
                baseline_std=std,
                z_score=z,
                severity=severity,
                description=(
                    f'Value {value:.4f} deviated from baseline {mean:.4f}±{std:.4f} '
                    f'(z={z:.1f}, confirmed x{baseline.outlier_streak})'
                ),
                onset_time=baseline.streak_start,
            )

        # Normal observation: reset confirmation streak and update baseline
        baseline.outlier_streak = 0
        baseline.streak_start = None
        baseline.update(value)
        return None

    def timeout_threshold(self, node: str, topic: str) -> Tuple[float, float]:
        """Returns (nominal_dt, timeout_threshold) for a (node, topic) stream."""
        dt_key = (node, f'{topic}:inter_arrival')
        b = self.baselines.get(dt_key)
        nominal_dt = b.mean if (b is not None and b.is_warmed_up) else 0.1
        return nominal_dt, max(self.timeout_floor_sec, self.timeout_multiplier * nominal_dt)

    def check_timeouts(self, current_time: float) -> List[Anomaly]:
        """Checks for message starvation / dropouts across monitored topics."""
        # Only start checking timeouts after initial warmup
        if current_time - self.start_time < self.warmup_duration_sec:
            return []

        timeout_anomalies = []
        for (node, topic), last_time in self.last_seen_times.items():
            dt = current_time - last_time
            nominal_dt, timeout_thresh = self.timeout_threshold(node, topic)

            if dt > timeout_thresh:
                last_alert = self.last_timeout_alert.get((node, topic), 0.0)
                # Rate limit timeout alerts to once every timeout_thresh
                if current_time - last_alert >= timeout_thresh:
                    self.last_timeout_alert[(node, topic)] = current_time
                    severity = min(1.0, 0.5 + (dt / (timeout_thresh * 4.0)))
                    # Onset time: the moment the starvation threshold was crossed.
                    onset = last_time + timeout_thresh
                    timeout_anomalies.append(Anomaly(
                        timestamp=current_time,
                        onset_time=onset,
                        node=node,
                        topic=topic,
                        metric='message_timeout',
                        value=dt,
                        baseline_mean=nominal_dt,
                        baseline_std=nominal_dt * 0.2,
                        z_score=dt / max(1e-4, nominal_dt),
                        severity=severity,
                        description=(
                            f'Message timeout on {topic}: no message for {dt:.2f}s '
                            f'(expected ~{nominal_dt:.3f}s, threshold {timeout_thresh:.2f}s, '
                            f'last seen t={last_time:.2f})'
                        ),
                    ))

        return timeout_anomalies

    def check_node_crashes(self, current_active_nodes: Set[str], current_time: float) -> List[Anomaly]:
        """Detects node termination / crash from the ROS graph.

        A node that is still missing is re-reported every `crash_realert_sec`
        (like an ongoing timeout) so the evidence stays inside the RCA window.
        """
        if current_time - self.start_time < self.warmup_duration_sec:
            self.active_nodes = set(current_active_nodes)
            return []

        crash_anomalies = []
        vanished_nodes = self.active_nodes - current_active_nodes
        for node in vanished_nodes:
            # Evidence onset = last moment the node was observed alive (if known).
            seen = [t for (n, _), t in self.last_seen_times.items() if n == node]
            last_alive = max(seen) if seen else current_time
            self.missing_nodes[node] = (last_alive, -1e18)
            # Stop timeout checks for a node known to be dead (the crash explains its silence)
            for key in [k for k in self.last_seen_times if k[0] == node]:
                self.last_seen_times.pop(key, None)

        for node in list(self.missing_nodes):
            if node in current_active_nodes:
                self.missing_nodes.pop(node)          # node restarted
                continue
            last_alive, last_alert = self.missing_nodes[node]
            if current_time - last_alert >= self.crash_realert_sec:
                self.missing_nodes[node] = (last_alive, current_time)
                crash_anomalies.append(Anomaly(
                    timestamp=current_time,
                    onset_time=last_alive,
                    node=node,
                    topic='',
                    metric='node_crash',
                    value=current_time - last_alive,
                    baseline_mean=0.0,
                    baseline_std=0.01,
                    z_score=100.0,
                    severity=1.0,
                    description=(f'Node {node} absent from ROS 2 graph (process crash or killed); '
                                 f'last seen alive t={last_alive:.2f}, missing for {current_time - last_alive:.1f}s'),
                ))

        # Update active nodes
        self.active_nodes = set(current_active_nodes)
        return crash_anomalies
