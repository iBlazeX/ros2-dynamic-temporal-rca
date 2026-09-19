"""Unit tests for AnomalyDetector edge-cases."""

import pytest
import time
from diagnostic_monitor.anomaly_detector import AnomalyDetector, MetricBaseline


def test_zero_std_dev_guard():
    baseline = MetricBaseline(warmup_samples=5, epsilon_std=1e-4)
    # Feed identical values
    for _ in range(10):
        baseline.update(5.0)

    assert baseline.mean == 5.0
    assert baseline.std >= 1e-4  # Guard prevents std from being 0.0


def test_startup_noise_rejection():
    # Warmup duration 2.0s
    detector = AnomalyDetector(warmup_duration_sec=2.0, z_threshold=3.0)
    now = time.time()

    # In warm-up: extreme values should not trigger alerts
    anom1 = detector.process_metric(now, 'node1', '/topic1', 'latency', 100.0)
    assert anom1 is None


def test_z_score_detection_after_warmup():
    detector = AnomalyDetector(warmup_duration_sec=0.1, z_threshold=3.0)
    # Simulate past start time
    detector.start_time = time.time() - 10.0

    # Train baseline with nominal values around 1.0 (std ~ 0.1)
    base_time = time.time()
    for i in range(25):
        detector.process_metric(base_time + i * 0.1, 'sensor_node', '/sensor/scan', 'latency', 1.0 + (0.05 if i % 2 == 0 else -0.05))

    # Test nominal value: no anomaly
    nom_anom = detector.process_metric(base_time + 5.0, 'sensor_node', '/sensor/scan', 'latency', 1.02)
    assert nom_anom is None

    # Test significant anomaly (latency = 3.5s, z >> 3): first sample only arms the
    # confirmation counter, the second consecutive sample confirms it
    assert detector.process_metric(base_time + 5.1, 'sensor_node', '/sensor/scan', 'latency', 3.5) is None
    fault_anom = detector.process_metric(base_time + 5.2, 'sensor_node', '/sensor/scan', 'latency', 3.5)
    assert fault_anom is not None
    assert fault_anom.node == 'sensor_node'
    assert fault_anom.z_score > 3.0
    assert fault_anom.severity > 0.5


def test_missing_data_timeout():
    detector = AnomalyDetector(warmup_duration_sec=0.1, timeout_multiplier=3.0)
    detector.start_time = time.time() - 10.0

    base_time = time.time()
    # Register topic activity with nominal 0.1s rate
    for i in range(20):
        detector.process_metric(base_time + i * 0.1, 'sensor_node', '/sensor/scan', '/sensor/scan:inter_arrival', 0.1)

    # Fast forward time by 2.0s without messages
    timeouts = detector.check_timeouts(base_time + 4.0)
    assert len(timeouts) == 1
    assert timeouts[0].node == 'sensor_node'
    assert timeouts[0].metric == 'message_timeout'
    assert timeouts[0].severity >= 0.5


def test_node_crash_detection():
    detector = AnomalyDetector(warmup_duration_sec=0.1)
    detector.start_time = time.time() - 10.0
    now = time.time()

    # Register active nodes
    detector.process_metric(now, 'sensor_node', '/sensor/scan', 'val', 1.0)
    detector.process_metric(now, 'perception_node', '/perception/obstacles', 'val', 1.0)

    # Perception node disappears from active set
    crashes = detector.check_node_crashes({'sensor_node'}, now + 1.0)
    assert len(crashes) == 1
    assert crashes[0].node == 'perception_node'
    assert crashes[0].metric == 'node_crash'
    assert crashes[0].severity == 1.0
