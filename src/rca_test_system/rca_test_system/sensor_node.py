"""Sensor node for rca_test_system.

Publishes simulated laser scan data at a steady rate.
Supports fault injection: latency (real wall-clock sleep), dropout, degradation (NaNs/noise),
processing delay, and node crash.
"""

import json
import math
import os
import random
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('num_readings', 60)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('nominal_distance', 3.0)

        self.rate_hz = self.get_parameter('rate_hz').value
        self.num_readings = self.get_parameter('num_readings').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.nominal_dist = self.get_parameter('nominal_distance').value

        self.publisher_ = self.create_publisher(LaserScan, '/sensor/scan', 10)
        self.fault_sub_ = self.create_subscription(
            String, '/rca/fault_command', self._fault_callback, 10
        )

        # Fault state
        self.fault_active = False
        self.fault_type = None
        self.fault_param = 0.0
        self.fault_end_time = 0.0

        self.msg_queue = []
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self._publish_scan)
        self.seq = 0
        self.get_logger().info(f'SensorNode initialized at {self.rate_hz} Hz')

    def _fault_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            target = cmd.get('target_node', '')
            if target not in ('sensor_node', 'all'):
                return

            f_type = cmd.get('fault_type', 'clear')
            if f_type == 'clear':
                self.fault_active = False
                self.fault_type = None
                self.msg_queue.clear()
                self.get_logger().info('SensorNode: Fault cleared.')
                return

            self.fault_active = True
            self.fault_type = f_type
            self.fault_param = float(cmd.get('param', 0.0))
            duration = float(cmd.get('duration', 10.0))
            self.fault_end_time = time.time() + duration
            self.get_logger().warn(
                f'SensorNode: Injected fault "{self.fault_type}" '
                f'(param={self.fault_param}, duration={duration}s)'
            )

            # Handle instant crash
            if self.fault_type == 'crash':
                self.get_logger().fatal('SensorNode: CRASH fault triggered - exiting process now.')
                os._exit(1)

        except Exception as e:
            self.get_logger().error(f'Failed to parse fault command: {e}')

    def _publish_scan(self):
        now_time = time.time()

        # Auto-expire temporary fault
        if self.fault_active and self.fault_end_time > 0 and now_time > self.fault_end_time:
            self.get_logger().info('SensorNode: Fault duration expired. Reverting to nominal.')
            self.fault_active = False
            self.fault_type = None

        # Fault: Dropout
        if self.fault_active and self.fault_type == 'dropout':
            drop_prob = self.fault_param if self.fault_param > 0 else 1.0
            if random.random() < drop_prob:
                # Drop this measurement entirely
                return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / max(1, self.num_readings - 1)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate_hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Generate range data
        if self.fault_active and self.fault_type == 'degradation':
            # Degraded sensor readings: NaNs and extreme corrupted values
            ranges = []
            for i in range(self.num_readings):
                if random.random() < 0.6:
                    ranges.append(float('nan'))
                else:
                    ranges.append(self.nominal_dist + random.uniform(20.0, 50.0))
        else:
            # Nominal sensor readings with slight gaussian noise
            ranges = [
                self.nominal_dist + random.gauss(0.0, 0.02)
                for _ in range(self.num_readings)
            ]

        scan.ranges = ranges

        # Fault: Latency / Message Delay - real wall-clock publication delay
        if self.fault_active and self.fault_type in ('latency', 'processing_delay', 'message_delay'):
            delay_sec = self.fault_param if self.fault_param > 0 else 0.35
            publish_at = now_time + delay_sec
        else:
            publish_at = now_time

        self.msg_queue.append((publish_at, scan))

        # Drain queued messages that have reached their scheduled real publication time
        remaining = []
        for pub_time, s in self.msg_queue:
            if pub_time <= now_time:
                self.publisher_.publish(s)
                self.seq += 1
            else:
                remaining.append((pub_time, s))
        self.msg_queue = remaining


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
