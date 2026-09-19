"""Perception node for rca_test_system.

Subscribes to /sensor/scan, detects obstacle poses, and publishes /perception/obstacles.
Cascades degradation when input scan is degraded or missing.
Supports direct fault injection: processing delay, dropout, degradation, and crash.
"""

import json
import math
import os
import random
import time

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.publisher_ = self.create_publisher(PoseArray, '/perception/obstacles', 10)
        self.scan_sub_ = self.create_subscription(
            LaserScan, '/sensor/scan', self._scan_callback, 10
        )
        self.fault_sub_ = self.create_subscription(
            String, '/rca/fault_command', self._fault_callback, 10
        )

        # Fault state
        self.fault_active = False
        self.fault_type = None
        self.fault_param = 0.0
        self.fault_end_time = 0.0

        self.last_scan_time = 0.0
        self.get_logger().info('PerceptionNode initialized, listening to /sensor/scan')

    def _fault_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            target = cmd.get('target_node', '')
            if target not in ('perception_node', 'all'):
                return

            f_type = cmd.get('fault_type', 'clear')
            if f_type == 'clear':
                self.fault_active = False
                self.fault_type = None
                self.get_logger().info('PerceptionNode: Fault cleared.')
                return

            self.fault_active = True
            self.fault_type = f_type
            self.fault_param = float(cmd.get('param', 0.0))
            duration = float(cmd.get('duration', 10.0))
            self.fault_end_time = time.time() + duration
            self.get_logger().warn(
                f'PerceptionNode: Injected fault "{self.fault_type}" '
                f'(param={self.fault_param}, duration={duration}s)'
            )

            if self.fault_type == 'crash':
                self.get_logger().fatal('PerceptionNode: CRASH fault triggered - exiting process now.')
                os._exit(1)

        except Exception as e:
            self.get_logger().error(f'Failed to parse fault command: {e}')

    def _scan_callback(self, scan_msg: LaserScan):
        now_time = time.time()
        self.last_scan_time = now_time

        # Auto-expire fault
        if self.fault_active and self.fault_end_time > 0 and now_time > self.fault_end_time:
            self.get_logger().info('PerceptionNode: Fault expired.')
            self.fault_active = False
            self.fault_type = None

        # Fault: Direct processing delay
        if self.fault_active and self.fault_type in ('latency', 'processing_delay'):
            delay = self.fault_param if self.fault_param > 0 else 0.3
            time.sleep(delay)

        # Fault: Dropout
        if self.fault_active and self.fault_type == 'dropout':
            drop_prob = self.fault_param if self.fault_param > 0 else 1.0
            if random.random() < drop_prob:
                return

        # Check input scan quality (cascading failure detection)
        valid_ranges = [
            r for r in scan_msg.ranges
            if not math.isnan(r) and not math.isinf(r) and scan_msg.range_min <= r <= scan_msg.range_max
        ]
        nan_count = len(scan_msg.ranges) - len(valid_ranges)

        obstacles = PoseArray()
        # Retain the source acquisition timestamp (standard ROS practice) so that
        # end-to-end pipeline delay is a real, measurable quantity downstream.
        obstacles.header.stamp = scan_msg.header.stamp
        obstacles.header.frame_id = 'map'

        # If scan is degraded or node has direct degradation fault
        is_degraded = (
            (self.fault_active and self.fault_type == 'degradation') or
            (nan_count > len(scan_msg.ranges) * 0.3)
        )

        if is_degraded:
            # Degraded perception output: empty or corrupted obstacle set
            self.get_logger().warn(
                f'PerceptionNode: Input scan degraded (NaNs={nan_count}/{len(scan_msg.ranges)}). Outputting degraded obstacles.'
            )
            # Empty obstacle set indicates perception failure
            self.publisher_.publish(obstacles)
            return

        # Nominal obstacle extraction: cluster valid ranges into obstacle points
        if valid_ranges:
            median_dist = sorted(valid_ranges)[len(valid_ranges) // 2]
            p1 = Pose()
            p1.position.x = median_dist
            p1.position.y = 0.5
            p1.position.z = 0.0
            p1.orientation.w = 1.0

            p2 = Pose()
            p2.position.x = median_dist + 1.0
            p2.position.y = -0.5
            p2.position.z = 0.0
            p2.orientation.w = 1.0

            obstacles.poses = [p1, p2]

        self.publisher_.publish(obstacles)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
