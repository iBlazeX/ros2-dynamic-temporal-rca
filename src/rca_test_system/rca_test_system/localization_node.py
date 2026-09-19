"""Localization node for rca_test_system.

Subscribes to /perception/obstacles and estimates robot pose.
Publishes /localization/pose (geometry_msgs/msg/PoseStamped).
Cascades failures when perception inputs are delayed or empty.
Supports fault injection: localization_failure (drift/jump), latency, processing delay, crash.
"""

import json
import os
import time

from geometry_msgs.msg import PoseArray, PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')

        self.publisher_ = self.create_publisher(PoseStamped, '/localization/pose', 10)
        self.obstacles_sub_ = self.create_subscription(
            PoseArray, '/perception/obstacles', self._obstacles_callback, 10
        )
        self.fault_sub_ = self.create_subscription(
            String, '/rca/fault_command', self._fault_callback, 10
        )

        # Fault state
        self.fault_active = False
        self.fault_type = None
        self.fault_param = 0.0
        self.fault_end_time = 0.0

        # Robot simulated state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update_time = time.time()
        self.consecutive_empty_inputs = 0

        self.get_logger().info('LocalizationNode initialized, listening to /perception/obstacles')

    def _fault_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            target = cmd.get('target_node', '')
            if target not in ('localization_node', 'all'):
                return

            f_type = cmd.get('fault_type', 'clear')
            if f_type == 'clear':
                self.fault_active = False
                self.fault_type = None
                self.get_logger().info('LocalizationNode: Fault cleared.')
                return

            self.fault_active = True
            self.fault_type = f_type
            self.fault_param = float(cmd.get('param', 0.0))
            duration = float(cmd.get('duration', 10.0))
            self.fault_end_time = time.time() + duration
            self.get_logger().warn(
                f'LocalizationNode: Injected fault "{self.fault_type}" '
                f'(param={self.fault_param}, duration={duration}s)'
            )

            if self.fault_type == 'crash':
                self.get_logger().fatal('LocalizationNode: CRASH fault triggered - exiting process now.')
                os._exit(1)

        except Exception as e:
            self.get_logger().error(f'Failed to parse fault command: {e}')

    def _obstacles_callback(self, obstacles_msg: PoseArray):
        now_time = time.time()
        dt = now_time - self.last_update_time
        self.last_update_time = now_time

        # Auto-expire fault
        if self.fault_active and self.fault_end_time > 0 and now_time > self.fault_end_time:
            self.get_logger().info('LocalizationNode: Fault expired.')
            self.fault_active = False
            self.fault_type = None

        # Fault: Processing delay / latency
        if self.fault_active and self.fault_type in ('latency', 'processing_delay'):
            delay = self.fault_param if self.fault_param > 0 else 0.35
            time.sleep(delay)

        # Fault: Dropout
        if self.fault_active and self.fault_type == 'dropout':
            return

        # Check for upstream perception degradation
        if len(obstacles_msg.poses) == 0:
            self.consecutive_empty_inputs += 1
        else:
            self.consecutive_empty_inputs = 0

        # Normal forward motion simulation
        if self.consecutive_empty_inputs < 3:
            self.x += 0.05
            self.y += 0.01

        # Check for direct localization failure fault
        has_direct_loc_fault = self.fault_active and self.fault_type in (
            'localization_failure', 'degradation'
        )

        pose_msg = PoseStamped()
        # Retain upstream acquisition timestamp so pipeline delay propagates physically
        pose_msg.header.stamp = obstacles_msg.header.stamp
        pose_msg.header.frame_id = 'map'

        if has_direct_loc_fault:
            # Massive sudden position drift / jump
            drift = self.fault_param if self.fault_param > 0 else 50.0
            pose_msg.pose.position.x = self.x + drift
            pose_msg.pose.position.y = self.y + drift
            pose_msg.pose.position.z = 99.9  # Corrupt z value
            pose_msg.pose.orientation.w = 0.0  # Invalid quaternion
            self.get_logger().warn(
                f'LocalizationNode: Emitting FAULT pose with jump {drift}m'
            )
        elif self.consecutive_empty_inputs >= 3:
            # Cascaded failure from upstream: pose freezes or becomes stale
            pose_msg.pose.position.x = self.x
            pose_msg.pose.position.y = self.y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.get_logger().warn(
                'LocalizationNode: Upstream perception missing! Pose updates frozen.'
            )
        else:
            pose_msg.pose.position.x = self.x
            pose_msg.pose.position.y = self.y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0

        self.publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
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
