"""Navigation node for rca_test_system.

Subscribes to /localization/pose and computes velocity commands.
Publishes /navigation/cmd_vel (geometry_msgs/msg/TwistStamped).
Triggers fail-safe emergency stop when localization is anomalous or stale.
Supports fault injection: navigation_failure, latency, processing delay, crash.
"""

import json
import math
import os
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.publisher_ = self.create_publisher(TwistStamped, '/navigation/cmd_vel', 10)
        self.pose_sub_ = self.create_subscription(
            PoseStamped, '/localization/pose', self._pose_callback, 10
        )
        self.fault_sub_ = self.create_subscription(
            String, '/rca/fault_command', self._fault_callback, 10
        )

        # Fault state
        self.fault_active = False
        self.fault_type = None
        self.fault_param = 0.0
        self.fault_end_time = 0.0

        self.last_pose_time = time.time()
        self.last_x = 0.0
        self.last_y = 0.0
        self.get_logger().info('NavigationNode initialized, listening to /localization/pose')

    def _fault_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            target = cmd.get('target_node', '')
            if target not in ('navigation_node', 'all'):
                return

            f_type = cmd.get('fault_type', 'clear')
            if f_type == 'clear':
                self.fault_active = False
                self.fault_type = None
                self.get_logger().info('NavigationNode: Fault cleared.')
                return

            self.fault_active = True
            self.fault_type = f_type
            self.fault_param = float(cmd.get('param', 0.0))
            duration = float(cmd.get('duration', 10.0))
            self.fault_end_time = time.time() + duration
            self.get_logger().warn(
                f'NavigationNode: Injected fault "{self.fault_type}" '
                f'(param={self.fault_param}, duration={duration}s)'
            )

            if self.fault_type == 'crash':
                self.get_logger().fatal('NavigationNode: CRASH fault triggered - exiting process now.')
                os._exit(1)

        except Exception as e:
            self.get_logger().error(f'Failed to parse fault command: {e}')

    def _pose_callback(self, pose_msg: PoseStamped):
        now_time = time.time()
        dt = now_time - self.last_pose_time
        self.last_pose_time = now_time

        # Auto-expire fault
        if self.fault_active and self.fault_end_time > 0 and now_time > self.fault_end_time:
            self.get_logger().info('NavigationNode: Fault expired.')
            self.fault_active = False
            self.fault_type = None

        # Fault: Processing delay / latency
        if self.fault_active and self.fault_type in ('latency', 'processing_delay'):
            delay = self.fault_param if self.fault_param > 0 else 0.3
            time.sleep(delay)

        # Fault: Dropout
        if self.fault_active and self.fault_type == 'dropout':
            return

        cmd = TwistStamped()
        # Retain upstream acquisition timestamp so pipeline delay propagates physically
        cmd.header.stamp = pose_msg.header.stamp
        cmd.header.frame_id = 'base_link'

        # Check for direct navigation failure
        if self.fault_active and self.fault_type == 'navigation_failure':
            # Erratic / unsafe velocity commands
            cmd.twist.linear.x = 9.99
            cmd.twist.angular.z = 4.5
            self.get_logger().warn('NavigationNode: Emitting erratic velocity due to navigation_failure!')
            self.publisher_.publish(cmd)
            return

        # Check input pose validity (cascading failure detection)
        cur_x = pose_msg.pose.position.x
        cur_y = pose_msg.pose.position.y
        cur_z = pose_msg.pose.position.z
        qw = pose_msg.pose.orientation.w

        pos_jump = math.sqrt((cur_x - self.last_x) ** 2 + (cur_y - self.last_y) ** 2)
        self.last_x = cur_x
        self.last_y = cur_y

        is_pose_anomalous = (
            cur_z > 1.0 or abs(qw) < 0.01 or pos_jump > 5.0 or math.isnan(cur_x)
        )

        if is_pose_anomalous:
            # Cascading failure: emergency stop
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.get_logger().warn(
                f'NavigationNode: Detected invalid pose (jump={pos_jump:.2f}m, z={cur_z:.2f})! Emergency stop triggered.'
            )
        else:
            # Nominal navigation velocity: steady cruising
            cmd.twist.linear.x = 0.5
            cmd.twist.angular.z = 0.02

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
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
