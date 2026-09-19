"""Fault injector node and CLI utility for rca_test_system.

Allows injecting faults into pipeline nodes:
- latency (real wall-clock publication delay)
- dropout (message suppression)
- degradation (corrupted values, NaNs)
- localization_failure (drift/jump)
- navigation_failure (erratic commands)
- node crash (process termination)
- message delay / processing delay
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')
        self.cmd_pub = self.create_publisher(String, '/rca/fault_command', 10)
        self.get_logger().info('FaultInjector ready on /rca/fault_command')

    def wait_for_subscribers(self, min_count: int = 4, timeout_sec: float = 5.0) -> int:
        """Blocks until DDS discovery has matched the pipeline subscribers (or timeout).

        A short-lived CLI node that publishes immediately after start-up loses
        its messages to the discovery race; waiting for matching makes injection
        reliable.
        """
        deadline = time.time() + timeout_sec
        count = self.cmd_pub.get_subscription_count()
        while count < min_count and time.time() < deadline:
            time.sleep(0.1)
            count = self.cmd_pub.get_subscription_count()
        if count < min_count:
            self.get_logger().warn(
                f'Only {count} subscriber(s) matched on /rca/fault_command after {timeout_sec}s'
            )
        else:
            time.sleep(0.2)  # let the last match settle before publishing
        return count

    def inject(self, target_node: str, fault_type: str, param: float = 0.0, duration: float = 10.0):
        self.wait_for_subscribers()
        payload = {
            'target_node': target_node,
            'fault_type': fault_type,
            'param': param,
            'duration': duration,
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        # Publish multiple times to guarantee delivery over DDS
        for _ in range(3):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().warn(
            f'Injected fault: target={target_node}, type={fault_type}, param={param}, duration={duration}s'
        )

    def clear(self, target_node: str = 'all'):
        self.wait_for_subscribers()
        payload = {
            'target_node': target_node,
            'fault_type': 'clear',
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        for _ in range(3):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().info(f'Cleared faults for target={target_node}')


def main(args=None):
    parser = argparse.ArgumentParser(description='RCA Fault Injector')
    parser.add_argument(
        '--target', type=str, default='sensor_node',
        choices=['sensor_node', 'perception_node', 'localization_node', 'navigation_node', 'all'],
        help='Target node name'
    )
    parser.add_argument(
        '--type', type=str, default='latency',
        choices=[
            'latency', 'dropout', 'degradation', 'localization_failure',
            'navigation_failure', 'crash', 'processing_delay', 'message_delay', 'clear'
        ],
        help='Fault type'
    )
    parser.add_argument('--param', type=float, default=0.35, help='Fault parameter (delay sec, drop rate, etc.)')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration in seconds')

    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    injector = FaultInjector()

    # DDS discovery is awaited inside inject()/clear()

    if parsed.type == 'clear':
        injector.clear(parsed.target)
    else:
        injector.inject(parsed.target, parsed.type, parsed.param, parsed.duration)

    time.sleep(0.5)
    injector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
