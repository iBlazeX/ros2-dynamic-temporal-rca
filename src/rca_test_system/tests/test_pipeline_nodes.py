"""Unit tests for rca_test_system nodes and fault commands."""

import json
import pytest
import rclpy
from rclpy.node import Node
from rca_test_system.fault_injector import FaultInjector
from rca_test_system.sensor_node import SensorNode
from rca_test_system.perception_node import PerceptionNode
from rca_test_system.localization_node import LocalizationNode
from rca_test_system.navigation_node import NavigationNode
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from std_msgs.msg import String


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_sensor_node_initialization_and_fault(ros_context):
    node = SensorNode()
    assert node.rate_hz == 10.0
    assert not node.fault_active

    # Test fault command parsing
    cmd = String()
    cmd.data = json.dumps({'target_node': 'sensor_node', 'fault_type': 'latency', 'param': 0.35, 'duration': 5.0})
    node._fault_callback(cmd)
    assert node.fault_active is True
    assert node.fault_type == 'latency'
    assert node.fault_param == 0.35

    # Clear fault
    cmd_clear = String()
    cmd_clear.data = json.dumps({'target_node': 'sensor_node', 'fault_type': 'clear'})
    node._fault_callback(cmd_clear)
    assert node.fault_active is False
    node.destroy_node()


def test_perception_node_handling(ros_context):
    node = PerceptionNode()
    assert not node.fault_active

    # Simulate empty scan
    scan = LaserScan()
    scan.ranges = []
    node._scan_callback(scan)
    node.destroy_node()


def test_localization_node_handling(ros_context):
    node = LocalizationNode()
    assert node.x == 0.0

    # Simulate obstacle callback
    obstacles = PoseArray()
    node._obstacles_callback(obstacles)
    assert node.consecutive_empty_inputs == 1
    node.destroy_node()


def test_navigation_node_handling(ros_context):
    node = NavigationNode()
    pose = PoseStamped()
    pose.pose.position.x = 1.0
    pose.pose.orientation.w = 1.0
    node._pose_callback(pose)
    assert node.last_x == 1.0
    node.destroy_node()
