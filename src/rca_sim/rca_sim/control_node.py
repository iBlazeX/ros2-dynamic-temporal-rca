"""Control: path following -> /cmd_vel (TwistStamped) at 20 Hz.

Input    /planning/path (nav_msgs/Path)
Output   /cmd_vel (TwistStamped) consumed by sim_world

An empty or stale path (> 1 s) yields a zero command (safety stop) - the last
hop of a cascade started anywhere upstream. Own faults: 'control_failure' /
'navigation_failure' (erratic commands), 'processing_delay', 'workload', link faults.
"""

import math
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path

from .sim_node_base import SimNode, spin_node
from .world import wrap_angle


class ControlNode(SimNode):
    V_CRUISE = 0.5
    K_HEADING = 1.2
    PATH_STALE = 1.0

    def __init__(self):
        super().__init__('control_node')
        self.pub = self.link(self.create_publisher(TwistStamped, '/cmd_vel', 10))
        self.create_subscription(Path, '/planning/path', self._on_path, 10)
        self.path = None
        self.path_time = 0.0
        self.phase = 0.0
        self.create_timer(0.05, self._control)
        self.get_logger().info('control_node ready (path -> /cmd_vel)')

    def _on_path(self, msg: Path):
        self.path = msg
        self.path_time = time.time()

    def _control(self):
        self.workload()
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_link'
        cmd.header.stamp = self.path.header.stamp if self.path is not None else self.stamp()

        erratic = self.faults.is_active('control_failure') or self.faults.is_active('navigation_failure')
        if erratic:
            self.phase += 0.6
            cmd.twist.linear.x = 3.0 * math.sin(self.phase)      # far outside the nominal band
            cmd.twist.angular.z = 2.5 * math.cos(self.phase)
            self.pub.publish(cmd)
            return

        if self.path is None or not self.path.poses or (time.time() - self.path_time) > self.PATH_STALE:
            self.pub.publish(cmd)                                # zero command: safety stop
            return

        p0, p1 = self.path.poses[0].pose, self.path.poses[min(2, len(self.path.poses) - 1)].pose
        desired = math.atan2(p1.position.y - p0.position.y, p1.position.x - p0.position.x)
        current = 2.0 * math.atan2(p0.orientation.z, p0.orientation.w)
        err = wrap_angle(desired - current)
        cmd.twist.angular.z = max(-1.2, min(1.2, self.K_HEADING * err))
        cmd.twist.linear.x = self.V_CRUISE * max(0.2, 1.0 - abs(err) / math.pi)
        self.pub.publish(cmd)


def main(args=None):
    spin_node(ControlNode)


if __name__ == '__main__':
    main()
