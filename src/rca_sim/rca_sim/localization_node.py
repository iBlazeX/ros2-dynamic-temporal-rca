"""Localization: odometry + IMU dead reckoning corrected by perception landmarks.

Inputs   /sim/odom (Odometry), /sim/imu/data (Imu), /perception/obstacles (PoseArray)
Output   /localization/pose (PoseWithCovarianceStamped), one per odometry message

The estimate is the odometry pose; each non-empty obstacle set acts as a landmark
correction that resets the uncertainty. Without corrections (perception empty,
delayed or absent) the covariance grows physically with dead-reckoning time, so
an upstream perception problem shows up here as growing pose uncertainty and
eventually as a safety stop downstream.

Own faults: 'localization_failure' (pose jump + covariance spike),
'degradation' (inflated covariance), 'processing_delay'/'workload', link faults.
"""

import math
import time

from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from .sim_node_base import SimNode, spin_node


class LocalizationNode(SimNode):
    BASE_SIGMA = 0.05
    DRIFT_PER_SEC = 0.12          # uncertainty growth without landmark corrections (m/s)
    CORRECTION_TIMEOUT = 0.6      # s: obstacles older than this do not count as a correction

    def __init__(self):
        super().__init__('localization_node')
        self.pub = self.link(self.create_publisher(PoseWithCovarianceStamped, '/localization/pose', 10))
        self.create_subscription(Odometry, '/sim/odom', self._on_odom, 10)
        self.create_subscription(Imu, '/sim/imu/data', self._on_imu, 10)
        self.create_subscription(PoseArray, '/perception/obstacles', self._on_obstacles, 10)
        self.last_correction = time.time()
        self.imu_rate = 0.0
        self.imu_time = 0.0
        self.get_logger().info('localization_node ready (odom + imu + landmarks -> /localization/pose)')

    def _on_imu(self, msg: Imu):
        self.imu_rate = msg.angular_velocity.z
        self.imu_time = time.time()

    def _on_obstacles(self, msg: PoseArray):
        if msg.poses:
            self.last_correction = time.time()

    def _on_odom(self, odom: Odometry):
        self.workload()
        now = time.time()
        since_corr = now - self.last_correction
        sigma = self.BASE_SIGMA + (self.DRIFT_PER_SEC * max(0.0, since_corr - self.CORRECTION_TIMEOUT))
        if now - self.imu_time > 1.0:
            sigma += 0.1                     # no IMU: heading less certain

        out = PoseWithCovarianceStamped()
        out.header.stamp = odom.header.stamp   # retain source timestamp
        out.header.frame_id = 'map'
        out.pose.pose = odom.pose.pose

        if self.faults.is_active('localization_failure'):
            jump = self.faults.param('localization_failure', 20.0)
            out.pose.pose.position.x += jump
            out.pose.pose.position.y -= jump
            sigma = max(sigma, 5.0)
        elif self.faults.is_active('degradation'):
            sigma += self.faults.param('degradation', 0.4)

        cov = [0.0] * 36
        cov[0] = cov[7] = sigma * sigma
        cov[35] = (0.02 + 0.1 * sigma) ** 2
        out.pose.covariance = cov
        self.pub.publish(out)


def main(args=None):
    spin_node(LocalizationNode)


if __name__ == '__main__':
    main()
