"""Perception: LiDAR clustering fused with camera confirmation -> /perception/obstacles.

Inputs   /sim/lidar/scan (LaserScan), /sim/camera/image (Image)
Output   /perception/obstacles (PoseArray, robot frame), one message per scan

Real symptoms it produces when degraded upstream or itself:
  - noisy / invalid scans (nan ratio)        -> empty obstacle set (cannot cluster)
  - camera confirmation older than 0.5 s     -> empty obstacle set (multi-modal gating)
  - 'degradation' fault                      -> empty or truncated obstacle set
  - 'workload' / 'processing_delay' faults   -> real CPU work per callback: lower output
                                                rate and higher end-to-end latency
"""

import math
import time

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image, LaserScan

from .sim_node_base import SimNode, spin_node


class PerceptionNode(SimNode):
    CLUSTER_JUMP = 0.5
    MAX_OBSTACLE_RANGE = 12.0   # walls of the 20 m arena are always within reach
    CAMERA_CONFIRM_SEC = 0.5    # obstacles are only reported while camera confirmation is fresh

    def __init__(self):
        super().__init__('perception_node')
        self.pub = self.link(self.create_publisher(PoseArray, '/perception/obstacles', 10))
        self.create_subscription(LaserScan, '/sim/lidar/scan', self._on_scan, 10)
        self.create_subscription(Image, '/sim/camera/image', self._on_image, 10)
        self.camera_bars = 0
        self.camera_time = 0.0
        self.get_logger().info('perception_node ready (lidar + camera -> /perception/obstacles)')

    def _on_image(self, img: Image):
        self.workload()
        # count bright columns in the middle band = camera detections
        row = (img.height // 2) * img.width
        bright = [c for c in range(img.width) if img.data[row + c] > 100]
        bars, prev = 0, -5
        for c in bright:
            if c - prev > 1:
                bars += 1
            prev = c
        self.camera_bars = bars
        self.camera_time = time.time()

    def _on_scan(self, scan: LaserScan):
        self.workload()
        out = PoseArray()
        out.header.stamp = scan.header.stamp        # retain acquisition time (real pipeline latency)
        out.header.frame_id = 'base_link'

        n = len(scan.ranges)
        invalid = sum(1 for r in scan.ranges if math.isnan(r) or math.isinf(r))
        if n == 0 or invalid > 0.3 * n:
            self.pub.publish(out)                  # cannot cluster a corrupted scan
            return
        if (time.time() - self.camera_time) > self.CAMERA_CONFIRM_SEC:
            # multi-modal gating: without fresh camera confirmation no obstacle is reported.
            # A lost low-rate camera therefore shows up downstream (empty obstacle set)
            # before the camera's own starvation timeout can be confirmed.
            self.pub.publish(out)
            return

        clusters, cur = [], []
        prev = None
        for i, r in enumerate(scan.ranges):
            valid = not (math.isnan(r) or math.isinf(r)) and r < self.MAX_OBSTACLE_RANGE
            if valid and (prev is None or abs(r - prev) < self.CLUSTER_JUMP):
                cur.append((i, r))
            else:
                if len(cur) >= 3:
                    clusters.append(cur)
                cur = [(i, r)] if valid else []
            prev = r if valid else None
        if len(cur) >= 3:
            clusters.append(cur)

        if self.faults.is_active('degradation'):
            keep = self.faults.param('degradation', 0.0)      # 0 = drop all, 0.5 = keep half
            clusters = clusters[:int(len(clusters) * keep)]

        for cl in clusters:
            i_mid, r_mid = cl[len(cl) // 2]
            ang = scan.angle_min + i_mid * scan.angle_increment
            p = Pose()
            p.position.x = r_mid * math.cos(ang)
            p.position.y = r_mid * math.sin(ang)
            p.orientation.w = 1.0
            out.poses.append(p)
        out.header.frame_id = 'base_link/fused'     # geometry from LiDAR, confirmed by camera
        self.pub.publish(out)


def main(args=None):
    spin_node(PerceptionNode)


if __name__ == '__main__':
    main()
