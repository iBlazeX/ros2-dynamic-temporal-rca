"""Planning: waypoint following with obstacle avoidance -> /planning/path.

Inputs   /localization/pose (PoseWithCovarianceStamped), /perception/obstacles (PoseArray)
Output   /planning/path (nav_msgs/Path) at 5 Hz

Safety behaviour (real cascade): if the pose is stale (> 1 s) or too uncertain
(sigma > 1.0 m) the planner publishes an empty path, which makes the controller
stop the robot.

Own faults: 'navigation_failure' (garbage goals / empty paths), 'processing_delay',
'workload', link faults, and 'topic_change': the pose source is switched at
runtime from /localization/pose to /sim/odom (raw odometry) and back, which
changes the ROS graph edges while the system keeps running.
"""

import math
import time

from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path

from .sim_node_base import SimNode, spin_node
from .world import waypoint_loop, wrap_angle


class PlanningNode(SimNode):
    WAYPOINT_TOL = 0.8
    MAX_SIGMA = 1.0
    POSE_STALE = 1.0

    def __init__(self):
        super().__init__('planning_node')
        self.pub = self.link(self.create_publisher(Path, '/planning/path', 10))
        self.create_subscription(PoseArray, '/perception/obstacles', self._on_obstacles, 10)
        self.pose_sub = None
        self._use_pose_source('localization')
        self.waypoints = waypoint_loop()
        self.wp_index = 0
        self.x = self.y = self.th = 0.0
        self.sigma = 0.0
        self.pose_time = 0.0
        self.pose_stamp = None
        self.obstacles = []
        self.pos_history = []            # (t, x, y) for stuck detection
        self.recover_until = 0.0
        self.recover_heading = 0.0
        self.create_timer(0.2, self._plan)
        self.get_logger().info('planning_node ready (pose + obstacles -> /planning/path)')

    # ------------------------------------------------------- pose sources

    def _use_pose_source(self, source: str):
        if self.pose_sub is not None:
            self.destroy_subscription(self.pose_sub)
        if source == 'odom':
            self.pose_sub = self.create_subscription(Odometry, '/sim/odom', self._on_odom_pose, 10)
        else:
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/localization/pose',
                                                     self._on_pose, 10)
        self.pose_source = source
        self.get_logger().warn(f'planning_node: pose source = {source}')

    def on_fault(self, f_type, cmd):
        if f_type == 'topic_change':
            self._use_pose_source('odom')

    def on_fault_expired(self, f_type):
        if f_type == 'topic_change':
            self._use_pose_source('localization')

    def on_fault_cleared(self):
        if self.pose_source != 'localization':
            self._use_pose_source('localization')

    # ---------------------------------------------------------- callbacks

    def _set_pose(self, pose, stamp, sigma):
        self.x, self.y = pose.position.x, pose.position.y
        self.th = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        self.sigma = sigma
        self.pose_time = time.time()
        self.pose_stamp = stamp

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        self._set_pose(msg.pose.pose, msg.header.stamp, math.sqrt(max(0.0, msg.pose.covariance[0])))

    def _on_odom_pose(self, msg: Odometry):
        self._set_pose(msg.pose.pose, msg.header.stamp, 0.3)     # raw odometry: fixed, larger uncertainty

    def _on_obstacles(self, msg: PoseArray):
        self.obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    # ------------------------------------------------------------- planning

    def _plan(self):
        self.workload()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.pose_stamp if self.pose_stamp is not None else self.stamp()

        stale = (time.time() - self.pose_time) > self.POSE_STALE
        if stale or self.sigma > self.MAX_SIGMA or self.pose_stamp is None:
            self.pub.publish(path)              # safety: no trustworthy pose -> empty path -> stop
            return

        gx, gy = self.waypoints[self.wp_index]
        if math.hypot(gx - self.x, gy - self.y) < self.WAYPOINT_TOL:
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            gx, gy = self.waypoints[self.wp_index]

        if self.faults.is_active('navigation_failure'):
            mode = self.faults.param('navigation_failure', 1.0)
            if mode >= 2.0:
                self.pub.publish(path)          # planner produces nothing
                return
            gx, gy = self.rng.uniform(-9, 9), self.rng.uniform(-9, 9)   # garbage goal every cycle

        # simple reactive avoidance: steer away from the nearest obstacle ahead
        heading = math.atan2(gy - self.y, gx - self.x)
        now = time.time()
        if now < self.recover_until:
            heading = self.recover_heading                # stuck recovery: turn away and drive
        else:
            for ox, oy in sorted(self.obstacles, key=lambda p: math.hypot(p[0], p[1])):
                d = math.hypot(ox, oy)
                bearing = math.atan2(oy, ox)               # robot frame
                if d < 2.2 and abs(wrap_angle(bearing - wrap_angle(heading - self.th))) < 0.8:
                    heading = wrap_angle(heading + (1.1 if bearing < 0 else -1.1))
                    break
            # stuck detection: no displacement for 2.5 s while a goal exists -> recovery turn
            self.pos_history.append((now, self.x, self.y))
            self.pos_history = [h for h in self.pos_history if now - h[0] <= 2.5]
            if len(self.pos_history) > 8 and math.hypot(self.x - self.pos_history[0][1],
                                                        self.y - self.pos_history[0][2]) < 0.05:
                self.recover_heading = wrap_angle(self.th + (2.2 if self.rng.random() < 0.5 else -2.2))
                self.recover_until = now + 2.5
                self.pos_history = []
                heading = self.recover_heading

        n = 5
        for k in range(n + 1):
            ps = PoseStamped()
            ps.header = path.header
            step = min(1.5, math.hypot(gx - self.x, gy - self.y)) * k / n
            ps.pose.position.x = self.x + step * math.cos(heading)
            ps.pose.position.y = self.y + step * math.sin(heading)
            # pose 0 carries the robot's current heading, the rest the path heading
            yaw = self.th if k == 0 else heading
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)
        self.pub.publish(path)


def main(args=None):
    spin_node(PlanningNode)


if __name__ == '__main__':
    main()
