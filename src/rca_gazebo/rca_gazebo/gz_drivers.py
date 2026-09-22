"""ROS 2 sensor driver nodes for the Gazebo Harmonic backend.

Layering
--------
    Gazebo Harmonic (gz sim)        physics + gpu_lidar / camera / imu / diff-drive odometry
        |  gz transport
    ros_gz_bridge (parameter_bridge)   bridges every simulator topic into the "/gz/..." namespace
        |  ROS 2
    this module: lidar_node / camera_node / imu_node / odometry_node
        |  ROS 2 (canonical application topics, identical to rca_sim)
    perception_node -> localization_node -> planning_node -> control_node   (reused from rca_sim)
        |
    sim_world (actuation sink) -> /gz/cmd_vel -> bridge -> Gazebo DiffDrive

Why the drivers exist
---------------------
1. They are the point where *real* runtime degradation is injected (dropout,
   stale repeats, noise, bias, rate reduction, latency, jitter, loss, crash,
   stop, restart), reusing rca_sim's degradation primitives unchanged. Nothing
   writes a fault label anywhere the diagnostic monitor can see.
2. They give the sensor layer genuine, separate ROS nodes, so the dependency
   graph the monitor discovers has one node per sensor exactly as a real robot
   stack would. The graph is still discovered at runtime, never declared.
3. They re-stamp each message with the ROS wall clock on receipt from the
   simulator. Gazebo stamps its sensor messages with *simulation* time, which is
   a different time base from the monitor's wall clock; measuring "latency" as
   wall_now - sim_stamp would be meaningless. The stamp applied here is the
   instant the measurement entered the ROS system, and every downstream node
   propagates it, so the latency the monitor measures is the real ROS-side
   pipeline delay. The un-stamped Gazebo->bridge hop is therefore NOT included
   in the measured latency; this is stated in the README.

The raw "/gz/..." topics are excluded from monitoring through the monitor's
`excluded_topic_prefixes` launch parameter (a deployment setting, not topology).
"""

import math

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan

from rca_sim.sim_node_base import SimNode, spin_node
from rca_sim.world import gauss

RAW_NS = '/gz'


class GzSensorDriver(SimNode):
    """Common behaviour: take a bridged simulator message, optionally degrade it
    for real, re-stamp it with the ROS wall clock, publish it downstream.

    Faults handled here (all produce real runtime symptoms):
      dropout  probability of not publishing a received measurement
      stale    re-publish the previous measurement (old stamp, repeated values)
      noise    measurement noise scale; heavy noise also yields invalid returns
      bias     constant measurement offset
    Faults handled by the shared DegradableLink: latency, jitter, loss, rate.
    Faults handled by SimNode: workload, cpu_pressure, crash, stop, restart.
    """

    def __init__(self, name: str, raw_topic: str, out_topic: str, msg_type, seed: int = 7):
        super().__init__(name, seed=seed)
        self.out = self.link(self.create_publisher(msg_type, out_topic, 10))
        self.create_subscription(msg_type, raw_topic, self._on_raw, 10)
        self.last_msg = None
        self.bias = 0.0
        self.received = 0
        self.published = 0
        self.get_logger().info(f'{name} ready ({raw_topic} -> {out_topic})')

    def _on_raw(self, msg):
        self.received += 1
        self.workload()
        f = self.faults
        if f.is_active('dropout') and self.rng.random() < min(1.0, f.param('dropout', 1.0)):
            return
        if f.is_active('stale') and self.last_msg is not None:
            self.out.publish(self.last_msg)      # repeated stamp + repeated values
            return
        self.bias = f.param('bias', 0.5) if f.is_active('bias') else 0.0
        sigma_scale = f.param('noise', 8.0) if f.is_active('noise') else 1.0
        out = self.convert(msg, sigma_scale)
        if out is None:
            return
        out.header.stamp = self.stamp()          # ROS wall clock: entry into the ROS system
        self.last_msg = out
        self.published += 1
        self.out.publish(out)

    def convert(self, msg, sigma_scale: float):
        raise NotImplementedError


class GzLidarDriver(GzSensorDriver):
    """gz gpu_lidar -> /sim/lidar/scan (sensor_msgs/LaserScan, ~10 Hz)."""

    SIGMA = 0.02
    MAX_RANGE = 15.0

    def __init__(self, seed: int = 7):
        super().__init__('lidar_node', f'{RAW_NS}/scan', '/sim/lidar/scan', LaserScan, seed)

    def convert(self, msg: LaserScan, sigma_scale: float):
        out = LaserScan()
        out.header.frame_id = 'lidar'
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min = max(0.1, float(msg.range_min))
        out.range_max = min(self.MAX_RANGE, float(msg.range_max) or self.MAX_RANGE)
        sigma = self.SIGMA * sigma_scale
        ranges = []
        for r in msg.ranges:
            v = float(r)
            if math.isinf(v) or math.isnan(v) or v > out.range_max:
                # No return within range. rca_sim reports max_range here (an
                # unobstructed ray), not NaN/Inf, and perception treats NaN/Inf
                # as an *invalid* return. Same convention in both backends, so an
                # invalid-return ratio means the same physical thing.
                v = out.range_max
            else:
                v += gauss(self.rng, sigma) + self.bias
            # Heavy noise corrupts any beam, obstructed or not, exactly as the
            # rca_sim LiDAR model does, so the invalid-return ratio the monitor
            # measures means the same physical thing in both backends.
            if sigma_scale > 3.0 and self.rng.random() < min(0.7, 0.04 * sigma_scale):
                v = float('nan')
            else:
                v = max(out.range_min, min(out.range_max, v))
            ranges.append(v)
        out.ranges = ranges
        out.intensities = []
        return out


class GzCameraDriver(GzSensorDriver):
    """gz camera (rgb8) -> /sim/camera/image (sensor_msgs/Image, mono8, ~5 Hz).

    Converted to mono8 so the reused rca_sim perception node sees exactly the
    format it expects. Perception only uses the camera for freshness gating, but
    the conversion is a real greyscale conversion, not a placeholder.
    """

    def __init__(self, seed: int = 7):
        super().__init__('camera_node', f'{RAW_NS}/image', '/sim/camera/image', Image, seed)

    def convert(self, msg: Image, sigma_scale: float):
        out = Image()
        out.header.frame_id = 'camera'
        out.height, out.width = msg.height, msg.width
        out.encoding, out.step, out.is_bigendian = 'mono8', msg.width, 0
        data = msg.data
        enc = (msg.encoding or 'rgb8').lower()
        px = bytearray(msg.width * msg.height)
        if enc in ('rgb8', 'bgr8'):
            for i in range(len(px)):
                j = 3 * i
                px[i] = (data[j] * 77 + data[j + 1] * 150 + data[j + 2] * 29) >> 8
        elif enc in ('rgba8', 'bgra8'):
            for i in range(len(px)):
                j = 4 * i
                px[i] = (data[j] * 77 + data[j + 1] * 150 + data[j + 2] * 29) >> 8
        else:
            px[:] = data[:len(px)]
        if sigma_scale > 1.0 or self.bias:
            noise = 6.0 * sigma_scale
            for i in range(len(px)):
                px[i] = max(0, min(255, int(px[i] + gauss(self.rng, noise) + 40 * self.bias)))
        out.data = bytes(px)
        return out


class GzImuDriver(GzSensorDriver):
    """gz imu -> /sim/imu/data (sensor_msgs/Imu, ~20 Hz)."""

    SIGMA_W, SIGMA_A = 0.01, 0.05

    def __init__(self, seed: int = 7):
        super().__init__('imu_node', f'{RAW_NS}/imu', '/sim/imu/data', Imu, seed)

    def convert(self, msg: Imu, sigma_scale: float):
        out = Imu()
        out.header.frame_id = 'imu'
        out.orientation = msg.orientation
        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = msg.angular_velocity.y
        out.angular_velocity.z = msg.angular_velocity.z + gauss(self.rng, self.SIGMA_W * sigma_scale) + self.bias
        out.linear_acceleration.x = msg.linear_acceleration.x + gauss(self.rng, self.SIGMA_A * sigma_scale)
        out.linear_acceleration.y = msg.linear_acceleration.y
        out.linear_acceleration.z = msg.linear_acceleration.z
        return out


class GzOdometryDriver(GzSensorDriver):
    """gz DiffDrive odometry -> /sim/odom (nav_msgs/Odometry, ~20 Hz)."""

    SLIP_SIGMA = 0.002

    def __init__(self, seed: int = 7):
        super().__init__('odometry_node', f'{RAW_NS}/odom', '/sim/odom', Odometry, seed)

    def convert(self, msg: Odometry, sigma_scale: float):
        out = Odometry()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose = msg.pose
        out.pose.pose.position.x += self.bias
        v = msg.twist.twist.linear.x * (1.0 + gauss(self.rng, self.SLIP_SIGMA * sigma_scale))
        w = msg.twist.twist.angular.z * (1.0 + gauss(self.rng, self.SLIP_SIGMA * sigma_scale))
        out.twist.twist.linear.x = v + self.bias * 0.2
        out.twist.twist.angular.z = w + self.bias * 0.1
        return out


class GzWorldNode(SimNode):
    """Actuation sink: /cmd_vel (TwistStamped) -> /gz/cmd_vel (Twist) -> Gazebo DiffDrive.

    The physical feedback loop (command -> Gazebo physics -> sensors) is closed
    inside the simulator, not through ROS, so the ROS communication/dependency
    graph the monitor discovers remains a DAG with this node as the sink. See the
    README section "ROS communication graph vs physical feedback".
    """

    def __init__(self, seed: int = 7):
        super().__init__('sim_world', seed=seed)
        self.pub = self.create_publisher(Twist, f'{RAW_NS}/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)
        self.commands = 0
        self.get_logger().info('sim_world ready (/cmd_vel -> Gazebo DiffDrive)')

    def _on_cmd(self, msg: TwistStamped):
        self.workload()
        t = Twist()
        t.linear.x = msg.twist.linear.x
        t.angular.z = msg.twist.angular.z
        self.commands += 1
        self.pub.publish(t)


def lidar_main(args=None):
    spin_node(GzLidarDriver)


def camera_main(args=None):
    spin_node(GzCameraDriver)


def imu_main(args=None):
    spin_node(GzImuDriver)


def odometry_main(args=None):
    spin_node(GzOdometryDriver)


def world_main(args=None):
    spin_node(GzWorldNode)
