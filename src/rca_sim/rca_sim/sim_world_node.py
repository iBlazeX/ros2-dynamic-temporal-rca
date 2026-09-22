"""Simulation world process.

Hosts the physical world (kinematics) and the *sensor* ROS nodes in one process:

    sim_world       subscribes /cmd_vel (actuation sink), steps the physics
    lidar_node      publishes /sim/lidar/scan       sensor_msgs/LaserScan   10 Hz
    camera_node     publishes /sim/camera/image     sensor_msgs/Image        5 Hz
    imu_node        publishes /sim/imu/data         sensor_msgs/Imu         20 Hz
    odometry_node   publishes /sim/odom             nav_msgs/Odometry       20 Hz

The physical feedback loop (actuation -> world -> sensors) is closed inside this
process, exactly as with a real robot: it is not a ROS communication dependency,
so the ROS graph seen by the diagnostic monitor is a DAG
    sensors -> perception/localization -> planning -> control -> sim_world.

Sensor nodes are individually degradable (noise, bias, dropout, stale, rate,
latency, jitter, loss) and can be crashed / stopped / restarted as ROS nodes
without killing the world (node disappearance + reappearance in the graph).
"""

import json
import math
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String

from .sim_node_base import FAULT_TOPIC, SimNode
from .world import World, gauss

SENSOR_NODES = ('lidar_node', 'camera_node', 'imu_node', 'odometry_node')


class SensorNode(SimNode):
    """Common sensor behaviour: measurement faults + stale repeats + dropout."""

    def __init__(self, name: str, world: World, rate_hz: float, lifecycle_cb, seed: int):
        super().__init__(name, lifecycle_cb=lifecycle_cb, seed=seed, use_global_arguments=False)
        self.world = world
        self.rate_hz = rate_hz
        self.last_msg = None
        self.bias = 0.0
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        f = self.faults
        if f.is_active('dropout') and self.rng.random() < min(1.0, f.param('dropout', 1.0)):
            return
        if f.is_active('stale') and self.last_msg is not None:
            # re-publish the last measurement (old stamp): the consumer sees stale data
            self.out.publish(self.last_msg)
            return
        if f.is_active('bias'):
            self.bias = f.param('bias', 0.5)
        else:
            self.bias = 0.0
        sigma_scale = f.param('noise', 8.0) if f.is_active('noise') else 1.0
        msg = self.measure(sigma_scale)
        self.last_msg = msg
        self.out.publish(msg)

    def measure(self, sigma_scale: float):
        raise NotImplementedError


class LidarNode(SensorNode):
    N_BEAMS, FOV, MAX_RANGE, SIGMA = 90, math.radians(180), 15.0, 0.02

    def __init__(self, world, lifecycle_cb, seed):
        super().__init__('lidar_node', world, 10.0, lifecycle_cb, seed)
        self.out = self.link(self.create_publisher(LaserScan, '/sim/lidar/scan', 10))

    def measure(self, sigma_scale):
        scan = LaserScan()
        scan.header.stamp = self.stamp()
        scan.header.frame_id = 'lidar'
        scan.angle_min, scan.angle_max = -self.FOV / 2, self.FOV / 2
        scan.angle_increment = self.FOV / (self.N_BEAMS - 1)
        scan.scan_time = 1.0 / self.rate_hz
        scan.range_min, scan.range_max = 0.1, self.MAX_RANGE
        sigma = self.SIGMA * sigma_scale
        ranges = []
        for r in self.world.lidar_scan(self.N_BEAMS, self.FOV, self.MAX_RANGE):
            v = r + gauss(self.rng, sigma) + self.bias
            if sigma_scale > 3.0 and self.rng.random() < min(0.7, 0.04 * sigma_scale):
                v = float('nan')             # heavy noise also produces invalid returns
            ranges.append(max(scan.range_min, min(self.MAX_RANGE, v)) if not math.isnan(v) else v)
        scan.ranges = ranges
        return scan


class CameraNode(SensorNode):
    W, H, FOV, MAX_RANGE = 32, 24, math.radians(70), 7.0

    def __init__(self, world, lifecycle_cb, seed):
        super().__init__('camera_node', world, 5.0, lifecycle_cb, seed)
        self.out = self.link(self.create_publisher(Image, '/sim/camera/image', 10))

    def measure(self, sigma_scale):
        img = Image()
        img.header.stamp = self.stamp()
        img.header.frame_id = 'camera'
        img.height, img.width, img.encoding, img.step = self.H, self.W, 'mono8', self.W
        px = bytearray(self.W * self.H)
        noise_sigma = 6.0 * sigma_scale
        for i in range(len(px)):
            px[i] = max(0, min(255, int(30 + gauss(self.rng, noise_sigma))))
        # render each visible obstacle as a bright vertical bar (column ~ bearing, brightness ~ 1/range)
        for rng, bearing in self.world.camera_detections(self.FOV, self.MAX_RANGE):
            col = int((0.5 - bearing / self.FOV) * (self.W - 1))
            col = max(0, min(self.W - 1, col + int(self.bias * 4)))
            val = max(120, min(255, int(255 - 25 * rng)))
            for row in range(self.H // 3, 2 * self.H // 3):
                px[row * self.W + col] = val
        img.data = bytes(px)
        return img


class ImuNode(SensorNode):
    SIGMA_W, SIGMA_A = 0.01, 0.05

    def __init__(self, world, lifecycle_cb, seed):
        super().__init__('imu_node', world, 20.0, lifecycle_cb, seed)
        self.out = self.link(self.create_publisher(Imu, '/sim/imu/data', 10))

    def measure(self, sigma_scale):
        w, a = self.world.imu_rates()
        m = Imu()
        m.header.stamp = self.stamp()
        m.header.frame_id = 'imu'
        m.angular_velocity.z = w + gauss(self.rng, self.SIGMA_W * sigma_scale) + self.bias
        m.linear_acceleration.x = a + gauss(self.rng, self.SIGMA_A * sigma_scale)
        m.orientation.w = 1.0
        return m


class OdometryNode(SensorNode):
    SLIP_SIGMA = 0.002

    def __init__(self, world, lifecycle_cb, seed):
        super().__init__('odometry_node', world, 20.0, lifecycle_cb, seed)
        self.out = self.link(self.create_publisher(Odometry, '/sim/odom', 10))
        x, y, th, _, _ = world.odometry()
        self.x, self.y, self.th = x, y, th          # dead-reckoned estimate (drifts a little)
        self.last_t = time.time()

    def measure(self, sigma_scale):
        _, _, _, v, w = self.world.odometry()
        now = time.time()
        dt = max(0.0, min(0.2, now - self.last_t))
        self.last_t = now
        v_m = v * (1.0 + gauss(self.rng, self.SLIP_SIGMA * sigma_scale)) + self.bias * 0.2
        w_m = w * (1.0 + gauss(self.rng, self.SLIP_SIGMA * sigma_scale)) + self.bias * 0.1
        self.x += v_m * math.cos(self.th) * dt
        self.y += v_m * math.sin(self.th) * dt
        self.th += w_m * dt
        m = Odometry()
        m.header.stamp = self.stamp()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = self.x, self.y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        m.twist.twist.linear.x, m.twist.twist.angular.z = v_m, w_m
        return m


class WorldNode(Node):
    """Actuation sink + physics stepping + sensor node lifecycle manager."""

    def __init__(self, executor: SingleThreadedExecutor, seed: int = 7, camera_enabled: bool = True):
        super().__init__('sim_world')
        self.declare_parameter('seed', seed)
        self.declare_parameter('camera_enabled', camera_enabled)
        self.seed = int(self.get_parameter('seed').value)
        self.world = World.default_arena(self.seed)
        self.executor = executor
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = 0.0
        self.sensors: Dict[str, Optional[SensorNode]] = {n: None for n in SENSOR_NODES}
        self.pending_restart: Dict[str, float] = {}
        self.pending_despawn = set()
        self._lock = threading.Lock()
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(String, FAULT_TOPIC, self._on_fault_for_absent, 10)
        self.create_timer(0.02, self._step)              # 50 Hz physics
        self.create_timer(0.25, self._manage)
        for n in SENSOR_NODES:
            if n == 'camera_node' and not bool(self.get_parameter('camera_enabled').value):
                continue
            self.spawn(n)
        self.get_logger().info(f'sim_world ready (seed={self.seed}, obstacles={len(self.world.obstacles)})')

    def _on_cmd(self, msg: TwistStamped):
        self.cmd_v, self.cmd_w = msg.twist.linear.x, msg.twist.angular.z
        self.last_cmd_time = time.time()

    def _step(self):
        if time.time() - self.last_cmd_time > 1.0:      # no controller: coast to stop
            self.cmd_v *= 0.9
            self.cmd_w *= 0.9
        self.world.step(0.02, self.cmd_v, self.cmd_w)

    # ----------------------------------------------------- sensor lifecycle

    def spawn(self, name: str):
        with self._lock:
            if self.sensors.get(name) is not None:
                return
            cb = (lambda action, after, _n=name: self.lifecycle(_n, action, after))
            cls = {'lidar_node': LidarNode, 'camera_node': CameraNode,
                   'imu_node': ImuNode, 'odometry_node': OdometryNode}[name]
            node = cls(self.world, cb, self.seed)
            self.sensors[name] = node
            self.executor.add_node(node)
            self.get_logger().info(f'sensor node started: {name}')

    def despawn(self, name: str):
        with self._lock:
            node = self.sensors.get(name)
            if node is None:
                return
            self.sensors[name] = None
            self.executor.remove_node(node)
            node.destroy_node()
            self.get_logger().warn(f'sensor node removed from ROS graph: {name}')

    def lifecycle(self, name: str, action: str, restart_after: float):
        # Called from inside the sensor node's own callback. Destroying a node while
        # its callback runs is unsafe, so the manager timer (same executor thread)
        # performs the removal on its next tick.
        self.pending_despawn.add(name)
        self.pending_restart[name] = time.time() + (restart_after if action == 'restart' else float('inf'))

    def _on_fault_for_absent(self, msg: String):
        """'restart'/'start' addressed to a sensor node that does not currently exist."""
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        target = cmd.get('target_node')
        if target in SENSOR_NODES and self.sensors.get(target) is None and \
                cmd.get('fault_type') in ('restart', 'start'):
            self.pending_restart[target] = time.time() + float(cmd.get('param', 0.0) or 0.0)

    def _manage(self):
        now = time.time()
        for name in list(self.pending_despawn):
            self.pending_despawn.discard(name)
            self.despawn(name)
        for name, t in list(self.pending_restart.items()):
            if now >= t and self.sensors.get(name) is None:
                self.pending_restart.pop(name)
                self.spawn(name)


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    world = WorldNode(executor)
    executor.add_node(world)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in list(world.sensors.values()):
            if n is not None:
                n.destroy_node()
        world.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
