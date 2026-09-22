"""2-D kinematic world for the lightweight mobile-robot simulation.

A differential-drive robot moves in a rectangular arena with circular
obstacles. The world integrates velocity commands and answers sensor queries
(LiDAR ray casts, camera field-of-view detections, IMU rates, wheel odometry).
Pure Python + math, deterministic given a seed. No ROS dependency, unit-testable.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Obstacle:
    x: float
    y: float
    r: float


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0        # linear velocity actually achieved (m/s)
    w: float = 0.0        # angular velocity actually achieved (rad/s)


@dataclass
class World:
    width: float = 20.0
    height: float = 20.0
    obstacles: List[Obstacle] = field(default_factory=list)
    robot: RobotState = field(default_factory=RobotState)
    max_v: float = 0.8
    max_w: float = 1.5
    accel: float = 1.5           # m/s^2 velocity slew
    seed: int = 7
    t: float = 0.0
    _rng: random.Random = field(default_factory=lambda: random.Random(7), repr=False)

    @classmethod
    def default_arena(cls, seed: int = 7) -> 'World':
        rng = random.Random(seed)
        w = cls(seed=seed)
        w._rng = rng
        # deterministic obstacle field, keep a free corridor around the start
        for _ in range(12):
            while True:
                x = rng.uniform(-8.5, 8.5)
                y = rng.uniform(-8.5, 8.5)
                if math.hypot(x, y) > 2.5:
                    break
            w.obstacles.append(Obstacle(x, y, rng.uniform(0.4, 0.9)))
        w.robot = RobotState(x=-6.0, y=-6.0, theta=math.pi / 4)
        return w

    # ------------------------------------------------------------ dynamics

    def step(self, dt: float, cmd_v: float, cmd_w: float):
        """Integrates one control step with velocity limits and slew rate."""
        cmd_v = max(-self.max_v, min(self.max_v, cmd_v))
        cmd_w = max(-self.max_w, min(self.max_w, cmd_w))
        dv = max(-self.accel * dt, min(self.accel * dt, cmd_v - self.robot.v))
        dw = max(-3.0 * self.accel * dt, min(3.0 * self.accel * dt, cmd_w - self.robot.w))
        self.robot.v += dv
        self.robot.w += dw
        nx = self.robot.x + self.robot.v * math.cos(self.robot.theta) * dt
        ny = self.robot.y + self.robot.v * math.sin(self.robot.theta) * dt
        if not self.collides(nx, ny):
            self.robot.x, self.robot.y = nx, ny
        else:
            self.robot.v = 0.0
        self.robot.theta = self._wrap(self.robot.theta + self.robot.w * dt)
        self.t += dt

    def collides(self, x: float, y: float, margin: float = 0.3) -> bool:
        half_w, half_h = self.width / 2 - margin, self.height / 2 - margin
        if abs(x) > half_w or abs(y) > half_h:
            return True
        return any(math.hypot(x - o.x, y - o.y) < o.r + margin for o in self.obstacles)

    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # -------------------------------------------------------------- sensors

    def ray_cast(self, angle_world: float, max_range: float) -> float:
        """Distance from the robot along `angle_world` to the nearest obstacle or wall."""
        rx, ry = self.robot.x, self.robot.y
        dx, dy = math.cos(angle_world), math.sin(angle_world)
        best = max_range
        # walls
        half_w, half_h = self.width / 2, self.height / 2
        for wall_t in (
            (half_w - rx) / dx if dx > 1e-9 else None,
            (-half_w - rx) / dx if dx < -1e-9 else None,
            (half_h - ry) / dy if dy > 1e-9 else None,
            (-half_h - ry) / dy if dy < -1e-9 else None,
        ):
            if wall_t is not None and 0 < wall_t < best:
                best = wall_t
        # circles: solve |p + t d - c|^2 = r^2
        for o in self.obstacles:
            fx, fy = rx - o.x, ry - o.y
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - o.r * o.r
            disc = b * b - 4 * c
            if disc < 0:
                continue
            t = (-b - math.sqrt(disc)) / 2
            if 0 < t < best:
                best = t
        return best

    def lidar_scan(self, n_beams: int, fov: float, max_range: float) -> List[float]:
        start = self.robot.theta - fov / 2
        step = fov / max(1, n_beams - 1)
        return [self.ray_cast(start + i * step, max_range) for i in range(n_beams)]

    def camera_detections(self, fov: float, max_range: float) -> List[Tuple[float, float]]:
        """Obstacles inside the camera frustum, as (range, bearing) in the robot frame."""
        out = []
        for o in self.obstacles:
            dx, dy = o.x - self.robot.x, o.y - self.robot.y
            rng = math.hypot(dx, dy)
            bearing = self._wrap(math.atan2(dy, dx) - self.robot.theta)
            if rng <= max_range and abs(bearing) <= fov / 2:
                out.append((rng, bearing))
        return out

    def imu_rates(self) -> Tuple[float, float]:
        """(angular velocity z, linear acceleration x) from the current state."""
        return self.robot.w, 0.0

    def odometry(self) -> Tuple[float, float, float, float, float]:
        return self.robot.x, self.robot.y, self.robot.theta, self.robot.v, self.robot.w


def waypoint_loop(radius: float = 5.0, n: int = 8) -> List[Tuple[float, float]]:
    """Closed loop of waypoints the planner cycles through."""
    return [(radius * math.cos(2 * math.pi * k / n), radius * math.sin(2 * math.pi * k / n)) for k in range(n)]


def wrap_angle(a: float) -> float:
    return World._wrap(a)


def gauss(rng: Optional[random.Random], sigma: float) -> float:
    return (rng or random).gauss(0.0, sigma) if sigma > 0 else 0.0
