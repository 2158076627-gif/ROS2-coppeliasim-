"""ROS/engine-independent geometry, wrench and motion contracts."""
from dataclasses import dataclass, asdict
import json
import math


def finite(values):
    return all(math.isfinite(v) for v in values)


@dataclass(frozen=True)
class SceneConfig:
    peg_radius_m: float = .005
    peg_length_m: float = .03
    peg_mass_kg: float = .1
    hole_radius_m: float = .0052  # polygon apothem, NOT mesh circumradius
    hole_depth_m: float = .02
    wall_thickness_m: float = .005
    wall_segments: int = 32
    hole_center_m: tuple = (.4, 0., .2)
    initial_tip_m: tuple = (.4, 0., .23)
    hole_offset_x_m: float = 0.
    friction: float = .2
    soft_erp: float = .2
    soft_cfm: float = 1e-5
    simulation_dt_s: float = .02
    dynamics_dt_s: float = .002
    motor_force_n: float = 60.
    stop_depth_m: float | None = None

    def __post_init__(self):
        for name in ('peg_radius_m', 'peg_length_m', 'peg_mass_kg', 'hole_radius_m',
                     'hole_depth_m', 'wall_thickness_m', 'simulation_dt_s',
                     'dynamics_dt_s', 'motor_force_n', 'soft_cfm'):
            v = getattr(self, name)
            if not math.isfinite(v) or v <= 0:
                raise ValueError(name)
        if (not isinstance(self.wall_segments, int) or not 8 <= self.wall_segments <= 128
                or not finite((self.friction, self.soft_erp, self.hole_offset_x_m))
                or self.friction < 0 or not 0 < self.soft_erp <= 1):
            raise ValueError('invalid engine/geometry parameters')
        if self.hole_radius_m <= self.peg_radius_m:
            raise ValueError('prototype requires a clearance fit')
        if any(len(p) != 3 or not finite(p) for p in (self.hole_center_m, self.initial_tip_m)):
            raise ValueError('position must be three finite values')
        ratio = self.simulation_dt_s / self.dynamics_dt_s
        if abs(ratio - round(ratio)) > 1e-8 or ratio < 1:
            raise ValueError('simulation step must be an integer multiple of dynamics step')
        if self.stop_depth_m is not None and not 0 < self.stop_depth_m < self.hole_depth_m:
            raise ValueError('stop must lie inside bore')
        if self.initial_tip_m[2] <= self.hole_center_m[2] + .001:
            raise ValueError('start above bore, without initial penetration')

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(**json.load(f))

    def walls(self):
        """Overlapping tangent cuboids form an open regular polygon bore."""
        radius = self.hole_radius_m + self.wall_thickness_m / 2
        # Use OUTER radius for length: neighboring blocks overlap outside the bore.
        length = 2 * (self.hole_radius_m + self.wall_thickness_m) * math.tan(math.pi / self.wall_segments)
        for i in range(self.wall_segments):
            theta = 2 * math.pi * i / self.wall_segments
            x, y, z = self.hole_center_m
            yield ([self.wall_thickness_m, length, self.hole_depth_m],
                   [x + self.hole_offset_x_m + radius * math.cos(theta),
                    y + radius * math.sin(theta), z - self.hole_depth_m / 2], theta)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def wrench_at_tip(matrix, force, torque, tip, sign):
    """Sensor-local reading -> world reaction at tip, xyzw pose elsewhere.

    matrix: CoppeliaSim row-major 3x4 world-from-sensor. sign applies to both
    force and moment; it MUST be checked with a known external contact.
    tau_tip = R*tau_sensor + (p_sensor - p_tip) x R*force.
    """
    if sign not in (-1, 1) or len(matrix) != 12 or len(force) != 3 or len(torque) != 3 or len(tip) != 3:
        raise ValueError('invalid wrench transform')
    if not finite((*matrix, *force, *torque, *tip)):
        raise ValueError('nonfinite wrench')
    rotate = lambda v: tuple(sign * sum(matrix[4*i+j]*v[j] for j in range(3)) for i in range(3))
    f, t = rotate(force), rotate(torque)
    lever = tuple(matrix[4*i+3] - tip[i] for i in range(3))
    shift = cross(lever, f)
    return (*f, *(t[i]+shift[i] for i in range(3)))


class MotionGuard:
    """One run, latched fault; an invalid command immediately replaces the old one."""
    def __init__(self, sign=0, timeout=.5, speed=.08, force_limit=45.):
        if sign not in (-1, 0, 1) or not finite((timeout, speed, force_limit)) or min(timeout, speed, force_limit) <= 0:
            raise ValueError('invalid guard settings')
        self.sign, self.timeout, self.speed, self.force_limit = sign, timeout, speed, force_limit
        self.command = (0., 0., 0.)
        self.received = self.stamp = None
        self.halted = False

    def accept(self, frame, stamp, now, received, linear, angular):
        self.command, self.received, self.stamp = (0., 0., 0.), None, None
        if (frame != 'robot_base' or len(linear) != 3 or len(angular) != 3
                or not finite((*linear, *angular, stamp, now, received))
                or stamp <= 0 or not 0 <= now-stamp <= self.timeout
                or any(abs(w) > 1e-12 for w in angular)
                or math.hypot(*linear) > self.speed):
            return False
        self.command, self.received, self.stamp = tuple(linear), received, stamp
        return True

    def velocity(self, now, monotonic_now, tip, reaction, valid):
        if len(tip) != 3 or not finite(tip) or not (.3 <= tip[0] <= .5 and -.1 <= tip[1] <= .1 and .18 <= tip[2] <= .4):
            self.halted = True
        if valid and (len(reaction) != 6 or not finite(reaction) or math.hypot(*reaction[:3]) >= self.force_limit):
            self.halted = True
        fresh = (self.received is not None and 0 <= monotonic_now-self.received <= self.timeout
                 and 0 <= now-self.stamp <= self.timeout)
        if self.halted or self.sign == 0 or not valid or not fresh:
            return (0., 0., 0.)
        return self.command


def config_dict(config):
    return asdict(config)
