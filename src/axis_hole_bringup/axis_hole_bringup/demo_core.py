"""Deterministic translational toy model; NOT a contact physics engine."""
from math import isfinite, sqrt
from compliant_servo_control.synthetic_force import synthetic_wrench


class DemoPlant:
    def __init__(self):
        self.position = (.4, 0., .25)

    def step(self, velocity, dt):
        if dt <= 0 or not all(isfinite(v) for v in (*velocity, dt)):
            raise ValueError('finite velocity and positive dt required')
        # Toy workspace and component speed limits, independent of the executor.
        self.position = tuple(max(lo, min(hi, p + max(-.08, min(.08, v)) * dt))
                              for p, v, lo, hi in zip(self.position, velocity,
                                                     (.3, -.1, .18), (.5, .1, .4)))

    def wrench(self):
        return synthetic_wrench(self.position)


class Executor:
    """Single-shot waypoint follower. Plans are fixed for the entire execution."""
    def __init__(self):
        self.index = 0
        self.dwell = 0.
        self.done = False
        self.halted = False

    def step(self, plan, position, correction, dt):
        if correction.emergency_stop:
            self.halted = True
        if self.halted or self.done or not plan:
            return (0., 0., 0.)
        waypoint = plan[self.index]
        error = tuple(t - p for t, p in zip(waypoint.pose.position, position))
        distance = sqrt(sum(e * e for e in error))
        if distance <= waypoint.tolerance_m:
            self.dwell += dt
            if self.dwell >= waypoint.dwell_s:
                self.index += 1
                self.dwell = 0.
                self.done = self.index == len(plan)
            return (0., 0., 0.)
        self.dwell = 0.
        speed = waypoint.speed_mps
        nominal = tuple(e / distance * min(speed, distance / dt) for e in error)
        if waypoint.name == 'insert':
            # Only the insertion leg uses the force-driven axial speed.
            return (max(-speed, min(speed, nominal[0] + correction.vx)),
                    max(-speed, min(speed, nominal[1] + correction.vy)),
                    max(-speed, min(speed, correction.vz)))
        return nominal
