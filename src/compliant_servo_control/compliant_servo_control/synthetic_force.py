"""Artificial feedback signal, with error-positive force sign convention.

Positive penetration gives positive Fz, so the heuristic slows negative-Z
insertion. This is not the signed environmental reaction wrench of a simulator.
"""
from .admittance_controller import WrenchSample


def synthetic_wrench(position):
    x, y, z = position
    penetration = max(0., .2 - z)
    active = penetration > 0.
    return WrenchSample(fx=2000. * (x - .4) if active else 0.,
                        fy=2000. * y if active else 0.,
                        fz=10000. * penetration,
                        tx=20. * y if active else 0.,
                        ty=20. * (x - .4) if active else 0., tz=0.)
