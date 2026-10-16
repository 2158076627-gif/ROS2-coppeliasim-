"""Force feedback to servo correction logic.

This module distills the force-guided insertion prototypes into a small,
testable controller. It does not command hardware directly. The ROS adapter
publishes a bounded TwistStamped correction for the synthetic demonstration.
A physical controller needs a separately validated adapter and sign convention.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from math import isfinite, hypot


@dataclass(frozen=True)
class WrenchSample:
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0


@dataclass(frozen=True)
class ServoCorrection:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    contact: bool = False
    emergency_stop: bool = False


@dataclass(frozen=True)
class AdmittanceConfig:
    lateral_force_threshold_n: float = 7.5
    axial_contact_threshold_n: float = 12.0
    emergency_force_threshold_n: float = 45.0
    torque_threshold_nm: float = 0.5
    lateral_gain: float = 0.0007
    axial_search_speed_mps: float = -0.0010
    contact_insert_speed_mps: float = -0.00035
    angular_gain: float = 0.010
    max_linear_speed_mps: float = 0.006
    max_angular_speed_rps: float = 0.040


class AdmittanceController:
    """Legacy-named bounded force-to-velocity heuristic; no dynamic admittance model."""

    def __init__(self, config: AdmittanceConfig | None = None):
        self.config = config or AdmittanceConfig()
        cfg = self.config
        if not all(isfinite(getattr(cfg, f.name)) for f in fields(cfg)):
            raise ValueError('controller parameters must be finite')
        for name in ('lateral_force_threshold_n', 'axial_contact_threshold_n',
                     'emergency_force_threshold_n', 'torque_threshold_nm',
                     'max_linear_speed_mps', 'max_angular_speed_rps'):
            if getattr(cfg, name) <= 0:
                raise ValueError(name + ' must be positive')
        if cfg.emergency_force_threshold_n <= cfg.axial_contact_threshold_n:
            raise ValueError('emergency threshold must exceed contact threshold')

    def compute(self, wrench: WrenchSample) -> ServoCorrection:
        cfg = self.config
        if not all(isfinite(getattr(wrench, f.name)) for f in fields(wrench)):
            return ServoCorrection(emergency_stop=True)
        force_norm = hypot(wrench.fx, wrench.fy, wrench.fz)
        emergency = force_norm >= cfg.emergency_force_threshold_n
        contact = abs(wrench.fz) >= cfg.axial_contact_threshold_n or emergency

        if emergency:
            return ServoCorrection(contact=True, emergency_stop=True)

        vx = -cfg.lateral_gain * wrench.fx if abs(wrench.fx) >= cfg.lateral_force_threshold_n else 0.0
        vy = -cfg.lateral_gain * wrench.fy if abs(wrench.fy) >= cfg.lateral_force_threshold_n else 0.0
        vz = cfg.contact_insert_speed_mps if contact else cfg.axial_search_speed_mps

        wx = -cfg.angular_gain * wrench.tx if abs(wrench.tx) >= cfg.torque_threshold_nm else 0.0
        wy = -cfg.angular_gain * wrench.ty if abs(wrench.ty) >= cfg.torque_threshold_nm else 0.0
        wz = -cfg.angular_gain * wrench.tz if abs(wrench.tz) >= cfg.torque_threshold_nm else 0.0

        return ServoCorrection(
            vx=clamp(vx, -cfg.max_linear_speed_mps, cfg.max_linear_speed_mps),
            vy=clamp(vy, -cfg.max_linear_speed_mps, cfg.max_linear_speed_mps),
            vz=clamp(vz, -cfg.max_linear_speed_mps, cfg.max_linear_speed_mps),
            wx=clamp(wx, -cfg.max_angular_speed_rps, cfg.max_angular_speed_rps),
            wy=clamp(wy, -cfg.max_angular_speed_rps, cfg.max_angular_speed_rps),
            wz=clamp(wz, -cfg.max_angular_speed_rps, cfg.max_angular_speed_rps),
            contact=contact,
            emergency_stop=False,
        )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
