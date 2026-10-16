"""Core trajectory planning primitives independent from ROS 2.

The planner turns a perceived assembly target pose into a short, explicit
sequence of Cartesian waypoints. The sequence is intentionally conservative:
observe, approach, contact-search, insert, hold, and retract.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt, isfinite
from typing import Iterable, List, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Pose3D:
    """Minimal Cartesian pose representation used by tests and ROS adapters."""

    position: Vector3
    orientation: Quaternion = (0.0, 0.0, 0.0, 1.0)

    def shifted(self, offset: Vector3) -> 'Pose3D':
        return Pose3D(
            position=(
                self.position[0] + offset[0],
                self.position[1] + offset[1],
                self.position[2] + offset[2],
            ),
            orientation=self.orientation,
        )


@dataclass(frozen=True)
class Waypoint:
    """One named stage in the assembly trajectory."""

    name: str
    pose: Pose3D
    speed_mps: float
    tolerance_m: float
    dwell_s: float = 0.0


@dataclass(frozen=True)
class PlannerConfig:
    approach_distance_m: float = 0.080
    observe_distance_m: float = 0.150
    contact_distance_m: float = 0.020
    insertion_depth_m: float = 0.040
    retract_distance_m: float = 0.060
    fast_speed_mps: float = 0.080
    align_speed_mps: float = 0.020
    insertion_speed_mps: float = 0.004
    fine_tolerance_m: float = 0.001
    coarse_tolerance_m: float = 0.005


class AssemblyTrajectoryPlanner:
    """Plan a deterministic peg-in-hole style Cartesian assembly sequence."""

    def __init__(self, config: PlannerConfig | None = None):
        self.config = config or PlannerConfig()

    def plan(self, target_pose: Pose3D, approach_axis: Sequence[float] = (0.0, 0.0, 1.0)) -> List[Waypoint]:
        axis = normalize3(approach_axis)
        cfg = self.config

        observe = target_pose.shifted(scale3(axis, cfg.observe_distance_m))
        approach = target_pose.shifted(scale3(axis, cfg.approach_distance_m))
        contact = target_pose.shifted(scale3(axis, cfg.contact_distance_m))
        insertion = target_pose.shifted(scale3(axis, -cfg.insertion_depth_m))
        retract = target_pose.shifted(scale3(axis, cfg.retract_distance_m))

        return [
            Waypoint('observe', observe, cfg.fast_speed_mps, cfg.coarse_tolerance_m),
            Waypoint('pre_align', approach, cfg.align_speed_mps, cfg.fine_tolerance_m),
            Waypoint('contact_search', contact, cfg.insertion_speed_mps, cfg.fine_tolerance_m),
            Waypoint('insert', insertion, cfg.insertion_speed_mps, cfg.fine_tolerance_m),
            Waypoint('hold', insertion, 0.0, cfg.fine_tolerance_m, dwell_s=0.5),
            Waypoint('retract', retract, cfg.align_speed_mps, cfg.coarse_tolerance_m),
        ]


def normalize3(vector: Sequence[float]) -> Vector3:
    if len(vector) != 3:
        raise ValueError('approach_axis must contain exactly three values')
    x, y, z = float(vector[0]), float(vector[1]), float(vector[2])
    norm = sqrt(x * x + y * y + z * z)
    if not isfinite(norm) or norm < 1e-9:
        raise ValueError('approach_axis must be non-zero')
    return (x / norm, y / norm, z / norm)


def scale3(vector: Sequence[float], scale: float) -> Vector3:
    return (float(vector[0]) * scale, float(vector[1]) * scale, float(vector[2]) * scale)


def waypoint_names(waypoints: Iterable[Waypoint]) -> List[str]:
    return [waypoint.name for waypoint in waypoints]
