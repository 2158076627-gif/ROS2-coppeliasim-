"""Single owner of stepping; never teleports the dynamic peg."""
import json
import math
from .contracts import SceneConfig, finite, wrench_at_tip
from .scene import connect


class CoppeliaBackend:
    def __init__(self, host='localhost', port=23000):
        self.client, self.sim = connect(host, port)
        s = self.sim
        if s.getSimulationState() != s.simulation_stopped:
            raise RuntimeError('Dedicated simulator must be stopped; do not share stepping ownership')
        self.root = s.getObject('/contact_rig')
        raw = s.readCustomDataBlock(self.root, 'contact_config')
        self.metadata = json.loads(raw)
        self.cfg = SceneConfig(**self.metadata['config'])
        self.dt = self.cfg.simulation_dt_s
        if (s.getInt32Param(s.intparam_dynamic_engine) != s.physics_ode
                or not math.isclose(s.getSimulationTimeStep(), self.dt, abs_tol=1e-8)
                or not math.isclose(s.getEngineFloatParam(s.ode_global_stepsize, -1), self.cfg.dynamics_dt_s, abs_tol=1e-8)
                or any(abs(v) > 1e-10 for v in s.getArrayParam(s.arrayparam_gravity))):
            raise RuntimeError('Scene engine/time/gravity differs from metadata; rebuild')
        self.joints = [s.getObject('/contact_rig/anchor/drive_x' + suffix) for suffix in (
            '', '/carriage_x/drive_y', '/carriage_x/drive_y/carriage_y/drive_z')]
        # Alias lookup across the hierarchy (unique within generated scene).
        self.sensor = s.getObject('/wrist_sensor')
        self.tip = s.getObject('/tool_tip')
        self.peg = s.getObject('/peg')
        self.last_time = None
        self.started = False

    def start(self):
        self.sim.setStepping(True)
        self.sim.startSimulation()
        self.started = True
        self.last_time = self.sim.getSimulationTime()

    def command(self, velocity):
        if len(velocity) != 3 or not finite(velocity):
            raise ValueError('invalid velocity')
        for joint, value in zip(self.joints, velocity):
            self.sim.setJointTargetVelocity(joint, value)

    def advance(self):
        self.sim.step()
        t = self.sim.getSimulationTime()
        if self.last_time is None or not math.isclose(t-self.last_time, self.dt, rel_tol=1e-5, abs_tol=1e-8):
            raise RuntimeError('Simulation reset, paused, or another stepping owner')
        self.last_time = t
        return self.sample()

    def sample(self):
        s = self.sim
        pose = s.getObjectPose(self.tip, s.handle_world)  # xyz, xyzw
        if len(pose) != 7 or not finite(pose) or abs(sum(v*v for v in pose[3:])-1) > 1e-3:
            raise RuntimeError('invalid tool pose')
        # Fixed orientation contract: reject physical drift, do not fake identity.
        if sum(abs(v) for v in pose[3:6]) > 1e-6:
            raise RuntimeError('rig orientation drift exceeds fixed-attitude contract')
        if pose[6] < 0:
            pose = [*pose[:3], *(-v for v in pose[3:])]
        result, force, torque = s.readForceSensor(self.sensor)
        force, torque = force or [], torque or []
        valid = bool(result & 1) and len(force) == 3 and len(torque) == 3 and finite((*force, *torque))
        contacts = []
        for index in range(256):
            contact = s.getContactInfo(s.handle_all, self.peg, index)
            if not contact or not contact[0]:
                break
            coll, point, reaction, normal = contact
            contacts.append({'objects': list(coll), 'point_world_m': list(point),
                             'reaction_world_n': list(reaction), 'normal_world': list(normal)})
        return {'simulation_time_s': s.getSimulationTime(), 'pose': list(pose),
                'sensor_matrix': list(s.getObjectMatrix(self.sensor, s.handle_world)),
                'raw_force': list(force), 'raw_torque': list(torque), 'sensor_valid': valid,
                'contacts': contacts, 'contact_list_truncated': len(contacts) == 256}

    def close(self):
        if self.started:
            try:
                self.command((0., 0., 0.))
            finally:
                # stopSimulation is asynchronous; do not release stepping and run freely.
                self.sim.stopSimulation()
                self.started = False


def reaction(sample, sign):
    if not sample['sensor_valid'] or sign == 0:
        return None
    return wrench_at_tip(sample['sensor_matrix'], sample['raw_force'], sample['raw_torque'],
                         sample['pose'][:3], sign)
