"""Deterministic scene source. Requires a stopped dedicated empty CoppeliaSim scene."""
import argparse
import json
import math
from pathlib import Path
from .contracts import SceneConfig, config_dict


def connect(host='localhost', port=23000):
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    import zmq
    client = RemoteAPIClient(host, port)
    # The client's default RPC timeout is ten minutes. Bound socket waits too.
    client.timeout = 3
    client.socket.setsockopt(zmq.RCVTIMEO, 3000)
    client.socket.setsockopt(zmq.SNDTIMEO, 3000)
    client.socket.setsockopt(zmq.LINGER, 0)
    return client, client.require('sim')


def build_scene(sim, cfg):
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Stop simulation before building')
    if sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0):
        raise RuntimeError('Use an empty scene: refusing to alter existing shapes')
    sim.setInt32Param(sim.intparam_dynamic_engine, sim.physics_ode)
    sim.setFloatParam(sim.floatparam_simulation_time_step, cfg.simulation_dt_s)
    sim.setEngineFloatParam(sim.ode_global_stepsize, -1, cfg.dynamics_dt_s)
    sim.setArrayParam(sim.arrayparam_gravity, [0., 0., 0.])
    root = sim.createDummy(.005)
    sim.setObjectAlias(root, 'contact_rig')
    sim.setObjectPosition(root, sim.handle_world, [0., 0., 0.])

    def shape(name, kind, sizes, position, dynamic=False, respondable=True, mass=.1, parent=root):
        h = sim.createPrimitiveShape(kind, sizes, 0)
        sim.setObjectAlias(h, name)
        sim.setObjectPosition(h, sim.handle_world, position)
        sim.setObjectParent(h, parent, True)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 0 if dynamic else 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, int(respondable))
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable_mask, 65535)
        # Set mass AND inertia together using geometric density, not mass alone.
        volume = math.pi*(sizes[0]/2)**2*sizes[2] if kind == sim.primitiveshape_cylinder else math.prod(sizes)
        sim.computeMassAndInertia(h, mass/volume)
        if respondable:
            for param, value in ((sim.ode_body_friction, cfg.friction),
                                 (sim.ode_body_softerp, cfg.soft_erp),
                                 (sim.ode_body_softcfm, cfg.soft_cfm)):
                sim.setEngineFloatParam(param, h, value)
                if not math.isclose(sim.getEngineFloatParam(param, h), value, rel_tol=1e-5, abs_tol=1e-8):
                    raise RuntimeError('Engine rejected material parameter')
        return h

    walls = []
    for i, (size, position, angle) in enumerate(cfg.walls()):
        wall = shape(f'wall_{i:02d}', sim.primitiveshape_cuboid, size, position)
        sim.setObjectOrientation(wall, sim.handle_world, [0., 0., angle])
        walls.append(wall)
    if cfg.stop_depth_m is not None:
        x, y, z = cfg.hole_center_m
        shape('physical_stop', sim.primitiveshape_cuboid,
              [.03, .03, .002], [x+cfg.hole_offset_x_m, y, z-cfg.stop_depth_m-.001])

    x, y, z = cfg.initial_tip_m
    sensor_position = [x, y, z+cfg.peg_length_m]
    parent = shape('anchor', sim.primitiveshape_cuboid, [.01]*3, sensor_position,
                   respondable=False)
    joints = []
    for axis, orientation in zip('xyz', ([0., math.pi/2, 0.], [-math.pi/2, 0., 0.], [0., 0., 0.])):
        joint = sim.createJoint(sim.joint_prismatic, sim.jointmode_dynamic, 0)
        sim.setObjectAlias(joint, f'drive_{axis}')
        sim.setObjectPosition(joint, sim.handle_world, sensor_position)
        sim.setObjectOrientation(joint, sim.handle_world, orientation)
        sim.setObjectParent(joint, parent, True)
        sim.setJointInterval(joint, False, [-.08, .16])
        sim.setObjectInt32Param(joint, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity)
        sim.setJointTargetForce(joint, cfg.motor_force_n)
        sim.setJointTargetVelocity(joint, 0.)
        # ODE needs auxiliary masses between consecutive dynamic joints.
        parent = shape(f'carriage_{axis}', sim.primitiveshape_cuboid, [.01]*3,
                       sensor_position, dynamic=True, respondable=False, parent=joint)
        joints.append(joint)
    sensor = sim.createForceSensor(0, [0, 1, 1, 0, 0], [.005, 1000., 100., 0., 0.])
    sim.setObjectAlias(sensor, 'wrist_sensor')
    sim.setObjectPosition(sensor, sim.handle_world, sensor_position)
    sim.setObjectParent(sensor, parent, True)
    peg = shape('peg', sim.primitiveshape_cylinder,
                [2*cfg.peg_radius_m, 2*cfg.peg_radius_m, cfg.peg_length_m],
                [x, y, z+cfg.peg_length_m/2], dynamic=True, mass=cfg.peg_mass_kg, parent=sensor)
    tip = sim.createDummy(.002)
    sim.setObjectAlias(tip, 'tool_tip')
    sim.setObjectPosition(tip, sim.handle_world, [x, y, z])
    sim.setObjectParent(tip, peg, True)
    metadata = {'schema': 1, 'source': 'physical_contact_prototype', 'config': config_dict(cfg),
                'engine': 'ODE', 'gravity_mps2': [0,0,0], 'robot_model': '3P Cartesian rig',
                'simulator_version': sim.getInt32Param(sim.intparam_program_version),
                'actual_dt_s': sim.getSimulationTimeStep(),
                'actual_dynamics_dt_s': sim.getEngineFloatParam(sim.ode_global_stepsize, -1)}
    if (not math.isclose(metadata['actual_dt_s'], cfg.simulation_dt_s, abs_tol=1e-8)
            or not math.isclose(metadata['actual_dynamics_dt_s'], cfg.dynamics_dt_s, abs_tol=1e-8)):
        raise RuntimeError('Engine step readback mismatch')
    sim.writeCustomDataBlock(root, 'contact_config', json.dumps(metadata))
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True, help='Absolute .ttt path on simulator host')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=23000)
    args = parser.parse_args()
    if not Path(args.output).is_absolute() or Path(args.output).suffix != '.ttt':
        parser.error('--output must be an absolute .ttt path')
    cfg = SceneConfig.load(args.config)
    _, sim = connect(args.host, args.port)
    metadata = build_scene(sim, cfg)
    sim.saveScene(args.output)
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
