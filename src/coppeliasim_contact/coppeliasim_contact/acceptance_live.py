"""Evidence acquisition primitives; the only live adapter remains CoppeliaBackend."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time

from .acceptance import (Limits, calibration_signature, contact_region, obstacles, startup_evidence)
from .backend import CoppeliaBackend, reaction
from .contracts import MotionGuard, config_dict, cross, finite, wrench_at_tip


def dump(path, value):
    with Path(path).open('x') as f:
        json.dump(value,f,indent=2,allow_nan=False)
        f.write('\n')


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            h.update(chunk)
    return h.hexdigest()


def code_version():
    root=Path(__file__).resolve().parents[3]
    def git(*args):
        return subprocess.check_output(['git','-C',str(root),*args],text=True).strip()
    return {'sha':git('rev-parse','HEAD'), 'dirty':bool(git('status','--porcelain')),
            'diff_sha256':hashlib.sha256(git('diff','HEAD','--').encode()).hexdigest()}


def seal(directory):
    directory=Path(directory)
    data={str(p.relative_to(directory)):sha256(p) for p in sorted(directory.rglob('*'))
          if p.is_file() and p!=directory/'artifacts.sha256.json'
          and 'derived' not in p.relative_to(directory).parts}
    dump(directory/'artifacts.sha256.json',{'schema':1,'files':data})


def verify_seal(directory):
    directory=Path(directory)
    try:
        record=json.loads((directory/'artifacts.sha256.json').read_text())
    except (OSError,ValueError):
        return ['missing/invalid artifacts.sha256.json']
    if (not isinstance(record,dict) or record.get('schema')!=1
            or not isinstance(record.get('files'),dict) or not record['files']):
        return ['invalid/empty evidence seal']
    data=record['files']
    actual={str(p.relative_to(directory)) for p in directory.rglob('*')
            if p.is_file() and p!=directory/'artifacts.sha256.json'
            and 'derived' not in p.relative_to(directory).parts}
    failures=[]
    if set(data)!=actual:
        failures.append('seal does not cover all immutable evidence files')
    for name,digest in data.items():
        if not isinstance(name,str) or not isinstance(digest,str) or len(digest)!=64:
            failures.append('invalid artifact seal entry')
            continue
        path=directory/name
        if not path.resolve().is_relative_to(directory.resolve()):
            failures.append(f'unsafe artifact path: {name}')
        elif not path.is_file() or sha256(path)!=digest:
            failures.append(f'missing/changed artifact: {name}')
    return failures


def validate_build(directory, run_meta=None):
    from .contracts import SceneConfig
    directory=Path(directory)
    errors=verify_seal(directory)
    required=('build.json','config.json','scene.ttt')
    if errors or not all((directory/p).is_file() for p in required):
        raise ValueError('invalid build bundle: '+'; '.join(errors or ['missing required build artifacts']))
    record=json.loads((directory/'build.json').read_text())
    if not isinstance(record,dict):
        raise ValueError('build.json must be an object')
    if not all(record.get(k) is True for k in ('complete','build_reload_verified','geometry_readback_verified')):
        raise ValueError('build/save/reload/readback has not passed')
    cfg=SceneConfig(**record['config'])
    if json.loads((directory/'config.json').read_text())!=json.loads(json.dumps(config_dict(cfg))):
        raise ValueError('build config snapshot mismatch')
    if sha256(directory/'scene.ttt')!=record['scene_sha256']:
        raise ValueError('scene hash mismatch')
    if run_meta is not None:
        for key in ('config','engine','scene_sha256'):
            if json.loads(json.dumps(record[key]))!=json.loads(json.dumps(run_meta.get(key))):
                raise ValueError(f'run/build evidence mismatch: {key}')
    return record


def readback(sim, cfg):
    """Read actual primitives, transforms, material and drive limits; reject edits.

    Unsupported API/version raises. It never substitutes intended config for
    measured geometry, or silently accepts another dynamics engine.
    """
    if sim.getSimulationState()!=sim.simulation_stopped:
        raise RuntimeError('geometry readback requires a stopped scene')
    objects={}
    geometry=[]
    expected=list(obstacles(cfg))
    x,y,z=cfg.initial_tip_m
    expected.append(('peg',[2*cfg.peg_radius_m,2*cfg.peg_radius_m,cfg.peg_length_m],
                     [x,y,z+cfg.peg_length_m/2],0.))
    for name,size,center,yaw in expected:
        handle=sim.getObject('/'+name)
        result,kind,dimensions=sim.getShapeGeomInfo(handle)
        target_type=sim.primitiveshape_cylinder if name=='peg' else sim.primitiveshape_cuboid
        if not result & 2 or result & 1 or kind!=target_type:
            raise RuntimeError(f'not expected primitive: {name}')
        actual_size=list(dimensions[:3])
        matrix=list(sim.getObjectMatrix(handle,sim.handle_world))
        target=[math.cos(yaw),-math.sin(yaw),0.,center[0],math.sin(yaw),math.cos(yaw),0.,center[1],0.,0.,1.,center[2]]
        if (len(actual_size)!=3 or len(matrix)!=12 or not finite(actual_size+matrix)
                or any(not math.isclose(a,b,abs_tol=1e-7) for a,b in zip(actual_size,size))
                or any(not math.isclose(a,b,abs_tol=1e-7) for a,b in zip(matrix,target))):
            raise RuntimeError(f'geometry/pose readback mismatch: {name}')
        static=sim.getObjectInt32Param(handle,sim.shapeintparam_static)
        respondable=sim.getObjectInt32Param(handle,sim.shapeintparam_respondable)
        mask=sim.getObjectInt32Param(handle,sim.shapeintparam_respondable_mask)
        if static!=int(name!='peg') or respondable!=1 or mask!=65535:
            raise RuntimeError(f'dynamics flags readback mismatch: {name}')
        material={}
        for key,param,wanted in (('friction',sim.ode_body_friction,cfg.friction),
                                 ('soft_erp',sim.ode_body_softerp,cfg.soft_erp),
                                 ('soft_cfm',sim.ode_body_softcfm,cfg.soft_cfm)):
            value=sim.getEngineFloatParam(param,handle)
            if not math.isclose(value,wanted,rel_tol=1e-5,abs_tol=1e-8):
                raise RuntimeError(f'material readback mismatch: {name}/{key}')
            material[key]=value
        objects[str(handle)]=name
        geometry.append(dict(name=name,handle=handle,kind=kind,size_m=actual_size,
                             matrix=matrix,static=static,respondable=respondable,mask=mask,material=material))
    mass=sim.getShapeMass(sim.getObject('/peg'))
    if not math.isclose(mass,cfg.peg_mass_kg,rel_tol=1e-5):
        raise RuntimeError('peg mass mismatch')
    sensor_matrix=list(sim.getObjectMatrix(sim.getObject('/wrist_sensor'),sim.handle_world))
    sensor_target=[1.,0.,0.,x,0.,1.,0.,y,0.,0.,1.,z+cfg.peg_length_m]
    if len(sensor_matrix)!=12 or any(not math.isclose(a,b,abs_tol=1e-7) for a,b in zip(sensor_matrix,sensor_target)):
        raise RuntimeError('wrist sensor frame/lever arm mismatch')
    tip_pose=sim.getObjectPose(sim.getObject('/tool_tip'),sim.handle_world)
    if len(tip_pose)!=7 or math.dist(tip_pose[:3],cfg.initial_tip_m)>1e-7:
        raise RuntimeError('initial tip mismatch')
    drive_limits=[sim.getJointTargetForce(sim.getObject('/drive_'+axis)) for axis in 'xyz']
    if any(not math.isclose(abs(v),cfg.motor_force_n,rel_tol=1e-5) for v in drive_limits):
        raise RuntimeError('motor force readback mismatch')
    engine=dict(version=sim.getInt32Param(sim.intparam_program_version),
                revision=sim.getInt32Param(sim.intparam_program_revision),engine='ODE',
                dt_s=sim.getSimulationTimeStep(),dynamics_dt_s=sim.getEngineFloatParam(sim.ode_global_stepsize,-1))
    if (sim.getInt32Param(sim.intparam_dynamic_engine)!=sim.physics_ode
            or not math.isclose(engine['dt_s'],cfg.simulation_dt_s,abs_tol=1e-8)
            or not math.isclose(engine['dynamics_dt_s'],cfg.dynamics_dt_s,abs_tol=1e-8)
            or any(abs(v)>1e-10 for v in sim.getArrayParam(sim.arrayparam_gravity))):
        raise RuntimeError('engine/time/gravity readback mismatch')
    return dict(objects=objects,geometry=geometry,engine=engine,peg_mass_kg=mass,
                sensor_matrix=sensor_matrix,motor_force_n=drive_limits,geometry_readback_verified=True)


class EvidenceBackend(CoppeliaBackend):
    """Also preserve each sub-step's contacts and the actual cylinder pose.

    All-pass contacts from the existing backend remain available for auditing.
    Last-pass contacts are synchronous with the pose used for region assessment.
    Sub-step positions are not exposed here: overlap remains outer-step sampled.
    """
    def __init__(self, host='localhost', port=23000):
        super().__init__(host,port)
        self.evidence=readback(self.sim,self.cfg)

    def sample(self):
        row=super().sample()
        passes=[]
        truncated=False
        for substep in range(round(self.cfg.simulation_dt_s/self.cfg.dynamics_dt_s)):
            contacts=[]
            for index in range(256):
                info=self.sim.getContactInfo(substep,self.peg,index)
                if not info or not info[0]:
                    break
                coll,point,force,normal=info
                contacts.append(dict(objects=list(coll),point_world_m=list(point),
                                     reaction_world_n=list(force),normal_world=list(normal)))
            truncated |= len(contacts)==256
            passes.append(contacts)
        row['contacts_all_substeps']=row['contacts']
        row['substep_contacts']=passes
        row['contacts']=passes[-1]
        row['contact_list_truncated'] |= truncated
        peg_pose=list(self.sim.getObjectPose(self.peg,self.sim.handle_world))
        expected=[*row['pose'][:2],row['pose'][2]+self.cfg.peg_length_m/2]
        if (len(peg_pose)!=7 or not finite(peg_pose) or math.dist(peg_pose[:3],expected)>1e-7
                or sum(abs(x) for x in peg_pose[3:6])>1e-6):
            raise RuntimeError('actual peg and tip geometry no longer agree')
        row['peg_pose_world']=peg_pose
        return row

    def close(self):
        super().close()
        deadline=time.monotonic()+5.
        while self.sim.getSimulationState()!=self.sim.simulation_stopped:
            if time.monotonic()>deadline:
                raise RuntimeError('engine did not reach stopped state after stopSimulation')
            time.sleep(.02)


def read_run(directory):
    directory=Path(directory)
    meta=json.loads((directory/'manifest.json').read_text())
    if not isinstance(meta,dict):
        raise ValueError('manifest.json must be an object')
    rows=[json.loads(line) for line in (directory/'samples.jsonl').read_text().splitlines() if line.strip()]
    return meta,rows


def load_calibration(directory):
    """Recompute from snapshotted raw logs; do not trust a hand-written PASS flag."""
    directory=Path(directory)
    errors=verify_seal(directory)
    if errors:
        raise ValueError('; '.join(errors))
    saved=json.loads((directory/'calibration.json').read_text())
    if not isinstance(saved,dict):
        raise ValueError('calibration.json must be an object')
    am,ar=read_run(directory/'axial')
    lm,lr=read_run(directory/'lateral')
    for axis,meta in (('axial',am),('lateral',lm)):
        errors=verify_seal(directory/axis)
        if errors:
            raise ValueError('; '.join(errors))
        validate_build(directory/axis/'build',meta)
    result=calibrate(am,ar,lm,lr,saved['sign'])
    result.update(axial_sha256=sha256(directory/'axial/samples.jsonl'),
                  lateral_sha256=sha256(directory/'lateral/samples.jsonl'))
    for key in ('status','sign','signature','engine','axial_sha256','lateral_sha256'):
        if result[key]!=saved.get(key):
            raise ValueError(f'calibration replay mismatch: {key}')
    if result['status']!='PASS':
        raise ValueError('calibration has not passed: '+'; '.join(result['reasons']))
    return result


def open_prepared(build_directory, host='localhost', port=23000, calibration_directory=None):
    """Load only a recorded .ttt, recheck its hash, metadata and actual readback."""
    from .contracts import SceneConfig
    from .scene import connect
    directory=Path(build_directory).resolve()
    current_code=code_version()
    if current_code['dirty']:
        raise RuntimeError('commit source changes before reproducible live acquisition')
    record=validate_build(directory)
    scene=directory/'scene.ttt'
    if sha256(scene)!=record['scene_sha256']:
        raise ValueError('scene hash mismatch')
    _,sim=connect(host,port)
    if sim.getSimulationState()!=sim.simulation_stopped:
        raise RuntimeError('stop the dedicated simulator before loading the acceptance scene')
    # Never replace an unrelated, possibly unsaved user scene.
    shapes=sim.getObjectsInTree(sim.handle_scene,sim.object_shape_type,0)
    if shapes and sim.getObject('/contact_rig',{'noError':True})<0:
        raise RuntimeError('refusing to replace an unrelated scene; use a dedicated empty/contact-rig scene')
    sim.loadScene(str(scene))
    backend=EvidenceBackend(host,port)
    if config_dict(backend.cfg)!=record['config']:
        raise ValueError('loaded config differs from recorded build')
    if backend.evidence['engine']!=record['engine']:
        raise ValueError('engine version/revision changed since build; rebuild and recalibrate')
    # Handle numbers may legitimately change after reload. Always use current map.
    meta=dict(schema=1,source='coppeliasim_physics',config=config_dict(backend.cfg),
              scene_sha256=record['scene_sha256'],build_reload_verified=True,
              **backend.evidence,code=current_code,complete=False)
    if calibration_directory:
        calibration=load_calibration(calibration_directory)
        if (calibration['signature']!=calibration_signature(backend.cfg)
                or calibration['engine']!=meta['engine']):
            raise ValueError('calibration no longer matches scene geometry/material/engine')
        meta['calibration']=calibration
    return backend,meta


def requested_velocity(case, tip, cfg):
    if case=='sidewall' and tip[2]<=cfg.hole_center_m[2]-.002+1e-10:
        return (.0002,0.,0.)
    if tip[2]>cfg.hole_center_m[2]-.004:
        return (0.,0.,-.001)
    return (0.,0.,0.)


def capture_direct(backend, cfg, case, sign, log, limits, duration_s=45.):
    guard=MotionGuard(sign=sign,speed=.002,force_limit=limits.overload_force_n)
    latest=None
    trigger=None
    result={'complete':False,'baseline_force_n':None}
    baseline=[]
    try:
        backend.start()
        for i in range(math.ceil(duration_s/backend.dt)):
            now=(i+1)*backend.dt
            request=(0.,0.,0.)
            if latest is not None and i*backend.dt>=.2:
                request=requested_velocity(case,latest['pose'][:3],cfg)
            if guard.halted:
                request=(0.,0.,-.001)  # deliberate latch challenge; never applied
            guard.accept('robot_base',now,now,now,request,(0.,0.,0.))
            f=reaction(latest,sign) if latest else None
            velocity=guard.velocity(now,now,latest['pose'][:3] if latest else cfg.initial_tip_m,
                                    f or (0.,)*6,f is not None)
            backend.command(velocity)
            sample=backend.advance()
            f=reaction(sample,sign)
            guard.velocity(now,now,sample['pose'][:3],f or (0.,)*6,f is not None)
            after=(0.,0.,0.) if guard.halted or f is None else velocity
            if after!=velocity:
                backend.command(after)
            row={**sample,'kind':'sample','phase':'settle' if i*backend.dt<.2 else 'trial',
                 'reaction_at_tip':f,'command_requested_mps':request,'command_applied_mps':velocity,
                 'command_after_sample_mps':after,'stop_latched':guard.halted,
                 'wall_stamp_ns':time.time_ns(),'received_monotonic_ns':time.monotonic_ns()}
            log.write(json.dumps(row,allow_nan=False)+'\n')
            log.flush()
            if row['phase']=='settle' and f is not None and not row['contacts']:
                baseline.append(math.hypot(*f[:3]))
            latest=sample
            if guard.halted and trigger is None:
                trigger=sample['simulation_time_s']
            if trigger is not None and sample['simulation_time_s']-trigger>=limits.stop_hold_s-1e-9:
                break
        result['complete']=True
    except Exception as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        try:
            backend.command((0.,0.,0.))
            backend.close()
        except Exception as exc:
            result['complete']=False
            result['cleanup_error']=f'{type(exc).__name__}: {exc}'
    result['baseline_force_n']=max(baseline) if baseline else None
    return result


def calibrate(axial_meta, axial_rows, lateral_meta, lateral_rows, sign):
    """Low-force direction + approximate static moment-origin cross-check.

    Uses measured raw values and contact identities. This is not a traceable
    force/torque gain calibration. Up to 0.01 N m centroid moment residual is
    allowed at <= 5 N drive force; broad/nonparallel manifolds remain uncertain.
    """
    from .contracts import SceneConfig
    status='PASS'
    reasons=[]
    metrics={}
    def issue(message, failure=False):
        nonlocal status
        reasons.append(message)
        status='FAIL' if failure or status=='FAIL' else 'INSUFFICIENT'
    if sign not in (-1,1):
        raise ValueError('sign must be -1 or +1')
    configs=[]
    for axis,meta,rows in (('axial',axial_meta,axial_rows),('lateral',lateral_meta,lateral_rows)):
        try:
            cfg=SceneConfig(**meta['config'])
            configs.append(cfg)
            if (meta.get('source')!='coppeliasim_physics' or meta.get('build_reload_verified') is not True
                    or meta.get('geometry_readback_verified') is not True
                    or len(meta.get('scene_sha256',''))!=64 or len(meta.get('code',{}).get('sha',''))!=40):
                issue(f'{axis}: missing live scene/source/readback provenance')
            startup=startup_evidence(rows,cfg,sign)
            if not startup['trace_start'] or not startup['initial_pose'] or startup['baseline_force_n'] is None:
                issue(f'{axis}: missing full trace start/raw stationary baseline')
            elif startup['baseline_force_n']>1.:
                issue(f'{axis}: measured startup baseline >1 N',True)
            if cfg.motor_force_n>5. or meta.get('calibration_trial')!=axis:
                issue(f'{axis}: require dedicated <=5 N calibration trial',True)
            if not meta.get('complete') or meta.get('baseline_force_n') is None:
                issue(f'{axis}: incomplete trial or missing free-motion baseline')
            elif meta['baseline_force_n']>1.:
                issue(f'{axis}: free-motion force >1 N',True)
            selected=[]
            invalid=False
            for row in rows:
                if not isinstance(row,dict):
                    invalid=True
                    continue
                if not row.get('sensor_valid'):
                    if row.get('phase')!='settle' or row.get('simulation_time_s',1.)>.2:
                        invalid=True
                    continue
                f=wrench_at_tip(row['sensor_matrix'],row['raw_force'],row['raw_torque'],row['pose'][:3],sign)
                regions=[contact_region(c,row,cfg,meta.get('objects',{}),Limits()) for c in row['contacts']]
                if any(r in ('unexpected','unresolved') for r in regions) or row.get('contact_list_truncated') is not False:
                    invalid=True
                required='stop' if axis=='axial' else 'sidewall'
                if required not in regions:
                    continue
                applied=row['command_applied_mps']
                if (axis=='axial' and applied[2]>=0) or (axis=='lateral' and applied[0]<=0):
                    continue
                strength=math.hypot(*f[:3])
                if strength<.5:
                    continue
                direction=f[2] if axis=='axial' else -f[0]
                if direction<.5:
                    issue(f'{axis}: reaction does not oppose applied motion',True)
                if strength>8.:
                    issue(f'{axis}: force too large for low-force calibration',True)
                contacts=[c for c,r in zip(row['contacts'],regions) if r==required]
                weights=[math.hypot(*c['reaction_world_n']) for c in contacts]
                if sum(weights)<=0:
                    invalid=True
                    continue
                centroid=[sum(c['point_world_m'][j]*w for c,w in zip(contacts,weights))/sum(weights) for j in range(3)]
                lever=[centroid[j]-row['pose'][j] for j in range(3)]
                expected=cross(lever,f[:3])
                residual=math.dist(f[3:],expected)
                if residual>.01:
                    issue(f'{axis}: tip moment/contact centroid residual >0.01 N m',True)
                selected.append((row['simulation_time_s'],direction,residual))
            if invalid:
                issue(f'{axis}: invalid sensor/contact samples')
            dwell=0.
            start=None
            previous=None
            for t,_,_ in selected:
                if previous is None or not math.isclose(t-previous,cfg.simulation_dt_s,abs_tol=1e-7):
                    start=t
                dwell=max(dwell,t-start)
                previous=t
            if dwell<.2-1e-9:
                issue(f'{axis}: need >=0.2 s contiguous known-contact samples')
            metrics[axis]=dict(samples=len(selected),dwell_s=dwell,
                               min_opposing_force_n=min((v[1] for v in selected),default=None),
                               max_moment_residual_nm=max((v[2] for v in selected),default=None))
        except (KeyError,ValueError,TypeError,AttributeError) as exc:
            issue(f'{axis}: invalid raw evidence: {exc}')
    if len(configs)!=2 or calibration_signature(configs[0])!=calibration_signature(configs[-1]):
        issue('calibration configurations differ',True)
    if axial_meta.get('engine')!=lateral_meta.get('engine'):
        issue('calibration engine version/revision/steps differ',True)
    return dict(schema=1,status=status,sign=sign,reasons=reasons,metrics=metrics,
                signature=calibration_signature(configs[0]) if configs else None,
                engine=axial_meta.get('engine'))
