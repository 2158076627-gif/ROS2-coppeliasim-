"""Acquisition protocol tests with explicit API doubles, never contact physics."""
from copy import deepcopy
from dataclasses import replace
import json

import pytest

from coppeliasim_contact import acceptance_live as live
from coppeliasim_contact.acceptance import Limits
from coppeliasim_contact.contracts import SceneConfig, config_dict
from coppeliasim_contact.experiments import case_config
from test_contact_acceptance import fixture, contact, loaded, trial_rows
from test_contact_backend import RecordingSim
from coppeliasim_contact.scene import build_scene


class GeometrySim(RecordingSim):
    def __init__(self):
        super().__init__()
        self.names={}; self.geometry={}; self.positions={}; self.yaws={}; self.ints={}; self.masses={}; self.forces={}
    def __getattr__(self,name):
        if name.startswith(('shape','primitive','joint','ode_','intparam','floatparam','arrayparam','physics_')):
            return name
        base=super().__getattr__(name)
        def call(*args):
            if name=='getObject': return self.names[args[0].split('/')[-1]]
            if name=='getShapeGeomInfo':
                kind,size=self.geometry[args[0]]
                return 2,kind,list(size)+[0.]
            if name=='getObjectInt32Param': return self.ints[args[:2]]
            if name=='getShapeMass': return self.masses[args[0]]
            if name=='getJointTargetForce': return self.forces[args[0]]
            if name=='getObjectPose': return list(self.positions[args[0]])+[0.,0.,0.,1.]
            if name=='getObjectMatrix':
                import math
                x,y,z=self.positions[args[0]]; angle=self.yaws.get(args[0],0.)
                return [math.cos(angle),-math.sin(angle),0.,x,math.sin(angle),math.cos(angle),0.,y,0.,0.,1.,z]
            value=base(*args)
            if name=='createPrimitiveShape': self.geometry[value]=args[:2]
            if name=='setObjectAlias': self.names[args[1]]=args[0]
            if name=='setObjectPosition': self.positions[args[0]]=args[2]
            if name=='setObjectOrientation': self.yaws[args[0]]=args[2][2]
            if name=='setObjectInt32Param': self.ints[args[:2]]=args[2]
            if name=='setJointTargetForce': self.forces[args[0]]=args[1]
            if name=='computeMassAndInertia':
                import math
                kind,size=self.geometry[args[0]]
                volume=math.pi*(size[0]/2)**2*size[2] if kind=='primitiveshape_cylinder' else math.prod(size)
                self.masses[args[0]]=volume*args[1]
            return value
        return call


def test_readback_rejects_edited_wall_instead_of_trusting_scene_metadata():
    s=GeometrySim(); cfg=SceneConfig(); build_scene(s,cfg)
    result=live.readback(s,cfg)
    assert result['geometry_readback_verified']
    assert len(result['geometry'])==33
    h=s.names['wall_00']
    s.geometry[h]=(s.geometry[h][0],[.006,.001,.02])
    with pytest.raises(RuntimeError,match='readback mismatch'):
        live.readback(s,cfg)


def test_readback_rejects_sensor_lever_arm_and_motor_limit_changes():
    s=GeometrySim(); cfg=SceneConfig(); build_scene(s,cfg)
    s.positions[s.names['wrist_sensor']]=[.4,0.,.25]
    with pytest.raises(RuntimeError,match='sensor'):
        live.readback(s,cfg)
    s.positions[s.names['wrist_sensor']]=[.4,0.,.26]
    s.forces[s.names['drive_z']]=100.
    with pytest.raises(RuntimeError,match='motor'):
        live.readback(s,cfg)


class TraceBackend:
    """Deterministic data playback; it does not integrate any physical equations."""
    dt=.02
    def __init__(self, rows):
        self.rows=iter(deepcopy(rows)); self.commands=[]; self.closed=False
    def start(self): pass
    def advance(self): return next(self.rows)
    def command(self,v): self.commands.append(tuple(v))
    def close(self): self.closed=True


def test_direct_capture_keeps_sampling_after_trigger_and_blocks_challenge(tmp_path):
    cfg,meta,rows=fixture('overload',count=100)
    rows=trial_rows(rows)
    for i,r in enumerate(rows):
        r['pose'][2]=.197
        # A real force sensor would not use this manufactured profile.
        loaded(r,[0.,0.,50. if i==15 else 2.],contact(cfg))
    b=TraceBackend(rows)
    path=tmp_path/'samples.jsonl'
    with path.open('x') as f:
        result=live.capture_direct(b,cfg,'overload',1,f,Limits(),duration_s=2.)
    recorded=[json.loads(s) for s in path.read_text().splitlines()]
    assert result['complete']
    assert len(recorded)>=65  # trigger plus >= 1 second at 20 ms
    assert any(r['stop_latched'] and r['reaction_at_tip'][2]<45. for r in recorded)
    trigger=next(i for i,r in enumerate(recorded) if r['stop_latched'])
    assert all(not any(r['command_applied_mps']) for r in recorded[trigger+1:])
    assert any(any(r['command_requested_mps']) for r in recorded[trigger+1:])
    assert b.closed


def test_capture_preserves_partial_rows_and_closes_on_transport_failure(tmp_path):
    cfg,meta,rows=fixture(count=3)
    rows=trial_rows(rows)
    b=TraceBackend(rows)
    with (tmp_path/'samples.jsonl').open('x') as f:
        result=live.capture_direct(b,cfg,'normal',1,f,Limits(),duration_s=2.)
    assert not result['complete'] and 'StopIteration' in result['error']
    assert b.closed and b.commands[-1]==(0.,0.,0.)


def test_case_profile_inserts_before_lateral_motion():
    cfg=case_config(SceneConfig(),'sidewall')
    assert live.requested_velocity('sidewall',[.4,0.,.201],cfg)==(0.,0.,-.001)
    assert live.requested_velocity('sidewall',[.4,0.,.198],cfg)==(.0002,0.,0.)


def test_hash_manifest_detects_tampering_and_missing_files(tmp_path):
    (tmp_path/'samples.jsonl').write_text('original\n')
    live.seal(tmp_path)
    assert live.verify_seal(tmp_path)==[]
    (tmp_path/'samples.jsonl').write_text('edited\n')
    assert live.verify_seal(tmp_path)
    (tmp_path/'samples.jsonl').unlink()
    assert live.verify_seal(tmp_path)


def calibration_bundle(axis):
    case='jam' if axis=='axial' else 'sidewall'
    cfg,meta,rows=fixture(case)
    cfg=replace(cfg,motor_force_n=5.)
    meta['config']=config_dict(cfg)
    meta['calibration_trial']=axis
    for r in trial_rows(rows):
        if axis=='axial':
            r['pose'][2]=.197
            loaded(r,[0.,0.,3.],contact(cfg))
        else:
            r['pose'][:3]=[.4002,0.,.198]
            r['sensor_matrix'][3]=.4002
            r['sensor_matrix'][11]=.228
            loaded(r,[-3.,0.,0.],contact(cfg,'wall_00',[.4052,0.,.199],[1.,0.,0.]),(.0002,0.,0.))
            r['raw_torque']=[0.,.087,0.]  # moment about wrist, 29 mm below it
    return meta,rows


def test_low_force_calibration_requires_two_axes_and_opposing_reactions():
    am,ar=calibration_bundle('axial'); lm,lr=calibration_bundle('lateral')
    result=live.calibrate(am,ar,lm,lr,1)
    assert result['status']=='PASS'
    assert live.calibrate(am,ar,lm,lr,-1)['status']=='FAIL'
    assert live.calibrate(am,ar,lm,[],1)['status']=='INSUFFICIENT'
    lm['config']['motor_force_n']=25.
    assert live.calibrate(am,ar,lm,lr,1)['status']=='FAIL'


def test_calibration_rejects_engine_change_and_bad_moment_origin():
    am,ar=calibration_bundle('axial'); lm,lr=calibration_bundle('lateral')
    lm['engine']['revision']=2
    assert live.calibrate(am,ar,lm,lr,1)['status']!='PASS'
    lm['engine']['revision']=1
    for r in lr:
        r['raw_torque']=[0.,1.,0.]
    assert live.calibrate(am,ar,lm,lr,1)['status']=='FAIL'


def test_calibration_replay_requires_live_build_provenance():
    am,ar=calibration_bundle('axial'); lm,lr=calibration_bundle('lateral')
    am['source']='synthetic_fixture_no_engine'
    del am['geometry_readback_verified']
    assert live.calibrate(am,ar,lm,lr,1)['status']=='INSUFFICIENT'
