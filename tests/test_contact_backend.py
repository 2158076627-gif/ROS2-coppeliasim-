"""Recording transport tests; the double does NOT calculate contact physics."""
from unittest.mock import patch
import pytest
from coppeliasim_contact.backend import CoppeliaBackend, reaction
from coppeliasim_contact.contracts import SceneConfig
from coppeliasim_contact.scene import build_scene


class RecordingSim:
    handle_scene=-1; object_shape_type=0; simulation_stopped=0; handle_world=-1
    def __init__(self):
        self.calls=[]; self.shapes=[]; self.values={}; self.counter=100; self.time=0.
        self.pose=[.4,0,.23,0,0,0,1]; self.valid=1
    def __getattr__(self, name):
        if name.startswith(('shape','primitive','joint','ode_','intparam','floatparam','arrayparam','physics_')):
            return name
        def call(*args):
            self.calls.append((name,args))
            if name.startswith('create'):
                self.counter+=1
                if name=='createPrimitiveShape': self.shapes.append(self.counter)
                return self.counter
            if name=='getSimulationState': return 0
            if name=='getObjectsInTree': return []
            if name=='setEngineFloatParam': self.values[args[:2]]=args[2]
            if name=='getEngineFloatParam': return self.values[args[:2]]
            if name=='getSimulationTimeStep': return .02
            if name=='getInt32Param': return 'physics_ode' if args[0]=='intparam_dynamic_engine' else 40700
            if name=='getArrayParam': return [0,0,0]
            if name=='writeCustomDataBlock': self.metadata=args[2]
            if name=='readCustomDataBlock': return self.metadata
            if name=='getObject': return args[0]
            if name=='getSimulationTime': return self.time
            if name=='step': self.time+=.02
            if name=='getObjectPose': return self.pose
            if name=='readForceSensor': return self.valid,[0,0,2],[0,0,0]
            if name=='getObjectMatrix': return [1,0,0,.4,0,1,0,0,0,0,1,.26]
            if name=='getContactInfo': return None
        return call


def backend(sim):
    build_scene(sim,SceneConfig())
    with patch('coppeliasim_contact.backend.connect',return_value=(object(),sim)):
        return CoppeliaBackend()


def test_builder_dynamic_chain_force_sensor_and_material_readback():
    s=RecordingSim()
    metadata=build_scene(s,SceneConfig(stop_depth_m=.003))
    assert metadata['robot_model']=='3P Cartesian rig'
    assert len([a for n,a in s.calls if n=='createJoint'])==3
    assert len([a for n,a in s.calls if n=='createForceSensor'])==1
    assert len(s.shapes)==38  # 32 walls, stop, anchor, three carriages, peg
    assert len([a for n,a in s.calls if n=='computeMassAndInertia'])==38
    masks=[a[2] for n,a in s.calls if n=='setObjectInt32Param' and a[1]=='shapeintparam_respondable_mask']
    assert set(masks)=={65535}
    s.getSimulationState=lambda: 1
    with pytest.raises(RuntimeError): build_scene(s,SceneConfig())


def test_stepping_feedback_invalid_sensor_and_reset():
    s=RecordingSim(); b=backend(s); b.start()
    b.command((0,0,-.001))
    sample=b.advance()
    assert sample['simulation_time_s']==pytest.approx(.02)
    assert sample['contacts']==[]
    assert reaction(sample,1)==pytest.approx((0,0,2,0,0,0))
    assert reaction(sample,0) is None
    s.valid=0
    assert reaction(b.advance(),1) is None
    s.time=-1
    with pytest.raises(RuntimeError,match='reset'): b.advance()
    b.close()
    assert ('stopSimulation',()) in s.calls
    assert [a[1] for n,a in s.calls if n=='setJointTargetVelocity'][-3:]==[0,0,0]


def test_contact_identity_and_pose_fault():
    s=RecordingSim(); b=backend(s); b.start()
    s.getContactInfo=lambda _,peg,i: ([123,456],[.4,0,.2],[0,0,10],[0,0,1]) if i==0 else ([],[],[],[])
    sample=b.advance()
    assert sample['contacts'][0]['objects']==[123,456]
    s.pose=[.4,0,.23,.001,0,0,1]
    with pytest.raises(RuntimeError): b.advance()
