"""Synthetic evaluator fixtures: these tests never establish engine validity."""
from copy import deepcopy
import math

import pytest

from coppeliasim_contact import acceptance as a
from coppeliasim_contact.contracts import SceneConfig, config_dict
from coppeliasim_contact.experiments import case_config


def fixture(case='normal', count=61):
    cfg = case_config(SceneConfig(), case)
    objects = {'1': 'peg', **{str(10+i): f'wall_{i:02d}' for i in range(cfg.wall_segments)}}
    if cfg.stop_depth_m is not None:
        objects['50'] = 'physical_stop'
    meta = dict(schema=1, case=case, mode='direct', source='coppeliasim_physics',
                config=config_dict(cfg), complete=True, objects=objects,
                engine={'version': 40700, 'revision': 1, 'engine': 'ODE', 'dt_s': .02,
                        'dynamics_dt_s': .002}, scene_sha256='a'*64, code={'sha': 'b'*40},
                build_reload_verified=True, geometry_readback_verified=True,
                calibration={'status': 'PASS', 'sign': 1, 'signature': a.calibration_signature(cfg),
                             'axial_sha256': 'c'*64, 'lateral_sha256': 'd'*64},
                baseline_force_n=0., limits=a.Limits().__dict__)
    meta['calibration']['engine']=deepcopy(meta['engine'])
    rows = []
    def append(tip,phase,command):
        i=len(rows)
        rows.append(dict(simulation_time_s=(i+1)*.02, pose=tip+[0., 0., 0., 1.],phase=phase,
                         sensor_matrix=[1.,0.,0.,.4,0.,1.,0.,0.,0.,0.,1.,tip[2]+.03],
                         raw_force=[0.,0.,0.], raw_torque=[0.,0.,0.], sensor_valid=True,
                         reaction_at_tip=[0.]*6, contacts=[], contact_list_truncated=False,
                         command_applied_mps=list(command), command_requested_mps=list(command),
                         command_after_sample_mps=list(command), stop_latched=False,
                         wall_stamp_ns=1_000_000_000+i*20_000_000))
    for _ in range(10): append([.4,0.,.23],'settle',(0.,0.,0.))
    target={'normal':.196,'offset':.2,'sidewall':.198,'jam':.197,'overload':.197}[case]
    steps=round((.23-target)/.00002)
    for i in range(steps): append([.4,0.,.23-(i+1)*.00002],'approach',(0.,0.,-.001))
    for _ in range(count): append([.4,0.,target],'trial',(0.,0.,0.))
    return cfg, meta, rows


def trial_rows(rows):
    return [r for r in rows if r['phase']=='trial']


def loaded(row, force, contact, command=(0.,0.,-.001)):
    row.update(raw_force=list(force), reaction_at_tip=list(force)+[0.,0.,0.],
               contacts=[contact], command_applied_mps=list(command),
               command_requested_mps=list(command), command_after_sample_mps=list(command))


def contact(cfg, role='physical_stop', point=None, normal=None):
    if point is None:
        point = [.4, 0., cfg.hole_center_m[2]-cfg.stop_depth_m]
    return dict(objects=[1, 50 if role=='physical_stop' else 10], point_world_m=point,
                reaction_world_n=[0.,0.,15.], normal_world=normal or [0.,0.,1.])


def test_no_rows_and_legacy_flags_are_insufficient():
    cfg, meta, rows = fixture()
    assert a.evaluate([], meta)['status']=='INSUFFICIENT'
    del meta['geometry_readback_verified']
    assert a.evaluate(rows, meta)['status']=='INSUFFICIENT'


def test_normal_requires_depth_valid_sensor_and_no_penetration():
    cfg, meta, rows = fixture()
    assert a.evaluate(rows, meta)['status']=='PASS'
    for r in trial_rows(rows):
        r['pose'][2]=.201
    assert a.evaluate(rows, meta)['status']=='FAIL'
    cfg,meta,rows=fixture()
    rows[-1]['sensor_valid']=False
    assert a.evaluate(rows, meta)['status']=='INSUFFICIENT'


def test_cylinder_overlap_checks_finite_wall_extent_and_rim():
    cfg=SceneConfig()
    # Axis above the entrance is not intersecting any finite cuboid.
    assert a.geometric_overlap([.4004,0.,.201],cfg)==pytest.approx(0.)
    assert a.geometric_overlap([.4,0.,.196],cfg)==pytest.approx(0.)
    assert a.geometric_overlap([.4004,0.,.196],cfg)==pytest.approx(.0002)
    assert a.geometric_overlap([.4004,0.,.19995],cfg)==pytest.approx(.00005)
    # Well beyond every finite wall must not be reported as penetrating a halfspace.
    assert a.geometric_overlap([.5,0.,.196],cfg)==pytest.approx(0.)


def test_contact_identity_and_point_are_both_required():
    cfg, meta, rows=fixture('sidewall')
    row=rows[0]
    row['pose'][:3]=[.4002,0.,.198]
    p=[.4052,0.,.199]
    c=contact(cfg,'wall_00',p,[1.,0.,0.])
    assert a.contact_region(c,row,cfg,meta['objects'],a.Limits())=='sidewall'
    c['point_world_m']=[.4052,0.,.2]
    assert a.contact_region(c,row,cfg,meta['objects'],a.Limits())=='rim'
    c['objects']=[1,999]
    assert a.contact_region(c,row,cfg,meta['objects'],a.Limits())=='unexpected'
    c['objects']=[1,10]; c['point_world_m']=[.4,0.,.15]
    assert a.contact_region(c,row,cfg,meta['objects'],a.Limits())=='unresolved'


def test_offset_entry_collision_is_not_sidewall_acceptance():
    cfg, meta, rows=fixture('offset')
    for r in trial_rows(rows):
        r['pose'][2]=.2
        loaded(r,[0.,0.,15.],contact(cfg,'wall_00',[.405,0.,.2]))
        # Positive-side wall moves with the offset.
        r['contacts'][0]['objects']=[1,26]
        r['contacts'][0]['point_world_m']=[.3952,0.,.2]
    assert a.evaluate(rows,meta)['status']=='PASS'
    meta['case']='sidewall'; meta['config']=config_dict(case_config(SceneConfig(),'sidewall'))
    assert a.evaluate(rows,meta)['status']=='FAIL'


def test_jam_needs_continuous_contact_force_command_and_bounded_travel():
    cfg, meta, rows=fixture('jam')
    for r in trial_rows(rows):
        r['pose'][2]=.197
        loaded(r,[0.,0.,15.],contact(cfg))
    result=a.evaluate(rows,meta)
    assert result['status']=='PASS'
    assert result['metrics']['jam_duration_s']>=.5
    for i,r in enumerate(trial_rows(rows)):
        if i%10==0:
            r['contacts']=[]
    assert a.evaluate(rows,meta)['status']=='FAIL'


def test_gap_and_truncated_contacts_are_insufficient():
    cfg, meta, rows=fixture()
    rows[30]['contact_list_truncated']=True
    assert a.evaluate(rows,meta)['status']=='INSUFFICIENT'
    rows[30]['contact_list_truncated']=False
    rows[30]['simulation_time_s']+=.005
    assert a.evaluate(rows,meta)['status']=='INSUFFICIENT'


def overload_fixture():
    cfg,meta,rows=fixture('overload')
    for i,r in enumerate(trial_rows(rows)):
        r['pose'][2]=.197
        loaded(r,[0.,0.,50. if i>=5 else 2.],contact(cfg))
        if i>=5:
            r['stop_latched']=True
            r['command_after_sample_mps']=[0.]*3
        if i>5:
            r['command_applied_mps']=[0.]*3
    return cfg,meta,rows


def test_overload_requires_post_trigger_hold_and_latched_zero_on_new_commands():
    cfg,meta,rows=overload_fixture()
    assert a.evaluate(rows,meta)['status']=='PASS'
    assert a.evaluate(rows[:len(rows)-55],meta)['status']=='INSUFFICIENT'
    rows[-1]['stop_latched']=False
    assert a.evaluate(rows,meta)['status']=='FAIL'


def test_stop_distance_includes_rebound_and_not_just_last_displacement():
    cfg,meta,rows=overload_fixture()
    trial_rows(rows)[10]['pose'][2]+=.0003
    result=a.evaluate(rows,meta)
    assert result['status']=='FAIL'
    assert result['metrics']['stop_distance_m']==pytest.approx(.0003)


def test_missing_post_stop_challenge_is_insufficient_not_a_failed_stop():
    cfg,meta,rows=overload_fixture()
    trigger=next(i for i,r in enumerate(rows) if r['stop_latched'])
    for r in rows[trigger+1:]:
        r['command_requested_mps']=[0.]*3
    assert a.evaluate(rows,meta)['status']=='INSUFFICIENT'


def test_nonfinite_invalid_calibration_and_mismatched_case_never_pass():
    cfg,meta,rows=fixture()
    rows[20]['raw_force'][2]=math.nan
    assert a.evaluate(rows,meta)['status']=='INSUFFICIENT'
    rows[20]['raw_force'][2]=0.
    meta['calibration']['signature']='wrong'
    assert a.evaluate(rows,meta)['status']=='INSUFFICIENT'
    meta['config']['hole_offset_x_m']=.0004
    assert a.evaluate(rows,meta)['status']!='PASS'


def test_a_known_safety_failure_takes_precedence_over_missing_provenance():
    cfg,meta,rows=fixture()
    del meta['scene_sha256']
    for r in trial_rows(rows):
        r['pose'][0]=.4004
    assert a.evaluate(rows,meta)['status']=='FAIL'


def test_calibration_is_bound_to_material_steps_and_sensor_geometry():
    cfg=SceneConfig()
    assert a.calibration_signature(cfg)==a.calibration_signature(case_config(cfg,'jam'))
    changed=config_dict(cfg); changed['friction']=.4
    assert a.calibration_signature(cfg)!=a.calibration_signature(SceneConfig(**changed))


def test_thresholds_reject_nan_zero_or_unphysical_time_windows():
    with pytest.raises(ValueError): a.Limits(penetration_m=math.nan)
    with pytest.raises(ValueError): a.Limits(stop_hold_s=0.)


def test_missing_free_baseline_and_changed_engine_make_evidence_insufficient():
    cfg,meta,rows=fixture()
    meta['baseline_force_n']=None
    assert a.evaluate(rows,meta)['status']=='INSUFFICIENT'
    meta['baseline_force_n']=0.
    meta['engine']['revision']=2
    assert a.evaluate(rows,meta)['status']=='INSUFFICIENT'


def test_transient_contact_between_outer_steps_cannot_pass_as_free_motion():
    cfg,meta,rows=fixture()
    rows[-1]['contacts_all_substeps']=[contact(cfg,'wall_00',[.4052,0.,.2])]
    assert a.evaluate(rows,meta)['status']!='PASS'


def test_missing_run_start_or_a_first_step_jump_cannot_pass():
    cfg,meta,rows=fixture()
    assert a.evaluate(rows[20:],meta)['status']=='INSUFFICIENT'
    rows[0]['pose'][2]=.196
    assert a.evaluate(rows,meta)['status']!='PASS'
