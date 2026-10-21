"""Offline physical acceptance. PASS is conditional on the attached raw evidence.

Nothing here synthesizes forces or establishes the authenticity of an engine log.
Geometric overlap is sampled cylinder/cuboid overlap, NOT solver penetration depth.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

from .contracts import SceneConfig, config_dict, finite, wrench_at_tip
from .experiments import CASES, assess, case_config


@dataclass(frozen=True)
class Limits:
    target_depth_m: float = .004
    depth_tolerance_m: float = .0002
    penetration_m: float = .0001
    contact_tolerance_m: float = .0002
    rim_band_m: float = .0002
    normal_force_n: float = 5.
    contact_force_n: float = .5
    jam_force_n: float = 12.
    jam_duration_s: float = .5
    jam_travel_m: float = .00005
    overload_force_n: float = 45.
    stop_hold_s: float = 1.
    stop_distance_m: float = .0002
    settle_window_s: float = .5
    settle_travel_m: float = .00002

    def __post_init__(self):
        if not finite(asdict(self).values()) or min(asdict(self).values()) <= 0:
            raise ValueError('acceptance limits must be finite and positive')
        if self.stop_hold_s < self.settle_window_s or self.depth_tolerance_m >= self.target_depth_m:
            raise ValueError('invalid acceptance windows')
        if not self.contact_force_n < self.jam_force_n < self.overload_force_n:
            raise ValueError('invalid force thresholds')


def calibration_signature(cfg):
    """Bind calibration to geometry, sensor lever arm and numerical/material settings.

    Motor force and physical interventions can change after low-force calibration.
    """
    values = config_dict(cfg)
    for key in ('motor_force_n', 'stop_depth_m', 'hole_offset_x_m'):
        values.pop(key)
    return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()


def obstacles(cfg):
    for i, (size, position, yaw) in enumerate(cfg.walls()):
        yield f'wall_{i:02d}', size, position, yaw
    if cfg.stop_depth_m is not None:
        x, y, z = cfg.hole_center_m
        yield 'physical_stop', [.03,.03,.002], [x+cfg.hole_offset_x_m,y,z-cfg.stop_depth_m-.001], 0.


def local_xy(point, center, yaw):
    x, y = point[0]-center[0], point[1]-center[1]
    return math.cos(yaw)*x+math.sin(yaw)*y, -math.sin(yaw)*x+math.cos(yaw)*y


def geometric_overlap(tip, cfg):
    """Smallest axial/radial separating translation for each finite box, in m.

    Valid only for this fixed-attitude circular cylinder and readback-verified boxes.
    Outside an obstacle's finite extents this correctly returns zero. Samples can
    miss inter-step peaks; this is not a bound on sub-step solver penetration.
    """
    peak = 0.
    for _, size, center, yaw in obstacles(cfg):
        z_overlap = min(tip[2]+cfg.peg_length_m-(center[2]-size[2]/2),
                        center[2]+size[2]/2-tip[2])
        if z_overlap <= 0:
            continue
        x, y = local_xy(tip, center, yaw)
        qx, qy = abs(x)-size[0]/2, abs(y)-size[1]/2
        signed_distance = math.hypot(max(qx,0.),max(qy,0.))+min(max(qx,qy),0.)
        peak = max(peak, min(z_overlap, max(0.,cfg.peg_radius_m-signed_distance)))
    return peak


def contact_region(c, row, cfg, objects, limits):
    """Classify ONLY a verified peg/obstacle pair and a point on both primitives."""
    try:
        pair = [objects.get(str(h)) for h in c['objects']]
        if len(pair)!=2 or 'peg' not in pair:
            return 'unexpected'
        other = pair[1] if pair[0]=='peg' else pair[0]
        boxes = {name: (size,center,yaw) for name,size,center,yaw in obstacles(cfg)}
        if other not in boxes:
            return 'unexpected'
        point, normal, force = c['point_world_m'], c['normal_world'], c['reaction_world_n']
        if any(len(v)!=3 or not finite(v) for v in (point,normal,force)):
            return 'unresolved'
        normal_norm=math.hypot(*normal)
        if not .5 <= normal_norm <= 1.5:
            return 'unresolved'
        tol = limits.contact_tolerance_m
        tip=row['pose'][:3]
        radial=math.hypot(point[0]-tip[0],point[1]-tip[1])
        height=point[2]-tip[2]
        if radial>cfg.peg_radius_m+tol+1e-12 or not -tol <= height <= cfg.peg_length_m+tol:
            return 'unresolved'
        if min(abs(radial-cfg.peg_radius_m),abs(height),abs(height-cfg.peg_length_m))>tol+1e-12:
            return 'unresolved'
        size,center,yaw=boxes[other]
        x,y=local_xy(point,center,yaw)
        z=point[2]-center[2]
        offsets=[abs(x)-size[0]/2, abs(y)-size[1]/2, abs(z)-size[2]/2]
        if max(offsets)>tol+1e-12 or min(abs(v) for v in offsets)>tol+1e-12:
            return 'unresolved'
        axial=abs(normal[2])/normal_norm
        if other=='physical_stop':
            return 'stop' if abs(z-size[2]/2)<=tol and axial>=.7 else 'unresolved'
        top=cfg.hole_center_m[2]
        if abs(point[2]-top)<=limits.rim_band_m+1e-12:
            return 'rim'
        normal_radial=abs(normal[0]*math.cos(yaw)+normal[1]*math.sin(yaw))/normal_norm
        if (point[2]<top-limits.rim_band_m and abs(x+size[0]/2)<=tol
                and normal_radial>=.7):
            return 'sidewall'
        return 'unresolved'
    except (KeyError,TypeError,ValueError,IndexError):
        return 'unresolved'


def span(points):
    return max((math.dist(p,q) for p in points for q in points),default=0.)


def jam_duration(rows, regions, limits, dt):
    """Continuous loaded/blocker intervals with total 3D travel within the limit."""
    window=[]
    longest=0.
    for r,region in zip(rows,regions):
        good=('stop' in region and math.hypot(*r['reaction_at_tip'][:3])>=limits.jam_force_n
              and r['command_applied_mps'][2]<0 and not r['stop_latched'])
        if not good or (window and not math.isclose(r['simulation_time_s']-window[-1]['simulation_time_s'],dt,abs_tol=1e-7)):
            window=[]
        if not good:
            continue
        window.append(r)
        # The maximum pairwise excursion, not just first-to-last displacement.
        while len(window)>1 and any(math.dist(window[-1]['pose'][:3],p['pose'][:3])>limits.jam_travel_m+1e-12 for p in window[:-1]):
            window.pop(0)
        longest=max(longest,r['simulation_time_s']-window[0]['simulation_time_s'])
    return longest


def startup_evidence(rows, cfg, sign):
    """Validate trace origin and derive the stationary baseline from raw samples."""
    first=next((r for r in rows if isinstance(r,dict)),None)
    starts=bool(first and math.isclose(first.get('simulation_time_s',-1.),cfg.simulation_dt_s,abs_tol=1e-7))
    initial_ok=math.dist(first['pose'][:3],cfg.initial_tip_m)<=.0001 if starts else None
    values=[]
    for r in rows:
        if not isinstance(r,dict):
            continue
        try:
            if (not 0<r['simulation_time_s']<=.2+1e-9 or not r['sensor_valid']
                    or math.dist(r['pose'][:3],cfg.initial_tip_m)>.0001
                    or math.hypot(*r['command_applied_mps'])>1e-12
                    or r.get('contacts_all_substeps',r['contacts'])):
                continue
            wrench=wrench_at_tip(r['sensor_matrix'],r['raw_force'],r['raw_torque'],r['pose'][:3],sign)
            values.append((r['simulation_time_s'],math.hypot(*wrench[:3])))
        except (KeyError,ValueError,TypeError):
            continue
    contiguous=all(math.isclose(b[0]-a[0],cfg.simulation_dt_s,abs_tol=1e-7) for a,b in zip(values,values[1:]))
    baseline=max((v[1] for v in values),default=None) if len(values)>=5 and contiguous else None
    return {'trace_start':starts,'initial_pose':initial_ok,'baseline_force_n':baseline,'baseline_samples':len(values)}


def _evaluate(rows, meta):
    """Return PASS / FAIL / INSUFFICIENT, reasons, metrics and existing coarse flags.

    Known safety failures dominate missing provenance. Missing/invalid samples,
    calibration, truncated contact lists or incomplete post-stop logs cannot pass.
    """
    checks={}
    metrics={}
    def check(name, value, reason):
        checks[name]={'status':'INSUFFICIENT' if value is None else ('PASS' if value else 'FAIL'), 'reason':reason}
    def result():
        states=[x['status'] for x in checks.values()]
        status='FAIL' if 'FAIL' in states else ('INSUFFICIENT' if 'INSUFFICIENT' in states else 'PASS')
        return {'schema':1,'case':meta.get('case'),'mode':meta.get('mode'),'status':status,
                'checks':checks,'metrics':metrics,
                'scope':'3P simulated contact only; no six-axis or real-robot verification'}
    if meta.get('calibration_trial'):
        check('calibration_trial',None,'uncalibrated candidate capture is not a physical acceptance trial')
        return result()
    try:
        cfg=SceneConfig(**meta['config'])
        limits=Limits(**meta.get('limits',{}))
        case=meta['case']
        if case not in CASES:
            raise ValueError('case')
    except (KeyError,ValueError,TypeError) as exc:
        check('configuration',None,f'invalid/missing configuration: {exc}')
        return result()
    expected=case_config(cfg,case)
    check('intervention',config_dict(cfg)==config_dict(expected),'loaded configuration must match existing case_config')
    calibration=meta.get('calibration',{})
    calibrated=(calibration.get('status')=='PASS' and calibration.get('sign') in (-1,1)
                and calibration.get('signature')==calibration_signature(cfg)
                and all(len(calibration.get(k,''))==64 for k in ('axial_sha256','lateral_sha256')))
    check('calibration',True if calibrated else None,'need verified axial/lateral raw logs and matching calibration signature')
    engine=meta.get('engine',{})
    check('calibration_engine',True if calibration.get('engine')==engine and engine else None,
          'force sign calibration must match engine version, revision and step settings')
    provenance=(meta.get('source')=='coppeliasim_physics' and meta.get('mode') in ('direct','ros')
                and meta.get('build_reload_verified') is True and meta.get('geometry_readback_verified') is True
                and len(meta.get('scene_sha256',''))==64 and len(meta.get('code',{}).get('sha',''))==40
                and engine.get('version') and engine.get('revision') is not None and engine.get('engine')=='ODE'
                and engine.get('dt_s')==cfg.simulation_dt_s and engine.get('dynamics_dt_s')==cfg.dynamics_dt_s
                and meta.get('objects'))
    check('provenance',True if provenance else None,'need build/save/reload readback, object map, scene hash, engine and code versions')
    check('run_complete',True if meta.get('complete') is True and not meta.get('error') else None,
          meta.get('error') or 'acquisition must finish without exception')
    good=[]
    malformed=0
    for r in rows:
        if not isinstance(r,dict):
            malformed+=1
            continue
        if (r.get('phase')=='settle' and 0<r.get('simulation_time_s',math.inf)<=.2+1e-9
                and not r.get('sensor_valid') and not r.get('contacts')
                and r.get('command_applied_mps') in ([0.,0.,0.],(0.,0.,0.))):
            # Keep these raw startup rows in the artifact, but do not turn missing
            # startup force into measured zero or count it toward any dwell.
            continue
        try:
            required=(('pose',7),('raw_force',3),('raw_torque',3),('sensor_matrix',12),
                      ('reaction_at_tip',6),('command_applied_mps',3),('command_requested_mps',3),
                      ('command_after_sample_mps',3))
            if any(len(r[k])!=n or not finite(r[k]) for k,n in required):
                raise ValueError('invalid vector')
            if not finite([r['simulation_time_s']]) or not r['sensor_valid']:
                raise ValueError('invalid time/sensor')
            if abs(sum(v*v for v in r['pose'][3:])-1.)>1e-6 or sum(abs(v) for v in r['pose'][3:6])>1e-6:
                raise ValueError('orientation is outside fixed-attitude geometry contract')
            # Recompute the wrench: an edited derived value cannot bypass raw data.
            sign=calibration.get('sign',1)
            recomputed=wrench_at_tip(r['sensor_matrix'],r['raw_force'],r['raw_torque'],r['pose'][:3],sign)
            if not all(math.isclose(x,y,abs_tol=1e-7,rel_tol=1e-6) for x,y in zip(recomputed,r['reaction_at_tip'])):
                raise ValueError('wrench transform does not match raw data')
            if not isinstance(r['contacts'],list) or not isinstance(r['stop_latched'],bool):
                raise ValueError('missing contacts/latch')
            good.append(r)
        except (KeyError,TypeError,ValueError):
            malformed+=1
    check('raw_samples',True if good and not malformed else None,f'{len(good)} usable samples; {malformed} invalid/missing samples')
    if not good:
        return result()
    startup=startup_evidence(rows,cfg,calibration.get('sign',1))
    check('trace_start',True if startup['trace_start'] else None,'must capture from the first engine step, not a successful tail window')
    check('initial_pose',startup['initial_pose'],'first measured pose must agree with stopped-scene readback within 0.1 mm')
    baseline=startup['baseline_force_n']
    baseline_declared=meta.get('baseline_force_n')
    valid_baseline=(baseline is not None and isinstance(baseline_declared,(int,float))
                    and math.isfinite(baseline_declared) and math.isclose(baseline,baseline_declared,abs_tol=1e-7))
    check('free_motion_baseline',None if not valid_baseline else baseline<=1.,
          'derive >=5 stationary contact-free startup samples; compare to saved <=1 N baseline')
    check('sampling_continuity',True if all(math.isclose(b['simulation_time_s']-a['simulation_time_s'],cfg.simulation_dt_s,abs_tol=1e-7)
          for a,b in zip(good,good[1:])) else None,'strictly contiguous engine steps; gaps cannot count toward dwell')
    speed=.002 if meta.get('mode')=='direct' else .08
    check('dynamic_chain_continuity',all(math.dist(b['pose'][:3],a['pose'][:3])<=speed*cfg.simulation_dt_s+2*limits.penetration_m
          for a,b in zip(good,good[1:])),'reject discontinuous physical jumps beyond command speed and overlap allowance')
    check('contact_completeness',True if all(r.get('contact_list_truncated') is False for r in good) else None,'contact list must explicitly be untruncated')
    regions=[[contact_region(c,r,cfg,meta.get('objects',{}),limits) for c in r['contacts']] for r in good]
    events=[[contact_region(c,r,cfg,meta.get('objects',{}),limits)
             for c in r.get('contacts_all_substeps',r['contacts'])] for r in good]
    check('contact_identity',False if any('unexpected' in v for v in regions) else
          (None if any('unresolved' in v for v in regions) else True),
          'check peg + known obstacle handles, point on both shapes, and contact normal')
    check('substep_contact_identity',False if any('unexpected' in v for v in events) else
          (None if any('unresolved' in v for v in events) else True),
          'all recorded sub-step events must be explainable; point vs final-pose ambiguity remains insufficient')
    depths=[cfg.hole_center_m[2]-r['pose'][2] for r in good]
    forces=[math.hypot(*r['reaction_at_tip'][:3]) for r in good]
    overlaps=[geometric_overlap(r['pose'][:3],cfg) for r in good]
    metrics.update(samples=len(good),max_depth_m=max(depths),max_force_n=max(forces),
                   max_sampled_geometric_overlap_m=max(overlaps),
                   region_samples={k:sum(k in v for v in regions) for k in ('rim','sidewall','stop')},
                   jam_duration_s=jam_duration(good,regions,limits,cfg.simulation_dt_s))
    metrics['existing_flags']=assess(good,cfg)
    check('sampled_penetration',max(overlaps)<=limits.penetration_m+1e-12,
          'finite-cylinder/cuboid overlap at sample times only; not solver/sub-step depth')
    check('workspace',all(.3<=r['pose'][0]<=.5 and -.1<=r['pose'][1]<=.1 and .18<=r['pose'][2]<=.4 for r in good),
          'measured tip must stay within MotionGuard workspace')
    observed=not malformed and checks['sampling_continuity']['status']=='PASS' and meta.get('complete') is True
    def condition(name, flag, reason):
        check(name,flag if flag or observed else None,reason)
    if case!='overload':
        check('no_overload',not any(f>=limits.overload_force_n or r['stop_latched'] for f,r in zip(forces,good)),
              'unexpected overload/latch invalidates this trial')
    if case=='normal':
        condition('depth',max(depths)>=limits.target_depth_m-limits.depth_tolerance_m,'actual tip must reach target depth')
        check('normal_free_path',not any(events) and max(forces)<=limits.normal_force_n,
              'coaxial path has no verified collision and stays below the force ceiling')
    elif case=='offset':
        condition('entry_collision',any('rim' in v and r['reaction_at_tip'][2]>=limits.contact_force_n
                  for r,v in zip(good,regions)),'offset must produce verified rim contact with upward reaction')
    elif case=='sidewall':
        condition('sidewall_after_insertion',any('sidewall' in v and r['command_applied_mps'][0]>0
                  and r['reaction_at_tip'][0]<=-limits.contact_force_n and abs(d-.002)<=.0003
                  for r,v,d in zip(good,regions,depths)),'insert 2 mm first, then +X; below-rim contact and opposing Fx')
        check('no_entry_collision',not any('rim' in v for v in events),'rim-first or later rim collision is not clean sidewall evidence')
    elif case=='jam':
        condition('sustained_blocking',metrics['jam_duration_s']+1e-9>=limits.jam_duration_s,
                  'continuous stop contact, downward applied command, force >=12 N, total 3D travel <=50 um')
    elif case=='overload':
        trigger=next((i for i,f in enumerate(forces) if f>=limits.overload_force_n),None)
        condition('overload_trigger',trigger is not None,'actual measured force must reach threshold; untriggered is a failed case')
        if trigger is not None:
            tail=good[trigger:]
            origin=tail[0]['pose'][:3]
            distances=[math.dist(r['pose'][:3],origin) for r in tail]
            metrics.update(trigger_time_s=tail[0]['simulation_time_s'],stop_distance_m=max(distances),
                           detection_interval_travel_m=math.dist(good[max(0,trigger-1)]['pose'][:3],origin),
                           post_stop_duration_s=tail[-1]['simulation_time_s']-tail[0]['simulation_time_s'])
            check('contact_at_overload','stop' in regions[trigger],'threshold must occur against the physical stop')
            check('observed_threshold_crossing',True if trigger>0 and not any(r['stop_latched'] for r in good[:trigger]) else None,
                  'need a below-threshold preceding sample and no pre-existing latch')
            check('stop_latch',all(r['stop_latched'] for r in tail),'latch must assert on threshold sample and never clear')
            check('zero_after_trigger',all(math.hypot(*r['command_after_sample_mps'])<=1e-12 for r in tail)
                  and all(math.hypot(*r['command_applied_mps'])<=1e-12 for r in tail[1:]),
                  'immediate zero after detecting force; all later applied commands zero')
            check('stop_distance',max(distances)<=limits.stop_distance_m+1e-12,'maximum 3D excursion from trigger includes rebound')
            held=metrics['post_stop_duration_s']+1e-9>=limits.stop_hold_s
            check('post_stop_hold',True if held else None,'keep sampling after trigger for the configured hold duration')
            challenged=any(math.hypot(*r['command_requested_mps'])>0 for r in tail[1:])
            check('latch_challenge',True if challenged else None,
                  'a new nonzero request must remain blocked after latching')
            stable=[r['pose'][:3] for r in tail if tail[-1]['simulation_time_s']-r['simulation_time_s']<=limits.settle_window_s+1e-9]
            check('settled',span(stable)<=limits.settle_travel_m if held else None,
                  'terminal position excursion must stay inside the settle tolerance')
    if meta.get('mode')=='ros':
        topics=meta.get('ros_topics',{})
        check('ros_topic_record',True if all(topics.get(t,0)>0 for t in ROS_TOPICS) else None,
              'all required ROS topics must be recorded by the independent observer')
    return result()


def evaluate(rows, meta):
    try:
        if not isinstance(meta,dict) or not isinstance(rows,list):
            raise TypeError('manifest must be an object and samples a list')
        return _evaluate(rows,meta)
    except (KeyError,TypeError,ValueError,AttributeError,OverflowError) as exc:
        return dict(schema=1,status='INSUFFICIENT',case=meta.get('case') if isinstance(meta,dict) else None,
                    checks={'artifact_schema':{'status':'INSUFFICIENT','reason':f'{type(exc).__name__}: {exc}'}},metrics={},
                    scope='no physical or real-robot acceptance established')


ROS_TOPICS=('/simulation/cmd_twist','/simulation/tool_pose','/force_torque/wrench',
            '/simulation/raw_sensor_wrench','/simulation/contact_telemetry','/simulation/bridge_stop',
            '/assembly/compliant_twist','/assembly/contact_state','/assembly/emergency_stop',
            '/assembly/execution_state','/perception/axis_hole_pose','/assembly/trajectory_plan')
