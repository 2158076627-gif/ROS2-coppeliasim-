"""Repeatable build -> calibrate -> acquire -> offline report. No robot driver."""
import argparse
import csv
from dataclasses import asdict, replace
import importlib.metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shutil
import sys
import time
import uuid

from .acceptance import CASES, Limits, ROS_TOPICS, evaluate, geometric_overlap
from .acceptance_live import (calibrate, capture_direct, code_version, dump,
                              load_calibration, open_prepared, readback, read_run,
                              seal, sha256, validate_build, verify_seal)
from .contracts import SceneConfig, config_dict
from .experiments import case_config
from .scene import build_scene, connect


def environment():
    def version(name):
        try:
            return importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            return None
    return dict(python=sys.version,platform=platform.platform(),
                ros_distro=os.environ.get('ROS_DISTRO'),ros2_cli=shutil.which('ros2'),
                ros2_importable=importlib.util.find_spec('rclpy') is not None,
                jazzy_setup_exists=Path('/opt/ros/jazzy/setup.bash').is_file(),
                coppeliasim_executable=shutil.which('coppeliaSim') or shutil.which('coppeliaSim.sh'),
                zmq_client_version=version('coppeliasim-zmqremoteapi-client'),
                engine_test_executed=False)


def new_directory(path):
    path=Path(path).resolve()
    path.mkdir(parents=True,exist_ok=False)
    return path


def prepare(config, output):
    base=SceneConfig.load(config)
    output=new_directory(output)
    for case in CASES:
        dump(output/(case+'.json'),config_dict(case_config(base,case)))
    for name,case in (('axial','jam'),('lateral','sidewall')):
        dump(output/('calibration_'+name+'.json'),config_dict(replace(case_config(base,case),motor_force_n=5.)))
    dump(output/'limits.json',asdict(Limits()))


def build(config, output, host, port):
    output=new_directory(output)
    cfg=SceneConfig.load(config)
    dump(output/'config.json',config_dict(cfg))
    record=dict(schema=1,config=config_dict(cfg),code=code_version(),environment=environment(),
                complete=False,build_reload_verified=False)
    try:
        if record['code']['dirty']:
            raise RuntimeError('commit source changes before a reproducible live build')
        _,sim=connect(host,port)
        record['builder_metadata']=build_scene(sim,cfg)
        scene=output/'scene.ttt'
        sim.saveScene(str(scene))
        if not scene.is_file():
            raise RuntimeError('saved .ttt is not readable on client; use a shared/local filesystem')
        sim.loadScene(str(scene))
        loaded=json.loads(sim.readCustomDataBlock(sim.getObject('/contact_rig'),'contact_config'))
        if loaded['config']!=config_dict(cfg):
            # JSON tuples become lists: normalize both through JSON.
            if loaded['config']!=json.loads(json.dumps(config_dict(cfg))):
                raise RuntimeError('config changed after save/reload')
        record.update(readback(sim,cfg),scene_sha256=sha256(scene),
                      build_reload_verified=True,complete=True)
    except Exception as exc:
        record['error']=f'{type(exc).__name__}: {exc}'
    dump(output/'build.json',record)
    seal(output)
    return record


def acquire(args):
    output=new_directory(args.out)
    dump(output/'environment.json',environment())
    limits=Limits(**json.loads(Path(args.limits).read_text())) if args.limits else Limits()
    meta=dict(schema=1,case=args.case,mode='direct',source='coppeliasim_physics',complete=False,
              run_id=str(uuid.uuid4()),limits=asdict(limits),started_wall_ns=time.time_ns())
    backend=None
    try:
        shutil.copytree(args.build,output/'build')
        if args.calibration:
            shutil.copytree(args.calibration,output/'calibration')
        backend,evidence=open_prepared(output/'build',args.host,args.port,
                                       output/'calibration' if args.calibration else None)
        meta.update(evidence)
        cfg=backend.cfg
        expected=case_config(cfg,args.case)
        if args.calibration_trial:
            required_case='jam' if args.calibration_trial=='axial' else 'sidewall'
            if args.case!=required_case:
                raise ValueError('axial calibration uses jam; lateral calibration uses sidewall')
            expected=replace(expected,motor_force_n=5.)
            meta['calibration_trial']=args.calibration_trial
            sign=args.candidate_sign
            meta['candidate_sign']=sign
        else:
            sign=meta['calibration']['sign']
        if json.loads(json.dumps(config_dict(expected)))!=json.loads(json.dumps(config_dict(cfg))):
            raise ValueError('loaded scene does not match existing five-case intervention')
        with (output/'samples.jsonl').open('x') as log:
            meta.update(capture_direct(backend,cfg,args.case,sign,log,limits,args.duration))
        backend=None  # capture_direct already closed it
    except Exception as exc:
        meta.update(complete=False,error=f'{type(exc).__name__}: {exc}')
    finally:
        if backend is not None:
            try:
                backend.close()
            except Exception as exc:
                meta['cleanup_error']=f'{type(exc).__name__}: {exc}'
        (output/'samples.jsonl').touch(exist_ok=True)
        meta['finished_wall_ns']=time.time_ns()
        dump(output/'manifest.json',meta)
        seal(output)
    if args.calibration_trial:
        return {'status':'ACQUIRED_UNCALIBRATED' if meta['complete'] else 'INSUFFICIENT',
                'error':meta.get('error'),'output':str(output)}
    return report(output)


def make_calibration(axial, lateral, sign, output):
    output=new_directory(output)
    for name,source in (('axial',axial),('lateral',lateral)):
        errors=verify_seal(source)
        if errors:
            raise ValueError('; '.join(errors))
        shutil.copytree(source,output/name)
    am,ar=read_run(output/'axial'); lm,lr=read_run(output/'lateral')
    result=calibrate(am,ar,lm,lr,sign)
    # Both calibration captures must themselves come from verified builds.
    for name,m in (('axial',am),('lateral',lm)):
        try:
            validate_build(output/name/'build',m)
        except (OSError,TypeError,ValueError,KeyError) as exc:
            result.update(status='INSUFFICIENT')
            result['reasons'].append(name+f': missing/mismatched build evidence: {exc}')
    result.update(axial_sha256=sha256(output/'axial/samples.jsonl'),
                  lateral_sha256=sha256(output/'lateral/samples.jsonl'))
    dump(output/'calibration.json',result)
    seal(output)
    return result


def write_curves(directory, rows, cfg):
    data=[]
    for r in rows:
        try:
            f=r['reaction_at_tip']
            value=[r['simulation_time_s'],cfg.hole_center_m[2]-r['pose'][2],*f,
                   geometric_overlap(r['pose'][:3],cfg),int(r['stop_latched']),
                   *r['pose'][:3],r['pose'][0]-cfg.initial_tip_m[0],math.hypot(*f[:3])]
            if all(math.isfinite(v) for v in value):
                data.append(value)
        except (KeyError,TypeError,ValueError):
            continue
    with (directory/'curves.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['simulation_time_s','measured_depth_m','fx_n','fy_n','fz_n',
                         'tx_at_tip_nm','ty_at_tip_nm','tz_at_tip_nm',
                         'max_sampled_geometric_overlap_m','stop_latched',
                         'tip_x_m','tip_y_m','tip_z_m','lateral_displacement_x_m','force_norm_n'])
        writer.writerows(data)
    # Standard scientific plotting; chronological measured points, no smoothing.
    if not data:
        return
    try:
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt
    except ImportError:
        (directory/'plot_status.json').write_text(json.dumps({'status':'UNAVAILABLE','reason':'install report extra / matplotlib; CSV is complete'}))
        return
    with plt.rc_context({'font.family':'serif','font.size':10,'svg.fonttype':'none'}):
        fig,axes=plt.subplots(1,2,figsize=(10,3.8),layout='constrained')
        axes[0].plot([d[1]*1000 for d in data],[d[4] for d in data],color='#166b9b',lw=1.2)
        axes[0].set(xlabel='Measured insertion depth (mm)',ylabel='Axial reaction Fz (N)',title='Axial force–displacement')
        axes[1].plot([d[13]*1000 for d in data],[d[2] for d in data],color='#b35a27',lw=1.2)
        axes[1].set(xlabel='Measured lateral displacement X (mm)',ylabel='Lateral reaction Fx (N)',title='Lateral force–displacement')
        for axis in axes:
            axis.spines[['top','right']].set_visible(False)
            axis.grid(alpha=.15)
        fig.savefig(directory/'force_displacement.svg')
        plt.close(fig)
        trigger=next((i for i,d in enumerate(data) if d[9]),None)
        if trigger is not None:
            tail=data[trigger:]
            origin=tail[0]
            fig,axes=plt.subplots(3,1,figsize=(7,6),sharex=True,layout='constrained')
            t=[d[0]-origin[0] for d in tail]
            axes[0].plot(t,[d[14] for d in tail],color='#166b9b')
            axes[0].set_ylabel('Force norm (N)')
            axes[1].plot(t,[math.dist(d[10:13],origin[10:13])*1000 for d in tail],color='#b35a27')
            axes[1].set_ylabel('3D excursion (mm)')
            axes[2].step(t,[d[9] for d in tail],where='post',color='#324954')
            axes[2].set(xlabel='Time since first recorded stop latch (s)',ylabel='Stop latch',ylim=(-.1,1.1))
            fig.savefig(directory/'stop_response.svg')
            plt.close(fig)


def replay_ros_topics(directory, meta, rows):
    counts={}
    telemetry=[]
    previous=None
    for line in (directory/'topics.jsonl').read_text().splitlines():
        event=json.loads(line)
        if not isinstance(event,dict) or not isinstance(event.get('message'),dict):
            raise ValueError('invalid raw ROS event')
        stamp=event['received_monotonic_ns']
        if not isinstance(stamp,int) or stamp<=0 or (previous is not None and stamp<previous):
            raise ValueError('invalid/reversed ROS receive times')
        if not isinstance(event['received_wall_ns'],int) or event['received_wall_ns']<=0:
            raise ValueError('invalid ROS receive wall stamp')
        previous=stamp
        topic=event['topic']
        counts[topic]=counts.get(topic,0)+1
        if topic=='/simulation/contact_telemetry':
            decoded=json.loads(event['message']['data'])
            evidence=decoded['metadata']['acceptance_evidence']
            for key in ('config','engine','scene_sha256','objects'):
                if evidence[key]!=meta[key]:
                    raise ValueError('raw ROS telemetry/run evidence mismatch: '+key)
            if decoded['reaction_sign']!=meta['calibration']['sign']:
                raise ValueError('raw ROS telemetry calibration mismatch')
            decoded.update(kind='sample',received_wall_ns=event['received_wall_ns'],
                           received_monotonic_ns=stamp)
            telemetry.append(decoded)
    if counts!=meta.get('ros_topics') or telemetry!=rows:
        raise ValueError('ROS counters/sample stream differ from raw topic records')
    return counts


def report(directory, require_calibration_files=True):
    """Offline replay is conservative even for partial, modified or legacy logs."""
    directory=Path(directory)
    try:
        meta,rows=read_run(directory)
    except (OSError,ValueError) as exc:
        meta={'error':str(exc)}; rows=[]
    errors=verify_seal(directory)
    try:
        validate_build(directory/'build',meta)
    except (OSError,ValueError,KeyError,TypeError) as exc:
        errors.append(f'build: {exc}')
    if require_calibration_files:
        try:
            meta['calibration']=load_calibration(directory/'calibration')
            if meta['calibration']['engine']!=meta.get('engine'):
                raise ValueError('calibration engine differs from run engine')
        except (OSError,ValueError,KeyError,TypeError) as exc:
            meta['calibration']={}
            errors.append(f'calibration: {exc}')
    if meta.get('mode')=='ros':
        try:
            meta['ros_topics']=replay_ros_topics(directory,meta,rows)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            meta['ros_topics']={}
            errors.append(f'raw ROS topics: {exc}')
    result=evaluate(rows,meta)
    if errors:
        result['checks']['artifact_integrity']={'status':'INSUFFICIENT','reason':'; '.join(errors)}
        if result['status']!='FAIL':
            result['status']='INSUFFICIENT'
    derived=directory/'derived'
    derived.mkdir(exist_ok=True)
    # Derived results may be regenerated; raw manifests/samples are never edited.
    (derived/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    if 'config' in meta:
        try:
            write_curves(derived,[r for r in rows if isinstance(r,dict)],SceneConfig(**meta['config']))
        except (TypeError,ValueError):
            pass  # Invalid config is already INSUFFICIENT; never invent a curve.
    return result


def observe(args):
    """Read-only ROS observer. NEVER creates a second engine stepping owner."""
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseArray, TwistStamped, WrenchStamped
    from std_msgs.msg import Bool, String
    from rosidl_runtime_py.convert import message_to_ordereddict
    output=new_directory(args.out)
    shutil.copytree(args.build,output/'build')
    shutil.copytree(args.calibration,output/'calibration')
    calibration=load_calibration(output/'calibration')
    dump(output/'environment.json',environment())
    if args.params:
        shutil.copyfile(args.params,output/'physical.yaml')
    rclpy.init()
    node=rclpy.create_node('contact_acceptance_observer')
    meta=dict(schema=1,case=args.case,mode='ros',source='coppeliasim_physics',complete=False,
              run_id=str(uuid.uuid4()),limits=asdict(Limits()),calibration=calibration,ros_topics={})
    latest=None
    baseline=[]
    errors=[]
    with (output/'topics.jsonl').open('x') as topics, (output/'samples.jsonl').open('x') as samples:
        def received(msg,topic):
            nonlocal latest
            event=dict(topic=topic,received_wall_ns=time.time_ns(),received_monotonic_ns=time.monotonic_ns(),
                       message=message_to_ordereddict(msg))
            topics.write(json.dumps(event,allow_nan=False)+'\n'); topics.flush()
            meta['ros_topics'][topic]=meta['ros_topics'].get(topic,0)+1
            if topic=='/simulation/contact_telemetry':
                try:
                    row=json.loads(msg.data)
                    evidence=row.get('metadata',{}).get('acceptance_evidence')
                    if not evidence:
                        raise ValueError('bridge lacks acceptance_evidence; use acceptance_build/calibration parameters')
                    if latest is not None and evidence!=latest:
                        raise ValueError('evidence metadata changed mid-run')
                    if row.get('reaction_sign')!=calibration['sign']:
                        raise ValueError('bridge sign differs from calibration')
                    latest=evidence
                    row.update(kind='sample',received_wall_ns=event['received_wall_ns'],
                               received_monotonic_ns=event['received_monotonic_ns'])
                    samples.write(json.dumps(row,allow_nan=False)+'\n'); samples.flush()
                    if (row['simulation_time_s']<=.2+1e-9 and row.get('reaction_at_tip') is not None
                            and math.hypot(*row['command_applied_mps'])<=1e-12
                            and not row.get('contacts_all_substeps',row['contacts'])):
                        baseline.append(math.hypot(*row['reaction_at_tip'][:3]))
                except (KeyError,TypeError,ValueError) as exc:
                    errors.append(str(exc))
        kinds=[TwistStamped,PoseStamped,WrenchStamped,WrenchStamped,String,Bool,
               TwistStamped,Bool,Bool,String,PoseStamped,PoseArray]
        subscriptions=[node.create_subscription(kind,topic,lambda msg,t=topic:received(msg,t),100)
                       for topic,kind in zip(ROS_TOPICS,kinds)]
        try:
            deadline=time.monotonic()+args.duration
            while rclpy.ok() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            meta['complete']=True
        except Exception as exc:
            errors.append(f'{type(exc).__name__}: {exc}')
        finally:
            node.destroy_node(); rclpy.shutdown()
    if latest:
        # The config and current handle map come from the bridge's live readback.
        meta.update({k:v for k,v in latest.items() if k not in ('complete','mode','case','calibration','limits')})
    meta['baseline_force_n']=max(baseline) if baseline else None
    if errors:
        meta.update(complete=False,error='; '.join(dict.fromkeys(errors)))
    dump(output/'manifest.json',meta)
    seal(output)
    return report(output)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='action',required=True)
    pre=commands.add_parser('preflight'); pre.add_argument('--out',required=True)
    prep=commands.add_parser('prepare'); prep.add_argument('--config',required=True); prep.add_argument('--out',required=True)
    builder=commands.add_parser('build'); builder.add_argument('--config',required=True); builder.add_argument('--out',required=True)
    capture=commands.add_parser('capture'); capture.add_argument('--case',choices=CASES,required=True)
    capture.add_argument('--build',required=True); capture.add_argument('--out',required=True)
    method=capture.add_mutually_exclusive_group(required=True)
    method.add_argument('--calibration'); method.add_argument('--calibration-trial',choices=['axial','lateral'])
    capture.add_argument('--candidate-sign',type=int,choices=[-1,1],default=1)
    capture.add_argument('--limits'); capture.add_argument('--duration',type=float,default=45.)
    cal=commands.add_parser('calibrate'); cal.add_argument('--axial',required=True); cal.add_argument('--lateral',required=True)
    cal.add_argument('--sign',type=int,choices=[-1,1],required=True); cal.add_argument('--out',required=True)
    replay=commands.add_parser('report'); replay.add_argument('run')
    obs=commands.add_parser('observe'); obs.add_argument('--case',choices=CASES,required=True)
    obs.add_argument('--build',required=True); obs.add_argument('--calibration',required=True); obs.add_argument('--out',required=True)
    obs.add_argument('--duration',type=float,default=60.); obs.add_argument('--params')
    for command in (builder,capture):
        command.add_argument('--host',default='localhost'); command.add_argument('--port',type=int,default=23000)
    args=parser.parse_args(argv)
    if hasattr(args,'duration') and (not math.isfinite(args.duration) or not .5<=args.duration<=300.):
        parser.error('--duration must be finite, 0.5..300 s')
    if args.action=='preflight': dump(args.out,environment()); return 0
    if args.action=='prepare': prepare(args.config,args.out); return 0
    if args.action=='build':
        result=build(args.config,args.out,args.host,args.port)
        print(json.dumps(result,indent=2)); return 0 if result['complete'] else 2
    if args.action=='capture': result=acquire(args)
    elif args.action=='calibrate': result=make_calibration(args.axial,args.lateral,args.sign,args.out)
    elif args.action=='observe': result=observe(args)
    else: result=report(args.run)
    print(json.dumps(result,indent=2,allow_nan=False))
    return {'PASS':0,'ACQUIRED_UNCALIBRATED':0,'FAIL':1,'INSUFFICIENT':2}[result['status']]


if __name__=='__main__':
    sys.exit(main())
