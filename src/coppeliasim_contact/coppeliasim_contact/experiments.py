"""Configure physical interventions and run an engine-only (non-ROS) probe."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
from .backend import CoppeliaBackend, reaction
from .contracts import SceneConfig, MotionGuard, config_dict

CASES = ('normal', 'offset', 'sidewall', 'jam', 'overload')


def case_config(base, case):
    if case not in CASES:
        raise ValueError(case)
    return replace(base, hole_offset_x_m=.0004 if case=='offset' else 0.,
                   stop_depth_m=.003 if case in ('jam','overload') else None,
                   motor_force_n=60. if case=='overload' else 25.)


def assess(rows, cfg):
    """Evidence flags, not learned classes or definitive two-point wedging labels."""
    valid = [r for r in rows if r.get('reaction_at_tip') is not None]
    contact = [r for r in valid if r['contacts']]
    force = lambda r: sum(x*x for x in r['reaction_at_tip'][:3])**.5
    jam = False
    for i, r in enumerate(valid):
        old = next((p for p in reversed(valid[:i]) if r['simulation_time_s']-p['simulation_time_s'] >= .5), None)
        if (old and r['command_applied_mps'][2] < 0 and r['contacts'] and force(r)>=12
                and abs(r['pose'][2]-old['pose'][2])<.00005
                and all(p['command_applied_mps'][2]<0 for p in valid if
                        old['simulation_time_s']<=p['simulation_time_s']<=r['simulation_time_s'])):
            jam = True
            break
    top = cfg.hole_center_m[2]
    return {'valid_samples':len(valid), 'contact_samples':len(contact),
            'depth_target_reached':any(r['pose'][2]<=top-.0038 for r in valid),
            'below_rim_contact_observed':any(p['point_world_m'][2]<top-.0002 for r in contact for p in r['contacts']),
            'jam_candidate':jam,
            'overload_observed':any(force(r)>=45 for r in valid),
            'max_force_n':max((force(r) for r in valid),default=None)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    cfg_parser=sub.add_parser('configure')
    cfg_parser.add_argument('--config',required=True)
    cfg_parser.add_argument('--case',choices=CASES,required=True)
    cfg_parser.add_argument('--output',required=True)
    run=sub.add_parser('run')
    run.add_argument('--case',choices=CASES,required=True)
    run.add_argument('--reaction-sign',type=int,choices=(-1,1),required=True)
    run.add_argument('--output',required=True)
    run.add_argument('--calibration',action='store_true',help='Require 5 N motors for sign calibration')
    run.add_argument('--host',default='localhost')
    run.add_argument('--port',type=int,default=23000)
    args=parser.parse_args()
    if args.action=='configure':
        cfg=case_config(SceneConfig.load(args.config),args.case)
        with open(args.output,'x') as f: json.dump(config_dict(cfg),f,indent=2)
        return
    # Open exclusively before starting; never overwrite a previous experiment.
    with open(args.output,'x') as log:
        backend=CoppeliaBackend(args.host,args.port)
        cfg=backend.cfg
        expected=case_config(cfg,args.case)
        if args.calibration:
            expected=replace(expected,motor_force_n=5.)
        if config_dict(expected)!=config_dict(cfg):
            raise ValueError('Loaded scene does not match intervention; rebuild using configure')
        guard=MotionGuard(sign=args.reaction_sign,speed=.002)
        rows=[]
        log.write(json.dumps({'kind':'metadata','case':args.case,'metadata':backend.metadata,
                              'source':'coppeliasim_physics','reaction_sign':args.reaction_sign})+'\n')
        try:
            backend.start()
            for i in range(round(45/backend.dt)):
                # First 0.2 s settles the sensor. No artificial force samples.
                cmd=(0.,0.,0.)
                if rows and i*backend.dt>=.2:
                    latest=rows[-1]
                    if args.case=='sidewall' and latest['pose'][2] <= cfg.hole_center_m[2]-.002:
                        cmd=(.0002,0.,0.)
                    elif latest['pose'][2] > cfg.hole_center_m[2]-.004:
                        cmd=(0.,0.,-.001)
                if rows and rows[-1]['reaction_at_tip'] is None:
                    cmd=(0.,0.,0.)
                backend.command(cmd)
                sample=backend.advance()
                f=reaction(sample,args.reaction_sign)
                guard.velocity(1.,1.,sample['pose'][:3],f or (0.,)*6,f is not None)
                row={**sample,'kind':'sample','reaction_at_tip':f,'command_applied_mps':cmd,
                     'stop_latched':guard.halted}
                rows.append(row)
                log.write(json.dumps(row,allow_nan=False)+'\n')
                if guard.halted:
                    backend.command((0.,0.,0.))
                    break
            summary=assess(rows,cfg)
            log.write(json.dumps({'kind':'summary',**summary})+'\n')
            print(json.dumps(summary,indent=2))
        finally:
            backend.close()


if __name__=='__main__':
    main()
