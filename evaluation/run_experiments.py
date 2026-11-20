#!/usr/bin/env python3
"""Run fresh Gazebo processes per trial. Execute inside the built ROS container.

No log is generated from a Python world model. A failed/missing simulator is a
failed run and cannot be included as a successful baseline comparison.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import subprocess
import time


def read_records(path):
    if not path.is_file(): return []
    rows=[]
    for line in path.read_text().splitlines():
        try: rows.append(json.loads(line))
        except json.JSONDecodeError: pass  # writer may currently be on the last line
    return rows


def scheduled_incidents(records,elapsed):
    """Incidents due in the scenario manager's simulated-time schedule.

    Infer the manager's time origin, then advance it using independent Gazebo
    poses. A dead manager's stale status must not erase later scheduled events.
    The one-second fallback only supports older logs without scenario status.
    """
    metadata=next((r['data'] for r in records if r['topic']=='run_metadata'),{})
    status=[r['data'] for r in records if r['topic']=='/hri/status'
            and r['data'].get('source')=='scenario' and 'elapsed' in r['data']]
    poses=[r['data']['stamp'] for r in records if r['topic']=='/human/odom']
    anchored=[s for s in status if 'stamp' in s]
    if anchored and poses:
        origin=anchored[0]['stamp']-anchored[0]['elapsed']
        scenario_elapsed=max(0,max(poses)-origin)
    else:
        scenario_elapsed=max((s['elapsed'] for s in status),default=max(0,elapsed-1))
    return sorted((h for h in metadata.get('config',{}).get('hazards',[])
                   if h.get('at',3)<=scenario_elapsed),key=lambda h:h.get('at',3))


def validate_evidence(records,elapsed):
    """Require independent Gazebo telemetry and the configured incident pipeline."""
    poses=[r['data'] for r in records if r['topic']=='/human/odom']
    if len(poses)<2 or poses[-1]['stamp']<=poses[0]['stamp']:
        return False,'missing_advancing_gazebo_odometry'
    if not any(r['topic']=='/hri/human_intent' for r in records):
        return False,'missing_human_behavior'
    if any(r['topic']=='/hri/events' and r['data'].get('event')=='injection_failed' for r in records):
        return False,'hazard_injection_failed'
    sensors={r['data'].get('source') for r in records if r['topic']=='/evaluation/sensors'}
    if 'human' not in sensors or not sensors.intersection({'robot','uav'}):
        return False,'missing_logical_camera_streams'
    scheduled=scheduled_incidents(records,elapsed)
    if scheduled:
        first=scheduled[0]['id']
        spawned={r['data'].get('hazard',{}).get('id') for r in records
                 if r['topic']=='/evaluation/truth' and r['data'].get('event')=='hazard_created'}
        if first not in spawned:
            return False,'missing_first_physical_incident'
        if any(h['id'] not in spawned for h in scheduled):
            return False,'missing_scheduled_physical_incident'
        observed={h.get('id') for r in records if r['topic']=='/hri/observations'
                  for h in r['data'].get('hazards',[])}
        if first not in observed:
            return False,'missing_first_incident_detection'
    return True,'duration_reached'


def trial(args,scenario,policy,seed,beta):
    label=(f'{scenario}_{policy}_seed{seed}_beta{beta:g}_assumed{args.assumed_beta:g}'
           f'_trust{args.trust:g}_radius{args.observation_radius:g}_attention{args.passive_map_rate:g}_duration{args.duration:g}')
    path=args.output/f'{label}.jsonl'
    if path.exists() and not args.overwrite:
        raise RuntimeError(f'{path} already exists; use --overwrite to intentionally replace it')
    command=['ros2','launch','hri_bringup','demo.launch.py','tom:=false',f'scenario:={scenario}',f'policy:={policy}',
             f'seed:={seed}',f'log_path:={path.resolve()}','dashboard:=false','nav2:=false','gui:=false',
             f'human_beta:={beta}',f'assumed_beta:={args.assumed_beta}',f'trust:={args.trust}',
             f'observation_radius:={args.observation_radius}',f'passive_map_rate:={args.passive_map_rate}']
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION=f'hri_eval_{os.getpid()}_{label}')
    start=time.monotonic()
    valid=False
    reason='startup_timeout'
    last_sim=0.0
    first_sim=None
    with path.with_suffix('.console.log').open('w') as console:
        process=subprocess.Popen(command,stdout=console,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        try:
            while process.poll() is None:
                records=read_records(path)
                poses=[r for r in records if r['topic']=='/human/odom']
                if poses:
                    first_sim=first_sim if first_sim is not None else poses[0]['data']['stamp']
                    last_sim=poses[-1]['data']['stamp']
                    # Keep trials a fixed duration even after safety, to measure
                    # later irrelevant updates and policy interruptions fairly.
                    if last_sim-first_sim >= args.duration:
                        valid,reason=validate_evidence(records,last_sim-first_sim)
                        break
                if time.monotonic()-start > args.wall_timeout:
                    reason='wall_timeout';break
                time.sleep(.5)
            else:
                reason=f'launch_exited_{process.returncode}'
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
    if not path.is_file():
        raise RuntimeError(f'No ROS event log for {label}; see {path.with_suffix(".console.log")}')
    with path.open('a') as stream:
        stream.write(json.dumps({'topic':'run_end','stamp':last_sim,'data':{
            'stamp':last_sim,'valid':valid,'reason':reason,'command':command,
            'human_beta':beta,'assumed_beta':args.assumed_beta,'trust':args.trust,
            'observation_radius':args.observation_radius,'passive_map_rate':args.passive_map_rate,
            'requested_duration_s':args.duration,
            'wall_seconds':time.monotonic()-start}})+'\n')
    print(f'{label}: {reason}; valid={valid}',flush=True)
    if not valid:
        raise RuntimeError(f'Failed integrated trial: {label}; see console log')
    return path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenarios',nargs='+',choices=['forest','industrial'],default=['forest','industrial'])
    parser.add_argument('--policies',nargs='+',choices=['B1','B2','B3','B4','oracle'],default=['B1','B2','B3','B4','oracle'])
    parser.add_argument('--seeds',nargs='+',type=int,default=[7,17,27])
    parser.add_argument('--human-betas',nargs='+',type=float,default=[1.2])
    parser.add_argument('--assumed-beta',type=float,default=1.2)
    parser.add_argument('--trust',type=float,default=.8)
    parser.add_argument('--observation-radius',type=float,default=3.0)
    parser.add_argument('--passive-map-rate',type=float,default=.05)
    parser.add_argument('--duration',type=float,default=90)
    parser.add_argument('--wall-timeout',type=float,default=900)
    parser.add_argument('--domain',type=int,default=83)
    parser.add_argument('--output',type=Path,default=Path('results/trials'))
    parser.add_argument('--overwrite',action='store_true')
    args=parser.parse_args()
    if not all(math.isfinite(v) for v in [args.duration,args.wall_timeout,args.trust,args.observation_radius,
                                        args.assumed_beta,args.passive_map_rate,*args.human_betas]):
        parser.error('Numeric conditions must be finite')
    if args.duration<=0 or args.wall_timeout<=0: parser.error('Durations must be positive')
    if not 0<=args.domain<=232: parser.error('ROS domain must be between 0 and 232')
    if not 0<=args.trust<=1: parser.error('Trust must lie in [0,1]')
    if args.observation_radius<=0 or min(args.human_betas)<0 or args.assumed_beta<0 or args.passive_map_rate<0:
        parser.error('Radius must be positive, rationalities and passive-map rate nonnegative')
    args.output.mkdir(parents=True,exist_ok=True)
    versions=subprocess.run(['gz','sim','--versions'],capture_output=True,text=True,check=True).stdout.strip()
    root=Path(__file__).resolve().parents[1]
    suffixes={'.py','.sdf','.json','.yaml','.yml','.xml','.xacro','.urdf','.config','.rviz','.sh','.toml','.cfg',
              '.obj','.mtl','.dae','.stl','.png','.jpg','.jpeg'}
    hashes={}
    for source_dir in ('ros2_ws/src','evaluation','docker','scripts'):
        for p in (root/source_dir).rglob('*'):
            if not p.is_file() or '__pycache__' in p.parts:
                continue
            if p.suffix in suffixes or p.name in ('CMakeLists.txt','Dockerfile'):
                hashes[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
    manifest={'platform':platform.platform(),'python':platform.python_version(),'gazebo':versions,
              'source_sha256':hashes,'arguments':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for scenario in args.scenarios:
        for beta in args.human_betas:
            for seed in args.seeds:
                for policy in args.policies:
                    trial(args,scenario,policy,seed,beta)


if __name__=='__main__': main()
