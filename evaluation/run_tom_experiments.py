#!/usr/bin/env python3
"""Six Phase 3 conditions in fresh real Gazebo processes, with saved evidence."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from phase1_acceptance import terminate
from run_experiments import read_records
from tom_metrics import evaluate

CONDITIONS={
    '1_unaware':{},
    '2_known':{'known':True},
    '3_noisy':{'human_beta':0.,'human_lapse':.15,'seed':27},
    '4_partial_observation':{'observation_sources':'robot'},
    '5_beta_mismatch':{'human_beta':.15,'assumed_beta':4.,'seed':27},
    '6_warning':{'warning_at':13.,'trust':1.,'assumed_compliance':1.},
}


def run_condition(label,condition,args):
    output=args.output/label;output.mkdir()
    config=json.loads((ROOT/'ros2_ws/src/hri_sim/config/forest_scenarios/F.json').read_text())
    config['hazards'][0]['at']=1.
    config['seed']=condition.get('seed',args.seed)
    config['crew_initial_knowledge']={'trail_a':'blocked'} if condition.get('known') else {}
    if label=='4_partial_observation':
        # A real sensor blind spot: tree is behind the stationary ground robot;
        # the experimental observer does not receive the UAV detection stream.
        config['robot_start']=[18.,0.,.02]
    configpath=output/'scenario.json';configpath.write_text(json.dumps(config,indent=2)+'\n')
    params={'scenario':'forest','forest_case':'F','scenario_config':str(configpath.resolve()),
        'seed':args.seed,'tom':'true','policy':'B1','dashboard':'false','gui':'false','mission':'false',
        'log_path':str((output/'events.jsonl').resolve()),'observation_radius':1.,'assumed_observation_radius':1.,
        'passive_map_rate':0.,'human_lapse':0.,'human_beta':1.2,'assumed_beta':1.2,'assumed_lapse':.03}
    params.update({k:v for k,v in condition.items() if k not in ('known','warning_at')})
    command=['ros2','launch','hri_bringup','demo.launch.py']+[f'{k}:={v}' for k,v in params.items()]
    env=dict(os.environ,ROS_DOMAIN_ID='49',GZ_PARTITION=f'tom_{os.getpid()}_{label}')
    (output/'command.json').write_text(json.dumps({'command':command,'ROS_DOMAIN_ID':env['ROS_DOMAIN_ID'],
                                                'GZ_PARTITION':env['GZ_PARTITION']},indent=2))
    warning_sent=False;audited=False;start=time.monotonic();success=False;failure='launch exited';last=0
    with (output/'simulator.log').open('w') as log:
        process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while process.poll() is None:
                records=read_records(output/'events.jsonl')
                poses=[r['data'] for r in records if r['topic']=='/human/odom']
                last=poses[-1]['stamp'] if poses else 0.
                if last>=2. and not audited:
                    with (output/'boundary-audit.log').open('w') as auditlog:
                        subprocess.run(['python3','scripts/audit_tom_boundary.py','--output',str(output/'boundary-audit.json')],
                            env=env,stdout=auditlog,stderr=subprocess.STDOUT,check=False,timeout=60)
                    audited=True
                if 'warning_at' in condition and last>=condition['warning_at'] and not warning_sent:
                    payload={'id':9001,'source':'experiment','edge':'trail_a','state':'blocked',
                        'action':'explicit','hazard_id':'hazard_tree','stamp':last,
                        'explanation':'Experimental warning: lookout reports a blocked trail.'}
                    warning_command=['ros2','topic','pub','--once','--wait-matching-subscriptions','3',
                                     '--max-wait-time-secs','15','--keep-alive','2',
                                     '--qos-reliability','reliable','/hri/warning','std_msgs/msg/String',
                                     json.dumps({'data':json.dumps(payload)})]
                    with (output/'warning.log').open('w') as warning_log:
                        subprocess.run(warning_command,env=env,stdout=warning_log,stderr=subprocess.STDOUT,check=True,timeout=30)
                    (output/'warning_command.json').write_text(json.dumps(warning_command,indent=2))
                    warning_sent=True
                if last>=args.duration:
                    success=True;break
                if time.monotonic()-start>args.wall_timeout:
                    failure='wall timeout';break
                time.sleep(.4)
        finally:
            # Drain Gazebo transport callbacks before ROS bridge destruction.
            # Active sensor callbacks raced bridge teardown on this ARM64 runtime.
            pause=['gz','service','-s','/world/forest/control','--reqtype','gz.msgs.WorldControl',
                   '--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true']
            with (output/'shutdown.log').open('w') as shutdownlog:
                subprocess.run(pause,env=env,stdout=shutdownlog,stderr=subprocess.STDOUT,timeout=6)
            time.sleep(.5)
            terminate(process)
    records=read_records(output/'events.jsonl')
    poses=[r['data'] for r in records if r['topic']=='/human/odom']
    beliefs=[r['data'] for r in records if r['topic']=='/hri/belief']
    evidence=[r['data'] for r in records if r['topic']=='/hri/tom_evidence']
    critical=[line for line in (output/'simulator.log').read_text().splitlines()
              if re.search(r'\[ERROR\]|\[Err\]|Traceback|Segmentation fault|process has died|terminate called',line)]
    motion=max((math.dist(p['position'],poses[0]['position']) for p in poses),default=0.)
    checks={'simulation_duration_reached':success,'actual_crew_motion':motion>2.,
        'posterior_published':bool(beliefs),'posterior_normalized':bool(beliefs) and all(
            abs(sum(b['posterior'].values())-1)<1e-8 and set(b['posterior'])==
            {'open_safe','blocked','open_hazardous','unknown_stale'}
            for b in beliefs for b in b['beliefs'].values()),
        'movement_likelihood_used':any(e.get('event')=='movement_action' for e in evidence),
        'physical_tree_created':any(r['topic']=='/evaluation/truth' for r in records),
        'clean_shutdown':process.returncode==0,'no_critical_errors':not critical,
        'runtime_input_boundary':audited and (output/'boundary-audit.json').exists() and json.loads((output/'boundary-audit.json').read_text())['status']=='passed'}
    if condition.get('warning_at'):
        checks['warning_transition_used']=any(e.get('event')=='warning_received' for e in evidence)
        checks['human_complied']=any(r['topic']=='/evaluation/human_belief' and r['data'].get('complied') for r in records)
    result={'status':'passed' if all(checks.values()) else 'failed','checks':checks,
        'sim_seconds':last,'crew_displacement_m':motion,'critical_log_lines':critical,
        'wall_seconds':time.monotonic()-start,'metrics':evaluate(records),'failure':None if success else failure}
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(label,result['status'],f'sim={last:.1f}s, crew={motion:.2f}m',flush=True)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=Path('results/phase3-experiments'))
    p.add_argument('--conditions',nargs='+',choices=list(CONDITIONS),default=list(CONDITIONS))
    p.add_argument('--duration',type=float,default=22.)
    p.add_argument('--wall-timeout',type=float,default=360.)
    p.add_argument('--seed',type=int,default=7)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    results={name:run_condition(name,CONDITIONS[name],args) for name in args.conditions}
    comparison=None
    if '1_unaware' in results and '2_known' in results:
        unknown=results['1_unaware']['metrics']['branch_snapshots']
        known=results['2_known']['metrics']['branch_snapshots']
        a=json.loads((args.output/'1_unaware/scenario.json').read_text())
        b=json.loads((args.output/'2_known/scenario.json').read_text())
        a.pop('crew_initial_knowledge');b.pop('crew_initial_knowledge')
        comparison={'identical_world_config':a==b,'unaware_first_branch':unknown[0] if unknown else None,
                    'known_first_branch':known[0] if known else None}
        comparison['passed']=bool(a==b and unknown and known and
            unknown[0]['action']['target']=='trail_a' and known[0]['action']['target']=='trail_b' and
            unknown[0]['posterior_after_action']['posterior']['open_safe']>
            known[0]['posterior_after_action']['posterior']['open_safe']+.4)
    passed=all(r['status']=='passed' for r in results.values()) and (comparison is None or comparison['passed'])
    (args.output/'summary.json').write_text(json.dumps({'status':'passed' if passed else 'failed',
        'matched_comparison':comparison,'conditions':results},indent=2)+'\n')
    raise SystemExit(0 if passed else 1)


if __name__=='__main__': main()
