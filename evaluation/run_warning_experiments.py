#!/usr/bin/env python3
"""Matched physical Gazebo trials for all warning baselines. No offline fake poses."""
import argparse,json,math,os,re,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from phase1_acceptance import terminate
from run_experiments import read_records
from warning_metrics import evaluate


def trial(args,policy,knowledge):
    label=f'{policy}_{knowledge}';out=args.output/label;out.mkdir()
    config=json.loads((ROOT/'ros2_ws/src/hri_sim/config/forest_scenarios/F.json').read_text())
    config['hazards'][0]['at']=1.;config['seed']=args.seed
    config['crew_initial_knowledge']={'trail_a':'blocked'} if knowledge=='known' else {}
    (out/'scenario.json').write_text(json.dumps(config,indent=2)+'\n')
    for name in ('warning_policy','human_response'):
        (out/f'{name}.json').write_text((ROOT/f'ros2_ws/src/hri_reasoning/config/{name}.json').read_text())
    parameters={'scenario':'forest','forest_case':'F','scenario_config':str((out/'scenario.json').resolve()),
        'seed':args.seed,'tom':'true','policy':policy,'warning_engine':args.engine,
        'warning_config':str((out/'warning_policy.json').resolve()),'response_config':str((out/'human_response.json').resolve()),
        'observe_enabled':'false','uav_controller':'external','dashboard':'false','mission':'false','gui':'false',
        'log_path':str((out/'events.jsonl').resolve()),'trust':.9,'observation_radius':1.,'assumed_observation_radius':1.,
        'human_lapse':0.,'passive_map_rate':0.,'q_policy':args.q_policy}
    cmd=['ros2','launch','hri_bringup','demo.launch.py']+[f'{k}:={v}' for k,v in parameters.items() if v!='']
    env=dict(os.environ,ROS_DOMAIN_ID='51',GZ_PARTITION=f'phase4_{os.getpid()}_{label}')
    (out/'command.json').write_text(json.dumps({'command':cmd,'environment':{k:env[k] for k in ('ROS_DOMAIN_ID','GZ_PARTITION')}},indent=2))
    started=time.monotonic();complete=False;last=0.
    with (out/'simulator.log').open('w') as log:
        process=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while process.poll() is None:
                records=read_records(out/'events.jsonl')
                poses=[r['data'] for r in records if r['topic']=='/human/odom']
                last=poses[-1]['stamp'] if poses else 0.
                if last>=args.duration:complete=True;break
                if time.monotonic()-started>args.wall_timeout:break
                time.sleep(.5)
        finally:
            try:
                with (out/'shutdown.log').open('w') as log:
                    subprocess.run(['gz','service','-s','/world/forest/control','--reqtype','gz.msgs.WorldControl',
                        '--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true'],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=6)
                time.sleep(.5)
            finally:terminate(process)
    records=read_records(out/'events.jsonl');poses=[r['data'] for r in records if r['topic']=='/human/odom']
    decisions=[r['data'] for r in records if r['topic']=='/hri/decision']
    critical=[l for l in (out/'simulator.log').read_text().splitlines() if re.search(r'\[ERROR\]|\[Err\]|Traceback|process has died|Segmentation fault',l)]
    metrics=evaluate(records) if records else {}
    motion=max((math.dist(p['position'],poses[0]['position']) for p in poses),default=0.)
    checks={'duration_reached':complete,'actual_movement':motion>2.,'posterior':any(r['topic']=='/hri/belief' for r in records),
        'warning_decisions':bool(decisions),'route_output':bool(metrics.get('route_revisions')),
        'blocked_routes_rejected':metrics.get('unsafe_blocked_recommendations')==0,
        'physical_incident':any(r['topic']=='/evaluation/truth' for r in records),
        'clean_logs':not critical,'clean_exit':process.returncode==0}
    result={'status':'passed' if all(checks.values()) else 'failed','checks':checks,'metrics':metrics,
        'sim_seconds':last,'crew_displacement_m':motion,'critical_log_lines':critical,'wall_seconds':time.monotonic()-started,
        'decisions_by_class':{c:sum(d['classification']==c for d in decisions) for c in sorted({d['classification'] for d in decisions})}}
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(label,result['status'],f't={last:.1f}, alerts={metrics.get("warning_count")}, completed={metrics.get("evacuation_completed")}',flush=True)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policies',nargs='+',choices=['B1','B2','B3','B4','oracle'],default=['B1','B2','B3','B4','oracle'])
    p.add_argument('--knowledge',nargs='+',choices=['known','unaware'],default=['known','unaware'])
    p.add_argument('--duration',type=float,default=80.)
    p.add_argument('--wall-timeout',type=float,default=600.)
    p.add_argument('--seed',type=int,default=7)
    p.add_argument('--engine',choices=['expected_loss','value_iteration','q_learning'],default='expected_loss')
    p.add_argument('--q-policy',default='')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    runs={f'{policy}_{knowledge}':trial(args,policy,knowledge) for policy in args.policies for knowledge in args.knowledge}
    signatures=[r['metrics'].get('semantic_robot_observations',[]) for r in runs.values()]
    same_observations=bool(signatures[0]) and all(s==signatures[0] for s in signatures)
    comparison=None
    if 'B4_known' in runs and 'B4_unaware' in runs:
        comparison={'known_alerts':runs['B4_known']['metrics']['warning_count'],
                    'unaware_alerts':runs['B4_unaware']['metrics']['warning_count']}
        comparison['different']=comparison['known_alerts']!=comparison['unaware_alerts']
    report={'status':'passed' if all(r['status']=='passed' for r in runs.values()) and same_observations else 'failed',
        'engine':args.engine,'identical_semantic_robot_observations':same_observations,
        'observation_control':'Same seeded physical tree and static robot/UAV poses; OBSERVE disabled. Sensor receipt timestamps remain asynchronous.',
        'belief_aware_comparison':comparison,'runs':runs}
    (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    raise SystemExit(0 if report['status']=='passed' else 1)


if __name__=='__main__':main()
