#!/usr/bin/env python3
"""Offline finite communication-MDP comparison; never trains during ROS launch."""
import argparse,json,time
from pathlib import Path
from hri_reasoning.warning_model import WarningMDP,ACTIONS

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
p.add_argument('--config',type=Path,default=Path('ros2_ws/src/hri_reasoning/config/warning_policy.json'))
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
config=json.loads(a.config.read_text());mdp=WarningMDP(config)
start=time.perf_counter();vi=mdp.value_iteration();vi['wall_seconds']=time.perf_counter()-start
settings=config['q_learning'];kwargs={k:settings[k] for k in ('seed','episodes','steps','alpha','epsilon')}
start=time.perf_counter();q=mdp.train_q(**kwargs);q['wall_seconds']=time.perf_counter()-start
again=mdp.train_q(**kwargs)
reproducible=all(q[k]==again[k] for k in ('q','policy','visits'))
residual=max(abs(q['q'][s][i]-(mdp.cost(s,action)+mdp.discount*sum(
    p*min(q['q'][n]) for n,p in mdp.transitions(s,action).items())))
    for s in range(mdp.terminal) for i,action in enumerate(ACTIONS))
agreement=sum(q['policy'][s]==vi['policy'][s] for s in range(mdp.terminal))/mdp.terminal
report={'status':'passed' if vi['converged'] and reproducible else 'failed',
    'model':'finite communication approximation, not Gazebo or human-subject learning',
    'value_iteration':{k:v for k,v in vi.items() if k not in ('values','policy')},
    'q_learning':{'seed':settings['seed'],'evaluation_seed':settings['evaluation_seed'],
        'episodes':settings['episodes'],'reproducible':reproducible,'bellman_residual':residual,
        'agreement_with_value_iteration':agreement,'converged_to_vi_tolerance':residual<1e-8,
        'wall_seconds':q['wall_seconds']},
    'evaluation':{'value_iteration':mdp.evaluate(vi['policy'],seed=settings['evaluation_seed']),
                  'q_learning':mdp.evaluate(q['policy'],seed=settings['evaluation_seed'])}}
for name,data in (('value_iteration',vi),('q_policy',q),('report',report),('config',config)):
    (a.output/f'{name}.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['status']=='passed' else 1)
