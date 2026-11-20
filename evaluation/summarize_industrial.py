#!/usr/bin/env python3
"""Assemble selected acceptance evidence without rewriting failed historical runs."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/phase5-report';OUT.mkdir(exist_ok=True)
paths={f'scenario_{case}':f'results/phase5-acceptance/{case}/result.json' for case in ('blockage','fire','gas','no_safe','unaware')}
paths.update({f'scenario_{case}':f'results/phase5-final/{case}/result.json' for case in ('normal','known')})
paths.update(collision='results/phase5-collision/result.json',forest_regression='results/phase5-forest-regression/summary.json',
    value_iteration='results/phase5-value-iteration/unaware/result.json',q_learning='results/phase5-q-learning/unaware/result.json',
    visible_demo='results/phase5-visible/result.json')
paths['unit_tests']='results/phase5-report/unit_tests.json'
checks={}
for name,path in paths.items():
    try:data=json.loads((ROOT/path).read_text())
    except FileNotFoundError:data={}
    checks[name]={'passed':data.get('status')=='passed','source':path}
policies={};observations=[];events=[]
for policy in ('B1','B2','B3','B4','oracle'):
    base=f'results/phase5-oracle-retest/{policy}' if policy=='oracle' else f'results/phase5-policies/{policy}'
    for case in ('known','unaware'):
        path=base+'/'+case+'/result.json'
        try:
            report=json.loads((ROOT/path).read_text())
            records=[json.loads(l) for l in (ROOT/base/case/'events.jsonl').read_text().splitlines()]
        except FileNotFoundError:report={};records=[]
        decisions=[d for d in report.get('history',{}).get('decision',[]) if d.get('hazard_id')]
        first=decisions[0] if decisions else {}
        policies[policy+'_'+case]={'passed':report.get('status')=='passed','source':path,
            'warnings':len(report.get('history',{}).get('warning',[])),
            'first_action':first.get('action'),'first_probability':first.get('context',{}).get('probability')}
        observed={}
        for row in records:
            if row['topic']=='/hri/observations':
                for h in row['data']['hazards']:observed[h['id']]=[h['kind'],h['edge'],[round(v,3) for v in h['position']],h['state']]
        observations.append(observed)
        events.append([(r['data']['hazard']['id'],r['data']['hazard']['edge'],r['data']['hazard']['at']) for r in records if r['topic']=='/evaluation/truth' and r['data'].get('event')=='hazard_created'])
checks['all_policy_trials']={'passed':all(v['passed'] for v in policies.values())}
checks['matched_semantic_observations']={'passed':bool(observations[0]) and all(o==observations[0] for o in observations)}
checks['same_seed_event_sequence']={'passed':bool(events[0]) and all(e==events[0] for e in events)}
checks['belief_aware_different']={'passed':policies['B4_known']['warnings']==0 and policies['B4_unaware']['warnings']>0}
core=json.loads((ROOT/'ros2_ws/src/hri_industrial/config/reused_core_sha256.json').read_text())
checks['research_core_unchanged']={'passed':all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==digest for p,digest in core.items())}
result={'status':'passed' if all(c['passed'] for c in checks.values()) else 'failed','checks':checks,'policies':policies,
    'historical_failures':'Original phase5-initial, phase5-acceptance/known, and phase5-policies/oracle/known failures are preserved and explained in docs/PHASE5_VALIDATION.md.',
    'limits':['One-seed modeled-operator comparison; no statistical human-trust claim','Ideal semantic detection and configurable fire/gas envelopes','Odometry navigation; no validated SLAM/AMCL','Existing Q-table is executable but not proven converged']}
(OUT/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='passed' else 1)
