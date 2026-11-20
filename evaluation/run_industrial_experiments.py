#!/usr/bin/env python3
"""Paired, physical operator-awareness trials through the common warning interface."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from verify_industrial import run_case

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
p.add_argument('--policies',nargs='+',default=['B1','B2','B3','B4','oracle'])
p.add_argument('--engine',default='expected_loss',choices=['expected_loss','value_iteration','q_learning'])
p.add_argument('--q-policy',default='')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);runs={};signatures=[]
for policy in a.policies:
    for case in ('known','unaware'):
        passed=run_case(case,a.output/policy,a.engine,policy,a.q_policy)
        folder=a.output/policy/case;report=json.loads((folder/'result.json').read_text())
        records=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
        decisions=[d for d in report['history'].get('decision',[]) if d.get('hazard_id')]
        observations={}
        for record in records:
            if record['topic']=='/hri/observations':
                for h in record['data']['hazards']:
                    observations[h['id']]=[h['kind'],h['edge'],[round(v,3) for v in h['position']],h['state']]
        signatures.append(observations)
        first=decisions[0] if decisions else {}
        runs[policy+'_'+case]={'passed':passed,'warnings':len(report['history'].get('warning',[])),
            'first_action':first.get('action'),'first_classification':first.get('classification'),
            'first_probability':first.get('context',{}).get('probability'),
            'operator_actions':report.get('belief',{}).get('action_evidence_count'),
            'responses':[r['data'] for r in records if r['topic']=='/evaluation/operator_response']}
same=bool(signatures[0]) and all(s==signatures[0] for s in signatures)
different=None
if 'B4' in a.policies:different=runs['B4_known']['warnings']<runs['B4_unaware']['warnings']
result={'status':'passed' if all(r['passed'] for r in runs.values()) and same and different is not False else 'failed',
    'engine':a.engine,'same_physical_hazard_and_semantic_observations':same,'belief_aware_different':different,'runs':runs,
    'control':'Robot stationary at identical surveyed spawn, same seeded debris and absolute simulation schedule; observation receipt remains asynchronous.',
    'scope':'Short paired operator-awareness demonstration, not a statistically powered human-subjects study.'}
(a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='passed' else 1)
