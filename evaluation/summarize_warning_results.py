#!/usr/bin/env python3
"""Combine explicit run directories, retaining failed attempts and retest provenance."""
import argparse,json
from pathlib import Path
from run_experiments import read_records
from warning_metrics import evaluate

p=argparse.ArgumentParser();p.add_argument('--runs',nargs='+',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
runs={};attempts={};sources={};branch={}
for directory in a.runs:
    for path in sorted(directory.glob('*/result.json')):
        name=path.parent.name;result=json.loads(path.read_text())
        attempts.setdefault(name,[]).append({'path':str(path),'status':result['status'],
            'critical_log_lines':result.get('critical_log_lines',[])})
        # Later explicitly supplied directories supersede earlier ones, even on failure.
        records=read_records(path.parent/'events.jsonl');result['metrics']=evaluate(records)
        runs[name]=result;sources[name]=str(path)
        beliefs=[r['data'] for r in records if r['topic']=='/hri/belief'
                 and (r['data'].get('last_movement_evidence') or {}).get('origin')=='fork']
        if beliefs:
            belief=beliefs[0]
            branch[name]={'stamp':belief['stamp'],'movement':belief['last_movement_evidence'],
                          'trail_a':belief['beliefs']['trail_a']}
expected={f'{p}_{k}' for p in ('B1','B2','B3','B4','oracle') for k in ('known','unaware')}
signatures=[r['metrics']['semantic_robot_observations'] for r in runs.values()]
same=bool(signatures and signatures[0]) and all(s==signatures[0] for s in signatures)
different=all(k in runs for k in ('B4_known','B4_unaware')) and runs['B4_known']['metrics']['warning_count']!=runs['B4_unaware']['metrics']['warning_count']
checks={'complete_matrix':set(runs)==expected,'all_selected_runs_passed':all(r['status']=='passed' for r in runs.values()),
        'same_semantic_robot_hazard_observations':same,'belief_aware_cases_differ':different}
report={'status':'passed' if all(checks.values()) else 'failed','checks':checks,'sources':sources,
        'attempt_history':attempts,'branch_evidence':branch,'runs':runs,
        'scope':'Seed-7 mechanistic demonstration; original failures retained, no claim of statistical superiority.'}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
for name,r in runs.items():
    m=r['metrics'];print(name,r['status'],m['warning_count'],m['false_alarms'],m['missed_warnings'],m['completion_time_s'])
raise SystemExit(0 if report['status']=='passed' else 1)
