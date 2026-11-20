#!/usr/bin/env python3
"""Audit frozen integrated logs without rewriting their original validity labels."""
import argparse
import hashlib
import json
from pathlib import Path

from run_experiments import scheduled_incidents, validate_evidence


def audit(path):
    raw=path.read_bytes()
    records=[json.loads(line) for line in raw.decode().splitlines() if line.strip()]
    metadata=next((r['data'] for r in records if r['topic']=='run_metadata'),{})
    ending=next((r['data'] for r in reversed(records) if r['topic']=='run_end'),{})
    poses=[r['data']['stamp'] for r in records if r['topic']=='/human/odom']
    elapsed=poses[-1]-poses[0] if poses else 0
    passed,reason=validate_evidence(records,elapsed)
    scheduled=scheduled_incidents(records,elapsed)
    created={r['data']['hazard']['id']:r['data']['stamp'] for r in records
             if r['topic']=='/evaluation/truth' and r['data'].get('event')=='hazard_created'}
    observed={}
    for row in records:
        if row['topic']!='/hri/observations': continue
        for hazard in row['data'].get('hazards',[]):
            observed.setdefault(hazard['id'],row['data']['stamp'])
    incidents=[{'id':h['id'],'scheduled_at':h.get('at',3),'created_at':created.get(h['id']),
                'first_detected_at':observed.get(h['id'])} for h in scheduled]
    return {'source':str(path),'log_sha256':hashlib.sha256(raw).hexdigest(),
            'scenario':metadata.get('scenario'),'policy':metadata.get('policy'),'seed':metadata.get('seed'),
            'original_run_end_valid':ending.get('valid',False),'audit_passed':passed,'reason':reason,
            'physical_duration_s':elapsed,'scheduled_count':len(scheduled),
            'created_count':sum(h['created_at'] is not None for h in incidents),
            'detected_count':sum(h['first_detected_at'] is not None for h in incidents),'incidents':incidents}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('logs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,default=Path('results/incident-audit.json'))
    args=parser.parse_args()
    runs=[audit(path) for path in args.logs]
    report={'kind':'post_run_incident_integrity_audit','runs':runs,
            'passed':sum(r['audit_passed'] for r in runs),'failed':sum(not r['audit_passed'] for r in runs),
            'validator_sha256':hashlib.sha256(Path(__file__).with_name('run_experiments.py').read_bytes()).hexdigest(),
            'note':'Original trial logs and source manifests are unchanged. Every due physical incident must exist; '
                   'only the first planned visible incident must be detected. Other missing detections can be legitimate frustum limits.'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(f'Incident audit: {report["passed"]} passed, {report["failed"]} failed; '
          f'{sum(r["created_count"] for r in runs)}/{sum(r["scheduled_count"] for r in runs)} scheduled incidents created; '
          f'{sum(r["detected_count"] for r in runs)} detected')
    if report['failed']: raise SystemExit(1)


if __name__=='__main__':main()
