#!/usr/bin/env python3
"""Aggregate genuine logged trials; excludes incomplete runs by default."""
import argparse
import csv
import json
from pathlib import Path
import statistics
from metrics import summarize

CONDITION_FIELDS=('scenario','policy','human_beta','assumed_beta','trust',
                  'observation_radius','passive_map_rate','requested_duration_s','config_sha256')


def condition_key(run):
    return '/'.join(f'{field}={run.get(field)}' for field in CONDITION_FIELDS)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('logs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,default=Path('results/comparison.json'))
    args=parser.parse_args()
    runs=[]
    for path in args.logs:
        try:
            records=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
            runs.append({**summarize(records),'source':str(path)})
        except (ValueError,KeyError) as exc:
            raise SystemExit(f'Invalid log {path}: {exc}')
    groups={}
    for run in runs:
        if run['status']!='complete': continue
        key=condition_key(run)
        groups.setdefault(key,[]).append(run)
    aggregate={}
    for key,group in groups.items():
        numeric={}
        for metric in group[0]:
            values=[float(r[metric]) for r in group if isinstance(r[metric],(int,float,bool))
                    and metric not in (*CONDITION_FIELDS,'seed')]
            if values:
                numeric[metric]={'mean':statistics.mean(values),'n':len(values),
                                 'stdev':statistics.stdev(values) if len(values)>1 else None}
        aggregate[key]={'condition':{field:group[0].get(field) for field in CONDITION_FIELDS},
                        'runs':len(group),'metrics':numeric}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'runs':runs,'aggregates':aggregate,
        'note':'Only integrated logs ending with run_end are aggregated; null means unobserved/undefined.'},indent=2)+'\n')
    with args.output.with_suffix('.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(runs[0]) if runs else [])
        writer.writeheader();writer.writerows(runs)
    print(f'Wrote {args.output}: {len(groups)} groups, {sum(r["status"]=="complete" for r in runs)} complete runs')


if __name__=='__main__': main()
