"""Opt-in evaluation adapter. NEVER imported by ROS inference or policy nodes."""
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def evaluate_log(path):
    if not path.exists():return {'available':False,'reason':'Waiting for experimental log'}
    records=[]
    for line in path.read_text().splitlines():
        try:records.append(json.loads(line))
        except ValueError:continue  # Logger may be writing its last line.
    private=[r for r in records if r['topic']=='/evaluation/human_belief' and 'knowledge' in r['data']]
    if not private:return {'available':False,'reason':'No simulated human ground truth; real operator belief cannot be scored'}
    sys.path.insert(0,str(ROOT/'ros2_ws/src/hri_reasoning'))
    sys.path.insert(0,str(ROOT/'evaluation'))
    from warning_metrics import evaluate
    metadata=next((r for r in records if r['topic']=='run_metadata'),None)
    if not metadata:return {'available':False,'reason':'Waiting for run metadata'}
    if metadata['data']['scenario']=='oil_facility':
        # This metric uses position for exposure; substitute measured robot odom
        # for the operator-controlled platform. Keep the private operator labels.
        records=[{**r,'topic':'/human/odom'} if r['topic']=='/robot/odom' else r for r in records]
    result=evaluate(records)
    return {'available':True,'source':'simulation ground truth / evaluation only','provisional':True,
        **{k:result[k] for k in ('warning_count','false_alarms','missed_warnings','warning_precision','warning_recall',
            'exposure_seconds','evacuation_completed','modeled_compliance','dangerous_episodes')},
        'notes':'Provisional episode counts can change until the trial ends. Needed alerts use private belief-optimal routes; no claim about real operator beliefs.'}
