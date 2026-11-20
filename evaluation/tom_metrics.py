"""Evaluation-only joins with private human beliefs and physical incident truth.

These functions are never imported by the inference or dashboard processes.
Scores describe timestamped simulation samples, not independent human subjects.
"""
import math
from hri_reasoning.tom import STATES, canonical


def score_samples(samples, threshold=.7):
    correct=true_probability=brier=0.
    tp=fp=fn=tn=abstentions=0
    bins=[{'count':0,'confidence_sum':0.,'correct_sum':0} for _ in range(10)]
    for sample in samples:
        p=sample['posterior']; truth=sample['true_hypothesis']
        estimate=max(STATES,key=lambda h:p[h]); hit=int(estimate==truth)
        correct+=hit;true_probability+=p[truth]
        brier+=sum((p[h]-int(h==truth))**2 for h in STATES)
        bucket=bins[min(9,int(p[estimate]*10))]
        bucket['count']+=1;bucket['confidence_sum']+=p[estimate];bucket['correct_sum']+=hit
        probability=sample['p_mismatch']
        abstentions+=int(probability is None)
        detected=probability is not None and probability>=threshold
        actual=sample['dangerous_mismatch']
        tp+=int(detected and actual);fp+=int(detected and not actual)
        fn+=int(not detected and actual);tn+=int(not detected and not actual)
    n=len(samples)
    reliability=[{'bin':[i/10,(i+1)/10],'count':b['count'],
        'confidence':b['confidence_sum']/b['count'],'accuracy':b['correct_sum']/b['count']}
        for i,b in enumerate(bins) if b['count']]
    return {'samples':n,'classification_accuracy':correct/n if n else None,
        'mean_true_hypothesis_probability':true_probability/n if n else None,
        'multiclass_brier':brier/n if n else None,
        'expected_calibration_error':sum(b['count']*abs(b['confidence']-b['accuracy']) for b in reliability)/n if n else None,
        'reliability_bins':reliability,'threshold':threshold,'tp':tp,'fp':fp,'fn':fn,'tn':tn,
        'mismatch_precision':tp/(tp+fp) if tp+fp else None,
        'mismatch_recall':tp/(tp+fn) if tp+fn else None,
        'unassessed_hazard_samples':abstentions}


def evaluate(records, edge='trail_a', threshold=.7):
    """Causal latest-at-or-before joins; never pair with a future private update."""
    ordered=sorted(enumerate(records),key=lambda item:(item[1]['data'].get('stamp',item[1].get('stamp',0)),item[0]))
    knowledge=None;intent={};active={};samples=[];detections=[];decisions=[];exposures=[]
    branch_snapshots=[];decisions_seen=set()
    for _,record in ordered:
        topic,data=record['topic'],record['data'];stamp=data.get('stamp',record.get('stamp',0))
        if topic=='/evaluation/human_belief' and 'knowledge' in data:
            knowledge=data['knowledge']
        elif topic=='/evaluation/truth' and data.get('event')=='hazard_created':
            h=data['hazard'];active[h['edge']]=h
        elif topic=='/hri/human_intent':
            intent=data
            if (data.get('origin')=='fork' and data.get('target')=='trail_a' and edge in active
                    and data.get('decision_id') not in decisions_seen):
                decisions.append(stamp);decisions_seen.add(data.get('decision_id'))
        elif topic=='/hri/tom_evidence' and data.get('event')=='movement_action':
            if data['origin']=='fork':
                branch_snapshots.append({'action':data,'posterior_after_action':None})
        elif topic=='/human/odom' and edge in active:
            h=active[edge]
            if math.dist(data['position'][:2],h['position'][:2])<=h.get('radius',2):
                exposures.append(stamp)
        elif topic=='/hri/belief' and knowledge is not None and edge in data.get('beliefs',{}):
            b=data['beliefs'][edge]
            if branch_snapshots and branch_snapshots[-1]['posterior_after_action'] is None:
                branch_snapshots[-1]['posterior_after_action']=b
            if edge not in active:
                continue  # Report the changed edge after physical incident creation.
            truth=canonical(knowledge[edge])
            path=intent.get('path',[])
            uses_edge=any(set(pair)=={'fork','trail_a'} for pair in zip(path,path[1:]))
            incompatible = ('open_safe','open_hazardous') if active[edge].get('kind','tree') in ('tree','debris') else ('open_safe',)
            actual=truth in incompatible and uses_edge and not intent.get('completed')
            samples.append({'stamp':stamp,'posterior':b['posterior'],'true_hypothesis':truth,
                'p_mismatch':b['p_mismatch'],'dangerous_mismatch':actual})
            if b['p_mismatch'] is not None and b['p_mismatch']>=threshold:
                detections.append(stamp)
    metrics=score_samples(samples,threshold)
    first=detections[0] if detections else None
    metrics.update(first_detection=first,first_hazardous_decision=decisions[0] if decisions else None,
        first_hazard_proximity=exposures[0] if exposures else None,
        lead_to_decision_s=decisions[0]-first if decisions and first is not None else None,
        lead_to_proximity_s=exposures[0]-first if exposures and first is not None else None,
        branch_snapshots=branch_snapshots,
        notes='Scores use the changed edge after creation; repeated samples are correlated. Proximity is a 2m research zone, not a physical dose. Missing hazard observations count as abstentions and false negatives when applicable.')
    return metrics
