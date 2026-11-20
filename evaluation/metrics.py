"""Metrics from actual integrated JSONL logs; no synthetic trial generation."""
from __future__ import annotations
import math


def summarize(records):
    metadata = next((r['data'] for r in records if r['topic']=='run_metadata'),{})
    ending = next((r['data'] for r in reversed(records) if r['topic']=='run_end'),{})
    condition = {key:ending.get(key,metadata.get(key)) for key in
                 ('human_beta','assumed_beta','trust','observation_radius','passive_map_rate','requested_duration_s')}
    graph = metadata.get('config',{})
    edge_lookup = {frozenset((e['source'],e['target'])):e['id'] for e in graph.get('edges',[])}
    active, detected, exposed, eligible, warned = {},{}, {},set(),set()
    human, intent, last_pose, last_stamp = {},{},None,None
    posterior_correct = posterior_samples = 0
    brier_sum = exposure_seconds = 0.0
    warnings = false_alarms = true_warnings = 0
    compliance = []
    mission_time = None
    first_stamp = last_time = None
    stationary_since = None
    entrapment = False
    sim_telemetry = False
    trust = None
    passive_glances = 0
    goal_position = graph.get('nodes',{}).get(graph.get('goal','safety'))

    def intended_edges():
        path = intent.get('path',[])
        return {edge_lookup.get(frozenset((a,b))) for a,b in zip(path,path[1:])}

    def mismatch(hid):
        return (hid in active and active[hid]['edge'] in intended_edges()
                and human.get(active[hid]['edge']) == 'clear' and not intent.get('completed',False))

    # Communication can reach independent subscribers in different orders.
    # Evaluate a warning against the pre-response belief at the same sim stamp.
    ordered = sorted(records,key=lambda r:(float(r['data'].get('stamp',r.get('stamp',0))),
                    0 if r['topic']=='/hri/warning' else 1))
    for row in ordered:
        topic,data = row['topic'],row['data']
        stamp = float(data.get('stamp',row.get('stamp',0)))
        if stamp > 0:
            last_time = stamp
        if topic == '/evaluation/truth' and data.get('event') == 'hazard_created':
            h = data['hazard']
            active[h['id']] = h
        elif topic == '/evaluation/human_belief':
            human = data.get('knowledge',human)
            trust = data.get('trust',trust)
            if 'complied' in data:
                compliance.append(bool(data['complied']))
        elif topic == '/hri/human_intent':
            intent = data
        elif topic == '/human/odom':
            position = data['position'][:2]
            sim_telemetry = True
            if first_stamp is None:
                first_stamp = stamp
            near = [hid for hid,h in active.items() if math.dist(position,h['position'][:2]) <= h.get('radius',2)]
            for hid in near:
                exposed.setdefault(hid,stamp)
            if last_stamp is not None:
                dt = max(0,min(1,stamp-last_stamp))
                if near:
                    exposure_seconds += dt
                if near and last_pose is not None and dt>0 and math.dist(position,last_pose)/dt < .05:
                    if stationary_since is None:
                        stationary_since = last_stamp
                    entrapment |= stamp-stationary_since >= 10
                else:
                    stationary_since = None
            last_stamp,last_pose = stamp,position
            if goal_position and math.dist(position,goal_position[:2])<.65 and mission_time is None:
                mission_time = stamp-first_stamp
        elif topic == '/hri/observations':
            for h in data.get('hazards',[]):
                detected.setdefault(h['id'],stamp)
        elif topic == '/hri/belief':
            hid = data.get('hazard_id')
            if hid in active and active[hid]['edge'] in human:
                label = human[active[hid]['edge']] == 'clear'
                p = data['posterior']['unaware']
                posterior_correct += (p>=.5)==label
                posterior_samples += 1
                brier_sum += (p-float(label))**2
        elif topic == '/hri/warning' and data.get('action') in ('soft','explicit'):
            warnings += 1
            hid = data['hazard_id']
            if mismatch(hid):
                true_warnings += 1
                warned.add(hid)
            else:
                false_alarms += 1
        elif topic == '/hri/events' and data.get('event')=='passive_map_glance':
            passive_glances += 1
        eligible.update(hid for hid in active if mismatch(hid))
    complete = sim_telemetry and first_stamp is not None and ending.get('valid',False)
    leads = [exposed[hid]-detected[hid] for hid in exposed if hid in detected]
    return {
        'scenario':metadata.get('scenario'),'seed':metadata.get('seed'),'policy':metadata.get('policy'),
        **condition,'config_sha256':metadata.get('config_sha256'),
        'status':'complete' if complete else 'incomplete',
        'physical_telemetry_present':sim_telemetry,
        'duration_s':None if first_stamp is None or last_time is None else last_time-first_stamp,
        'belief_accuracy':posterior_correct/posterior_samples if posterior_samples else None,
        'belief_brier_score':brier_sum/posterior_samples if posterior_samples else None,
        'belief_samples':posterior_samples,
        'detection_lead_time_s':sum(leads)/len(leads) if leads else None,
        'hazard_exposure_s':exposure_seconds if sim_telemetry else None,
        'entrapment':entrapment if complete else None,
        'time_to_safety_s':mission_time,
        'time_to_safety_censored':bool(complete and mission_time is None),
        'detected_incidents':len(detected),'exposed_incidents':len(exposed),
        'lead_time_samples':len(leads),
        'mission_completed':mission_time is not None if complete else None,
        'warning_precision':true_warnings/warnings if warnings else None,
        'warning_recall':len(warned & eligible)/len(eligible) if eligible else None,
        'missed_warnings':len(eligible-warned),'false_alarms':false_alarms,
        'interruptions':warnings,'eligible_mismatch_incidents':len(eligible),
        'passive_map_glances':passive_glances,
        'final_modeled_trust':trust,
        'simulated_compliance':sum(compliance)/len(compliance) if compliance else None,
        'compliance_samples':len(compliance),
    }
