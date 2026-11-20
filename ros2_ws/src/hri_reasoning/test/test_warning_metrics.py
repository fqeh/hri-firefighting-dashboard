"""Evaluation tests keep private truth out of the deployed policy package."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'evaluation'))
from warning_metrics import evaluate


def records(known=False):
    config=json.loads((ROOT/'ros2_ws/src/hri_sim/config/forest_scenarios/F.json').read_text())
    knowledge={e['id']:'open_safe' for e in config['edges']}
    if known:knowledge['trail_a']='blocked'
    hazard=config['hazards'][0]
    return config,knowledge,[
        {'topic':'run_metadata','data':{'config':config}},
        {'topic':'/evaluation/human_belief','data':{'stamp':0.,'knowledge':knowledge}},
        {'topic':'/evaluation/truth','data':{'stamp':1.,'event':'hazard_created','hazard':hazard}},
        {'topic':'/human/odom','data':{'stamp':2.,'position':[0,-2,0]}}]


def test_precision_uses_pre_response_belief_despite_delivery_order():
    config,before,rows=records()
    after={**before,'trail_a':'blocked'}
    # A logger can receive the response before the warning at the same stamp.
    rows.extend([
        {'topic':'/evaluation/human_belief','data':{'stamp':3.,'warning_id':'B4-1',
            'knowledge_before':before,'knowledge':after,'complied':True,'route_changed':True}},
        {'topic':'/hri/warning','data':{'stamp':3.,'id':'B4-1','hazard_id':config['hazards'][0]['id']}},
        {'topic':'/human/odom','data':{'stamp':4.,'position':[0,-2,0]}}])
    result=evaluate(rows)
    assert result['warning_precision']==1 and result['warning_recall']==1
    assert result['modeled_compliance']==1 and result['mean_time_to_warning_s']==2.


def test_known_hazard_alert_is_false_alarm_and_recall_is_undefined():
    config,_,rows=records(known=True)
    rows.append({'topic':'/hri/warning','data':{'stamp':3.,'id':'B3-1','hazard_id':config['hazards'][0]['id']}})
    result=evaluate(rows)
    assert result['false_alarms']==1 and result['warning_precision']==0
    assert result['dangerous_episodes']==0 and result['warning_recall'] is None


def test_passive_unaware_episode_is_missed_even_when_crew_later_escapes():
    _,_,rows=records()
    rows.append({'topic':'/human/odom','data':{'stamp':40.,'position':[16,6,0]}})
    result=evaluate(rows)
    assert result['missed_warnings']==1 and result['warning_recall']==0
    assert result['warning_precision'] is None and result['evacuation_completed']


def test_exposure_uses_growing_fire_zone_in_world_coordinates():
    _,_,rows=records()
    zone={'center':[2,-2],'radii':[.5]*12}
    rows.extend([
        {'topic':'/forest/fire_state','data':{'stamp':2.1,'zones':[zone]}},
        {'topic':'/human/odom','data':{'stamp':2.2,'position':[0,-2,0]}},
        {'topic':'/forest/fire_state','data':{'stamp':2.3,'zones':[{**zone,'radii':[3.]*12}]}},
        {'topic':'/human/odom','data':{'stamp':2.4,'position':[0,-2,0]}}])
    assert abs(evaluate(rows)['exposure_seconds']-.2)<1e-8


def test_blockage_replan_latency_does_not_start_at_earlier_fire_observation():
    config,_,rows=records()
    tree=config['hazards'][0];fire={**tree,'kind':'fire','id':'fire'}
    rows.extend([
        {'topic':'/hri/observations','data':{'stamp':2.,'hazards':[fire]}},
        {'topic':'/hri/observations','data':{'stamp':8.,'hazards':[tree]}},
        {'topic':'/hri/rescue_route','data':{'stamp':8.5,'revision':1,'blocked_edges':['trail_a'],
            'edges':['trail_b'],'replan_duration_ms':1.}}])
    assert evaluate(rows)['first_blockage_replan_latency_s']==.5
