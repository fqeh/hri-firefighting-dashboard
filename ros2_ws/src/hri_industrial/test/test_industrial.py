import hashlib,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from hri_industrial.contracts import requested_path,safe_recommendation,navigation_mask,on_road
from hri_industrial.world import build_world
from hri_industrial.registry import load_equipment_registry
from hri_industrial.incident_config import (add_tank_damage_hazard,execute_tank_damage_aftermath,
    load_tank_damage_incident,tank_damage_hazard)
from hri_reasoning.tom import public_map,RouteToM

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT.parent
LAYOUT=json.loads((ROOT/'config/layout.json').read_text())
MAP=public_map(LAYOUT)
COST=json.loads((SRC/'hri_reasoning/config/warning_policy.json').read_text())['route']
def hazards(case):return json.loads((ROOT/f'config/scenarios/{case}.json').read_text())['hazards']
def plan(case,clear=None):return safe_recommendation(MAP,LAYOUT,[0,0],'assembly',hazards(case),
    LAYOUT['initial_survey_edges'] if clear is None else clear,COST)

def test_normal_and_alternatives():
    assert 'corridor_b' in plan('normal')['edges']
    assert 'corridor_b' not in plan('blockage')['edges']
    assert 'corridor_c' in plan('blockage')['edges']
    assert plan('fire')['selected_exposure']==0
    assert 'corridor_b' not in plan('fire')['edges']
    assert 'corridor_d' in plan('gas')['edges']
    assert 'corridor_c' not in plan('gas')['edges']

@pytest.mark.parametrize('case,clear',[('no_safe',None),('normal',[])])
def test_refuse_when_no_safe_known_route(case,clear):
    result=plan(case,clear)
    assert not result['navigation_allowed'] and not result['waypoints']

def test_offroad_connector_rejected():
    result=safe_recommendation(MAP,LAYOUT,[0,-4],'assembly',[],LAYOUT['initial_survey_edges'],COST)
    assert not result['navigation_allowed']

def test_midroad_pose_routes_from_its_surveyed_edge():
    result=safe_recommendation(MAP,LAYOUT,[8.28,-5],'assembly',hazards('combined'),
        LAYOUT['initial_survey_edges'],COST)
    assert result['navigation_allowed']
    assert result['recommended']==['south_service','assembly']
    assert 'south_road' in result['edges']
    assert 'corridor_b' not in result['edges']

def test_operator_evidence_changes_posterior_without_private_truth():
    b=RouteToM(MAP);c=RouteToM(MAP)
    path,branch=requested_path(MAP,[0,0],'assembly','corridor_b')
    alternate,other=requested_path(MAP,[0,0],'assembly','corridor_c')
    assert branch==('junction','corridor_b') and other==('junction','north_junction')
    for model,action in [(b,branch),(c,other)]:
        model.observe('corridor_b','blocked',8,[6,0],'debris')
        assert model.action(*action,10)
    pb=b.summary(['corridor_b'])['corridor_b'];pc=c.summary(['corridor_c'])['corridor_b']
    assert pb['p_outdated']>pc['p_outdated']
    assert sum(pb['posterior'].values())==pytest.approx(1)

def test_mask_and_geometry(tmp_path):
    config=json.loads((ROOT/'config/scenarios/normal.json').read_text())
    sdf=build_world(SRC/'hri_sim',LAYOUT,config,tmp_path)
    world=ET.parse(sdf).find('world');models={m.attrib['name']:m for m in world.findall('model')}
    for name in ('tank_0','pipeline_0','control_building','process_column_0','corridor_b_barrier_1.5','personnel_0'):
        assert models[name].find('.//collision') is not None
    for edge in LAYOUT['edges']:
        a,b=[LAYOUT['nodes'][edge[k]] for k in ('source','target')]
        road=models['route_'+edge['id']];pose=list(map(float,road.findtext('pose').split()))
        assert pose[:2]==pytest.approx([(a[0]+b[0])/2,(a[1]+b[1])/2])
        assert float(road.findtext('.//visual/geometry/box/size').split()[0])==pytest.approx(math.dist(a,b))
    width,height,mask=navigation_mask(LAYOUT,hazards('gas'))
    def cell(x,y):return mask[int((y-LAYOUT['bounds'][1])/LAYOUT['mask_resolution'])*width+int((x-LAYOUT['bounds'][0])/LAYOUT['mask_resolution'])]
    assert cell(0,0)==0 and cell(6,0)==100 and cell(3,3)==100 and cell(6,3.6)==100
    robot=ET.parse(tmp_path/'oil_robot/model.sdf')
    assert robot.find('.//inertial') is not None and robot.find('.//collision') is not None
    assert robot.find('.//sensor[@type="gpu_lidar"]') is not None

def test_equipment_registry_matches_generated_models_and_marks_unmodeled_values(tmp_path):
    config=json.loads((ROOT/'config/scenarios/normal.json').read_text())
    sdf=build_world(SRC/'hri_sim',LAYOUT,config,tmp_path)
    registry=load_equipment_registry(ROOT/'config/equipment_registry.json',sdf)
    entries=registry['equipment']
    assert len(entries)==13
    assert registry['source']=='configuration'
    assert len({entry['equipment_id'] for entry in entries})==len(entries)
    assert len({entry['gazebo_model'] for entry in entries})==len(entries)
    assert all(entry['inventory']['value'] is None for entry in entries)
    assert all(entry['temperature']['status']=='not_modeled' for entry in entries)
    assert all(entry['pressure']['status']=='not_modeled' for entry in entries)

def test_tank_damage_aftermath_is_manual_only_and_preserves_existing_scenarios():
    incident=load_tank_damage_incident(ROOT/'config/industrial_incidents.json',LAYOUT)
    original=json.loads((ROOT/'config/scenarios/normal.json').read_text())
    updated=add_tank_damage_hazard(original,incident)
    assert len(original['hazards'])==0
    assert updated['hazards'][-1]['event_type']=='tank_damage_aftermath'
    assert updated['hazards'][-1]['manual_only'] is True
    assert updated['hazards'][-1]['at']>1e8
    assert updated['hazards'][-1]['equipment_id']=='TK-01'
    assert updated['hazards'][-1]['edge']=='north_road'
    for name in ('blockage','fire','gas','combined','known','unaware','no_safe'):
        assert json.loads((ROOT/f'config/scenarios/{name}.json').read_text())['hazards']
    with pytest.raises(ValueError):
        add_tank_damage_hazard(updated,incident)

def test_tank_damage_aftermath_sdf_is_visual_only_and_route_hazard_avoids_north_road():
    incident=load_tank_damage_incident(ROOT/'config/industrial_incidents.json',LAYOUT)
    damage=tank_damage_hazard(incident)
    sdf=ET.parse(ROOT/'models/tank_damage_visual/model.sdf')
    assert sdf.find('.//model[@name="tank_damage_visual"]') is not None
    assert sdf.find('.//visual[@name="darkened_tank_surface"]') is not None
    assert sdf.find('.//collision') is None
    result=safe_recommendation(MAP,LAYOUT,[0,0],'assembly',[damage],
        LAYOUT['initial_survey_edges'],COST)
    assert result['navigation_allowed']
    assert 'north_road' not in result['edges']
    assert 'south_road' in result['edges'] or 'corridor_b' in result['edges']

def test_tank_damage_aftermath_creation_is_repeatable_and_logs_ordered_events(tmp_path):
    incident=load_tank_damage_incident(ROOT/'config/industrial_incidents.json',LAYOUT)
    hazard=tank_damage_hazard(incident)
    spawned=set();completed=set();events=[]
    def spawn(path,name,position,yaw):
        assert path.is_absolute()
        assert len(position)==3
        spawned.add(name)
        return True
    publish=lambda topic,data:events.append((topic,data))
    assert execute_tank_damage_aftermath(hazard,tmp_path,tmp_path,spawn,publish,completed,0.0)
    assert spawned=={'tank_0_damage_visual',hazard['id']}
    assert [entry[1]['event'] for entry in events]==[
        'tank_damage_aftermath_requested','tank_damage_aftermath_created']
    created=events[-1][1]
    assert created['affected_edge']=='north_road'
    assert 'no impact dynamics' in created['limitations'].lower()
    assert 'temperature' in created['limitations'].lower()
    assert not any(key in created for key in ('temperature','pressure','inventory','integrity'))
    assert not execute_tank_damage_aftermath(hazard,tmp_path,tmp_path,spawn,publish,completed,0.0)
    assert len(events)==2

def test_tank_damage_aftermath_failure_is_logged_and_retryable(tmp_path):
    incident=load_tank_damage_incident(ROOT/'config/industrial_incidents.json',LAYOUT)
    hazard=tank_damage_hazard(incident)
    completed=set();events=[];attempts=[]
    def spawn(path,name,position,yaw):
        attempts.append(name)
        if name==hazard['id'] and attempts.count(name)==1:
            raise RuntimeError('Gazebo create service unavailable')
        return True
    publish=lambda topic,data:events.append((topic,data))
    with pytest.raises(RuntimeError,match='Gazebo create service unavailable'):
        execute_tank_damage_aftermath(hazard,tmp_path,tmp_path,spawn,publish,completed,0.0)
    assert events[-1][1]['event']=='tank_damage_aftermath_failed'
    assert not completed
    assert execute_tank_damage_aftermath(hazard,tmp_path,tmp_path,spawn,publish,completed,0.0)
    assert completed=={hazard['id']}

def test_research_core_unchanged_and_inference_inputs_public():
    for name,digest in json.loads((ROOT/'config/reused_core_sha256.json').read_text()).items():
        assert hashlib.sha256((SRC/'hri_reasoning/hri_reasoning'/Path(name).name).read_bytes()).hexdigest()==digest
    for name in ('operator_inference.py','perception.py','routing.py','operator_input.py'):
        source=(ROOT/'hri_industrial'/name).read_text()
        assert '/evaluation/' not in source and 'operator_initial_knowledge' not in source
    source=(ROOT/'hri_industrial/launching.py').read_text()
    assert "'public_map':str(path)" in source

def test_paired_trials_differ_only_in_private_knowledge():
    known=json.loads((ROOT/'config/scenarios/known.json').read_text())
    unaware=json.loads((ROOT/'config/scenarios/unaware.json').read_text())
    assert known.pop('operator_initial_knowledge')!=unaware.pop('operator_initial_knowledge')
    known.pop('case');unaware.pop('case')
    assert known==unaware
