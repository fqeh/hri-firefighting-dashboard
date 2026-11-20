import copy,json,math
from pathlib import Path
from hri_reasoning.forest_model import FireModel,fuel_at,inside_zone,plan_route,incident_sequence

ROOT=Path(__file__).resolve().parents[2]/'hri_sim/config/forest_scenarios'
def config(case='D'):return json.loads((ROOT/f'{case}.json').read_text())

def test_fire_changes_with_time_and_fuel():
 c=config();f=FireModel(c['fire'],7)
 assert not f.state(0)
 assert max(f.state(20)[0]['radii'])>max(f.state(5)[0]['radii'])
 barren=copy.deepcopy(c['fire']);barren['fuel']={'default':0,'patches':[]}
 assert max(FireModel(barren,7).state(20)[0]['radii'])==.6

def test_wind_and_seed_reproducible():
 c=config();a=FireModel(c['fire'],7).state(30);b=FireModel(c['fire'],7).state(30)
 assert a==b
 assert a[0]['radii'][3]>a[0]['radii'][9]
 f=FireModel(c['fire'],7)
 for t in range(31):result=f.state(t)
 assert result==a
 assert incident_sequence(c)==incident_sequence(c)

def test_inside_zone_wraps_at_zero_angle_boundary():
 zone={'center':[0,0],'radii':[1,2,3,4,5,6,7,8,9,10,11,12]}
 assert inside_zone([.5,-1e-300],zone)


def test_blockage_and_alternative():
 c=config('C');clear={e['id']:'clear' for e in c['edges']}
 a=plan_route(c,[0,-2],clear,[])
 b=plan_route(c,[0,-2],{**clear,'trail_a':'blocked'},[])
 assert 'trail_a' in a['edges'] and 'trail_a' not in b['edges']
 assert 'trail_b' in b['edges']
 assert b['verified']


def test_unknown_no_route_and_exposure_cost():
 c=config();unknown=plan_route(c,[0,-2],{},[])
 assert not unknown['verified'] and unknown['safety']=='conditional'
 none=plan_route(c,[0,-2],{'approach':'blocked'},[])
 assert none['recommended']==[] and none['safety']=='unavailable'
 clear={e['id']:'clear' for e in c['edges']}
 risk=plan_route(c,[0,-2],clear,FireModel(c['fire'],7).state(25))
 assert risk['exposure_costs']['trail_a']>0
 assert risk['edge_states']['trail_a']=='hazardous'
 assert 'trail_a' not in risk['edges']

def test_scenarios_and_knowledge_pair():
 a=config('A');assert not a['hazards'] and not a['fire']['origins']
 e,f=config('E'),config('F')
 assert e['hazards']==f['hazards'] and e['seed']==f['seed']
 assert e['crew_initial_knowledge']['trail_a']=='blocked' and not f['crew_initial_knowledge']
 for case in 'ABCDEF':
  c=config(case)
  for h in c['hazards']:
   assert h['edge'] in {x['id'] for x in c['edges']}
