import copy,json,math
from pathlib import Path
from hri_reasoning.forest_model import FireModel,fuel_at,inside_zone,plan_route,incident_sequence
from hri_reasoning.tom import semantic_occluder,semantic_hazard_kind,occluded,SMOKE_RADIUS

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
 assert plan_route(c,[0,-2],{'approach':'blocked'},[])['recommended'][:2]==['start','trail_d']  # north route from the start
 none=plan_route(c,[0,-2],{'approach':'blocked','trail_d':'blocked'},[])
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

def test_demo_incidents_can_close_all_four_escape_routes_without_changing_the_scheduled_run():
 for case in 'EF':
  c=config(case)
  assert [(h['id'],h['at']) for h in c['hazards'] if not h.get('manual_only')]==[('hazard_tree',8.0)]
  manual={h['id']:h for h in c['hazards'] if h.get('manual_only')}
  assert {h['edge'] for h in manual.values()}=={'trail_b','trail_c','trail_d'}
  assert manual['hazard_tree_b']['yaw']==1.5707963  # lies across Trail B
  assert all(manual[k]['state']=='blocked' and manual[k]['model']=='hazard_fire_smoke' for k in ('hazard_fire_smoke_c','hazard_fire_smoke_d'))
  assert c['fire']['origins']==[]

def test_smoke_occludes_observations_but_is_not_a_hazard():
 assert semantic_occluder('hazard_smoke')=='smoke' and semantic_hazard_kind('hazard_smoke') is None
 assert semantic_occluder('hazard_tree') is None and semantic_hazard_kind('hazard_tree')=='tree'
 cloud=[[7,-2,0]]
 assert occluded((7,-2),cloud) and occluded((9.5,-2),cloud) and not occluded((10.6,-2),cloud)

def test_four_escape_routes_and_the_operator_escalation_condition():
 from hri_reasoning.core import RouteGraph
 from hri_reasoning.tom import route,hazard_route_state
 from hri_reasoning.route_risk import recommend,escape_route_exists
 c=config('F');public={k:c[k] for k in ('nodes','edges','start','goal')};g=RouteGraph(public)
 path_edges=lambda path:{e for a,b in zip(path,path[1:]) for n,e in g.adjacency[a] if n==b}
 # A blocked and the bypass on fire: the crew turns back for a long route instead of crossing
 assert c['hazard_multiplier']==8.0
 path,_=route(g,'trail_b','safety',{'trail_a':'blocked','bypass':'open_hazardous'},hazard_multiplier=c['hazard_multiplier'])
 assert path[-1]=='safety' and not {'trail_a','bypass'}&{e.id for e in path_edges(path)}
 assert route(g,'fork','safety',{'trail_a':'blocked'})[0]==['fork','trail_b','merge','safety']  # unchanged when B is clear
 settings=json.loads((Path(__file__).resolve().parents[1]/'config/warning_policy.json').read_text())['route']
 tree={'id':'hazard_tree','kind':'tree','edge':'trail_a','position':[7,-2,.45],'radius':2,'state':'blocked'}
 fires=[{'id':f'f{i}','kind':'fire','edge':'bypass','position':[5+i*.8,6,0],'radius':.6,'state':'open_hazardous'} for i in range(8)]
 plan=recommend(public,[4,6],[tree]+fires,['approach','trail_b','bypass'],settings)
 assert plan['recommended'][-1]=='safety' and 'merge' not in plan['recommended']
 # escalation: only when every route is closed (A and B and C and D)
 assert hazard_route_state('hazard_fire_smoke_c','fire')=='blocked' and hazard_route_state('hazard_fire_front_0_3','fire')=='open_hazardous'
 assert escape_route_exists(public,'fork',{'trail_a','trail_b','trail_c'})  # D is still open
 assert not escape_route_exists(public,'fork',{'trail_a','trail_b','trail_c','trail_d'})
 assert escape_route_exists(public,'safety',{'trail_a','trail_b','trail_c','trail_d'})
