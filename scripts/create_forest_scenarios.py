#!/usr/bin/env python3
"""Author explicit A–F scenarios, preserving the original baseline config."""
import copy,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]/'ros2_ws/src/hri_sim/config'
base=json.loads((root/'forest.json').read_text())
for case,title in [('A','normal_evacuation'),('B','growing_fire'),('C','fallen_tree'),('D','fire_and_tree'),('E','known_hazard'),('F','unknown_hazard')]:
 c=copy.deepcopy(base)
 c.update(case=case,description=title,seed=7,crew_initial_knowledge={},exposure_weight=2,unknown_penalty=1.5)
 c['hazards']=[{**base['hazards'][0],'at':8.0,'display_name':'Fallen tree on Trail A'}] if case in 'CDEF' else []
 c['fire']={'base_rate':.14,'max_radius':4.5,'wind':[0,1.5],'wind_gain':.4,
            'fuel':{'default':1.0,'patches':[{'center':[8,-2],'radius':3,'load':1.3},{'center':[4,6],'radius':3,'load':.2}]},
            'origins':[{'id':'hazard_fire','position':[8,-4,0],'at':4.0,'radius':.6,'exposure_cost':8}] if case in 'BD' else []}
 if c['fire']['origins']:
  c['hazards'].append({'id':'hazard_fire','model':'hazard_fire','edge':'trail_a','position':[8,-4,0],'at':4.0,'kind':'fire','radius':.6,'display_name':'Fire beside Trail A'})
 # Patrol: the original northern loop, then a southern sweep that verifies Trails C/D and Exit D.
 c['uav']={'controller':'scripted_pose','speed':1.4,'waypoints':[[6,0,8],[10,0,8],[10,6,8],[4,6,8],[6,0,8],[6,-5,8],[16,-5,8],[16,0,8],[6,0,8]],'loop':True,'camera_hfov':1.2,
           'observe_dwell_s':5.0}  # look closer, then keep patrolling
 # The UAV is the only observer (wildfire lookout); no parked ground robot in the forest.
 # Starting over Trail B puts the patrol above Trail A when the tree falls (t = 8 s).
 c['ground_robot']=False
 # Crew (and the ToM assumption) avoid trails believed hazardous: 8x their length.
 c['hazard_multiplier']=8.0
 c['uav_start']=[4,6,8]
 if case=='E':c['crew_initial_knowledge']={'trail_a':'blocked'}
 if case in 'EF':
  # Demo incidents (manual_only, injected from the dashboard). With the scheduled tree on
  # Trail A they can close all four escape routes, which escalates to the human operator.
  c['hazards']+=[
   {'id':'hazard_tree_b','model':'hazard_tree','edge':'trail_b','position':[4,2,.45],'yaw':1.5707963,'at':1e9,
    'kind':'tree','radius':2.0,'manual_only':True,'display_name':'Fallen tree on Trail B'},
   {'id':'hazard_fire_smoke_c','model':'hazard_fire_smoke','edge':'trail_c','position':[4,-5.5,0],'at':1e9,
    'kind':'fire','state':'blocked','radius':2.0,'manual_only':True,'display_name':'Fire and smoke on Trail C'},
   {'id':'hazard_fire_smoke_d','model':'hazard_fire_smoke','edge':'trail_d','position':[-.6,4,0],'at':1e9,
    'kind':'fire','state':'blocked','radius':2.0,'manual_only':True,'display_name':'Fire and smoke on Trail D'}]
 (root/'forest_scenarios'/f'{case}.json').write_text(json.dumps(c,indent=2)+'\n')
