#!/usr/bin/env python3
"""Author explicit A–F scenarios, preserving the original baseline config."""
import copy,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]/'ros2_ws/src/hri_sim/config'
base=json.loads((root/'forest.json').read_text())
for case,title in [('A','normal_evacuation'),('B','growing_fire'),('C','fallen_tree'),('D','fire_and_tree'),('E','known_hazard'),('F','unknown_hazard')]:
 c=copy.deepcopy(base)
 c.update(case=case,description=title,seed=7,crew_initial_knowledge={},exposure_weight=2,unknown_penalty=1.5)
 c['hazards']=[{**base['hazards'][0],'at':8.0}] if case in 'CDEF' else []
 c['fire']={'base_rate':.14,'max_radius':4.5,'wind':[0,1.5],'wind_gain':.4,
            'fuel':{'default':1.0,'patches':[{'center':[8,-2],'radius':3,'load':1.3},{'center':[4,6],'radius':3,'load':.2}]},
            'origins':[{'id':'hazard_fire','position':[8,-4,0],'at':4.0,'radius':.6,'exposure_cost':8}] if case in 'BD' else []}
 if c['fire']['origins']:
  c['hazards'].append({'id':'hazard_fire','model':'hazard_fire','edge':'trail_a','position':[8,-4,0],'at':4.0,'kind':'fire','radius':.6})
 c['uav']={'controller':'scripted_pose','speed':1.4,'waypoints':[[6,0,8],[10,0,8],[10,6,8],[4,6,8],[6,0,8]],'loop':True,'camera_hfov':1.2}
 if case=='E':c['crew_initial_knowledge']={'trail_a':'blocked'}
 (root/'forest_scenarios'/f'{case}.json').write_text(json.dumps(c,indent=2)+'\n')
