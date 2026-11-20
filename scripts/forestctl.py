#!/usr/bin/env python3
"""Programmatic full-process scenario lifecycle; reset never resets physics alone."""
import argparse,json,os,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('action',choices=['start','pause','resume','reset','replay','stop','status'])
p.add_argument('--case',choices=list('ABCDEF'),default=None)
p.add_argument('--seed',type=int,default=None)
p.add_argument('--belief',choices=['known','unaware'],help='Matched Phase 3 tree scenario; inference has no private condition label')
p.add_argument('--policy',choices=['B1','B2','B3','B4','oracle'])
p.add_argument('--engine',choices=['expected_loss','value_iteration','q_learning','legacy'])
p.add_argument('--observe',choices=['true','false'])
p.add_argument('--human-radius',type=float)
p.add_argument('--map-rate',type=float,help='Simulated passive dashboard glances per second')
p.add_argument('--lapse',type=float,help='Simulated decision-noise probability')
p.add_argument('--trust',type=float,help='Initial modeled compliance/trust variable')
p.add_argument('--play',action='store_true',help='Start immediately instead of waiting for GUI Play')
a=p.parse_args();statefile=root/'results/forest-session.json'
state=json.loads(statefile.read_text()) if statefile.exists() else {'case':'D','seed':7}
base=['docker','compose','-f','compose.yaml','-f','compose.gui.yaml','--profile','gui']
env=dict(os.environ)
if a.action in ('start','reset','replay'):
 state.update(case=a.case or state['case'],seed=a.seed if a.seed is not None else state['seed'])
 if a.belief:state.update(belief=a.belief,case='F')
 elif a.case:state.pop('belief',None)
 state['policy']=a.policy or ('B4' if a.case else state.get('policy','B1' if state.get('belief') else 'B4'))
 state['engine']=a.engine or state.get('engine','expected_loss')
 state['observe']=a.observe or state.get('observe','true')
 output=f'results/forest_runs/{time.time_ns()}_{state["case"]}'
 (root/output).mkdir(parents=True)
 env.update(HRI_FOREST_CASE=state['case'],HRI_FOREST_SEED=str(state['seed']),HRI_GUI_MISSION='false',
            HRI_GUI_START_PAUSED=str(not a.play).lower(),HRI_GUI_OUTPUT='/work/'+output)
 if state.get('belief'):
  config=json.loads((root/'ros2_ws/src/hri_sim/config/forest_scenarios/F.json').read_text())
  config['hazards'][0]['at']=1.
  config['crew_initial_knowledge']={'trail_a':'blocked'} if state['belief']=='known' else {}
  config['seed']=state['seed']
  (root/output/'scenario.json').write_text(json.dumps(config,indent=2)+'\n')
  env.update(HRI_SCENARIO_CONFIG='/work/'+output+'/scenario.json',
             HRI_HUMAN_RADIUS='1.0',HRI_MAP_RATE='0.0',HRI_HUMAN_LAPSE='0.0')
 if a.human_radius is not None:state['human_radius']=a.human_radius
 if state.get('human_radius') is not None:env['HRI_HUMAN_RADIUS']=str(state['human_radius'])
 for field,key in (('map_rate','HRI_MAP_RATE'),('lapse','HRI_HUMAN_LAPSE'),('trust','HRI_HUMAN_TRUST')):
  value=getattr(a,field)
  if value is not None:state[field]=value
  if field in state:env[key]=str(state[field])
 env.update(HRI_POLICY=state['policy'],HRI_WARNING_ENGINE=state['engine'],HRI_OBSERVE=state['observe'])
 state.update(output=output,paused_at_start=not a.play)
 subprocess.run(base+['up','-d','--force-recreate','gui'],cwd=root,env=env,check=True)
 statefile.write_text(json.dumps(state,indent=2)+'\n')
 print('Gazebo: http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale')
 print('Dashboard: http://127.0.0.1:5173 ·',output)
elif a.action in ('pause','resume'):
 subprocess.run(base+['exec','-T','gui','/hri-entrypoint.sh','gz','service','-s','/world/forest/control',
  '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req',
  'pause: '+str(a.action=='pause').lower()],cwd=root,check=True)
elif a.action=='stop':subprocess.run(base+['stop','gui'],cwd=root,check=True)
else:
 subprocess.run(base+['ps','gui'],cwd=root,check=True);print(json.dumps(state,indent=2))
