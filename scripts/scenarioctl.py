#!/usr/bin/env python3
"""One command selects either native Gazebo environment and resets all ROS state."""
import argparse,json,os,subprocess,time
from pathlib import Path

root=Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0,str(root/'ros2_ws/src/hri_industrial'))
from hri_industrial.incident_config import add_tank_damage_hazard,load_tank_damage_incident
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('action',choices=['start','pause','resume','reset','replay','stop','status'])
p.add_argument('--scenario',choices=['forest','oil_facility'])
p.add_argument('--case',help='Forest A–F; oil normal/blockage/fire/gas/combined/known/unaware/no_safe')
p.add_argument('--seed',type=int);p.add_argument('--policy',choices=['B1','B2','B3','B4','oracle'])
p.add_argument('--engine',choices=['expected_loss','value_iteration','q_learning'])
p.add_argument('--simulated-operator',action='store_true',help='Seeded trial operator; interactive approval stays required')
p.add_argument('--play',action='store_true')
p.add_argument('--human-radius',type=float)
p.add_argument('--map-rate',type=float)
p.add_argument('--human-lapse',type=float)
p.add_argument('--manual-incidents',action='store_true',help='Disable scheduled incidents; predefined incidents remain injectable')
a=p.parse_args();statefile=root/'results/scenario-session.json'
state=json.loads(statefile.read_text()) if statefile.exists() else {'scenario':'oil_facility','case':'blockage','seed':7,'policy':'B4','engine':'expected_loss'}
base=['docker','compose','-f','compose.yaml','-f','compose.gui.yaml','--profile','gui']
if a.action in ('start','reset','replay'):
    scenario=a.scenario or state['scenario']
    case=a.case or (state['case'] if scenario==state['scenario'] else ('D' if scenario=='forest' else 'blockage'))
    allowed=list('ABCDEF') if scenario=='forest' else ['normal','blockage','fire','gas','combined','known','unaware','no_safe']
    if case not in allowed:p.error(f'{scenario} case must be one of {allowed}')
    state.update(scenario=scenario,case=case,seed=a.seed if a.seed is not None else state['seed'],
        policy=a.policy or state['policy'],engine=a.engine or state['engine'])
    if a.action=='start':
        state['simulated_operator']=a.simulated_operator
        state['manual_incidents']=a.manual_incidents
    for field,key,default in (('human_radius','human_radius',3.0),('map_rate','map_rate',0.05),('human_lapse','human_lapse',0.03)):
        value=getattr(a,field)
        state[key]=value if value is not None else state.get(key,default)
    output=f'results/{scenario}_runs/{time.time_ns()}_{case}';(root/output).mkdir(parents=True)
    state.update(output=output,paused_at_start=not a.play,paused=not a.play)
    env=dict(os.environ,HRI_GUI_SCENARIO=scenario,HRI_GUI_MISSION='false',HRI_GUI_DASHBOARD='true',
        HRI_GUI_START_PAUSED=str(not a.play).lower(),HRI_GUI_OUTPUT='/work/'+output,HRI_SCENARIO_CONFIG='',
        HRI_FOREST_CASE=case if scenario=='forest' else '',HRI_OIL_CASE=case if scenario=='oil_facility' else 'blockage',
        HRI_FOREST_SEED=str(state['seed']),HRI_POLICY=state['policy'],HRI_WARNING_ENGINE=state['engine'],
        HRI_NAV2='true' if scenario=='oil_facility' else 'false',
        HRI_SIMULATED_OPERATOR=str(state.get('simulated_operator',False)).lower(),HRI_OBSERVE='true',
        HRI_HUMAN_RADIUS=str(state['human_radius']),HRI_MAP_RATE=str(state['map_rate']),HRI_HUMAN_LAPSE=str(state['human_lapse']))
    if state.get('manual_incidents') or scenario=='oil_facility':
        source=root/('ros2_ws/src/hri_sim/config/forest_scenarios/'+case+'.json' if scenario=='forest' else 'ros2_ws/src/hri_industrial/config/scenarios/'+case+'.json')
        config=json.loads(source.read_text())
        if state.get('manual_incidents'):
            config['manual_incidents']=True
            for hazard in config.get('hazards',[]):hazard['at']=1e9
            for origin in config.get('fire',{}).get('origins',[]):origin['at']=1e9
        if scenario=='oil_facility':
            industrial=root/'ros2_ws/src/hri_industrial'
            layout=json.loads((industrial/'config/layout.json').read_text())
            incident=load_tank_damage_incident(industrial/'config/industrial_incidents.json',layout)
            config=add_tank_damage_hazard(config,incident)
        config['seed']=state['seed']
        (root/output/'scenario.json').write_text(json.dumps(config,indent=2)+'\n')
        env['HRI_SCENARIO_CONFIG']='/work/'+output+'/scenario.json'
    subprocess.run(base+['up','-d','--force-recreate','gui'],cwd=root,env=env,check=True)
    statefile.write_text(json.dumps(state,indent=2)+'\n')
    print('Gazebo: http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale')
    print('Dashboard: http://127.0.0.1:5173 ·',output)
elif a.action in ('pause','resume'):
    subprocess.run(base+['exec','-T','gui','/hri-entrypoint.sh','gz','service','-s',f'/world/{state["scenario"]}/control',
        '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req',
        'pause: '+str(a.action=='pause').lower()],cwd=root,check=True)
    state['paused']=a.action=='pause'
    statefile.write_text(json.dumps(state,indent=2)+'\n')
elif a.action=='stop':subprocess.run(base+['stop','gui'],cwd=root,check=True)
else:
    subprocess.run(base+['ps','gui'],cwd=root,check=True);print(json.dumps(state,indent=2))
