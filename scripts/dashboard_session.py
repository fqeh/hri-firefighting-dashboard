#!/usr/bin/env python3
"""Loopback session supervisor, independent of ROS launch lifetime and scenario.

No Docker socket is exposed to the browser. Only allowlisted scenarioctl commands
run, with argument arrays. The ROS gateway remains the sole telemetry source.
"""
import argparse
import copy
import json
import mimetypes
import os
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, unquote
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/hri_industrial'))
CASES={'forest':list('ABCDEF'),'oil_facility':['normal','blockage','fire','gas','combined','known','unaware','no_safe']}
POLICIES=['B1','B2','B3','B4','oracle']
EQUIPMENT_REGISTRY=Path('ros2_ws/src/hri_industrial/config/equipment_registry.json')

def configuration(state):
    scenario=state.get('scenario','forest');case=state.get('case','F')
    path=ROOT/('ros2_ws/src/hri_sim/config/forest_scenarios/'+case+'.json' if scenario=='forest' else 'ros2_ws/src/hri_industrial/config/scenarios/'+case+'.json')
    config=json.loads(path.read_text())
    active=ROOT/state.get('output','')/'scenario.json'
    if active.is_file():config=json.loads(active.read_text())
    # Explicit public/configuration allowlist; hidden knowledge never enters UI.
    public={k:config[k] for k in ('nodes','edges','goal','robot_start','uav','fire') if k in config}
    public['incidents']=[{'id':h['id'],'kind':h['kind'],'edge':h['edge'],
                          **({'display_name':h['display_name']} if isinstance(h.get('display_name'),str) else {})}
                         for h in config.get('hazards',[])]
    public['source']='scenario configuration / public map prior; not a live observation'
    if scenario=='oil_facility':
        import xml.etree.ElementTree as ET
        from hri_industrial.incident_config import add_tank_damage_hazard,load_tank_damage_incident
        industrial=ROOT/'ros2_ws/src/hri_industrial'
        layout=json.loads((industrial/'config/layout.json').read_text())
        incident=load_tank_damage_incident(industrial/'config/industrial_incidents.json',layout)
        if not any(item.get('event_id')==incident['event_id'] for item in config.get('hazards',[]) if isinstance(item,dict)):
            config=add_tank_damage_hazard(config,incident)
            public['incidents']=[{'id':h['id'],'kind':h['kind'],'edge':h['edge'],
                                  **({'display_name':h['display_name']} if isinstance(h.get('display_name'),str) else {})}
                                 for h in config.get('hazards',[])]
        world=ET.parse(ROOT/'ros2_ws/src/hri_industrial/worlds/oil_facility.sdf')
        public['personnel']=[list(map(float,m.findtext('pose').split()[:2])) for m in world.findall('.//model')
            if m.get('name','').startswith('personnel_') and not m.get('name','').startswith('personnel_helmet_')]
    public['battery']='not modeled' 
    public['warning_costs']=json.loads((ROOT/'ros2_ws/src/hri_reasoning/config/warning_policy.json').read_text())
    return public

def validate_command(data):
    if not isinstance(data,dict):raise ValueError('Expected command object')
    command=data.get('command')
    allowed={'switch','reset','pause','resume','start_experiment','stop_experiment','evaluation'}
    if command not in allowed:raise ValueError('Unsupported session command')
    fields={'command','scenario','case','seed','policy','engine','manual_incidents','duration','enabled','human_radius','map_rate','human_lapse'}
    if set(data)-fields:raise ValueError('Unexpected command fields')
    if command in ('switch','start_experiment'):
        if data.get('scenario') not in CASES or data.get('case') not in CASES[data['scenario']]:raise ValueError('Invalid scenario/case')
        if data.get('policy') not in POLICIES:raise ValueError('Invalid policy')
        if data.get('engine') not in ('expected_loss','value_iteration'):raise ValueError('Select a configured policy engine')
        if type(data.get('seed')) is not int or not 0<=data['seed']<=2147483647:raise ValueError('Seed must be an integer in [0, 2147483647]')
        if type(data.get('manual_incidents')) is not bool:raise ValueError('manual_incidents must be boolean')
    for key,maximum in (('human_radius',20),('map_rate',1),('human_lapse',1)):
        if key in data and (type(data[key]) not in (int,float) or not 0<=data[key]<=maximum):raise ValueError('Invalid human behavior parameter')
    if command=='start_experiment' and (type(data.get('duration')) not in (int,float) or not 10<=data['duration']<=600):raise ValueError('Experiment duration must be 10–600 simulation seconds')
    if command=='evaluation' and type(data.get('enabled')) is not bool:raise ValueError('enabled must be boolean')
    return data

def update_equipment_contents(data, registry_path):
    if not isinstance(data,dict) or set(data)!={'equipment_id','contents'}:
        raise ValueError('Expected equipment_id and contents only')
    equipment_id=data['equipment_id']
    contents=data['contents']
    fields={'label':120,'material_class':80,'source':200}
    if not isinstance(equipment_id,str) or not equipment_id or not isinstance(contents,dict) or set(contents)!=set(fields):
        raise ValueError('Invalid equipment contents update')
    for field,maximum in fields.items():
        value=contents[field]
        if (not isinstance(value,str) or not value.strip() or len(value)>maximum
                or any(ord(char)<32 and char not in '\n\t' for char in value)):
            raise ValueError(f'Invalid {field}')
    path=Path(registry_path)
    registry=json.loads(path.read_text(encoding='utf-8'))
    entries=registry.get('equipment')
    if not isinstance(entries,list):raise ValueError('Equipment registry has no equipment list')
    if any(not isinstance(entry,dict) for entry in entries):
        raise ValueError('Equipment registry contains an invalid entry')
    match=next((entry for entry in entries if entry.get('equipment_id')==equipment_id),None)
    if match is None:raise ValueError('Unknown equipment_id')
    match['contents']=dict(contents)
    temp_path=None
    try:
        with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=path.parent,delete=False) as file:
            temp_path=Path(file.name)
            json.dump(registry,file,indent=2,allow_nan=False)
            file.write('\n')
            file.flush()
            os.fsync(file.fileno())
        os.replace(temp_path,path)
    finally:
        if temp_path and temp_path.exists():temp_path.unlink()
    return registry

class Session:
    def __init__(self, root=ROOT, runner=subprocess.run):
        self.root=root;self.runner=runner;self.lock=threading.RLock();self.busy=False
        self.operation=None;self.error=None;self.state={};self.telemetry=None;self.ros_reachable=False
        self.experiment=None;self.evaluation=False;self.metrics=None;self.stopping=False
        self.reload()

    def reload(self):
        path=self.root/'results/scenario-session.json'
        if path.exists():self.state=json.loads(path.read_text())

    def snapshot(self):
        with self.lock:
            output=self.state.get('output');files=[]
            if output and (self.root/output).is_dir():
                files=[
                    {'name':p.name,'url':'/api/session/artifacts/'+str(p.relative_to(self.root))}
                    for p in (self.root/output).iterdir() if p.is_file() and p.suffix in ('.json','.jsonl','.log','.csv')
                    and (self.evaluation or p.name not in ('events.jsonl','scenario.json'))]
                if (self.root/output/'events.jsonl').exists():files.append({'name':'public-events.jsonl','url':'/api/session/artifacts/'+output+'/public-events.jsonl'})
            clock=(self.telemetry or {}).get('topics',{}).get('/clock',{}).get('data',{}).get('seconds')
            return {'schema_version':1,'session':copy.deepcopy(self.state),'busy':self.busy,'operation':self.operation,
                'error':self.error,'ros_reachable':self.ros_reachable,'sim_time':clock,
                'parameters':configuration(self.state) if self.state else {},'cases':CASES,'policies':POLICIES,
                'experiment':copy.deepcopy(self.experiment),'evaluation_enabled':self.evaluation,
                'evaluation':self.metrics,'artifacts':files}

    def submit(self, data):
        validate_command(data)
        with self.lock:
            if self.busy:raise RuntimeError('Another session operation is still running')
            if data['command']=='evaluation':
                self.evaluation=data['enabled'];self.metrics=None;return
            if self.experiment and self.experiment['state']=='running' and data['command'] in ('switch','reset','start_experiment'):
                raise RuntimeError('Stop the active experiment before restarting the scenario')
            self.busy=True;self.error=None;self.operation=data['command']
            threading.Thread(target=self.execute,args=(copy.deepcopy(data),),daemon=True).start()

    def execute(self, data):
        command=data['command'];args=[sys.executable,str(self.root/'scripts/scenarioctl.py')]
        try:
            if command in ('switch','start_experiment'):
                args+=['start','--scenario',data['scenario'],'--case',data['case'],'--seed',str(data['seed']),
                    '--policy',data['policy'],'--engine',data['engine']]
                if data['manual_incidents']:args+=['--manual-incidents']
                for key,arg in (('human_radius','--human-radius'),('map_rate','--map-rate'),('human_lapse','--human-lapse')):
                    if key in data:args += [arg,str(data[key])]
                if command=='start_experiment':args+=['--play']
            else:args+=['pause' if command=='stop_experiment' else command]
            result=self.runner(args,cwd=self.root,text=True,capture_output=True,timeout=150)
            if result.returncode:raise RuntimeError((result.stderr or result.stdout)[-3000:])
            with self.lock:
                self.reload()
                self.state['paused']=command not in ('resume','start_experiment')
                if command in ('switch','reset','start_experiment'):
                    self.telemetry=None;self.ros_reachable=False;self.metrics=None
                    self.state['generation']=str(uuid.uuid4())
                if command=='start_experiment':
                    self.experiment={'id':str(uuid.uuid4()),'state':'running','start':0.,'duration':data['duration'],'elapsed':0.,'progress':0.}
                elif command=='switch':self.experiment=None
                elif command=='stop_experiment' and self.experiment:
                    self.experiment['state']='stopped';self.save_report()
                (self.root/'results/scenario-session.json').write_text(json.dumps(self.state,indent=2)+'\n')
        except Exception as exc:
            with self.lock:self.error=str(exc)
        finally:
            with self.lock:self.busy=False

    def save_report(self):
        output=self.state.get('output')
        if output:
            report={'session':self.state,'experiment':self.experiment,'evaluation':self.metrics,
                'source':'Actual ROS/Gazebo event log; no synthetic telemetry','evaluation_enabled':self.evaluation}
            (self.root/output/'dashboard-experiment.json').write_text(json.dumps(report,indent=2)+'\n')

    def monitor(self):
        while not self.stopping:
            try:
                with self.lock:
                    if not self.busy:
                        old=self.state.get('output');self.reload()
                        if old and old!=self.state.get('output'):
                            self.telemetry=None;self.metrics=None
                            if self.experiment and self.experiment['state']=='running':self.experiment['state']='aborted_by_external_restart'
                with urlopen('http://127.0.0.1:8080/api/state',timeout=1.5) as response:snapshot=json.load(response)
                with self.lock:
                    if not self.busy:
                        self.telemetry=snapshot;self.ros_reachable=True
                        exp=self.experiment
                        now=snapshot.get('topics',{}).get('/clock',{}).get('data',{}).get('seconds',0)
                        if exp and exp['state']=='running':
                            exp['elapsed']=max(0,now-exp['start']);exp['progress']=min(1,exp['elapsed']/exp['duration'])
                            if exp['elapsed']>=exp['duration']:self.submit({'command':'stop_experiment'})
            except Exception:
                with self.lock:self.ros_reachable=False
            if self.evaluation and not self.busy:
                try:
                    from dashboard_evaluation import evaluate_log
                    path=self.root/self.state['output']/'events.jsonl'
                    self.metrics=evaluate_log(path)
                except Exception as exc:self.metrics={'available':False,'reason':str(exc)}
            time.sleep(1)

def handler(session):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,status,value,kind='application/json'):
            body=value if isinstance(value,bytes) else json.dumps(value,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(body)
        def do_GET(self):
            path=urlparse(self.path).path
            if path=='/api/session':self.reply(200,session.snapshot());return
            prefix='/api/session/artifacts/'
            if path.startswith(prefix):
                target=(session.root/unquote(path[len(prefix):])).resolve();output=session.state.get('output')
                directory=(session.root/output).resolve() if output else None
                if directory and target.parent==directory and target.name=='public-events.jsonl':
                    lines=[]
                    for line in (directory/'events.jsonl').read_text().splitlines():
                        try:
                            row=json.loads(line)
                            if row['topic'].startswith('/hri/') or row['topic'] in ('/robot/odom','/human/odom'):lines.append(json.dumps(row))
                        except (ValueError,KeyError):continue
                    self.reply(200,('\n'.join(lines)+'\n').encode(),'text/plain; charset=utf-8');return
                if target.name in ('events.jsonl','scenario.json') and not session.evaluation:
                    self.reply(403,{'error':'Enable evaluation truth to download private research logs'});return
                if directory and target.parent==directory and target.is_file() and target.suffix in ('.json','.jsonl','.log','.csv'):
                    self.reply(200,target.read_bytes(),'text/plain; charset=utf-8');return
            self.reply(404,{'error':'Unknown session endpoint'})
        def do_POST(self):
            path=urlparse(self.path).path
            if path not in ('/api/session/control','/api/session/equipment-registry'):
                self.reply(404,{'error':'Unknown endpoint'});return
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':self.reply(415,{'error':'Expected application/json'});return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=4096:raise ValueError('Invalid request size')
                data=json.loads(self.rfile.read(length))
                if path=='/api/session/equipment-registry':
                    with session.lock:
                        if session.busy:raise RuntimeError('Wait for the current session operation')
                        if session.experiment and session.experiment['state']=='running':
                            raise RuntimeError('Stop the active experiment before editing equipment metadata')
                        update_equipment_contents(data,session.root/EQUIPMENT_REGISTRY)
                else:session.submit(data)
            except (ValueError,TypeError) as exc:self.reply(400,{'error':str(exc)});return
            except RuntimeError as exc:self.reply(409,{'error':str(exc)});return
            except OSError as exc:self.reply(500,{'error':f'Cannot save equipment metadata: {exc}'});return
            self.reply(200 if path.endswith('equipment-registry') else 202,
                {'accepted':True,'note':'Registry metadata saved; ROS configuration telemetry will refresh.'}
                if path.endswith('equipment-registry') else
                {'accepted':True,'command':data['command'],'note':'Await session operation result'})
    return Handler

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port',type=int,default=8081);args=parser.parse_args()
    session=Session();threading.Thread(target=session.monitor,daemon=True).start()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),handler(session))
    print(f'Dashboard session supervisor: http://127.0.0.1:{args.port}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:session.stopping=True;server.server_close()

if __name__=='__main__':main()
