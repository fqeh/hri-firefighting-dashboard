#!/usr/bin/env python3
"""Run isolated real Gazebo A and paired D forest tests; preserve each run."""
import argparse,json,os,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from phase1_acceptance import terminate
p=argparse.ArgumentParser();p.add_argument('--output',default='results/phase2-acceptance');a=p.parse_args();root=Path(a.output)
root.mkdir(parents=True,exist_ok=False)
reports=[]
for label,case in [('normal','A'),('combined','D'),('replay','D')]:
 out=root/label;out.mkdir()
 env=dict(os.environ,ROS_DOMAIN_ID='48',GZ_PARTITION=f'forest_test_{os.getpid()}_{label}')
 cmd=['ros2','launch','hri_bringup','demo.launch.py','scenario:=forest',f'forest_case:={case}','forest_ground_robot:=true','dashboard:=false','mission:=false',f'log_path:={out}/events.jsonl']
 with (out/'simulator.log').open('w') as log:
  sim=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   with (out/'probe.log').open('w') as probe:
    result=subprocess.run(['python3','scripts/verify_forest.py','--case',case,'--output',str(out)],env=env,stdout=probe,stderr=subprocess.STDOUT,timeout=330)
  finally:terminate(sim)
 r=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {'status':'failed','error':'probe did not produce result'}
 r.update(launch_command=cmd,probe_exit_code=result.returncode,launch_exit_code=sim.returncode)
 r['critical_log_lines']=[line for line in (out/'simulator.log').read_text().splitlines() if any(token in line for token in ('[ERROR]','[Err]','Traceback','Segmentation fault'))]
 if r['critical_log_lines']:r['status']='failed'
 reports.append(r);print(label,r['status'],flush=True)
replay=reports[1].get('configured_event_sequence')==reports[2].get('configured_event_sequence') and bool(reports[1].get('configured_event_sequence'))
result={'status':'passed' if all(r['status']=='passed' for r in reports) and replay else 'failed','replay_configured_events_identical':replay,'runs':reports}
(root/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
raise SystemExit(0 if result['status']=='passed' else 1)
