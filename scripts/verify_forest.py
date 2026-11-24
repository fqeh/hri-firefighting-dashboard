#!/usr/bin/env python3
"""Validate physical forest telemetry, a solid log collision, and rerouting."""
import argparse,csv,json,math,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from hri_bringup.verify_telemetry import Probe,position

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--case',default='D');args=p.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=Probe();hist={};poses={'human':[],'uav':[]}
for name in poses:
 def record(msg,n=name):
  node.receive(n,msg);poses[n].append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,*position(msg)])
 node.create_subscription(Odometry,f'/{name}/odom',record,qos_profile_sensor_data)
node.create_subscription(Image,'/uav/camera/image',lambda msg:node.receive('uav_image',msg),qos_profile_sensor_data)
for topic in ('/forest/routes','/forest/fire_state','/evaluation/truth','/evaluation/human_belief','/hri/events','/hri/uav_status'):
 node.create_subscription(String,topic,lambda msg,t=topic:hist.setdefault(t,[]).append(json.loads(msg.data)),20)
goal_pub=node.create_publisher(PoseStamped,'/uav/observation_goal',10)
report={'status':'failed','case':args.case};placed=False;sent=False;contact=[]
try:
 assert node.spin_until(lambda:all(n in node.latest for n in ('clock','human','uav','world_pose','image','uav_image')),90),'missing physical sensors'
 deadline=time.monotonic()+240
 while node.simulation_time()<32 and time.monotonic()<deadline:
  now=node.simulation_time()
  if args.case=='D' and now>=10 and not placed:
   result=subprocess.run(['gz','service','-s','/world/forest/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','5000','--req','name: "robot" position: {x: 5 y: -2 z: .02} orientation: {w: 1}'],capture_output=True,text=True,timeout=8)
   assert 'data: true' in result.stdout,'collision fixture placement failed'
   placed=True
  if placed and 10<now<21:
   node.command(.35)
   if now>18:contact.append([node.stamps['world_pose'][-1],*position(node.latest['world_pose'])])
  else:node.command(0.0)
  if now>=21 and not sent:
   msg=PoseStamped();msg.header.frame_id='world';msg.pose.position.x=6.;msg.pose.position.y=0.;msg.pose.position.z=8.;msg.pose.orientation.w=1.
   goal_pub.publish(msg);sent=True
  rclpy.spin_once(node,timeout_sec=.02)
 assert node.simulation_time()>=32,'simulation did not advance'
 assert all(len(v)>50 for v in poses.values()),'insufficient actual poses'
 displacement={n:max(math.dist(v[0][1:],row[1:]) for row in v) for n,v in poses.items()}
 assert all(d>1 for d in displacement.values()),displacement
 assert node.counts['uav_image']>10 and len(set(node.latest['uav_image'].data))>8
 # The UAV now visits an observation goal and then resumes its patrol (observe_dwell_s),
 # so check that it reached the goal after the command rather than where it ends up.
 after=[row for row in poses['uav'] if row[0]>=21]
 closest=min(after,key=lambda row:math.dist(row[1:],[6,0,8])) if after else None
 assert closest and math.dist(closest[1:],[6,0,8])<.35,'UAV did not reach commanded observation goal'
 routes=hist.get('/forest/routes',[]);assert routes,'no route telemetry'
 sequence=[r['hazard'] for r in hist.get('/evaluation/truth',[]) if r.get('event')=='hazard_created']
 if args.case=='D':
  assert {h['id'] for h in sequence}=={'hazard_tree','hazard_fire'}
  assert any('trail_a' in r['edges'] for r in routes),'original trail was not initially selected'
  assert any(r['edge_states']['trail_a']=='blocked' and 'trail_a' not in r['edges'] and r['recommended'] for r in routes),'no alternative after blockage'
  radii=[max(z['radii']) for r in hist['/forest/fire_state'] for z in r['zones']]
  assert max(radii)-min(radii)>1,'fire did not grow'
  # Independent Gazebo pose, sustained commands, and fresh samples at contact.
  unique={row[0]:row[1:] for row in contact}
  samples=list(unique.values())
  assert len(samples)>20 and 6.05<samples[-1][0]<6.4,samples[-1:]
  assert max(math.dist(samples[-1],q) for q in samples[-15:])<.04,'robot passed through tree'
  report['collision']={'fixture':'robot placed at (5,-2) before forward command; tree was dynamically created', 'final_physical_pose':samples[-1],'fresh_contact_samples':len(samples)}
 else:
  assert not sequence and all(not r['zones'] for r in hist['/forest/fire_state']),'normal scenario contains hazards'
 report.update(status='passed',max_displacement_m=displacement,uav_goal_reached=closest[1:],
  configured_event_sequence=[{'id':h['id'],'at':h['at']} for h in sequence],
  observed_event_times=[{'id':r['hazard']['id'],'stamp':r['stamp']} for r in hist.get('/evaluation/truth',[]) if r.get('event')=='hazard_created'],
  route_before=routes[0],route_after=next((r for r in routes if r['edge_states']['trail_a']=='blocked'),routes[-1]),
  camera_frames=node.counts['uav_image'])
except Exception as exc:report['error']=str(exc)
finally:
 node.command(0.0);node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 for name,rows in poses.items():
  with (out/f'{name}.csv').open('w') as f:
   writer=csv.writer(f);writer.writerow(['sim_time','x','y','z']);writer.writerows(rows)
 (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 (out/'observations.json').write_text(json.dumps(hist)+'\n')
 print(json.dumps(report,indent=2))
raise SystemExit(0 if report['status']=='passed' else 1)
