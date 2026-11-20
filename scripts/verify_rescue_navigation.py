#!/usr/bin/env python3
"""Launch the industrial world and verify approval/replacement through real Nav2."""
import argparse,json,math,os,re,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
from phase1_acceptance import terminate

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
os.environ.update(ROS_DOMAIN_ID='52',GZ_PARTITION='phase4_navigation')
command=['ros2','launch','hri_bringup','demo.launch.py','scenario:=industrial','nav2:=true',
    'dashboard:=false','policy:=B1','warning_engine:=expected_loss',f'log_path:={a.output.resolve()}/events.jsonl']
report={'status':'failed','command':command,'environment':{k:os.environ[k] for k in ('ROS_DOMAIN_ID','GZ_PARTITION')}}
log=(a.output/'simulator.log').open('w')
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('rescue_navigation_probe');latest={};events=[];trajectory=[]
pub=node.create_publisher(String,'/hri/control',10)
action=ActionClient(node,NavigateToPose,'/navigate_to_pose')
active_client=node.create_client(GetState,'/bt_navigator/get_state')
active_request=None
def navigation_active():
    global active_request
    if not active_client.service_is_ready():return False
    if active_request is None:active_request=active_client.call_async(GetState.Request())
    if not active_request.done():return False
    active=active_request.result().current_state.id==3;active_request=None
    return active
def receive_route(msg):latest['route']=json.loads(msg.data)
def receive_nav(msg):events.append(json.loads(msg.data))
def receive_pose(msg):
    v=msg.pose.pose.position
    latest['pose']=[v.x,v.y,v.z]
    trajectory.append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,v.x,v.y,v.z])
node.create_subscription(String,'/hri/robot_route',receive_route,10)
node.create_subscription(String,'/hri/navigation',receive_nav,10)
node.create_subscription(Odometry,'/evaluation/robot_pose',receive_pose,qos_profile_sensor_data)
def until(predicate,timeout=100):
    end=time.monotonic()+timeout
    while time.monotonic()<end and process.poll() is None:
        rclpy.spin_once(node,timeout_sec=.1)
        if predicate():return
    raise AssertionError(f'Navigation timeout: events={events[-5:]}, state={latest}')
def send(data):pub.publish(String(data=json.dumps(data)))
try:
    until(lambda:action.server_is_ready() and 'route' in latest and 'pose' in latest and navigation_active())
    initial=latest['pose'];route=latest['route'];report['initial_route']=route
    send({'command':'approve_route','revision':-1})
    until(lambda:any(e['state']=='rejected' for e in events),15)
    report['stale_revision_rejected']=True
    send({'command':'approve_route','revision':latest['route']['revision']})
    until(lambda:any(e['state']=='route_approved' for e in events),15)
    if 'trail_a' in route['edges']:
        until(lambda:any(e['state']=='reapproval_required' for e in events),40)
        until(lambda:any(e['state']=='canceled' for e in events),30)
        report['physical_incident_required_reapproval']=True
        assert 'trail_a' not in latest['route']['edges']
        index=len(events);position=latest['pose']
        send({'command':'approve_route','revision':latest['route']['revision']})
        until(lambda:any(e['state']=='navigating' for e in events[index:]),30)
        until(lambda:math.dist(position[:2],latest['pose'][:2])>.15,40)
    until(lambda:math.dist(initial[:2],latest['pose'][:2])>.35)
    report['approved_route_physical_displacement_m']=math.dist(initial[:2],latest['pose'][:2])
    index=len(events)
    send({'command':'waypoint','x':2.,'y':0.})
    until(lambda:any(e['state']=='canceled' for e in events[index:]),30)
    until(lambda:any(e['state']=='requested' and e.get('frame')=='odom' for e in events[index:]),20)
    until(lambda:any(e['state']=='succeeded' for e in events[index:]),120)
    assert math.dist(latest['pose'][:2],[2,0])<.45,latest['pose']
    report.update(status='passed',replacement_goal=[2,0],final_world_pose=latest['pose'],events=events)
except Exception as exc:report.update(error=str(exc),events=events)
finally:
    send({'command':'cancel_waypoint'})
    for _ in range(5):rclpy.spin_once(node,timeout_sec=.1)
    subprocess.run(['gz','service','-s','/world/industrial/control','--reqtype','gz.msgs.WorldControl',
        '--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true'],stdout=subprocess.DEVNULL,timeout=6)
    time.sleep(.5);node.destroy_node();rclpy.shutdown();terminate(process);log.close()
critical=[l for l in (a.output/'simulator.log').read_text().splitlines()
          if re.search(r'\[ERROR\]|\[Err\]|Traceback|process has died|Segmentation fault',l)]
report.update(critical_log_lines=critical,launch_exit=process.returncode,trajectory=trajectory)
if critical or process.returncode!=0:report['status']='failed'
(a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('trajectory','initial_route')},indent=2))
raise SystemExit(0 if report['status']=='passed' else 1)
