#!/usr/bin/env python3
"""Real Gazebo/Nav2 acceptance probe. Run inside the built ROS container."""
import argparse,json,math,os,re,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,LaserScan
from lifecycle_msgs.srv import GetState
from phase1_acceptance import terminate

def run_case(case,root,engine,policy,q_policy=''):
    output=root/case;output.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_DOMAIN_ID='61',GZ_PARTITION=f'oil_acceptance_{os.getpid()}_{case}')
    os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','GZ_PARTITION')})
    command=['ros2','launch','hri_bringup','demo.launch.py','scenario:=oil_facility',f'oil_case:={case}',
        'dashboard:=false',f'policy:={policy}',f'warning_engine:={engine}',
        f'simulated_operator:={str(case in ("known","unaware")).lower()}',f'log_path:={output.resolve()}/events.jsonl']
    if q_policy:command.append('q_policy:='+q_policy)
    report={'status':'failed','case':case,'command':command,'checks':{}}
    log=(output/'simulator.log').open('w');process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('industrial_acceptance');latest={};history={};trajectory=[];scans=[]
    pub=node.create_publisher(String,'/hri/control',10)
    active_client=node.create_client(GetState,'/bt_navigator/get_state');active_request=None
    def active():
        nonlocal active_request
        if not active_client.service_is_ready():return False
        if active_request is None:active_request=active_client.call_async(GetState.Request())
        if not active_request.done():return False
        result=active_request.result().current_state.id==3;active_request=None;return result
    def receive(msg,key):
        data=json.loads(msg.data);latest[key]=data;history.setdefault(key,[]).append(data)
    for key in ('robot_route','navigation','belief','decision','operator_actions','warning','observations','tom_evidence'):
        node.create_subscription(String,'/hri/'+key,lambda msg,k=key:receive(msg,k),20)
    def pose(msg):
        p=msg.pose.pose.position;latest['pose']=[p.x,p.y,p.z];latest['time']=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        trajectory.append([latest['time'],p.x,p.y,p.z])
    def scan(msg):
        valid=[v for v in msg.ranges if math.isfinite(v) and msg.range_min<=v<=msg.range_max]
        forward=[v for i,v in enumerate(msg.ranges) if abs(msg.angle_min+i*msg.angle_increment)<.025 and math.isfinite(v)]
        latest['scan']={'count':len(valid),'minimum':min(valid,default=None),'forward':min(forward,default=None)}
        scans.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,**latest['scan']})
    node.create_subscription(Odometry,'/evaluation/robot_pose',pose,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/robot/scan',scan,qos_profile_sensor_data)
    node.create_subscription(Image,'/robot/camera/image',lambda m:latest.update(image=[m.width,m.height,len(m.data)]),qos_profile_sensor_data)
    def until(predicate,timeout=75):
        end=time.monotonic()+timeout
        while time.monotonic()<end and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise AssertionError(f'Timeout: t={latest.get("time")}, pose={latest.get("pose")}, nav={history.get("navigation",[])[-5:]}, route={latest.get("robot_route")}')
    def send(**data):pub.publish(String(data=json.dumps(data)))
    def wait_sim(seconds):
        end=latest['time']+seconds;until(lambda:latest['time']>=end,seconds*8+15)
    try:
        until(lambda:'robot_route' in latest and 'pose' in latest and 'image' in latest and latest.get('scan',{}).get('count',0)>10 and active())
        report['checks']['gazebo_nav2_camera_lidar_ready']=True
        initial=latest['pose'][:];report['initial_pose']=initial
        if case in ('known','unaware'):
            until(lambda:latest.get('belief',{}).get('action_evidence_count',0)>0)
        else:
            destination='junction' if case=='normal' else 'assembly'
            send(command='select_destination',destination=destination,**({} if case=='normal' else {'via':'corridor_b'}))
            until(lambda:latest['robot_route'].get('request_id',0)>0)
            wait_sim(1.)
            assert math.dist(initial[:2],latest['pose'][:2])<.08
            report['checks']['selection_does_not_authorize_motion']=True
            if case in ('normal','blockage'):
                send(command='approve_route',revision=latest['robot_route']['revision'])
                until(lambda:any(e['state']=='navigating' for e in history.get('navigation',[])))
        if case=='normal':
            until(lambda:sum(e['state']=='succeeded' for e in history.get('navigation',[]))>=2 and math.dist(latest['pose'][:2],[3,0])<.4,150)
            report['checks']['nav2_goal_reached_with_physics']=True
        else:
            expected='hazardous' if case=='fire' else 'blocked'
            until(lambda:latest['robot_route']['edge_states']['corridor_b']==expected)
            report['checks']['physical_incident_observed']=True
            report['post_incident_route']=latest['robot_route']
            until(lambda:bool(history.get('decision')) and latest['time']>=10)
            report['first_hazard_decision']=history['decision'][next(i for i,d in enumerate(history['decision']) if d.get('hazard_id'))] if any(d.get('hazard_id') for d in history['decision']) else latest['decision']
            if case=='blockage':
                until(lambda:any(e['state']=='reapproval_required' for e in history.get('navigation',[])))
                until(lambda:any(e['state']=='canceled' for e in history.get('navigation',[])))
                position=latest['pose'][:];wait_sim(1)
                assert math.dist(position[:2],latest['pose'][:2])<.12
                report['checks']['hazard_stops_approved_route']=True
                send(command='select_destination',destination='assembly',via='corridor_b')
                until(lambda:latest.get('belief',{}).get('action_evidence_count',0)>=2)
                if policy=='B4':until(lambda:bool(history.get('warning')))
                assert 'corridor_c' in latest['robot_route']['edges']
                send(command='approve_route',revision=latest['robot_route']['revision'])
                until(lambda:latest['pose'][1]>2,150)
                report['checks']['approved_alternative_moves_robot']=True
            elif case in ('gas','combined','no_safe'):
                until(lambda:latest['robot_route']['edge_states']['corridor_c']=='hazardous')
                if case=='no_safe':
                    until(lambda:latest['robot_route']['edge_states']['corridor_d']=='blocked')
                    assert not latest['robot_route']['navigation_allowed']
                    send(command='approve_route',revision=latest['robot_route']['revision'])
                    until(lambda:any(e['state']=='rejected' for e in history.get('navigation',[])))
                    position=latest['pose'][:];wait_sim(2)
                    assert math.dist(position[:2],latest['pose'][:2])<.08
                    report['checks']['no_safe_route_refused_without_motion']=True
                else:
                    assert 'corridor_d' in latest['robot_route']['edges']
                    report['checks']['gas_excluded_from_alternative']=True
            else:
                assert 'corridor_b' not in latest['robot_route']['edges']
                report['checks']['observed_hazard_excluded']=True
            if case in ('gas','no_safe','known','unaware'):
                # Stationary robot: the forward beam must hit the newly spawned
                # colliding debris, independently of the semantic camera.
                returns=[s['forward'] for s in scans if s['stamp']>9. and s['forward'] is not None]
                assert returns and any(4.7<distance<5.5 for distance in returns),returns[-5:]
                report['checks']['lidar_detects_physical_debris']=True
            assert latest['belief']['action_evidence_count']>0
            assert latest['belief']['last_movement_evidence'] is None
            report['checks']['operator_evidence_and_warning_policy_publish']=True
        report.update(status='passed',final_pose=latest['pose'],final_route=latest['robot_route'],belief=latest.get('belief'))
    except Exception as exc:report['error']=str(exc)
    finally:
        send(command='cancel_waypoint')
        for _ in range(30):rclpy.spin_once(node,timeout_sec=.05)
        subprocess.run(['gz','service','-s','/world/oil_facility/control','--reqtype','gz.msgs.WorldControl',
            '--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true'],env=env,stdout=subprocess.DEVNULL,timeout=6)
        time.sleep(.5);node.destroy_node();rclpy.shutdown();terminate(process);log.close()
    critical=[line for line in (output/'simulator.log').read_text().splitlines() if re.search(r'\[ERROR\]|\[Err\]|Traceback|process has died|Segmentation fault|terminate called',line)]
    if critical or process.returncode!=0:report['status']='failed'
    report.update(critical_log_lines=critical,launch_exit=process.returncode,history=history,trajectory=trajectory,scans=scans)
    (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('case','status','checks','critical_log_lines','error') if k in report}),flush=True)
    return report['status']=='passed'

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',default='normal,blockage,fire,gas,no_safe,known,unaware');p.add_argument('--engine',default='expected_loss');p.add_argument('--policy',default='B4')
    p.add_argument('--q-policy',default='')
    a=p.parse_args();results=[run_case(case,a.output,a.engine,a.policy,a.q_policy) for case in a.cases.split(',')]
    raise SystemExit(0 if all(results) else 1)
