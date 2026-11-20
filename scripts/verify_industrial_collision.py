#!/usr/bin/env python3
"""Independent physical contact fixture. Nav2 is off; collisions remain enabled."""
import argparse,json,math,os,re,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from phase1_acceptance import terminate

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
os.environ.update(ROS_DOMAIN_ID='62',GZ_PARTITION='oil_collision_fixture')
command=['ros2','launch','hri_bringup','demo.launch.py','scenario:=oil_facility','oil_case:=blockage',
    'nav2:=false','dashboard:=false',f'log_path:={a.output.resolve()}/events.jsonl']
log=(a.output/'simulator.log').open('w');process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('industrial_contact_probe');pub=node.create_publisher(Twist,'/robot/cmd_vel',10);poses=[]
def receive(msg):
    p=msg.pose.pose.position;poses.append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,p.x,p.y,p.z])
node.create_subscription(Odometry,'/evaluation/robot_pose',receive,qos_profile_sensor_data)
report={'status':'failed','command':command,'fixture':'Sustained forward command toward dynamically spawned colliding debris; no pose teleport or disabled collisions'}
try:
    end=time.monotonic()+150
    while time.monotonic()<end and process.poll() is None:
        rclpy.spin_once(node,timeout_sec=.02)
        if not poses:continue
        msg=Twist();msg.linear.x=.35 if poses[-1][0]<30 else 0.;pub.publish(msg)
        if poses[-1][0]>=30:break
    assert poses and poses[-1][0]>=30,'Simulation did not reach contact observation interval'
    assert 4.8<poses[-1][1]<5.1,poses[-1]
    contact=[r[1:] for r in poses if r[0]>27]
    assert len(contact)>30 and max(math.dist(contact[-1],p) for p in contact)<.04
    report.update(status='passed',final_pose=poses[-1],fresh_contact_samples=len(contact))
except Exception as exc:report['error']=str(exc)
finally:
    pub.publish(Twist())
    for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
    subprocess.run(['gz','service','-s','/world/oil_facility/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true'],stdout=subprocess.DEVNULL,timeout=6)
    node.destroy_node();rclpy.shutdown();terminate(process);log.close()
critical=[l for l in (a.output/'simulator.log').read_text().splitlines() if re.search(r'\[ERROR\]|\[Err\]|Traceback|process has died|Segmentation fault',l)]
if critical or process.returncode!=0:report['status']='failed'
report.update(trajectory=poses,critical_log_lines=critical,launch_exit=process.returncode)
(a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='trajectory'},indent=2))
raise SystemExit(0 if report['status']=='passed' else 1)
