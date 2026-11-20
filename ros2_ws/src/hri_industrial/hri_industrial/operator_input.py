"""Observed operator commands, independent of private simulated operator knowledge."""
import json,math
from pathlib import Path
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from hri_reasoning.tom_runtime import PublicNode
from hri_reasoning.core import pose_is_fresh
from hri_reasoning.lifecycle import run
from .contracts import requested_path


class OperatorInput(PublicNode):
    def __init__(self):
        super().__init__('operator_input')
        self.layout=json.loads(Path(self.parameter('facility_layout','')).read_text())
        self.position=None;self.pose_stamp=None;self.decision=0;self.destination=self.map['goal']
        self.path=[];self.branch=(None,None);self.completed=True;self.plan=None;self.warnings={};self.acked=set()
        self.create_subscription(Odometry,'/robot/odom',self.pose,qos_profile_sensor_data)
        self.subscribe_json('/hri/control',self.control)
        self.subscribe_json('/hri/robot_route',lambda data:setattr(self,'plan',data))
        self.subscribe_json('/hri/warning',lambda data:self.warnings.update({data['hazard_id']:data}))
        self.create_timer(.2,self.tick)

    def pose(self,msg):
        p=msg.pose.pose.position;origin=self.layout['robot_origin_xy'];self.position=[p.x+origin[0],p.y+origin[1]]
        self.pose_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def control(self,data):
        if not isinstance(data,dict):raise ValueError('Command must be a JSON object')
        command=data.get('command')
        if command not in ('waypoint','select_destination','approve_route','cancel_waypoint','acknowledge_warning'):return
        if command=='acknowledge_warning':
            warning=self.warnings.get(data.get('hazard_id'))
            if warning and warning['id'] not in self.acked:
                self.acked.add(warning['id'])
                self.publish_json('/hri/acknowledgement',{'warning_id':warning['id'],'hazard_id':warning['hazard_id'],
                    'edge':warning['edge'],'revision':warning['revision'],'source':'operator','received':True,
                    'meaning':'Explicit message receipt; private compliance remains unknown'})
            return
        if command=='cancel_waypoint':self.completed=True;return
        if self.position is None or not pose_is_fresh(self.stamp(),self.pose_stamp):return
        try:
            if command=='approve_route':
                if not self.plan or self.plan['revision']!=data.get('revision') or not self.plan.get('navigation_allowed'):
                    raise ValueError('A current admissible route revision is required')
                destination=self.plan['destination'];path=self.plan['recommended']
                branch=next(((a,b) for a,b in zip(path,path[1:]) if len(self.graph.adjacency[a])>=3),(None,None))
            else:
                destination=data.get('destination')
                if command=='waypoint':
                    point=[float(data['x']),float(data['y'])]
                    if not all(math.isfinite(v) for v in point):raise ValueError('Waypoint must be finite')
                    destination=self.graph.nearest(point)
                    if math.dist(point,self.graph.nodes[destination])>.75:
                        raise ValueError('Select a displayed surveyed waypoint (within 0.75 m); arbitrary off-road goals are rejected')
                path,branch=requested_path(self.map,self.position,destination,data.get('via'))
            self.destination=destination;self.path=list(path);self.branch=branch;self.completed=False;self.decision+=1
            event={'event':'operator_action','subject':'operator','source':'operator_input','command':command,
                'decision_id':self.decision,'destination':destination,'path':path,'origin':branch[0],'target':branch[1],
                'position':self.position,'source_pose_stamp':self.pose_stamp,
                'evidence':'Explicit operator command; not autonomous robot motion or a hidden belief label'}
            self.publish_json('/hri/operator_actions',event);self.publish_json('/hri/events',event)
            self.tick()
        except (ValueError,KeyError,TypeError) as exc:
            self.publish_json('/hri/events',{'event':'operator_request_rejected','reason':str(exc),'command':command})

    def tick(self):
        if self.position is None or not pose_is_fresh(self.stamp(),self.pose_stamp):return
        if self.decision and math.dist(self.position,self.graph.nodes[self.destination])<.65:self.completed=True
        edge=self.graph.edge_between(*self.branch).id if all(self.branch) else None
        data={'subject':'operator','source':'operator_input','position':self.position,'source_pose_stamp':self.pose_stamp,
            'origin':self.branch[0],'target':self.branch[1],'edge':edge,'path':self.path,'decision_id':self.decision,
            'destination':self.destination,'completed':self.completed}
        for topic in ('/hri/operator_intent','/hri/human_intent'):self.publish_json(topic,data)
        self.publish_json('/hri/facility',{'name':self.layout['name'],'destinations':self.layout['destinations'],
            'nodes':self.map['nodes'],'corridors':list(self.graph.edges),'scenario':'oil_facility',
            'control_mode':'select destination, inspect recommendation, explicitly approve current revision',
            'localization':'Wheel odometry initialized at surveyed spawn; no AMCL/SLAM or ground-truth feedback'})


def main():run(OperatorInput)
