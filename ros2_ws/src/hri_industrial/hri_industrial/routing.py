"""Industrial goal/traffic adapter around the existing risk route implementation."""
import json,time
from pathlib import Path
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from hri_reasoning.tom_runtime import PublicNode
from hri_reasoning.rescue_runtime import load_policy
from hri_reasoning.core import pose_is_fresh
from hri_reasoning.lifecycle import run
from .contracts import safe_recommendation,navigation_mask


class IndustrialRouting(PublicNode):
    def __init__(self):
        super().__init__('rescue_routing')
        self.layout=json.loads(Path(self.parameter('facility_layout','')).read_text())
        self.costs=load_policy(self)['route'];self.hazards={};self.position=None;self.pose_stamp=None
        self.destination=self.map['goal'];self.request_id=0;self.revision=0;self.signature=None
        self.clear=set(self.layout['initial_survey_edges']);self.mask_dirty=True
        self.subscribe_json('/hri/observations',self.observe)
        self.subscribe_json('/hri/operator_intent',self.intent)
        self.create_subscription(Odometry,'/robot/odom',self.pose,qos_profile_sensor_data)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.mask_pub=self.create_publisher(OccupancyGrid,'/oil/navigation_mask',qos)
        self.info_pub=self.create_publisher(CostmapFilterInfo,'/oil/filter_info',qos)
        # Publish comfortably inside the shared 0.5-second pose freshness bound.
        self.create_timer(.2,self.tick)

    def pose(self,msg):
        p=msg.pose.pose.position;origin=self.layout['robot_origin_xy'];self.position=[p.x+origin[0],p.y+origin[1]]
        self.pose_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def intent(self,data):
        if data['destination'] in self.graph.nodes:self.destination=data['destination'];self.request_id=data['decision_id']

    def observe(self,data):
        for hazard in data.get('hazards',[]):
            old=self.hazards.get(hazard['id'])
            if old is None or old['edge']!=hazard['edge'] or old['radius']!=hazard['radius']:self.mask_dirty=True
            self.hazards[hazard['id']]={**hazard,'observed_at':data['stamp'],'source':data['source']}

    def tick(self):
        if self.position is None or not pose_is_fresh(self.stamp(),self.pose_stamp):return
        started=time.perf_counter()
        plan=safe_recommendation(self.map,self.layout,self.position,self.destination,list(self.hazards.values()),self.clear,self.costs)
        signature=(tuple(plan['recommended']),tuple(sorted(plan['edge_states'].items())),self.destination,self.request_id,plan['navigation_allowed'])
        changed=signature!=self.signature
        if changed:self.signature=signature;self.revision+=1
        plan.update(revision=self.revision,request_id=self.request_id,source_pose_stamp=self.pose_stamp,
            replan_duration_ms=(time.perf_counter()-started)*1000,observed_hazards=list(self.hazards.values()),
            source='Public commissioned road survey plus measured robot semantic observations; live truth excluded')
        for topic in ('/hri/robot_route','/hri/routes','/hri/rescue_route'):self.publish_json(topic,plan)
        if changed:self.publish_json('/hri/events',{'event':'industrial_route_replanned',**plan})
        if self.mask_dirty:
            width,height,values=navigation_mask(self.layout,list(self.hazards.values()))
            msg=OccupancyGrid();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
            msg.info.resolution=self.layout['mask_resolution'];msg.info.width=width;msg.info.height=height
            msg.info.origin.position.x=float(self.layout['bounds'][0]);msg.info.origin.position.y=float(self.layout['bounds'][1]);msg.info.origin.orientation.w=1.
            msg.data=values;self.mask_pub.publish(msg)
            info=CostmapFilterInfo();info.header=msg.header;info.type=0;info.filter_mask_topic='/oil/navigation_mask';info.base=0.;info.multiplier=1.
            self.info_pub.publish(info);self.mask_dirty=False
            self.publish_json('/hri/events',{'event':'navigation_mask_updated','hazard_ids':list(self.hazards),
                'meaning':'Traffic keepout and measured hazards; physical obstacle marking also uses actual LiDAR'})


def main():run(IndustrialRouting)
