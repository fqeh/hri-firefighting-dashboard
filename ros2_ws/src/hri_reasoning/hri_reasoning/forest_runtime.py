"""Gazebo adapters for Phase 2. Scripted UAV pose commands are NOT flight dynamics."""
import json,math,time
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import LogicalCameraImage
from .runtime import ScenarioManager,ResearchNode
from .forest_model import FireModel,plan_route,incident_sequence


def pose_request(name,point):
    return 'name: '+json.dumps(name)+' position: {x: %s y: %s z: %s} orientation: {w: 1}'%tuple(point)


def visual_sdf(name,point,radius,color):
    return f'<sdf version="1.9"><model name="{name}"><static>true</static><pose>{point[0]} {point[1]} {point[2]} 0 0 0</pose><link name="link"><visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>.025</length></cylinder></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material><cast_shadows>false</cast_shadows></visual></link></model></sdf>'


class ForestManager(ScenarioManager):
    def __init__(self):
        super().__init__()
        self.start_time=0.0
        self.coverage={e:set() for e in self.graph.edges}
        self.hazards={h['id']:h for h in incident_sequence(self.config)}
        self.fire=FireModel(self.config['fire'],self.seed)
        self.observed={}
        self.position=self.config['human_start'][:2]
        self.fire_seen=False
        self.rendered=set()
        self.last_plan=None
        self.route_visual=False
        self.external_routing=bool(self.declare_parameter('external_routing',False).value)
        if self.external_routing:self.subscribe_json('/hri/rescue_route',self.external_route)
        self.create_subscription(LogicalCameraImage,'/uav/detections',self.observe_routes,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/human/odom',self.crew_pose,qos_profile_sensor_data)
        self.subscribe_json('/hri/observations',self.observations)
        self.create_timer(2.0,self.update_forest)

    def inject(self,hazard):
        already=hazard['id'] in self.spawned
        super().inject(hazard)
        if not already and self.config.get('manual_incidents') and hazard['kind']=='fire':
            for origin in self.fire.config.get('origins',[]):
                if origin['id']==hazard['id']:origin['at']=self.stamp()

    def crew_pose(self,msg): self.position=[msg.pose.pose.position.x,msg.pose.pose.position.y]

    def external_route(self,plan):
        self.publish_json('/forest/routes',plan)
        signature=(tuple(plan['recommended']),tuple(sorted(plan['edge_states'].items())))
        if signature!=self.last_plan:
            self.show_route(plan);self.last_plan=signature

    def observations(self,data):
        for hazard in data.get('hazards',[]):
            if hazard['kind']=='tree':self.observed[hazard['edge']]='blocked'
            if hazard['kind']=='fire':self.fire_seen=True

    def observe_routes(self,msg):
        for model in msg.model:
            name=model.name.split('::')[0]
            if name.startswith('route_'):
                edge=name[len('route_'):]
                if edge in self.graph.edges:
                    e=self.graph.edges[edge]
                    a,b=self.graph.nodes[e.source],self.graph.nodes[e.target]
                    camera=msg.pose.position
                    half=max(0,camera.z)*math.tan(self.config['uav']['camera_hfov']/2)
                    for i in range(21):
                        point=[a[j]+(b[j]-a[j])*i/20 for j in (0,1)]
                        if abs(point[0]-camera.x)<half-.4 and abs(point[1]-camera.y)<half-.4:
                            self.coverage[edge].add(i)
                    if len(self.coverage[edge])==21:self.observed.setdefault(edge,'clear')
            if name in self.hazards and self.hazards[name]['kind']=='tree':
                self.observed[self.hazards[name]['edge']]='blocked'
            if name.startswith('hazard_fire'):self.fire_seen=True

    def update_forest(self):
        if self.stamp()<=0:return
        zones=self.fire.state(self.stamp())
        self.publish_json('/forest/fire_state',{'zones':zones,'source':'research hazard model truth',
            'wind':self.config['fire']['wind'],'model':'seeded fuel/wind radial fronts; unvalidated wildfire prediction'})
        try:
            for zone in zones:
                # Quantized footprint overlays show the mathematical hazard boundary.
                # Flame/smoke assets remain separate non-colliding visual cues.
                level=int(max(zone['radii']))
                key=(zone['id'],level)
                if key not in self.rendered:
                    self.rendered.add(key)
                    for index in range(0,len(zone['polygon']),3):
                        p=zone['polygon'][index]
                        name=f'{zone["id"]}_front_{level}_{index}'
                        request=f'sdf_filename: {json.dumps(str(self.model_root/"hazard_fire/model.sdf"))} name: {json.dumps(name)} pose: {{position: {{x: {p[0]} y: {p[1]} z: 0}}}}'
                        self.service('create','gz.msgs.EntityFactory',request)
            if self.external_routing:return
            plan=plan_route(self.config,self.position,self.observed,zones if self.fire_seen else [])
            plan.update(nodes=self.graph.nodes,crew_position=self.position,source='UAV logical observations + observed-fire model estimate',
                        fire_observed=self.fire_seen)
            self.publish_json('/forest/routes',plan)
            signature=(tuple(plan['recommended']),tuple(sorted(plan['edge_states'].items())))
            if signature!=self.last_plan:
                self.publish_json('/hri/events',{'source':'forest','event':'route_recalculated',**plan})
                self.show_route(plan)
                self.last_plan=signature
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.publish_json('/hri/events',{'source':'forest','event':'forest_update_failed','error':str(exc)})

    def show_route(self,plan):
        parts=[]
        chosen=set(plan['edges'])
        for edge in self.graph.edges.values():
            blocked=plan['edge_states'][edge.id]=='blocked'
            if edge.id not in chosen and not blocked:continue
            a,b=self.graph.nodes[edge.source],self.graph.nodes[edge.target]
            yaw=math.atan2(b[1]-a[1],b[0]-a[0])
            color='1 .08 .02 1' if blocked else '.05 .75 1 1'
            parts.append(f'<visual name="{edge.id}"><pose>{(a[0]+b[0])/2} {(a[1]+b[1])/2} .06 0 0 {yaw}</pose><geometry><box><size>{edge.length} .14 .04</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse><emissive>{color}</emissive></material></visual>')
        if self.route_visual:self.service('remove','gz.msgs.Entity','name: "forest_route_overlay" type: MODEL')
        sdf='<sdf version="1.9"><model name="forest_route_overlay"><static>true</static><link name="link">'+''.join(parts)+'</link></model></sdf>'
        self.service('create','gz.msgs.EntityFactory','sdf: '+json.dumps(sdf))
        self.route_visual=True


class ScriptedUAV(ResearchNode):
    def __init__(self):
        super().__init__('scripted_uav')
        self.service=ScenarioManager.service.__get__(self)
        self.position=None
        self.pose_stamp=None
        self.goals=list(self.config['uav']['waypoints'])
        self.index=0
        self.last_time=None
        self.path=RosPath();self.path.header.frame_id='world'
        self.path_pub=self.create_publisher(RosPath,'/uav/trajectory',10)
        self.create_subscription(Odometry,'/uav/odom',self.pose,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/uav/observation_goal',self.goal,10)
        self.created_footprint=False
        self.footprint_radius=None
        self.footprint_position=None
        self.create_timer(.5,self.tick)

    def pose(self,msg):
        p=msg.pose.pose.position
        self.position=[p.x,p.y,p.z]
        self.pose_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not self.path.poses or self.pose_stamp-(self.path.poses[-1].header.stamp.sec+self.path.poses[-1].header.stamp.nanosec/1e9)>.5:
            sample=PoseStamped();sample.header=msg.header;sample.pose=msg.pose.pose
            self.path.poses.append(sample);self.path.poses=self.path.poses[-600:]
            self.path.header.stamp=msg.header.stamp;self.path_pub.publish(self.path)

    def goal(self,msg):
        p=msg.pose.position
        point=[p.x,p.y,p.z]
        if msg.header.frame_id!='world' or not all(math.isfinite(v) for v in point) or not(-5<=p.x<=20 and -6<=p.y<=12 and 6<=p.z<=12):
            self.publish_json('/hri/events',{'source':'uav','event':'waypoint_rejected','reason':'requires world frame, x[-5,20], y[-6,12], z[6,12]'})
            return
        self.goals=[point];self.index=0
        self.publish_json('/hri/events',{'source':'uav','event':'waypoint_accepted','position':point})

    def tick(self):
        now=self.stamp()
        if now<=0 or self.position is None:return
        fresh=self.pose_stamp is not None and -.1<=now-self.pose_stamp<2
        if not fresh:
            self.publish_json('/hri/uav_status',{'state':'telemetry_stale','constraint':'scripted kinematics; no validated flight dynamics'})
            return
        dt=.5 if self.last_time is None else max(0,min(1,now-self.last_time))
        self.last_time=now
        target=self.goals[self.index]
        distance=math.dist(self.position,target)
        state='observing' if distance<.15 else 'flying_scripted'
        try:
            if distance>=.15:
                fraction=min(1,self.config['uav']['speed']*dt/distance)
                point=[a+(b-a)*fraction for a,b in zip(self.position,target)]
                self.service('set_pose','gz.msgs.Pose',pose_request('uav',point))
            elif len(self.goals)>1:
                self.index=(self.index+1)%len(self.goals) if self.config['uav'].get('loop',True) else min(self.index+1,len(self.goals)-1)
            width=2*(self.position[2]-.15)*math.tan(self.config['uav']['camera_hfov']/2)
            if self.created_footprint and abs(width/2-self.footprint_radius)>.25:
                self.service('remove','gz.msgs.Entity','name: \"uav_footprint\" type: MODEL')
                self.created_footprint=False
            if not self.created_footprint:
                sdf=visual_sdf('uav_footprint',[*self.position[:2],.04],width/2,'.1 .6 1 .12')
                self.service('create','gz.msgs.EntityFactory','sdf: '+json.dumps(sdf))
                self.created_footprint=True
                self.footprint_radius=width/2
                self.footprint_position=list(self.position[:2])
            elif math.dist(self.position[:2],self.footprint_position)>.05:
                self.service('set_pose','gz.msgs.Pose',pose_request('uav_footprint',[*self.position[:2],.04]))
                self.footprint_position=list(self.position[:2])
            self.publish_json('/hri/uav_status',{'state':state,'mission':'scripted_observation_patrol','position':self.position,
                'pose_stamp':self.pose_stamp,'pose_fresh':fresh,'goal':target,'waypoint_index':self.index,
                'footprint_width_m':width,'footprint_note':'inscribed circle in downward square frustum; visual radius updated in 0.25m increments',
                'constraint':'Gazebo pose-service kinematics; no validated flight dynamics, avoidance or PX4 control'})
        except Exception as exc:
            self.publish_json('/hri/events',{'source':'uav','event':'controller_failed','error':str(exc)})
            self.get_logger().error(str(exc))


from .lifecycle import run


def forest_main():run(ForestManager)
def uav_main():run(ScriptedUAV)
