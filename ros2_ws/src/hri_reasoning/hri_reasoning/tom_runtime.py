"""Public-evidence ROS adapters for Phase 3.

Neither node loads a scenario or subscribes to evaluation / human belief topics.
Perception classifies sensor identities (ideal semantic detection) and associates
their measured positions to the public trail geometry. Inference verifies a
declared branch against actual Gazebo odometry before using its likelihood.
"""
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import LogicalCameraImage
from .core import WarningPolicy, pose_is_fresh
from .tom import (RouteToM, DEFAULT_PRIOR, DEFAULT_TRANSITION, validate_public_map, MotionStall,
                  semantic_hazard_kind)


class PublicNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('public_map', '')
        self.map = json.loads(Path(self.get_parameter('public_map').value).read_text())
        self.graph = validate_public_map(self.map)
        self.outputs = {}

    def parameter(self, name, default):
        return self.declare_parameter(name, default).value

    def stamp(self):
        return self.get_clock().now().nanoseconds/1e9

    def publish_json(self, topic, data):
        if topic not in self.outputs:
            self.outputs[topic] = self.create_publisher(String, topic, 20)
        self.outputs[topic].publish(String(data=json.dumps({'stamp': self.stamp(), **data}, allow_nan=False)))

    def subscribe_json(self, topic, callback):
        def parse(message):
            try:
                callback(json.loads(message.data))
            except (KeyError, ValueError, TypeError) as exc:
                self.get_logger().warning(f'Rejected {topic}: {exc}')
        return self.create_subscription(String, topic, parse, 20)


def segment_distance(point, a, b):
    dx,dy = b[0]-a[0],b[1]-a[1]
    ratio = max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(point[0]-a[0]-ratio*dx,point[1]-a[1]-ratio*dy)


class PublicPerception(PublicNode):
    def __init__(self):
        super().__init__('public_perception')
        sources = self.parameter('observation_sources', 'robot,uav').split(',')
        if any(source not in ('robot','uav') for source in sources):
            raise ValueError('Observation sources must be robot and/or uav')
        self.known = {}
        self.coverage = {edge:set() for edge in self.graph.edges}
        for source in sources:
            self.create_subscription(LogicalCameraImage, f'/{source}/detections',
                lambda message,s=source: self.observe(message,s), qos_profile_sensor_data)

    def observe(self, message, source):
        detected = []
        for model in message.model:
            name = model.name.split('::')[0]
            if name.startswith('route_') and name[6:] in self.graph.edges:
                edge=self.graph.edges[name[6:]]
                cp,q=message.pose.position,message.pose.orientation
                for i in range(21):
                    a,b=self.graph.nodes[edge.source],self.graph.nodes[edge.target]
                    x=a[0]+(b[0]-a[0])*i/20-cp.x;y=a[1]+(b[1]-a[1])*i/20-cp.y;z=.05-cp.z
                    # World point into the measured sensor frame (inverse quaternion).
                    ux,uy,uz=-q.x,-q.y,-q.z
                    tx,ty,tz=2*(uy*z-uz*y),2*(uz*x-ux*z),2*(ux*y-uy*x)
                    lx=x+q.w*tx+uy*tz-uz*ty;ly=y+q.w*ty+uz*tx-ux*tz;lz=z+q.w*tz+ux*ty-uy*tx
                    half=lx*math.tan((1.2 if source=='uav' else 1.7)/2)-.25
                    if .05<lx<(14 if source=='robot' else 18) and abs(ly)<half and abs(lz)<half:
                        self.coverage[edge.id].add(i)
            # Static semantic classes, not an incident list or scheduled positions.
            kind = semantic_hazard_kind(name)
            if kind is None:
                continue
            cp,cq,p = message.pose.position,message.pose.orientation,model.pose.position
            ux,uy,uz = cq.x,cq.y,cq.z
            tx,ty,tz = 2*(uy*p.z-uz*p.y),2*(uz*p.x-ux*p.z),2*(ux*p.y-uy*p.x)
            xyz = [cp.x+p.x+cq.w*tx+uy*tz-uz*ty,
                   cp.y+p.y+cq.w*ty+uz*tx-ux*tz,cp.z+p.z+cq.w*tz+ux*ty-uy*tx]
            edge = min(self.graph.edges.values(), key=lambda e: segment_distance(
                xyz,self.graph.nodes[e.source],self.graph.nodes[e.target]))
            if segment_distance(xyz,self.graph.nodes[edge.source],self.graph.nodes[edge.target]) > 3:
                continue
            state = 'open_hazardous' if kind in ('fire','gas') else 'blocked'
            hazard = {'id':name,'kind':kind,'edge':edge.id,'position':xyz,
                      'radius':.6 if '_front_' in name else 2.,'state':state}
            detected.append(hazard)
            if name not in self.known:
                self.publish_json('/hri/events', {'source':source,'event':'hazard_detected',
                    'hazard_id':name,'edge':edge.id,'sensor':'Gazebo logical camera, geometric map association'})
            self.known[name] = hazard
        if detected:
            self.publish_json('/hri/observations', {'source':source,'hazards':detected,
                'known_edges':{h['edge']:h['state'] for h in self.known.values()}})
        self.publish_json('/hri/route_observations',{'source':source,
            'clear_edges':[edge for edge,points in self.coverage.items() if len(points)==21],
            'assumption':'ideal logical-camera coverage; no visual occlusion model'})


class BayesianToM(PublicNode):
    def __init__(self):
        super().__init__('bayesian_tom')
        self.model = RouteToM(self.map, beta=float(self.parameter('assumed_beta',1.2)),
            lapse=float(self.parameter('assumed_lapse',.03)),
            prior=json.loads(self.parameter('tom_prior',json.dumps(DEFAULT_PRIOR))),
            transition=json.loads(self.parameter('tom_transition',json.dumps(DEFAULT_TRANSITION))))
        self.radius = float(self.parameter('assumed_observation_radius',3.))
        self.fov = float(self.parameter('assumed_observation_fov',1.57))
        self.visual_acquisition = float(self.parameter('visual_acquisition',.9))
        self.warning_compliance = float(self.parameter('assumed_compliance',.8))
        if self.radius <= 0 or not 0 < self.fov <= 2*math.pi or any(
                not math.isfinite(p) or not 0 <= p <= 1 for p in (self.visual_acquisition,self.warning_compliance)):
            raise ValueError('Invalid assumed observation/compliance parameters')
        self.policy_name = self.parameter('policy','B4')
        self.external_policy = bool(self.parameter('external_policy',False))
        if self.policy_name == 'oracle' and not self.external_policy:
            raise ValueError('Use tom:=false policy:=oracle for the explicitly privileged baseline')
        self.policy = WarningPolicy('B1' if self.external_policy else self.policy_name)
        self.position = self.pose_stamp = self.intent = None
        self.yaw = 0.
        self.pending = None
        self.last_decision = None
        self.seen_opportunities = set()
        self.warnings_seen = set()
        self.next_warning = 0
        self.stall = MotionStall()
        self.encounters_seen = set()
        self.create_subscription(Odometry,'/human/odom',self.odom,qos_profile_sensor_data)
        self.subscribe_json('/hri/human_intent',self.on_intent)
        self.subscribe_json('/hri/observations',self.observe)
        self.subscribe_json('/hri/warning',self.warning)
        self.create_timer(.2,self.tick)

    def odom(self, message):
        p,q = message.pose.pose.position,message.pose.pose.orientation
        self.position = (p.x,p.y)
        self.pose_stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def on_intent(self, data):
        self.intent = data
        if data['decision_id'] != self.last_decision and self.position is not None:
            self.last_decision = data['decision_id']
            origin,target = data.get('origin'),data.get('target')
            self.pending = None
            if origin in self.graph.nodes and target in [n for n,_ in self.graph.adjacency[origin]]:
                self.pending = (origin,target,self.position,self.stamp())

    def observe(self, data):
        self.model.advance(self.stamp())
        for h in data.get('hazards',[]):
            state = h.get('state', 'open_hazardous' if h['kind'] in ('fire','gas') else 'blocked')
            if self.model.observed.get(h['edge'],{}).get('state')=='blocked' and state!='blocked':continue
            self.model.observe(h['edge'],state,data['stamp'],h['position'],h['id'])

    def warning(self, data):
        key = (data.get('source','policy'), data.get('id'))
        edge = data.get('edge')
        if key in self.warnings_seen or edge not in self.model.filters or data.get('action') not in ('soft','explicit'):
            return
        self.model.advance(self.stamp())
        self.warnings_seen.add(key)
        probability = self.warning_compliance*(.55 if data['action']=='soft' else 1.)
        self.model.information(edge,data.get('state','blocked'),probability,self.stamp())
        self.publish_json('/hri/tom_evidence',{'event':'warning_received','edge':edge,
            'acquisition_probability':probability,'warning_id':data.get('id')})

    def tick(self):
        now = self.stamp()
        self.model.advance(now)
        if not self.intent or not pose_is_fresh(now,self.pose_stamp) or not pose_is_fresh(now,self.intent.get('source_pose_stamp')):
            self.stall.update(now,self.position,False)
            return
        target = self.intent.get('target')
        attempting = False
        if target in self.graph.nodes and self.intent.get('edge') in self.graph.edges:
            target_xy = self.graph.nodes[target]
            heading = math.atan2(target_xy[1]-self.position[1],target_xy[0]-self.position[0])
            angle = math.atan2(math.sin(heading-self.yaw),math.cos(heading-self.yaw))
            attempting = math.dist(target_xy,self.position) > .65 and abs(angle) < .7
        stalled = self.stall.update(now,self.position,attempting)
        if self.pending:
            origin,target,start,at = self.pending
            a,b = self.graph.nodes[origin],self.graph.nodes[target]
            dx,dy = b[0]-a[0],b[1]-a[1]
            along = ((self.position[0]-start[0])*dx+(self.position[1]-start[1])*dy)/math.hypot(dx,dy)
            if along >= .15:
                self.model.action(origin,target,now)
                self.publish_json('/hri/tom_evidence',{'event':'movement_action','origin':origin,
                    'target':target,'decision_id':self.last_decision,'distance_along_edge':along,
                    'source_pose_stamp':self.pose_stamp,'decision_observed_at':at})
                self.pending = None
        for edge,observed in self.model.observed.items():
            dx,dy = observed['position'][0]-self.position[0],observed['position'][1]-self.position[1]
            angle = math.atan2(math.sin(math.atan2(dy,dx)-self.yaw),math.cos(math.atan2(dy,dx)-self.yaw))
            # A geometric opportunity, not access to the human sensor or hidden update.
            if edge not in self.seen_opportunities and math.hypot(dx,dy) <= self.radius and abs(angle) <= self.fov/2:
                self.model.information(edge,observed['state'],self.visual_acquisition,now)
                self.seen_opportunities.add(edge)
                self.publish_json('/hri/tom_evidence',{'event':'visual_opportunity','edge':edge,
                    'acquisition_probability':self.visual_acquisition,'source_pose_stamp':self.pose_stamp})
            if stalled and edge == self.intent.get('edge') and edge not in self.encounters_seen and math.hypot(dx,dy) <= 2.:
                self.model.information(edge,observed['state'],.95,now)
                self.encounters_seen.add(edge)
                self.publish_json('/hri/tom_evidence',{'event':'unexpected_blockage_opportunity','edge':edge,
                    'acquisition_probability':.95,'source_pose_stamp':self.pose_stamp})
        intended = []
        for a,b in zip(self.intent.get('path',[]),self.intent.get('path',[])[1:]):
            try: intended.append(self.graph.edge_between(a,b).id)
            except (StopIteration, KeyError): pass
        if self.intent.get('completed'):
            intended = []
        beliefs = self.model.summary(intended)
        focus = max(beliefs, key=lambda e: (beliefs[e]['hazard_observed'],beliefs[e]['relevant'],beliefs[e]['p_outdated'] or 0))
        self.publish_json('/hri/belief', {**beliefs[focus], 'model':'factorized_route_hmm_v1',
            'beliefs':beliefs,'inferred_route':self.intent.get('path',[]),
            'inferred_destination':self.map['goal'],'intent_source':'broadcast branch, checked against Gazebo odometry',
            'last_movement_evidence':self.model.last_action,'action_evidence_count':self.model.action_count,
            'hazard_coverage':'partial; undetected hazards cannot be assessed'})
        if self.external_policy:
            self.publish_json('/hri/status',{'source':'inference','state':'running',
                'model':'factorized_route_hmm_v1','movement_actions':self.model.action_count})
            return
        for edge,observed in self.model.observed.items():
            belief = beliefs[edge]
            action = self.policy.choose(observed['id'],now,belief['p_outdated'],belief['relevant'],
                                       math.dist(self.position,observed['position'][:2]))
            if action != 'silent':
                self.next_warning += 1
                self.publish_json('/hri/warning', {'id':self.next_warning,'source':'bayesian_tom',
                    'hazard_id':observed['id'],'edge':edge,'state':observed['state'],'action':action,
                    'severity':'critical' if action=='explicit' else 'advisory','policy':self.policy_name,
                    'p_mismatch':belief['p_mismatch'],'explanation':
                    f'{edge}: observed {observed["state"]}; estimated incompatible route belief {belief["p_outdated"]:.0%}, based on observable actions and information opportunities.'})
        known = {e: ('hazardous' if o['state']=='open_hazardous' else 'blocked') for e,o in self.model.observed.items()}
        recommendation = self.graph.shortest(self.graph.nearest(self.position),self.map['goal'],known)
        self.publish_json('/hri/routes', {'nodes':self.graph.nodes,'recommended':recommendation['path'],
            'intended':self.intent.get('path',[]),'unknown_edges':recommendation['unknown_edges'],
            'blocked_edges':list(known),'safety':recommendation['safety']})
        self.publish_json('/hri/status',{'source':'policy','state':'running','model':'factorized_route_hmm_v1',
            'observed_hazards':len(known),'warnings':self.next_warning,'movement_actions':self.model.action_count})


from .lifecycle import run


def perception_main(): run(PublicPerception)
def inference_main(): run(BayesianToM)
