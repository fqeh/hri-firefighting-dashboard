"""ROS adapters. Gazebo owns all physical state and observation sensors.

Each class is run in a separate ROS process. Only the scenario manager and
evaluation logger publish/read incident truth. The oracle is an explicitly
selected experimental reference; normal inference never subscribes to beliefs.
"""
from __future__ import annotations
import json
import hashlib
import math
from pathlib import Path
import random
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import LogicalCameraImage
from ament_index_python.packages import get_package_share_directory

from .core import (RouteGraph, BeliefFilter, WarningPolicy, sample_action,
                   validate_behavior_parameters, pose_is_fresh, behavior_rng, PassiveMapAttention)
from .tom import canonical, action_distribution, route, WAIT, MotionStall
from .response_model import HumanResponse


class ResearchNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('scenario', 'forest')
        self.declare_parameter('config', '')
        self.declare_parameter('seed', 42)
        self.declare_parameter('policy', 'B4')
        self.scenario = self.get_parameter('scenario').value
        self.seed = self.get_parameter('seed').value
        self.policy_name = self.get_parameter('policy').value
        share = Path(get_package_share_directory('hri_sim'))
        path = self.get_parameter('config').value or str(share/'config'/f'{self.scenario}.json')
        self.config = json.loads(Path(path).read_text())
        self.graph = RouteGraph(self.config)
        self.model_root = share/'models'
        self.hazards = {h['id']:h for h in self.config.get('hazards', [])}
        self.publishers_by_topic = {}
        self.rng = random.Random(self.seed)

    def stamp(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_json(self, topic, payload):
        if topic not in self.publishers_by_topic:
            self.publishers_by_topic[topic] = self.create_publisher(String, topic, 20)
        self.publishers_by_topic[topic].publish(String(data=json.dumps({'stamp':self.stamp(), **payload}, allow_nan=False)))

    def subscribe_json(self, topic, callback):
        def parse(msg):
            try:
                callback(json.loads(msg.data))
            except (ValueError, KeyError, TypeError) as exc:
                self.get_logger().warning(f'Rejected invalid {topic} message: {exc}')
        return self.create_subscription(String, topic, parse, 20)

    def status(self, source, state, **extra):
        self.publish_json('/hri/status', {'source':source,'state':state,'scenario':self.scenario,
                          'seed':self.seed,'policy':self.policy_name, **extra})


class ScenarioManager(ResearchNode):
    def __init__(self):
        super().__init__('scenario_manager')
        self.spawned = set()
        self.failed = set()
        self.start_time = None
        self.subscribe_json('/hri/control', self.control)
        self.create_timer(0.5, self.tick)

    def service(self, suffix, request_type, request):
        world = self.config.get('world', self.scenario)
        command = ['gz','service','-s',f'/world/{world}/{suffix}','--reqtype',request_type,
                   '--reptype','gz.msgs.Boolean','--timeout','5000','--req',request]
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError((result.stderr or result.stdout or f'exit {result.returncode}').strip())

    def inject(self, hazard):
        if hazard['id'] in self.spawned:
            return
        x,y,z = hazard['position']
        model = self.model_root/hazard.get('model',hazard['id'])/'model.sdf'
        if not model.is_file():
            raise RuntimeError(f'Missing physical hazard SDF: {model}')
        request = (f'sdf_filename: {json.dumps(str(model))} name: {json.dumps(hazard["id"])} '
                   + 'pose: {position: {x: %s y: %s z: %s}} allow_renaming: false' % (x,y,z))
        self.service('create','gz.msgs.EntityFactory',request)
        self.spawned.add(hazard['id'])
        self.publish_json('/evaluation/truth',{'event':'hazard_created','hazard':hazard})
        self.publish_json('/hri/events',{'source':'scenario','event':'incident_injected',
                          'id':hazard['id'],'detail':'Physical Gazebo entity created; detection remains sensor-dependent.'})

    def control(self, data):
        command = data['command']
        if command in ('waypoint', 'cancel_waypoint', 'acknowledge_warning','approve_route','uav_waypoint','dismiss_warning'):
            return  # Navigation adapter owns operator waypoint commands.
        try:
            if command in ('pause','resume'):
                self.service('control','gz.msgs.WorldControl',f'pause: {str(command == "pause").lower()}')
            elif command == 'inject':
                hazard = self.hazards.get(data['hazard_id']) if 'hazard_id' in data else next(iter(self.hazards.values()), None)
                if hazard is None:raise ValueError('No configured incident with this ID')
                if hazard is not None:
                    self.inject(hazard)
            elif command == 'reset':
                raise ValueError('Reset requires restarting the launch process with the same seed; see scripts/run_demo.sh')
            else:
                raise ValueError(f'Unknown command {command}')
            self.publish_json('/hri/events',{'source':'scenario','event':'control_applied','command':command})
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
            self.publish_json('/hri/events',{'source':'scenario','event':'control_failed','command':command,'error':str(exc)})

    def tick(self):
        now = self.stamp()
        if now <= 0:
            return
        if self.start_time is None:
            self.start_time = now
        for hazard in self.hazards.values():
            if hazard.get('manual_only'):
                continue
            if hazard['id'] not in self.spawned | self.failed and now-self.start_time >= hazard.get('at',3):
                try:
                    self.inject(hazard)
                except (RuntimeError, subprocess.TimeoutExpired) as exc:
                    self.failed.add(hazard['id'])
                    self.get_logger().error(f'Hazard injection failed: {exc}')
                    self.publish_json('/hri/events',{'source':'scenario','event':'injection_failed','id':hazard['id'],'error':str(exc)})
        self.status('scenario','running',elapsed=now-self.start_time,active_incidents=len(self.spawned))


class HumanBehavior(ResearchNode):
    def __init__(self):
        super().__init__('human_behavior')
        self.declare_parameter('human_beta', 1.2)
        self.declare_parameter('observation_radius', 3.0)
        self.declare_parameter('trust', 0.8)
        self.declare_parameter('passive_map_rate', 0.05)
        self.tom = self.declare_parameter('tom', False).value
        self.lapse = float(self.declare_parameter('human_lapse', 0.03).value)
        if not math.isfinite(self.lapse) or not 0 <= self.lapse <= 1:
            raise ValueError('Human lapse must be in [0,1]')
        self.wait_until = 0.
        self.stall = MotionStall()
        self.beta = float(self.get_parameter('human_beta').value)
        self.radius = float(self.get_parameter('observation_radius').value)
        self.trust = float(self.get_parameter('trust').value)
        response_path=self.declare_parameter('response_config','').value
        if not response_path:response_path=str(Path(get_package_share_directory('hri_reasoning'))/'config/human_response.json')
        self.response=HumanResponse(json.loads(Path(response_path).read_text()),self.seed,self.trust)
        self.pending_acknowledgements=[]
        self.passive_map_rate = float(self.get_parameter('passive_map_rate').value)
        validate_behavior_parameters(beta=self.beta,trust=self.trust,observation_radius=self.radius,
                                     passive_map_rate=self.passive_map_rate)
        self.route_rng = behavior_rng(self.seed,'route')
        self.compliance_rng = behavior_rng(self.seed,'compliance')
        self.attention = PassiveMapAttention(self.passive_map_rate,self.seed)
        self.passive_map = {}
        self.knowledge = {eid:'clear' for eid in self.graph.edges}  # explicitly stale prior map
        self.knowledge.update({k:v for k,v in self.config.get('crew_initial_knowledge',{}).items() if k in self.knowledge and v in ('clear','blocked','hazardous','unknown')})
        if self.tom:
            self.knowledge = {edge:canonical(state) for edge,state in self.knowledge.items()}
        self.position = None
        self.odom_stamp = None
        self.yaw = 0.0
        self.current = self.config.get('start','start')
        self.goal = self.config.get('goal','safety')
        self.target = None
        self.path = []
        self.decision_id = 0
        self.completed = False
        self.started = None
        self.command = self.create_publisher(Twist,'/human/cmd_vel',10)
        self.create_subscription(Odometry,'/human/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(LogicalCameraImage,'/human/detections',self.observe,qos_profile_sensor_data)
        self.subscribe_json('/hri/warning',self.warning)
        self.subscribe_json('/hri/observations',self.on_map_update)
        self.create_timer(0.1,self.tick)

    def on_map_update(self,data):
        # The shared display contains robot observations, never incident truth.
        # Merely receiving display data does not update private human knowledge.
        for hazard in data.get('hazards',[]):
            edge = hazard.get('edge')
            if edge in self.graph.edges:
                if self.passive_map.get(edge)!='blocked':self.passive_map[edge] = self.hazard_state(hazard)

    def hazard_state(self, hazard):
        if not self.tom:
            return 'blocked'
        return 'open_hazardous' if hazard.get('kind') in ('fire','gas') else 'blocked'

    def glance(self):
        events = self.attention.poll(self.stamp())
        if not events or self.completed:
            return
        changed = any(self.knowledge[edge] != state for edge,state in self.passive_map.items())
        self.knowledge.update(self.passive_map)
        if changed:
            self.replan()
        for scheduled_at in events:
            self.publish_json('/hri/events',{'source':'human','event':'passive_map_glance',
                              'scheduled_at':scheduled_at,'available_observed_edges':len(self.passive_map)})

    def odom(self, msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        self.position = (p.x,p.y)
        self.odom_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def observe(self, msg):
        for model in msg.model:
            name = model.name.split('::')[0]
            hazard = self.hazards.get(name)
            p = model.pose.position
            if hazard and math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z) <= self.radius:
                state = self.hazard_state(hazard)
                if self.knowledge[hazard['edge']]=='blocked' and state!='blocked':continue
                if self.knowledge[hazard['edge']] != state:
                    self.knowledge[hazard['edge']] = state
                    self.response.verified_information();self.trust=self.response.trust
                    self.replan()
                    self.publish_json('/hri/events',{'source':'human','event':'local_observation','hazard_id':name})

    def warning(self, data):
        if data.get('action') not in ('soft','explicit'):
            return
        edge = data['edge']
        if edge not in self.knowledge:
            return
        response=self.response.receive((data.get('source','policy'),data['id']),data['action'])
        if response is None:return
        before=dict(self.knowledge)
        if response['complied']:
            if self.knowledge[edge]!='blocked':
                self.knowledge[edge] = canonical(data.get('state','blocked')) if self.tom else 'blocked'
        if response['route_changed']:
            self.replan()
        self.trust=self.response.trust
        if response['acknowledged']:
            self.pending_acknowledgements.append((self.stamp()+self.response.config['acknowledgement_delay_s'],
                {'warning_id':data['id'],'hazard_id':data.get('hazard_id'),'edge':edge,
                 'revision':data.get('revision',1),'source':'simulated_crew','received':True,
                 'meaning':'Message receipt, not proof of compliance or private belief'}))
        self.publish_json('/evaluation/human_belief',{'knowledge':self.knowledge,'trust':self.trust,
                          'warning_id':data['id'],'knowledge_before':before,**response})

    def replan(self):
        if self.position is None or self.completed:
            return
        self.current = self.graph.nearest(self.position)
        # Return to the nearest physical waypoint before choosing a new branch.
        self.target = self.current
        self.path = [self.current]
        self.decision_id += 1

    def decide(self):
        if self.stamp() < self.wait_until:
            return
        dist = (action_distribution(self.graph,self.current,self.goal,self.knowledge,self.beta,self.lapse)
                if self.tom else self.graph.action_distribution(self.current,self.goal,self.knowledge,self.beta))
        self.target = sample_action(dist,self.route_rng)
        self.decision_id += 1
        if self.target == WAIT:
            self.target = None
            self.path = [self.current]
            self.wait_until = self.stamp()+2.
            return
        if self.target:
            if self.tom:
                remainder,_ = route(self.graph,self.target,self.goal,self.knowledge,excluded=(self.current,))
                self.path = [self.current]+(remainder or [self.target])
            else:
                remainder = self.graph.shortest(self.target,self.goal,self.knowledge,unknown_penalty=1)
                self.path = [self.current]+remainder['path']
        else:
            self.path = []

    def tick(self):
        for at,data in list(self.pending_acknowledgements):
            if self.stamp()>=at:
                self.publish_json('/hri/acknowledgement',data)
                self.pending_acknowledgements.remove((at,data))
        if self.position is None:
            return
        if not pose_is_fresh(self.stamp(),self.odom_stamp):
            self.stall.update(self.stamp(),self.position,False)
            self.command.publish(Twist())
            self.status('human','waiting_for_fresh_odometry',source_pose_stamp=self.odom_stamp)
            return
        if self.started is None:
            self.started = self.stamp()
        self.glance()
        if not self.completed and math.dist(self.position,self.graph.nodes[self.goal]) < 0.65:
            self.completed = True
            self.publish_json('/hri/events',{'source':'human','event':'mission_completed','elapsed':self.stamp()-self.started})
        if not self.completed:
            if self.target is None:
                self.decide()
            elif math.dist(self.position,self.graph.nodes[self.target]) < 0.45:
                self.current = self.target
                self.decide()
        twist = Twist()
        if self.target and not self.completed:
            tx,ty = self.graph.nodes[self.target]
            error = math.atan2(math.sin(math.atan2(ty-self.position[1],tx-self.position[0])-self.yaw),
                               math.cos(math.atan2(ty-self.position[1],tx-self.position[0])-self.yaw))
            twist.angular.z = max(-1.2,min(1.2,2.0*error))
            twist.linear.x = 0.55*max(0.0,math.cos(error)) if abs(error)<0.7 else 0.0
        self.command.publish(twist)
        edge = None
        if self.target and self.current != self.target:
            try:
                edge = self.graph.edge_between(self.current,self.target).id
            except StopIteration:
                pass
        if self.tom and self.stall.update(self.stamp(),self.position,
                edge is not None and twist.linear.x > .2 and not self.completed):
            self.knowledge[edge] = 'blocked'
            self.command.publish(Twist())
            self.publish_json('/hri/events',{'source':'human','event':'unexpected_motion_blockage',
                'edge':edge,'position':list(self.position),'source_pose_stamp':self.odom_stamp})
            self.stall.update(self.stamp(),self.position,False)
            self.replan()
            edge = None
        self.publish_json('/hri/human_intent',{'position':list(self.position),'origin':self.current,
                          'target':self.target,'edge':edge,'path':self.path,'decision_id':self.decision_id,
                          'source_pose_stamp':self.odom_stamp,'completed':self.completed})
        self.publish_json('/evaluation/human_belief',{'knowledge':self.knowledge,'trust':self.trust,
                          'position':list(self.position),'completed':self.completed,'beta':self.beta})


class BeliefWarning(ResearchNode):
    def __init__(self):
        super().__init__('belief_warning')
        self.declare_parameter('assumed_beta',1.2)
        self.assumed_beta = float(self.get_parameter('assumed_beta').value)
        validate_behavior_parameters(assumed_beta=self.assumed_beta)
        self.observed = {}
        self.filters = {}
        self.last_decision = {}
        self.intent = None
        self.oracle = None
        self.policy = WarningPolicy(self.policy_name)
        self.next_warning = 0
        self.goal = self.config.get('goal','safety')
        self.subscribe_json('/hri/human_intent',self.on_intent)
        for source in ('robot','uav'):
            self.create_subscription(LogicalCameraImage,f'/{source}/detections',
                lambda msg, s=source:self.observe(msg,s),qos_profile_sensor_data)
        if self.policy_name == 'oracle':
            self.subscribe_json('/evaluation/human_belief',self.on_oracle)
        self.create_timer(1.0,self.tick)

    def on_oracle(self,data):
        self.oracle = data.get('knowledge')

    def on_intent(self,data):
        self.intent = data

    def observe(self,msg,source):
        detected = []
        for model in msg.model:
            name = model.name.split('::')[0]
            if name not in self.hazards:
                continue
            catalog_entry = self.hazards[name]
            # Semantic edge annotation is part of the prior map. Position is
            # reconstructed from the actual Gazebo camera measurement; event
            # schedule and undiscovered incident truth are never published.
            cp,cq,p = msg.pose.position,msg.pose.orientation,model.pose.position
            ux,uy,uz = cq.x,cq.y,cq.z
            tx,ty,tz = 2*(uy*p.z-uz*p.y),2*(uz*p.x-ux*p.z),2*(ux*p.y-uy*p.x)
            position = [cp.x+p.x+cq.w*tx+uy*tz-uz*ty,
                        cp.y+p.y+cq.w*ty+uz*tx-ux*tz,
                        cp.z+p.z+cq.w*tz+ux*ty-uy*tx]
            hazard = {'id':name,'kind':catalog_entry.get('kind','obstacle'),
                      'edge':catalog_entry['edge'],'radius':catalog_entry.get('radius',2),
                      'position':position}
            detected.append(hazard)
            if name not in self.observed:
                self.observed[name] = {'hazard':hazard,'detected_at':self.stamp(),'source':source}
                self.filters[name] = BeliefFilter(assumed_beta=self.assumed_beta)
                self.publish_json('/hri/events',{'source':source,'event':'hazard_detected','hazard_id':name,
                                  'edge':hazard['edge'],'sensor':'Gazebo logical camera'})
        if detected:
            self.publish_json('/hri/observations',{'source':source,'hazards':detected,
                              'known_edges':{o['hazard']['edge']:'blocked' for o in self.observed.values()}})

    def tick(self):
        if not self.intent:
            return
        intent = self.intent
        path_edges = set()
        for a,b in zip(intent.get('path',[]),intent.get('path',[])[1:]):
            try:
                path_edges.add(self.graph.edge_between(a,b).id)
            except StopIteration:
                pass
        for hid, observation in self.observed.items():
            hazard = observation['hazard']
            posterior = self.filters[hid]
            posterior.predict()
            if intent['decision_id'] != self.last_decision.get(hid):
                origin, choice = intent.get('origin'),intent.get('target')
                if origin in self.graph.nodes and choice in {n for n,_ in self.graph.adjacency[origin]}:
                    posterior.update_action(self.graph,origin,choice,self.goal,hazard['edge'])
                self.last_decision[hid] = intent['decision_id']
            relevant = hazard['edge'] in path_edges and not intent.get('completed',False)
            distance = math.dist(intent['position'],hazard['position'][:2])
            self.publish_json('/hri/belief',{'hazard_id':hid,'edge':hazard['edge'],
                              'posterior':{'unaware':posterior.unaware,'aware':1-posterior.unaware},
                              'uncertainty':posterior.entropy,'p_mismatch':posterior.unaware if relevant else 0,
                              'relevant':relevant})
            oracle_unaware = None if self.oracle is None else self.oracle.get(hazard['edge']) == 'clear'
            if self.policy_name == 'oracle' and self.oracle is None:
                continue
            action = self.policy.choose(hid,self.stamp(),posterior.unaware,relevant,distance,oracle_unaware)
            if action != 'silent':
                self.next_warning += 1
                self.publish_json('/hri/warning',{'id':self.next_warning,'hazard_id':hid,'edge':hazard['edge'],
                                  'action':action,'severity':'critical' if action=='explicit' else 'advisory',
                                  'policy':self.policy_name,'p_mismatch':posterior.unaware,
                                  'explanation':f'{hazard["edge"]} changed: {hazard.get("kind","obstacle")} observed by {observation["source"]}; '
                                  f'intended route uses it; estimated outdated belief {posterior.unaware:.0%}.'})
                posterior.predict(steps=0,warning_compliance=0.44 if action=='soft' else 0.8)
        known = {o['hazard']['edge']:'blocked' for o in self.observed.values()}
        start = self.graph.nearest(intent['position'])
        route = self.graph.shortest(start,self.goal,known)
        self.publish_json('/hri/routes',{'nodes':self.graph.nodes,'recommended':route['path'],
                          'intended':intent.get('path',[]),'unknown_edges':route['unknown_edges'],
                          'blocked_edges':list(known),'safety':route['safety'],
                          'explanation':'Avoids observed obstructions. Unobserved segments require confirmation.'})
        self.status('policy','running',observed_hazards=len(self.observed),warnings=self.next_warning)


class EventLogger(ResearchNode):
    def __init__(self):
        super().__init__('event_logger')
        self.declare_parameter('log_path','/tmp/hri-events.jsonl')
        self.path = Path(self.get_parameter('log_path').value)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.stream = self.path.open('w',buffering=1)
        condition = {}
        for name,default in (('human_beta',1.2),('assumed_beta',1.2),('trust',.8),
                             ('observation_radius',3.0),('passive_map_rate',.05),('human_lapse',.03),
                             ('assumed_lapse',.03),('tom',False),('observation_sources','robot,uav'),
                             ('assumed_observation_radius',3.),('assumed_compliance',.8)):
            self.declare_parameter(name,default)
            condition[name] = self.get_parameter(name).value
        self.write('run_metadata',{'scenario':self.scenario,'seed':self.seed,'policy':self.policy_name,
                                  'environment':'Gazebo Sim','schema_version':2,'config':self.config,
                                  'config_sha256':hashlib.sha256(json.dumps(self.config,sort_keys=True).encode()).hexdigest(),
                                  **condition})
        for topic in ('/evaluation/truth','/evaluation/human_belief','/hri/observations','/hri/human_intent',
                      '/hri/belief','/hri/warning','/hri/routes','/hri/status','/hri/events',
                      '/forest/fire_state','/forest/routes','/hri/uav_status','/hri/tom_evidence',
                      '/hri/decision','/hri/acknowledgement','/hri/rescue_route','/hri/robot_route',
                      '/hri/navigation','/hri/route_observations','/hri/operator_actions','/hri/operator_intent',
                      '/hri/facility','/hri/operator_receipt','/evaluation/industrial_hazards','/evaluation/operator_response'):
            self.subscribe_json(topic,lambda data,t=topic:self.write(t,data))
        self.create_subscription(Odometry,'/human/odom',self.on_pose,qos_profile_sensor_data)
        for topic in ('/robot/odom','/evaluation/robot_pose'):
            self.create_subscription(Odometry,topic,lambda msg,t=topic:self.on_pose(msg,t),qos_profile_sensor_data)
        for source in ('human','robot','uav'):
            self.create_subscription(LogicalCameraImage,f'/{source}/detections',
                lambda msg,s=source:self.on_camera(msg,s),qos_profile_sensor_data)

    def on_pose(self,msg,topic='/human/odom'):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        self.write(topic,{'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                   'frame_id':msg.header.frame_id,'position':[p.x,p.y,p.z],
                   'orientation':[q.x,q.y,q.z,q.w],
                   'linear_velocity':[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]})

    def on_camera(self,msg,source):
        self.write('/evaluation/sensors',{'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                   'source':source,'sensor':'logical_camera','models':[m.name for m in msg.model]})

    def write(self,topic,data):
        self.stream.write(json.dumps({'topic':topic,'stamp':self.stamp(),'wall_time':time.time(),'data':data},allow_nan=False)+'\n')

    def destroy_node(self):
        self.stream.close()
        return super().destroy_node()


from .lifecycle import run


def scenario_main(): run(ScenarioManager)
def human_main(): run(HumanBehavior)
def inference_main(): run(BeliefWarning)
def logger_main(): run(EventLogger)
