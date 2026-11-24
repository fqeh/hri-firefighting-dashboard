"""ROS adapters for public risk routing and posterior-based warning decisions."""
import json,math,time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from .tom_runtime import PublicNode,run
from .core import pose_is_fresh
from .warning_model import Context,DecisionPolicy,validate_config
from .route_risk import recommend,escape_route_exists


def load_policy(node):
    default=Path(get_package_share_directory('hri_reasoning'))/'config/warning_policy.json'
    path=node.parameter('warning_config','') or str(default)
    return validate_config(json.loads(Path(path).read_text()))

ROUTE_UPDATE_GRACE_SECONDS=1.0

def warning_route_recommendation(route, edge, now, waiting_since):
    if not route:
        return [], None
    if edge not in route.get('edges', []):
        return route.get('recommended', []), None
    if route.get('edge_states', {}).get(edge) in ('blocked', 'hazardous'):
        return [], None
    waiting_since = now if waiting_since is None else waiting_since
    # Hazard perception and route planning run in separate callback cycles.
    if now - waiting_since < ROUTE_UPDATE_GRACE_SECONDS:
        return None, waiting_since
    return [], waiting_since


class RescueRouting(PublicNode):
    def __init__(self):
        super().__init__('rescue_routing')
        self.settings=load_policy(self)
        self.hazards={};self.clear=set();self.clear_by_source={};self.positions={};self.signatures={};self.revisions={}
        self.robot_origin=[float(self.parameter('robot_origin_x',0.)),float(self.parameter('robot_origin_y',0.))]
        self.trapped=False;self.trapped_since=None;self.dispatched_at=None
        self.subscribe_json('/hri/control',self.control)
        self.subscribe_json('/hri/observations',self.observe)
        self.subscribe_json('/hri/route_observations',self.coverage)
        for entity,topic in (('crew','/human/odom'),('robot','/robot/odom')):
            self.create_subscription(Odometry,topic,lambda msg,e=entity:self.pose(msg,e),qos_profile_sensor_data)
        self.create_timer(.5,self.tick)

    def pose(self,msg,entity):
        p=msg.pose.pose.position
        position=[p.x,p.y]
        if entity=='robot':position=[x+y for x,y in zip(position,self.robot_origin)]
        self.positions[entity]=(position,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)

    def observe(self,data):
        for h in data.get('hazards',[]):self.hazards[h['id']]={**h,'observed_at':data['stamp'],'source':data['source']}

    # Escalation to the human operator: when every escape route is closed the crew cannot
    # evacuate on foot, so warning the crew is no longer enough.
    def escalate(self,plan,position):
        nodes=self.map['nodes'];now=self.stamp()
        start=min(nodes,key=lambda n:math.dist(position[:2],nodes[n]))
        blocked=sorted(e for e,s in plan['edge_states'].items() if s=='blocked')
        trapped=not escape_route_exists(self.map,start,set(blocked))
        if trapped!=self.trapped:
            self.trapped=trapped;self.trapped_since=now if trapped else None
            if not trapped:self.dispatched_at=None
            self.publish_json('/hri/events',{'event':'no_escape_route' if trapped else 'escape_route_available',
                'source':'rescue_routing','blocked_edges':blocked,'crew_node':start})
        self.publish_json('/hri/escalation',{'active':trapped,'level':'critical' if trapped else 'none',
            'reason':'no_escape_route' if trapped else None,'blocked_edges':blocked,'crew_node':start,
            'crew_position':list(position[:2]),'since':self.trapped_since,'operator_dispatched_at':self.dispatched_at,
            'requested_action':'send_operator' if trapped and self.dispatched_at is None else None,
            'message':('Every escape route to the safety zone is blocked. The crew cannot evacuate on foot: '
                       'send an operator (rescue team or aerial extraction).') if trapped else 'At least one escape route is open.'})

    def control(self,data):
        if data.get('command')=='send_operator' and self.trapped and self.dispatched_at is None:
            self.dispatched_at=self.stamp()
            self.publish_json('/hri/events',{'event':'operator_dispatched','source':'operator',
                'detail':'Human operator dispatched a rescue; the rescue itself is not simulated.'})

    def coverage(self,data):
        # Latest cumulative list per sensor, so a segment can lose verification (e.g. under smoke).
        self.clear_by_source[data.get('source','')]=set(data.get('clear_edges',[]))
        self.clear=set().union(*self.clear_by_source.values())

    def tick(self):
        for entity,(position,stamp) in self.positions.items():
            if not pose_is_fresh(self.stamp(),stamp):continue
            start=time.perf_counter()
            plan=recommend(self.map,position,list(self.hazards.values()),self.clear,self.settings['route'],entity)
            signature=(tuple(plan['recommended']),tuple(sorted(plan['edge_states'].items())))
            changed=signature!=self.signatures.get(entity)
            if changed:self.revisions[entity]=self.revisions.get(entity,0)+1;self.signatures[entity]=signature
            plan.update(revision=self.revisions[entity],source_pose_stamp=stamp,
                replan_duration_ms=(time.perf_counter()-start)*1000,
                source='public logical-camera coverage and observed hazard geometry',
                observed_hazards=list(self.hazards.values()))
            if entity=='crew':self.escalate(plan,position)
            topic='/hri/rescue_route' if entity=='crew' else '/hri/robot_route'
            self.publish_json(topic,plan)
            if entity=='crew':self.publish_json('/hri/routes',plan)
            if changed:self.publish_json('/hri/events',{'event':'rescue_route_replanned',**plan})


class WarningDecision(PublicNode):
    """Normal baselines receive the same public inputs; no private subscription."""
    def __init__(self):
        super().__init__('warning_decision')
        self.settings=load_policy(self)
        self.baseline=self.parameter('policy','B4')
        engine=self.parameter('warning_engine','expected_loss')
        table_path=self.parameter('q_policy','');table=None
        if table_path:
            try:table=json.loads(Path(table_path).read_text())
            except (OSError,ValueError) as exc:self.get_logger().warning(f'Cannot load frozen policy; using expected loss: {exc}')
        self.policy=DecisionPolicy(self.settings,self.baseline,engine,table)
        self.observe_enabled=bool(self.parameter('observe_enabled',True))
        self.hazards={};self.beliefs=None;self.intent=None;self.route=None;self.route_waiting={}
        self.counter=0;self.last_status=None;self.alerts={};self.uav_requests=0
        self.subscribe_json('/hri/observations',self.observe)
        self.subscribe_json('/hri/belief',lambda data:setattr(self,'beliefs',data))
        self.subscribe_json('/hri/human_intent',lambda data:setattr(self,'intent',data))
        self.subscribe_json('/hri/rescue_route',lambda data:setattr(self,'route',data))
        self.subscribe_json('/hri/acknowledgement',self.acknowledge)
        self.uav=self.create_publisher(PoseStamped,'/uav/observation_goal',10)
        self.create_timer(.5,self.tick)

    def observe(self,data):
        for h in data.get('hazards',[]):
            old=self.hazards.get(h['id'])
            signature=(h.get('state'),h['edge'],round(h.get('radius',2.),1))
            revision=1 if old is None else old['revision']+int(old['signature']!=signature)
            self.hazards[h['id']]={**h,'revision':revision,'signature':signature,
                'observed_at':data['stamp'],'source':data['source']}

    def acknowledge(self,data):
        match=self.alerts.get(data.get('warning_id'))
        if match:self.policy.acknowledge(match[0],match[1],self.stamp())

    def estimate(self,hazard,belief):
        posterior=belief['posterior']
        return posterior['open_safe']+(posterior['open_hazardous'] if hazard.get('state')=='blocked' else 0.)

    def explanation(self,h,belief,result):
        names={'tree':'a fallen tree','debris':'debris','fire':'observed fire','gas':'an observed gas hazard'}
        kind=names.get(h['kind'],h['kind'])
        action=self.beliefs.get('last_movement_evidence')
        evidence=(f'Gazebo movement confirmed {action["origin"]} → {action["target"]} at {action["stamp"]:.1f}s.'
                  if action else 'No informative movement has yet been confirmed.')
        path=result.get('route_recommendation',[])
        alternative=' → '.join(path) if path else 'none currently available'
        approach=(f'The crew appears to be continuing toward {h["edge"].replace("_"," ").title()}. '
                  if result['context']['approaching'] else 'The currently declared route avoids this edge. ')
        return (f'{h["edge"].replace("_"," ").title()} is {h.get("state","hazardous").replace("_"," ")} due to {kind}; '
            f'observed by {h["source"]} at {h["observed_at"]:.1f}s. {evidence} '
            +approach+
            f'Estimated incompatible route belief: {result["context"]["probability"]:.0%}; this is an inference. '
            f'Recommended alternative: {alternative}. '
            + ('Unobserved segments require checking. ' if self.route and not self.route.get('verified') else '')+result['reason'])

    def tick(self):
        now=self.stamp()
        if not self.beliefs or not self.intent or not pose_is_fresh(now,self.intent.get('source_pose_stamp')):return
        decisions=[]
        # One controlling incident per edge: a physical blockage dominates fire
        # on the same corridor, while all measured fire patches remain in routing.
        selected={}
        for candidate in self.hazards.values():
            key=(candidate.get('state')=='blocked',self.settings['severity_by_kind'][candidate['kind']])
            old=selected.get(candidate['edge'])
            if old is None or key>(old.get('state')=='blocked',self.settings['severity_by_kind'][old['kind']]):
                selected[candidate['edge']]=candidate
        for h in selected.values():
            b=self.beliefs.get('beliefs',{}).get(h['edge'])
            if not b:continue
            p=self.estimate(h,b)
            if p is None:continue
            position=self.intent['position'];distance=math.dist(position,h['position'][:2])
            approaching=b.get('relevant',False) and not self.intent.get('completed',False)
            t=self.settings['thresholds']
            urgency=2 if distance<=t['emergency_distance_m'] else int(distance<=t['near_distance_m'])
            context=Context(float(p),self.settings['severity_by_kind'][h['kind']],urgency,
                min(1.,b['uncertainty']/math.log(4)),not self.intent.get('completed',False),approaching,
                b.get('informative_actions',0),distance)
            route_recommendation,waiting_since=warning_route_recommendation(
                self.route,h['edge'],now,self.route_waiting.get(h['id']))
            if waiting_since is None:self.route_waiting.pop(h['id'],None)
            else:self.route_waiting[h['id']]=waiting_since
            if route_recommendation is None:
                decisions.append({'action':'SILENT','classification':'route_update_pending',
                    'context':{'probability':float(p),'approaching':approaching},
                    'route_recommendation':[],
                    'explanation':'Waiting for the route planner to incorporate the observed hazard before suggesting a path.'})
                continue
            result=self.policy.choose(h['id'],h['revision'],now,context,self.observe_enabled)
            result.update(hazard_id=h['id'],edge=h['edge'],revision=h['revision'],policy=self.baseline,
                posterior=b['posterior'],evidence_stamp=b.get('last_relevant_observation'),
                route_revision=self.route.get('revision') if self.route else None,
                route_recommendation=route_recommendation)
            result['explanation']=self.explanation(h,b,result)
            if result['action'] in ('SOFT_CUE','EXPLICIT_WARNING'):
                self.counter+=1;identifier=f'{self.baseline}-{self.counter}'
                self.alerts[identifier]=(h['id'],h['revision'])
                self.publish_json('/hri/warning',{'id':identifier,'source':'warning_decision',
                    'hazard_id':h['id'],'edge':h['edge'],'state':h.get('state','blocked'),
                    'revision':h['revision'],'action':'soft' if result['action']=='SOFT_CUE' else 'explicit',
                    'warning_action':result['action'],'severity':'critical' if urgency==2 else 'advisory',
                    'policy':self.baseline,'p_mismatch':p if approaching else 0.,
                    'recommended':result['route_recommendation'],'explanation':result['explanation']})
                result['warning_id']=identifier
            elif result['action']=='OBSERVE':
                self.uav_requests+=1
                goal=PoseStamped();goal.header.frame_id='world';goal.header.stamp=self.get_clock().now().to_msg()
                goal.pose.position.x=float(max(-5,min(20,h['position'][0])))
                goal.pose.position.y=float(max(-6,min(12,h['position'][1])))
                goal.pose.position.z=8.;goal.pose.orientation.w=1.;self.uav.publish(goal)
                self.publish_json('/hri/events',{'event':'observation_requested','hazard_id':h['id'],
                    'position':[goal.pose.position.x,goal.pose.position.y,8.],
                    'note':'Reposition only; posterior changes require actual new evidence, not the MDP sample'})
            decisions.append(result)
        if decisions:
            focus=max(decisions,key=lambda d:(d['action']!='SILENT',d['context']['approaching'],d['context']['probability']))
            self.publish_json('/hri/decision',{**focus,'decisions':decisions,'warnings_sent':self.counter,
                'observation_requests':self.uav_requests})
        else:
            self.publish_json('/hri/decision',{'action':'SILENT','classification':'no_relevant_hazard',
                'policy':self.baseline,'engine':self.policy.engine,'decisions':[],
                'warnings_sent':self.counter,'observation_requests':self.uav_requests,
                'route_recommendation':self.route.get('recommended',[]) if self.route else [],
                'explanation':'No relevant hazard has been observed. Continue monitoring the public sensors and crew movement.'})
        self.publish_json('/hri/status',{'source':'warning_policy','state':'running','policy':self.baseline,
            'engine':self.policy.engine,'fallback':self.policy.fallback,'warnings':self.counter,
            'observations':self.uav_requests})


def route_main():run(RescueRouting)
def warning_main():run(WarningDecision)
