"""Privileged trial action generation; never imported by inference or routing."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from hri_reasoning.runtime import ResearchNode
from hri_reasoning.tom import action_distribution,route,WAIT
from hri_reasoning.response_model import HumanResponse
from hri_reasoning.core import behavior_rng
from hri_reasoning.lifecycle import run


class SimulatedOperator(ResearchNode):
    def __init__(self):
        super().__init__('simulated_operator')
        self.settings=self.config['operator'];self.position=None;self.started=0.;self.requests=0
        self.knowledge={e:'open_safe' for e in self.graph.edges}
        self.knowledge.update(self.config.get('operator_initial_knowledge',{}))
        self.rng=behavior_rng(self.config['seed'],'operator_decisions')
        self.declare_parameter('response_config','')
        response=Path(self.get_parameter('response_config').value or
            str(Path(get_package_share_directory('hri_reasoning'))/'config/human_response.json'))
        self.response=HumanResponse(json.loads(response.read_text()),self.config['seed'],self.settings['trust'])
        self.create_subscription(Odometry,'/robot/odom',self.pose,qos_profile_sensor_data)
        self.subscribe_json('/hri/warning',self.warning)
        self.subscribe_json('/hri/robot_route',self.plan)
        self.subscribe_json('/hri/operator_actions',self.selected)
        self.approval_pending=None;self.acks=[];self.create_timer(.2,self.tick)

    def pose(self,msg):
        p=msg.pose.pose.position;origin=self.config['robot_start'];self.position=[p.x+origin[0],p.y+origin[1]]

    def warning(self,data):
        result=self.response.receive(data['id'],data['action'])
        if result is None:return
        edge=data['edge']
        if result['complied']:self.knowledge[edge]=data['state']
        if result['acknowledged']:
            self.acks.append((self.stamp()+self.response.config['acknowledgement_delay_s'],data['hazard_id']))
        self.publish_json('/evaluation/operator_response',{'warning_id':data['id'],**result})
        if result['route_changed']:self.request()

    def plan(self,data):
        if not self.settings.get('auto_approve') or not self.approval_pending or not data.get('navigation_allowed'):return
        # One automatic approval per destination selection, only in explicitly
        # enabled experimental runs. Interactive operation always asks the user.
        key=data.get('request_id')
        if key==self.approval_pending:
            self.approval_pending=None
            self.publish_json('/hri/control',{'command':'approve_route','revision':data['revision']})

    def selected(self,data):
        if data.get('command')=='select_destination':self.approval_pending=data['decision_id']

    def request(self):
        if self.position is None:return
        destination=self.settings['destination'];start=self.graph.nearest(self.position)
        path,_=route(self.graph,start,destination,self.knowledge)
        branch=next((a for a in path[:-1] if len(self.graph.adjacency[a])>=3),start)
        distribution=action_distribution(self.graph,branch,destination,self.knowledge,
            self.settings['beta'],self.settings['lapse'])
        draw=self.rng.random();selected=None
        for action,probability in distribution.items():
            draw-=probability
            if draw<=0:selected=action;break
        if selected in (None,WAIT):return
        edge=self.graph.edge_between(branch,selected)
        self.publish_json('/hri/control',{'command':'select_destination','destination':destination,'via':edge.id})

    def tick(self):
        now=self.stamp()
        if now<=0 or self.position is None:return
        for at,hazard in self.acks[:]:
            if now>=at:
                self.publish_json('/hri/control',{'command':'acknowledge_warning','hazard_id':hazard})
                self.acks.remove((at,hazard))
        if self.requests<self.settings['repeat_count'] and now-self.started>=self.settings['request_at']+self.requests*self.settings['repeat_every_s']:
            self.requests+=1;self.request()
        self.publish_json('/evaluation/human_belief',{'subject':'operator','knowledge':self.knowledge,
            'trust':self.response.trust,'requests':self.requests,'purpose':'private trial generation/evaluation/oracle only'})


def main():run(SimulatedOperator)
