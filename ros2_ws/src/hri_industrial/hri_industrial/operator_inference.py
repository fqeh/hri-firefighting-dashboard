"""Operator evidence adapter; imports the existing HMM and decision implementation."""
import json
from hri_reasoning.tom import RouteToM,DEFAULT_PRIOR,DEFAULT_TRANSITION
from hri_reasoning.tom_runtime import PublicNode,BayesianToM
from hri_reasoning.rescue_runtime import WarningDecision
from hri_reasoning.core import pose_is_fresh
from hri_reasoning.lifecycle import run


class OperatorInference(PublicNode):
    observe=BayesianToM.observe
    warning=BayesianToM.warning

    def __init__(self):
        super().__init__('bayesian_tom')
        self.model=RouteToM(self.map,beta=float(self.parameter('assumed_beta',1.2)),
            lapse=float(self.parameter('assumed_lapse',.03)),
            prior=json.loads(self.parameter('tom_prior',json.dumps(DEFAULT_PRIOR))),
            transition=json.loads(self.parameter('tom_transition',json.dumps(DEFAULT_TRANSITION))))
        self.warning_compliance=float(self.parameter('assumed_compliance',.8));self.warnings_seen=set()
        self.ack_acquisition=float(self.parameter('ack_acquisition',.9));self.acknowledged=set()
        self.intent=None;self.last_decision=None
        self.subscribe_json('/hri/observations',self.observe)
        self.subscribe_json('/hri/warning',self.warning)
        self.subscribe_json('/hri/acknowledgement',self.ack)
        self.subscribe_json('/hri/operator_intent',self.action)
        self.create_timer(.2,self.tick)

    def action(self,data):
        if data.get('subject')!='operator' or data.get('source')!='operator_input':return
        if not pose_is_fresh(self.stamp(),data.get('source_pose_stamp')):return
        self.intent=data
        if data['decision_id']==self.last_decision:return
        self.last_decision=data['decision_id'];self.model.advance(self.stamp())
        self.model.goal=data['destination']
        if self.model.action(data.get('origin'),data.get('target'),self.stamp()):
            self.publish_json('/hri/tom_evidence',{'event':'operator_action','origin':data['origin'],
                'target':data['target'],'decision_id':data['decision_id'],'source_pose_stamp':data['source_pose_stamp'],
                'evidence_modality':'explicit waypoint/corridor selection; robot motion is not operator evidence'})

    def ack(self,data):
        edge=data.get('edge');identifier=data.get('warning_id')
        if identifier in self.acknowledged or edge not in self.model.observed:return
        self.model.advance(self.stamp());self.acknowledged.add(identifier)
        self.model.information(edge,self.model.observed[edge]['state'],self.ack_acquisition,self.stamp())

    def tick(self):
        now=self.stamp();self.model.advance(now)
        if not self.intent or not pose_is_fresh(now,self.intent.get('source_pose_stamp')):return
        path=self.intent.get('path',[])
        edges=[self.graph.edge_between(a,b).id for a,b in zip(path,path[1:])]
        beliefs=self.model.summary([] if self.intent['completed'] else edges)
        focus=max(beliefs,key=lambda e:(beliefs[e]['hazard_observed'],beliefs[e]['relevant'],beliefs[e]['p_outdated'] or 0))
        payload={**beliefs[focus],'subject':'operator','model':'factorized_route_hmm_v1','beliefs':beliefs,
            'inferred_route':path,'inferred_destination':self.intent['destination'],
            'intent_source':'explicit observed operator commands','last_operator_evidence':self.model.last_action,
            'last_movement_evidence':None,'action_evidence_count':self.model.action_count,
            'hazard_coverage':'robot observations only; camera availability does not imply the operator attended to it'}
        self.publish_json('/hri/belief',payload)
        self.publish_json('/hri/status',{'source':'inference','state':'running','subject':'operator',
            'model':'factorized_route_hmm_v1','operator_actions':self.model.action_count})


class OperatorWarningDecision(WarningDecision):
    def publish_json(self,topic,data):
        if topic=='/hri/decision' and not data.get('hazard_id'):
            data['explanation']='No relevant observed hazard. Continue monitoring robot sensors and explicit operator selections.'
        super().publish_json(topic,data)

    def explanation(self,h,belief,result):
        action=self.beliefs.get('last_operator_evidence')
        evidence=(f'Operator selected {action["origin"]} → {action["target"]} at {action["stamp"]:.1f}s.'
                  if action else 'No discriminating operator selection has been observed yet.')
        route=result.get('route_recommendation',[])
        return (f'{h["edge"].replace("_"," ").title()} has observed {h["kind"]} ({h["source"]}, {h["observed_at"]:.1f}s). '
            f'{evidence} Estimated incompatible operator belief: {result["context"]["probability"]:.0%}; this remains uncertain. '
            f'Recommended alternative: {" → ".join(route) or "no sufficiently safe known route; stop and request assistance"}. '
            'The operator must approve a revised route. '+result['reason'])


def main():run(OperatorInference)
def warning_main():run(OperatorWarningDecision)
