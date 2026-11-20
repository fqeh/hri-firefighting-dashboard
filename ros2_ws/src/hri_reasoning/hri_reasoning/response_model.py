"""Seeded simulated warning response; modeled trust is not measured human trust."""
import math
from .core import behavior_rng


class HumanResponse:
    def __init__(self,config,seed,trust):
        for name,value in config.items():
            if name in ('note','schema_version'):continue
            if not isinstance(value,(float,int)) or not math.isfinite(value) or value<0:
                raise ValueError(f'Invalid response setting {name}')
            if name!='acknowledgement_delay_s' and value>1:raise ValueError('Response probabilities must be <=1')
        if not 0<=trust<=1:raise ValueError('Modeled trust must be in [0,1]')
        self.config=config;self.trust=trust;self.seen=set()
        self.compliance=behavior_rng(seed,'compliance');self.ack=behavior_rng(seed,'acknowledgement')
        self.reroute=behavior_rng(seed,'route_response')

    def receive(self,key,action):
        if key in self.seen:return None
        self.seen.add(key);c=self.config;before=self.trust
        probability=c['base_compliance']*self.trust*(c['soft_multiplier'] if action=='soft' else 1.)
        complies=self.compliance.random()<probability
        result={'complied':complies,'route_changed':complies and self.reroute.random()<c['route_change_probability'],
            'acknowledged':self.ack.random()<c['acknowledgement_probability'],
            'compliance_probability':probability,'trust_before':before}
        self.trust=max(c['trust_floor'],self.trust-c['alert_fatigue'])
        result['trust_after']=self.trust
        return result

    def verified_information(self):self.trust=min(1.,self.trust+self.config['verified_information_gain'])
