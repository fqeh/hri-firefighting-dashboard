"""Public-posterior warning decisions. Costs are research parameters, not utilities
elicited from people. No Gazebo state, scenario file or private belief is read here.
"""
from dataclasses import dataclass, asdict
import hashlib
import itertools
import json
import math
import random

ACTIONS = ('SILENT','SOFT_CUE','EXPLICIT_WARNING','OBSERVE')
ALERTS = ACTIONS[1:3]


def validate_config(c):
    for group in ('costs','response','dynamics','thresholds'):
        for name,value in c[group].items():
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
                raise ValueError(f'{group}.{name} must be finite and nonnegative')
    for name in ('soft_effectiveness','explicit_effectiveness','observation_accuracy'):
        if not 0<=c['response'][name]<=1: raise ValueError('Invalid response probability')
    if c['response']['observation_accuracy']<.5: raise ValueError('Observation accuracy must be >= .5')
    for name in ('completion_probability','local_discovery','urgency_progress','acknowledgement_probability'):
        if not 0<=c['dynamics'][name]<=1: raise ValueError('Invalid transition probability')
    if not 0<=c['discount']<1: raise ValueError('Discount must lie in [0,1)')
    if c['probability_bins']<3: raise ValueError('At least three probability bins required')
    if type(c['probability_bins']) is not int:raise ValueError('Probability bins must be an integer')
    if not math.isfinite(c['step_seconds']) or c['step_seconds']<=0:raise ValueError('Decision step must be positive')
    if len(c['urgency_multipliers'])!=3 or not c['severity_bins']:raise ValueError('Invalid discretization')
    for value in [*c['urgency_multipliers'],*c['severity_bins'],*c['severity_by_kind'].values(),*c['route'].values()]:
        if not math.isfinite(value) or value<0:raise ValueError('Severity, urgency and route costs must be nonnegative')
    if c['route']['unknown_penalty']<1:raise ValueError('Unknown routes cannot receive a cost discount')
    return c


def config_digest(config):
    return hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()


@dataclass(frozen=True)
class Context:
    probability: float
    severity: float = 1.
    urgency: int = 0
    uncertainty: float = 1.
    task_relevant: bool = True
    approaching: bool = True
    informative_actions: int = 1
    distance: float = 10.
    acknowledged: bool = False
    repeats: int = 0

    def __post_init__(self):
        if not 0<=self.probability<=1 or not 0<=self.uncertainty<=1:
            raise ValueError('Belief and uncertainty must be probabilities')
        if self.urgency not in (0,1,2) or not math.isfinite(self.severity) or self.severity<0:
            raise ValueError('Invalid hazard severity or urgency')


def signal_outcomes(p,accuracy):
    """Bayesian posteriors for a hypothetical imperfect binary observation."""
    result=[]
    for likelihood_true,likelihood_false in ((accuracy,1-accuracy),(1-accuracy,accuracy)):
        mass=p*likelihood_true+(1-p)*likelihood_false
        if mass>0: result.append((mass,p*likelihood_true/mass))
    return result


def immediate_losses(context, config):
    c,r=config['costs'],config['response']
    p=context.probability if context.approaching and context.task_relevant else 0.
    urgency=config['urgency_multipliers'][context.urgency]
    harm=context.severity*urgency*(c['missed_danger']+c['hazard_exposure'])
    losses={'SILENT':p*harm}
    for action,prefix in (('SOFT_CUE','soft'),('EXPLICIT_WARNING','explicit')):
        effect=r[prefix+'_effectiveness']
        false_scale=.5 if prefix=='soft' else 1.
        losses[action]=(p*harm*(1-effect)+(1-p)*c['unnecessary_alert']*false_scale+
            c[prefix+'_interruption']+c['evacuation_delay']*r[prefix+'_delay_s']+
            c['repeated_warning']*context.repeats)
    losses['OBSERVE']=(c['observation']+c['evacuation_delay']*r['observation_delay_s']+
        p*context.severity*urgency*c['hazard_exposure']*r['observation_delay_s']/config['step_seconds'])
    return losses


def expected_losses(context,config,allow_observe=True):
    losses=immediate_losses(context,config)
    if allow_observe and context.uncertainty>config['thresholds']['observe_uncertainty'] and context.urgency<2:
        future=0.
        for weight,p in signal_outcomes(context.probability,config['response']['observation_accuracy']):
            next_context=Context(**{**asdict(context),'probability':p,'uncertainty':0.})
            future+=weight*min(v for a,v in immediate_losses(next_context,config).items() if a!='OBSERVE')
        losses['OBSERVE']+=config['discount']*future
    else:
        losses.pop('OBSERVE')
    return losses


class WarningMDP:
    """Finite communication approximation, not a physics simulator.

    State: (p-bin, severity-bin, urgency, memory [new/pending/ack], uncertainty).
    One absorbing completion state. Reward is negative immediate expected loss.
    """
    def __init__(self,config):
        self.config=validate_config(config)
        self.bins=config['probability_bins'];self.discount=config['discount']
        self.states=list(itertools.product(range(self.bins),range(len(config['severity_bins'])),range(3),range(3),range(2)))
        self.terminal=len(self.states);self.index={s:i for i,s in enumerate(self.states)}
        self.states.append(None)
        self.transitions_cache={}
        self.loss_cache={}
        for s in range(len(self.states)):
            for a in ACTIONS:
                self.transitions_cache[s,a]=self._transitions(s,a)
                self.loss_cache[s,a]=self._cost(s,a)

    def context(self,state):
        p,s,u,m,h=self.states[state]
        return Context(p/(self.bins-1),self.config['severity_bins'][s],u,float(h),
                       acknowledged=m==2,repeats=int(m!=0))

    def state_for(self,context):
        return self.index[(round(context.probability*(self.bins-1)),
            min(range(len(self.config['severity_bins'])),key=lambda i:abs(self.config['severity_bins'][i]-context.severity)),
            context.urgency,2 if context.acknowledged else int(context.repeats>0),int(context.uncertainty>.5))]

    def _cost(self,state,action):
        return 0. if state==self.terminal else immediate_losses(self.context(state),self.config)[action]

    def cost(self,state,action): return self.loss_cache[state,action]
    def reward(self,state,action): return -self.cost(state,action)
    def transitions(self,state,action): return self.transitions_cache[state,action]

    def _transitions(self,state,action):
        if state==self.terminal: return {state:1.}
        p_index,severity,urgency,memory,uncertain=self.states[state]
        p=p_index/(self.bins-1);c=self.config;d=c['dynamics']
        if action=='OBSERVE': beliefs=signal_outcomes(p,c['response']['observation_accuracy']);uncertain=0
        else:
            effect={'SILENT':0.,'SOFT_CUE':c['response']['soft_effectiveness'],
                    'EXPLICIT_WARNING':c['response']['explicit_effectiveness']}[action]
            # Acquisition reduces the probability of incompatible human knowledge.
            beliefs=[(1.,p*(1-effect)*(1-d['local_discovery']*urgency/2))]
        memories=[(1.,memory)] if action not in ALERTS else [(d['acknowledgement_probability'],2),(1-d['acknowledgement_probability'],1)]
        urgencies=[(1-d['urgency_progress'],urgency),(d['urgency_progress'],min(2,urgency+1))]
        result={self.terminal:d['completion_probability']}
        for bp,p_next in beliefs:
            x=p_next*(self.bins-1);lo=int(math.floor(x));hi=min(self.bins-1,lo+1)
            for interp,index in ((1-(x-lo),lo),(x-lo,hi)):
                for mp,mem in memories:
                    for up,urg in urgencies:
                        mass=(1-d['completion_probability'])*bp*interp*mp*up
                        if mass:
                            nxt=self.index[(index,severity,urg,mem,uncertain)]
                            result[nxt]=result.get(nxt,0.)+mass
        return result

    def value_iteration(self,tolerance=1e-8,max_iterations=2000):
        if tolerance<=0 or max_iterations<=0: raise ValueError('Invalid convergence settings')
        values=[0.]*len(self.states);policy={};residual=math.inf
        for iteration in range(max_iterations):
            updated=[]
            for s in range(len(values)):
                q={a:self.cost(s,a)+self.discount*sum(p*values[n] for n,p in self.transitions(s,a).items()) for a in ACTIONS}
                policy[s]=min(q,key=q.get);updated.append(q[policy[s]])
            residual=max(abs(x-y) for x,y in zip(values,updated));values=updated
            if residual<tolerance: break
        return {'values':values,'policy':policy,'iterations':iteration+1,'residual':residual,
                'converged':residual<tolerance,'config_sha256':config_digest(self.config)}

    def train_q(self,seed=7,episodes=12000,steps=25,alpha=.15,epsilon=.25):
        if episodes<=0 or steps<=0 or not 0<alpha<=1 or not 0<=epsilon<=1:
            raise ValueError('Invalid Q-learning parameters')
        rng=random.Random(seed);q=[[0.]*4 for _ in self.states];visits=[[0]*4 for _ in self.states]
        for _ in range(episodes):
            state=rng.randrange(self.terminal)
            for _ in range(steps):
                a=rng.randrange(4) if rng.random()<epsilon else min(range(4),key=lambda i:q[state][i])
                t=self.transitions(state,ACTIONS[a]);nxt=rng.choices(list(t),weights=list(t.values()))[0]
                target=self.cost(state,ACTIONS[a])+self.discount*min(q[nxt])
                q[state][a]+=alpha*(target-q[state][a]);visits[state][a]+=1;state=nxt
                if state==self.terminal: break
        return {'q':q,'policy':{s:ACTIONS[min(range(4),key=lambda i:q[s][i])] for s in range(len(q))},
            'visits':visits,'seed':seed,'episodes':episodes,'steps':steps,'alpha':alpha,'epsilon':epsilon,
            'config_sha256':config_digest(self.config),'training':'abstract communication MDP only'}

    def evaluate(self,policy,seed=1007,episodes=1000,steps=40):
        """Frozen policy evaluation: no Q table mutation and no exploration."""
        rng=random.Random(seed);costs=[]
        for _ in range(episodes):
            s=rng.randrange(self.terminal);total=0.
            for t in range(steps):
                a=policy[s];total+=self.discount**t*self.cost(s,a)
                nxt=self.transitions(s,a);s=rng.choices(list(nxt),weights=list(nxt.values()))[0]
                if s==self.terminal: break
            costs.append(total)
        return {'mean_discounted_loss':sum(costs)/len(costs),'seed':seed,'episodes':episodes,'exploration':0}


class DecisionPolicy:
    def __init__(self,config,baseline='B4',engine='expected_loss',table=None):
        self.config=validate_config(config)
        if baseline not in ('B1','B2','B3','B4','oracle'): raise ValueError('Unknown baseline')
        if engine not in ('expected_loss','value_iteration','q_learning'): raise ValueError('Unknown engine')
        self.baseline,self.engine=baseline,engine
        self.memory={};self.latest={};self.mdp=None;self.table=None;self.fallback=None
        if engine=='value_iteration':
            self.mdp=WarningMDP(config);result=self.mdp.value_iteration()
            if result['converged']: self.table=result['policy']
            else:self.fallback='Value iteration did not converge'
        elif engine=='q_learning':
            if isinstance(table,dict) and table.get('config_sha256')==config_digest(config):
                self.mdp=WarningMDP(config)
                try:self.table={int(s):a for s,a in table['policy'].items()}
                except (ValueError,KeyError,TypeError,AttributeError):self.table=None
                if (self.table is None or set(self.table)!=set(range(len(self.mdp.states))) or
                    any(a not in ACTIONS for a in self.table.values())):
                    self.table=None;self.fallback='Invalid frozen Q policy'
            else:self.fallback='Missing or incompatible frozen Q policy; using expected loss'

    def acknowledge(self,hazard_id,revision,now):
        key=(hazard_id,revision)
        if key in self.memory:self.memory[key]['acknowledged_at']=now

    def choose(self,hazard_id,revision,now,context,allow_observe=True):
        c=self.config;t=c['thresholds'];key=(hazard_id,revision)
        memory=self.memory.get(key,{'count':0,'last':self.latest.get(hazard_id,-math.inf),'observed':False})
        ack='acknowledged_at' in memory
        context=Context(**{**asdict(context),'acknowledged':ack,'repeats':memory['count']})
        losses=expected_losses(context,c,allow_observe and not memory['observed'])
        emergency=context.task_relevant and context.urgency==2
        category='belief_uncertain'
        action='SILENT';reason='No intervention has lower expected loss'
        if self.baseline=='B1':category='passive_dashboard';reason='B1 never issues proactive alerts'
        elif not context.task_relevant:category='no_relevant_hazard';reason='Observed hazard is outside the evacuation task'
        elif emergency and (memory.get('last_level')!='emergency' or now-memory['last']>=t['emergency_repeat_s']):
            action='EXPLICIT_WARNING';category='immediate_danger';reason='Physical proximity overrides belief uncertainty and acknowledgement'
        elif now-memory['last']<t['repeat_cooldown_s']:
            category='repeated_suppressed';reason='Same hazard revision is in its warning cooldown'
        elif ack and now-memory['acknowledged_at']<t['acknowledgement_hold_s']:
            category='previously_acknowledged';reason='Receipt was acknowledged; compliance remains uncertain'
        elif self.baseline=='B2':
            action='EXPLICIT_WARNING' if context.distance<=t['proximity_m'] else 'SILENT'
            category='proximity';reason='B2 uses distance, without the belief posterior'
        elif self.baseline=='B3':
            action='EXPLICIT_WARNING' if memory['count']==0 else 'SILENT'
            category='hazard_change';reason='B3 alerts once per relevant observed hazard revision'
        elif not context.approaching:
            category='likely_aware_or_avoiding';reason='Current observed route avoids this hazard; belief remains an estimate'
        elif c['require_informative_action'] and context.informative_actions==0 and self.baseline!='oracle':
            category='belief_uncertain';reason='Awaiting an informative crew action before using a non-emergency belief warning'
            if 'OBSERVE' in losses and losses['OBSERVE']<losses['SILENT']:action='OBSERVE'
        else:
            category=('probably_unaware' if context.probability>=t['unaware_probability'] else
                      'probably_aware' if context.probability<=t['aware_probability'] else 'belief_uncertain')
            if self.table:
                action=self.table[self.mdp.state_for(context)]
                if action not in losses:action=min(losses,key=losses.get)
            else:action=min(losses,key=losses.get)
            reason='Minimum modeled expected loss' if not self.table else f'Frozen {self.engine} policy'
        if action in ALERTS:
            memory.update(last=now,count=memory['count']+1,last_level='emergency' if emergency else 'normal')
            self.latest[hazard_id]=now
        if action=='OBSERVE':memory['observed']=True
        self.memory[key]=memory
        return {'action':action,'classification':category,'reason':reason,'losses':losses,
            'context':asdict(context),'engine':self.engine if self.table else 'expected_loss',
            'fallback':self.fallback,'warning_count':memory['count'],'emergency':emergency}
