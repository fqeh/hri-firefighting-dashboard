import copy,json,math
from pathlib import Path
import pytest
from hri_reasoning.warning_model import (ACTIONS,Context,DecisionPolicy,WarningMDP,
    immediate_losses,expected_losses,signal_outcomes,warning_threshold,baseline_actions)
from hri_reasoning.route_risk import recommend
from hri_reasoning.rescue_runtime import warning_route_recommendation
from hri_reasoning.response_model import HumanResponse

ROOT=Path(__file__).resolve().parents[1]
def config():return json.loads((ROOT/'config/warning_policy.json').read_text())
def ctx(**kw):return Context(**{'probability':.9,'severity':1.,'urgency':1,'uncertainty':.2,**kw})


def test_expected_loss_is_parameterized_and_severity_sensitive():
    c=config();low=immediate_losses(ctx(severity=.5),c);high=immediate_losses(ctx(severity=2),c)
    assert high['SILENT']==4*low['SILENT']
    assert high['EXPLICIT_WARNING']<high['SILENT']
    c['costs']['explicit_interruption']=1000
    assert min(expected_losses(ctx(),c,False),key=expected_losses(ctx(),c,False).get)!='EXPLICIT_WARNING'
    assert immediate_losses(ctx(approaching=False),config())['SILENT']==0


def test_posterior_changes_action_with_other_context_held_constant():
    a=DecisionPolicy(config()).choose('tree',1,1,ctx(probability=.01),False)
    b=DecisionPolicy(config()).choose('tree',1,1,ctx(probability=.9),False)
    assert a['action']=='SILENT'
    assert b['action']=='EXPLICIT_WARNING'


def test_classification_and_uncertain_observation():
    p=DecisionPolicy(config())
    result=p.choose('tree',1,1,ctx(informative_actions=0,uncertainty=.95),True)
    assert result['action']=='OBSERVE' and result['classification']=='belief_uncertain'
    assert p.choose('tree',1,2,ctx(informative_actions=0),True)['action']=='SILENT'
    assert p.choose('other',1,3,ctx(task_relevant=False),True)['classification']=='no_relevant_hazard'
    assert p.choose('avoided',1,3,ctx(approaching=False),True)['classification']=='likely_aware_or_avoiding'
    expensive=config();expensive['costs']['observation']=10000
    assert DecisionPolicy(expensive).choose('h',1,1,ctx(informative_actions=0,uncertainty=.95),True)['action']=='SILENT'


def test_baselines_share_interface_and_suppress_repeats():
    for baseline in ('B1','B2','B3','B4','oracle'):
        p=DecisionPolicy(config(),baseline)
        result=p.choose('h',1,1,ctx(distance=3),False)
        assert result['action'] in ACTIONS
        if baseline=='B1':assert result['action']=='SILENT'
        else:
            assert result['action']=='EXPLICIT_WARNING'
            assert p.choose('h',1,2,ctx(distance=3),False)['classification']=='repeated_suppressed'
    b3=DecisionPolicy(config(),'B3')
    b3.choose('h',1,1,ctx(),False)
    assert b3.choose('h',1,20,ctx(),False)['action']=='SILENT'
    assert b3.choose('h',2,20,ctx(),False)['action']=='EXPLICIT_WARNING'
    # A noisy new revision must not evade the hazard-level interruption limit.
    assert b3.choose('h',3,21,ctx(),False)['action']=='SILENT'


def test_acknowledgement_is_not_permanent_immunity_to_emergency():
    p=DecisionPolicy(config())
    p.choose('h',1,1,ctx(),False);p.acknowledge('h',1,2)
    assert p.choose('h',1,15,ctx(),False)['classification']=='previously_acknowledged'
    emergency=p.choose('h',1,16,ctx(probability=.01,urgency=2,distance=.7),False)
    assert emergency['action']=='EXPLICIT_WARNING' and emergency['emergency']
    assert p.choose('h',1,17,ctx(urgency=2),False)['action']=='SILENT'
    assert DecisionPolicy(config(),'B1').choose('h',1,1,ctx(urgency=2),False)['action']=='SILENT'
    assert DecisionPolicy(config()).choose('h',1,1,ctx(urgency=2,approaching=False),False)['action']=='EXPLICIT_WARNING'


def test_mdp_stochastic_rows_terminal_and_reward():
    mdp=WarningMDP(config())
    for s in range(len(mdp.states)):
        for action in ACTIONS:
            row=mdp.transitions(s,action)
            assert sum(row.values())==pytest.approx(1)
            assert all(p>=0 for p in row.values())
            assert mdp.reward(s,action)==-mdp.cost(s,action)
    assert mdp.transitions(mdp.terminal,'OBSERVE')=={mdp.terminal:1.}
    assert mdp.cost(mdp.terminal,'EXPLICIT_WARNING')==0


def test_mdp_acquisition_and_observation_preserve_correct_probabilities():
    c=config();mdp=WarningMDP(c);s=mdp.state_for(ctx(probability=1,urgency=0))
    row=mdp.transitions(s,'EXPLICIT_WARNING')
    mean=sum(mdp.context(n).probability*p for n,p in row.items() if n!=mdp.terminal)/(1-c['dynamics']['completion_probability'])
    assert mean==pytest.approx(1-c['response']['explicit_effectiveness'])
    assert sum(m*p for m,p in signal_outcomes(.3,.8))==pytest.approx(.3)


def test_value_iteration_convergence_and_discount_zero():
    mdp=WarningMDP(config());result=mdp.value_iteration()
    assert result['converged'] and result['residual']<1e-8 and result['iterations']<1000
    c=config();c['discount']=0;mdp=WarningMDP(c);result=mdp.value_iteration()
    for s in range(mdp.terminal):
        assert result['values'][s]==min(mdp.cost(s,a) for a in ACTIONS)
    assert not WarningMDP(config()).value_iteration(max_iterations=1)['converged']


def test_q_seed_reproducible_evaluation_frozen_and_fallback():
    mdp=WarningMDP(config())
    a=mdp.train_q(seed=9,episodes=200,steps=5);b=mdp.train_q(seed=9,episodes=200,steps=5)
    assert a==b
    before=copy.deepcopy(a)
    first=mdp.evaluate(a['policy'],seed=19,episodes=50)
    assert first==mdp.evaluate(a['policy'],seed=19,episodes=50)
    assert a==before and first['exploration']==0
    fallback=DecisionPolicy(config(),engine='q_learning')
    assert fallback.fallback and fallback.choose('h',1,1,ctx(),False)['engine']=='expected_loss'
    assert DecisionPolicy(config(),engine='q_learning',table=a).table
    broken={**a,'policy':{'invalid-index':'OBSERVE'}}
    assert DecisionPolicy(config(),engine='q_learning',table=broken).fallback
    assert DecisionPolicy(config(),engine='q_learning',table=['invalid']).fallback


def test_risk_route_avoids_blockage_and_can_prefer_longer_path():
    c=json.loads((ROOT.parent/'hri_sim/config/forest_scenarios/F.json').read_text())
    clear={e['id'] for e in c['edges']};costs=config()['route']
    a=recommend(c,[0,-2],[],clear,costs)
    h={'id':'h','kind':'tree','edge':'trail_a','position':[7,-2,0],'state':'blocked','radius':2.}
    b=recommend(c,[0,-2],[h],clear,costs)
    assert 'trail_a' in a['edges'] and 'trail_a' not in b['edges'] and 'trail_b' in b['edges']
    fire={**h,'kind':'fire','state':'open_hazardous'}
    f=recommend(c,[0,-2],[fire],clear,costs)
    assert f['edge_costs']['trail_a']['weighted_exposure']>0 and 'trail_a' not in f['edges']
    assert sum(f['edge_costs'][e]['distance_m'] for e in f['edges'])>sum(a['edge_costs'][e]['distance_m'] for e in a['edges'])
    unknown=recommend(c,[0,-2],[],[],costs)
    assert not unknown['verified'] and 'No verified safe route' in unknown['explanation']
    shut={**h,'edge':'approach','position':[2,-2,0]}
    north={**h,'id':'h2','edge':'trail_d','position':[-.6,4,0]}  # Trail D also leaves from the start
    assert recommend(c,[0,-2],[shut],clear,costs)['recommended'][:2]==['start','trail_d']
    assert recommend(c,[0,-2],[shut,north],clear,costs)['safety']=='unavailable'

def test_warning_waits_for_safe_route_and_never_recommends_observed_hazard():
    stale={'edges':['access','corridor_b','east_exit'],'recommended':['gate','junction','corridor_b','assembly'],
        'edge_states':{'corridor_b':'clear'}}
    recommendation,waiting_since=warning_route_recommendation(stale,'corridor_b',10.,None)
    assert recommendation is None and waiting_since==10.

    updated={**stale,'edges':['access','corridor_c','north_road','assembly_road'],
        'recommended':['junction','north_junction','north_service','assembly'],
        'edge_states':{'corridor_b':'blocked'}}
    recommendation,waiting_since=warning_route_recommendation(updated,'corridor_b',10.2,waiting_since)
    assert recommendation==updated['recommended'] and 'corridor_b' not in recommendation
    assert waiting_since is None

    recommendation,waiting_since=warning_route_recommendation(stale,'corridor_b',11.1,10.)
    assert recommendation==[] and waiting_since==10.
    unsafe={'edges':stale['edges'],'recommended':stale['recommended'],
        'edge_states':{'corridor_b':'blocked'}}
    assert warning_route_recommendation(unsafe,'corridor_b',10.,None)==([],None)
    assert warning_route_recommendation(None,'corridor_b',10.,None)==([],None)


def test_modeled_response_ack_compliance_and_duplicate_separation():
    c=json.loads((ROOT/'config/human_response.json').read_text())
    c.update(acknowledgement_probability=1.,base_compliance=0.)
    model=HumanResponse(c,7,.8);r=model.receive(('source',1),'explicit')
    assert r['acknowledged'] and not r['complied'] and not r['route_changed']
    assert model.receive(('source',1),'explicit') is None
    assert model.trust==pytest.approx(.795)
    c.update(base_compliance=1.);model=HumanResponse(c,7,1.)
    assert model.receive(('source',1),'explicit')['route_changed']


def test_normal_policy_module_has_no_private_input():
    source=(ROOT/'hri_reasoning/rescue_runtime.py').read_text()
    assert '/evaluation/' not in source and 'crew_initial_knowledge' not in source
    assert 'ResearchNode' not in source


@pytest.mark.parametrize('mutation',[lambda c:c.update(discount=1),
    lambda c:c['costs'].update(missed_danger=-1),lambda c:c['response'].update(explicit_effectiveness=2),
    lambda c:c['route'].update(exposure_weight=-1),lambda c:c['route'].update(unknown_penalty=.5),
    lambda c:c.update(step_seconds=0),lambda c:c.update(probability_bins=3.5)])
def test_bad_costs_and_probabilities_rejected(mutation):
    c=config();mutation(c)
    with pytest.raises(ValueError):DecisionPolicy(c)


def test_threshold_is_the_break_even_probability_for_alerting():
    c=config()
    for urgency in (0,1):
        threshold=warning_threshold(ctx(urgency=urgency),c)
        assert 0<threshold['probability']<1
        for delta,alert_wins in ((-1e-6,False),(1e-6,True)):
            losses=immediate_losses(ctx(urgency=urgency,probability=threshold['probability']+delta),c)
            best_alert=min(losses['SOFT_CUE'],losses['EXPLICIT_WARNING'])
            assert (best_alert<losses['SILENT'])==alert_wins
    # Closer hazards lower the bar for speaking.
    assert warning_threshold(ctx(urgency=1),c)['probability']<warning_threshold(ctx(urgency=0),c)['probability']
    assert warning_threshold(ctx(approaching=False),c) is None


def test_baseline_actions_contrast_belief_aware_and_change_alerts():
    c=config()
    aware=baseline_actions(ctx(probability=.05,approaching=False,distance=10),c)
    assert aware=={'B1':'SILENT','B2':'SILENT','B3':'EXPLICIT_WARNING','B4':'SILENT'}
    unaware=baseline_actions(ctx(probability=.9,distance=10),c)
    assert unaware['B4']!='SILENT' and unaware['B2']=='SILENT'
    assert baseline_actions(ctx(urgency=2,distance=1),c)['B4']=='EXPLICIT_WARNING'
    result=DecisionPolicy(c).choose('h',1,1,ctx(probability=.9),False)
    assert result['baselines']['B4']==result['action'] and result['threshold']['probability']<.9
