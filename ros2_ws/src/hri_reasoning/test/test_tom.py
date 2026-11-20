import ast
import copy
import math
from pathlib import Path
import pytest
from hri_reasoning.tom import (STATES, CategoricalHMM, RouteToM, public_map,
                               action_distribution, validate_public_map, semantic_hazard_kind)


def world():
    return {'nodes':{'start':[0,0],'short':[1,0],'long':[1,2],'goal':[2,0]},
        'edges':[{'id':'a','source':'start','target':'short'},
                 {'id':'b','source':'short','target':'goal'},
                 {'id':'c','source':'start','target':'long'},
                 {'id':'d','source':'long','target':'goal'}], 'start':'start','goal':'goal'}


def test_exact_bayes_and_contradiction():
    f=CategoricalHMM((.25,)*4)
    f.evidence([.8,.1,.05,.05])
    assert f.p == pytest.approx((.8,.1,.05,.05))
    f.evidence([.01,1,1,1])
    assert f.p == pytest.approx((.008/.208,.1/.208,.05/.208,.05/.208))
    assert sum(f.p) == pytest.approx(1)


def test_exact_hmm_and_information_transition():
    transition=((.8,.2,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
    f=CategoricalHMM((1,0,0,0),transition)
    f.predict(2)
    assert f.p == pytest.approx((.64,.36,0,0))
    f.information('blocked',.75)
    assert f.p == pytest.approx((.16,.84,0,0))


def test_uninformative_and_impossible_evidence():
    f=CategoricalHMM()
    before=f.p
    assert f.evidence([1,1,1,1])
    assert f.p == pytest.approx(before)
    assert not f.evidence([0,0,0,0])
    assert f.p == pytest.approx(before)


def test_extreme_beta_likelihood_and_repeated_normalization():
    model=RouteToM(world(),beta=1e6,lapse=.001)
    for _ in range(200):
        model.action('start','short',1.)
        for f in model.filters.values():
            assert sum(f.p) == pytest.approx(1)
            assert all(math.isfinite(x) for x in f.p)
    f=CategoricalHMM((.25,)*4)
    f.evidence([1e-300,2e-300,3e-300,4e-300])
    assert f.p == pytest.approx((.1,.2,.3,.4))


def test_safe_choice_supports_safe_and_contradicts_blocked():
    model=RouteToM(world())
    before=model.filters['a'].p
    model.action('start','short',1.)
    after=model.filters['a'].p
    assert after[0] > before[0]
    assert after[1] < before[1]


def test_avoiding_shortcut_supports_blockage():
    model=RouteToM(world())
    before=model.filters['a'].p
    model.action('start','long',1.)
    assert model.filters['a'].p[1] > before[1]
    assert model.filters['a'].p[0] < before[0]


def test_robot_hazard_sighting_does_not_set_human_belief():
    model=RouteToM(world())
    before=model.filters['a'].p
    model.observe('a','blocked',1,[.5,0,0],'seen_tree')
    assert model.filters['a'].p == before
    assert model.summary(['a'])['a']['p_mismatch'] == before[0]+before[2]
    model.information('a','blocked',.9,2.)
    assert model.filters['a'].p[1] > .9
    assert model.summary(['a'])['a']['last_relevant_observation'] == 2.


def test_missing_observation_is_not_safe_and_no_mismatch_is_invented():
    model=RouteToM(world())
    model.action('start','short',2.)
    assert model.summary(['a'])['a']['p_mismatch'] is None
    assert not model.summary(['a'])['a']['hazard_observed']


@pytest.mark.parametrize(('name', 'kind'), [
    ('hazard_debris_tk01_aftermath', 'debris'),
    ('hazard_debris_front_south', 'debris'),
    ('hazard_fire_f1', 'fire'),
    ('hazard_gas', 'gas'),
    ('hazard_tree_fallen', 'tree'),
    ('hazard_unknown_tk01', None),
])
def test_semantic_hazard_ids_include_configured_incident_suffixes(name, kind):
    assert semantic_hazard_kind(name) == kind


def test_unknown_is_separate_from_safe_and_upper_bound():
    model=RouteToM(world(),prior=(.1,.2,.3,.4))
    model.observe('a','open_hazardous',1,[.5,0,0],'seen_fire')
    result=model.summary(['a'])['a']
    assert result['p_mismatch'] == .1
    assert result['mismatch_upper_bound'] == .5
    assert set(result['posterior']) == set(STATES)


def test_unexpected_actions_and_zero_rationality():
    model=RouteToM(world(),beta=0,lapse=.1)
    assert not model.action('not_a_node','teleport',1)
    assert model.action('start','__wait__',2)
    for f in model.filters.values():
        assert sum(f.p) == pytest.approx(1)
        assert all(x > 0 for x in f.p)


def test_public_schema_excludes_private_truth_and_counterfactual_invariance():
    first={**world(),'crew_initial_knowledge':{'a':'blocked'},'hazards':[{'at':1,'edge':'a'}]}
    second=copy.deepcopy(first)
    second['crew_initial_knowledge']={'a':'clear'}
    second['hazards']=[{'at':900,'edge':'d'}]
    with pytest.raises(ValueError): RouteToM(first)
    a,b=RouteToM(public_map(first)),RouteToM(public_map(second))
    for model in (a,b):
        model.observe('a','blocked',1,[.5,0,0],'observed')
        model.action('start','short',2)
    assert a.summary(['a']) == b.summary(['a'])


def test_ros_inference_subscription_allowlist():
    source=(Path(__file__).parents[1]/'hri_reasoning/tom_runtime.py').read_text()
    tree=ast.parse(source)
    topics=set()
    for node in ast.walk(tree):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
            if node.func.attr=='subscribe_json': topics.add(ast.literal_eval(node.args[0]))
            if node.func.attr=='create_subscription' and isinstance(node.args[1],ast.Constant):
                topics.add(node.args[1].value)
    assert topics == {'/human/odom','/hri/human_intent','/hri/observations','/hri/warning'}
    assert '/evaluation/' not in source
    assert 'ResearchNode' not in source
    assert 'crew_initial_knowledge' not in source


@pytest.mark.parametrize('kwargs',[
    {'prior':(0,0,0,0)},{'prior':(-1,1,1,1)}, {'prior':(math.nan,1,1,1)},
    {'transition':((1,1,1,1),)*4}, {'transition':((1,0),)*4}])
def test_invalid_hmm_rejected(kwargs):
    with pytest.raises(ValueError): CategoricalHMM(**kwargs)


def test_fixed_simulation_time_predictions_and_reset_rejection():
    model=RouteToM(world())
    model.advance(.1)
    model.advance(.9)
    before=model.filters['a'].p
    model.advance(1.)
    assert model.filters['a'].p != before
    with pytest.raises(ValueError): model.advance(0.)


def test_factorization_is_linear_in_edges():
    model=RouteToM(world())
    assert len(model.filters)==4
    assert sum(len(f.p) for f in model.filters.values())==16
    graph=validate_public_map(world())
    distribution=action_distribution(graph,'start','goal',{'a':'blocked'},lapse=0)
    assert distribution['short']==0
    assert distribution['long']>0


def test_stall_requires_attempted_travel_and_persistent_lack_of_progress():
    from hri_reasoning.tom import MotionStall
    stall=MotionStall()
    assert not stall.update(0,(0,0),True)
    assert not stall.update(1,(.1,0),True)
    assert not stall.update(2,(.1,0),True)
    assert stall.update(3,(.1,0),True)
    assert not stall.update(4,(.1,0),False)
    assert not stall.update(8,(.1,0),True)
