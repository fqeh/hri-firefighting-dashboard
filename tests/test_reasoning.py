import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/hri_reasoning'))
from hri_reasoning.core import (RouteGraph, BeliefFilter, WarningPolicy, BeliefMDP, action_costs,
                                sample_action, validate_behavior_parameters, pose_is_fresh,
                                PassiveMapAttention, glance_probability, behavior_rng)


CONFIG = {'nodes':{'start':[0,0],'a':[2,0],'b':[0,3],'goal':[4,0]},
          'edges':[{'id':'short','source':'start','target':'a'},
                   {'id':'exit','source':'a','target':'goal'},
                   {'id':'long','source':'start','target':'b'},
                   {'id':'bypass','source':'b','target':'goal'}]}


class ReasoningTests(unittest.TestCase):
    def setUp(self): self.graph = RouteGraph(CONFIG)

    def test_unknown_routes_are_conditional(self):
        route = self.graph.shortest('start','goal',{})
        self.assertEqual(route['safety'],'conditional')
        self.assertEqual(set(route['unknown_edges']),{'short','exit'})

    def test_obstruction_replans_on_physical_graph(self):
        route = self.graph.shortest('start','goal',{'short':'blocked'})
        self.assertEqual(route['path'],['start','b','goal'])
        self.assertNotIn('short',route['unknown_edges'])
        self.assertEqual(self.graph.shortest('start','goal',{'short':'blocked','long':'blocked'})['safety'],'unavailable')

    def test_boltzmann_probability_and_rationality(self):
        neutral = self.graph.action_distribution('start','goal',{},0)
        rational = self.graph.action_distribution('start','goal',{},2)
        self.assertAlmostEqual(sum(rational.values()),1)
        self.assertEqual(neutral,{'a':.5,'b':.5})
        self.assertGreater(rational['a'],.99)
        first, second = random.Random(3),random.Random(3)
        self.assertEqual([sample_action(neutral,first) for _ in range(20)],
                         [sample_action(neutral,second) for _ in range(20)])

    def test_hmm_actions_disambiguate_hidden_beliefs(self):
        unaware,aware = BeliefFilter(),BeliefFilter()
        unaware.update_action(self.graph,'start','a','goal','short')
        aware.update_action(self.graph,'start','b','goal','short')
        self.assertGreater(unaware.unaware,.95)
        self.assertLess(aware.unaware,.1)
        self.assertLess(unaware.entropy,1)

    def test_uncertain_prior_and_transition(self):
        belief = BeliefFilter()
        self.assertAlmostEqual(belief.entropy,1)
        belief.predict(steps=0,warning_compliance=.8)
        self.assertAlmostEqual(belief.unaware,.1)
        belief.predict(steps=100)
        self.assertTrue(0<=belief.unaware<=1)

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError): BeliefFilter(prior_unaware=2)
        with self.assertRaises(ValueError): action_costs(-1)
        with self.assertRaises(ValueError): self.graph.action_distribution('start','goal',{},-2)
        with self.assertRaises(ValueError): BeliefMDP(discount=1)

    def test_behavior_parameters_cannot_hide_nonfinite_or_invalid_values(self):
        for kwargs in ({'beta':float('nan')},{'assumed_beta':float('inf')},{'trust':1.1},
                       {'trust':-.2},{'observation_radius':0},{'beta':-1},
                       {'passive_map_rate':-.01},{'passive_map_rate':float('inf')}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError): validate_behavior_parameters(**kwargs)
        validate_behavior_parameters(beta=0,trust=0,observation_radius=.1)

    def test_stale_odometry_cannot_drive_human_controller(self):
        self.assertTrue(pose_is_fresh(10,9.9))
        self.assertTrue(pose_is_fresh(10,10.05))
        self.assertFalse(pose_is_fresh(10,9))
        self.assertFalse(pose_is_fresh(10,None))
        self.assertFalse(pose_is_fresh(10,12))
        self.assertFalse(pose_is_fresh(10,float('nan')))

    def test_passive_map_probability_has_per_second_units(self):
        self.assertAlmostEqual(glance_probability(.05,20),1-math.exp(-1))
        self.assertEqual(glance_probability(0,20),0)
        self.assertEqual(glance_probability(.05,0),0)
        with self.assertRaises(ValueError): glance_probability(-1,1)

    def test_passive_map_clock_reproducible_and_tick_independent(self):
        dense,sparse = PassiveMapAttention(.05,7),PassiveMapAttention(.05,7)
        dense.poll(0);sparse.poll(0)
        dense_events = [event for tick in range(1,1001) for event in dense.poll(tick/10)]
        self.assertEqual(dense_events,sparse.poll(100))
        self.assertTrue(dense_events)
        disabled = PassiveMapAttention(0,7)
        self.assertEqual(disabled.poll(0),[])
        self.assertEqual(disabled.poll(10000),[])

    def test_warning_draws_do_not_change_routes_or_map_glances(self):
        route_a,route_b = behavior_rng(7,'route'),behavior_rng(7,'route')
        compliance = behavior_rng(7,'compliance')
        attention = PassiveMapAttention(.05,7)
        attention.poll(0);attention.poll(500)
        for _ in range(100): compliance.random()
        self.assertEqual([route_a.random() for _ in range(10)],[route_b.random() for _ in range(10)])

    def test_baselines_have_distinct_rules(self):
        def choose(policy,p=.95,relevant=True,distance=10):
            return policy.choose('fire',10,p,relevant,distance)
        self.assertEqual(choose(WarningPolicy('B1')),'silent')
        self.assertEqual(choose(WarningPolicy('B2')),'silent')
        self.assertEqual(choose(WarningPolicy('B2'),distance=1),'explicit')
        every = WarningPolicy('B3')
        self.assertEqual(choose(every,p=.01),'explicit')
        self.assertEqual(every.choose('fire',50,.95,True,1),'silent')
        self.assertEqual(choose(WarningPolicy('B4'),p=.01),'silent')
        self.assertEqual(choose(WarningPolicy('B4')),'explicit')
        self.assertEqual(choose(WarningPolicy('B4'),relevant=False),'silent')

    def test_oracle_is_explicit(self):
        oracle = WarningPolicy('oracle')
        with self.assertRaises(ValueError): oracle.choose('tree',1,.9,True,1)
        self.assertEqual(oracle.choose('tree',1,.9,True,1,False),'silent')
        self.assertEqual(oracle.choose('tree',2,.1,True,1,True),'explicit')

    def test_cooldown_prevents_repeated_interruptions(self):
        policy = WarningPolicy()
        self.assertEqual(policy.choose('tree',1,.9,True,1),'explicit')
        self.assertEqual(policy.choose('tree',2,.9,True,1),'silent')
        self.assertEqual(policy.choose('tree',12,.9,True,1),'explicit')

    def test_asymmetric_miss_cost(self):
        self.assertGreater(action_costs(.9)['silent'],action_costs(.9)['explicit'])
        self.assertLess(action_costs(.01)['silent'],action_costs(.01)['explicit'])

    def test_mdp_converges_and_q_learning_is_reproducible(self):
        model = BeliefMDP()
        solved = model.value_iteration()
        self.assertTrue(solved['converged'])
        self.assertEqual(solved['policy'][0],'silent')
        self.assertTrue(all(math.isfinite(v) for v in solved['values']))
        learned = model.q_learning(seed=2,episodes=300)
        self.assertEqual(learned,model.q_learning(seed=2,episodes=300))
        self.assertIn(learned['policy'][10],model.actions)


if __name__ == '__main__': unittest.main()
