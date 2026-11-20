import unittest
from hri_bringup.operator_model import OperatorModel, segment_distance

CONFIG={'nodes':{'s':[0,0],'a':[4,0],'b':[0,4],'g':[4,4]},'edges':[
    {'id':'a','source':'s','target':'a'},{'id':'exit','source':'a','target':'g'},
    {'id':'b','source':'s','target':'b'},{'id':'other','source':'b','target':'g'}]}


class OperatorModelTest(unittest.TestCase):
    def test_hazard_presence_does_not_warn_without_goal(self):
        model=OperatorModel(CONFIG)
        model.observe([{'id':'h','edge':'a','position':[2,0,0],'radius':.5}])
        model.position=(0,0)
        self.assertEqual(model.evaluate(1),([],[],None))

    def test_actual_plan_and_acknowledgment_change_intervention(self):
        model=OperatorModel(CONFIG)
        model.position=(0,0)
        model.observe([{'id':'h','edge':'a','position':[2,0,0],'radius':.5}])
        model.request(4,0)
        model.plan=[(0,0),(4,0)]
        beliefs,warnings,route=model.evaluate(1)
        self.assertEqual(warnings[0]['action'],'explicit')
        self.assertGreater(beliefs[0]['posterior']['unaware'],.5)
        self.assertEqual(route['safety'],'conditional')
        self.assertTrue(model.acknowledge('h'))
        beliefs,warnings,_=model.evaluate(20)
        self.assertFalse(warnings)
        self.assertTrue(beliefs[0]['acknowledged'])
        self.assertEqual(beliefs[0]['posterior']['unaware'],0.)

    def test_nav2_detour_removes_path_relevance(self):
        model=OperatorModel(CONFIG)
        model.position=(0,0)
        model.observe([{'id':'h','edge':'a','position':[2,0,0],'radius':.5}])
        model.request(4,0)
        model.plan=[(0,0),(0,3),(4,3),(4,0)]
        beliefs,warnings,_=model.evaluate(1)
        self.assertFalse(beliefs[0]['relevant'])
        self.assertFalse(warnings)

    def test_segment_distance_handles_duplicates_and_endpoints(self):
        self.assertEqual(segment_distance([2,0],[0,0],[0,0]),2.)
        self.assertEqual(segment_distance([2,1],[0,0],[4,0]),1.)
