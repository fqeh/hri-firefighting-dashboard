import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'evaluation'))
from metrics import summarize
from compare import condition_key
from run_experiments import validate_evidence, scheduled_incidents


class MetricsTests(unittest.TestCase):
    def test_empty_is_incomplete_with_undefined_metrics(self):
        result=summarize([])
        self.assertEqual(result['status'],'incomplete')
        self.assertIsNone(result['belief_accuracy'])
        self.assertIsNone(result['mission_completed'])
        self.assertIsNone(result['warning_precision'])

    def test_known_ground_truth_yields_defined_metrics(self):
        def row(topic,stamp,**data):return {'topic':topic,'stamp':stamp,'data':{'stamp':stamp,**data}}
        records=[row('run_metadata',0,scenario='forest',seed=7,policy='B4',
                     config={'nodes':{'s':[0,0],'g':[3,0]},'goal':'g','edges':[{'id':'a','source':'s','target':'g'}]}),
            row('/evaluation/truth',1,event='hazard_created',hazard={'id':'h','edge':'a','position':[2,0,0],'radius':1}),
            row('/evaluation/human_belief',1.1,knowledge={'a':'clear'},trust=.8),
            row('/hri/human_intent',1.2,position=[0,0],path=['s','g'],completed=False),
            row('/human/odom',1.2,position=[0,0,0]),
            row('/hri/observations',1.3,hazards=[{'id':'h'}]),
            row('/hri/belief',1.4,hazard_id='h',posterior={'unaware':.9}),
            row('/hri/warning',1.5,hazard_id='h',action='explicit'),
            row('/evaluation/human_belief',1.6,knowledge={'a':'blocked'},trust=.79,complied=True),
            row('/hri/human_intent',2,position=[2,0],path=['s','g'],completed=False),
            row('/human/odom',2,position=[2,0,0]),
            row('/hri/human_intent',3,position=[3,0],path=['g'],completed=True),
            row('/human/odom',3,position=[3,0,0]),
            row('run_end',4,valid=True,human_beta=.2,assumed_beta=1.2,trust=.8,observation_radius=3,requested_duration_s=3)]
        result=summarize(records)
        self.assertEqual(result['status'],'complete')
        self.assertEqual(result['warning_precision'],1)
        self.assertEqual(result['warning_recall'],1)
        self.assertAlmostEqual(result['belief_brier_score'],.01)
        self.assertAlmostEqual(result['detection_lead_time_s'],.7)
        self.assertTrue(result['mission_completed'])
        self.assertEqual(result['simulated_compliance'],1)
        self.assertEqual(result['missed_warnings'],0)
        self.assertEqual(result['human_beta'],.2)
        self.assertFalse(result['time_to_safety_censored'])
        self.assertEqual(result['lead_time_samples'],1)

    def test_failed_run_end_is_never_complete(self):
        result=summarize([{'topic':'/hri/human_intent','stamp':1,'data':{'position':[0,0],'path':[]}},
                          {'topic':'run_end','stamp':2,'data':{'valid':False}}])
        self.assertEqual(result['status'],'incomplete')

    def test_declared_intent_is_not_independent_physical_evidence(self):
        result=summarize([{'topic':'/hri/human_intent','stamp':1,'data':{'position':[0,0],'path':[],'completed':True}},
                          {'topic':'run_end','stamp':2,'data':{'valid':True}}])
        self.assertFalse(result['physical_telemetry_present'])
        self.assertEqual(result['status'],'incomplete')
        self.assertIsNone(result['time_to_safety_s'])

    def test_all_behavior_conditions_remain_separate(self):
        base={'scenario':'forest','policy':'B4','human_beta':1.2,'assumed_beta':1.2,'trust':.8,
              'observation_radius':3,'passive_map_rate':.05,'requested_duration_s':90,'config_sha256':'hash'}
        for key,value in (('human_beta',.2),('assumed_beta',.2),('trust',.5),('observation_radius',2),
                          ('passive_map_rate',0),('requested_duration_s',10),('config_sha256','other')):
            self.assertNotEqual(condition_key(base),condition_key({**base,key:value}))

    def test_trial_requires_real_pose_sensors_and_incident(self):
        def row(topic,**data): return {'topic':topic,'data':data}
        records=[row('run_metadata',config={'hazards':[{'id':'h','at':3}]}),
                 row('/human/odom',stamp=1,position=[0,0,0]),row('/human/odom',stamp=6,position=[1,0,0]),
                 row('/hri/human_intent',position=[1,0],path=['s','g']),
                 row('/evaluation/sensors',source='human'),row('/evaluation/sensors',source='uav')]
        self.assertEqual(validate_evidence(records,5),(False,'missing_first_physical_incident'))
        records.append(row('/evaluation/truth',event='hazard_created',hazard={'id':'h'}))
        self.assertEqual(validate_evidence(records,5),(False,'missing_first_incident_detection'))
        records.append(row('/hri/observations',hazards=[{'id':'h'}]))
        self.assertTrue(validate_evidence(records,5)[0])
        records.append(row('/hri/events',event='injection_failed'))
        self.assertEqual(validate_evidence(records,5),(False,'hazard_injection_failed'))

    def test_later_scheduled_incidents_must_exist_but_may_be_unobserved(self):
        def row(topic,**data): return {'topic':topic,'data':data}
        records=[row('run_metadata',config={'hazards':[{'id':'first','at':3},{'id':'later','at':24}]}),
                 row('/human/odom',stamp=1,position=[0,0,0]),row('/human/odom',stamp=30,position=[1,0,0]),
                 row('/hri/human_intent',position=[1,0],path=['s','g']),
                 row('/hri/status',source='scenario',elapsed=29),
                 row('/evaluation/sensors',source='human'),row('/evaluation/sensors',source='uav'),
                 row('/evaluation/truth',event='hazard_created',hazard={'id':'first'}),
                 row('/hri/observations',hazards=[{'id':'first'}])]
        self.assertEqual(validate_evidence(records,29),(False,'missing_scheduled_physical_incident'))
        records.append(row('/evaluation/truth',event='hazard_created',hazard={'id':'later'}))
        self.assertTrue(validate_evidence(records,29)[0])

    def test_incident_deadlines_follow_scenario_clock_not_pose_start(self):
        records=[{'topic':'run_metadata','data':{'config':{'hazards':[{'id':'later','at':24}]}}},
                 {'topic':'/hri/status','data':{'stamp':.5,'source':'scenario','elapsed':0}},
                 {'topic':'/human/odom','data':{'stamp':24.4}}]
        self.assertEqual(scheduled_incidents(records,30),[])
        records.append({'topic':'/human/odom','data':{'stamp':24.5}})
        self.assertEqual([h['id'] for h in scheduled_incidents(records,30)],['later'])

    def test_dead_scenario_manager_cannot_hide_later_due_events(self):
        records=[{'topic':'run_metadata','data':{'config':{'hazards':[{'id':'later','at':24}]}}},
                 {'topic':'/hri/status','data':{'stamp':.5,'source':'scenario','elapsed':0}},
                 {'topic':'/human/odom','data':{'stamp':70}}]
        self.assertEqual([h['id'] for h in scheduled_incidents(records,69)],['later'])


if __name__=='__main__':unittest.main()
