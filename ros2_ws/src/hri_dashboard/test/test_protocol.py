import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hri_dashboard.contracts import validate_payload, provenance
from hri_dashboard.state import TelemetryStore
from hri_dashboard.server import JSON_TOPICS

class ProtocolTests(unittest.TestCase):
    def test_normalized_beliefs_only(self):
        p={'open_safe':.6,'blocked':.2,'open_hazardous':.1,'unknown_stale':.1}
        self.assertEqual(validate_payload('/hri/belief',{'posterior':p})['posterior'],p)
        for value in (math.nan,math.inf,-.1,.9):
            with self.assertRaises(ValueError):validate_payload('/hri/belief',{'posterior':{**p,'open_safe':value}})
        with self.assertRaises(ValueError):validate_payload('/hri/belief',{'posterior':p,'beliefs':{'a':{'posterior':{'aware':1}}}})

    def test_malformed_sensor_payload_is_rejected(self):
        for data in ({'hazards':{}},{'hazards':[None]},{'hazards':[{'id':'a','position':['x',2]}]}, {'stamp':float('nan'),'hazards':[]}):
            with self.assertRaises(ValueError):validate_payload('/hri/observations',data)

    def test_warning_history_is_deduplicated_persistent_and_receipt_is_not_belief(self):
        now=[1.];store=TelemetryStore(clock=lambda:now[0]);warning={'id':'B4-1','stamp':3.,'hazard_id':'tree','action':'explicit'}
        store.update('/hri/warning',warning);store.update('/hri/warning',warning);now[0]=50
        snapshot=store.snapshot();self.assertEqual(len(snapshot['warnings']),1);self.assertTrue(snapshot['topics']['/hri/warning']['stale'])
        store.warning_action('B4-1','acknowledge_warning',4.)
        self.assertTrue(store.snapshot()['warnings'][0]['acknowledged']);self.assertNotIn('/hri/belief',store.snapshot()['topics'])
        store.warning_action('B4-1','dismiss_warning',5.)
        self.assertTrue(store.snapshot()['warnings'][0]['dismissed'])
        with self.assertRaises(ValueError):store.warning_action('missing','dismiss_warning',6.)

    def test_provenance_epoch_and_private_information_boundary(self):
        first=TelemetryStore();second=TelemetryStore();self.assertNotEqual(first.epoch,second.epoch)
        for topic in ('/human/odom','/robot/odom','/hri/belief','/forest/fire_state'):first.update(topic,{'stamp':4.})
        snapshot=first.snapshot();self.assertEqual(snapshot['schema_version'],1)
        self.assertEqual(snapshot['topics']['/human/odom']['source'],'simulation_truth')
        self.assertEqual(snapshot['topics']['/robot/odom']['source'],'observation')
        self.assertEqual(snapshot['topics']['/hri/belief']['source'],'inference')
        self.assertEqual(provenance('/hri/equipment_registry'),'configuration')
        first.update('/hri/equipment_registry',{'schema_version':1,'stamp':4.,'equipment':[{
            'equipment_id':'TK-01','gazebo_model':'tank_0','position_m':[6,3.6,1.6],
            'contents':{'label':'Unspecified product','material_class':'unknown','source':'Not supplied'}}]})
        self.assertEqual(first.snapshot()['topics']['/hri/equipment_registry']['source'],'configuration')
        with self.assertRaises(ValueError):
            validate_payload('/hri/equipment_registry',{'schema_version':1,'equipment':[{'equipment_id':'bad'}]})
        self.assertFalse(any('/evaluation/' in t for t in JSON_TOPICS))
        self.assertIn('/hri/equipment_registry',JSON_TOPICS)

if __name__=='__main__':unittest.main()
