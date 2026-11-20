import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request,urlopen

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from dashboard_session import Session,validate_command,configuration,handler,update_equipment_contents

class SessionTests(unittest.TestCase):
    def test_allowlisted_commands_and_numeric_bounds(self):
        good={'command':'switch','scenario':'forest','case':'F','seed':7,'policy':'B4','engine':'expected_loss','manual_incidents':True}
        self.assertEqual(validate_command(good),good)
        for fields in ({'scenario':'forest; rm -rf /'}, {'case':'../private'}, {'seed':True},{'seed':-1},{'policy':'shell'},{'extra':'untrusted'},{'manual_incidents':'true'}):
            with self.assertRaises(ValueError):validate_command({**good,**fields})
        with self.assertRaises(ValueError):validate_command({**good,'command':'start_experiment','duration':float('nan')})

    def test_public_parameters_exclude_private_human_knowledge(self):
        for scenario,case in [('forest','E'),('oil_facility','known')]:
            public=configuration({'scenario':scenario,'case':case})
            serialized=json.dumps(public)
            self.assertNotIn('initial_knowledge',serialized);self.assertIn('nodes',public);self.assertIn('incidents',public)
            self.assertNotIn('position',json.dumps(public['incidents']))
            if scenario=='oil_facility':
                aftermath=next(item for item in public['incidents'] if item.get('display_name'))
                self.assertEqual(aftermath['display_name'],'Tank TK-01 damage aftermath')

    def test_reset_uses_full_process_command_and_reports_failures(self):
        calls=[]
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'results').mkdir();(root/'results/scenario-session.json').write_text(json.dumps({'scenario':'forest','case':'F'}))
            runner=lambda args,**kwargs:calls.append(args) or SimpleNamespace(returncode=0,stdout='',stderr='')
            session=Session(root,runner);session.execute({'command':'reset'})
            self.assertEqual(calls[0][-1],'reset');self.assertTrue(session.state['paused']);self.assertFalse(session.busy)
            session.runner=lambda *a,**kw:SimpleNamespace(returncode=1,stdout='',stderr='Docker unavailable')
            session.execute({'command':'resume'});self.assertEqual(session.error,'Docker unavailable')

    def test_active_experiment_rejects_destructive_session_switch(self):
        with tempfile.TemporaryDirectory() as folder:
            session=Session(Path(folder));session.experiment={'state':'running'}
            with self.assertRaises(RuntimeError):session.submit({'command':'reset'})
            session.busy=True
            with self.assertRaises(RuntimeError):session.submit({'command':'resume'})

    def test_equipment_contents_edit_only_changes_registry_metadata(self):
        source=ROOT/'ros2_ws/src/hri_industrial/config/equipment_registry.json'
        original=json.loads(source.read_text())
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'equipment_registry.json';path.write_text(json.dumps(original))
            updated=update_equipment_contents({'equipment_id':'TK-01','contents':{
                'label':'Demo product','material_class':'unknown','source':'Operator supplied label'}},path)
            saved=json.loads(path.read_text())
            self.assertEqual(saved,updated)
            self.assertEqual(saved['equipment'][0]['contents']['label'],'Demo product')
            self.assertEqual(saved['equipment'][0]['gazebo_model'],original['equipment'][0]['gazebo_model'])
            self.assertEqual(saved['equipment'][0]['temperature'],original['equipment'][0]['temperature'])
            self.assertEqual(saved['equipment'][1],original['equipment'][1])

    def test_equipment_contents_edit_rejects_unknown_fields_and_ids(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'equipment_registry.json'
            path.write_text((ROOT/'ros2_ws/src/hri_industrial/config/equipment_registry.json').read_text())
            with self.assertRaises(ValueError):
                update_equipment_contents({'equipment_id':'missing','contents':{
                    'label':'X','material_class':'unknown','source':'test'}},path)
            with self.assertRaises(ValueError):
                update_equipment_contents({'equipment_id':'TK-01','contents':{
                    'label':'X','material_class':'unknown','source':'test','temperature':200}},path)

    def test_equipment_registry_endpoint_enforces_experiment_lock_and_saves(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);path=root/'ros2_ws/src/hri_industrial/config/equipment_registry.json'
            path.parent.mkdir(parents=True)
            path.write_text((ROOT/'ros2_ws/src/hri_industrial/config/equipment_registry.json').read_text())
            session=Session(root);session.experiment={'state':'running'}
            server=ThreadingHTTPServer(('127.0.0.1',0),handler(session))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            request_data={'equipment_id':'TK-01','contents':{
                'label':'Operator label','material_class':'unknown','source':'Operator supplied'}}
            request=Request(f'http://127.0.0.1:{server.server_port}/api/session/equipment-registry',
                data=json.dumps(request_data).encode(),headers={'Content-Type':'application/json'},method='POST')
            try:
                with self.assertRaises(HTTPError) as rejected:urlopen(request)
                self.assertEqual(rejected.exception.code,409)
                session.experiment={'state':'stopped'}
                with urlopen(request) as response:
                    self.assertEqual(response.status,200)
                    self.assertTrue(json.load(response)['accepted'])
                saved=json.loads(path.read_text())
                self.assertEqual(saved['equipment'][0]['contents']['label'],'Operator label')
                self.assertEqual(saved['equipment'][0]['temperature']['status'],'not_modeled')
            finally:
                server.shutdown();server.server_close();thread.join(timeout=2)

if __name__=='__main__':unittest.main()
