import json
import math
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from scene_export import compose, export_scene, quaternion, transform
from dashboard_session import Session, handler


class SceneExportTests(unittest.TestCase):
    def test_pose_composition_matches_sdf_fixed_axis_rpy(self):
        # Roll 90° turns a Z-axis cylinder into a log lying along -Y.
        q = quaternion(transform([0, 0, 0, math.pi / 2, 0, 0]))
        self.assertAlmostEqual(q[0], math.sqrt(.5), 4)
        self.assertAlmostEqual(q[3], math.sqrt(.5), 4)
        m = compose(transform([1, 0, 0, 0, 0, math.pi / 2]), transform([2, 0, 0, 0, 0, 0]))
        self.assertAlmostEqual(m[0][3], 1, 6)
        self.assertAlmostEqual(m[1][3], 2, 6)

    def test_forest_scene_has_world_entities_and_no_runtime_hazards(self):
        scene = export_scene(ROOT, {'scenario': 'forest', 'case': 'F', 'seed': 7})
        names = [p['name'] for p in scene['scenery']]
        self.assertIn('route_trail_a/link/visual_visual', names)
        self.assertTrue(any(p['shape'] == 'mesh' and p['indices'] for p in scene['scenery']), 'terrain mesh')
        self.assertEqual(set(scene['entities']), {'human', 'uav'})  # UAV-only lookout: no parked ground robot
        self.assertEqual(scene['spawns']['uav']['position'], [4.0, 6.0, 8.0])
        self.assertFalse([n for n in names if n.startswith('hazard')], 'hazards must come from telemetry only')
        self.assertEqual(scene['sensors']['uav']['horizontal_fov'], 1.2)
        config = json.loads((ROOT / 'ros2_ws/src/hri_sim/config/forest_scenarios/F.json').read_text())
        self.assertEqual(scene['spawns']['human']['position'][:2], config['human_start'][:2])
        self.assertEqual(scene['spawns']['uav']['position'][2], config['uav_start'][2])
        self.assertEqual(set(scene['hazards']), {'tree', 'fire', 'gas', 'debris', 'smoke', 'fire_smoke'})
        json.dumps(scene, allow_nan=False)

    def test_forest_world_is_uav_only_unless_a_test_asks_for_the_ground_robot(self):
        import shutil
        import xml.etree.ElementTree as ET
        sys.path.insert(0, str(ROOT / 'ros2_ws/src/hri_bringup'))
        from hri_bringup.forest_world import prepare_world
        share = ROOT / 'ros2_ws/src/hri_sim'
        for flag, expected in ((None, {'human', 'uav'}), (True, {'human', 'uav', 'robot'})):
            world_file, config_file, directory = prepare_world(share, 'F', 7, None, flag)
            try:
                names = {i.findtext('name') for i in ET.parse(world_file).getroot().iter('include')}
                self.assertEqual(names - {None}, expected)
                self.assertEqual(json.loads(Path(config_file).read_text())['ground_robot'], bool(flag))
            finally:
                shutil.rmtree(directory, ignore_errors=True)

    def test_oil_scene_places_the_ground_robot_and_facility(self):
        scene = export_scene(ROOT, {'scenario': 'oil_facility', 'case': 'unaware'})
        names = [p['name'] for p in scene['scenery']]
        self.assertEqual(list(scene['entities']), ['robot'])
        self.assertTrue(any(n.startswith('tank_0/') for n in names))
        self.assertTrue(any(n.startswith('personnel_0/') for n in names))

    def test_supervisor_serves_the_cached_scene(self):
        session = Session(ROOT, runner=lambda *a, **k: None)
        session.state = {'scenario': 'forest', 'case': 'F', 'seed': 7}
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler(session))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            url = f'http://127.0.0.1:{server.server_address[1]}/api/session/scene'
            first = json.load(urlopen(url, timeout=10))
            self.assertEqual(first['schema_version'], 1)
            self.assertIs(session.scene(), session.scene())
        finally:
            server.shutdown()


if __name__ == '__main__':
    unittest.main()
