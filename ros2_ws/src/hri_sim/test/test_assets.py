"""Structural contracts only. These do not replace a Gazebo runtime test."""
from pathlib import Path
import json
import math
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

class AssetsTest(unittest.TestCase):
    def test_worlds_have_physics_and_live_robot(self):
        for name in ('minimal','forest','industrial'):
            world=ET.parse(ROOT/'worlds'/f'{name}.sdf').getroot().find('world')
            self.assertEqual(world.attrib['name'],name)
            self.assertTrue(any(p.attrib['name'].endswith('Physics') for p in world.findall('plugin')))
            self.assertIn('model://robot',[i.findtext('uri') for i in world.findall('include')])
            for node in world.findall('.//include/uri'):
                model=node.text.removeprefix('model://')
                self.assertTrue((ROOT/'models'/model/'model.sdf').exists(),model)
    def test_robot_sensors_and_collisions(self):
        robot=ET.parse(ROOT/'models/robot/model.sdf').getroot()
        self.assertGreaterEqual(len(robot.findall('.//collision')),4)
        self.assertEqual({'gpu_lidar','imu','camera','logical_camera'},{s.attrib['type'] for s in robot.findall('.//sensor')})
        self.assertEqual(robot.findtext('.//plugin/topic'),'/robot/cmd_vel')
        self.assertTrue(robot.find('.//link/inertial') is not None)
    def test_link_child_names_are_unique(self):
        # libsdformat requires uniqueness across visual, collision and sensor tags.
        for file in list((ROOT/'models').rglob('*.sdf'))+list((ROOT/'worlds').glob('*.sdf')):
            for link in ET.parse(file).getroot().findall('.//link'):
                names=[child.attrib['name'] for child in link if 'name' in child.attrib]
                self.assertEqual(len(names),len(set(names)),f'{file}: {link.attrib}')
    def test_wheel_and_world_pose_are_separate(self):
        robot=ET.parse(ROOT/'models/robot/model.sdf').getroot()
        plugins=robot.findall('.//plugin')
        self.assertEqual({p.findtext('odom_topic') for p in plugins},{'/robot/odom','/evaluation/robot_pose'})
        world=[p for p in plugins if p.attrib['name'].endswith('OdometryPublisher')][0]
        self.assertEqual(world.findtext('odom_frame'),'world')
        self.assertEqual(world.findtext('dimensions'),'3')
    def test_graph_matches_embedded_trails_and_hazards(self):
        for name in ('forest','industrial'):
            config=json.loads((ROOT/'config'/f'{name}.json').read_text())
            world=ET.parse(ROOT/'worlds'/f'{name}.sdf').getroot()
            models={m.attrib['name'] for m in world.findall('./world/model')}
            for edge in config['edges']:
                self.assertIn('route_'+edge['id'],models)
                a,b=config['nodes'][edge['source']],config['nodes'][edge['target']]
                self.assertAlmostEqual(edge['length'],math.dist(a,b))
            self.assertEqual(config['human_start'][:2],config['nodes'][config['start']])
            for hazard in config['hazards']:
                self.assertTrue((ROOT/'models'/hazard['model']/'model.sdf').exists())
                self.assertIn(hazard['edge'],{e['id'] for e in config['edges']})
    def test_assets_are_offline_and_terrain_is_3d(self):
        for file in (ROOT/'models').rglob('*.sdf'):
            ET.parse(file)
            self.assertNotIn('https://',file.read_text())
        mesh=(ROOT/'models/terrain/meshes/hills.obj').read_text().splitlines()
        heights={float(line.split()[3]) for line in mesh if line.startswith('v ')}
        self.assertGreater(max(heights)-min(heights),3)
    def test_terrain_has_vertex_normals_for_physics_import(self):
        mesh=(ROOT/'models/terrain/meshes/hills.obj').read_text().splitlines()
        vertices=[line for line in mesh if line.startswith('v ')]
        normals=[tuple(map(float,line.split()[1:])) for line in mesh if line.startswith('vn ')]
        self.assertEqual(len(vertices),len(normals))
        self.assertGreater(len(normals),0)
        for normal in normals:
            self.assertAlmostEqual(sum(value*value for value in normal),1,places=7)
        for line in mesh:
            if line.startswith('f '):
                for reference in line.split()[1:]:
                    vertex,normal=map(int,reference.split('//'))
                    self.assertTrue(1<=vertex<=len(vertices))
                    self.assertTrue(1<=normal<=len(normals))
    def test_only_solid_obstacles_have_collision(self):
        for name in ('hazard_tree','hazard_debris'):
            self.assertIsNotNone(ET.parse(ROOT/'models'/name/'model.sdf').getroot().find('.//collision'))
        for name in ('hazard_fire','hazard_gas'):
            self.assertIsNone(ET.parse(ROOT/'models'/name/'model.sdf').getroot().find('.//collision'))

if __name__=='__main__': unittest.main()
