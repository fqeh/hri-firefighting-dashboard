import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hri_bringup.waypoint_mission import WaypointFollower,front_clearance,parse_waypoints


class WaypointMissionTests(unittest.TestCase):
    def test_progress_depends_on_observed_pose(self):
        follower=WaypointFollower([[1,0],[1,1]],relative=False)
        for _ in range(100):
            linear,angular,state=follower.step((0,0,0),5)
        self.assertGreater(linear,0)
        self.assertEqual(angular,0)
        self.assertEqual(follower.index,0)
        self.assertFalse(follower.completed)
        linear,angular,state=follower.step((1,0,0),5)
        self.assertEqual(follower.index,1)
        self.assertEqual(linear,0)
        self.assertGreater(angular,0)
        self.assertEqual(follower.step((1,1,math.pi/2),5),(0,0,'completed'))
        self.assertTrue(follower.completed)

    def test_relative_points_anchor_once_in_initial_robot_frame(self):
        follower=WaypointFollower([[1,0],[1,1]])
        follower.step((2,3,math.pi/2),5)
        self.assertAlmostEqual(follower.points[0][0],2)
        self.assertAlmostEqual(follower.points[0][1],4)
        self.assertAlmostEqual(follower.points[1][0],1)
        self.assertAlmostEqual(follower.points[1][1],4)
        follower.step((2,3.5,math.pi/2),5)
        self.assertAlmostEqual(follower.points[0][1],4)

    def test_lidar_stop_and_unknown_scan_prevent_motion(self):
        follower=WaypointFollower([[1,0]],relative=False)
        self.assertEqual(follower.step((0,0,0),.2),(0,0,'obstacle_stop'))
        self.assertEqual(follower.step((0,0,0),None),(0,0,'waiting_for_scan'))
        self.assertEqual(follower.step((0,0,0),math.nan),(0,0,'waiting_for_scan'))
        self.assertGreater(follower.step((0,0,0),5)[0],0)

    def test_scan_sector_keeps_forward_obstacles_and_handles_no_returns(self):
        self.assertEqual(front_clearance([.1,4,.3,4,.1],-1,.5,.12,10,.2),.3)
        self.assertEqual(front_clearance([math.inf],0,.1,.12,10),10)
        self.assertEqual(front_clearance([.01],0,.1,.12,10),0)
        self.assertIsNone(front_clearance([math.nan],0,.1,.12,10))
        self.assertIsNone(front_clearance([1],2,.1,.12,10))
        self.assertIsNone(front_clearance([1],0,0,.12,10))

    def test_invalid_waypoints_and_pose_rejected(self):
        for value in ('[]','[[true,0]]','[[1,2,3]]','[["1",0]]',[[math.inf,0]]):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):parse_waypoints(value)
        follower=WaypointFollower([[1,0]])
        self.assertEqual(follower.step((math.nan,0,0),5),(0,0,'invalid_odometry'))
        self.assertIsNone(follower.points)


if __name__=='__main__':unittest.main()
