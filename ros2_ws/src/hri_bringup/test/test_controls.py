import unittest
from hri_bringup.controls import waypoint_coordinates


class WaypointValidationTest(unittest.TestCase):
    def test_nonobjects_are_rejected_without_crashing_the_adapter(self):
        for value in (None, [], True, 'waypoint', 2):
            with self.subTest(value=value), self.assertRaises(ValueError):
                waypoint_coordinates(value)

    def test_other_controls_are_left_to_their_owners(self):
        self.assertIsNone(waypoint_coordinates({'command': 'pause'}))

    def test_nonphysical_coordinates_are_rejected(self):
        for value in (True, '2', None, float('nan'), float('inf'), 101, -101):
            with self.subTest(value=value), self.assertRaises(ValueError):
                waypoint_coordinates({'command': 'waypoint', 'x': value, 'y': 0})

    def test_valid_goal_preserves_coordinates(self):
        self.assertEqual(waypoint_coordinates({'command': 'waypoint', 'x': 2, 'y': -1.5}), (2., -1.5))
