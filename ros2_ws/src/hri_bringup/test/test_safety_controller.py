import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hri_bringup.safety_controller import CommandGate,ZERO_COMMAND


class SafetyControllerTests(unittest.TestCase):
    def setUp(self):
        self.motion=(.35,0.,0.,0.,0.,.2)

    def test_stop_latches_against_continued_motion_commands(self):
        gate=CommandGate()
        self.assertTrue(gate.accept(self.motion,10))
        self.assertEqual(gate.output(10.1),self.motion)
        gate.set_stop(True)
        for stamp in (10.2,11.,30.):
            self.assertFalse(gate.accept(self.motion,stamp))
            self.assertEqual(gate.output(stamp),ZERO_COMMAND)
        self.assertTrue(gate.stopped)

    def test_release_requires_new_command_and_never_replays_old_input(self):
        gate=CommandGate()
        gate.accept(self.motion,1)
        gate.set_stop(True)
        gate.accept(self.motion,2)
        gate.set_stop(False)
        self.assertEqual(gate.output(2.1),ZERO_COMMAND)
        self.assertTrue(gate.accept(self.motion,2.2))
        self.assertEqual(gate.output(2.3),self.motion)

    def test_wall_watchdog_expires_without_simulation_time_advancing(self):
        gate=CommandGate(timeout=.5)
        gate.accept(self.motion,100)
        self.assertEqual(gate.output(100.49),self.motion)
        self.assertEqual(gate.output(100.5),ZERO_COMMAND)
        self.assertEqual(gate.reason,'command_watchdog')
        self.assertEqual(gate.output(120),ZERO_COMMAND)

    def test_invalid_command_stops_previous_motion_immediately(self):
        for bad in ((math.nan,0,0,0,0,0),(0,0,0,0,0,math.inf),(1,2),None):
            with self.subTest(bad=bad):
                gate=CommandGate()
                gate.accept(self.motion,1)
                self.assertFalse(gate.accept(bad,1.1))
                self.assertEqual(gate.output(1.2),ZERO_COMMAND)
                self.assertEqual(gate.reason,'invalid_command')

    def test_startup_and_invalid_timestamps_fail_closed(self):
        gate=CommandGate()
        self.assertEqual(gate.output(1),ZERO_COMMAND)
        self.assertFalse(gate.accept(self.motion,math.nan))
        gate.accept(self.motion,2)
        self.assertEqual(gate.output(1),ZERO_COMMAND)
        for timeout in (0,-1,math.inf,math.nan):
            with self.assertRaises(ValueError):CommandGate(timeout)


if __name__=='__main__':unittest.main()
