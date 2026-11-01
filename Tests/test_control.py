import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'Robot/Scripts'))
from control import valid_name, relative, command, yaw

class NavigationTests(unittest.TestCase):
    def test_relative_progress_uses_initial_heading(self):
        forward, lateral, heading = relative((3.,4.,math.pi/2), (3.,5.,math.pi/2))
        self.assertAlmostEqual(forward, 1.)
        self.assertAlmostEqual(lateral, 0.)

    def test_sideways_distance_is_not_goal_progress(self):
        self.assertEqual(relative((0,0,0),(0,.1,0))[:2], (0.,.1))

    def test_stop_at_goal(self):
        self.assertEqual(command(1.,.1,1.,0.,0.,.1,.05), (0.,0.,True))

    def test_acceleration_bound(self):
        speed, turn, done = command(2.,.15,0.,0.,0.,0.,.05)
        self.assertAlmostEqual(speed, .0075)

    def test_maximum_speed_still_brakes(self):
        speed,_,_ = command(1.,.26,.90,0.,0.,.26,.05)
        self.assertLess(speed, .26)
        speed,_,_ = command(10.,.26,0.,0.,0.,.26,.05)
        self.assertAlmostEqual(speed, .26)

    def test_deceleration_near_goal(self):
        speed,_,_ = command(1.,.15,.98,0.,0.,.15,.05)
        self.assertLess(speed, .1)

    def test_abort_lateral_drift(self):
        with self.assertRaises(RuntimeError):command(1.,.1,.1,.2,0.,.1,.05)

    def test_abort_heading_deviation(self):
        with self.assertRaises(RuntimeError):command(1.,.1,.1,0.,math.pi/2,.1,.05)

    def test_reject_nonfinite(self):
        with self.assertRaises(ValueError):command(1.,float('nan'),0.,0.,0.,0.,.05)

    def test_reject_bad_quaternion(self):
        with self.assertRaises(ValueError):yaw(SimpleNamespace(x=0,y=0,z=0,w=0))

    def test_name_cannot_escape_bag_directory(self):
        for name in ['../other','a/b','x;touch hello','', 'a'*81]:
            with self.assertRaises(ValueError):valid_name(name)
        self.assertEqual(valid_name('run_01-A'), 'run_01-A')

if __name__ == '__main__':unittest.main()
