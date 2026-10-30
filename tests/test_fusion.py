import unittest

import numpy as np

from fusion.models import Measurement
from fusion.tracker import Track, Tracker


def pos(t, xyz, sid="radar", s=5.0):
    return Measurement(t, sid, "position", np.array(xyz, float), np.eye(3) * s * s, np.zeros(3))


class TestFusion(unittest.TestCase):
    def test_track_confirms_and_follows(self):
        trk = Tracker()
        for t in range(10):
            trk.end_cycle(t, trk.process_scan(t, [pos(t, [100 + 10 * t, 0, 50])]))
        c = trk.confirmed()
        self.assertEqual(len(c), 1)
        self.assertAlmostEqual(c[0].x[3], 10, delta=2)

    def test_bearing_update_reduces_cross_range_error(self):
        tr = Track(np.array([1000., 50, 50, 0, 0, 0]), np.diag([400.] * 3 + [100.] * 3), 0)
        b = Measurement(0, "rf", "bearing", np.array([0.0]), np.array([[np.deg2rad(1) ** 2]]), np.zeros(3))
        tr.update(b)
        self.assertLess(abs(tr.x[1]), 50)

    def test_clutter_does_not_confirm(self):
        trk, rng = Tracker(), np.random.default_rng(0)
        for t in range(10):
            trk.end_cycle(t, trk.process_scan(t, [pos(t, rng.uniform(-2000, 2000, 3))]))
        self.assertEqual(trk.confirmed(), [])


if __name__ == "__main__":
    unittest.main()
