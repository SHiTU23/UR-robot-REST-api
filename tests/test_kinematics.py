"""IK round trip on the cell URDF: FK(q) -> IK must contain q and every solution must reproduce the pose.
Run in the image: python3 -m unittest discover -s /app/tests (needs xacro for the URDF)."""
import math
import os
import subprocess
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ur_api"))
import geom  # noqa: E402
import kinematics as K  # noqa: E402
from cell import Cell  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def cell_urdf():
    return subprocess.run(["xacro", os.path.join(ROOT, "ros/urdf/ur_cell.urdf.xacro"), "ur_type:=ur10e", "name:=ur10e",
                           "use_mock_hardware:=true", f"cell_config:={os.path.join(ROOT, 'config/cell.yaml')}"],
                          check=True, capture_output=True, text=True).stdout


class TestKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cell = Cell(os.path.join(ROOT, "config/cell.yaml"))
        cls.kin = K.Kinematics(cell_urdf(), cls.cell)

    def test_dh_base_is_base_link_turned_pi(self):
        # the DH base (UR controller "base") is base_link rotated by pi about z
        self.assertLess(np.abs(self.kin.T_bl_dh - geom.rot_z(math.pi)).max(), 1e-9)

    def test_tcp_offset(self):
        self.assertAlmostEqual(self.kin.T_tool0_tcp[2, 3], 0.320, places=9)

    def test_round_trip(self):
        rng = np.random.default_rng(1)
        worst_p = worst_r = 0.0
        found = 0
        for _ in range(200):
            q = rng.uniform(-math.pi, math.pi, 6)
            q[2] = rng.uniform(-2.8, 2.8)            # elbow limited to about +-pi
            if min(abs(math.sin(q[4])), abs(math.sin(q[2]))) < 0.05:
                continue                             # skip singular poses
            T = self.kin.fk(q)
            sols = self.kin.solutions(T, q + rng.normal(0, 0.05, 6), limit_margin=0.0)
            self.assertTrue(sols, f"no solution for {q}")
            for _, s in sols:
                p, r = K.pose_error(self.kin.fk(s), T)
                worst_p, worst_r = max(worst_p, p), max(worst_r, r)
            if any(np.max(np.abs(np.array(s) - q)) < 1e-6 for _, s in sols):
                found += 1
        self.assertLess(worst_p, 1e-5)
        self.assertLess(worst_r, 1e-4)
        self.assertGreater(found, 150)             # q itself (nearest wrap to a nearby seed) comes back
        print(f"\n  round trip: {found} poses, worst {worst_p * 1e6:.3f} um / {math.degrees(worst_r) * 3600:.4f} arcsec")

    def test_branches(self):
        q = [0.4, -1.3, 1.6, -1.8, -1.5, 0.7]
        sols = self.kin.solutions(self.kin.fk(q), q, limit_margin=0.0)
        self.assertGreaterEqual(len(sols), 8)       # 8 branches for a generic pose (+ wraps)
        self.assertLess(sols[0][0], 1e-6)           # cheapest = the seed's own configuration


if __name__ == "__main__":
    unittest.main()
