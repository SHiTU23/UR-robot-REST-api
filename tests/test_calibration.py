"""The calibration file must describe the same arm as the controller's DH values."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ur_api"))
import ur_calibration as cal  # noqa: E402

# a real UR10e's values: joint 2 and 3 carry the large opposing d values the correction has to remove
DH = {"theta": [4.51701e-07, 0.45219, -0.56044, 0.10785, -1.0e-05, 0.0],
      "a": [0.000100203, -0.550767, -0.564035, 2.37433e-05, 9.97447e-06, 0.0],
      "d": [0.18084, 383.60742664, -363.74715524, -19.68575532, 0.11984796, 0.11556566],
      "alpha": [1.57029, 0.000697423, 0.00471, 1.57128, -1.57087, 0.0], "checksum": [0] * 6, "calibration_status": 1}


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    r = np.eye(4)
    i, j = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}[axis]
    r[i, i], r[i, j], r[j, i], r[j, j] = c, -s, s, c
    return r


def trans(x, y, z):
    t = np.eye(4)
    t[:3, 3] = x, y, z
    return t


def fk_dh(k, q):
    T = np.eye(4)
    for i in range(6):
        T = T @ rot("z", k["theta"][i] + q[i]) @ trans(0, 0, k["d"][i]) @ trans(k["a"][i], 0, 0) @ rot("x", k["alpha"][i])
    return T


def fk_file(kin, q):
    """As the driver's URDF builds it: each joint at x y z + rpy, turning about its own z."""
    T = np.eye(4)
    for i, name in enumerate(cal.LINKS):
        p = kin[name]
        T = T @ trans(p["x"], p["y"], p["z"]) @ rot("z", p["yaw"]) @ rot("y", p["pitch"]) @ rot("x", p["roll"]) @ rot("z", q[i])
    return T


class TestCalibration(unittest.TestCase):
    def test_same_arm(self):
        kin = cal.to_kinematics(DH)
        rng = np.random.default_rng(3)
        for _ in range(200):
            q = rng.uniform(-math.pi, math.pi, 6)
            A, B = fk_dh(DH, q), fk_file(kin, q)
            self.assertLess(np.linalg.norm(A[:3, 3] - B[:3, 3]), 2e-3)      # the sample DH values are rounded
            self.assertLess(np.abs(A[:3, :3] - B[:3, :3]).max(), 2e-3)

    def test_large_d_removed(self):
        kin = cal.to_kinematics(DH)
        self.assertEqual(kin["upper_arm"]["z"], 0.0)
        self.assertEqual(kin["forearm"]["z"], 0.0)
        self.assertLess(abs(kin["wrist_1"]["z"]), 1.0)

    def test_hash(self):
        # values from g++'s std::hash<std::string>
        self.assertEqual(cal.std_hash(b""), 6142509188972423790)
        self.assertEqual(cal.std_hash(b"x"), 13272411544345499535)
        self.assertEqual(cal.std_hash(b"12345678"), 10113836418918046885)
        self.assertEqual(cal.std_hash(b"123456789"), 12315982496267331008)


if __name__ == "__main__":
    unittest.main()
