"""The decision that keeps the arm from carrying nothing to the placement position."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ur_api"))
from gripper import check_width_feedback, grasp_verdict  # noqa: E402

EMPTY = 0.003


class TestGraspVerdict(unittest.TestCase):
    def verdict(self, width, target=0.0, given=False, eps_in=0.005, eps_out=0.005):
        return grasp_verdict(width, target, eps_in, eps_out, EMPTY, given)[:2]

    def test_contact_mode(self):
        self.assertEqual(self.verdict(0.040), (True, "success"))
        self.assertEqual(self.verdict(0.0), (False, "object_missed"))
        self.assertEqual(self.verdict(EMPTY), (False, "object_missed"))

    def test_expected_width(self):
        self.assertEqual(self.verdict(0.040, 0.040, True), (True, "success"))
        self.assertEqual(self.verdict(0.0, 0.040, True), (False, "object_missed"))
        self.assertEqual(self.verdict(0.050, 0.040, True), (False, "action_failed"))

    def test_width_zero_with_tolerance(self):
        # what the website sends: width=0, eps_in=0.005, eps_out=0.035
        self.assertEqual(self.verdict(0.0008, 0.0, True, eps_out=0.035), (False, "object_missed"))
        self.assertEqual(self.verdict(0.020, 0.0, True, eps_out=0.035), (True, "success"))
        self.assertEqual(self.verdict(0.050, 0.0, True, eps_out=0.035), (False, "action_failed"))

    def test_thin_object_given_explicitly(self):
        self.assertEqual(self.verdict(0.002, 0.002, True, eps_in=0.001, eps_out=0.001), (True, "success"))


class TestWidthFeedback(unittest.TestCase):
    def test_negative_reading_is_rejected(self):
        # 2026-11-11: the RG6 program reported -15.8 mm after a grasp
        width, warning = check_width_feedback(-0.0158)
        self.assertIsNone(width)
        self.assertIn("-15.8 mm", warning)

    def test_plausible_readings_pass(self):
        self.assertEqual(check_width_feedback(0.035), (0.035, None))
        self.assertEqual(check_width_feedback(0.0), (0.0, None))
        self.assertEqual(check_width_feedback(-0.0003), (0.0, None))   # noise around 0
        self.assertEqual(check_width_feedback(None), (None, None))

if __name__ == "__main__":
    unittest.main()
