"""velocity= and acceleration= of the motion endpoints (routes_robot.parse_speed), without ROS."""
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ur_api"))
try:
    import routes_robot as RR  # noqa: E402
except Exception as e:  # noqa: BLE001  (needs the container's ROS python)
    RR = None
    WHY = f"{type(e).__name__}: {e}"


def args(**kw):
    d = {k: str(v) for k, v in kw.items()}
    return lambda k, default=None: d.get(k, default)


@unittest.skipIf(RR is None, "routes_robot not importable here (run inside the ur-api container)")
class TestParseSpeed(unittest.TestCase):
    def test_defaults(self):
        self.assertEqual(RR.parse_speed("0.2", "0.1", args()), (0.2, 0.1))
        self.assertEqual(RR.parse_speed("0.3", "0.05", args()), (0.3, 0.05))

    def test_acceleration_follows_velocity_without_default(self):
        self.assertEqual(RR.parse_speed("0.05", None, args()), (0.05, 0.05))
        self.assertEqual(RR.parse_speed("0.05", None, args(velocity=0.2)), (0.2, 0.2))
        self.assertEqual(RR.parse_speed("0.05", None, args(velocity=0.2, acceleration=0.4)), (0.2, 0.4))

    def test_given(self):
        self.assertEqual(RR.parse_speed("0.2", "0.1", args(velocity=0.5, acceleration=0.25)), (0.5, 0.25))

    def test_range(self):
        for bad in (dict(velocity=0.6), dict(velocity=0.005), dict(acceleration=0.0), dict(acceleration=0.51),
                    dict(acceleration="nan"), dict(velocity="x")):
            with self.assertRaises(ValueError, msg=str(bad)):
                RR.parse_speed("0.2", "0.1", args(**bad))


if __name__ == "__main__":
    unittest.main()
