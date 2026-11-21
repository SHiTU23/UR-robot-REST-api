"""camera.reset_exposure (2026-11-20): back to cell.yaml's exposure settings, or to auto when it sets none."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ur_api"))
import camera  # noqa: E402


class Stub(camera.NoCamera):
    def __init__(self):
        super().__init__(640, 480, 15)
        self.calls = []

    def set_exposure(self, auto=None, exposure=None, gain=None):
        self.calls.append((auto, exposure, gain))
        return {"auto": auto, "exposure": exposure, "gain": gain}


class TestExposureReset(unittest.TestCase):
    def test_default_is_auto(self):
        c = Stub()
        c.reset_exposure()
        self.assertEqual(c.calls, [(True, None, None)])

    def test_config_values(self):
        for cfg, want in (({"auto_exposure": False, "exposure": 200, "gain": 20}, (False, 200, 20)),
                          ({"exposure": 150, "gain": 10}, (False, 150, 10)),
                          ({"auto_exposure": True}, (True, None, None))):
            cam = camera.make_camera({"backend": "none", **cfg})
            s = Stub()
            s.exposure_default = cam.exposure_default
            s.reset_exposure()
            self.assertEqual(s.calls, [want], cfg)

    def test_make_camera_without_settings(self):
        self.assertEqual(camera.make_camera({"backend": "none"}).exposure_default, {"auto": True})


if __name__ == "__main__":
    unittest.main()
