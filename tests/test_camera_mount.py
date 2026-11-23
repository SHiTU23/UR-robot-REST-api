"""ur_api/camera_mount.py: the hand-eye camera pose the URDF is built with (2026-11-20)."""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ur_api"))
import camera_mount as cm  # noqa: E402

TOOL = {"changer_radius_m": 0.0375, "camera_box_center_m": [0.0, 0.0, -0.015], "bracket_size_m": [0.045, 0.110, 0.030]}
# pipeFit's accepted hand-eye of 2026-11-20 12:31 (T_flange_cam)
T = [[0.3799102717720475, -0.9248277789970852, 0.019020110341619385, 0.08425438045583084],
     [0.9249438106970133, 0.3800669216157871, 0.005299259079036568, -0.05137946096915787],
     [-0.012129816790727405, 0.015579290382347831, 0.9998050576266384, 0.03187600391834407],
     [0.0, 0.0, 0.0, 1.0]]


class TestCameraMount(unittest.TestCase):
    def test_rpy_round_trip(self):
        rpy = cm.R_to_rpy([r[:3] for r in T[:3]])
        R = cm.rpy_to_R(rpy)
        self.assertLess(max(abs(R[i][j] - T[i][j]) for i in range(3) for j in range(3)), 1e-9)

    def test_check(self):
        self.assertIsNone(cm.check_T(T))
        self.assertIn("metres", cm.check_T([[1, 0, 0, 84], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        self.assertIn("orthonormal", cm.check_T([[2, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        self.assertIn("reflection", cm.check_T([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        self.assertIn("4x4", cm.check_T([[1, 0, 0], [0, 1, 0]]))

    def test_bracket_reaches_from_changer_to_camera(self):
        rpy = cm.R_to_rpy([r[:3] for r in T[:3]])
        size, xyz, br = cm.bracket([T[i][3] for i in range(3)], rpy, TOOL)
        r = math.hypot(xyz[0], xyz[1])
        self.assertAlmostEqual(r - size[0] / 2, TOOL["changer_radius_m"], places=6)        # starts at the changer's surface
        self.assertAlmostEqual(math.atan2(xyz[1], xyz[0]), br[2], places=6)                 # along the radial line
        self.assertEqual(size[1:], [0.045, 0.030])

    def test_bracket_is_the_plate_arm(self):
        tool = dict(TOOL, camera_mount_plate={"arm_x_m": [-0.0295, 0.0295], "arm_y_m": [0.0300, 0.1073], "thickness_m": 0.0046})
        rpy = cm.R_to_rpy([r[:3] for r in T[:3]])
        size, xyz, br = cm.bracket([T[i][3] for i in range(3)], rpy, tool)
        self.assertAlmostEqual(math.degrees(br[2]), -112.3, delta=0.1)                       # the plate's turn on the flange
        # the screw holes' midpoint (plate (-0.0002, 0.10035)) lands next to the lens (84.3, -51.4 mm): 15.7 / 2.9 mm off
        c, s = math.cos(br[2]), math.sin(br[2])
        hx, hy = c * -0.0002 - s * 0.10035, s * -0.0002 + c * 0.10035
        self.assertLess(math.hypot(hx - T[0][3], hy - T[1][3]), 0.017)
        self.assertEqual([round(v, 6) for v in size], [0.059, 0.0773, 0.0046])

    def test_plate_lies_on_the_camera_back(self):
        """(2026-11-21) the plate is right on top of the camera body (it was on the flange face, ~60 mm above it)"""
        tool = dict(TOOL, camera_box_m=[0.092, 0.027, 0.032], camera_box_center_m=[0.0157, -0.0029, -0.0123],
                    camera_mount_plate={"arm_x_m": [-0.0295, 0.0295], "arm_y_m": [0.0300, 0.1073], "thickness_m": 0.0046})
        xyz, rpy = [-0.057499, -0.106006, 0.09273], [0.001115, 0.002597, -0.399642]      # the hand-eye of 2026-11-21
        size, bx, _ = cm.bracket(xyz, rpy, tool)
        R = cm.rpy_to_R(rpy)
        body_z = xyz[2] + sum(R[2][k] * tool["camera_box_center_m"][k] for k in range(3))
        back = body_z - R[2][2] * 0.032 / 2
        self.assertAlmostEqual(bx[2] + size[2] / 2, back, delta=1e-6)       # the plate's underside on the camera's back face
        self.assertAlmostEqual(back, 0.0644, delta=0.0005)

    def test_plate_boxes_from_the_stl(self):
        """(2026-11-22) camera_mount_plate.boxes_m (the mount's STL): the bracket is the plate frame on the flange axis, its underside on
        the camera's back, and the boxes sit in it; mirror_x moves the wing to the other side"""
        plate = {"thickness_m": 0.0116, "boxes_m": [{"name": "arm", "x": [-0.02955, 0.02955], "y": [0.0203, 0.1073]},
                                                     {"name": "wing", "x": [0.02955, 0.06695], "y": [0.0917, 0.1073]}]}
        tool = dict(TOOL, camera_box_m=[0.092, 0.027, 0.032], camera_box_center_m=[0.0157, -0.0029, -0.0123], camera_mount_plate=plate)
        xyz, rpy = [-0.057499, -0.106006, 0.09273], [0.001115, 0.002597, -0.399642]      # the hand-eye of 2026-11-21
        size, bx, br = cm.bracket(xyz, rpy, tool)
        self.assertEqual(bx[:2], [0.0, 0.0])                                             # on the flange axis
        self.assertAlmostEqual(bx[2] + 0.0116 / 2, 0.0644, delta=0.0005)                 # underside on the camera's back face
        self.assertAlmostEqual(math.degrees(br[2]), 157.1, delta=0.1)
        self.assertEqual([round(v, 4) for v in size], [0.0965, 0.087, 0.0116])
        boxes = cm.plate_boxes(tool)
        self.assertEqual([[round(v, 5) for v in c] for c, _ in boxes], [[0.0, 0.0638, 0.0], [0.04825, 0.0995, 0.0]])
        mirrored = cm.plate_boxes(dict(tool, camera_mount_plate=dict(plate, mirror_x=True)))
        self.assertAlmostEqual(mirrored[1][0][0], -0.04825, places=6)
        self.assertEqual(len(cm.as_arg({"xyz_m": xyz, "rpy_rad": rpy}, tool).split()), 15)

    def test_gripper_sets(self):
        """(2026-11-22) each gripper set has its own camera holder (gripper yaml camera_holder) and its own camera mount file; a set
        never calibrated on its own falls back to the shared file"""
        old = {"arm_x_m": [-0.0295, 0.0295], "arm_y_m": [0.0300, 0.1073], "thickness_m": 0.0046}
        new = {"thickness_m": 0.0116, "boxes_m": [{"name": "arm", "x": [-0.02955, 0.02955], "y": [0.0203, 0.1073]}]}
        tool = dict(TOOL, camera_mount_plate=old)
        self.assertIs(cm.tool_for(tool, {}), tool)                                      # no camera_holder: cell.yaml's
        self.assertEqual(cm.tool_for(tool, {"camera_holder": new})["camera_mount_plate"], new)
        with tempfile.TemporaryDirectory() as d:
            cfg = {"tool": {"camera_mount_file": os.path.join(d, "camera_mount.yaml"),
                            "camera_mount_file_per_gripper": os.path.join(d, "camera_mount_{model}.yaml")}}
            self.assertEqual(cm.load_for(cfg, "rg6"), (None, None, False))
            rpy = cm.R_to_rpy([r[:3] for r in T[:3]])
            cm.save(cm.path_for(cfg), [T[i][3] for i in range(3)], rpy, note="shared")
            doc, path, own = cm.load_for(cfg, "robotiq_2f85")
            self.assertEqual((path, own), (cfg["tool"]["camera_mount_file"], False))   # not calibrated on its own: the shared one
            cm.save(cm.path_for(cfg, "robotiq_2f85"), [0.0, -0.1, 0.09], [0.0, 0.0, 0.0], note="robotiq set")
            doc, path, own = cm.load_for(cfg, "robotiq_2f85")
            self.assertEqual((path.endswith("camera_mount_robotiq_2f85.yaml"), own, doc["xyz_m"]), (True, True, [0.0, -0.1, 0.09]))
            self.assertFalse(cm.load_for(cfg, "rg6")[2])                                 # the other set is untouched

    def test_save_load_arg(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "calibration", "camera_mount.yaml")
            rpy = cm.R_to_rpy([r[:3] for r in T[:3]])
            cm.save(p, [T[i][3] for i in range(3)], rpy, note="test")
            doc = cm.load(p)
            self.assertEqual(len(cm.as_arg(doc, TOOL).split()), 15)
            self.assertTrue(cm.same(doc["xyz_m"], doc["rpy_rad"], [T[i][3] for i in range(3)], rpy))
            self.assertFalse(cm.same(doc["xyz_m"], doc["rpy_rad"], [-0.067, -0.117, 0.107], [-0.034, 0.015, -0.508]))
        self.assertIsNone(cm.load(os.path.join(d, "missing.yaml")))


if __name__ == "__main__":
    unittest.main()
