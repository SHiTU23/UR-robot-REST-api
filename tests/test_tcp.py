"""The TCP read from the controller: rotation vector -> URDF rpy, the saved file, and the URDF built from it."""
import math
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ur_api"))
import ur_tcp  # noqa: E402
from kinematics import Urdf  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def rotvec_matrix(rv):
    th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.eye(3)
    k = np.array(rv) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class TestTcp(unittest.TestCase):
    def test_rotvec_to_rpy(self):
        for rv in ([0, 0, 0], [0.1, 0, 0], [0, 0, -0.508], [0.3, -0.2, 1.1], [math.pi, 0, 0], [0.01, 2.5, -0.4]):
            self.assertLess(np.abs(rpy_matrix(*ur_tcp.rotvec_to_rpy(rv)) - rotvec_matrix(rv)).max(), 1e-9, rv)

    def test_rpy_to_rotvec_round_trip(self):
        for rv in ([0, 0, 0], [0.1, 0, 0], [0, 0, -0.508], [0.3, -0.2, 1.1], [0, 0, -math.pi], [math.pi, 0, 0],
                   [0.01, 2.5, -0.4], [0, 0, 3.14159]):
            back = ur_tcp.rpy_to_rotvec(ur_tcp.rotvec_to_rpy(rv))
            self.assertLess(np.abs(rotvec_matrix(back) - rotvec_matrix(rv)).max(), 1e-9, rv)

    def test_plausibility(self):
        self.assertIsNone(ur_tcp.check([0, 0, 0.322]))
        self.assertIn("millimetres", ur_tcp.check([0, 0, 322]))
        self.assertIsNotNone(ur_tcp.check([0.3, 0, 0.322]))

    def test_saved_by_the_api(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "tcp.yaml")
            ur_tcp.save(path, [0, 0, 0.322, 0, 0, 0], "rg6", None, "api", "fix")
            got = ur_tcp.load(path, "rg6")
            self.assertEqual(got["source"]["set_by"], "api")
            self.assertTrue(ur_tcp.describe(got).startswith("set through the API"))
            self.assertEqual(ur_tcp.as_arg(got), "0.0 0.0 0.322 0.0 0.0 0.0")

    def test_saved_file_only_for_its_gripper(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "tcp.yaml")
            ur_tcp.save(path, [0.001, -0.002, 0.33, 0.0, 0.0, 0.1], "rg6", "test")
            self.assertEqual(ur_tcp.load(path, "rg6")["xyz_m"], [0.001, -0.002, 0.33])
            self.assertIsNone(ur_tcp.load(path, "robotiq_2f85"))
            self.assertIsNone(ur_tcp.load(os.path.join(d, "missing.yaml"), "rg6"))

    def test_urdf_uses_the_controller_tcp(self):
        tcp = [0.001, -0.002, 0.33] + ur_tcp.rotvec_to_rpy([0.0, 0.0, 0.1])
        env = dict(os.environ, UR_TCP=" ".join(repr(v) for v in tcp))
        xml = subprocess.run(["xacro", os.path.join(ROOT, "ros/urdf/ur_cell.urdf.xacro"), "ur_type:=ur10e", "name:=ur10e",
                              "use_mock_hardware:=true", f"cell_config:={os.path.join(ROOT, 'config/cell.yaml')}"],
                             check=True, capture_output=True, text=True, env=env).stdout
        T = Urdf(xml).fk("tcp", {}, "tool0")
        self.assertLess(np.abs(T[:3, 3] - [0.001, -0.002, 0.33]).max(), 1e-9)
        self.assertLess(np.abs(T[:3, :3] - rotvec_matrix([0.0, 0.0, 0.1])).max(), 1e-9)


class TestTcpRoutes(unittest.TestCase):
    """The /robot/tcp endpoints against a stub cell (no ROS, no robot)."""

    def setUp(self):
        import types
        from flask import Flask
        import ctx
        import errors
        import routes_tcp
        self.dir = tempfile.TemporaryDirectory()
        cell = types.SimpleNamespace(tcp_xy=[0.0, 0.0], tcp_z=0.23435, tcp_rpy=[0.0, 0.0, -3.14159], tcp_source="x",
                                     tcp_file=os.path.join(self.dir.name, "tcp.yaml"), gripper_model="rg6",
                                     model_tcp_z=0.322, robot_ip="192.0.2.1", cfg={"robot": {}})
        self._old = {k: getattr(ctx.C, k, None) for k in ("cell", "mock")}
        ctx.C.cell, ctx.C.mock = cell, True
        app = Flask("t")
        app.register_blueprint(routes_tcp.bp)
        app.register_error_handler(errors.ApiError, lambda e: (jsonify_err(e), e.status))
        self.c = app.test_client()

    def tearDown(self):
        import ctx
        for k, v in self._old.items():
            setattr(ctx.C, k, v)
        self.dir.cleanup()

    def test_put_then_restart_required(self):
        r = self.c.put("/robot/tcp", json={"xyz_m": [0, 0, 0.322], "note": "pendant hand_eye_TCP"})
        self.assertEqual(r.status_code, 200, r.get_json())
        j = r.get_json()
        self.assertEqual(j["saved"]["xyz_m"], [0.0, 0.0, 0.322])
        self.assertTrue(j["restart_required"])
        self.assertEqual(j["warnings"], [])
        self.assertTrue(self.c.get("/robot/tcp").get_json()["restart_required"])

    def test_put_refuses_millimetres(self):
        r = self.c.put("/robot/tcp", json={"xyz_m": [0, 0, 322]})
        self.assertEqual(r.status_code, 422)
        self.assertFalse(os.path.exists(os.path.join(self.dir.name, "tcp.yaml")))

    def test_rotation_round_trips(self):
        j = self.c.put("/robot/tcp", json={"xyz_m": [0, 0, 0.23435], "rotvec_rad": [0, 0, -3.14159]}).get_json()
        self.assertFalse(j["restart_required"], j)      # the same as the stub's in-use TCP
        self.assertEqual(len(j["warnings"]), 2)          # short of the model, and rotated


def jsonify_err(e):
    from flask import jsonify
    return jsonify({"error": getattr(e, "code", "error")})


if __name__ == "__main__":
    unittest.main()
