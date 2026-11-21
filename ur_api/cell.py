"""Everything cell-specific in one place: config/cell.yaml (+ environment overrides), joint and link names,
frames. The rest of the API asks this object instead of hard-coding UR or gripper names."""
import json
import math
import os

import numpy as np
import yaml

from gripper_select import selected_gripper

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Cell:
    def __init__(self, path=None):
        path = path or os.environ.get("UR_CELL_CONFIG", os.path.join(ROOT, "config", "cell.yaml"))
        with open(path) as f:
            self.cfg = yaml.safe_load(f)
        self.path = path
        r, t, g = self.cfg["robot"], self.cfg["tool"], self.cfg["gripper"]
        p = os.environ.get("UR_TF_PREFIX", "")
        self.prefix = p
        self.ur_type = r["ur_type"]
        self.robot_ip = os.environ.get("UR_ROBOT_IP") or r["ip"]    # compose passes it empty when unset
        self.arm_joints = [p + n for n in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
        self.n_joints = len(self.arm_joints)
        # the gripper model (config/grippers/<model>.yaml), chosen as in the URDF (ros/scripts/cell.sh)
        self.gripper_model, self.gripper_model_source = selected_gripper(g.get("model", "rg6"))
        with open(os.path.join(ROOT, "config", "grippers", self.gripper_model + ".yaml")) as f:
            gm = yaml.safe_load(f)
        self.gripper_spec = gm
        self.finger_joint = p + gm["joint"]       # the driving joint; the other finger joints mimic it
        self.finger_joints = [self.finger_joint]
        rows = sorted(gm["table"], key=lambda r: r[1])          # by width, for np.interp
        self._tw = [float(r[1]) for r in rows]
        self._tj = [float(r[0]) for r in rows]
        self._tt = [float(r[2]) for r in rows]
        self.group = "ur_manipulator"
        self.base_frame = p + "base_link"          # the API's frame
        self.planning_frame = "world"              # MoveIt's (the URDF root, identical to base_link); set at startup
        self.tcp_link = p + "tcp"
        self.flange_link = p + "tool0"
        self.hand_link = p + gm["hand_link"]
        self.hand_links = [p + n for n in gm["links"]] + [p + "tcp"]
        self.wrench_topic = "/force_torque_sensor_broadcaster/wrench"
        self.wrench_frame = p + "tool0"
        self.config_home_q = [float(v) for v in r["home_joints_rad"]]
        self.home_q = list(self.config_home_q)     # replaced by a saved home: load_saved_home, POST /robot/home
        self.home_saved_at = None
        # How the base is turned in the room (seen from above, + counter-clockwise): RViz's view, and frame=room
        self.calibration_file = r.get("kinematics_params_file")     # the arm's calibration (GET /robot/calibration)
        self.room_yaw_rad = math.radians(float(r.get("room_yaw_deg", 0.0)))
        self.limit_margin = float(r.get("joint_limit_margin_rad", 0.02))
        self.max_travel_rad = math.radians(float(r.get("max_joint_travel_deg", 100)))
        self.travel_exempt = tuple(p + n for n in r.get("travel_exempt_joints", ["wrist_3_joint"]))
        # TCP = closed fingertip centre: changer + the model's mount offset + closed tip depth (last table row)
        self.tcp_z = float(t["changer_length_m"]) + float(gm["mount_offset_m"]) + float(gm["table"][-1][2])
        self.tcp_xy = [float(v) for v in gm.get("tcp_xy_m", [0.0, 0.0])]   # calibrated offset from the flange axis
        self.model_tcp_z = self.tcp_z
        self.tcp_rpy = [0.0, 0.0, 0.0]
        self.tcp_source = f"config/grippers/{self.gripper_model}.yaml"
        self.tcp_file = (r.get("tcp_file") or "").replace("{model}", self.gripper_model) or None   # per gripper (ur_tcp.py)
        import ur_tcp
        self.tcp_mode = ur_tcp.mode_for(self.cfg, self.gripper_model)    # file | controller | follow
        # the model's limits; cell.yaml's gripper section (control settings) on top
        self.gripper = {k: gm[k] for k in ("max_width_m", "min_force_n", "max_force_n", "default_force_n")}
        # blocks named after a model (gripper.robotiq_2f85: {...}) override the shared lines for that model only
        self.gripper.update({k: v for k, v in g.items() if not isinstance(v, dict)})
        self.gripper.update(g.get(self.gripper_model) or {})
        self.gripper["model"] = self.gripper_model
        self.gripper["name"] = gm["name"]
        self.max_width = float(self.gripper["max_width_m"])
        self.open_threshold = float(g.get("open_threshold_m", 0.03))
        self.world_boxes = self.cfg.get("world_boxes") or []
        self.builtin_world = tuple(b["name"] for b in self.world_boxes)
        self.camera = dict(self.cfg.get("camera") or {})
        self.camera["backend"] = os.environ.get("UR_CAMERA") or self.camera.get("backend", "none")   # compose passes it empty when unset
        self.joint_limits = {}                     # name -> (lo, hi), from the URDF at startup
        # IK: base / shoulder / elbow moves are what bystanders see as "big"; the wrists are cheap and
        # wrist_3 (tool roll) cheapest, so it takes the yaw and the big joints stay put.
        self.ik_weights = (2.0, 1.5, 2.0, 1.0, 1.0, 0.15)
        self.ik_limit_margin = 0.05

    def use_urdf_tcp(self, T_tool0_tcp):
        """Take the TCP from the robot description actually loaded: cell.launch.py may have built it from the
        saved TCP (robot.tcp_file), which may have changed since (PUT /robot/tcp waits for a restart)."""
        import ur_tcp
        self.tcp_xy = [float(T_tool0_tcp[0, 3]), float(T_tool0_tcp[1, 3])]
        self.tcp_z = float(T_tool0_tcp[2, 3])
        R = T_tool0_tcp[:3, :3]
        self.tcp_rpy = [math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0])]
        saved = ur_tcp.load(self.tcp_file, self.gripper_model) if self.tcp_file else None
        if saved and all(abs(a - b) < 1e-6 for a, b in zip(saved["xyz_m"], self.tcp_xy + [self.tcp_z])):
            self.tcp_source = f"{self.tcp_file} ({ur_tcp.describe(saved)})"
        return saved

    def load_saved_home(self, path):
        """Use the home pose saved with POST /robot/home, if there is a valid one; otherwise the cell.yaml one."""
        try:
            with open(path) as f:
                d = json.load(f)
            q = [float(v) for v in d["joints_rad"]]
            if len(q) != self.n_joints or not all(math.isfinite(v) for v in q) or d.get("ur_type") != self.ur_type:
                raise ValueError("wrong joints or robot type")
        except FileNotFoundError:
            return
        except (OSError, ValueError, KeyError, TypeError) as e:
            import logging
            logging.getLogger("ur_rest_api").error("ignoring the saved home in %s: %s", path, e)
            return
        self.home_q, self.home_saved_at = q, d.get("saved_at")

    def finger_q(self, width):
        """{driving finger joint: value} for a gripper width (m), through the model's table."""
        return {self.finger_joint: float(np.interp(width, self._tw, self._tj))}

    def tip_depth(self, width):
        """Fingertip depth below the gripper mount face at this width (m); the TCP is at the closed depth."""
        return float(np.interp(width, self._tw, self._tt))

    def touch_links(self, link):
        """Links allowed to touch an object attached to ``link`` (the hand and fingers by default)."""
        return list(dict.fromkeys([link] + self.hand_links))

    def frames_ok(self):
        return (self.base_frame, self.planning_frame, "base")

    def limits(self):
        return [self.joint_limits.get(n, (-2 * math.pi, 2 * math.pi)) for n in self.arm_joints]

    def near_limits(self, q, margin=None):
        """Joints of q within ``margin`` of a limit (empty = all clear)."""
        margin = self.limit_margin if margin is None else margin
        bad = []
        for i, (v, (lo, hi)) in enumerate(zip(q, self.limits())):
            d = min(v - lo, hi - v)
            if d < margin:
                bad.append({"joint": i + 1, "name": self.arm_joints[i], "current_rad": round(v, 4),
                            "limit_lo_rad": lo, "limit_hi_rad": hi, "distance_to_nearest_limit_rad": round(d, 4),
                            "distance_to_nearest_limit_deg": round(math.degrees(d), 2)})
        return bad
