"""Everything cell-specific in one place: config/cell.yaml (+ environment overrides), joint and link names,
frames. The rest of the API asks this object instead of hard-coding UR or RG6 names."""
import math
import os

import yaml

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
        self.robot_ip = os.environ.get("UR_ROBOT_IP", r["ip"])
        self.arm_joints = [p + n for n in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
        self.n_joints = len(self.arm_joints)
        self.finger_joint = p + "rg6_finger_joint"
        self.finger_joints = [p + "rg6_finger_joint", p + "rg6_right_finger_joint"]
        self.group = "ur_manipulator"
        self.base_frame = p + "base_link"          # the API's frame
        self.planning_frame = "world"              # MoveIt's (the URDF root, identical to base_link); set at startup
        self.tcp_link = p + "tcp"
        self.flange_link = p + "tool0"
        self.hand_link = p + "rg6_body"
        self.hand_links = [p + n for n in ("rg6_body", "rg6_left_finger", "rg6_right_finger", "tcp")]
        self.wrench_topic = "/force_torque_sensor_broadcaster/wrench"
        self.wrench_frame = p + "tool0"
        self.home_q = [float(v) for v in r["home_joints_rad"]]
        self.limit_margin = float(r.get("joint_limit_margin_rad", 0.02))
        self.max_travel_rad = math.radians(float(r.get("max_joint_travel_deg", 100)))
        self.travel_exempt = tuple(p + n for n in r.get("travel_exempt_joints", ["wrist_3_joint"]))
        self.tcp_z = float(t["tcp_z_m"])
        self.gripper = g
        self.max_width = float(g["max_width_m"])
        self.open_threshold = float(g.get("open_threshold_m", 0.03))
        self.world_boxes = self.cfg.get("world_boxes") or []
        self.builtin_world = tuple(b["name"] for b in self.world_boxes)
        self.camera = dict(self.cfg.get("camera") or {})
        self.camera["backend"] = os.environ.get("UR_CAMERA", self.camera.get("backend", "none"))
        self.joint_limits = {}                     # name -> (lo, hi), from the URDF at startup
        # IK: base / shoulder / elbow moves are what bystanders see as "big"; the wrists are cheap and
        # wrist_3 (tool roll) cheapest, so it takes the yaw and the big joints stay put.
        self.ik_weights = (2.0, 1.5, 2.0, 1.0, 1.0, 0.15)
        self.ik_limit_margin = 0.05

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
