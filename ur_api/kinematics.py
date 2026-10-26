"""Forward and inverse kinematics of the cell robot, in Python, from the URDF that robot_state_publisher
publishes (so FK uses the same -- possibly calibrated -- kinematics as MoveIt and the driver).

IK = all configurations, not one numerical answer: the closed-form UR solution (Hawkins, "Analytic Inverse
Kinematics for the Universal Robots UR-5/UR-10 Arms", 2013) on the nominal DH parameters gives up to 8
branches; each is moved by multiples of 2 pi to the wrap nearest the seed, refined by damped least squares
against the URDF FK (exact also for calibrated kinematics), checked against the joint limits, and ranked by
the weighted largest joint change from the seed. The motion layer then picks the first collision-free one,
so a pose goal never makes the base swing round or the elbow flip when a nearby configuration exists.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

import geom

# Nominal DH parameters (UR's published values; ur_description default_kinematics.yaml matches them).
UR_DH = {
    "ur3e": (0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921),
    "ur5e": (0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996),
    "ur10e": (0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655),
    "ur16e": (0.1807, -0.4784, -0.36, 0.17415, 0.11985, 0.11655),
    "ur3": (0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819),
    "ur5": (0.089159, -0.425, -0.39225, 0.10915, 0.09465, 0.0823),
    "ur10": (0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922),
}
ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)


def _rpy(r, p, y):
    T = np.eye(4)
    T[:3, :3] = geom.euler_to_matrix(r, p, y)
    return T


def _axis_angle(axis, a):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    return T


class Urdf:
    """Joint tree of a URDF string: FK between any two links."""

    def __init__(self, xml):
        root = ET.fromstring(xml)
        self.name = root.get("name")
        self.links = [l.get("name") for l in root.findall("link")]
        self.joints = {}
        self.parent_joint = {}           # child link -> joint name
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            lim = j.find("limit")
            mim = j.find("mimic")
            d = {"name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                 "child": j.find("child").get("link"),
                 "origin": geom.trans(*xyz) @ _rpy(*rpy),
                 "axis": [float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()],
                 "limit": (float(lim.get("lower", "-inf")), float(lim.get("upper", "inf"))) if lim is not None else None,
                 "mimic": (mim.get("joint"), float(mim.get("multiplier", "1")), float(mim.get("offset", "0")))
                 if mim is not None else None}
            self.joints[d["name"]] = d
            self.parent_joint[d["child"]] = d["name"]

    def chain(self, link, base):
        """Joints from ``base`` down to ``link`` (outermost last)."""
        out = []
        while link != base:
            if link not in self.parent_joint:
                raise KeyError(f"{link!r} is not below {base!r}")
            j = self.joints[self.parent_joint[link]]
            out.append(j)
            link = j["parent"]
        return out[::-1]

    def _value(self, j, q):
        if j["mimic"]:
            src, m, off = j["mimic"]
            return q.get(src, 0.0) * m + off
        return q.get(j["name"], 0.0)

    def fk(self, link, q, base):
        """4x4 pose of ``link`` in ``base`` for joint values ``q`` (dict; missing = 0)."""
        T = np.eye(4)
        for j in self.chain(link, base):
            T = T @ j["origin"]
            if j["type"] in ("revolute", "continuous"):
                T = T @ _axis_angle(j["axis"], self._value(j, q))
            elif j["type"] == "prismatic":
                T = T @ geom.trans(*(np.asarray(j["axis"], float) * self._value(j, q)))
        return T

    def limits(self, names):
        return {n: self.joints[n]["limit"] for n in names if n in self.joints and self.joints[n]["limit"]}


def _dh(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])


def dh_fk(q, dh):
    d1, a2, a3, d4, d5, d6 = dh
    T = np.eye(4)
    for th, d, a, al in zip(q, (d1, 0, 0, d4, d5, d6), (0, a2, a3, 0, 0, 0), ALPHA):
        T = T @ _dh(th, d, a, al)
    return T


def dh_ik(T06, dh):
    """All (up to 8) closed-form solutions for the DH flange pose T06 (in the DH base). Angles in (-pi, pi]."""
    d1, a2, a3, d4, d5, d6 = dh
    sols = []
    p05 = T06 @ np.array([0, 0, -d6, 1.0])
    r = math.hypot(p05[0], p05[1])
    if r < abs(d4):
        return sols
    psi, phi = math.atan2(p05[1], p05[0]), math.acos(d4 / r)
    T60 = np.linalg.inv(T06)
    for t1 in (psi + phi + math.pi / 2, psi - phi + math.pi / 2):
        s1, c1 = math.sin(t1), math.cos(t1)
        c5 = (T06[0, 3] * s1 - T06[1, 3] * c1 - d4) / d6
        if abs(c5) > 1 + 1e-9:
            continue
        c5 = max(-1.0, min(1.0, c5))
        for t5 in (math.acos(c5), -math.acos(c5)):
            s5 = math.sin(t5)
            if abs(s5) < 1e-9:
                t6 = 0.0                                 # wrist singularity: q6 is free
            else:
                t6 = math.atan2((-T60[1, 0] * s1 + T60[1, 1] * c1) / s5, (T60[0, 0] * s1 - T60[0, 1] * c1) / s5)
            T01 = _dh(t1, d1, 0, ALPHA[0])
            T45 = _dh(t5, d5, 0, ALPHA[4])
            T56 = _dh(t6, d6, 0, ALPHA[5])
            T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T45 @ T56)
            px, pz = T14[0, 3], T14[2, 3]
            pxz = math.hypot(px, pz)
            c3 = (pxz ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
            if abs(c3) > 1 + 1e-9:
                continue
            c3 = max(-1.0, min(1.0, c3))
            for t3 in (math.acos(c3), -math.acos(c3)):
                t2 = math.atan2(-pz, -px) - math.asin(max(-1.0, min(1.0, -a3 * math.sin(t3) / pxz)))
                T12 = _dh(t2, 0, a2, ALPHA[1])
                T23 = _dh(t3, 0, a3, ALPHA[2])
                T34 = np.linalg.inv(T23) @ np.linalg.inv(T12) @ T14
                t4 = math.atan2(T34[1, 0], T34[0, 0])
                sols.append([math.atan2(math.sin(v), math.cos(v)) for v in (t1, t2, t3, t4, t5, t6)])
    return sols


def pose_error(Ta, Tb):
    """(position error m, rotation error rad) between two 4x4 poses."""
    return float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])), geom.angle_between(Ta, Tb)


class Kinematics:
    """FK / IK of the arm group (base_link -> tcp) for one cell."""

    def __init__(self, urdf_xml, cell):
        self.cell = cell
        self.urdf = Urdf(urdf_xml)
        self.dh = UR_DH.get(cell.ur_type)
        self.T_tool0_tcp = self.urdf.fk(cell.tcp_link, {}, cell.flange_link)
        lim = self.urdf.limits(cell.arm_joints)
        self.lo = np.array([lim.get(n, (-2 * math.pi, 2 * math.pi))[0] for n in cell.arm_joints])
        self.hi = np.array([lim.get(n, (-2 * math.pi, 2 * math.pi))[1] for n in cell.arm_joints])
        # The UR DH base frame is base_link turned by pi about z (= the controller's "base" frame); the DH
        # flange is tool0. Checked against the URDF at a test pose and kept, so a different description
        # (or calibration) only costs a few refinement steps.
        q0 = [0.3, -1.2, 1.4, -1.5, -1.6, 0.4]
        self.T_bl_dh = (self.fk(q0, cell.flange_link) @ np.linalg.inv(dh_fk(q0, self.dh))) if self.dh else None

    def qdict(self, q, extra=None):
        d = dict(zip(self.cell.arm_joints, (float(v) for v in q)))
        if extra:
            d.update(extra)
        return d

    def fk(self, q, link=None, base=None, extra=None):
        return self.urdf.fk(link or self.cell.tcp_link, self.qdict(q, extra), base or self.cell.base_frame)

    def _jacobian(self, q, link):
        T0 = self.fk(q, link)
        J = np.zeros((6, 6))
        h = 1e-6
        for i in range(6):
            dq = np.array(q, float)
            dq[i] += h
            T = self.fk(dq, link)
            J[:3, i] = (T[:3, 3] - T0[:3, 3]) / h
            R = T0[:3, :3].T @ T[:3, :3]
            J[3:, i] = T0[:3, :3] @ np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * h)
        return T0, J

    def refine(self, T_goal, q, link=None, iters=30, tol_m=1e-7, tol_rad=1e-7):
        """Damped least squares from q to the pose T_goal of ``link``. -> (q, pos err m, rot err rad)."""
        link = link or self.cell.tcp_link
        q = np.array(q, float)
        for _ in range(iters):
            T, J = self._jacobian(q, link)
            e_p = T_goal[:3, 3] - T[:3, 3]
            R = T[:3, :3].T @ T_goal[:3, :3]
            ang = geom.angle_between(T, T_goal)
            if ang < 1e-12:
                w = np.zeros(3)
            else:
                w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
                w = T[:3, :3] @ (w / (2 * math.sin(ang)) * ang) if math.sin(ang) > 1e-9 else T[:3, :3] @ (w / 2)
            if np.linalg.norm(e_p) < tol_m and ang < tol_rad:
                break
            e = np.concatenate([e_p, w])
            lam = 1e-6
            q = q + J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
        p, r = pose_error(self.fk(q, link), T_goal)
        return q, p, r

    def _nearest_wrap(self, q, seed):
        out = []
        for v, s, lo, hi in zip(q, seed, self.lo, self.hi):
            best = None
            for k in range(-3, 4):
                c = v + 2 * math.pi * k
                if lo <= c <= hi and (best is None or abs(c - s) < abs(best - s)):
                    best = c
            if best is None:
                return None
            out.append(best)
        return out

    def solutions(self, T_tcp, seed, weights=None, limit_margin=None, link=None):
        """All IK configurations for the pose of ``link`` (default tcp) in base_link, nearest-wrap to ``seed``,
        within the limits minus ``limit_margin``. -> [(cost, q)] sorted by cost (weighted max |q - seed|)."""
        link = link or self.cell.tcp_link
        weights = np.array(weights or self.cell.ik_weights)
        margin = self.cell.ik_limit_margin if limit_margin is None else limit_margin
        seed = np.array(seed, float)
        T_link_tool0 = np.linalg.inv(self.urdf.fk(link, {}, self.cell.flange_link))
        T_flange = T_tcp @ T_link_tool0
        branches = []
        if self.dh is not None:
            branches = dh_ik(np.linalg.inv(self.T_bl_dh) @ T_flange, self.dh)
        branches.append(list(seed))          # the seed itself as a branch: covers singular / unknown models
        out, seen = [], []
        for b in branches:
            w = self._nearest_wrap(b, seed)
            if w is None:
                continue
            q, ep, er = self.refine(T_tcp, w, link)
            if ep > 1e-5 or er > 1e-4:
                continue
            q = self._nearest_wrap(q, seed)
            if q is None:
                continue
            q = np.array(q)
            if np.any(q < self.lo + margin) or np.any(q > self.hi - margin):
                continue
            if any(np.max(np.abs(q - s)) < 1e-4 for s in seen):
                continue
            seen.append(q)
            out.append((float(np.max(weights * np.abs(q - seed))), [float(v) for v in q]))
        out.sort(key=lambda c: c[0])
        return out
