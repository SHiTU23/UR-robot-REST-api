"""The wrist camera's pose on the flange (T_tool0_camera, the hand-eye calibration) the robot description is built with
(added 2026-11-20).

Before: tool.camera_xyz_m / camera_rpy_rad in config/cell.yaml, typed in once (hand-eye of 2026-09-15). The camera was
moved since and the client (pipeFit) calibrated it again, but the URDF still had the old pose: RViz showed the camera
where it no longer is, and the planning scene checked the bin against a camera that is not there.

Now: the client sends its accepted hand-eye (PUT /robot/camera_mount, routes_tcp.py) -> saved here, in
tool.camera_mount_file -> ros/launch/cell.launch.py hands it to the URDF (env UR_CAMERA_MOUNT) at every start. With
"apply": true the API and the ROS side restart at once (as for the TCP: the camera link is part of the robot
description). No file: cell.yaml's values, as before.

The camera bracket's box (tool.bracket_*, hand-estimated for the old mount) would stay where the camera used to be, a
phantom block in the collision checks. With a saved mount it is derived instead: a bar from the tool changer's surface
out to the camera body, at the camera's height (bracket()). Its width / thickness are cell.yaml's bracket_size_m.

Standard library + PyYAML only: it runs in the ROS launch and in the API."""
import math
import os
import time

import yaml

XYZ_MAX_M = 0.35          # plausibility: the camera's optical centre within 350 mm of the flange centre
ORTHO_TOL = 1e-3          # the rotation part of a sent matrix: orthonormal within this
SAME_MM = 1.0             # a saved mount within this and SAME_DEG of the one in use: no restart needed
SAME_DEG = 0.2


def path_for(cell_cfg):
    t = (cell_cfg or {}).get("tool") or {}
    return t.get("camera_mount_file") or None


def rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def R_to_rpy(R):
    """URDF roll, pitch, yaw (R = Rz(yaw) Ry(pitch) Rx(roll))"""
    sp = -R[2][0]
    if abs(sp) < 1.0 - 1e-9:
        return [math.atan2(R[2][1], R[2][2]), math.asin(sp), math.atan2(R[1][0], R[0][0])]
    return [math.atan2(-R[1][2], R[1][1]), math.copysign(math.pi / 2, sp), 0.0]     # gimbal lock: yaw folded into roll


def check_T(T):
    """None if T (4x4 nested list, tool0 -> camera) is a plausible camera mount, else why not"""
    if not (isinstance(T, list) and len(T) in (3, 4) and all(isinstance(r, list) and len(r) == 4 for r in T)
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for r in T for v in r)):
        return "T_tool0_camera must be a 4x4 (or 3x4) matrix of finite numbers"
    R = [[float(v) for v in r[:3]] for r in T[:3]]
    for i in range(3):
        for j in range(3):
            d = sum(R[k][i] * R[k][j] for k in range(3)) - (1.0 if i == j else 0.0)
            if abs(d) > ORTHO_TOL:
                return "the rotation part of T_tool0_camera is not orthonormal"
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    if det < 0:
        return "the rotation part of T_tool0_camera is a reflection (det < 0)"
    return check_xyz([float(r[3]) for r in T[:3]])


def check_xyz(xyz):
    n = math.sqrt(sum(v * v for v in xyz))
    if n > XYZ_MAX_M:
        return f"the camera would be {n * 1000:.0f} mm from the flange centre (at most {XYZ_MAX_M * 1000:.0f}; metres, not millimetres?)"
    return None


def save(path, xyz, rpy, set_by="api", note=None, extra=None):
    doc = {"xyz_m": [round(float(v), 6) for v in xyz], "rpy_rad": [round(float(v), 6) for v in rpy],
           "source": {"set_by": set_by, "at": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "note": note or "set through the API (PUT /robot/camera_mount)",
                      **(extra or {})}}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("# The wrist camera's pose on the flange (tool0 -> camera optical frame: x right, y down, z forward), the\n"
                "# hand-eye calibration (ur_api/camera_mount.py). Set with PUT /robot/camera_mount; used at the next start.\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    os.replace(tmp, path)
    return doc


def load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    if not (isinstance(d.get("xyz_m"), list) and isinstance(d.get("rpy_rad"), list)):
        return None
    return d


def bracket(xyz, rpy, tool):
    """(size [3], xyz [3], rpy [3]) of the bracket box in tool0. With tool.camera_mount_plate (the mount's real shape): its
    arm, turned about tool0 z so the plate's x runs along minus the image x (the camera's screw holes). Else a bar from
    the changer's surface out to the camera body's centre, at the body's height (bracket_size_m: width, thickness)."""
    R = rpy_to_R(rpy)
    pl = tool.get("camera_mount_plate")
    if pl:
        ax, ay = -R[0][0], -R[1][0]                     # minus the image x, flattened onto the flange plane
        yaw = math.atan2(ay, ax)
        (x0, x1), (y0, y1), th = pl["arm_x_m"], pl["arm_y_m"], float(pl["thickness_m"])
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        c, s = math.cos(yaw), math.sin(yaw)
        return [x1 - x0, y1 - y0, th], [c * cx - s * cy, s * cx + c * cy, th / 2], [0.0, 0.0, yaw]
    cc = tool.get("camera_box_center_m") or [0.0, 0.0, 0.0]
    b = [xyz[i] + sum(R[i][k] * cc[k] for k in range(3)) for i in range(3)]       # the body's centre in tool0
    r1 = math.hypot(b[0], b[1])
    r0 = float(tool.get("changer_radius_m", 0.0375))
    bs = tool.get("bracket_size_m") or [0.045, 0.110, 0.030]
    width, thick = float(bs[0]), float(bs[2])
    if r1 < r0 + 0.005:                       # the camera over the changer: no bar to draw, a small block at the camera
        return [0.01, 0.01, 0.01], b, [0.0, 0.0, 0.0]
    yaw = math.atan2(b[1], b[0])
    rm = (r0 + r1) / 2
    return [r1 - r0, width, thick], [rm * math.cos(yaw), rm * math.sin(yaw), b[2]], [0.0, 0.0, yaw]


def as_arg(d, tool):
    """env UR_CAMERA_MOUNT for the URDF: camera x y z roll pitch yaw, bracket x y z roll pitch yaw, bracket size x y z"""
    size, bx, br = bracket(d["xyz_m"], d["rpy_rad"], tool)
    return " ".join(repr(float(v)) for v in list(d["xyz_m"]) + list(d["rpy_rad"]) + list(bx) + list(br) + list(size))


def same(a_xyz, a_rpy, b_xyz, b_rpy):
    if max(abs(x - y) for x, y in zip(a_xyz, b_xyz)) * 1000 > SAME_MM:
        return False
    Ra, Rb = rpy_to_R(a_rpy), rpy_to_R(b_rpy)
    tr = sum(Ra[k][i] * Rb[k][i] for i in range(3) for k in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))) <= SAME_DEG
