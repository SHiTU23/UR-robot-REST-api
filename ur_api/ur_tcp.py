"""The TCP the robot description is built with (the `tcp` link): saved in robot.tcp_file (config/cell.yaml), set through
the API (PUT /robot/tcp) or copied from the controller's active TCP (POST /robot/tcp/from_controller, ./run.sh save-tcp).

The saved file is what counts: ros/launch/cell.launch.py builds the tcp link from it at each start. The controller is
read only when no file exists yet (robot.tcp_mode: file, the default), at every start (tcp_mode: controller), or kept
up to date while running (tcp_mode: follow, ur_api/tcp_follow.py: a changed active TCP is saved and the stack restarts).
A gripper's own block (gripper.<model>.tcp_mode in config/cell.yaml) overrides robot.tcp_mode for that gripper. Why
not always the controller: a program can switch the active TCP behind the operator's back (the OnRobot URCap sets its
own, 234 mm turned 180 deg, when an RG6 node runs), and the API's motions never use the controller's TCP anyway: they
are joint trajectories planned to the URDF's tcp link.

Reading is READ-ONLY on the robot: it subscribes to the RTDE output tcp_offset (port 30004) and sends no command, so it
can run next to the driver. Standard library + PyYAML only: it runs in the ROS launch, in the API, and in tools/."""
import math
import os
import socket
import struct
import time

import yaml

RTDE_PORT = 30004
MODES = ("file", "controller", "follow")
# The mode chosen through the API (PUT /robot/tcp or POST /robot/tcp/from_controller with "mode"), per gripper: wins over
# config/cell.yaml, so "accept these values" on a client is not undone by follow. {"robotiq_2f85": "file", ...}
MODE_FILE = os.path.join(os.environ.get("UR_STATE_DIR") or os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "state"), "tcp_mode.json")
SAME_MM = 0.2                      # a controller TCP within this (any axis) and SAME_DEG of the one in use is the same TCP
SAME_DEG = 0.05
XY_MAX_M = 0.2                     # plausibility: the fingertip centre within 200 mm of the flange axis
Z_RANGE_M = (0.02, 0.8)            # and 20..800 mm along it


def read_tcp(ip, timeout_s=3.0):
    """[x, y, z, rx, ry, rz]: the controller's active TCP in the flange frame (m, rotation vector rad)."""
    with socket.create_connection((ip, RTDE_PORT), timeout_s) as s:
        s.settimeout(timeout_s)

        def send(cmd, payload=b""):
            s.sendall(struct.pack(">HB", 3 + len(payload), cmd) + payload)

        def recv():
            n, c = struct.unpack(">HB", s.recv(3, socket.MSG_WAITALL))
            return c, (s.recv(n - 3, socket.MSG_WAITALL) if n > 3 else b"")

        send(86, struct.pack(">H", 2))                        # protocol version 2
        recv()
        send(79, struct.pack(">d", 10.0) + b"tcp_offset")     # output recipe: only tcp_offset
        c, d = recv()
        if c != 79 or b"NOT_FOUND" in d:
            raise RuntimeError(f"the controller refused the tcp_offset recipe: {d!r}")
        send(83)                                              # start
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            c, d = recv()
            if c == 85:                                       # data package: recipe id, then 6 doubles
                send(80)                                      # pause
                return list(struct.unpack(">6d", d[1:49]))
        raise TimeoutError("no RTDE data package")


def rotvec_to_rpy(rv):
    """Rotation vector -> URDF roll, pitch, yaw (fixed axes x, y, z: R = Rz(yaw) Ry(pitch) Rx(roll))."""
    th = math.sqrt(sum(v * v for v in rv))
    if th < 1e-12:
        return [0.0, 0.0, 0.0]
    x, y, z = (v / th for v in rv)
    c, s, C = math.cos(th), math.sin(th), 1 - math.cos(th)
    r20, r21, r22 = z * x * C - y * s, z * y * C + x * s, c + z * z * C
    r10, r00 = y * x * C + z * s, c + x * x * C
    return [math.atan2(r21, r22), math.asin(max(-1.0, min(1.0, -r20))), math.atan2(r10, r00)]


def rpy_to_rotvec(rpy):
    """URDF roll, pitch, yaw -> rotation vector (the pendant's RX, RY, RZ)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
         [-sp, cp * sr, cp * cr]]
    cos_th = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
    th = math.acos(cos_th)
    if th < 1e-12:
        return [0.0, 0.0, 0.0]
    anti = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]       # 2 sin(th) * axis
    if math.pi - th > 1e-3:
        k = 2 * math.sin(th)
        return [th * a / k for a in anti]
    # near 180 deg sin(th) is tiny: the axis from the symmetric part, R = cos I + (1 - cos) a a^T + sin [a]x
    C = 1 - math.cos(th)
    ax = [math.sqrt(max(0.0, (R[i][i] - math.cos(th)) / C)) for i in range(3)]
    i = max(range(3), key=lambda j: ax[j])
    for j in range(3):
        if j != i:
            ax[j] = math.copysign(ax[j], R[i][j] + R[j][i])
    if anti[i] < 0:                                   # the antisymmetric part still tells the sign below 180 deg
        ax = [-v for v in ax]
    n = math.sqrt(sum(v * v for v in ax))
    return [th * v / n for v in ax]


def mode_override(gripper_model, path=None):
    """The mode set through the API for this gripper (state/tcp_mode.json), or None."""
    try:
        with open(path or MODE_FILE) as f:
            m = (yaml.safe_load(f) or {}).get(gripper_model)
    except (OSError, yaml.YAMLError, AttributeError):
        return None
    return m if m in MODES else None


def set_mode_override(gripper_model, mode, path=None):
    """Remember the mode for this gripper (None: forget it, the config's applies again)."""
    path = path or MODE_FILE
    try:
        with open(path) as f:
            d = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        d = {}
    if mode is None:
        d.pop(gripper_model, None)
    else:
        if mode not in MODES:
            raise ValueError(f"mode {mode!r}: one of {MODES}")
        d[gripper_model] = mode
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".tmp", "w") as f:
        f.write(__import__("json").dumps(d, indent=1) + "\n")       # JSON is YAML: mode_override reads it
    os.replace(path + ".tmp", path)


def mode_for(cell_cfg, gripper_model, override_path=None):
    """file | controller | follow: the one set through the API (state/tcp_mode.json) wins, then the gripper's block
    (gripper.<model>.tcp_mode), then robot.tcp_mode."""
    block = (cell_cfg.get("gripper") or {}).get(gripper_model)
    m = mode_override(gripper_model, override_path) or \
        (block.get("tcp_mode") if isinstance(block, dict) else None) or (cell_cfg.get("robot") or {}).get("tcp_mode") or "file"
    if m not in MODES:
        raise ValueError(f"tcp_mode {m!r} for {gripper_model}: one of {MODES}")
    return m


def same(tcp, xyz_m, rpy_rad):
    """True if the controller TCP [x, y, z, rx, ry, rz] is the TCP xyz_m + rpy_rad (within SAME_MM / SAME_DEG)."""
    if max(abs(a - b) for a, b in zip(tcp[:3], xyz_m)) * 1000 > SAME_MM:
        return False
    a, b = _R(rotvec_to_rpy(tcp[3:6])), _R(rpy_rad)
    tr = sum(a[i][j] * b[i][j] for i in range(3) for j in range(3))          # trace(a^T b)
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))) <= SAME_DEG


def _R(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def check(xyz):
    """None if [x, y, z] is a plausible TCP, else why not."""
    if math.hypot(xyz[0], xyz[1]) > XY_MAX_M:
        return f"x, y put the TCP {math.hypot(xyz[0], xyz[1]) * 1000:.0f} mm off the flange axis (at most {XY_MAX_M * 1000:.0f})"
    if not Z_RANGE_M[0] <= xyz[2] <= Z_RANGE_M[1]:
        return f"z {xyz[2]:.4f} m is outside {Z_RANGE_M[0]}..{Z_RANGE_M[1]} m (metres, not millimetres)"
    return None


def save(path, tcp, gripper_model, ip=None, set_by="controller", note=None):
    """Write the TCP file (atomically) and return what was written. tcp = [x, y, z, rx, ry, rz] (rotation vector)."""
    src = {"set_by": set_by, "at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    if ip:
        src["robot_ip"] = ip
    src["note"] = note or ("the controller's active TCP (pendant: Installation > General > TCP)"
                           if set_by == "controller" else "set through the API (PUT /robot/tcp)")
    doc = {"gripper_model": gripper_model,
           "xyz_m": [round(float(v), 6) for v in tcp[:3]],
           "rotvec_rad": [round(float(v), 6) for v in tcp[3:6]],
           "rpy_rad": [round(v, 6) for v in rotvec_to_rpy(tcp[3:6])],
           "source": src}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("# The TCP the robot description is built with (ur_api/ur_tcp.py). Change it with PUT /robot/tcp or\n"
                "# POST /robot/tcp/from_controller, then restart the stack (./run.sh restart).\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    os.replace(tmp, path)
    return doc


def load(path, gripper_model):
    """The saved TCP {xyz_m, rpy_rad, ...} if the file exists and belongs to this gripper model, else None."""
    if not path or not os.path.exists(path):
        return None
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    if d.get("gripper_model") != gripper_model:
        return None
    return d


def describe(d):
    """Where a saved TCP came from, in a few words."""
    s = d.get("source") or {}
    at = s.get("at") or s.get("read_at")
    who = "set through the API" if s.get("set_by") == "api" else "copied from the controller"
    return f"{who} {at or ''}".strip()


def as_arg(d):
    """The saved TCP as the space-separated string the URDF takes (env UR_TCP): x y z roll pitch yaw."""
    return " ".join(repr(float(v)) for v in list(d["xyz_m"]) + list(d["rpy_rad"]))
