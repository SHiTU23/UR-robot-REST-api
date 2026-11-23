"""The arm's factory calibration: read from the controller, turned into the file ur_robot_driver takes as
kinematics_params_file.

Same result as `ros2 launch ur_calibration calibration_correction.launch.py`, whose released package (3.9.0, with
ur_client_library 2.15.2) never connects and retries forever. The method follows ur_calibration's calibration.cpp and the
hash follows ur_client_library's KinematicsInfo::toHash. It only reads, on the controller's read-only primary port.
"""
import math
import socket
import struct
import time

import numpy as np

PORT = 30011                 # primary interface, read-only
ROBOT_STATE, KINEMATICS_INFO = 16, 5
LINKS = ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")


def read_kinematics(ip, timeout_s=5.0):
    """{checksum, theta, a, d, alpha, calibration_status} from the controller."""
    s = socket.create_connection((ip, PORT), timeout=3.0)
    s.settimeout(1.0)
    try:
        buf, t0 = b"", time.time()
        while time.time() - t0 < timeout_s:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise OSError("the controller closed the connection")
            buf += chunk
            while len(buf) >= 5:
                size, ptype = struct.unpack(">iB", buf[:5])
                if size < 5:
                    raise OSError(f"bad primary package size {size}")
                if len(buf) < size:
                    break
                body, buf = buf[5:size], buf[size:]
                found = _find_kinematics(body) if ptype == ROBOT_STATE else None
                if found:
                    return found
    finally:
        s.close()
    raise TimeoutError(f"no kinematics data from {ip}:{PORT} within {timeout_s:.0f} s")


def _find_kinematics(body):
    off = 0
    while off + 5 <= len(body):
        size, sub = struct.unpack(">iB", body[off:off + 5])
        if size < 5:
            return None
        if sub == KINEMATICS_INFO and size >= 5 + 24 + 192 + 4:
            p = body[off + 5:off + size]
            checksum = struct.unpack(">6I", p[:24])
            d = struct.unpack(">24d", p[24:216])
            return {"checksum": list(checksum), "theta": list(d[0:6]), "a": list(d[6:12]), "d": list(d[12:18]),
                    "alpha": list(d[18:24]), "calibration_status": struct.unpack(">I", p[216:220])[0]}
        off += size
    return None


def _shift_mix(v):
    return v ^ (v >> 47)


def std_hash(data):
    """libstdc++'s 64-bit std::hash<std::string> (MurmurHash64A-style, seed 0xc70f6907)."""
    mul, mask = 0xc6a4a7935bd1e995, 2 ** 64 - 1
    h = (0xc70f6907 ^ (len(data) * mul)) & mask
    whole = len(data) // 8 * 8
    for i in range(0, whole, 8):
        k = (_shift_mix((int.from_bytes(data[i:i + 8], "little") * mul) & mask) * mul) & mask
        h = ((h ^ k) * mul) & mask
    if len(data) % 8:
        h = ((h ^ int.from_bytes(data[whole:], "little")) * mul) & mask
    h = (_shift_mix(h) * mul) & mask
    return _shift_mix(h)


def calibration_hash(k):
    """The hash the driver compares with the file. C++ `ss << double` writes %g with 6 significant digits."""
    text = "".join("%g%g%g%g" % (k["theta"][i], k["d"][i], k["a"][i], k["alpha"][i]) for i in range(6))
    return f"calib_{std_hash(text.encode())}"


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _chain(k):
    """Each DH segment as two transforms: (theta, d), then (a, alpha)."""
    chain = []
    for i in range(6):
        t = np.eye(4)
        t[:3, :3] = _rz(k["theta"][i])
        t[2, 3] = k["d"][i]
        chain.append(t)
        t = np.eye(4)
        t[:3, :3] = _rx(k["alpha"][i])
        t[0, 3] = k["a"][i]
        chain.append(t)
    return chain


def _correct_axis(chain, k, link):
    """Set d of this joint to 0 and keep the structure: the next joint's axis is met in this joint's XY plane."""
    d_theta, a_alpha = chain[2 * link], chain[2 * link + 1]
    if d_theta[2, 3] == 0.0:
        return
    root = d_theta @ a_alpha
    p0 = root[:3, 3]
    p1 = (root @ chain[2 * link + 2])[:3, 3]
    direction = (p1 - p0) / np.linalg.norm(p1 - p0)
    param = -p0[2] / direction[2]             # along the next joint's axis, to the XY plane
    point = p0 + param * direction
    subtract = math.pi if abs(a_alpha[0, 3]) > 0 else 0.0
    new_theta = math.atan2(point[1], point[0]) - subtract
    d_theta[2, 3] = 0.0
    d_theta[:3, :3] = _rz(new_theta)
    a_alpha[0, 3] = -float(np.linalg.norm(point))    # upper and lower arm have negative length in UR's DH
    a_alpha[:3, :3] = _rz(k["theta"][link] - new_theta) @ _rx(k["alpha"][link])
    chain[2 * link + 2][2, 3] -= param * (1.0 if direction[2] > 0 else -1.0)


def _rpy(r):
    """URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return (math.atan2(r[2, 1], r[2, 2]), math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0])), math.atan2(r[1, 0], r[0, 0]))


def to_kinematics(k):
    """The `kinematics` section of the driver's file: {shoulder: {x, y, z, roll, pitch, yaw}, ..., hash}."""
    chain = _chain(k)
    _correct_axis(chain, k, 1)
    _correct_axis(chain, k, 2)
    simplified = [chain[0]] + [chain[i] @ chain[i + 1] for i in range(1, 11, 2)]
    out = {}
    for name, t in zip(LINKS, simplified):
        roll, pitch, yaw = _rpy(t)
        out[name] = {"x": float(t[0, 3]), "y": float(t[1, 3]), "z": float(t[2, 3]),
                     "roll": roll, "pitch": pitch, "yaw": yaw}
    out["hash"] = calibration_hash(k)
    return out
