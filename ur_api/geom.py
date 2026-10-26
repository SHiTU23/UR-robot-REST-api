"""Small rigid-transform helpers (ROS 2 has no tf.transformations).

Quaternions are [x, y, z, w]. Euler angles are 'sxyz' (static axes, roll
about x, then pitch about y, then yaw about z), the convention the REST API
uses for roll/pitch/yaw: gripper pointing straight down = roll 180.
"""
import math

import numpy as np


def quat_to_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(R):
    """[x, y, z, w] with w >= 0 (Shepperd's method, numerically safe)."""
    R = np.asarray(R, float)[:3, :3]
    t = np.trace(R)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    q = np.array(q)
    q /= np.linalg.norm(q)
    return (-q if q[3] < 0 else q).tolist()


def euler_to_quat(roll, pitch, yaw):
    """sxyz Euler angles (radians) -> [x, y, z, w]."""
    return matrix_to_quat(euler_to_matrix(roll, pitch, yaw))


def euler_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_to_euler(R):
    """Rotation matrix -> sxyz (roll, pitch, yaw) radians."""
    R = np.asarray(R, float)[:3, :3]
    cy = math.hypot(R[0, 0], R[1, 0])
    if cy > 1e-9:
        return (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], cy), math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], cy), 0.0)


def pose_to_T(pose):
    """{"position": [3], "orientation": [x, y, z, w]} -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(pose["orientation"])
    T[:3, 3] = pose["position"]
    return T


def T_to_pose(T):
    return {"position": [float(v) for v in np.asarray(T)[:3, 3]], "orientation": matrix_to_quat(T)}


def rot_z(a):
    T = np.eye(4)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return T


def trans(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def angle_between(Ra, Rb):
    """Rotation angle (radians) between two rotation matrices."""
    R = np.asarray(Ra)[:3, :3].T @ np.asarray(Rb)[:3, :3]
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))


# The UR controller's "base" frame (pendant, RTDE, URScript poses) is ROS's
# base_link turned 180 deg about z. The API works in base_link and converts.
T_BASELINK_BASE = rot_z(math.pi)
