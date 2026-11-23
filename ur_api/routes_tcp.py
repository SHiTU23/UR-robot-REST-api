"""/robot/tcp: the TCP (fingertip centre) the robot description is built with, and changing it.

    GET  /robot/tcp                      in_use (the loaded tcp link), saved (robot.tcp_file), restart_required;
                                         ?controller=1 also reads the controller's active TCP (read-only, RTDE)
    PUT  /robot/tcp                      {xyz_m: [x, y, z], rpy_rad | rotvec_rad: [...] (default no rotation), note}
                                         -> saved; used after a restart
    POST /robot/tcp/from_controller      the controller's active TCP (as the pendant shows it) -> saved
    Both also take "mode": file | controller | follow (kept per gripper in state/tcp_mode.json, wins over cell.yaml;
    PUT with follow is refused: follow would replace the values at once), and "apply": true = restart the API and the
    ROS side now so the TCP is used (202; 409 while anything runs; ~30-60 s without answers).

The tcp link is part of the robot description that move_group, the driver's state publisher and this server load at
start, so a new TCP takes effect after `./run.sh restart` (GET reports restart_required until then). Nothing here moves
the robot or writes to the controller: the pendant's own TCP is left as it is (the API's motions do not use it)."""
import logging
import math
import os
import threading
import time

from flask import Blueprint, jsonify, request

import ur_tcp
from ctx import C
from errors import err

bp = Blueprint("tcp", __name__)
log = logging.getLogger("ur_rest_api")
STATE_DIR = os.environ.get("UR_STATE_DIR", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "state"))
RESTART = "restart the stack to use it: ./run.sh restart (or ./run.sh up)"


def _in_use():
    c = C.cell
    return {"xyz_m": [round(v, 6) for v in c.tcp_xy + [c.tcp_z]], "rpy_rad": [round(v, 6) for v in c.tcp_rpy],
            "rotvec_rad": [round(v, 6) for v in ur_tcp.rpy_to_rotvec(c.tcp_rpy)], "source": c.tcp_source}


def _differs(a, b):
    return (max(abs(x - y) for x, y in zip(a["xyz_m"], b["xyz_m"])) > 1e-5
            or max(abs(math.remainder(x - y, 2 * math.pi)) for x, y in zip(a["rpy_rad"], b["rpy_rad"])) > 1e-5)


def _status(extra=None):
    saved = ur_tcp.load(C.cell.tcp_file, C.cell.gripper_model) if C.cell.tcp_file else None
    use = _in_use()
    out = {"in_use": use, "saved": saved, "file": C.cell.tcp_file, "gripper_model": C.cell.gripper_model,
           "mode": C.cell.tcp_mode, "mode_saved": ur_tcp.mode_for(C.cell.cfg, C.cell.gripper_model), "model_tcp_z_m": round(C.cell.model_tcp_z, 6),
           "restart_required": bool(saved) and _differs(saved, use)}
    if out["restart_required"]:
        out["note"] = "the saved TCP differs from the one in use: " + RESTART
    out.update(extra or {})
    return out


def _warnings(doc):
    w = []
    dz = doc["xyz_m"][2] - C.cell.model_tcp_z
    if abs(dz) > 0.005:
        w.append(f"z is {abs(dz) * 1000:.1f} mm {'beyond' if dz > 0 else 'short of'} the {C.cell.gripper_model} "
                 f"model's closed fingertips ({C.cell.model_tcp_z:.3f} m)")
    if any(abs(v) > 1e-4 for v in doc["rpy_rad"]):
        w.append(f"the TCP is rotated against the flange (rpy {doc['rpy_rad']} rad): poses and rpy_deg will refer to "
                 "that rotated frame")
    return w


def _vec(d, key, n=3):
    v = d.get(key)
    if not isinstance(v, list) or len(v) != n or not all(
            isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v):
        raise err(400, "bad_request", f"{key} must be [{', '.join('xyz'[:n]) if n == 3 else '...'}] finite numbers", key)
    return [float(x) for x in v]


def _mode_apply(d, allowed):
    """(mode or None, apply) from the body, checked before anything is saved."""
    mode = d.get("mode")
    if mode is not None and mode not in allowed:
        raise err(400, "bad_request", f"mode must be one of {list(allowed)}", "mode")
    apply = d.get("apply", False)
    if not isinstance(apply, bool):
        raise err(400, "bad_request", "apply must be true or false", "apply")
    if apply and C.mock:
        raise err(409, "mock_hardware", "apply restarts the stack: not with mock hardware (use ./run.sh restart)")
    if apply and not C.lock.acquire(blocking=False):
        raise err(409, "busy", "Robot is busy: apply the TCP when the current motion / gripper action is done")
    return mode, apply


def _finish(doc, mode, apply):
    """Store the mode, then (apply) restart: answer first, keep the lock until the process exits."""
    if mode:
        ur_tcp.set_mode_override(C.cell.gripper_model, mode)
    out = _status({"warnings": _warnings(doc)})
    if not apply:
        return jsonify(out), 200
    log.warning("TCP %s %s saved through the API with apply: restarting the API and the ROS side",
                doc["xyz_m"], doc["rotvec_rad"])
    import tcp_follow

    def _go():
        time.sleep(1.0)
        tcp_follow.restart_stack(STATE_DIR, "tcp set through the API")
    threading.Thread(target=_go, daemon=True).start()
    out.update(status="restarting", msg="the API and the ROS side restart with this TCP: back in about 30-60 s")
    return jsonify(out), 202


def _save(tcp, set_by, note=None):
    if not C.cell.tcp_file:
        raise err(409, "no_tcp_file", "robot.tcp_file is not set in config/cell.yaml")
    why = ur_tcp.check(tcp[:3])
    if why:
        raise err(422, "implausible_tcp", why, "xyz_m")
    return ur_tcp.save(C.cell.tcp_file, tcp, C.cell.gripper_model, None if C.mock else C.cell.robot_ip, set_by, note)


@bp.route("/robot/tcp", methods=["GET"])
def tcp_get():
    """The TCP in use (the loaded tcp link, tool0 frame), the saved one, restart_required; ?controller=1 also reads the
    controller's active TCP over RTDE (read-only)."""
    extra = {}
    if request.args.get("controller") in ("1", "true", "yes"):
        if C.mock:
            extra["controller"] = None
        else:
            try:
                live = ur_tcp.read_tcp(C.cell.robot_ip)
                extra["controller"] = {"xyz_m": [round(v, 6) for v in live[:3]],
                                       "rotvec_rad": [round(v, 6) for v in live[3:]],
                                       "rpy_rad": [round(v, 6) for v in ur_tcp.rotvec_to_rpy(live[3:])]}
            except Exception as e:  # noqa: BLE001
                extra["controller_error"] = str(e)
    return jsonify(_status(extra)), 200


@bp.route("/robot/tcp", methods=["PUT", "POST"])
def tcp_set():
    """Save a new TCP: {xyz_m: [x, y, z] (metres, flange/tool0 frame, as the pendant's X, Y, Z), rpy_rad or rotvec_rad
    (the pendant's RX, RY, RZ; default no rotation), note}. Used after a restart (./run.sh restart)."""
    d = request.get_json(silent=True)
    if not isinstance(d, dict):
        raise err(400, "bad_request", "JSON body must be an object")
    xyz = _vec(d, "xyz_m")
    if "rpy_rad" in d and "rotvec_rad" in d:
        raise err(400, "bad_request", "give rpy_rad or rotvec_rad, not both", "rpy_rad")
    rv = _vec(d, "rotvec_rad") if "rotvec_rad" in d else (
        ur_tcp.rpy_to_rotvec(_vec(d, "rpy_rad")) if "rpy_rad" in d else [0.0, 0.0, 0.0])
    note = d.get("note")
    if note is not None and (not isinstance(note, str) or len(note) > 200):
        raise err(400, "bad_request", "note is a string of at most 200 characters", "note")
    if ur_tcp.check(xyz):
        raise err(422, "implausible_tcp", ur_tcp.check(xyz), "xyz_m")
    mode, apply = _mode_apply(d, ("file", "controller"))
    try:
        doc = _save(xyz + rv, "api", note)
    except Exception:
        if apply:
            C.lock.release()
        raise
    return _finish(doc, mode, apply)


@bp.route("/robot/tcp/from_controller", methods=["POST"])
def tcp_from_controller():
    """Save the controller's active TCP (read-only RTDE read). Check first with GET /robot/tcp?controller=1: a program
    (the RG6 URCap) may have switched the active TCP away from the one set on the pendant."""
    if C.mock:
        raise err(409, "mock_hardware", "no controller to read the TCP from (mock hardware)")
    d = request.get_json(silent=True) or {}
    if not isinstance(d, dict):
        raise err(400, "bad_request", "JSON body must be an object")
    try:
        live = ur_tcp.read_tcp(C.cell.robot_ip)
    except Exception as e:  # noqa: BLE001
        raise err(503, "robot_unreachable", f"could not read the TCP from {C.cell.robot_ip}: {e}")
    mode, apply = _mode_apply(d, ur_tcp.MODES)
    try:
        doc = _save(live, "controller")
    except Exception:
        if apply:
            C.lock.release()
        raise
    return _finish(doc, mode, apply)


# ------------------------------------------------------------------ the wrist camera's mount (2026-11-20)
import camera_mount  # noqa: E402


def _camera_in_use():
    """tool0 -> wrist_camera of the loaded robot description (fixed joints: any joint values)"""
    import numpy as np
    import geom
    links = set(C.kin.urdf.links)
    if "wrist_camera" not in links:
        return None
    qd = C.kin.qdict([0.0] * C.cell.n_joints, C.cell.finger_q(0.0))
    root = C.cell.planning_frame
    T = np.linalg.inv(C.kin.urdf.fk("tool0", qd, root)) @ C.kin.urdf.fk("wrist_camera", qd, root)
    return {"xyz_m": [round(float(v), 6) for v in T[:3, 3]], "rpy_rad": [round(float(v), 6) for v in geom.matrix_to_euler(T)]}


def _camera_status(extra=None):
    # (2026-11-22) the mounted gripper SET's camera mount: its own file, else the shared one (own_calibration false)
    saved, path, own = camera_mount.load_for(C.cell.cfg, C.cell.gripper_model)
    use = _camera_in_use()
    out = {"in_use": use, "saved": saved, "file": path, "gripper_set": C.cell.gripper_model, "own_calibration": own,
           "save_to": camera_mount.path_for(C.cell.cfg, C.cell.gripper_model),
           "restart_required": bool(saved and use) and not camera_mount.same(saved["xyz_m"], saved["rpy_rad"], use["xyz_m"], use["rpy_rad"])}
    if out["restart_required"]:
        out["note"] = "the saved camera mount differs from the one in use: " + RESTART
    out.update(extra or {})
    return out


@bp.route("/robot/camera_mount", methods=["GET"])
def camera_mount_get():
    """The wrist camera's pose on the flange in use (the loaded wrist_camera link, tool0 frame), the saved one
    (tool.camera_mount_file), restart_required."""
    return jsonify(_camera_status()), 200


@bp.route("/robot/camera_mount", methods=["PUT", "POST"])
def camera_mount_set():
    """Save the hand-eye calibration: {T_tool0_camera: 4x4 (tool0 -> camera optical frame: x right, y down, z forward),
    or xyz_m + rpy_rad; note; source (a small object kept with it); apply}. apply: true restarts the API and the ROS side
    now (202; 409 while anything runs) so RViz and the collision checks use it; else at the next restart. Unchanged
    (within 1 mm / 0.2 deg of the one in use): saved, no restart (200, restarted false)."""
    d = request.get_json(silent=True)
    if not isinstance(d, dict):
        raise err(400, "bad_request", "JSON body must be an object")
    path = camera_mount.path_for(C.cell.cfg, C.cell.gripper_model)     # (2026-11-22) the mounted gripper set's own file
    if not path:
        raise err(409, "no_camera_mount_file", "tool.camera_mount_file is not set in config/cell.yaml")
    if "T_tool0_camera" in d:
        why = camera_mount.check_T(d["T_tool0_camera"])
        if why:
            raise err(422, "implausible_camera_mount", why, "T_tool0_camera")
        T = d["T_tool0_camera"]
        xyz, rpy = [float(T[i][3]) for i in range(3)], camera_mount.R_to_rpy([[float(v) for v in T[i][:3]] for i in range(3)])
    else:
        xyz, rpy = _vec(d, "xyz_m"), _vec(d, "rpy_rad")
        why = camera_mount.check_xyz(xyz)
        if why:
            raise err(422, "implausible_camera_mount", why, "xyz_m")
    note, src = d.get("note"), d.get("source")
    if note is not None and (not isinstance(note, str) or len(note) > 200):
        raise err(400, "bad_request", "note is a string of at most 200 characters", "note")
    if src is not None and (not isinstance(src, dict) or len(str(src)) > 1000):
        raise err(400, "bad_request", "source is a small object", "source")
    _, apply = _mode_apply({"apply": d.get("apply", False)}, ())
    try:
        doc = camera_mount.save(path, xyz, rpy, "api", note, {"client": src} if src else None)
    except Exception:
        if apply:
            C.lock.release()
        raise
    use = _camera_in_use()
    if apply and use and camera_mount.same(xyz, rpy, use["xyz_m"], use["rpy_rad"]):
        C.lock.release()
        return jsonify(_camera_status({"restarted": False, "msg": "saved; the robot description already has this camera mount"})), 200
    out = _camera_status({"saved": doc})
    if not apply:
        return jsonify(out), 200
    log.warning("camera mount xyz %s rpy %s saved through the API with apply: restarting the API and the ROS side", doc["xyz_m"], doc["rpy_rad"])
    import tcp_follow

    def _go():
        time.sleep(1.0)
        tcp_follow.restart_stack(STATE_DIR, "camera mount set through the API")
    threading.Thread(target=_go, daemon=True).start()
    out.update(status="restarting", restarted=True, msg="the API and the ROS side restart with this camera mount: back in about 30-60 s")
    return jsonify(out), 202
