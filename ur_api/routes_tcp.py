"""/robot/tcp: the TCP (fingertip centre) the robot description is built with, and changing it.

    GET  /robot/tcp                      in_use (the loaded tcp link), saved (robot.tcp_file), restart_required;
                                         ?controller=1 also reads the controller's active TCP (read-only, RTDE)
    PUT  /robot/tcp                      {xyz_m: [x, y, z], rpy_rad | rotvec_rad: [...] (default no rotation), note}
                                         -> saved; used after a restart
    POST /robot/tcp/from_controller      the controller's active TCP (as the pendant shows it) -> saved

The tcp link is part of the robot description that move_group, the driver's state publisher and this server load at
start, so a new TCP takes effect after `./run.sh restart` (GET reports restart_required until then). Nothing here moves
the robot or writes to the controller: the pendant's own TCP is left as it is (the API's motions do not use it)."""
import math

from flask import Blueprint, jsonify, request

import ur_tcp
from ctx import C
from errors import err

bp = Blueprint("tcp", __name__)
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
           "mode": C.cell.cfg["robot"].get("tcp_mode", "file"), "model_tcp_z_m": round(C.cell.model_tcp_z, 6),
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
    doc = _save(xyz + rv, "api", note)
    return jsonify(_status({"warnings": _warnings(doc)})), 200


@bp.route("/robot/tcp/from_controller", methods=["POST"])
def tcp_from_controller():
    """Save the controller's active TCP (read-only RTDE read). Check first with GET /robot/tcp?controller=1: a program
    (the RG6 URCap) may have switched the active TCP away from the one set on the pendant."""
    if C.mock:
        raise err(409, "mock_hardware", "no controller to read the TCP from (mock hardware)")
    try:
        live = ur_tcp.read_tcp(C.cell.robot_ip)
    except Exception as e:  # noqa: BLE001
        raise err(503, "robot_unreachable", f"could not read the TCP from {C.cell.robot_ip}: {e}")
    doc = _save(live, "controller")
    return jsonify(_status({"warnings": _warnings(doc)})), 200
