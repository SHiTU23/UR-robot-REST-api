"""State, motion, gripper, force, health: the Franka API's robot endpoints for the UR (6 joints, RG6 / 2F-85)."""
import functools
import hashlib
import logging
import math
import time

import numpy as np
from flask import Blueprint, jsonify, request

import geom
from ctx import C
from errors import ApiError, err
from gripper import OUTCOME_STATUS
from motion import ROBOT_MODE_NAMES, SAFETY_NAMES, decode_error, joint_travel, traj_points

bp = Blueprint("robot", __name__)
log = logging.getLogger("ur_rest_api")


# ------------------------------------------------------------------ helpers

def busy():
    return jsonify({"error": "Robot is busy"}), 409


def with_lock(fn):
    """Motion / gripper endpoints share one NON-blocking lock: a second request gets 409, never queues."""
    @functools.wraps(fn)
    def wrapper(*a, **kw):
        if not C.lock.acquire(blocking=False):
            return busy()
        try:
            return fn(*a, **kw)
        finally:
            C.lock.release()
    return wrapper


def _flag(v, default=True):
    if v is None:
        return default
    return str(v) not in ("0", "false", "False", "no", "")


def last_error(since=None):
    return C.ros.last_error(since)


def frame_to_base_link(frame):
    """4x4 that takes coordinates in ``frame`` to base_link (base = the pendant/RTDE frame)."""
    frame = frame or C.cell.base_frame
    if frame in (C.cell.base_frame, C.cell.planning_frame):
        return np.eye(4)
    if frame == "base":
        return geom.T_BASELINK_BASE
    raise err(400, "bad_frame", f"frame must be {C.cell.base_frame} (default) or base (the pendant frame)")


def parse_orientation():
    """qx..qw (base frame quaternion) or roll, pitch, yaw (degrees, sxyz; gripper straight down = 180, 0, 0).
    -> [x, y, z, w] normalised, or None. ValueError on partial / bad input."""
    q = [request.args.get(k) for k in ("qx", "qy", "qz", "qw")]
    e = [request.args.get(k) for k in ("roll", "pitch", "yaw")]
    if any(v is not None for v in q):
        if any(v is None for v in q):
            raise ValueError("quaternion needs all of qx,qy,qz,qw")
        q = [float(v) for v in q]
    elif any(v is not None for v in e):
        if any(v is None for v in e):
            raise ValueError("euler orientation needs all of roll,pitch,yaw (degrees)")
        q = geom.euler_to_quat(*(math.radians(float(v)) for v in e))
    else:
        return None
    if not all(math.isfinite(v) for v in q):
        raise ValueError("orientation must be finite")
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-6:
        raise ValueError("orientation quaternion has ~zero norm")
    return [v / n for v in q]


def parse_motion_args():
    x, y, z = float(request.args.get("x")), float(request.args.get("y")), float(request.args.get("z"))
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("x, y, z must be finite")
    return {"x": x, "y": y, "z": z, "auto_recover": _flag(request.args.get("auto_recover")),
            "max_retries": int(request.args.get("max_retries", "1")),
            "preserve_orientation": _flag(request.args.get("preserve_orientation")),
            "orientation": parse_orientation(), "frame": request.args.get("frame") or C.cell.base_frame}


def parse_max_travel():
    raw = request.args.get("max_joint_travel_deg")
    if raw is None:
        return None
    deg = float(raw)
    if deg < 0:
        raise ValueError("max_joint_travel_deg must be >= 0")
    return math.radians(deg)


def tcp_pose(q):
    return geom.T_to_pose(C.kin.fk(q))


def target_pose(m):
    """The TCP target of a motion request in base_link: position x,y,z; orientation explicit, kept (current),
    or the canonical 'straight down' [1, 0, 0, 0]."""
    T_fb = frame_to_base_link(m["frame"])
    cur = geom.pose_to_T(tcp_pose(C.motion.current_q()))
    if m["orientation"] is not None:
        R = T_fb[:3, :3] @ geom.quat_to_matrix(m["orientation"])
    elif m["preserve_orientation"]:
        R = cur[:3, :3]
    else:
        R = geom.quat_to_matrix([1.0, 0.0, 0.0, 0.0])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (T_fb @ np.array([m["x"], m["y"], m["z"], 1.0]))[:3]
    return geom.T_to_pose(T)


def preflight():
    """None, or the 400 response when the arm already sits too close to a joint limit."""
    bad = C.cell.near_limits(C.motion.current_q())
    if bad:
        return jsonify({"error": "joint_near_limit",
                        "msg": "one or more joints are too close to their limits; home the robot or freedrive it first",
                        "joints_near_limit": bad,
                        "hint": "call GET /control/home, /control/nudge_joint, or freedrive the flagged joints away"}), 400
    return None


def motion_response(outcome, extra=None):
    body = {"outcome": outcome["outcome"], "attempts": outcome["attempts"],
            "recovery_triggered": outcome["recovery_triggered"],
            "recovery_succeeded": outcome["recovery_succeeded"] if outcome["recovery_triggered"] else None}
    body.update(extra or {})
    o = outcome["outcome"]
    meta = outcome.get("plan_metadata") or {}
    if meta.get("execute_error"):
        body["execute_error"] = meta["execute_error"]
    if o == "success":
        resp = jsonify({**body, "msg": "Plan 100%"}), 200
    elif o == "stopped":
        body["stop_reason"] = outcome.get("stop_reason", "user /control/stop")
        resp = jsonify({**body, "msg": "motion stopped by user /control/stop"}), 200
    elif o == "stopped_external_force":
        body["stop_reason"] = outcome.get("stop_reason")
        body["hint"] = "the UR made a protective stop (contact / push). GET /recover unlocks it (5 s after the stop)"
        resp = jsonify({**body, "msg": "motion stopped by a protective stop (contact or push); not retrying"}), 200
    elif o == "stopped_user_button":
        body["stop_reason"] = outcome.get("stop_reason")
        body["hint"] = "release the emergency stop / safeguard on the hardware (and confirm on the pendant) before " \
                       "retrying; software cannot clear this state"
        resp = jsonify({**body, "msg": "motion blocked: emergency stop or safeguard stop is active"}), 409
    elif o == "plan_failed":
        body["last_error"] = last_error()
        resp = jsonify({**body, "msg": "Plan not 100%"}), 202
    elif o == "plan_would_violate_limits":
        body["trajectory_violations"] = outcome.get("trajectory_violations", [])
        body["hint"] = "the plan would drive a joint near its limit mid-path; try another target, " \
                       "preserve_orientation=0, or plan_joint_path with planner_id=RRTConnect"
        resp = jsonify({**body, "msg": "plan rejected: would drift a joint past safe margin"}), 409
    elif o == "plan_too_large":
        body["joint_travel_deg"] = {j: round(math.degrees(t), 1) for j, t in outcome.get("joint_travel", {}).items()}
        body["max_joint_travel_deg"] = round(math.degrees(outcome.get("max_joint_travel_rad", 0.0)), 1)
        body["hint"] = "the plan would swing a joint a long way (elbow flip / base spin); move in smaller steps, or " \
                       "pass max_joint_travel_deg=<bigger> (0 disables) if the big move is intended"
        resp = jsonify({**body, "msg": "plan rejected: joint travel too large"}), 409
    elif o == "execute_failed":
        body["last_error"] = last_error()
        resp = jsonify({**body, "msg": "execution failed"}), 500
    elif o == "recovery_failed":
        body["last_error"] = last_error()
        resp = jsonify({**body, "msg": "execute failed and recovery also failed"}), 500
    else:
        resp = jsonify({**body, "msg": "unknown outcome"}), 500
    log.info("motion %s -> HTTP %s outcome=%s attempts=%s from %s", request.full_path, resp[1], o,
             outcome["attempts"], request.remote_addr)
    return resp


# ------------------------------------------------------------------ state

@bp.route("/joints", methods=["GET"])
def get_joints():
    """Current joint state: the 6 arm joints with position (rad, deg), velocity (rad/s), effort (the driver's
    joint currents/torques), the limits and the distance to the nearest limit (closer than joint_limit_margin_rad
    -> motion endpoints refuse with joint_near_limit). "positions" is the plain 6-list in radians. Also the RG6
    driving finger joint (from the gripper model's width table) and gripper_width_m (+ whether it was measured). Lock-free;
    503 until /joint_states arrives."""
    js = C.ros.joint_state(C.cell.arm_joints)
    if js is None:
        return jsonify({"error": "no_joint_state", "msg": "no /joint_states for the arm received yet"}), 503
    joints = []
    for i, n in enumerate(C.cell.arm_joints):
        q, v, e, rx, st = js[n]
        lo, hi = C.cell.limits()[i]
        m = min(q - lo, hi - q)
        joints.append({"name": n, "index": i + 1, "position_rad": q, "position_deg": math.degrees(q),
                       "velocity_rad_s": v, "effort_nm": e, "limit_lo_rad": lo, "limit_hi_rad": hi,
                       "to_limit_rad": m, "to_limit_deg": math.degrees(m)})
    g = C.gripper.state()
    w = g["width_m"]
    newest = max(js[n][3] for n in C.cell.arm_joints)
    return jsonify({
        "positions": [j["position_rad"] for j in joints], "positions_deg": [j["position_deg"] for j in joints],
        "joints": joints,
        "fingers": {} if w is None else C.cell.finger_q(w),
        "gripper_width_m": w, "gripper_width_measured": g["width_measured"],
        "near_limit": [j["name"] for j in joints if j["to_limit_rad"] < C.cell.limit_margin],
        "stamp": max(js[n][4] for n in C.cell.arm_joints), "age_s": round(time.time() - newest, 3)}), 200


@bp.route("/state", methods=["GET"])
def get_state():
    """Current TCP pose (fingertip centre, metres, base_link) + orientation quaternion + rpy_deg (roll/pitch/yaw
    in degrees, sxyz -- the convention the motion endpoints take) + gripper (1 = open, 0 = closed) + joints (6,
    rad). ur_base gives the same pose in the pendant / RTDE "base" frame (base_link turned 180 deg about z).
    GET /joints for velocity, effort and limits."""
    q = C.ros.arm_q()
    if q is None:
        return jsonify({"error": "no_joint_state", "msg": "no /joint_states for the arm received yet"}), 503
    T = C.kin.fk(q)
    p = geom.T_to_pose(T)
    r, pt, y = geom.matrix_to_euler(T)
    Tb = np.linalg.inv(geom.T_BASELINK_BASE) @ T
    w = C.gripper.state()["width_m"]
    return jsonify({
        "position": dict(zip("xyz", p["position"])), "orientation": dict(zip("xyzw", p["orientation"])),
        "gripper": 1 if (w is not None and w > C.cell.open_threshold) else 0, "gripper_width_m": w,
        "joints": list(q), "rpy_deg": {"roll": math.degrees(r), "pitch": math.degrees(pt), "yaw": math.degrees(y)},
        "frame_id": C.cell.base_frame, "tcp_link": C.cell.tcp_link,
        "ur_base": {"position": [float(v) for v in Tb[:3, 3]], "orientation": geom.matrix_to_quat(Tb),
                    "note": "the pendant/RTDE 'base' frame; its x and y are base_link's negated"}}), 200


# ------------------------------------------------------------------ gripper

def _gripper_call(width, force, closing, timeout_s, auto_recover, max_retries):
    attempts, rec_trig, rec_ok, res = 0, False, False, None
    while attempts <= max_retries:
        attempts += 1
        res = C.gripper.move(width, force, closing=closing, timeout_s=timeout_s)
        if res["outcome"] in ("success", "stopped", "not_supported"):
            break
        if not auto_recover or attempts > max_retries:
            break
        rec_trig = True
        rec_ok, why = C.motion.recover()
        if not rec_ok:
            res["outcome"] = "recovery_failed"
            res["error"] = f"{res.get('error')}; recovery failed: {why}"
            break
    return res, attempts, rec_trig, rec_ok


def _gripper_body(res, attempts, rec_trig, rec_ok, **extra):
    out = {"outcome": res["outcome"], "success": res["outcome"] == "success", "attempts": attempts,
           "recovery_triggered": rec_trig, "recovery_succeeded": rec_ok if rec_trig else None,
           "error_msg": res.get("error"), "gripper_width_m": res.get("width_m"),
           "width_measured": res.get("width_measured", False), "program": res.get("program"),
           "control_restored": res.get("control_restored"), "elapsed_s": res.get("elapsed_s"),
           "notes": res.get("notes", [])}
    out.update(extra)
    return jsonify(out), OUTCOME_STATUS.get(res["outcome"], 500)


def _gripper_args(defaults):
    a = request.args
    try:
        vals = {k: float(a.get(k, d)) for k, d in defaults.items()}
        vals["auto_recover"] = _flag(a.get("auto_recover"))
        vals["max_retries"] = int(a.get("max_retries", "1"))
    except (TypeError, ValueError):
        raise err(400, "bad_request", f"bad gripper params (numbers: {', '.join(defaults)})")
    return vals


@bp.route("/control/gripper_open_force", methods=["GET"])
@with_lock
def gripper_open_force():
    """Open the RG6 to width=<m> (default 0.08): writes width/force into the RTDE registers and plays the open
    program, then hands control back to the driver. Params width, speed (ignored: the RG6 sets its own speed),
    timeout (s, default the config's), auto_recover=1, max_retries=1 (0 while holding an object in transit).
    Outcomes: success, action_timeout (504), action_failed (500), recovery_failed (500), server_unreachable (503)."""
    v = _gripper_args({"width": 0.08, "speed": 0.1, "timeout": C.cell.gripper.get("program_timeout_s", 10.0)})
    res, n, rt, ro = _gripper_call(v["width"], None, False, v["timeout"], v["auto_recover"], v["max_retries"])
    return _gripper_body(res, n, rt, ro, final_width_cmd=min(max(v["width"], 0.0), C.cell.max_width))


@bp.route("/control/gripper_grasp", methods=["GET"])
@with_lock
def gripper_grasp():
    """Force grasp: close the RG6 with force=<N> (RG6: 25..120 N, clamped) towards width=<m> (default 0.0 =
    close until contact). The program is sent (width - eps_in - close_squeeze_m), so the force limit, not the
    position, ends the close. Success (Franka semantics) = final width within [width - eps_in, width + eps_out]
    when the gripper reports its width (gripper.width_feedback_register); without that feedback the program
    finishing counts as success and width_measured=false. Params width, speed (ignored), force=20, eps_in=0.005,
    eps_out=0.005, timeout, auto_recover=1, max_retries=1 (use 0 while transporting a held object)."""
    v = _gripper_args({"width": 0.0, "speed": 0.05, "force": 20.0, "eps_in": 0.005, "eps_out": 0.005,
                       "timeout": C.cell.gripper.get("program_timeout_s", 10.0)})
    cmd = max(0.0, v["width"] - v["eps_in"] - float(C.cell.gripper.get("close_squeeze_m", 0.010)))
    res, n, rt, ro = _gripper_call(cmd, v["force"], True, v["timeout"], v["auto_recover"], v["max_retries"])
    if res["outcome"] == "success" and res.get("width_measured"):
        w = res["width_m"]
        if not (v["width"] - v["eps_in"] <= w <= v["width"] + v["eps_out"]):
            res["outcome"] = "action_failed"
            res["error"] = (f"grasp ended at {w * 1000:.1f} mm, outside [{(v['width'] - v['eps_in']) * 1000:.1f}, "
                            f"{(v['width'] + v['eps_out']) * 1000:.1f}] mm (widen eps_out for unknown widths)")
    force = min(max(v["force"], C.gripper.min_force), C.gripper.max_force)
    return _gripper_body(res, n, rt, ro, used_width_cmd=v["width"], program_width_cmd=cmd, force=force)


def _simple_move(width):
    res = C.gripper.move(width, None, closing=width < (C.gripper.state()["width_m"] or C.cell.max_width))
    return jsonify({"result": res["outcome"] == "success", "outcome": res["outcome"], "width_m": res.get("width_m"),
                    "error": res.get("error")}), 200 if res["outcome"] == "success" else 500


@bp.route("/control/go_to_gripper_state", methods=["GET"])
@with_lock
def go_to_gripper_state():
    """Move the RG6 to width=<m> (0..0.160) with the default force. No grasp check."""
    try:
        w = float(request.args.get("width"))
    except (TypeError, ValueError):
        return jsonify({"error": "No width arg"}), 400
    return _simple_move(w)


@bp.route("/control/gripper_open", methods=["GET"])
@with_lock
def gripper_open():
    """Open the RG6 fully (its 160 mm stroke). No force feedback."""
    return _simple_move(C.cell.max_width)


@bp.route("/control/gripper_close", methods=["GET"])
@with_lock
def gripper_close():
    """Close the RG6 fully with the default force. No grasp check."""
    return _simple_move(0.0)


@bp.route("/control/gripper_homing", methods=["GET"])
@with_lock
def gripper_homing():
    """The RG6 needs no homing (the Franka hand does); kept for API parity: opens the gripper fully and reports
    the width. Moves the FINGERS only."""
    res = C.gripper.move(C.cell.max_width, None, closing=False)
    ok = res["outcome"] == "success"
    return jsonify({"status": "homed" if ok else "failed", "success": ok, "error_msg": res.get("error"),
                    "gripper_width_m": res.get("width_m"),
                    "note": "RG6: no homing needed; this opened the gripper fully"}), 200 if ok else 500


# ------------------------------------------------------------------ motion

def _remember(**kw):
    C.last_motion = kw


def _err_counter():
    base = C.ros.error_count()
    return lambda: C.ros.last_error(base)


def do_cartesian(m, max_travel, velocity=0.05):
    rej = preflight()
    if rej:
        return rej
    pose = target_pose(m)
    fraction = {"v": None}

    def plan_fn():
        traj, frac, code = C.motion.plan_cartesian([pose], vel=velocity, acc=velocity)
        fraction["v"] = frac
        return traj, frac >= 0.999 and code == 1, {"fraction": frac}

    out = C.motion.plan_and_execute(plan_fn, m["auto_recover"], m["max_retries"], max_travel)
    extra = {"fraction": str(fraction["v"]), "target_pose": pose}
    if out["outcome"] == "plan_failed":
        extra["moveit_error"] = (f"PARTIAL_PATH (fraction={fraction['v']}, likely unreachable target, a collision on "
                                 "the line, or a joint limit)")
    return motion_response(out, extra)


@bp.route("/control/plan_cartesian_path", methods=["GET"])
@with_lock
def plan_cartesian_path():
    """Straight-line (MoveL) TCP move to ABSOLUTE x,y,z (metres; frame=base_link default, or frame=base for the
    pendant frame). Orientation: roll&pitch&yaw (degrees, sxyz; straight down = 180,0,0) or qx&qy&qz&qw;
    else preserve_orientation=1 keeps the current one, 0 resets to straight down. velocity=0.05 (scaling),
    auto_recover=1, max_retries=1, max_joint_travel_deg=100 (big-swing guard, wrist_3 exempt; 0 disables).
    WARNING: moves the arm."""
    try:
        m = parse_motion_args()
        mt = parse_max_travel()
        vel = float(request.args.get("velocity", "0.05"))
        if not 0.01 <= vel <= 0.5:
            raise ValueError("velocity must be in 0.01..0.5")
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad motion args", "msg": str(e)}), 400
    _remember(kind="cartesian", m=m, max_travel=mt, velocity=vel)
    return do_cartesian(m, mt, vel)


def do_joint_path(m, planner_id, max_travel):
    rej = preflight()
    if rej:
        return rej
    pose = target_pose(m)

    def plan_fn():
        ok, traj, t, code, meta = C.motion.plan_pose(pose, planner_id, m["preserve_orientation"]
                                                     and m["orientation"] is None)
        return traj, ok, {"planning_time": t, "error_code": code, "planner_id": planner_id, **meta}

    out = C.motion.plan_and_execute(plan_fn, m["auto_recover"], m["max_retries"], max_travel)
    meta = out.get("plan_metadata") or {}
    extra = {"planning_time": str(meta.get("planning_time")), "planner_id": planner_id, "target_pose": pose}
    if out["outcome"] == "plan_failed":
        extra["moveit_error"] = decode_error(meta.get("error_code"))
    return motion_response(out, extra)


@bp.route("/control/plan_joint_path", methods=["GET"])
@with_lock
def plan_joint_path():
    """Joint-space plan + execute of the TCP to x,y,z (same params as plan_cartesian_path) with planner_id=LIN
    (Pilz straight line, default) or RRTConnect (the target is first solved by IK for the configuration
    nearest the current joints, so the arm keeps its configuration; the least-travel of 3 plans runs).
    WARNING: moves the arm."""
    try:
        m = parse_motion_args()
        pid = request.args.get("planner_id", "LIN")
        if pid in ("RRTConnectkConfigDefault",):
            pid = "RRTConnect"
        if pid not in ("LIN", "RRTConnect"):
            raise ValueError("planner_id must be LIN or RRTConnect")
        mt = parse_max_travel()
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad motion args", "msg": str(e)}), 400
    _remember(kind="joint", m=m, planner_id=pid, max_travel=mt)
    return do_joint_path(m, pid, mt)


def parse_goto(current):
    data = request.get_json(silent=True) if request.method == "POST" else None
    data = data if isinstance(data, dict) else {}
    get = lambda k, d=None: data.get(k, request.args.get(k, d))  # noqa: E731
    unit = str(get("unit") or "rad").lower()
    if unit not in ("rad", "deg"):
        raise ValueError("unit must be rad or deg")
    conv = math.radians if unit == "deg" else float
    n = C.cell.n_joints
    target, given = list(current), False
    raw = get("joints")
    if raw is not None:
        vals = raw if isinstance(raw, list) else [v for v in str(raw).replace("[", "").replace("]", "").split(",")
                                                  if v.strip()]
        if len(vals) != n:
            raise ValueError(f"joints needs exactly {n} values, got {len(vals)}")
        target, given = [conv(float(v)) for v in vals], True
    for i in range(1, n + 1):
        v = get(f"j{i}")
        if v is not None:
            target[i - 1], given = conv(float(v)), True
    if not given:
        raise ValueError(f"give joints=<{n} values> and/or j1..j{n}=<value>")
    if not all(math.isfinite(v) for v in target):
        raise ValueError("joint values must be finite")
    velocity = float(get("velocity", "0.2"))
    if not 0.01 <= velocity <= 0.5:
        raise ValueError("velocity must be in 0.01..0.5")
    mjt = get("max_joint_travel_deg")
    mt = None if mjt is None else math.radians(float(mjt))
    if mt is not None and mt < 0:
        raise ValueError("max_joint_travel_deg must be >= 0")
    return {"target": target, "unit": unit, "velocity": velocity, "max_travel": mt,
            "auto_recover": _flag(get("auto_recover")), "max_retries": int(get("max_retries", "1")),
            "dry_run": _flag(get("dry_run", "0"), False)}


def do_goto(g):
    target = g["target"]
    bad = C.cell.near_limits(target)
    if bad:      # a goal ON a limit plans fine and then never executes
        return jsonify({"error": "target_near_limit", "msg": f"target puts a joint within {C.cell.limit_margin} rad "
                        "of its limit", "joints_near_limit": bad}), 400
    try:
        valid, contacts = C.motion.check_state(target)
    except ApiError as e:
        return jsonify({"error": "validity_check_unavailable", "msg": e.msg}), 503
    if not valid:
        return jsonify({"error": "target_in_collision", "msg": "the target configuration collides with the robot "
                        "itself or the planning scene", "contacts": C.motion.contact_pairs(contacts)}), 400
    if not g["dry_run"]:
        rej = preflight()
        if rej:
            return rej

    def plan_fn():
        ok, traj, t, code, planner = C.motion.plan_joint_goal(target, vel=g["velocity"], acc=0.1)
        return traj, ok, {"planning_time": t, "error_code": code, "planner_used": planner}

    if g["dry_run"]:
        traj, ok, meta = plan_fn()
        travel = joint_travel(traj) if ok else {}
        limit = C.cell.max_travel_rad if g["max_travel"] is None else g["max_travel"]
        pts = traj.joint_trajectory.points if ok else []
        body = {"outcome": "planned" if ok else "plan_failed", "dry_run": True, "planner_used": meta["planner_used"],
                "planning_time": meta["planning_time"], "waypoints": len(pts),
                "duration_s": (pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9) if pts else None,
                "joint_travel_deg": {j: round(math.degrees(t), 1) for j, t in travel.items()},
                "would_be_rejected_as_too_large": [j for j, t in travel.items() if limit and t > limit
                                                   and j not in C.cell.travel_exempt],
                "target_rad": target}
        if not ok:
            body["moveit_error"] = decode_error(meta["error_code"])
        return jsonify(body), 200 if ok else 202
    out = C.motion.plan_and_execute(plan_fn, g["auto_recover"], g["max_retries"], g["max_travel"])
    meta = out.get("plan_metadata") or {}
    extra = {"planner_used": meta.get("planner_used"), "planning_time": str(meta.get("planning_time")),
             "target_rad": target}
    if out["outcome"] == "plan_failed":
        extra["moveit_error"] = decode_error(meta.get("error_code"))
    return motion_response(out, extra)


@bp.route("/control/goto_joints", methods=["GET", "POST"])
def goto_joints():
    """Move to an explicit joint configuration: joints=<6 values> (shoulder_pan .. wrist_3) and/or j1..j6=<value>
    to change single joints from the current pose; unit=rad|deg (default rad); POST JSON {"joints": [...],
    "unit": "deg"} too. Pilz PTP (straight in joint space, every joint monotonic), OMPL RRTConnect as fallback.
    velocity=0.2 (0.01..0.5 scaling of the joint speed limits). dry_run=1 plans only and reports planner,
    duration and per-joint travel. Target must be > joint_limit_margin_rad inside the limits (400
    target_near_limit) and collision-free (400 target_in_collision); big-swing guard max_joint_travel_deg=100
    except wrist_3 (409 plan_too_large; 0 disables); auto_recover=1, max_retries=1; /control/stop and
    /control/resume work. WARNING: moves the arm (unless dry_run=1)."""
    try:
        g = parse_goto(C.motion.current_q())
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad goto_joints args", "msg": str(e)}), 400
    except ApiError as e:
        return jsonify({"error": e.code, "msg": e.msg}), e.status
    if not C.lock.acquire(blocking=False):
        return busy()
    try:
        if not g["dry_run"]:
            _remember(kind="joints", g=g)
        return do_goto(g)
    finally:
        C.lock.release()


@bp.route("/control/resume", methods=["GET"])
@with_lock
def resume():
    """Re-issue the last requested motion (same kind, target and settings) from where the arm is now, e.g. after
    a stopped / stopped_external_force outcome. Runs /recover first (unlock a protective stop, give control
    back to the driver)."""
    lm = C.last_motion
    if lm is None:
        return jsonify({"error": "no_motion_to_resume", "msg": "no prior motion request has been issued since "
                        "server start"}), 400
    ok, why = C.motion.recover()
    if not ok:
        return jsonify({"error": "recovery_failed", "msg": why}), 409
    if lm["kind"] == "cartesian":
        return do_cartesian(lm["m"], lm["max_travel"], lm["velocity"])
    if lm["kind"] == "joint":
        return do_joint_path(lm["m"], lm["planner_id"], lm["max_travel"])
    return do_goto(dict(lm["g"], dry_run=False))


@bp.route("/control/nudge_joint", methods=["GET"])
@with_lock
def nudge_joint():
    """Minimal-motion recovery for one joint: joint=1..6, delta=<rad>. Skips the near-limit preflight (this IS
    the tool for a joint stuck near its limit); the goal is clamped 0.01 rad inside the limits."""
    try:
        j, d = int(request.args.get("joint")), float(request.args.get("delta"))
    except (TypeError, ValueError):
        return jsonify({"error": f"missing or bad 'joint' (1..{C.cell.n_joints}) or 'delta' (rad) param"}), 400
    if not 1 <= j <= C.cell.n_joints:
        return jsonify({"error": f"joint must be 1..{C.cell.n_joints}"}), 400
    q = list(C.motion.current_q())
    lo, hi = C.cell.limits()[j - 1]
    q[j - 1] = max(lo + 0.01, min(hi - 0.01, q[j - 1] + d))
    C.motion.clear_stop()
    r = C.motion.move_joints(q, vel=0.15, acc=0.05)
    return jsonify({"status": "nudged" if r["executed"] else "failed", **r, "target_joint": j,
                    "target_value_rad": round(q[j - 1], 4), "last_error": last_error()}), 200 if r["executed"] else 500


@bp.route("/control/home", methods=["GET"])
@with_lock
def home():
    """Joint move to the cell's home pose (cell.yaml robot.home_joints_rad: the recorded eye-in-hand home).
    PTP, RRTConnect as fallback, velocity scaling 0.3. Not subject to the big-swing guard. WARNING: moves the arm."""
    C.motion.clear_stop()
    r = C.motion.move_joints(C.cell.home_q, vel=0.3, acc=0.1)
    if r["executed"]:
        return jsonify({"status": "homed", "msg": "arm moved to the home joint pose", "planner_used": r["planner_used"],
                        "planning_time": str(r["planning_time"]), "home_rad": C.cell.home_q}), 200
    return jsonify({"status": "failed", "msg": "home motion failed", "planner_tried": ["PTP", "RRTConnect"],
                    "plan_error_codes": r.get("plan_error_codes"), "last_error": last_error()}), 500


@bp.route("/control/stop", methods=["GET"])
def stop():
    """Stop any ongoing motion (cancels the running trajectory; the arm decelerates) and any running gripper
    program, and suppress the auto-recovery retry: the in-flight request returns outcome=stopped. Lock-free."""
    n = C.motion.stop()
    C.gripper.stop()
    return jsonify({"status": "stopped", "msg": "Robot stopped", "trajectories_cancelled": n}), 200


@bp.route("/recover", methods=["GET"])
def recover():
    """Clear what software may clear: unlock a protective stop (the controller refuses this in the first 5 s),
    close safety popups, and resend the driver's program when it is not running. Emergency / safeguard stops
    need the hardware. 504 if it did not work within 15 s."""
    ok, why = C.motion.recover()
    if not ok:
        return jsonify({"status": "timeout", "error": why or "recovery did not complete within timeout",
                        "safety_mode": SAFETY_NAMES.get(C.motion.safety_mode()),
                        "robot_mode": ROBOT_MODE_NAMES.get(C.motion.robot_mode()),
                        "program_running": C.motion.program_running()}), 504
    return jsonify({"status": "recovered", "msg": "Robot recovered"}), 200


@bp.route("/health", methods=["GET"])
def health():
    """Reachability of move_group, the trajectory action, the driver program, the dashboard and the gripper; the
    motion-lock state; robot / safety mode; the last ROS error. Use when the API feels stuck."""
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.srv import GetMotionPlan
    js = C.ros.joint_state(C.cell.arm_joints)
    move_server = C.ros.action(ExecuteTrajectory, "/execute_trajectory").wait_for_server(timeout_sec=0.1)
    dash_ok = None
    if C.dash is not None:
        try:
            C.dash.running()
            dash_ok = True
        except Exception:      # noqa: BLE001
            dash_ok = False
    return jsonify({
        "move_server": bool(move_server), "grasp_server": C.gripper.available(), "gripper_backend": C.gripper.backend,
        "gripper_model": C.cell.gripper_model, "gripper_name": C.cell.gripper["name"],
        "planner": C.ros.service_ready(GetMotionPlan, "/plan_kinematic_path"),
        "lock_busy": C.lock.locked(), "last_error": last_error(),
        "joint_state_age_s": None if js is None else round(time.time() - max(v[3] for v in js.values()), 3),
        "driver_program_running": C.motion.program_running(), "dashboard": dash_ok, "mock_hardware": C.mock,
        "robot_mode": ROBOT_MODE_NAMES.get(C.motion.robot_mode()),
        "safety_mode": SAFETY_NAMES.get(C.motion.safety_mode()),
    }), 200


# ------------------------------------------------------------------ demo box (/simulation/*)

_BOX = "box"


@bp.route("/simulation/add_box", methods=["GET"])
def add_box():
    """Add the demo box (75 mm cube) between the fingertips, in the TCP frame."""
    obj = {"id": _BOX, "frame_id": C.cell.tcp_link,
           "pose": {"position": [0.0, 0.0, -0.04], "orientation": [0.0, 0.0, 0.0, 1.0]},
           "shapes": [{"type": "box", "size": [0.075, 0.075, 0.075],
                       "pose": {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}}]}
    C.scene_io.apply([("add", _BOX, obj)], C.core.mesh_cache)
    return jsonify({"status": "success", "msg": "box added on the tool"}), 200


@bp.route("/simulation/remove_box", methods=["GET"])
def remove_box():
    """Remove the demo box (never built-in geometry)."""
    C.scene_io.apply([("remove", _BOX, None)], C.core.mesh_cache)
    return jsonify({"status": "success", "msg": "box removed"}), 200


@bp.route("/simulation/attach_box", methods=["GET"])
def attach_box():
    """Attach the demo box to the hand (collision-checked as part of it)."""
    C.scene_io.apply([("attach", _BOX, {"link": C.cell.hand_link, "touch_links": C.cell.touch_links(C.cell.hand_link)})],
                     C.core.mesh_cache)
    return jsonify({"status": "success", "msg": "box attached on the tool"}), 200


@bp.route("/simulation/detach_box", methods=["GET"])
def detach_box():
    """Detach the demo box (it stays in the world where the hand is)."""
    C.scene_io.apply([("detach", _BOX, {"link": C.cell.hand_link})], C.core.mesh_cache)
    return jsonify({"status": "success", "msg": "box detached"}), 200


# ------------------------------------------------------------------ force

@bp.route("/force", methods=["GET"])
def force():
    """Current end-effector wrench from the UR e-series flange F/T sensor: raw / net force and torque (net = after
    tare or compensation), magnitudes, lever arm (|net torque| / |net force|, ~the CoM offset of a held object),
    frame. Lock-free. 503 until the first wrench arrives."""
    d = C.force.get_force()
    return jsonify(d), 200 if d.get("available") else 503


@bp.route("/force/tare", methods=["GET"])
def force_tare():
    """Snapshot the current raw wrench as the baseline (tare with an empty gripper, then grasp to see the weight)."""
    b = C.force.tare()
    if b is None:
        return jsonify({"status": "no_data", "msg": "no wrench received yet; cannot tare"}), 503
    return jsonify({"status": "tared", "baseline": b}), 200


@bp.route("/force/untare", methods=["GET"])
def force_untare():
    """Clear the software baseline (raw values again)."""
    C.force.untare()
    return jsonify({"status": "untared", "msg": "baseline cleared"}), 200


@bp.route("/force/zero_sensor", methods=["GET"])
def force_zero_sensor():
    """Zero the F/T sensor itself (the driver's zero_ftsensor; UR e-series only). Do it with the gripper empty and
    the arm still. Not in the Franka API (its F_ext cannot be zeroed)."""
    from std_srvs.srv import Trigger
    r = C.ros.call(Trigger, "/io_and_status_controller/zero_ftsensor", Trigger.Request(), timeout_s=5.0)
    return jsonify({"status": "zeroed" if r.success else "failed", "msg": r.message}), 200 if r.success else 500


@bp.route("/force/config", methods=["GET"])
def force_config():
    """RViz arrows: scale (m/N, 0.02), threshold (N, 0.5), max_force (20), torque_scale (m/(N m), 0.15),
    torque_threshold (0.05), max_torque (2.0). MarkerArray on /ur_ee_force."""
    try:
        cfg = C.force.set_config(**{k: request.args.get(k) for k in ("scale", "threshold", "max_force", "torque_scale",
                                                                     "torque_threshold", "max_torque")})
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad params", "msg": str(e)}), 400
    return jsonify({"status": "updated", "config": cfg}), 200


@bp.route("/force/identify/empty", methods=["GET"])
@with_lock
def identify_empty():
    """Empty-gripper pass of the payload identification: captures the current joints as base, visits 5 wrist
    poses, samples the wrench at each, returns to base. Also installs the per-pose residuals as the compensation
    table. WARNING: moves the wrist. Then grasp the object and call /force/identify/loaded."""
    data = C.payload.record_empty()
    n = C.force.set_calibration(C.payload.calibration())
    return jsonify({"status": "empty_recorded", "calibration_points": n, **data}), 200


@bp.route("/force/identify/loaded", methods=["GET"])
@with_lock
def identify_loaded():
    """Loaded pass: the SAME poses as the empty pass, holding the object. WARNING: moves the wrist."""
    try:
        return jsonify({"status": "loaded_recorded", **C.payload.record_loaded()}), 200
    except ValueError as e:
        return jsonify({"error": "sequence", "msg": str(e)}), 409


@bp.route("/force/identify/compute", methods=["GET"])
def identify_compute():
    """Mass (median |dF| / g) and a rough CoM offset (least squares dtau = r x dF) from the two passes."""
    try:
        return jsonify({"status": "ok", **C.payload.compute()}), 200
    except ValueError as e:
        return jsonify({"error": "incomplete", "msg": str(e)}), 409


@bp.route("/force/identify/reset", methods=["GET"])
def identify_reset():
    """Discard the recorded passes."""
    C.payload.reset()
    return jsonify({"status": "reset"}), 200


@bp.route("/force/compensate", methods=["GET"])
def force_compensate():
    """on=1/0: subtract the residual of the nearest calibrated pose (from /force/identify/empty) instead of the
    tare baseline, so net_force is the true external force at the current pose."""
    on = _flag(request.args.get("on"))
    if not C.force.set_compensation(on):
        return jsonify({"error": "no_calibration", "msg": "run /force/identify/empty first to record an "
                        "empty-gripper calibration"}), 409
    return jsonify({"status": "compensation_on" if on else "compensation_off", "compensated": on}), 200


# ------------------------------------------------------------------ robot model facts (/robot/*)

@bp.route("/robot/tf", methods=["GET"])
def robot_tf():
    """Pose of link `to` in link `from` (robot links, default base_link -> tcp) from the URDF FK, for the
    current joints or joints=q1,...,q6 (rad). {pose:{position, orientation:[qx,qy,qz,qw]}, rpy_deg, from, to,
    joints}. Read-only."""
    src, dst = request.args.get("from", C.cell.base_frame), request.args.get("to", C.cell.tcp_link)
    links = set(C.kin.urdf.links)
    for name, v in (("from", src), ("to", dst)):
        if v not in links:
            raise err(404, "unknown_object", f"{name}={v!r} is not a robot link")
    raw = request.args.get("joints")
    if raw is not None:
        try:
            q = [float(v) for v in raw.split(",")]
        except ValueError:
            raise err(400, "bad_request", f"joints must be {C.cell.n_joints} comma-separated numbers (rad)")
        if len(q) != C.cell.n_joints or not all(math.isfinite(v) for v in q):
            raise err(400, "bad_request", f"joints must be {C.cell.n_joints} finite numbers (rad)")
    else:
        q = C.motion.current_q()
    w = C.gripper.state()["width_m"] or 0.0
    extra = C.cell.finger_q(w)
    root = C.cell.planning_frame
    T = np.linalg.inv(C.kin.urdf.fk(src, C.kin.qdict(q, extra), root)) @ C.kin.urdf.fk(dst, C.kin.qdict(q, extra), root)
    r, p, y = geom.matrix_to_euler(T)
    return jsonify({"from": src, "to": dst, "pose": geom.T_to_pose(T),
                    "rpy_deg": [math.degrees(r), math.degrees(p), math.degrees(y)], "joints": list(q),
                    "stamp": time.time()}), 200


@bp.route("/robot/description", methods=["GET"])
def robot_description():
    """URDF and SRDF as move_group uses them (+ sha256), arm joint limits, planning frame, groups, the TCP link,
    the link names -- for offline FK/IK and frame checks."""
    urdf, srdf = C.ros.urdf or "", C.ros.srdf or ""
    return jsonify({"urdf": urdf, "srdf": srdf, "urdf_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
                    "srdf_sha256": hashlib.sha256(srdf.encode()).hexdigest(),
                    "joint_limits": {n: {"lower": lo, "upper": hi} for n, (lo, hi) in zip(C.cell.arm_joints,
                                                                                         C.cell.limits())},
                    "planning_frame": C.cell.planning_frame, "base_frame": C.cell.base_frame,
                    "groups": {"arm": C.cell.group, "hand": "gripper"}, "ee_link": C.cell.tcp_link,
                    "flange_link": C.cell.flange_link, "links": C.kin.urdf.links}), 200


__all__ = ["bp", "with_lock", "traj_points"]
