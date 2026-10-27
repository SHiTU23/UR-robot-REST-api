"""/plan/* -- plan-only checks. Nothing here moves the robot or the gripper (safety rule S1): only move_group's
stateless services (/check_state_validity, /plan_kinematic_path, /compute_cartesian_path) and the Python FK/IK
are used, never an execution. No motion lock, so they can run while the arm moves (dry_run_sequence is the
exception: it changes the scene temporarily and holds the lock for that)."""
import copy
import json
import math
import threading
import time

import numpy as np
from flask import Blueprint, jsonify, request

import geom
from ctx import C
from errors import err
from motion import PLANNERS, decode_error, joint_travel, traj_points
from scene_core import canon_pose

bp = Blueprint("plan", __name__)
PLAN_TTL_S = 600
_plans, _plans_lock, _seq = {}, threading.Lock(), [0]


def _body():
    raw = request.stream.read(2 ** 20 + 1)
    if len(raw) > 2 ** 20:
        raise err(413, "too_large", "body exceeds 1 MB")
    try:
        d = json.loads(raw.decode("utf-8") or "{}")
    except (UnicodeDecodeError, ValueError) as e:
        raise err(400, "bad_request", f"body is not valid JSON: {e}")
    if not isinstance(d, dict):
        raise err(400, "bad_request", "body must be a JSON object")
    return d


def _check_version(d):
    exp = d.get("expect_scene_version")
    if exp is not None and exp != C.core.version:
        raise err(409, "version_conflict", f"scene_version is {C.core.version}, request expects {exp}",
                  scene_version=C.core.version)


def _joints(v, path):
    n = C.cell.n_joints
    if not isinstance(v, list) or len(v) != n:
        raise err(400, "bad_request", f"joints must be a list of {n} numbers (rad)", path)
    out = []
    for i, x in enumerate(v):
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
            raise err(400, "bad_request", "joint values must be finite numbers (rad)", f"{path}[{i}]")
        out.append(float(x))
    return out


def _width(d, key="gripper_width_m"):
    w = d.get(key)
    if w is None:
        return None
    if isinstance(w, bool) or not isinstance(w, (int, float)) or not 0.0 <= w <= C.cell.max_width:
        raise err(400, "bad_request", f"gripper_width_m is the total opening, 0..{C.cell.max_width} m", key)
    return float(w)


def _frame_pose(pd, path):
    frame = pd.get("frame_id") or C.cell.base_frame
    pose = canon_pose({k: v for k, v in pd.items() if k != "frame_id"}, path)
    if frame == "base":
        return geom.T_to_pose(geom.T_BASELINK_BASE @ geom.pose_to_T(pose))
    if frame not in (C.cell.base_frame, C.cell.planning_frame):
        raise err(400, "bad_frame", f"pose.frame_id must be {C.cell.base_frame} (or base, the pendant frame)",
                  f"{path}.frame_id")
    return pose


def _state(q, width, attached=()):
    if q is None:
        q = C.motion.current_q()
    return C.motion.robot_state(q, width, list(attached)), list(q)


def _valid(st):
    return C.motion.check_state(state=st)


def _set_q(st, q):
    st = copy.deepcopy(st)
    names, pos = list(st.joint_state.name), list(st.joint_state.position)
    for n, v in zip(C.cell.arm_joints, q):
        if n in names:
            pos[names.index(n)] = float(v)
        else:
            names.append(n)
            pos.append(float(v))
    st.joint_state.name, st.joint_state.position = names, pos
    return st


def interp_check(points, base_state, max_step):
    """Densify the joint path to max_step (rad) and collision-check every sample. move_group itself only checks
    the planner's waypoints. -> (valid, first bad segment, contacts, states checked)."""
    if len(points) == 1:
        v, c = _valid(_set_q(base_state, points[0]))
        return v, (None if v else 0), c, 1
    checked = 0
    for k in range(len(points) - 1):
        a, b = np.array(points[k], float), np.array(points[k + 1], float)
        n = max(1, int(math.ceil(float(np.max(np.abs(b - a))) / max_step)))
        for j in range(n + 1 if k == len(points) - 2 else n):
            v, c = _valid(_set_q(base_state, a + (b - a) * (j / n)))
            checked += 1
            if not v:
                return False, k, c, checked
    return True, None, [], checked


# ------------------------------------------------------------------ check_state

@bp.route("/plan/check_state", methods=["POST"])
def check_state():
    """Collision-check one configuration against the current scene, without moving: joints:[6] (rad) or
    use_current:true, gripper_width_m (total opening, 0..max width of the gripper model), expect_scene_version. -> valid, contacts
    [{a, b, depth_m, point}], joints, joint_limit_margin_ok, scene_version. No motion lock."""
    d = _body()
    _check_version(d)
    q = None if d.get("use_current") else _joints(d.get("joints"), "joints")
    st, q = _state(q, _width(d))
    v, c = _valid(st)
    return jsonify({"valid": v, "contacts": c, "joints": q, "joint_limit_margin_ok": not C.cell.near_limits(q),
                    "scene_version": C.core.version, "scene_dirty": C.core.dirty}), 200


# ------------------------------------------------------------------ dry_run

def _start_state(d, width, attached=()):
    start = d.get("start", "current")
    if start == "current":
        return _state(None, width, attached)
    if isinstance(start, dict):
        return _state(_joints(start.get("joints"), "start.joints"), width, attached)
    raise err(400, "bad_request", f'start must be "current" or {{joints:[{C.cell.n_joints}]}}', "start")


def plan_one(d, start_state, start_q, t0):
    """Plan one target from start_state (a diff RobotState); never executes. -> the response dict."""
    planner = d.get("planner", "RRTConnect")
    if planner not in PLANNERS:
        raise err(400, "bad_request", f"planner must be one of {list(PLANNERS)}", "planner")
    pipeline = PLANNERS[planner]
    vel, acc = float(d.get("velocity_scaling", 0.2)), float(d.get("acceleration_scaling", 0.05))
    if not (0.01 <= vel <= 1.0 and 0.01 <= acc <= 1.0):
        raise err(400, "bad_request", "velocity/acceleration_scaling must be in 0.01..1")
    t_alloc, attempts = float(d.get("allowed_planning_time_s", 5.0)), int(d.get("attempts", 3))
    max_step = float(d.get("max_step_rad", 0.01))
    if not 0.001 <= max_step <= 0.1:
        raise err(400, "bad_request", "max_step_rad must be in 0.001..0.1")
    target = d.get("target")
    if not isinstance(target, dict) or target.get("type") not in ("pose", "joints", "cartesian"):
        raise err(400, "bad_request", "target.type must be pose | joints | cartesian", "target.type")
    ttype, link = target["type"], target.get("link", C.cell.tcp_link)
    resp = {"planner": planner, "pipeline": pipeline, "scene_version": C.core.version, "target_type": ttype,
            "start_joints": start_q}
    sv, sc = _valid(start_state)
    if not sv:
        resp.update(success=False, error_code="START_STATE_IN_COLLISION", start_contacts=sc)
        return resp
    goal_q, pose = None, None
    if ttype == "joints":
        goal_q = _joints(target.get("joints"), "target.joints")
    elif ttype == "pose":
        pose = _frame_pose(target.get("pose") or {}, "target.pose")
        if planner != "LIN":
            if link != C.cell.tcp_link:
                raise err(400, "bad_request", f"pose targets for RRTConnect/PTP must use link {C.cell.tcp_link}")
            goal_q, n = C.motion.ik_near(pose, start_q, start_state, max_checks=40)
            if goal_q is None:
                resp.update(success=False, error_code="NO_IK_SOLUTION" if n == 0 else "GOAL_IN_COLLISION",
                            ik_solutions_found=n)
                return resp
    if goal_q is not None:
        resp["goal_joints"] = goal_q
        resp["would_be_rejected"] = {"target_near_limit": bool(C.cell.near_limits(goal_q))}
        gv, gc = _valid(_set_q(start_state, goal_q))
        if not gv:
            resp.update(success=False, error_code="GOAL_IN_COLLISION", goal_contacts=gc)
            return resp
    fraction = None
    if ttype == "cartesian":
        wps = target.get("waypoints")
        if not isinstance(wps, list) or not wps:
            raise err(400, "bad_request", "cartesian target needs waypoints:[pose...]", "target.waypoints")
        poses = [_frame_pose(w, f"target.waypoints[{i}]") for i, w in enumerate(wps)]
        traj, fraction, code = C.motion.plan_cartesian(poses, vel, acc, start_state,
                                                       float(target.get("eef_step_m", 0.005)), link)
        ok = code == 1 and fraction >= 0.999
        code_s = decode_error(code) if code != 1 else ("SUCCESS" if ok else "PARTIAL_PATH")
    else:
        goal = C.motion.joint_goal(goal_q) if goal_q is not None else C.motion.pose_goal(pose, link)
        ok, traj, _t, code = C.motion.plan(goal, pipeline, planner, vel, acc, start_state,
                                           1 if pipeline != "ompl" else max(1, attempts), t_alloc)
        code_s = "SUCCESS" if ok else decode_error(code)
    resp.update(success=ok, error_code=code_s)
    if fraction is not None:
        resp["fraction"] = fraction
    if not ok:
        resp["elapsed_ms"] = int((time.time() - t0) * 1000)
        return resp
    pts = traj_points(traj, C.cell.arm_joints)
    path = [q for _, q in pts]
    resp["trajectory"] = {"joint_names": C.cell.arm_joints, "points": [{"t": t, "q": q} for t, q in pts]}
    resp["duration_s"] = pts[-1][0] if pts else 0.0
    travel = joint_travel(traj)
    resp["joint_travel_deg"] = {j: round(math.degrees(t), 2) for j, t in travel.items() if j in C.cell.arm_joints}
    too_large = sorted(j for j, t in travel.items() if t > C.cell.max_travel_rad and j not in C.cell.travel_exempt)
    near = C.motion.traj_limit_violations(traj)
    wbr = resp.setdefault("would_be_rejected", {})
    wbr.update(near_limit=bool(near), too_large=bool(too_large), too_large_joints=too_large)
    if near:
        wbr["near_limit_first"] = near[0]
    if goal_q is None and path:
        wbr["target_near_limit"] = bool(C.cell.near_limits(path[-1]))
    resp["final_tcp_pose"] = geom.T_to_pose(C.kin.fk(path[-1])) if path else None
    v, bad, contacts, checked = interp_check(path, start_state, max_step)
    resp.update(trajectory_valid=v, contacts=contacts, interpolated_states_checked=checked)
    if not v:
        resp["first_invalid_segment"] = bad
    resp["end_joints"] = path[-1] if path else start_q
    resp["elapsed_ms"] = int((time.time() - t0) * 1000)
    return resp


def _remember(resp):
    with _plans_lock:
        now = time.time()
        for k in [k for k, (exp, _) in _plans.items() if exp < now]:
            _plans.pop(k)
        _seq[0] += 1
        pid = f"p-{_seq[0]:06d}"
        _plans[pid] = (now + PLAN_TTL_S, {"scene_version": C.core.version, "start_joints": resp["start_joints"],
                                          "points": [p["q"] for p in resp["trajectory"]["points"]],
                                          "planner": resp["planner"]})
    return pid


@bp.route("/plan/dry_run", methods=["POST"])
def dry_run():
    """Plan only, never executes. target {type: "pose", link (tcp), pose:{frame_id, position, orientation}} |
    {type: "joints", joints:[6]} | {type: "cartesian", waypoints:[pose...], eef_step_m (0.005)}; planner
    RRTConnect | PTP | LIN; start "current" | {joints:[6]}; gripper_width_m; velocity_scaling (0.2);
    acceleration_scaling (0.05); allowed_planning_time_s (5); attempts (3); max_step_rad (0.01, interpolated
    recheck); expect_scene_version. Pose targets for RRTConnect/PTP become the nearest collision-free IK
    configuration (as real motions). -> success, error_code, trajectory {joint_names, points [{t, q}]},
    duration_s, joint_travel_deg, final_tcp_pose, would_be_rejected, trajectory_valid + contacts, plan_id."""
    t0 = time.time()
    d = _body()
    _check_version(d)
    st, q = _start_state(d, _width(d))
    resp = plan_one(d, st, q, t0)
    if resp.get("success") and resp.get("trajectory"):
        resp["plan_id"] = _remember(resp)
    return jsonify(resp), 200


# ------------------------------------------------------------------ ik

@bp.route("/plan/ik", methods=["POST"])
def plan_ik():
    """IK + reachability, no motion: pose {frame_id, position, orientation} of the tcp, seeds [[6]] (default the
    current joints), n_solutions (4), avoid_collisions (true: collision-free first), gripper_width_m. Uses the
    closed-form UR solution (8 branches) refined on the URDF, so it returns DISTINCT configurations, each
    collision-checked: {solutions:[{q, valid, contacts, min_limit_margin_rad, cost}], reachable,
    analytic_solutions}, ordered by closeness to the first seed (base / elbow weighted, as real motions)."""
    d = _body()
    _check_version(d)
    if d.get("link", C.cell.tcp_link) != C.cell.tcp_link:
        raise err(400, "bad_request", f"only link {C.cell.tcp_link} is supported", "link")
    pose = _frame_pose(d.get("pose") or {}, "pose")
    n = int(d.get("n_solutions", 4))
    if not 1 <= n <= 32:
        raise err(400, "bad_request", "n_solutions must be 1..32", "n_solutions")
    base, q0 = _state(None, _width(d))
    seeds = [_joints(s, f"seeds[{i}]") for i, s in enumerate(d.get("seeds") or [q0])]
    T = geom.pose_to_T(pose)
    cands = []
    for s in seeds:
        cands += C.kin.solutions(T, s)
    w, s0 = np.array(C.cell.ik_weights), np.array(seeds[0])
    cands.sort(key=lambda c: float(np.max(w * np.abs(np.array(c[1]) - s0))))
    picked = []
    for _, q in cands:
        if all(np.max(np.abs(np.array(q) - np.array(p))) > 0.2 for p in picked):
            picked.append(q)
        if len(picked) >= max(n * 3, 12):
            break
    out = []
    for q in picked:
        v, c = _valid(_set_q(base, q))
        margin = min(min(a - lo, hi - a) for a, (lo, hi) in zip(q, C.cell.limits()))
        out.append({"q": q, "valid": v, "contacts": c, "min_limit_margin_rad": margin,
                    "cost": float(np.max(w * np.abs(np.array(q) - s0)))})
    if d.get("avoid_collisions", True):
        out.sort(key=lambda o: (not o["valid"], o["cost"]))
    out = out[:n]
    return jsonify({"solutions": out, "reachable": any(o["valid"] for o in out), "analytic_solutions": len(cands),
                    "scene_version": C.core.version}), 200


# ------------------------------------------------------------------ check_trajectory

@bp.route("/plan/check_trajectory", methods=["POST"])
def check_trajectory():
    """Densify a joint trajectory (max_step_rad, default 0.01) and collision-check every sample against the
    current scene; no motion, no lock. points [[6]] or [{t, q}] (joint_names to reorder) or plan_id from
    /plan/dry_run; gripper_width_m. -> valid, first_invalid_index (segment start), first_invalid_t, contacts,
    states_checked, scene_version."""
    d = _body()
    _check_version(d)
    max_step = float(d.get("max_step_rad", 0.01))
    if not 0.001 <= max_step <= 0.1:
        raise err(400, "bad_request", "max_step_rad must be in 0.001..0.1", "max_step_rad")
    times = None
    if d.get("plan_id"):
        with _plans_lock:
            rec = _plans.get(d["plan_id"])
        if rec is None or rec[0] < time.time():
            raise err(404, "unknown_plan", "plan_id unknown or expired (plans are kept 10 min)", "plan_id")
        pts = rec[1]["points"]
    else:
        raw = d.get("points")
        if not isinstance(raw, list) or not raw:
            raise err(400, "bad_request", "points must be a non-empty list", "points")
        order = None
        if d.get("joint_names"):
            jn = list(d["joint_names"])
            if sorted(jn) != sorted(C.cell.arm_joints):
                raise err(400, "bad_request", f"joint_names must be {C.cell.arm_joints}", "joint_names")
            order = [jn.index(n) for n in C.cell.arm_joints]
        pts, times = [], []
        for i, p in enumerate(raw):
            q = _joints(p.get("q") if isinstance(p, dict) else p, f"points[{i}]")
            pts.append([q[k] for k in order] if order else q)
            times.append(p.get("t") if isinstance(p, dict) else None)
    st, _ = _state(pts[0], _width(d))
    v, k, contacts, checked = interp_check(pts, st, max_step)
    return jsonify({"valid": v, "first_invalid_index": k,
                    "first_invalid_t": times[k] if times and k is not None else None, "contacts": contacts,
                    "states_checked": checked, "points": len(pts), "scene_version": C.core.version}), 200


# ------------------------------------------------------------------ dry_run_sequence

def _virtual_attached(q, width, attached):
    """AttachedCollisionObjects for objects 'held' in a sequence: full geometry posed relative to their link."""
    out = []
    for oid, a in attached.items():
        obj = dict(a["obj"], frame_id=a["link"], pose=a["rel"])
        out.append(C.scene_io.attached_object(obj, a["link"], a["touch_links"], C.core.mesh_cache))
    return out


def _link_T(q, link, width):
    return C.kin.fk(q, link, extra=C.cell.finger_q(width or 0.0))


@bp.route("/plan/dry_run_sequence", methods=["POST"])
def dry_run_sequence():
    """Plan a whole pick (pre-grasp -> descend -> close -> attach -> lift) without moving: {start,
    gripper_width_m, defaults:{planner, velocity_scaling, ...}, steps:[{plan: <dry_run body without start>} |
    {gripper_width_m: w} | {attach: {id, link, touch_links}} | {detach: {id}} | {acm: {entries:[{a, b,
    allowed}]}}]}. Each step starts where the previous ended; the first failing step ends it. attach / detach /
    acm only affect the sequence: the held object travels in the planning start state, its world copy and the
    ACM entries are changed TEMPORARILY (under the motion lock) and restored in one confirmed diff at the end
    (failure to restore marks the scene dirty). -> {success, first_failed_step, steps, restored}."""
    t0 = time.time()
    d = _body()
    _check_version(d)
    steps = d.get("steps")
    if not isinstance(steps, list) or not steps:
        raise err(400, "bad_request", "steps must be a non-empty list", "steps")
    if len(steps) > 20:
        raise err(400, "bad_request", "at most 20 steps", "steps")
    defaults = d.get("defaults") or {}
    width = _width(d)
    _, q = _start_state(d, width)
    if width is None:
        width = C.gripper.state()["width_m"] or C.cell.max_width
    if not C.lock.acquire(timeout=5.0):
        raise err(409, "busy", "robot is busy (motion in progress)")
    attached, temp_removed, temp_added, temp_acm = {}, {}, {}, {}
    acm0 = C.scene_io.get_acm()
    results, failed = [], None
    core = C.core
    try:
        with core._lock:
            for i, step in enumerate(steps):
                path = f"steps[{i}]"
                if not isinstance(step, dict) or len(step) != 1:
                    raise err(400, "bad_request", "each step is an object with exactly one key", path)
                kind, arg = next(iter(step.items()))
                r = {"index": i, "kind": kind}
                vstate = lambda: C.motion.robot_state(q, width, _virtual_attached(q, width, attached))  # noqa: E731
                if kind == "plan":
                    if not isinstance(arg, dict):
                        raise err(400, "bad_request", "plan step needs a dry_run body", path)
                    res = plan_one(dict(defaults, **arg), vstate(), q, time.time())
                    wbr = res.get("would_be_rejected") or {}
                    ok = bool(res.get("success") and res.get("trajectory_valid", True) and not wbr.get("too_large")
                              and not wbr.get("near_limit") and not wbr.get("target_near_limit"))
                    r.update({k: res[k] for k in ("success", "error_code", "planner", "pipeline", "duration_s",
                                                  "joint_travel_deg", "would_be_rejected", "trajectory_valid",
                                                  "contacts", "fraction", "final_tcp_pose", "goal_contacts",
                                                  "start_contacts", "trajectory") if k in res})
                    r["ok"] = ok
                    if ok:
                        q = res["end_joints"]
                elif kind == "gripper_width_m":
                    width = _width({"w": arg}, "w")
                    v, c = _valid(vstate())
                    r.update(ok=v, gripper_width_m=width, valid=v, contacts=c)
                elif kind == "attach":
                    oid = (arg or {}).get("id")
                    o = core.objects.get(oid)
                    if o is None:
                        raise err(404, "unknown_object", f"{oid!r} is not a /scene/batch object", f"{path}.id")
                    if oid in attached or o.get("attached_link"):
                        raise err(409, "object_attached", f"{oid!r} is already attached", f"{path}.id")
                    link = arg.get("link", C.cell.hand_link)
                    touch = arg.get("touch_links") or C.cell.touch_links(link)
                    pose = temp_added[oid]["pose"] if oid in temp_added else o["pose"]
                    rel = geom.T_to_pose(np.linalg.inv(_link_T(q, link, width)) @ geom.pose_to_T(pose))
                    attached[oid] = {"obj": o, "link": link, "touch_links": touch, "rel": rel}
                    core._apply_and_confirm([("remove", oid, None)], set(), {oid})     # the world copy is "in the hand"
                    if oid in temp_added:
                        temp_added.pop(oid)
                    else:
                        temp_removed[oid] = o
                    v, c = _valid(vstate())
                    r.update(ok=v, id=oid, link=link, valid=v, contacts=c)
                elif kind == "detach":
                    oid = (arg or {}).get("id")
                    if oid not in attached:
                        raise err(409, "bad_request", f"{oid!r} is not attached in this sequence", f"{path}.id")
                    a = attached.pop(oid)
                    placed = dict(a["obj"], pose=geom.T_to_pose(_link_T(q, a["link"], width) @ geom.pose_to_T(a["rel"])),
                                  frame_id=C.cell.base_frame, attached_link=None)
                    core._apply_and_confirm([("add", oid, placed)], {oid}, set())
                    temp_added[oid] = placed
                    v, c = _valid(vstate())
                    r.update(ok=v, id=oid, placed_pose=placed["pose"], valid=v, contacts=c)
                elif kind == "acm":
                    entries = (arg or {}).get("entries")
                    if not isinstance(entries, list) or not entries:
                        raise err(400, "bad_request", "acm step needs entries", f"{path}.entries")
                    for j, e in enumerate(entries):
                        if not isinstance(e, dict) or not all(isinstance(e.get(k), str) for k in ("a", "b")) \
                                or not isinstance(e.get("allowed"), bool):
                            raise err(400, "bad_request", "entry needs a, b, allowed", f"{path}.entries[{j}]")
                        a_, b_ = sorted((e["a"], e["b"]))
                        temp_acm[f"{a_}|{b_}"] = {"a": a_, "b": b_, "allowed": e["allowed"]}
                    merged = dict(core.acm_entries)
                    merged.update(temp_acm)
                    core._apply_and_confirm([], set(), set(), acm=core._acm_msg_with(merged))
                    r.update(ok=True, entries=len(temp_acm))
                else:
                    raise err(400, "bad_request", "step kind must be plan | gripper_width_m | attach | detach | acm",
                              path)
                r["joints"] = q
                results.append(r)
                if not r["ok"]:
                    failed = i
                    break
    finally:
        restored = True
        try:
            ops = [("remove", oid, None) for oid in temp_added]
            ops += [("add", oid, o) for oid, o in temp_removed.items()]
            ops += [("attach", oid, {"link": o["attached_link"], "touch_links": o.get("touch_links")})
                    for oid, o in temp_removed.items() if o.get("attached_link")]
            if ops or temp_acm:
                present = {oid for oid, o in temp_removed.items() if not o.get("attached_link")}
                core._apply_and_confirm(ops, present, set(temp_added) - set(temp_removed),
                                        acm=acm0 if temp_acm else None)
        except Exception as e:      # noqa: BLE001
            restored = False
            core.mark_dirty(f"dry_run_sequence could not restore the scene: {e}",
                            {oid.rsplit('.', 1)[0] for oid in list(temp_removed) + list(temp_added)} or {"pipefit"})
            core.save()
        C.lock.release()
    return jsonify({"success": failed is None, "first_failed_step": failed, "steps": results, "restored": restored,
                    "scene_version": core.version, "elapsed_ms": int((time.time() - t0) * 1000)}), 200
