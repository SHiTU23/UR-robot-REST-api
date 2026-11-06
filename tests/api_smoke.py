"""End-to-end check of the REST API against a running stack (mock hardware or URSim -- never the real arm
unless you mean it: this MOVES the robot).

    python3 tests/api_smoke.py --url http://127.0.0.1:5100 [--only motion,scene,...] [--no-motion]

Every check prints PASS/FAIL with what it saw; the exit code is the number of failures."""
import argparse
import io
import json
import math
import os
import struct
import sys
import threading
import time
import urllib.error
import urllib.request

FAILS = []
GM = {}         # the gripper model the API runs (config/grippers/<model>.yaml), set in __main__
TCP_Z = None    # expected flange -> TCP, from the repo's config files (not from the API)
try:
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "state", "api_token")) as _f:
        TOKEN = os.environ.get("UR_API_TOKEN") or _f.read().strip()
except OSError:
    TOKEN = os.environ.get("UR_API_TOKEN")


def load_gripper(base):
    import yaml
    model = call(base, "GET", "/health")[1]["gripper_model"]
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config")
    GM.update(yaml.safe_load(open(os.path.join(root, "grippers", model + ".yaml"))))
    GM["model"] = model
    cell = yaml.safe_load(open(os.path.join(root, "cell.yaml")))
    global TCP_Z
    TCP_Z = cell["tool"]["changer_length_m"] + GM["mount_offset_m"] + GM["table"][-1][2]


def call(base, method, path, body=None, raw=None, ctype=None, timeout=120, auth=True):
    data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
    req = urllib.request.Request(base + path, data=data, method=method)
    if TOKEN and auth:
        req.add_header("Authorization", "Bearer " + TOKEN)
    if body is not None:
        req.add_header("Content-Type", "application/json")
    if ctype:
        req.add_header("Content-Type", ctype)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            content = r.read()
            status = r.status
    except urllib.error.HTTPError as e:
        content, status = e.read(), e.code
    try:
        return status, json.loads(content)
    except ValueError:
        return status, content


def check(name, cond, detail=""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""), flush=True)
    if not cond:
        FAILS.append(name)


def box_stl(sx, sy, sz):
    """Binary STL of an axis-aligned box centred at the origin (12 triangles)."""
    x, y, z = sx / 2, sy / 2, sz / 2
    v = [(-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z), (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6),
         (3, 0, 4), (3, 4, 7)]
    out = io.BytesIO()
    out.write(b"\0" * 80 + struct.pack("<I", len(f)))
    for a, b, c in f:
        out.write(struct.pack("<3f", 0, 0, 0))
        for i in (a, b, c):
            out.write(struct.pack("<3f", *v[i]))
        out.write(b"\0\0")
    return out.getvalue()


# a working pose above the table, wrist down (found with /plan/ik in the first run of this script)
WORK_DEG = [0, -100, 100, -90, -90, 0]


def motion(B):
    s, j = call(B, "GET", "/joints")
    check("GET /joints", s == 200 and len(j["positions"]) == 6, f"{s} q={[round(v, 3) for v in j.get('positions', [])]}")
    q = ",".join(str(v) for v in WORK_DEG)
    s, r = call(B, "GET", f"/control/goto_joints?joints={q}&unit=deg&dry_run=1")
    check("goto_joints dry_run", s == 200 and r["outcome"] == "planned", f"{s} {r.get('planner_used')} "
          f"{r.get('duration_s')} s travel {r.get('joint_travel_deg')}")
    s, r = call(B, "GET", f"/control/goto_joints?joints={q}&unit=deg&velocity=0.5&max_joint_travel_deg=0")
    check("goto_joints move", s == 200 and r["outcome"] == "success", f"{s} {r.get('outcome')} {r.get('msg')}")
    s, j = call(B, "GET", "/joints")
    err = max(abs(math.degrees(a) - b) for a, b in zip(j["positions"], WORK_DEG))
    check("arrived at target", err < 0.2, f"max error {err:.3f} deg")
    s, st = call(B, "GET", "/state")
    p0 = st["position"]
    check("GET /state", s == 200, f"tcp {[round(p0[k], 4) for k in 'xyz']} rpy {st['rpy_deg']}")
    z1 = p0["z"] - 0.05
    s, r = call(B, "GET", f"/control/plan_cartesian_path?x={p0['x']}&y={p0['y']}&z={z1}&velocity=0.3")
    s2, st2 = call(B, "GET", "/state")
    dz = st2["position"]["z"] - z1
    dxy = math.hypot(st2["position"]["x"] - p0["x"], st2["position"]["y"] - p0["y"])
    check("plan_cartesian_path 5 cm down", s == 200 and r["outcome"] == "success" and abs(dz) < 1e-3 and dxy < 1e-3,
          f"{s} {r.get('outcome')} fraction {r.get('fraction')} z err {dz * 1000:.2f} mm xy err {dxy * 1000:.2f} mm")
    s, r = call(B, "GET", f"/control/plan_joint_path?x={p0['x'] + 0.05}&y={p0['y']}&z={p0['z']}")
    check("plan_joint_path LIN", s == 200 and r["outcome"] == "success", f"{s} {r.get('outcome')} {r.get('moveit_error')}")
    s, r = call(B, "GET", f"/control/plan_joint_path?x={p0['x']}&y={p0['y'] + 0.05}&z={p0['z']}&planner_id=RRTConnect"
                          f"&roll=180&pitch=0&yaw=90")
    check("plan_joint_path RRTConnect + rpy", s == 200 and r["outcome"] == "success",
          f"{s} {r.get('outcome')} {r.get('moveit_error')}")
    s, st3 = call(B, "GET", "/state")
    rpy = st3["rpy_deg"]
    check("orientation reached", abs(abs(rpy["roll"]) - 180) < 0.2 and abs(rpy["pitch"]) < 0.2 and abs(rpy["yaw"] - 90) < 0.2,
          f"rpy {rpy}")
    # frame=base (pendant frame): same point given with x, y negated
    s, r = call(B, "GET", f"/control/plan_cartesian_path?x={-p0['x']}&y={-p0['y']}&z={p0['z']}&frame=base&velocity=0.3")
    s2, st4 = call(B, "GET", "/state")
    d = math.dist([st4["position"][k] for k in "xyz"], [p0[k] for k in "xyz"])
    check("frame=base (pendant) target", s == 200 and r["outcome"] == "success" and d < 1e-3, f"{r.get('outcome')} {d * 1000:.2f} mm")
    # guards
    s, r = call(B, "GET", "/control/goto_joints?j1=180&unit=deg&dry_run=1")
    check("big swing reported in dry run", s == 200 and r["would_be_rejected_as_too_large"],
          f"{r.get('would_be_rejected_as_too_large')}")
    s, r = call(B, "GET", "/control/goto_joints?j1=180&unit=deg")
    check("big swing refused (409 plan_too_large)", s == 409 and r["outcome"] == "plan_too_large",
          f"{s} {r.get('joint_travel_deg')}")
    s, r = call(B, "GET", "/control/goto_joints?j3=180&unit=deg")
    check("target near limit refused", s == 400 and r.get("error") == "target_near_limit", f"{s} {r.get('error')}")
    s, r = call(B, "GET", "/control/goto_joints?joints=0,-10,140,-90,-90,0&unit=deg")
    check("target in collision refused", s == 400 and r.get("error") == "target_in_collision",
          f"{s} {r.get('error')} {r.get('contacts')}")
    # stop mid-motion
    res = {}
    t = threading.Thread(target=lambda: res.update(r=call(B, "GET", "/control/goto_joints?j1=60&unit=deg&velocity=0.05")))
    t.start()
    time.sleep(1.5)
    s, b = call(B, "GET", "/control/goto_joints?j1=30&unit=deg&dry_run=1")
    check("second motion while moving -> 409 busy", s == 409, f"{s} {b}")
    s, r = call(B, "GET", "/control/stop")
    t.join()
    s2, r2 = res["r"]
    _, j = call(B, "GET", "/joints")
    j1 = math.degrees(j["positions"][0])
    check("stop mid-motion -> stopped short of the target", s == 200 and r2.get("outcome") == "stopped" and abs(j1 - 60) > 1,
          f"{s2} {r2.get('outcome')} j1 now {j1:.1f} deg (target 60)")
    s, r = call(B, "GET", "/control/resume")
    _, j = call(B, "GET", "/joints")
    check("resume completes it", s == 200 and r["outcome"] == "success" and abs(math.degrees(j["positions"][0]) - 60) < 0.2,
          f"{s} {r.get('outcome')} j1 {math.degrees(j['positions'][0]):.2f}")
    s, r = call(B, "GET", "/control/nudge_joint?joint=6&delta=0.1")
    check("nudge_joint", s == 200 and r["executed"], f"{s} {r.get('status')}")
    s, r = call(B, "GET", "/control/home")
    _, j = call(B, "GET", "/joints")
    s3, h = call(B, "GET", "/help")
    check("home", s == 200 and r["status"] == "homed", f"{s} {r.get('status')} {r.get('planner_used')}")
    call(B, "GET", f"/control/goto_joints?joints={q}&unit=deg&velocity=0.5&max_joint_travel_deg=0")


def gripper(B):
    s, r = call(B, "GET", "/control/gripper_open_force?width=0.08")
    check("gripper_open_force", s == 200 and r["success"], f"{s} {r.get('gripper_width_m')} {r.get('notes')}")
    s, r = call(B, "GET", "/control/gripper_grasp?width=0.013&eps_in=0.007&eps_out=0.010&force=20&speed=0.03")
    check("gripper_grasp (pipeFit params)", s == 200 and r["success"] and r["force"] == max(20.0, GM["min_force_n"]),
          f"{s} force {r.get('force')} notes {r.get('notes')} program width {r.get('program_width_cmd')}")
    s, j = call(B, "GET", "/joints")
    check("/joints gripper width + fingers", j["gripper_width_m"] is not None and list(j["fingers"]) == [GM["joint"]],
          f"{j['gripper_width_m']} {j['fingers']}")
    for path in ("/control/gripper_open", "/control/gripper_close", "/control/go_to_gripper_state?width=0.05",
                 "/control/gripper_homing"):
        s, r = call(B, "GET", path)
        check(path, s == 200, f"{s} {r}")


def scene(B):
    s, info = call(B, "GET", "/scene/info")
    check("scene/info", s == 200 and info["planning_frame"], f"v{info.get('scene_version')} frame "
          f"{info.get('planning_frame')} tcp {info.get('flange_to_tcp_m')} builtin {info.get('builtin_world')}")
    v0 = info["scene_version"]
    body = {"namespace": "pipefit.test1", "mode": "replace", "objects": [
        {"id": "pipefit.test1.bin", "pose": {"position": [0.0, 0.7, 0.05], "orientation": [0, 0, 0, 1]},
         "shapes": [{"type": "box", "size": [0.4, 0.3, 0.02], "pose": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}},
                    {"type": "cylinder", "radius": 0.02, "length": 0.2}], "color": [0.1, 0.2, 0.9]}]}
    s, r = call(B, "POST", "/scene/batch", body)
    check("scene/batch add", s == 200 and r["added"] == ["pipefit.test1.bin"] and r["scene_version"] == v0 + 1,
          f"{s} {r.get('error')} {r.get('msg')} v{r.get('scene_version')}")
    s, r = call(B, "POST", "/scene/batch", body)
    check("scene/batch identical -> unchanged", s == 200 and r.get("unchanged") is True, f"{r.get('unchanged')}")
    bad = json.loads(json.dumps(body))
    bad["objects"][0]["shapes"][0]["size"] = [0.4, -1, 0.02]
    s, r = call(B, "POST", "/scene/batch", bad)
    check("scene/batch invalid -> 400, untouched", s == 400 and r["error"] == "bad_shape", f"{s} {r.get('path')}")
    s, r = call(B, "POST", "/plan/check_state", {"joints": [math.radians(v) for v in WORK_DEG]})
    check("plan/check_state", s == 200 and "valid" in r, f"valid {r.get('valid')} contacts {len(r.get('contacts', []))}")
    # mesh via /files + legacy models
    s, r = call(B, "PUT", "/files/cad/test_box.stl", raw=box_stl(0.1, 0.1, 0.1), ctype="application/octet-stream")
    check("PUT /files", s == 201 and r["size"] == 684, f"{s} {r.get('size')} {r.get('sha256', '')[:12]}")
    s, r = call(B, "GET", "/files?q=*.stl")
    check("files search", s == 200 and len(r["entries"]) >= 1, f"{len(r.get('entries', []))}")
    s, r = call(B, "POST", "/scene/models", {"name": "testbox", "file": "cad/test_box.stl", "overwrite": 1})
    check("scene/models register", s == 201 and r["triangles"] == 12, f"{s} {r.get('error')} {r.get('msg')} "
          f"bbox {r.get('bbox_size')}")
    s, r = call(B, "POST", "/scene/objects", {"name": "box_1", "model": "testbox", "x": 0.3, "y": 0.6, "z": 0.1, "yaw": 30})
    check("scene/objects add", s == 201, f"{s} {r.get('error')} {r.get('msg')}")
    s, r = call(B, "PUT", "/scene/objects/sync", {"source": "detector", "objects": [
        {"name": "det_1", "model": "testbox", "x": -0.3, "y": 0.6, "z": 0.1}]})
    check("scene/objects sync", s == 200 and r["added"] == ["det_1"], f"{s} {r}")
    s, r = call(B, "GET", "/scene/objects?geometry=1")
    ids = [o["id"] for o in r.get("objects", [])]
    check("scene/objects v2 listing", s == 200 and "box_1" in ids and "pipefit.test1.bin" in ids and "floor" in ids,
          f"{ids}")
    mesh_body = {"namespace": "pipefit.test2", "objects": [
        {"id": "pipefit.test2.part", "pose": {"position": [0.3, 0.8, 0.1], "orientation": [0, 0, 0, 1]},
         "shapes": [{"type": "mesh", "file": "cad/test_box.stl", "scale": 1.0}]}]}
    s, r = call(B, "POST", "/scene/batch", mesh_body)
    check("scene/batch mesh", s == 200 and r["counts"]["triangles"] == 12, f"{s} {r.get('error')} {r.get('msg')}")
    s, r = call(B, "POST", "/scene/objects/pipefit.test2.part/attach", {})
    check("attach batch object", s == 200 and r.get("attached_link") == GM["hand_link"], f"{s} {r.get('error')} {r.get('msg')}")
    s, r = call(B, "POST", "/scene/objects/pipefit.test2.part/detach", {})
    check("detach batch object", s == 200 and r.get("attached_link") is None, f"{s} {r.get('error')} {r.get('msg')}")
    s, r = call(B, "POST", "/scene/acm", {"entries": [{"a": "pipefit.test1.bin", "b": GM["pad_links"][0], "allowed": True}]})
    check("scene/acm set", s == 200, f"{s} {r.get('error')} {r.get('msg')}")
    s, r = call(B, "GET", "/scene/acm?names=pipefit.test1.bin")
    check("scene/acm get", s == 200 and any(e["client"] for e in r["entries"]), f"{len(r.get('entries', []))} entries")
    s, r = call(B, "DELETE", "/scene/acm")
    check("scene/acm restore", s == 200 and r["removed"], f"{r.get('removed')}")
    s, r = call(B, "POST", "/scene/padding", {"default_m": 0.005, "links": {GM["pad_links"][0]: 0.002}})
    check("scene/padding", s == 200, f"{s} {r.get('error')} {r.get('msg')}")
    call(B, "POST", "/scene/padding", {"default_m": 0.0})
    s, r = call(B, "GET", "/scene/snapshot?namespace=pipefit")
    check("scene/snapshot", s == 200 and len(r["objects"]) == 2, f"{len(r.get('objects', []))} objects")
    s, r = call(B, "GET", "/scene/unreferenced?prefix=cad&older_than_s=0")
    check("scene/unreferenced", s == 200, f"{r}")
    s, r = call(B, "DELETE", "/scene/namespaces/pipefit")
    check("delete namespace", s == 200 and len(r["removed"]) == 2, f"{r.get('removed')}")
    for n in ("box_1", "det_1"):
        call(B, "DELETE", f"/scene/objects/{n}")
    s, r = call(B, "DELETE", "/scene/models/testbox")
    check("delete model", s == 200, f"{s} {r}")
    s, r = call(B, "DELETE", "/scene/objects/floor")
    check("built-in floor protected", s == 409 and r["error"] == "builtin", f"{s} {r.get('error')}")


def plan(B):
    q = [math.radians(v) for v in WORK_DEG]
    s, st = call(B, "GET", "/state")
    pose = {"position": [st["position"][k] for k in "xyz"], "orientation": [st["orientation"][k] for k in "xyzw"]}
    s, r = call(B, "POST", "/plan/ik", {"pose": pose, "n_solutions": 8})
    check("plan/ik", s == 200 and r["reachable"] and len(r["solutions"]) >= 2,
          f"{len(r.get('solutions', []))} solutions, {r.get('analytic_solutions')} analytic, "
          f"valid {[o['valid'] for o in r.get('solutions', [])]}")
    p2 = dict(pose, position=[pose["position"][0], pose["position"][1], pose["position"][2] - 0.1])
    for planner, target in (("RRTConnect", {"type": "pose", "pose": p2}), ("PTP", {"type": "joints", "joints": q}),
                            ("LIN", {"type": "pose", "pose": p2}),
                            ("RRTConnect", {"type": "cartesian", "waypoints": [p2]})):
        s, r = call(B, "POST", "/plan/dry_run", {"planner": planner, "target": target})
        check(f"plan/dry_run {planner} {target['type']}", s == 200 and r.get("success"),
              f"{r.get('error_code')} {r.get('duration_s')} s valid {r.get('trajectory_valid')} "
              f"checked {r.get('interpolated_states_checked')} plan {r.get('plan_id')}")
    pid = r.get("plan_id")
    s, r = call(B, "POST", "/plan/check_trajectory", {"plan_id": pid})
    check("plan/check_trajectory plan_id", s == 200 and r["valid"], f"{r.get('states_checked')} states")
    q2 = list(q)
    q2[1] += 0.3
    s, r = call(B, "POST", "/plan/check_trajectory", {"points": [q, q2], "gripper_width_m": 0.08})
    check("plan/check_trajectory points", s == 200 and r["valid"], f"{r.get('states_checked')} states")
    bad = list(q)
    bad[1], bad[2] = math.radians(-10), math.radians(140)
    s, r = call(B, "POST", "/plan/check_trajectory", {"points": [q, bad]})
    check("check_trajectory finds the collision", s == 200 and not r["valid"],
          f"first invalid {r.get('first_invalid_index')} {[(c['a'], c['b']) for c in r.get('contacts', [])][:2]}")
    call(B, "POST", "/scene/batch", {"namespace": "pipefit.seq", "objects": [
        {"id": "pipefit.seq.tube", "pose": {"position": [p2["position"][0], p2["position"][1], p2["position"][2] - 0.02],
                                            "orientation": [0, 0, 0, 1]},
         "shapes": [{"type": "cylinder", "radius": 0.012, "length": 0.15,
                     "pose": {"position": [0, 0, 0], "orientation": [0.7071068, 0, 0, 0.7071068]}}]}]})
    s, r = call(B, "POST", "/plan/dry_run_sequence", {"gripper_width_m": 0.08, "defaults": {"planner": "LIN"}, "steps": [
        {"plan": {"target": {"type": "pose", "pose": p2}}},
        {"acm": {"entries": [{"a": "pipefit.seq.tube", "b": GM["pad_links"][0], "allowed": True},
                             {"a": "pipefit.seq.tube", "b": GM["pad_links"][1], "allowed": True}]}},
        {"gripper_width_m": 0.024}, {"attach": {"id": "pipefit.seq.tube"}},
        {"plan": {"target": {"type": "pose", "pose": pose}}}, {"detach": {"id": "pipefit.seq.tube"}}]})
    check("plan/dry_run_sequence", s == 200 and r["success"] and r["restored"],
          f"{s} failed step {r.get('first_failed_step')} {[(x['kind'], x['ok']) for x in r.get('steps', [])]} "
          f"{r.get('error')} {r.get('msg')}")
    s, snap = call(B, "GET", "/scene/snapshot?namespace=pipefit.seq")
    check("sequence restored the scene", s == 200 and snap["objects"][0]["pose"]["position"][2] < p2["position"][2],
          f"{snap.get('objects', [{}])[0].get('pose')}")
    call(B, "DELETE", "/scene/namespaces/pipefit.seq")


def misc(B):
    s, r = call(B, "GET", "/help")
    check("/help", s == 200 and len(r["endpoints"]) > 60, f"{len(r.get('endpoints', []))} endpoints")
    s, r = call(B, "GET", "/robot/tf?from=tool0&to=tcp")
    check("/robot/tf tool0->tcp", s == 200 and abs(r["pose"]["position"][2] - TCP_Z) < 1e-9, f"{r.get('pose')} want z {TCP_Z}")
    s, r = call(B, "GET", "/robot/description")
    check("/robot/description", s == 200 and "tcp" in r["links"] and r["joint_limits"], f"{len(r.get('links', []))} links")
    s, r = call(B, "GET", "/force")      # mock hardware: no sensor (NaN) -> 503; URSim / real arm: a reading
    check("/force", (s == 200 and r["available"]) or (s == 503 and not r["available"]), f"{s} {r.get('raw_force') or r.get('msg')}")
    for p in ("/force/untare", "/force/config?scale=0.03"):
        s, r = call(B, "GET", p)
        check(p, s == 200, f"{s}")
    s, r = call(B, "GET", "/camera/info")
    check("/camera/info", s == 200, f"{r.get('backend')} {r.get('last_error')}")
    s, r = call(B, "GET", "/simulation/add_box")
    s2, r2 = call(B, "GET", "/simulation/remove_box")
    check("/simulation box", s == 200 and s2 == 200, "")
    s, r = call(B, "GET", "/control/goto_joints?joints=1,2,3&dry_run=1")
    check("bad args -> 400", s == 400, f"{r}")


def safety(B, ursim_ip):
    """URSim only: a protective stop in the middle of a motion (URScript protective_stop() as a secondary
    program, which runs next to the driver's), then /recover and /control/resume."""
    import socket
    q0 = math.degrees(call(B, "GET", "/joints")[1]["positions"][0])
    res = {}
    t = threading.Thread(target=lambda: res.update(r=call(B, "GET", f"/control/goto_joints?j1={q0 - 40}&unit=deg"
                                                                    "&velocity=0.05")))
    t.start()
    time.sleep(2.0)
    s = socket.create_connection((ursim_ip, 30002), timeout=5)
    s.sendall(b"sec trigger_ps():\n  protective_stop()\nend\n")
    s.close()
    t_ps = time.time()
    t.join()
    dt = time.time() - t_ps
    st, r = res["r"]
    check("protective stop -> stopped_external_force, promptly", r.get("outcome") == "stopped_external_force" and dt < 3,
          f"{st} {r.get('outcome')} after {dt:.2f} s")
    _, h = call(B, "GET", "/health")
    check("health shows it", h["safety_mode"] == "PROTECTIVE_STOP", f"{h['safety_mode']} program {h['driver_program_running']}")
    s, r = call(B, "GET", "/control/goto_joints?j1=0&unit=deg")
    check("motion refused while protective-stopped", r.get("outcome") == "stopped_external_force" or s >= 400,
          f"{s} {r.get('outcome') or r.get('error')}")
    s, r = call(B, "GET", "/recover")
    _, h = call(B, "GET", "/health")
    check("/recover unlocks + gives control back", s == 200 and h["safety_mode"] == "NORMAL" and h["driver_program_running"],
          f"{s} {r} -> {h['safety_mode']} program {h['driver_program_running']}")
    s, r = call(B, "GET", "/control/resume")     # resumes the LAST requested motion: the refused j1=0 (Franka rule)
    q = math.degrees(call(B, "GET", "/joints")[1]["positions"][0])
    check("resume runs the last requested motion", s == 200 and abs(q) < 0.2, f"{s} {r.get('outcome')} j1 {q:.2f} (want 0)")


def quat_angle_deg(a, b):
    return math.degrees(2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(a, b))))))


def rotate(B):
    q = ",".join(str(v) for v in WORK_DEG)
    call(B, "GET", f"/control/goto_joints?joints={q}&unit=deg&velocity=0.5&max_joint_travel_deg=0")

    def tcp():
        s = call(B, "GET", "/state")[1]
        return [s["position"][k] for k in "xyz"], [s["orientation"][k] for k in "xyzw"]
    p0, q0 = tcp()
    j0 = call(B, "GET", "/joints")[1]["positions"]
    moved = lambda: max(abs(a - b) for a, b in zip(call(B, "GET", "/joints")[1]["positions"], j0))  # noqa: E731
    xyz = f"x={p0[0]}&y={p0[1]}&z={p0[2] + 0.02}"
    s, r = call(B, "GET", f"/control/plan_cartesian_path?{xyz}&dry_run=1")
    time.sleep(0.5)
    check("plan_cartesian_path dry_run does not move", s == 200 and r["outcome"] == "planned" and moved() < 1e-3, f"{s} {moved():.5f} rad")
    call(B, "GET", "/control/gripper_open")
    for path in (f"/control/plan_cartesian_path?{xyz}&dry_run=1&require_object=1", "/control/rotate_tcp?axis=z&deg=5&dry_run=1&require_object=1"):
        s, r = call(B, "GET", path)
        check(f"dry run allowed, nothing moves: {path[:30]}", s == 200 and moved() < 1e-3, str(s))
    for path in (f"/control/plan_joint_path?{xyz}&dry_run=1&require_object=1", f"/control/home?dry_run=1&require_object=1", "/control/nudge_joint?joint=1&deg=1&dry_run=1&require_object=1"):
        s, r = call(B, "GET", path)
        check(f"dry_run is no way past the checks: {path[:30]}", s == 409 and r.get("error") == "object_not_held" and moved() < 1e-3, f"{s} {r.get('error')}")
    for frame in ("base_link", "tool", "base"):
        for ax in "xyz":
            p1, q1 = tcp()
            s, r = call(B, "GET", f"/control/rotate_tcp?axis={ax}&deg=10&frame={frame}&dry_run=1&velocity=0.3")
            ok = s == 200 and r["outcome"] == "planned" and r["tcp_drift_from_start_mm"] < 1.0
            check(f"plan rotate {ax} 10 deg ({frame})", ok, f"{s} {r.get('outcome')} drift {r.get('tcp_drift_from_start_mm')} mm, {r.get('waypoints')} points")
            s, r = call(B, "GET", f"/control/rotate_tcp?axis={ax}&deg=10&frame={frame}&velocity=0.3")
            time.sleep(0.2)
            p2, q2 = tcp()
            d = math.dist(p1, p2) * 1000
            a = quat_angle_deg(q1, q2)
            check(f"rotate {ax} 10 deg ({frame})", s == 200 and r["outcome"] == "success" and d < 1.0 and abs(a - 10) < 0.2, f"{s} {r.get('outcome')} moved {d:.3f} mm, turned {a:.2f} deg")
            call(B, "GET", f"/control/rotate_tcp?axis={ax}&deg=-10&frame={frame}&velocity=0.3")
    pz, qz = tcp()
    call(B, "GET", "/control/rotate_tcp?axis=z&deg=20&frame=base_link&velocity=0.3")
    pa, qa = tcp()
    R = lambda v: (math.cos(math.radians(v) / 2), math.sin(math.radians(v) / 2))  # noqa: E731
    c, sn = R(20)
    x, y, z, w = qz
    exp = [c * x - sn * y, c * y + sn * x, c * z + sn * w, c * w - sn * z]     # (rotation about base z) * q
    check("base z turns about the base axis, not the tool's", quat_angle_deg(exp, qa) < 0.2, f"{quat_angle_deg(exp, qa):.3f} deg off")
    call(B, "GET", "/control/rotate_tcp?axis=z&deg=-20&frame=base_link&velocity=0.3")
    for path in ("axis=w&deg=5", "axis=z&deg=0", "axis=z&deg=90", "axis=z&deg=nan", "axis=z", "axis=z&deg=5&frame=moon"):
        s, _ = call(B, "GET", f"/control/rotate_tcp?{path}")
        check(f"rotate_tcp {path} -> 400", s == 400, str(s))


def gripper_buttons(B):
    s, r = call(B, "GET", "/control/gripper_close")
    check("close button", s == 200 and r["result"], f"{s} {r.get('width_m')}")
    check("  /state says closed", call(B, "GET", "/state")[1]["gripper"] == 0)
    s, r = call(B, "GET", "/control/go_to_gripper_state?width=0.05")
    check("go to 50 mm", s == 200 and abs(r["width_m"] - 0.05) < 1e-3, f"{s} {r.get('width_m')}")
    s, r = call(B, "GET", "/control/go_to_gripper_state?width=0.02")
    check("go to a smaller width", s == 200 and abs(r["width_m"] - 0.02) < 1e-3, f"{s} {r.get('width_m')}")
    s, r = call(B, "GET", "/control/gripper_open")
    check("open button", s == 200 and r["width_m"] == GM["max_width_m"], f"{s} {r.get('width_m')}")
    s, _ = call(B, "GET", "/control/go_to_gripper_state?width=abc")
    check("width not a number -> 400", s == 400, str(s))


def busy_display(B):
    box = {}
    t = threading.Thread(target=lambda: box.update(r=call(B, "GET", "/control/gripper_close")))
    t.start()
    time.sleep(0.05)
    h = call(B, "GET", "/health")[1]
    s, r = call(B, "GET", "/control/gripper_open")
    t.join()
    check("/health says what is running", h["lock_busy"] and h["activity"]["what"] == "Closing gripper", str(h.get("activity")))
    check("second press -> 409 naming it", s == 409 and r.get("activity", {}).get("what") == "Closing gripper", f"{s} {r.get('activity')}")
    check("idle again afterwards", call(B, "GET", "/health")[1]["activity"] is None)
    call(B, "GET", "/control/gripper_open")


def named_poses(B):
    from urllib.parse import quote
    n1, n2 = "smoke pose 1", "smoke_pose.2"
    for n in (n1, n2):
        call(B, "DELETE", "/robot/poses/" + quote(n))
    a = [0, -100, 100, -90, -90, 0]
    b = [10, -95, 95, -90, -90, 10]
    goto = lambda v: call(B, "GET", "/control/goto_joints?unit=deg&velocity=0.5&max_joint_travel_deg=0&joints=" + ",".join(map(str, v)))  # noqa: E731
    now = lambda: [math.degrees(v) for v in call(B, "GET", "/joints")[1]["positions"]]  # noqa: E731
    off = lambda t: max(abs(x - y) for x, y in zip(now(), t))  # noqa: E731
    goto(a)
    s, r = call(B, "POST", "/robot/poses", {"name": n1})
    check("save a named pose", s == 200 and r["status"] == "saved" and max(abs(x - y) for x, y in zip(r["joints_deg"], a)) < 0.05, f"{s} {r.get('msg', '')}")
    s, r = call(B, "POST", "/robot/poses", {"name": n1})
    check("same name again -> 409", s == 409 and r["error"] == "pose_exists", str(s))
    goto(b)
    s, r = call(B, "POST", "/robot/poses", {"name": n2})
    check("save a second pose", s == 200, str(s))
    names = [p["name"] for p in call(B, "GET", "/robot/poses")[1]["poses"]]
    check("both are listed", n1 in names and n2 in names, str(names))
    s, r = call(B, "GET", f"/control/goto_pose?name={quote(n1)}&dry_run=1")
    check("preview does not move", s == 200 and r["outcome"] == "planned" and off(b) < 0.01, f"{s} {r.get('outcome')}")
    s, r = call(B, "GET", f"/control/goto_pose?name={quote(n1)}&velocity=0.5")
    check("go to the first pose", s == 200 and r["outcome"] == "success" and off(a) < 0.05, f"{s} {r.get('outcome')} off {off(a):.3f} deg")
    s, r = call(B, "GET", f"/control/goto_pose?name={quote(n2)}&velocity=0.5")
    check("go to the second pose", s == 200 and r["outcome"] == "success" and off(b) < 0.05, f"{s} off {off(b):.3f} deg")
    s, r = call(B, "POST", "/robot/poses", {"name": n1, "overwrite": True})
    check("overwrite replaces it", s == 200 and max(abs(x - y) for x, y in zip(r["joints_deg"], b)) < 0.05, str(s))
    call(B, "GET", "/control/gripper_open")
    s, r = call(B, "GET", f"/control/goto_pose?name={quote(n2)}&require_object=1")
    check("require_object applies", s == 409 and r["error"] == "object_not_held", f"{s} {r.get('error')}")
    s, _ = call(B, "GET", "/control/goto_pose?name=nothing-here")
    check("unknown pose -> 404", s == 404, str(s))
    for bad in ("", "   ", "../../etc/x", "<script>", "x" * 41, 5):
        s, _ = call(B, "POST", "/robot/poses", {"name": bad})
        check(f"bad name {str(bad)[:12]!r} -> 400", s == 400, str(s))
    for n in (n1, n2):
        s, _ = call(B, "DELETE", "/robot/poses/" + quote(n))
        check(f"delete {n}", s == 200, str(s))
    s, _ = call(B, "DELETE", "/robot/poses/" + quote(n1))
    check("delete again -> 404", s == 404, str(s))


def room_axes(B):
    q = ",".join(str(v) for v in WORK_DEG)
    call(B, "GET", f"/control/goto_joints?joints={q}&unit=deg&velocity=0.5&max_joint_travel_deg=0")
    s0 = call(B, "GET", "/state")[1]
    yaw = math.radians(s0["room"]["yaw_deg"])
    px, py, pz = s0["room"]["position"]
    bx, by = (s0["position"][k] for k in "xy")
    s, r = call(B, "GET", f"/control/plan_cartesian_path?x={px}&y={py + 0.02}&z={pz}&frame=room&velocity=0.3")
    time.sleep(0.2)
    s1 = call(B, "GET", "/state")[1]
    dx, dy = s1["position"]["x"] - bx, s1["position"]["y"] - by
    ex, ey = 0.02 * math.sin(yaw), 0.02 * math.cos(yaw)
    check("room +y moves along the room's y", s == 200 and math.hypot(dx - ex, dy - ey) < 1e-3, f"robot dx {dx:.4f} dy {dy:.4f}, expected {ex:.4f} {ey:.4f}")
    rx, ry, rz = s1["room"]["position"]
    check("/state room position agrees", abs(ry - py - 0.02) < 1e-3 and abs(rx - px) < 1e-3, f"room dx {rx - px:.4f} dy {ry - py:.4f}")
    a0 = [s1["orientation"][k] for k in "xyzw"]
    s, r = call(B, "GET", "/control/rotate_tcp?axis=z&deg=10&frame=room&velocity=0.3")
    a1 = [call(B, "GET", "/state")[1]["orientation"][k] for k in "xyzw"]
    check("rotate about the room's z", s == 200 and r["outcome"] == "success" and abs(quat_angle_deg(a0, a1) - 10) < 0.2, f"{quat_angle_deg(a0, a1):.2f} deg")
    call(B, "GET", "/control/rotate_tcp?axis=z&deg=-10&frame=room&velocity=0.3")
    s, r = call(B, "GET", "/control/rotate_tcp?axis=x&deg=10&frame=room&dry_run=1")
    t = r.get("target_pose", {}).get("orientation")
    half = math.radians(5)
    ax = (math.cos(yaw) * math.sin(half), -math.sin(yaw) * math.sin(half), 0.0, math.cos(half))   # room x in the robot's frame
    x1, y1, z1, w1 = ax
    x2, y2, z2, w2 = a0
    e = [w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2]
    check("rotate about the room's x is about the turned axis", s == 200 and t and quat_angle_deg(e, t) < 0.2, f"{quat_angle_deg(e, t):.3f} deg off")
    s, _ = call(B, "GET", f"/control/plan_cartesian_path?x={px}&y={py}&z={pz}&frame=moon")
    check("unknown frame -> 400", s == 400, str(s))


def calibration(B):
    s, r = call(B, "GET", "/robot/calibration")
    if s == 404:
        check("no stored calibration -> 404", r["error"] == "no_calibration", str(r.get("error")))
        return
    check("GET /robot/calibration", s == 200 and r["available"] and r["hash"].startswith("calib_"), f"{s} {r.get('hash')}")
    check("the running model uses it", r["applied"] is True, f"applied {r.get('applied')}, matches_robot {r.get('matches_robot')}")
    check("all six links", all(k in r["kinematics"] for k in ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")))
    s, raw = call(B, "GET", "/robot/calibration?format=yaml")
    check("format=yaml gives the file", s == 200 and raw.startswith(b"kinematics:") and r["hash"].encode() in raw, str(s))
    s, _ = call(B, "GET", "/robot/calibration?format=xml")
    check("unknown format -> 400", s == 400, str(s))


def access(B):
    if call(B, "GET", "/state", auth=False)[0] == 200:
        check("UR_API_AUTH=off: /help says the API is open", "open" in call(B, "GET", "/help", auth=False)[1]["conventions"]["access"])
        s, r = call(B, "GET", "/robot/home", auth=False)
        check("UR_API_AUTH=off: no token needed", s == 200, str(s))
        return
    for path in ("/state", "/control/home", "/control/goto_joints?joints=0,0,0,0,0,0", "/files", "/help"):
        s, _ = call(B, "GET", path, auth=False)
        check(f"no token: {path} -> 401", s == 401, str(s))
    s, _ = call(B, "POST", "/robot/home", auth=False)
    check("no token: POST /robot/home -> 401", s == 401, str(s))
    s, _ = call(B, "GET", "/control/stop", auth=False)
    check("no token: stop still works", s == 200, str(s))
    s, _ = call(B, "POST", "/login", {"token": "wrong-token-wrong-token"}, auth=False)
    check("login with a wrong token -> 401", s == 401, str(s))


def bad_requests(B):
    q = {"position": [0.5, 0.2, 0.3], "orientation": [1, 0, 0, 0]}
    for name, body in (("empty body", {}), ("pose misnamed at top level", q), ("no orientation", {"pose": {"position": [0.5, 0.2, 0.3]}}),
                       ("no position", {"pose": {"orientation": [1, 0, 0, 0]}}), ("pose not an object", {"pose": 3}),
                       ("n_solutions not a number", {"pose": q, "n_solutions": "many"})):
        s, r = call(B, "POST", "/plan/ik", body)
        check(f"/plan/ik {name} -> 400", s == 400, f"{s} {r.get('msg') if isinstance(r, dict) else r}")
    s, r = call(B, "POST", "/plan/ik", {"pose": q})
    check("/plan/ik good pose -> 200", s == 200 and r["analytic_solutions"] > 0, str(s))
    for path in ("/control/gripper_open_force?width=nan", "/control/gripper_grasp?force=inf",
                 "/control/go_to_gripper_state?width=nan", "/control/gripper_grasp?max_retries=-1"):
        s, _ = call(B, "GET", path)
        check(f"{path} -> 400", s == 400, str(s))


def home(B):
    s, r0 = call(B, "GET", "/robot/home")
    check("GET /robot/home", s == 200 and len(r0["home_rad"]) == 6, f"{s} {r0.get('source')}")
    was_saved = r0["source"] == "saved"
    s, r = call(B, "POST", "/robot/home")
    q = call(B, "GET", "/joints")[1]["positions"]
    check("save current pose as home", s == 200 and r["source"] == "saved" and
          max(abs(a - b) for a, b in zip(r["home_rad"], q)) < 1e-3, f"{s} {r.get('msg')}")
    check("/control/home target follows", call(B, "GET", "/robot/home")[1]["home_rad"] == r["home_rad"])
    if not was_saved:
        s, r = call(B, "DELETE", "/robot/home")
        check("reset to the config home", s == 200 and r["source"] == "config" and r["home_rad"] == r0["config_home_rad"], str(s))


def grasp_check(B):
    """Needs nothing from the arm: with no width feedback, the object must not count as confirmed."""
    call(B, "GET", "/control/gripper_open")
    q = ",".join(str(v) for v in call(B, "GET", "/joints")[1]["positions"])
    s, r = call(B, "GET", f"/control/goto_joints?joints={q}&require_object=1")
    check("opened: require_object refuses", s == 409 and r["error"] == "object_not_held", f"{s} {r.get('error')}")
    s, r = call(B, "GET", f"/control/goto_joints?joints={q}&require_object=1&dry_run=1")
    check("require_object lets a dry run through", s == 200, str(s))
    s, r = call(B, "GET", "/state")
    check("/state reports object_held", s == 200 and "object_held" in r, str(r.get("object_held")))
    call(B, "GET", "/control/gripper_open")


GROUPS = {"access": access, "bad_requests": bad_requests, "home": home, "grasp_check": grasp_check, "gripper_buttons": gripper_buttons, "busy_display": busy_display, "named_poses": named_poses, "calibration": calibration, "room_axes": room_axes, "rotate": rotate, "motion": motion,
          "gripper": gripper, "scene": scene, "plan": plan, "misc": misc}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:5100")
    ap.add_argument("--only", default=",".join(GROUPS))
    ap.add_argument("--ursim-ip", default=None, help="adds the 'safety' group (protective stop), URSim only")
    a = ap.parse_args()
    if a.ursim_ip:
        GROUPS["safety"] = lambda B: safety(B, a.ursim_ip)
        if "safety" not in a.only:
            a.only += ",safety"
    t0 = time.time()
    load_gripper(a.url.rstrip("/"))
    print(f"gripper model: {GM['model']}, expected TCP {TCP_Z:.5f} m")
    for g in a.only.split(","):
        print(f"--- {g}", flush=True)
        try:
            GROUPS[g](a.url.rstrip("/"))
        except Exception as e:      # noqa: BLE001
            check(f"{g}: crashed", False, f"{type(e).__name__}: {e}")
    print(f"\n{len(FAILS)} failed ({time.time() - t0:.0f} s): {FAILS}")
    sys.exit(len(FAILS))
