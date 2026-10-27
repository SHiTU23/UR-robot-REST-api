"""UR REST API: the HTTP contract of XiaoxiaoZ's Franka-Emika-Panda-robot-REST-api, for a UR arm with an
OnRobot RG6 or Robotiq 2F-85, on ROS 2 Jazzy + MoveIt 2 (ur_robot_driver).

    python3 ur_api/server.py          # after ros/launch/cell.launch.py is up (same ROS_DOMAIN_ID)

Environment: UR_API_HOST (0.0.0.0), UR_API_PORT (5000), UR_MOCK_HARDWARE=1 (no robot: simulated gripper, no
dashboard), UR_ROBOT_IP, UR_CAMERA, UR_CELL_CONFIG, UR_STATE_DIR (scene state), UR_FILES_ROOT (+ _MAX_BYTES,
_QUOTA_BYTES), UR_API_TEST_HOOKS=1 (scene failure injection; never in production).
"""
import inspect
import logging
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flask import Flask, jsonify, request  # noqa: E402

from ctx import C  # noqa: E402
from errors import ApiError  # noqa: E402

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger("ur_rest_api")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

app = Flask(__name__)


@app.errorhandler(ApiError)
def _api_error(e):
    return jsonify({"error": e.code, "msg": e.msg, **e.extra}), e.status


@app.errorhandler(Exception)
def _unhandled(e):
    from werkzeug.exceptions import HTTPException
    if isinstance(e, HTTPException):
        return jsonify({"error": e.name, "message": e.description}), e.code
    log.exception("unhandled exception in handler")
    return jsonify({"error": type(e).__name__, "message": str(e)}), 500


# Safety gate for every motion endpoint (rules S2 + S4): refuse while the scene is dirty (a commit could not
# be confirmed or rolled back) and when the caller's expect_scene_version is stale. /control/stop always
# passes, and dry runs do not move.
@app.before_request
def _motion_gate():
    p = request.path
    if not p.startswith("/control/") or p == "/control/stop":
        return None
    if request.args.get("dry_run", "0") not in ("0", "false", ""):
        return None
    if C.core.dirty:
        return jsonify({"error": "scene_dirty", "msg": "the planning scene is in an unconfirmed state; push a "
                        "successful /scene/batch or DELETE /scene/namespaces/<ns> first",
                        "dirty_reason": C.core.dirty_reason}), 409
    exp = request.args.get("expect_scene_version")
    if exp is not None:
        try:
            exp = int(exp)
        except ValueError:
            return jsonify({"error": "bad_request", "msg": "expect_scene_version must be an integer"}), 400
        if exp != C.core.version:
            return jsonify({"error": "version_conflict", "scene_version": C.core.version,
                            "msg": f"scene_version is {C.core.version}, request expects {exp}"}), 409
    return None


@app.route("/", methods=["GET"])
def banner():
    return jsonify({"data": f"Hello, this is {C.cell.ur_type.upper()} with an {C.cell.gripper['name']} (UR REST API)",
                    "help": "GET /help for a full API usage guide", "mock_hardware": C.mock}), 200


@app.route("/help", methods=["GET"])
def api_help():
    """Machine-readable guide: every endpoint with methods and description (from the route docstrings),
    conventions (units, frames, locking, outcomes) and a quickstart."""
    eps = []
    for rule in app.url_map.iter_rules():
        if rule.endpoint == "static":
            continue
        view = app.view_functions.get(rule.endpoint)
        eps.append({"path": rule.rule, "methods": sorted(m for m in rule.methods if m not in ("HEAD", "OPTIONS")),
                    "description": inspect.getdoc(view) or ""})
    eps.sort(key=lambda e: e["path"])
    n = C.cell.n_joints
    return jsonify({
        "name": f"{C.cell.ur_type.upper()} + {C.cell.gripper['name']} REST API (Franka Emika Panda REST API contract)",
        "quickstart": [
            "GET /state for the TCP pose and gripper; GET /joints for joint angles, velocities, efforts and limits.",
            "GET /force for the flange F/T sensor wrench.",
            "GET /control/plan_cartesian_path?x=&y=&z= to move the TCP in a straight line (metres, base_link; "
            "MOVES THE ARM).",
            "GET /control/gripper_grasp?force=40 to grasp; /control/gripper_open_force?width=0.08 to open.",
            f"GET /control/goto_joints?joints=<{n} values>&unit=deg (dry_run=1 to preview; MOVES THE ARM).",
            "GET /control/stop to stop motion; GET /recover after a protective stop.",
            "GET /camera/stream for a live view; /camera/frame for colour + depth in one .npz.",
            "PUT /files/cad/part.stl (raw body) to store a CAD model; POST /scene/models, /scene/objects to place it.",
            "Scene API v2: GET /scene/info, POST /scene/batch, /scene/acm, /scene/padding, /scene/snapshot; plan-only: "
            "POST /plan/check_state, /plan/dry_run, /plan/ik, /plan/check_trajectory, /plan/dry_run_sequence; "
            "GET /robot/tf, /robot/description."],
        "conventions": {
            "units": "metres, newtons, N*m, radians (unit=deg where offered); positions absolute",
            "frames": f"{C.cell.base_frame} (ROS). The pendant / RTDE 'base' frame is base_link turned 180 deg about "
                      "z (x and y negated); motion endpoints take frame=base to use it",
            "tcp": f"{C.cell.tcp_link}: fingertip centre with the gripper closed, {C.cell.tcp_z:.3f} m along tool0 z",
            "joints": C.cell.arm_joints,
            "concurrency": "motion/gripper endpoints share one non-blocking lock -> 409 'Robot is busy' instead of "
                           "queuing; /state /joints /force /health /control/stop are lock-free",
            "outcomes": "motion endpoints return outcome: success | plan_failed | plan_would_violate_limits | "
                        "plan_too_large | execute_failed | recovery_failed | stopped | stopped_external_force "
                        "(UR protective stop) | stopped_user_button (emergency / safeguard stop) (+ attempts, "
                        "recovery_* fields)",
            "gripper": _gripper_note(C.cell, C.gripper)},
        "endpoints": eps,
        "readme": "https://github.com/SHiTU23/UR-robot-REST-api"}), 200


@app.route("/ui", methods=["GET"])
def web_ui():
    """Browser control panel (one self-contained page): state, gripper, jog, presets, force, camera, STOP."""
    try:
        with open(os.path.join(ROOT, "ur_api", "webui.html"), encoding="utf-8") as f:
            return f.read(), 200, {"Content-Type": "text/html; charset=utf-8"}
    except OSError:
        return jsonify({"error": "webui.html not found"}), 404


# ------------------------------------------------------------------ start-up

def _root_link(urdf):
    children = {j["child"] for j in urdf.joints.values()}
    roots = [l for l in urdf.links if l not in children]
    return roots[0] if roots else C.cell.base_frame


def _gripper_note(cell, gripper):
    g = cell.gripper
    how = {"rg6": "via pendant programs (RTDE registers + dashboard); the driver regains control after every "
                  "gripper action",
           "sim": "SIMULATED (mock hardware, or no control for this gripper yet): the fingers only move in the model",
           "none": "NOT CONTROLLABLE on this arm yet: gripper endpoints answer 501"}.get(gripper.backend, gripper.backend)
    return (f"{g['name']} ({cell.gripper_model}), {how}; 0..{g['max_width_m']:.3f} m, "
            f"{g['min_force_n']:.0f}..{g['max_force_n']:.0f} N")


def _builtin_tool(cell):
    """The tool parts as listed by /scene/info and /scene/objects (they are URDF links, see cell_tool.xacro)."""
    t = cell.cfg["tool"]
    ident = {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    out = []
    for oid, link, shape, est in (
            ("tool_changer", "tool_changer", {"type": "cylinder", "radius": t["changer_radius_m"],
                                              "length": t["changer_length_m"]}, True),
            ("wrist_camera", "wrist_camera", {"type": "box", "size": t["camera_box_m"]}, False),
            ("camera_bracket", "camera_bracket", {"type": "box", "size": t["bracket_size_m"]}, True)):
        out.append({"id": oid, "link": cell.prefix + link, "frame_id": cell.prefix + link, "pose": ident,
                    "shapes": [dict(shape, pose=ident)], "estimated": est, "in_urdf": True})
    for link in cell.gripper_spec["links"]:      # gripper: mesh links of its model (shapes: GET /robot/description)
        out.append({"id": link, "link": cell.prefix + link, "frame_id": cell.prefix + link, "pose": ident,
                    "shapes": [], "model": cell.gripper_model, "estimated": False, "in_urdf": True})
    return out


def _world_boxes(cell):
    ops = []
    for b in cell.world_boxes:
        obj = {"id": b["name"], "frame_id": cell.base_frame,
               "pose": {"position": [float(v) for v in b["center"]], "orientation": [0.0, 0.0, 0.0, 1.0]},
               "shapes": [{"type": "box", "size": [float(v) for v in b["size"]],
                           "pose": {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}}]}
        ops.append(("add", b["name"], obj))
    if not ops:
        return True
    for _ in range(6):
        try:
            C.scene_io.apply(ops, C.core.mesh_cache)
            if set(cell.builtin_world) <= set(C.scene_io.world_names()):
                return True
        except Exception as e:      # noqa: BLE001
            log.warning("built-in world geometry: %s", e)
        time.sleep(1.0)
    return False


def build():
    from camera import make_camera
    from cell import Cell
    from dashboard import Dashboard
    from file_store import FileStore
    from force import ForceMonitor
    from gripper import make_gripper
    from kinematics import Kinematics
    from legacy_scene import LegacyScene
    from motion import Motion
    from payload import PayloadIdentifier
    from ros_io import RosIO
    from scene_core import SceneCore
    from scene_io import MoveItSceneIO

    C.mock = os.environ.get("UR_MOCK_HARDWARE", "0") == "1"
    C.cell = cell = Cell()
    C.ros = RosIO(cell)
    log.info("waiting for /robot_description and /robot_description_semantic ...")
    if not C.ros.wait_for_description(60.0):
        raise SystemExit("no /robot_description within 60 s: is ros/launch/cell.launch.py running (same ROS_DOMAIN_ID)?")
    C.kin = Kinematics(C.ros.urdf, cell)
    cell.joint_limits = {n: (float(lo), float(hi)) for n, lo, hi in zip(cell.arm_joints, C.kin.lo, C.kin.hi)}
    cell.planning_frame = _root_link(C.kin.urdf)
    T = C.kin.urdf.fk(cell.base_frame, {}, cell.planning_frame)
    if abs(T - __import__("numpy").eye(4)).max() > 1e-9:
        raise SystemExit(f"{cell.planning_frame} -> {cell.base_frame} is not the identity; the API assumes it is")
    log.info("waiting for /joint_states ...")
    t0 = time.time()
    while C.ros.arm_q() is None and time.time() - t0 < 60:
        time.sleep(0.1)
    if C.ros.arm_q() is None:
        raise SystemExit("no /joint_states for the arm within 60 s")
    C.dash = None if C.mock else Dashboard(cell.robot_ip)
    C.motion = Motion(C.ros, cell, C.kin, C.dash, C.mock)
    C.gripper = make_gripper(cell, C.ros, C.dash, C.mock)
    C.force = ForceMonitor(C.ros, cell)
    C.payload = PayloadIdentifier(C.motion, C.force.get_force)
    C.camera = make_camera(cell.camera)
    state_dir = os.environ.get("UR_STATE_DIR", os.path.join(ROOT, "state"))
    os.makedirs(state_dir, exist_ok=True)
    C.store = FileStore(os.environ.get("UR_FILES_ROOT", os.path.join(ROOT, "data")),
                        max_file_bytes=int(os.environ.get("UR_FILES_MAX_BYTES", 100 * 2 ** 20)),
                        quota_bytes=int(os.environ.get("UR_FILES_QUOTA_BYTES", 2 * 2 ** 30)))
    C.scene_io = MoveItSceneIO(C.ros)
    C.core = SceneCore(C.scene_io, C.store, os.path.join(state_dir, "scene_v2.json"), cell, logger=log.warning)
    C.core.robot_links = lambda: list(C.kin.urdf.links)
    C.core.builtin = _builtin_tool(cell)
    C.core.builtin_colors = {b: [0.55, 0.45, 0.3, 0.6] for b in cell.builtin_world}
    C.legacy = LegacyScene(C.scene_io, C.store, os.path.join(state_dir, "scene.json"), cell, C.core, logger=log.warning)
    C.core.legacy = C.legacy
    if not _world_boxes(cell):
        log.error("built-in world geometry (%s) NOT confirmed in the planning scene!", cell.builtin_world)
    log.info("legacy scene restored: %s", C.legacy.restore())
    log.info("scene v2 restored: %s", C.core.load_and_restore())
    C.core.send_colors()
    C.core.start_color_keeper()
    log.info("ready: %s, planning frame %s, tcp %s, gripper %s, camera %s%s", cell.ur_type, cell.planning_frame,
             cell.tcp_link, C.gripper.backend, cell.camera["backend"], " (MOCK HARDWARE)" if C.mock else "")


def register():
    import routes_io
    import routes_plan
    import routes_robot
    import routes_scene
    for bp in (routes_robot.bp, routes_scene.bp, routes_plan.bp, routes_io.bp):
        app.register_blueprint(bp)


def main():
    build()
    register()
    host, port = os.environ.get("UR_API_HOST", "0.0.0.0"), int(os.environ.get("UR_API_PORT", "5000"))

    def hard_stop(*_):
        try:
            C.motion.stop()
        finally:
            os._exit(0)
    signal.signal(signal.SIGINT, hard_stop)
    signal.signal(signal.SIGTERM, hard_stop)
    log.info("serving on http://%s:%d (GET /help)", host, port)
    app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
