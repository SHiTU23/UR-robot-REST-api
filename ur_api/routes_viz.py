"""Display-only overlays in RViz (added 2026-11-21): boxes a client wants to SEE on the robot or in the cell, e.g. the
collision shapes the client computes with itself (pipeFit's hand body, the red box of its AnyGrasp tab), to compare them
with the robot model. They are RViz markers on /ur_cell_overlays (MarkerArray, latched), NOT planning scene objects:
nothing here is ever collision-checked, and the robot never moves.

    PUT    /viz/overlays/<id>   {frame_id, boxes: [{size [x,y,z] m, position [x,y,z] m, orientation [qx,qy,qz,qw]?}],
                                 color [r,g,b,a]?, label?}   frame_id = any robot link (tcp, tool0, base_link, ...): a box
                                 on tcp moves with the gripper. Replaces the overlay of that id.
    GET    /viz/overlays        all overlays
    DELETE /viz/overlays/<id>   remove one

Kept in state/overlays.json and shown again after an API restart. RViz: the "Overlays" MarkerArray display (cell.rviz)."""
import json
import math
import os
import re
import threading

from flask import Blueprint, jsonify, request

from ctx import C
from errors import err

bp = Blueprint("viz", __name__)
TOPIC = "/ur_cell_overlays"
FILE = os.path.join(os.environ.get("UR_STATE_DIR") or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "state"),
                    "overlays.json")
MAX_BOXES = 50
_lock = threading.Lock()
_pub = {"p": None}


def _load():
    try:
        with open(FILE) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _save(d):
    os.makedirs(os.path.dirname(FILE), exist_ok=True)
    tmp = FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=1)
    os.replace(tmp, FILE)


# (2026-11-21) the robot's box colliders, always drawn: read from the robot model MoveIt checks (C.ros.urdf), in their own links,
# so they follow the hand-eye mount and the gripper. Not stored, not removable through the API.
ROBOT_COLLIDERS = {"wrist_camera": ("camera body", [0.1, 0.55, 1.0, 0.35]),
                   "camera_bracket": ("camera mount plate", [1.0, 0.85, 0.0, 0.5]),
                   "gripper_collider": ("gripper at the allowed opening", [1.0, 0.45, 0.0, 0.3])}


def robot_colliders() -> dict:
    """{robot_colliders.<link>: overlay} of the collision boxes of ROBOT_COLLIDERS' links in the live robot model"""
    import xml.etree.ElementTree as ET
    xml = getattr(C.ros, "urdf", None)
    if not xml:
        return {}
    out = {}
    for link in ET.fromstring(xml).findall("link"):
        name = link.get("name")
        key = "gripper_collider" if name.startswith("gripper_collider_") else name
        if key not in ROBOT_COLLIDERS:
            continue
        boxes = []
        for c in link.findall("collision"):
            g = c.find("geometry/box")
            if g is None:
                continue
            o = c.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            r, p, y = (float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split())
            cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
            q = [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]
            boxes.append({"size": [float(v) for v in g.get("size").split()], "position": xyz, "orientation": q})
        if boxes:
            label, color = ROBOT_COLLIDERS[key]
            out[f"robot_colliders.{name}"] = {"frame_id": name, "boxes": boxes, "color": color, "label": label}
    return out


def publish():
    """every overlay as one latched MarkerArray (DELETEALL first, so removed ones go). Stamp 0: RViz draws each box at the
    newest TF of its frame, so a box on tcp follows the gripper"""
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
    from ros_io import _latched
    if _pub["p"] is None:
        _pub["p"] = C.ros.node.create_publisher(MarkerArray, TOPIC, _latched())
    out = [Marker(action=Marker.DELETEALL)]
    for oid, o in sorted({**_load(), **robot_colliders()}.items()):
        c = o.get("color") or [1.0, 0.0, 0.0, 0.35]
        for k, b in enumerate(o["boxes"]):
            m = Marker(ns=oid, id=k, type=Marker.CUBE, action=Marker.ADD)
            m.header.frame_id = o["frame_id"]
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in b["position"])
            q = b.get("orientation") or [0.0, 0.0, 0.0, 1.0]
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(v) for v in q)
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in b["size"])
            m.color = ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(c[3]))
            out.append(m)
            e = Marker(ns=oid + "/edges", id=k, type=Marker.LINE_LIST, action=Marker.ADD)   # the edges, so the box reads in front of the gripper
            e.header.frame_id, e.pose = o["frame_id"], m.pose
            e.scale.x = 0.0015
            e.color = ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0)
            hx, hy, hz = (float(v) / 2 for v in b["size"])
            corners = [(sx * hx, sy * hy, sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
            for i, a in enumerate(corners):
                for j, bb in enumerate(corners):
                    if i < j and sum(x != y for x, y in zip(a, bb)) == 1:
                        e.points += [Point(x=a[0], y=a[1], z=a[2]), Point(x=bb[0], y=bb[1], z=bb[2])]
            out.append(e)
    _pub["p"].publish(MarkerArray(markers=out))
    return len(out) - 1


def _nums(v, n, path, lo=-10.0, hi=10.0):
    if not (isinstance(v, list) and len(v) == n and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                                                         and math.isfinite(x) and lo <= x <= hi for x in v)):
        raise err(400, "bad_request", f"{path} must be {n} numbers in {lo}..{hi}", path)
    return [float(x) for x in v]


@bp.route("/viz/overlays", methods=["GET"])
def overlays_list():
    return jsonify({"topic": TOPIC, "overlays": _load()}), 200


@bp.route("/viz/overlays/<oid>", methods=["PUT", "POST"])
def overlay_put(oid):
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", oid):
        raise err(400, "bad_request", "the id is 1-64 letters, digits, _ . -", "id")
    d = request.get_json(silent=True)
    if not isinstance(d, dict):
        raise err(400, "bad_request", "JSON body must be an object")
    frame = d.get("frame_id") or C.cell.base_frame
    if frame not in C.kin.urdf.links and frame not in (C.cell.base_frame, C.cell.planning_frame, "world"):
        raise err(400, "bad_frame", f"frame_id {frame!r} is not a robot link", "frame_id")
    boxes = d.get("boxes")
    if not isinstance(boxes, list) or not 1 <= len(boxes) <= MAX_BOXES:
        raise err(400, "bad_request", f"boxes is a list of 1..{MAX_BOXES}", "boxes")
    clean = []
    for i, b in enumerate(boxes):
        if not isinstance(b, dict):
            raise err(400, "bad_request", "each box is an object", f"boxes[{i}]")
        box = {"size": _nums(b.get("size"), 3, f"boxes[{i}].size", 0.0005, 5.0), "position": _nums(b.get("position"), 3, f"boxes[{i}].position")}
        if b.get("orientation") is not None:
            q = _nums(b["orientation"], 4, f"boxes[{i}].orientation", -1.0, 1.0)
            n = math.sqrt(sum(x * x for x in q))
            if n < 1e-6:
                raise err(400, "bad_request", "orientation is a unit quaternion", f"boxes[{i}].orientation")
            box["orientation"] = [x / n for x in q]
        clean.append(box)
    o = {"frame_id": frame, "boxes": clean, "label": str(d.get("label") or "")[:200]}
    if d.get("color") is not None:
        o["color"] = _nums(d["color"], 4, "color", 0.0, 1.0)
    with _lock:
        all_ = _load()
        all_[oid] = o
        _save(all_)
        n = publish()
    return jsonify({"id": oid, "overlay": o, "markers": n, "topic": TOPIC, "note": "display only: not in the planning scene"}), 200


@bp.route("/viz/overlays/<oid>", methods=["DELETE"])
def overlay_delete(oid):
    with _lock:
        all_ = _load()
        gone = all_.pop(oid, None) is not None
        _save(all_)
        publish()
    return jsonify({"id": oid, "removed": gone}), 200


# (2026-11-21) a client's joint path, animated in RViz's "Planned path" display (/display_planned_path, loops): e.g. pipeFit's pick plan
# (waypoints it checked itself, never a MoveIt plan) or the path of a candidate removed for a collision. Display only: nothing moves.
MAX_POINTS = 2000


@bp.route("/viz/trajectory", methods=["POST"])
def trajectory_show():
    """{points: [[6 joints rad], ...] (2..MAX_POINTS), widths: [gripper width m per point]?, dt: s between points (0.02..2, 0.25),
    label?} -> the path from the first point, with the fingers at each point's width"""
    from builtin_interfaces.msg import Duration
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    d = request.get_json(silent=True)
    if not isinstance(d, dict):
        raise err(400, "bad_request", "JSON body must be an object")
    pts = d.get("points")
    if not isinstance(pts, list) or not 2 <= len(pts) <= MAX_POINTS:
        raise err(400, "bad_request", f"points is a list of 2..{MAX_POINTS} joint vectors", "points")
    n = C.cell.n_joints
    q = [_nums(p, n, f"points[{i}]", -7.0, 7.0) for i, p in enumerate(pts)]
    widths = d.get("widths")
    if widths is not None:
        if not isinstance(widths, list) or len(widths) != len(q):
            raise err(400, "bad_request", "widths has one value per point", "widths")
        widths = _nums(widths, len(q), "widths", 0.0, C.cell.max_width)
    dt = float(d.get("dt") or 0.25)
    if not 0.02 <= dt <= 2.0:
        raise err(400, "bad_request", "dt is 0.02..2 s", "dt")
    names = list(C.cell.arm_joints) + ([C.cell.finger_joint] if widths is not None else [])
    jt = JointTrajectory(joint_names=names)
    for i, p in enumerate(q):
        pos = list(p) + ([C.cell.finger_q(widths[i])[C.cell.finger_joint]] if widths is not None else [])
        t = dt * i
        jt.points.append(JointTrajectoryPoint(positions=pos, time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
    C.motion.display(RobotTrajectory(joint_trajectory=jt),
                     C.motion.robot_state(q[0], widths[0] if widths is not None else None))
    return jsonify({"shown": len(q), "seconds": round(dt * (len(q) - 1), 2), "label": str(d.get("label") or "")[:200],
                    "topic": "/display_planned_path", "note": "display only: nothing moves"}), 200


@bp.route("/robot/colliders", methods=["GET"])
def colliders_get():
    """(2026-11-21) the robot's box colliders (ROBOT_COLLIDERS: camera body, camera mount plate, gripper boxes) as they are at a
    gripper opening: ?width= m (default the allowed opening, allowed_max_width_m), ?frame= any robot link (default tcp).
    -> {frame, width_m, boxes: [{link, kind, label, color, size [x,y,z] m, T [4x4] box centre in frame}]} from the robot model
    MoveIt checks; a client draws them or measures with them (pipeFit's 3-D view)"""
    import numpy as np
    import xml.etree.ElementTree as ET
    frame = request.args.get("frame") or C.cell.tcp_link
    if frame not in C.kin.urdf.links:
        raise err(400, "bad_frame", f"frame {frame!r} is not a robot link", "frame")
    spec = C.cell.gripper_spec
    w = request.args.get("width")
    try:
        w = float(w) if w not in (None, "") else float(spec.get("allowed_max_width_m", spec["max_width_m"]))
    except ValueError:
        raise err(400, "bad_request", "width is a number (m)", "width") from None
    if not 0.0 <= w <= C.cell.max_width:
        raise err(400, "bad_request", f"width is 0..{C.cell.max_width} m", "width")
    q = C.cell.finger_q(w)
    root = C.cell.base_frame                       # Urdf.fk walks down the tree only: both through the base (the arm at 0, it cancels)
    T_frame_root = np.linalg.inv(C.kin.urdf.fk(frame, q, root))
    boxes = []
    for oid, o in robot_colliders().items():
        link = o["frame_id"]
        T_f_link = T_frame_root @ C.kin.urdf.fk(link, q, root)
        key = "gripper_collider" if link.startswith("gripper_collider_") else link
        for b in o["boxes"]:
            x, y, z, s_ = b["orientation"]
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * s_), 2 * (x * z + y * s_)],
                          [2 * (x * y + z * s_), 1 - 2 * (x * x + z * z), 2 * (y * z - x * s_)],
                          [2 * (x * z - y * s_), 2 * (y * z + x * s_), 1 - 2 * (x * x + y * y)]])
            T_b = np.eye(4)
            T_b[:3, :3], T_b[:3, 3] = R, b["position"]
            boxes.append({"link": link, "kind": key, "label": o["label"], "color": o["color"], "size": b["size"],
                          "T": np.round(T_f_link @ T_b, 6).tolist()})
    return jsonify({"frame": frame, "width_m": w, "gripper": C.cell.gripper_model, "boxes": boxes}), 200
