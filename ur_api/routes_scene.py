"""/scene/*: the legacy model/object API and scene API v2. None of these move the robot or the gripper
(safety rule S1); changes take the motion lock (wait up to 5 s, else 409 busy), so the world never changes
under a running plan."""
import json
import os

from flask import Blueprint, jsonify, request

from ctx import C
from errors import err
from scene_core import API_VERSION, SHAPE_TYPES

bp = Blueprint("scene", __name__)
MESH_FORMATS = [".stl", ".obj", ".dae", ".ply"]


def _body():
    d = request.get_json(silent=True)
    if d is None:
        d = request.values.to_dict()
    if not isinstance(d, dict):
        raise err(400, "bad_request", "JSON body must be an object")
    return d


def _json_body():
    """JSON with our own size cap (a global MAX_CONTENT_LENGTH would also cap the 100 MB /files uploads)."""
    limit = C.core.limits["max_body_bytes"]
    if request.content_length is not None and request.content_length > limit:
        raise err(413, "too_large", f"body exceeds {limit} bytes")
    raw = request.stream.read(limit + 1)
    if len(raw) > limit:
        raise err(413, "too_large", f"body exceeds {limit} bytes")
    try:
        return json.loads(raw.decode("utf-8") or "null")
    except (UnicodeDecodeError, ValueError) as e:
        raise err(400, "bad_request", f"body is not valid JSON: {e}")


def _flag(d, key, default=False):
    v = d.get(key, request.args.get(key))
    if v is None:
        return default
    return v if isinstance(v, bool) else str(v) not in ("0", "false", "no", "")


def _int_arg(name):
    v = request.args.get(name)
    if v is None:
        return None
    try:
        return int(v)
    except ValueError:
        raise err(400, "bad_request", f"{name} must be an integer")


class _Locked:
    def __enter__(self):
        if not C.lock.acquire(timeout=5.0):
            raise err(409, "busy", "Robot is busy; scene change refused (retry shortly)")

    def __exit__(self, *exc):
        C.lock.release()


# ------------------------------------------------------------------ legacy: models

@bp.route("/scene/models", methods=["GET"])
def models_list():
    """Registered CAD models with triangle counts, bounding boxes and the objects that use them."""
    return jsonify({"models": C.legacy.list_models()}), 200


@bp.route("/scene/models", methods=["POST"])
def models_register():
    """Register a mesh from /files as a named model: name, file (e.g. cad/part.stl), scale=1 (0.001 for mm STLs),
    collision=mesh|convex_hull, overwrite=0. Re-registering with overwrite=1 refreshes every object using it."""
    d = _body()
    if not d.get("name") or not d.get("file"):
        raise err(400, "bad_request", "name and file are required")
    with _Locked():
        info = C.legacy.register_model(d["name"], d["file"], d.get("scale", 1.0), d.get("collision", "mesh"),
                                       overwrite=_flag(d, "overwrite"))
    return jsonify({"status": "registered", **info}), 201


@bp.route("/scene/models/<name>", methods=["GET"])
def models_get(name):
    return jsonify(C.legacy.get_model(name)), 200


@bp.route("/scene/models/<name>", methods=["DELETE"])
def models_delete(name):
    """Delete a model; 409 while objects use it unless force=1 (removes them too)."""
    with _Locked():
        C.legacy.delete_model(name, force=_flag({}, "force"))
    return jsonify({"status": "deleted", "name": name}), 200


# ------------------------------------------------------------------ legacy: objects

@bp.route("/scene/objects", methods=["GET"])
def objects_list():
    """Everything in the planning scene: model instances (model, pose, source, attached_link) and built-ins
    (table, walls). With namespace=<ns> and/or geometry=1 the v2 format: {scene_version, objects:[{id, namespace,
    frame_id, pose, attached_link, meta, shapes (geometry=1)}]} over batch, legacy and built-in objects."""
    if "namespace" in request.args or "geometry" in request.args:
        ns = request.args.get("namespace") or None
        geometry = request.args.get("geometry", "0") not in ("0", "false", "")
        try:
            world = C.scene_io.world_names()
        except Exception:      # noqa: BLE001
            world = None
        with C.core._lock:
            return jsonify({"scene_version": C.core.version,
                            "objects": C.core.list_objects(ns, geometry, world_names=world)}), 200
    return jsonify({"objects": C.legacy.list_objects()}), 200


@bp.route("/scene/objects", methods=["POST"])
def objects_add():
    """Place a model: name, model, x,y,z (m, base_link) and roll,pitch,yaw (deg) or qx,qy,qz,qw; frame=base_link,
    source=manual. The same name replaces the object."""
    d = _body()
    if not d.get("name") or not d.get("model"):
        raise err(400, "bad_request", "name and model are required")
    with _Locked():
        info = C.legacy.add_object(d["name"], d["model"], d, frame=d.get("frame"), source=d.get("source", "manual"))
    return jsonify({"status": "added", **info}), 201


@bp.route("/scene/objects/sync", methods=["PUT", "POST"])
def objects_sync():
    """Replace all objects of one source at once (a detector after every run): source, objects:[{name, model,
    x,y,z, roll,pitch,yaw | qx..qw, frame}]. Other sources, built-ins and attached objects are untouched;
    404 unknown_model lists models that still need POST /scene/models."""
    d = _body()
    with _Locked():
        res = C.legacy.sync(d.get("source"), d.get("objects"))
    return jsonify({"status": "synced", "source": d.get("source"), **res}), 200


@bp.route("/scene/objects/<name>", methods=["GET"])
def objects_get(name):
    return jsonify(C.legacy.get_object(name)), 200


@bp.route("/scene/objects/<name>", methods=["PATCH", "PUT"])
def objects_update(name):
    """Move an object: any of x,y,z, roll,pitch,yaw | qx..qw, frame (the rest keeps its value)."""
    d = _body()
    with _Locked():
        info = C.legacy.update_object(name, d, frame=d.get("frame"))
    return jsonify({"status": "updated", **info}), 200


@bp.route("/scene/objects/<name>", methods=["DELETE"])
def objects_delete(name):
    """Remove an object (force=1 for built-in safety geometry)."""
    with _Locked():
        C.legacy.remove_object(name, force=_flag({}, "force"))
    return jsonify({"status": "removed", "name": name}), 200


@bp.route("/scene/objects/<name>/attach", methods=["POST"])
def objects_attach(name):
    """Attach the object to a robot link (default rg6_body) so it moves with the gripper and is checked as part
    of it: link, touch_links (default the hand, fingers and TCP)."""
    d = _body()
    if name in C.core.objects:            # a /scene/batch object: one confirmed diff in the v2 core
        return jsonify({"status": "attached", **C.core.attach(name, d.get("link", C.cell.hand_link),
                                                              d.get("touch_links"), C.lock)}), 200
    with _Locked():
        info = C.legacy.attach(name, link=d.get("link", C.cell.hand_link), touch_links=d.get("touch_links"))
    return jsonify({"status": "attached", **info}), 200


@bp.route("/scene/objects/<name>/detach", methods=["POST"])
def objects_detach(name):
    """Detach the object; it stays in the world where the hand left it."""
    if name in C.core.objects:
        return jsonify({"status": "detached", **C.core.detach(name, C.lock)}), 200
    with _Locked():
        info = C.legacy.detach(name)
    return jsonify({"status": "detached", **info}), 200


# ------------------------------------------------------------------ v2

def robot_info():
    import hashlib
    sha = lambda s: hashlib.sha256(s.encode()).hexdigest() if s else None  # noqa: E731
    return {"moveit_version": _moveit_version(), "planning_frame": C.cell.planning_frame,
            "base_frame": C.cell.base_frame, "robot_name": C.kin.urdf.name, "urdf_sha256": sha(C.ros.urdf),
            "srdf_sha256": sha(C.ros.srdf), "groups": {"arm": C.cell.group, "hand": "gripper"},
            "ee_link": C.cell.tcp_link,
            f"planning_frame_to_{C.cell.base_frame}": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}}


def _moveit_version():
    try:
        import xml.etree.ElementTree as ET
        from ament_index_python.packages import get_package_share_directory
        return ET.parse(os.path.join(get_package_share_directory("moveit_core"), "package.xml")).find("version").text
    except Exception:      # noqa: BLE001
        return None


@bp.route("/scene/info", methods=["GET"])
def scene_info():
    """Scene API v2 capabilities and state: frames, robot description hashes, groups, scene version + hash +
    per-namespace hashes, dirty flag, limits, shape types, mesh formats, the exact hash definition, the tool
    model (TCP offset, built-in tool parts). Call it before pushing."""
    info = {"api_version": API_VERSION, **robot_info()}
    with C.core._lock:
        info.update(scene_version=C.core.version, scene_hash=C.core.scene_hash(), namespace_hashes=C.core.namespaces(),
                    dirty=C.core.dirty, dirty_namespaces=sorted(C.core.dirty_namespaces),
                    dirty_reason=C.core.dirty_reason, updated_at=C.core.updated_at)
    info.update({
        "motion_enabled": True, "mock_hardware": C.mock, "builtin": C.core.builtin,
        "builtin_attached": [b["id"] for b in C.core.builtin], "builtin_world": list(C.cell.builtin_world),
        "flange_to_tcp_m": C.cell.tcp_z, "acm_client_entries": len(C.core.acm_entries), "padding": C.core.padding,
        "limits": dict(C.core.limits), "shape_types": list(SHAPE_TYPES), "mesh_formats": MESH_FORMATS,
        "frames_accepted": [C.cell.base_frame, C.cell.planning_frame],
        "hash_spec": "scene_hash = sha256 of {objects: [per-object dicts], acm: [[a,b,allowed]...] sorted (client "
                     "entries), padding: {default_m, links} (%.6f strings) | null}; namespace_hash = sha256 of the "
                     "per-object list only. Per object, sorted by id: {id, frame_id, pose:{position:[%.6f x3], "
                     "orientation:[%.9f x4, normalised, w>=0]}, attached_link, shapes:[{type, dims:[%.6f...] (box "
                     "[x,y,z], sphere [r], cylinder [r,length], mesh [scale]), pose, mesh_sha256|null}]}; numbers as "
                     "strings, '-0.0..' without sign; json sort_keys, separators (',',':'); 'sha256:'+hex"})
    return jsonify(info), 200


@bp.route("/scene/batch", methods=["POST"])
def scene_batch():
    """Atomic update of one namespace: validated completely first (any error -> 4xx, scene unchanged), then ONE
    PlanningScene diff, confirmed by reading the scene back; on failure the previous state is re-applied (or the
    namespace is marked dirty -> 504). Body: {namespace, mode: replace|upsert|remove, objects:[{id, frame_id,
    pose:{position, orientation:[qx,qy,qz,qw]}, shapes:[{type: box(size)|sphere(radius)|cylinder(radius,length)|
    mesh(file,scale,sha256?), pose}], meta, color}], ids (remove), if_version, validate_only, keep_attached, meta}.
    Identical content -> unchanged:true, version unchanged. Never moves the robot."""
    req = C.core.validate(_json_body())             # meshes are parsed here, outside the lock
    return jsonify(C.core.batch(req, C.lock)), 200


@bp.route("/scene/namespaces/<ns>", methods=["DELETE"])
def scene_delete_namespace(ns):
    """Remove every object of a namespace (and its sub-namespaces), batch and legacy, in one confirmed diff.
    if_version=, force_detach=1 also drops attached objects. A successful delete clears the dirty flag."""
    return jsonify(C.core.delete_namespace(ns, C.lock, force_detach=_flag({}, "force_detach"),
                                           if_version=_int_arg("if_version"))), 200


@bp.route("/scene/namespaces/", methods=["DELETE"])
def scene_delete_namespace_empty():
    raise err(400, "bad_request", "namespace must not be empty")


@bp.route("/scene/acm", methods=["GET"])
def scene_acm_get():
    """Allowed-collision matrix: allowed pairs (names=a,b,... to filter), which ones clients wrote, defaults."""
    names = [n for n in request.args.get("names", "").split(",") if n]
    with C.core._lock:
        return jsonify(C.core.acm_get(names or None)), 200


@bp.route("/scene/acm", methods=["POST"])
def scene_acm_post():
    """Write ACM entries {entries:[{a, b, allowed}], if_version}; a/b are robot links or scene objects. The
    original value of each pair is remembered for DELETE; entries of removed objects are dropped."""
    d = _json_body()
    if not isinstance(d, dict):
        raise err(400, "bad_request", "body must be a JSON object")
    return jsonify(C.core.acm_set(d.get("entries"), C.lock, if_version=d.get("if_version"))), 200


@bp.route("/scene/acm", methods=["DELETE"])
def scene_acm_delete():
    """Restore client-written ACM entries (all, or namespace=<ns>); SRDF/default entries are never touched."""
    return jsonify(C.core.acm_delete(C.lock, namespace=request.args.get("namespace") or None,
                                     if_version=_int_arg("if_version"))), 200


@bp.route("/scene/padding", methods=["GET"])
def scene_padding_get():
    """Robot link padding: the configured {default_m, links} and what move_group uses now."""
    return jsonify({"padding": C.core.padding, "applied": C.scene_io.get_padding(),
                    "scene_version": C.core.version}), 200


@bp.route("/scene/padding", methods=["POST"])
def scene_padding_post():
    """Set robot link padding (robot vs world): {default_m: 0.01, links: {"rg6_left_finger": 0.003}, if_version},
    0..0.1 m; persisted and re-applied after a restart. Objects are not padded here."""
    d = _json_body()
    if not isinstance(d, dict) or "default_m" not in d:
        raise err(400, "bad_request", "body needs default_m")
    return jsonify(C.core.padding_set(d["default_m"], d.get("links"), C.lock, if_version=d.get("if_version"))), 200


@bp.route("/scene/unreferenced", methods=["GET"])
def scene_unreferenced():
    """Files under prefix= (pipefit/meshes) no object or model uses; recent ones (older_than_s=600) are protected."""
    try:
        older = float(request.args.get("older_than_s", "600"))
    except ValueError:
        raise err(400, "bad_request", "older_than_s must be a number")
    files = C.core.unreferenced(request.args.get("prefix", "pipefit/meshes"), older)
    return jsonify({"files": files, "count": len(files), "bytes": sum(f["size"] for f in files)}), 200


@bp.route("/scene/cleanup", methods=["POST"])
def scene_cleanup():
    """Delete unreferenced, unprotected mesh files (and with models=true unused legacy models). JSON: prefix,
    older_than_s (600), dry_run (TRUE by default -- set false to delete), models (false)."""
    d = _json_body() or {}
    if not isinstance(d, dict):
        raise err(400, "bad_request", "body must be a JSON object")
    try:
        older = float(d.get("older_than_s", 600))
    except (TypeError, ValueError):
        raise err(400, "bad_request", "older_than_s must be a number")
    return jsonify(C.core.cleanup(d.get("prefix", "pipefit/meshes"), older, dry_run=d.get("dry_run", True) is not False,
                                  models=bool(d.get("models", False)))), 200


@bp.route("/scene/snapshot", methods=["GET"])
def scene_snapshot():
    """The complete canonical scene: objects with geometry and meta, client ACM entries, padding, versions and
    hashes (namespace=<ns> to restrict; adds namespace_hash)."""
    ns = request.args.get("namespace") or None
    if ns:
        C.core.check_namespace(ns)
    with C.core._lock:
        return jsonify(C.core.snapshot(ns)), 200


if os.environ.get("UR_API_TEST_HOOKS") == "1":         # never in production
    @bp.route("/scene/_test/fail_next_apply", methods=["POST"])
    def _test_fail():
        C.core.test_hooks["fail_next_apply"] = int(request.args.get("n", "1"))
        return jsonify(C.core.test_hooks), 200

    @bp.route("/scene/_test/delay_next_apply", methods=["POST"])
    def _test_delay():
        C.core.test_hooks["delay_next_apply_s"] = float(request.args.get("s", "3"))
        return jsonify(C.core.test_hooks), 200
