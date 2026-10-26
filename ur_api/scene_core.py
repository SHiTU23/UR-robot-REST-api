"""Namespaced, versioned collision scene (scene API v2: /scene/batch & co).

Object ids look like ``pipefit.<scene>.<object>``; the namespace is everything
before the last dot and is hierarchical (``pipefit`` covers ``pipefit.*``).
An object is a list of shapes (box / sphere / cylinder / mesh), each posed
relative to the object pose.

A batch is validated completely (ids, frames, poses, shapes, mesh files,
limits) before anything is sent to MoveIt, then committed as ONE
PlanningScene diff whose result is confirmed by reading the scene back. On a
failed commit the previous state of every touched object is re-applied; if
that also fails, the namespace is marked dirty and the motion endpoints
refuse to run until a later batch or delete on it succeeds.

Hash (the client computes the same value) -- per object, sorted by id:
    {"id", "frame_id",
     "pose": {"position": [%.6f x3], "orientation": [%.9f x4]},   quaternion normalised, w >= 0
     "attached_link": str | null,
     "shapes": [{"type", "dims": [%.6f ...], "pose": {...}, "mesh_sha256": str | null}]}
dims: box [x, y, z], sphere [r], cylinder [r, length], mesh [scale]. Numbers
are printf-formatted STRINGS, a negative zero is written without its sign;
json with sort_keys and separators (",", ":"); "sha256:" + hex digest.
namespace_hash covers that object list only; scene_hash covers
{"objects", "acm": sorted [[a, b, allowed]] client entries,
 "padding": {"default_m", "links"} as %.6f strings | null}.

ROS access goes through ``io`` (ros_io.MoveItSceneIO; tests pass a fake).
"""
import copy
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

from errors import ApiError, err
from file_store import FileStoreError

API_VERSION = "2.0"
ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
NS_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*(\.[A-Za-z0-9][A-Za-z0-9_-]*)*$")
RESERVED_NS = ("builtin", "manual")
SHAPE_TYPES = ("box", "sphere", "cylinder", "mesh")
MODES = ("replace", "upsert", "remove")
MESH_EXTENSIONS = (".stl", ".obj", ".dae", ".ply")
MAX_DIM_M = 10.0
MESH_PARSE_TIMEOUT_S = 60
MESH_CACHE_MAX_TRIANGLES = 3_000_000

DEFAULT_LIMITS = {
    "max_body_bytes": 8 * 2 ** 20,
    "max_objects_per_batch": 500,
    "max_shapes_per_object": 1024,
    "max_primitives_total": 20000,
    "max_triangles_per_mesh": 20000,
    "max_triangles_total": 300000,
    "max_id_len": 64,
    "max_batch_meta_bytes": 16 * 1024,
    "max_object_meta_bytes": 4 * 1024,
}
IDENTITY_POSE = {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}


def now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


# ------------------------------------------------------------------ meshes

def parse_mesh_isolated(abs_path):
    """(vertices N x 3, faces M x 3) from mesh_parse.py in a subprocess, so a
    loader crash on a malformed upload kills only the child."""
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mesh_parse.py")
    fd, out = tempfile.mkstemp(suffix=".npz")
    os.close(fd)
    try:
        try:
            p = subprocess.run([sys.executable, script, abs_path, out], capture_output=True, text=True,
                               timeout=MESH_PARSE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            raise err(422, "mesh_timeout", f"parsing took longer than {MESH_PARSE_TIMEOUT_S} s")
        if p.returncode < 0:
            raise err(422, "mesh_unreadable", f"the mesh parser crashed (signal {-p.returncode}); "
                                              "the file is malformed or unsupported")
        if p.returncode != 0:
            raise err(422, "mesh_unreadable", (p.stderr.strip().splitlines() or ["mesh parser failed"])[-1])
        with np.load(out) as d:
            return d["vertices"].astype(float), d["faces"].astype(np.int64)
    finally:
        try:
            os.unlink(out)
        except OSError:
            pass


def convex_hull(verts):
    """(verts, faces) of the convex hull, re-indexed to the used vertices."""
    from scipy.spatial import ConvexHull
    h = ConvexHull(verts)
    used = np.unique(h.simplices)
    remap = np.full(len(verts), -1, np.int64)
    remap[used] = np.arange(len(used))
    return verts[used], remap[h.simplices]


# ----------------------------------------------------------------- numbers

def _num(v, path, positive=False, upper=None):
    code = "bad_shape" if positive else "bad_pose"
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise err(400, code, "must be a number", path)
    v = float(v)
    if not math.isfinite(v):
        raise err(400, code, "must be finite", path)
    if positive and v <= 0:
        raise err(400, "bad_shape", "must be > 0", path)
    if upper is not None and v > upper:
        raise err(400, "bad_shape", f"must be <= {upper}", path)
    return v


def fmt(v, digits):
    """printf %.<digits>f without a sign on zero ("-0.000" -> "0.000")."""
    s = "%.*f" % (digits, v)
    return s[1:] if s.startswith("-") and float(s) == 0.0 else s


def canon_pose(d, path):
    """{"position": [3], "orientation": [qx, qy, qz, qw]} -> normalised, w >= 0."""
    if d is None:
        return copy.deepcopy(IDENTITY_POSE)
    if not isinstance(d, dict):
        raise err(400, "bad_pose", "pose must be an object {position, orientation}", path)
    extra = set(d) - {"position", "orientation"}
    if extra:
        raise err(400, "bad_pose", f"unknown pose fields {sorted(extra)} (rpy is not accepted; use a quaternion)",
                  path)
    pos, ori = d.get("position", [0, 0, 0]), d.get("orientation", [0, 0, 0, 1])
    if not isinstance(pos, list) or len(pos) != 3:
        raise err(400, "bad_pose", "position must be [x, y, z]", f"{path}.position")
    if not isinstance(ori, list) or len(ori) != 4:
        raise err(400, "bad_pose", "orientation must be [qx, qy, qz, qw]", f"{path}.orientation")
    p = [_num(v, f"{path}.position[{i}]") for i, v in enumerate(pos)]
    q = [_num(v, f"{path}.orientation[{i}]") for i, v in enumerate(ori)]
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-6:
        raise err(400, "bad_pose", "quaternion norm < 1e-6", f"{path}.orientation")
    q = [v / n for v in q]
    return {"position": p, "orientation": [-v for v in q] if q[3] < 0 else q}


# ------------------------------------------------------------------- hash

def _hash_pose(p):
    return {"position": [fmt(v, 6) for v in p["position"]], "orientation": [fmt(v, 9) for v in p["orientation"]]}


def shape_dims(s):
    return {"box": lambda: list(s["size"]), "sphere": lambda: [s["radius"]],
            "cylinder": lambda: [s["radius"], s["length"]], "mesh": lambda: [s["scale"]]}[s["type"]]()


def hash_object(o):
    return {"id": o["id"], "frame_id": o["frame_id"], "pose": _hash_pose(o["pose"]),
            "attached_link": o.get("attached_link") or None,
            "shapes": [{"type": s["type"], "dims": [fmt(v, 6) for v in shape_dims(s)], "pose": _hash_pose(s["pose"]),
                        "mesh_sha256": s.get("sha256") if s["type"] == "mesh" else None} for s in o["shapes"]]}


def _sha(payload):
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "sha256:" + hashlib.sha256(raw).hexdigest()


def hash_objects(objs):
    return _sha([hash_object(o) for o in sorted(objs, key=lambda o: o["id"])])


def namespace_of(oid):
    return oid.rsplit(".", 1)[0] if "." in oid else "manual"


def in_namespace(oid, ns):
    return oid.startswith(ns + ".")


def _json_size(v):
    return len(json.dumps(v, separators=(",", ":")).encode("utf-8"))


# ------------------------------------------------------------------- core

class SceneCore:
    """State, validation and commits of the v2 scene. ``cell`` gives the frame
    and the default touch links; ``io`` does every MoveIt call."""

    def __init__(self, io, store, state_path, cell, limits=None, logger=None):
        self.io, self.store, self.state_path, self.cell = io, store, state_path, cell
        self.planning_frame = cell.planning_frame
        self.limits = dict(DEFAULT_LIMITS, **(limits or {}))
        self.log = logger or (lambda *a: None)
        self.legacy = None                     # legacy_scene.LegacyScene (set by the app)
        self._lock = threading.RLock()
        self.mesh_cache = {}                   # sha256 -> (verts, faces), unscaled
        self._mesh_tris = 0
        self.version = 0
        self.dirty_namespaces = set()
        self.dirty_reason = None
        self.updated_at = None
        self.objects = {}                      # id -> canonical object
        self.batch_meta = {}
        self.acm_entries = {}                  # "a|b" -> {a, b, allowed, original}
        self.padding = None                    # {"default_m", "links"}
        self.builtin = []                      # built-in attached tool parts (see tool_model.py)
        self.builtin_colors = {}               # id -> rgba for built-in geometry (walls ...)
        self.robot_links = lambda: []
        self._recent_files = {}                # relpath -> last time a batch used it
        self.test_hooks = {"fail_next_apply": 0, "delay_next_apply_s": 0.0}

    # ---- dirty ----

    @property
    def dirty(self):
        return bool(self.dirty_namespaces)

    @staticmethod
    def _covers(ns, d):
        return d == ns or d.startswith(ns + ".")

    def _is_dirty(self, ns):
        return any(self._covers(ns, d) for d in self.dirty_namespaces)

    def _clear_dirty(self, ns):
        self.dirty_namespaces = {d for d in self.dirty_namespaces if not self._covers(ns, d)}
        if not self.dirty_namespaces:
            self.dirty_reason = None

    def mark_dirty(self, reason, namespaces):
        self.dirty_namespaces |= set(namespaces)
        self.dirty_reason = reason
        self.log("scene marked DIRTY (%s): %s" % (sorted(namespaces), reason))

    def _bump(self):
        self.version += 1
        self.updated_at = now_iso()
        self.save()

    # ---- persistence ----

    def save(self):
        os.makedirs(os.path.dirname(self.state_path), exist_ok=True)
        tmp = self.state_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"api_version": API_VERSION, "version": self.version,
                       "dirty_namespaces": sorted(self.dirty_namespaces), "dirty_reason": self.dirty_reason,
                       "acm_entries": self.acm_entries, "padding": self.padding, "updated_at": self.updated_at,
                       "objects": self.objects, "batch_meta": self.batch_meta}, f, indent=1, sort_keys=True)
        os.replace(tmp, self.state_path)

    def load_and_restore(self):
        """Read the state file and push everything back into move_group in one
        diff (it forgets its world on a restart). A failure marks the scene dirty."""
        if not os.path.isfile(self.state_path):
            return {"objects": 0, "restored": True}
        with open(self.state_path) as f:
            d = json.load(f)
        with self._lock:
            self.version = int(d.get("version", 0))
            self.updated_at = d.get("updated_at")
            self.objects = d.get("objects", {})
            self.batch_meta = d.get("batch_meta", {})
            self.dirty_namespaces = set(d.get("dirty_namespaces", []))
            self.dirty_reason = d.get("dirty_reason")
            self.acm_entries = d.get("acm_entries") or {}
            self.padding = d.get("padding")
            if not (self.objects or self.acm_entries or self.padding):
                return {"objects": 0, "restored": True, "dirty": self.dirty}
            try:
                for o in self.objects.values():
                    for i, s in enumerate(o["shapes"]):
                        if s["type"] == "mesh":
                            self._resolve_mesh(s["file"], s.get("sha256"), f"{o['id']}.shapes[{i}]")
                world = set(self.io.world_names())
                attached = {oid for oid, o in self.objects.items() if o.get("attached_link")}
                ops = [("remove", oid, None) for oid in self.objects if oid in world]
                ops += [("add", oid, o) for oid, o in self.objects.items()]
                ops += [("attach", oid, {"link": self.objects[oid]["attached_link"],
                                         "touch_links": self.objects[oid].get("touch_links")}) for oid in attached]
                self._apply_and_confirm(ops, set(self.objects) - attached, set(),
                                        acm=self._acm_msg_with(self.acm_entries) if self.acm_entries else None,
                                        padding=self._padding_map(self.padding) if self.padding else None,
                                        expect_attached=attached)
                self.send_colors()
            except ApiError as e:
                self.mark_dirty(f"restore failed: {e.msg}", {namespace_of(oid) for oid in self.objects})
                self.save()
                return {"objects": len(self.objects), "restored": False, "error": e.msg}
            return {"objects": len(self.objects), "restored": True, "dirty": self.dirty}

    # ---- meshes ----

    def _resolve_mesh(self, file, sha_expected, path):
        """Check a mesh reference, parse + cache it. -> (relpath, sha256, triangles)."""
        if not isinstance(file, str) or not file:
            raise err(400, "bad_shape", "mesh needs a file path in the /files store", f"{path}.file")
        try:
            ap = self.store.resolve(file)
        except FileStoreError as e:
            raise err(400, "bad_shape", e.msg, f"{path}.file")
        if not os.path.isfile(ap):
            raise err(404, "unknown_mesh", f"no file {file!r} in the /files store", f"{path}.file")
        if not ap.lower().endswith(MESH_EXTENSIONS):
            raise err(415, "unsupported_mesh", f"mesh must be one of {MESH_EXTENSIONS}", f"{path}.file")
        sha = self.store.sha256(ap)
        if sha_expected is not None and sha_expected != sha:
            raise err(409, "mesh_sha_mismatch", f"file sha256 is {sha}, request says {sha_expected}",
                      f"{path}.sha256")
        if sha not in self.mesh_cache:
            if os.path.getsize(ap) == 0:
                raise err(422, "mesh_unreadable", "the file is empty (0 bytes)", f"{path}.file")
            try:
                self.cache_mesh(sha, *parse_mesh_isolated(ap))
            except ApiError as e:
                raise err(422, e.code if e.code == "mesh_timeout" else "mesh_unreadable", e.msg, f"{path}.file")
        self._recent_files[self.store.relpath(ap)] = time.time()
        tris = len(self.mesh_cache[sha][1])
        if tris > self.limits["max_triangles_per_mesh"]:
            raise err(422, "limits_exceeded", f"mesh has {tris} triangles, limit is "
                      f"{self.limits['max_triangles_per_mesh']} (the server does not simplify meshes)", f"{path}.file")
        return self.store.relpath(ap), sha, tris

    def cache_mesh(self, sha, verts, faces):
        self.mesh_cache[sha] = (verts, faces)
        self._mesh_tris += len(faces)
        while self._mesh_tris > MESH_CACHE_MAX_TRIANGLES and len(self.mesh_cache) > 1:
            old = next(iter(self.mesh_cache))           # insertion order = age
            if old.startswith(("builtin-", "model:")):  # built-ins and registered models stay: move to the back
                self.mesh_cache[old] = self.mesh_cache.pop(old)
                if all(k.startswith(("builtin-", "model:")) for k in self.mesh_cache):
                    break
                continue
            self._mesh_tris -= len(self.mesh_cache.pop(old)[1])

    # ---- validation ----

    def _canon_shape(self, s, path):
        if not isinstance(s, dict):
            raise err(400, "bad_shape", "shape must be an object", path)
        t = s.get("type")
        if t not in SHAPE_TYPES:
            raise err(400, "bad_shape", f"type must be one of {SHAPE_TYPES}", f"{path}.type")
        out = {"type": t, "pose": canon_pose(s.get("pose"), f"{path}.pose")}
        if t == "box":
            size = s.get("size")
            if not isinstance(size, list) or len(size) != 3:
                raise err(400, "bad_shape", "box needs size [x, y, z]", f"{path}.size")
            out["size"] = [_num(v, f"{path}.size[{i}]", True, MAX_DIM_M) for i, v in enumerate(size)]
        elif t == "sphere":
            out["radius"] = _num(s.get("radius"), f"{path}.radius", True, MAX_DIM_M)
        elif t == "cylinder":
            out["radius"] = _num(s.get("radius"), f"{path}.radius", True, MAX_DIM_M)
            out["length"] = _num(s.get("length"), f"{path}.length", True, MAX_DIM_M)
        else:
            out["scale"] = _num(s.get("scale", 1.0), f"{path}.scale", True, 1000.0)
            sha = s.get("sha256")
            if sha is not None and not isinstance(sha, str):
                raise err(400, "bad_shape", "sha256 must be a hex string", f"{path}.sha256")
            rel, sha, tris = self._resolve_mesh(s.get("file"), sha, path)
            out.update(file=rel, sha256=sha, triangles=tris)
        return out

    def _check_id(self, oid, ns, path):
        if not isinstance(oid, str) or not oid:
            raise err(400, "bad_id", "id must be a non-empty string", path)
        if not ID_RE.match(oid):
            raise err(400, "bad_id", "id must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}", path)
        if oid.split(".", 1)[0] in RESERVED_NS:
            raise err(400, "bad_id", f"namespaces {RESERVED_NS} are reserved", path)
        if not in_namespace(oid, ns):
            raise err(400, "bad_id", f"id must start with '{ns}.'", path)

    def check_namespace(self, ns, path="namespace"):
        if not isinstance(ns, str) or not ns or not NS_RE.match(ns):
            raise err(400, "bad_request", "namespace must be dot-separated [A-Za-z0-9_-] parts", path)
        if ns.split(".", 1)[0] in RESERVED_NS:
            raise err(400, "bad_request", f"namespaces {RESERVED_NS} are reserved", path)
        return ns

    def check_frame(self, frame, path):
        frame = frame or self.cell.base_frame
        if frame not in (self.cell.base_frame, self.planning_frame):
            raise err(400, "bad_frame", f"frame_id must be {self.cell.base_frame} (the planning frame)", path)
        return frame

    def validate(self, body):
        """Complete validation of a /scene/batch body (meshes are parsed here,
        outside the lock). -> request dict; ApiError leaves the scene untouched."""
        if not isinstance(body, dict):
            raise err(400, "bad_request", "body must be a JSON object")
        ns = self.check_namespace(body.get("namespace"))
        mode = body.get("mode", "replace")
        if mode not in MODES:
            raise err(400, "bad_request", f"mode must be one of {MODES}", "mode")
        if_version = body.get("if_version")
        if if_version is not None and (isinstance(if_version, bool) or not isinstance(if_version, int)):
            raise err(400, "bad_request", "if_version must be an integer", "if_version")
        meta = body.get("meta")
        if meta is not None and _json_size(meta) > self.limits["max_batch_meta_bytes"]:
            raise err(413, "too_large", f"meta exceeds {self.limits['max_batch_meta_bytes']} bytes", "meta")
        req = {"namespace": ns, "mode": mode, "if_version": if_version, "meta": meta,
               "validate_only": bool(body.get("validate_only", False)),
               "keep_attached": bool(body.get("keep_attached", True)), "objects": {}, "ids": []}
        if mode == "remove":
            ids = body.get("ids")
            if not isinstance(ids, list) or not ids:
                raise err(400, "bad_request", "mode=remove needs a non-empty ids list", "ids")
            for i, oid in enumerate(ids):
                self._check_id(oid, ns, f"ids[{i}]")
            req["ids"] = list(dict.fromkeys(ids))
            return req
        objs = body.get("objects")
        if not isinstance(objs, list):
            raise err(400, "bad_request", "objects must be a list", "objects")
        if len(objs) > self.limits["max_objects_per_batch"]:
            raise err(422, "limits_exceeded", f"more than {self.limits['max_objects_per_batch']} objects", "objects")
        for i, o in enumerate(objs):
            p = f"objects[{i}]"
            if not isinstance(o, dict):
                raise err(400, "bad_request", "object must be a JSON object", p)
            oid = o.get("id")
            self._check_id(oid, ns, f"{p}.id")
            if oid in req["objects"]:
                raise err(400, "bad_id", f"duplicate id {oid!r}", f"{p}.id")
            shapes = o.get("shapes")
            if not isinstance(shapes, list) or not shapes:
                raise err(400, "bad_shape", "object needs a non-empty shapes list", f"{p}.shapes")
            if len(shapes) > self.limits["max_shapes_per_object"]:
                raise err(422, "limits_exceeded", f"more than {self.limits['max_shapes_per_object']} shapes",
                          f"{p}.shapes")
            ometa = o.get("meta")
            if ometa is not None and _json_size(ometa) > self.limits["max_object_meta_bytes"]:
                raise err(413, "too_large", f"meta exceeds {self.limits['max_object_meta_bytes']} bytes", f"{p}.meta")
            color = o.get("color")
            if color is not None:
                if (not isinstance(color, list) or len(color) not in (3, 4) or any(
                        isinstance(c, bool) or not isinstance(c, (int, float)) or not 0.0 <= c <= 1.0 for c in color)):
                    raise err(400, "bad_request", "color must be [r, g, b] or [r, g, b, a] with values 0..1",
                              f"{p}.color")
                color = [float(c) for c in color] + ([1.0] if len(color) == 3 else [])
            req["objects"][oid] = {"id": oid, "frame_id": self.check_frame(o.get("frame_id"), f"{p}.frame_id"),
                                   "pose": canon_pose(o.get("pose"), f"{p}.pose"),
                                   "shapes": [self._canon_shape(s, f"{p}.shapes[{j}]") for j, s in enumerate(shapes)],
                                   "attached_link": None, "meta": ometa, "color": color}
        return req

    # ---- views ----

    def legacy_objects(self):
        return self.legacy.as_v2_objects() if self.legacy is not None else {}

    def all_objects(self):
        out = dict(self.legacy_objects())
        out.update(self.objects)
        return out

    def namespace_hash(self, ns):
        return hash_objects([o for oid, o in self.all_objects().items() if in_namespace(oid, ns)])

    def scene_hash(self):
        objs = [hash_object(o) for o in sorted(self.all_objects().values(), key=lambda o: o["id"])]
        acm = sorted([e["a"], e["b"], bool(e["allowed"])] for e in self.acm_entries.values())
        pad = None
        if self.padding:
            pad = {"default_m": fmt(self.padding["default_m"], 6),
                   "links": {k: fmt(v, 6) for k, v in sorted(self.padding.get("links", {}).items())}}
        return _sha({"objects": objs, "acm": acm, "padding": pad})

    def namespaces(self):
        tops = sorted({oid.split(".", 1)[0] for oid in self.all_objects() if "." in oid})
        return {ns: self.namespace_hash(ns) for ns in tops}

    @staticmethod
    def counts(objs):
        meshes = [s for o in objs for s in o["shapes"] if s["type"] == "mesh"]
        return {"objects": len(objs), "primitives": sum(len(o["shapes"]) for o in objs) - len(meshes),
                "meshes": len(meshes), "triangles": sum(s.get("triangles", 0) for s in meshes)}

    # ---- commit ----

    def _apply_and_confirm(self, ops, expect_present, expect_absent, acm=None, padding=None,
                           expect_attached=frozenset(), expect_detached=frozenset(), colors=None):
        """ops as ONE diff, then read back world (and attached) names.
        Raises ApiError(504, apply_unconfirmed)."""
        delay = self.test_hooks.get("delay_next_apply_s") or 0.0
        if delay:
            self.test_hooks["delay_next_apply_s"] = 0.0
            time.sleep(delay)
        if self.test_hooks.get("fail_next_apply"):
            self.test_hooks["fail_next_apply"] -= 1
            raise err(504, "apply_unconfirmed", "apply_planning_scene returned success=false (test hook)")
        if not self.io.apply(ops, self.mesh_cache, acm=acm, padding=padding, colors=colors):
            raise err(504, "apply_unconfirmed", "apply_planning_scene returned success=false")
        world = set(self.io.world_names())
        missing, lingering = sorted(set(expect_present) - world), sorted(set(expect_absent) & world)
        if expect_attached or expect_detached:
            att = set(self.io.attached_names())
            missing += sorted(set(expect_attached) - att)
            lingering += sorted(set(expect_detached) & att)
        if missing or lingering:
            raise err(504, "apply_unconfirmed", "planning scene does not match after apply",
                      missing=missing[:20], lingering=lingering[:20])
        if any(op in ("attach", "detach", "add") for op, _, _ in ops):
            self.send_colors()

    def all_colors(self):
        out = {oid: o["color"] for oid, o in self.objects.items() if o.get("color")}
        out.update(self.builtin_colors)
        return out

    def send_colors(self):
        """Send the complete colour table in one diff (a diff with colours
        replaces the whole table, and MoveIt drops them on attach/detach and
        removes). Display-only: failures are logged, never raised."""
        want = self.all_colors()
        if not want:
            return True
        try:
            return bool(self.io.apply([], self.mesh_cache, colors=want))
        except Exception as e:      # noqa: BLE001
            self.log(f"sending object colours failed: {e}")
            return False

    def start_color_keeper(self, interval_s=2.0):
        """Re-send the colour table every interval_s (RViz restarts, MoveIt
        dropping colours); skips a tick while a commit holds the lock."""
        def loop():
            while True:
                time.sleep(interval_s)
                if self._lock.acquire(blocking=False):
                    try:
                        if self.all_colors():
                            self.io.apply([], self.mesh_cache, colors=self.all_colors())
                    except Exception as e:     # noqa: BLE001
                        self.log(f"colour keeper: {e}")
                    finally:
                        self._lock.release()
        threading.Thread(target=loop, name="color-keeper", daemon=True).start()

    def _rollback(self, before, touched, acm_before=None):
        world, attached = set(self.io.world_names()), set(self.io.attached_names())
        ops = [("detach", oid, {"link": before[oid]["attached_link"]}) for oid in touched
               if oid in attached and before.get(oid, {}).get("attached_link")]
        ops += [("remove", oid, None) for oid in touched if oid in world or oid in attached]
        ops += [("add", oid, before[oid]) for oid in touched if oid in before]
        ops += [("attach", oid, {"link": before[oid]["attached_link"], "touch_links": before[oid].get("touch_links")})
                for oid in touched if before.get(oid, {}).get("attached_link")]
        present = {oid for oid in touched if oid in before and not before[oid].get("attached_link")}
        self._apply_and_confirm(ops, present, set(touched) - set(before), acm=acm_before)

    def _locked(self, motion_lock, timeout_s=5.0):
        if not motion_lock.acquire(timeout=timeout_s):
            raise err(409, "busy", "robot is busy (motion in progress); scene not changed")
        return motion_lock

    def _check_if_version(self, if_version):
        if if_version is not None and if_version != self.version:
            raise err(409, "version_conflict", f"scene_version is {self.version}, request expects {if_version}",
                      scene_version=self.version)

    def batch(self, req, motion_lock):
        """Commit a validated request; the motion lock is held for the commit only."""
        t0 = time.time()
        if req["validate_only"]:
            with self._lock:
                return self._batch(req, True, t0)
        self._locked(motion_lock)
        try:
            with self._lock:
                return self._batch(req, False, t0)
        finally:
            motion_lock.release()

    def _batch(self, req, dry, t0):
        ns, mode = req["namespace"], req["mode"]
        self._check_if_version(req["if_version"])
        legacy_here = sorted(n for n in self.legacy_objects() if in_namespace(n, ns))
        if legacy_here:
            raise err(409, "namespace_owned", "namespace contains objects created through the legacy /scene/objects "
                      "API; DELETE /scene/namespaces/<ns> first", objects=legacy_here[:20])
        current = {oid: o for oid, o in self.objects.items() if in_namespace(oid, ns)}
        target, kept, warnings = dict(current), [], []
        if mode == "remove":
            unknown = [oid for oid in req["ids"] if oid not in current]
            if unknown:
                raise err(404, "unknown_object", f"not in namespace {ns!r}: {unknown[:20]}", objects=unknown[:20])
            for oid in req["ids"]:
                if current[oid].get("attached_link"):
                    raise err(409, "object_attached", f"{oid!r} is attached; detach it first")
                target.pop(oid)
        else:
            if mode == "replace":
                for oid, o in current.items():
                    if oid in req["objects"]:
                        continue
                    if o.get("attached_link"):
                        if not req["keep_attached"]:
                            raise err(409, "object_attached", f"{oid!r} is attached (keep_attached=false)")
                        kept.append(oid)
                    else:
                        target.pop(oid)
            for oid, o in req["objects"].items():
                if current.get(oid, {}).get("attached_link"):
                    if mode == "replace" and req["keep_attached"]:
                        kept.append(oid)
                        continue
                    raise err(409, "object_attached", f"{oid!r} is attached; detach it first")
                target[oid] = o
        c = self.counts([o for oid, o in self.objects.items() if not in_namespace(oid, ns)] + list(target.values()))
        if c["primitives"] > self.limits["max_primitives_total"]:
            raise err(422, "limits_exceeded", f"scene would have {c['primitives']} primitives "
                      f"(limit {self.limits['max_primitives_total']})")
        if c["triangles"] > self.limits["max_triangles_total"]:
            raise err(422, "limits_exceeded", f"scene would have {c['triangles']} mesh triangles "
                      f"(limit {self.limits['max_triangles_total']})")
        added = sorted(oid for oid in target if oid not in current)
        removed = sorted(oid for oid in current if oid not in target)
        updated = sorted(oid for oid in target if oid in current and hash_object(target[oid]) != hash_object(current[oid]))
        meta_only = [oid for oid in target if oid in current and oid not in updated
                     and target[oid].get("meta") != current[oid].get("meta")]
        color_only = [oid for oid in target if oid in current and oid not in updated
                      and target[oid].get("color") != current[oid].get("color")]   # cosmetic: no version bump
        world = set() if dry else set(self.io.world_names())
        resync = self._is_dirty(ns)
        if resync:      # move_group's content of this namespace is unknown: re-send all of it
            removed = sorted(set(removed) | {oid for oid in world if in_namespace(oid, ns) and oid not in target})
            missing = sorted(oid for oid in target if oid not in added and not target[oid].get("attached_link"))
            warnings.append("namespace was dirty: fully re-synchronised")
        else:
            missing = [] if dry else sorted(oid for oid in target if oid not in added and oid not in world
                                            and not target[oid].get("attached_link"))
            if missing:
                warnings.append(f"{len(missing)} object(s) were missing from move_group (restarted?) and are re-sent")
        result = {"ok": True, "namespace": ns, "mode": mode, "added": added, "updated": updated, "removed": removed,
                  "kept_attached": sorted(set(kept)), "counts": self.counts(list(target.values())),
                  "namespace_hash": hash_objects(list(target.values())), "warnings": warnings}
        changed = bool(added or updated or removed or missing or resync)
        if dry:
            result.update(validate_only=True, would_change=changed, scene_version=self.version,
                          apply_ms=int((time.time() - t0) * 1000))
            return result
        if not changed:
            for oid in color_only:
                self.objects[oid]["color"] = target[oid].get("color")
            if color_only:
                if not self.send_colors():
                    warnings.append("colour update could not be confirmed in move_group")
                result["recolored"] = sorted(color_only)
            for oid in meta_only:
                self.objects[oid]["meta"] = target[oid].get("meta")
            if req["meta"] is not None:
                self.batch_meta[ns] = req["meta"]
            if meta_only or color_only or req["meta"] is not None:
                self.save()
            result.update(unchanged=True, confirmed=True, scene_version=self.version, scene_hash=self.scene_hash(),
                          apply_ms=int((time.time() - t0) * 1000))
            return result
        touched = sorted(set(added) | set(updated) | set(removed) | set(missing))
        ops = [("remove", oid, None) for oid in sorted(set(updated) | set(removed) | set(missing)) if oid in world]
        ops += [("add", oid, target[oid]) for oid in sorted(set(added) | set(updated) | set(missing))]
        before = copy.deepcopy(current)
        acm_msg, acm_drop = self._acm_for_removed(set(removed))
        acm_before = self.io.get_acm() if acm_msg is not None else None
        try:
            self._apply_and_confirm(ops, set(added) | set(updated) | set(missing), set(removed), acm=acm_msg)
        except ApiError as e:
            try:
                self._rollback(before, touched, acm_before)
                e.extra["rolled_back"] = True
            except ApiError as e2:
                self.mark_dirty(f"batch on {ns!r} failed ({e.msg}) and rollback failed ({e2.msg})", {ns})
                e.extra["rolled_back"] = False
            e.extra["dirty"] = self.dirty
            self.save()
            raise
        for oid in removed:
            self.objects.pop(oid, None)
        self.objects.update(target)
        for k in acm_drop:
            self.acm_entries.pop(k, None)
        if req["meta"] is not None:
            self.batch_meta[ns] = req["meta"]
        self._clear_dirty(ns)
        self._bump()
        result.update(unchanged=False, confirmed=True, scene_version=self.version, scene_hash=self.scene_hash(),
                      apply_ms=int((time.time() - t0) * 1000))
        return result

    def delete_namespace(self, ns, motion_lock, force_detach=False, if_version=None):
        ns = self.check_namespace(ns)
        self._locked(motion_lock)
        try:
            with self._lock:
                self._check_if_version(if_version)
                ours = sorted(oid for oid in self.objects if in_namespace(oid, ns))
                legacy_objs = self.legacy_objects()
                legacy = sorted(n for n in legacy_objs if in_namespace(n, ns))
                attached = [oid for oid in ours if self.objects[oid].get("attached_link")]
                attached += [n for n in legacy if legacy_objs[n].get("attached_link")]
                if attached and not force_detach:
                    raise err(409, "object_attached", f"attached objects in namespace: {attached}", objects=attached)
                world = set(self.io.world_names())
                stray = sorted(oid for oid in world if in_namespace(oid, ns) and oid not in self.objects
                               and oid not in legacy_objs)
                acm_msg, acm_drop = self._acm_for_removed(set(ours) | set(legacy) | set(stray))
                if ours or stray or acm_msg is not None:
                    before = {oid: self.objects[oid] for oid in ours}
                    acm_before = self.io.get_acm() if acm_msg is not None else None
                    ops = [("detach", oid, {"link": self.objects[oid]["attached_link"]})
                           for oid in ours if self.objects[oid].get("attached_link")]
                    ops += [("remove", oid, None) for oid in ours + stray
                            if oid in world or self.objects.get(oid, {}).get("attached_link")]
                    try:
                        self._apply_and_confirm(ops, set(), set(ours), acm=acm_msg)
                    except ApiError as e:
                        try:
                            self._rollback(before, ours, acm_before)
                        except ApiError as e2:
                            self.mark_dirty(f"namespace delete failed ({e.msg}), rollback failed ({e2.msg})", {ns})
                        self.save()
                        raise
                    for oid in ours:
                        self.objects.pop(oid)
                    for k in acm_drop:
                        self.acm_entries.pop(k, None)
                if legacy:
                    self.legacy.remove_many(legacy)
                for k in [k for k in self.batch_meta if self._covers(ns, k)]:
                    self.batch_meta.pop(k)
                removed = ours + legacy + stray
                if removed or self._is_dirty(ns):
                    self._clear_dirty(ns)
                    self._bump()
                return {"removed": removed, "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def bump_for_legacy_change(self):
        """A legacy /scene/objects change also changes the scene hash."""
        with self._lock:
            self._bump()

    # ---- allowed collision matrix ----
    # The ACM is handled as a plain dict here: {"names": [...], "values": {(a, b): bool},
    # "defaults": {name: bool}}; ros_io converts it from/to the message.

    @staticmethod
    def pair_key(a, b):
        a, b = sorted((a, b))
        return f"{a}|{b}"

    @staticmethod
    def _default_allowed(acm, a, b):
        """MoveIt's value for a pair without an explicit entry: allowed when
        either name has a default entry that allows everything."""
        return bool(acm["defaults"].get(a, False) or acm["defaults"].get(b, False))

    def _acm_msg_with(self, entries, drop_names=()):
        """The scene's ACM with ``entries`` written over it and ``drop_names``
        removed (rows and columns). The message is dense, so a pair that had no
        explicit entry gets the default-derived value, not False."""
        cur = self.io.get_acm()
        names = list(cur["names"])
        vals = dict(cur["values"])
        for e in entries.values():
            for n in (e["a"], e["b"]):
                if n not in names:
                    names.append(n)
            vals[(e["a"], e["b"])] = vals[(e["b"], e["a"])] = bool(e["allowed"])
        names = [n for n in names if n not in set(drop_names)]
        return {"names": names,
                "values": {(a, b): vals[(a, b)] if (a, b) in vals else self._default_allowed(cur, a, b)
                           for a in names for b in names},
                "defaults": dict(cur["defaults"])}

    @staticmethod
    def _acm_value(acm, a, b):
        return acm["values"].get((a, b)) if a in acm["names"] and b in acm["names"] else None

    def _known_names(self):
        names = set(self.robot_links()) | set(self.all_objects())
        try:
            names |= set(self.io.world_names()) | set(self.io.attached_names())
        except Exception:       # noqa: BLE001
            pass
        return names

    def acm_get(self, names=None):
        acm = self.io.get_acm()
        pick = set(names) if names else None
        entries = []
        all_names = list(acm["names"])
        for i, a in enumerate(all_names):
            for b in all_names[i + 1:]:
                if pick is not None and not (a in pick or b in pick):
                    continue
                if acm["values"].get((a, b)):
                    entries.append({"a": a, "b": b, "allowed": True, "client": self.pair_key(a, b) in self.acm_entries})
        return {"entries": entries, "client_entries": list(self.acm_entries.values()),
                "default_entries": dict(acm["defaults"]), "scene_version": self.version}

    def acm_set(self, entries, motion_lock, if_version=None):
        if not isinstance(entries, list) or not entries:
            raise err(400, "bad_request", "entries must be a non-empty list", "entries")
        known = self._known_names()
        for i, e in enumerate(entries):
            if not isinstance(e, dict) or not isinstance(e.get("a"), str) or not isinstance(e.get("b"), str):
                raise err(400, "bad_request", "entry needs string a, b and bool allowed", f"entries[{i}]")
            if e["a"] == e["b"]:
                raise err(400, "bad_request", "a and b must differ", f"entries[{i}]")
            for k in ("a", "b"):
                if e[k] not in known:
                    raise err(404, "unknown_object", f"{e[k]!r} is neither a robot link nor a scene object",
                              f"entries[{i}].{k}")
            if not isinstance(e.get("allowed"), bool):
                raise err(400, "bad_request", "allowed must be true/false", f"entries[{i}].allowed")
        self._locked(motion_lock)
        try:
            with self._lock:
                self._check_if_version(if_version)
                cur = self.io.get_acm()
                new = dict(self.acm_entries)
                for e in entries:
                    key = self.pair_key(e["a"], e["b"])
                    a, b = sorted((e["a"], e["b"]))
                    orig = new[key]["original"] if key in new else self._acm_value(cur, a, b)
                    new[key] = {"a": a, "b": b, "allowed": e["allowed"], "original": orig}
                self._apply_and_confirm([], set(), set(), acm=self._acm_msg_with(new))
                after = self.io.get_acm()
                wrong = [k for k, e in new.items() if self._acm_value(after, e["a"], e["b"]) != e["allowed"]]
                if wrong:
                    raise err(504, "apply_unconfirmed", "ACM read-back does not match", pairs=wrong[:20])
                self.acm_entries = new
                self._bump()
                return {"entries": list(new.values()), "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def acm_delete(self, motion_lock, namespace=None, if_version=None):
        """Put client-written entries (all, or those touching ``namespace``)
        back to their original value."""
        if namespace is not None:
            self.check_namespace(namespace)
        self._locked(motion_lock)
        try:
            with self._lock:
                self._check_if_version(if_version)
                keys = [k for k, e in self.acm_entries.items() if namespace is None
                        or in_namespace(e["a"], namespace) or in_namespace(e["b"], namespace)]
                if keys:
                    cur = self.io.get_acm()
                    restore = {}
                    for k in keys:
                        e = self.acm_entries[k]
                        allowed = bool(e["original"]) if e["original"] is not None else \
                            self._default_allowed(cur, e["a"], e["b"])
                        restore[k] = dict(e, allowed=allowed)
                    self._apply_and_confirm([], set(), set(), acm=self._acm_msg_with(restore))
                    for k in keys:
                        self.acm_entries.pop(k)
                    self._bump()
                return {"removed": keys, "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def _acm_for_removed(self, removed):
        """(acm or None, client keys to forget) when objects go away: MoveIt
        does not clean their ACM rows up by itself."""
        if not removed:
            return None, []
        keys = [k for k, e in self.acm_entries.items() if e["a"] in removed or e["b"] in removed]
        if not keys and not (set(self.io.get_acm()["names"]) & removed):
            return None, []
        keep = {k: e for k, e in self.acm_entries.items() if k not in keys}
        return self._acm_msg_with(keep, drop_names=removed), keys

    # ---- link padding ----

    def _padding_map(self, pad):
        return {link: float(pad.get("links", {}).get(link, pad["default_m"])) for link in self.robot_links()}

    def padding_set(self, default_m, links, motion_lock, if_version=None):
        dv = _num(default_m, "default_m")
        if not 0.0 <= dv <= 0.1:
            raise err(400, "bad_request", "default_m must be in 0..0.1", "default_m")
        links = links or {}
        if not isinstance(links, dict):
            raise err(400, "bad_request", "links must be {link: metres}", "links")
        robot_links, clean = set(self.robot_links()), {}
        for k, v in links.items():
            if k not in robot_links:
                raise err(404, "unknown_object", f"{k!r} is not a robot link", f"links.{k}")
            v = _num(v, f"links.{k}")
            if not 0.0 <= v <= 0.1:
                raise err(400, "bad_request", "padding must be in 0..0.1 m", f"links.{k}")
            clean[k] = v
        pad = {"default_m": dv, "links": clean}
        self._locked(motion_lock)
        try:
            with self._lock:
                self._check_if_version(if_version)
                want = self._padding_map(pad)
                self._apply_and_confirm([], set(), set(), padding=want)
                got = self.io.get_padding()
                wrong = [k for k, v in want.items() if abs(got.get(k, -1.0) - v) > 1e-9]
                if wrong:
                    raise err(504, "apply_unconfirmed", "link padding read-back does not match", links=wrong[:20])
                self.padding = None if (dv == 0 and not any(clean.values())) else pad
                self._bump()
                return {"padding": pad, "applied": want, "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    # ---- attach / detach ----

    def attach(self, oid, link, touch_links, motion_lock):
        links = set(self.robot_links())
        if link not in links:
            raise err(400, "bad_request", f"{link!r} is not a robot link", "link")
        touch = list(touch_links) if touch_links else self.cell.touch_links(link)
        bad = [t for t in touch if t not in links]
        if bad:
            raise err(400, "bad_request", f"touch_links not robot links: {bad}", "touch_links")
        self._locked(motion_lock)
        try:
            with self._lock:
                o = self.objects.get(oid)
                if o is None:
                    raise err(404, "unknown_object", f"no /scene/batch object {oid!r}")
                if o.get("attached_link"):
                    raise err(409, "object_attached", f"{oid!r} is already attached to {o['attached_link']}")
                self._apply_and_confirm([("attach", oid, {"link": link, "touch_links": touch})], set(), {oid},
                                        expect_attached={oid})
                o["attached_link"], o["touch_links"] = link, touch        # only after MoveIt confirmed it
                self._bump()
                return {"id": oid, "attached_link": link, "touch_links": touch, "confirmed": True,
                        "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def detach(self, oid, motion_lock):
        self._locked(motion_lock)
        try:
            with self._lock:
                o = self.objects.get(oid)
                if o is None:
                    raise err(404, "unknown_object", f"no /scene/batch object {oid!r}")
                if not o.get("attached_link"):
                    raise err(409, "bad_request", f"{oid!r} is not attached")
                self._apply_and_confirm([("detach", oid, {"link": o["attached_link"]})], {oid}, set(),
                                        expect_detached={oid})
                pose = self.io.world_poses({oid}).get(oid)          # MoveIt left it where the hand was
                if pose is not None:
                    o["pose"] = canon_pose(pose, oid)
                o["frame_id"] = self.cell.base_frame
                o["attached_link"] = None
                o.pop("touch_links", None)
                self._bump()
                return {"id": oid, "attached_link": None, "pose": o["pose"], "confirmed": True,
                        "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    # ---- unreferenced files ----

    def referenced_files(self):
        refs = {s["file"] for o in self.objects.values() for s in o["shapes"] if s["type"] == "mesh" and s.get("file")}
        if self.legacy is not None:
            refs |= self.legacy.referenced_files()
        return refs

    def unreferenced(self, prefix="pipefit/meshes", older_than_s=600.0):
        """Files under ``prefix`` no object or model uses; files changed or used
        by a batch within ``older_than_s`` are listed as protected."""
        try:
            root = self.store.resolve(prefix)
        except FileStoreError as e:
            raise err(400, "bad_request", e.msg, "prefix")
        if not os.path.isdir(root):
            return []
        refs, now, out = self.referenced_files(), time.time(), []
        for d, _dirs, files in os.walk(root):
            for name in sorted(files):
                if name.startswith("."):
                    continue
                ap = os.path.join(d, name)
                rel = self.store.relpath(ap)
                if rel in refs:
                    continue
                st = os.stat(ap)
                last = max(st.st_mtime, self._recent_files.get(rel, 0.0))
                out.append({"path": rel, "size": st.st_size, "age_s": round(now - last, 1),
                            "protected": now - last < older_than_s})
        return out

    def cleanup(self, prefix="pipefit/meshes", older_than_s=600.0, dry_run=True, models=False):
        with self._lock:
            files = self.unreferenced(prefix, older_than_s)
            victims = [f for f in files if not f["protected"]]
            unused = self.legacy.unused_models() if (models and self.legacy is not None) else []
            out = {"dry_run": dry_run, "prefix": prefix, "older_than_s": older_than_s,
                   "files": [f["path"] for f in victims], "protected": [f["path"] for f in files if f["protected"]],
                   "bytes": sum(f["size"] for f in victims), "models": unused}
            if dry_run:
                return out
            for f in victims:
                ap = self.store.resolve(f["path"])
                try:
                    sha = self.store.sha256(ap)
                except OSError:
                    sha = None
                self.store.delete(ap)
                if sha in self.mesh_cache:
                    self._mesh_tris -= len(self.mesh_cache.pop(sha)[1])
                self._recent_files.pop(f["path"], None)
            if unused:
                self.legacy.delete_models(unused)
            return out

    # ---- snapshot / listing ----

    def snapshot(self, namespace=None):
        objs = self.all_objects()
        acm = list(self.acm_entries.values())
        if namespace:
            objs = {k: v for k, v in objs.items() if in_namespace(k, namespace)}
            acm = [e for e in acm if in_namespace(e["a"], namespace) or in_namespace(e["b"], namespace)]
        out = {"api_version": API_VERSION, "scene_version": self.version, "scene_hash": self.scene_hash(),
               "dirty": self.dirty, "dirty_namespaces": sorted(self.dirty_namespaces), "updated_at": self.updated_at,
               "objects": [{k: v for k, v in o.items() if k != "legacy"} for _, o in sorted(objs.items())],
               "acm": acm, "padding": self.padding, "builtin": self.builtin}
        if namespace:
            out.update(namespace=namespace, namespace_hash=self.namespace_hash(namespace))
        return out

    def list_objects(self, namespace=None, geometry=False, world_names=None):
        objs = self.all_objects()
        out = []
        for oid in sorted(objs):
            o, ns = objs[oid], namespace_of(oid)
            if namespace and not self._covers(namespace, ns):
                continue
            item = {"id": oid, "namespace": ns, "frame_id": o["frame_id"], "pose": o["pose"],
                    "attached_link": o.get("attached_link"), "meta": o.get("meta"), "color": o.get("color")}
            if geometry:
                item["shapes"] = [{k: v for k, v in s.items() if k != "triangles"} for s in o["shapes"]]
            out.append(item)
        if namespace in (None, "builtin"):
            for n in sorted(world_names or []):
                if n in self.cell.builtin_world:
                    out.append({"id": n, "namespace": "builtin", "frame_id": self.planning_frame, "pose": None,
                                "attached_link": None, "meta": None})
            for b in self.builtin:
                item = {"id": b["id"], "namespace": "builtin", "frame_id": b["frame_id"], "pose": b["pose"],
                        "attached_link": b["link"], "meta": {"estimated": b.get("estimated", False)}}
                if geometry:
                    item["shapes"] = b["shapes"]
                out.append(item)
        return out
