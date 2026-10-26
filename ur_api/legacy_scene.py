"""Named CAD models + placed objects (the first scene API: /scene/models, /scene/objects).

Two layers, so a model file is parsed once and placed many times (a detector re-running every few seconds):
  * models  -- a mesh in the /files store (STL/OBJ/PLY/DAE) at a scale, optionally reduced to its convex hull;
  * objects -- instances of a model at a pose, each tagged with a ``source`` so sync() can replace one
    source's set at once without touching other sources, attached objects or the built-in geometry.
Persisted to state/scene.json and re-applied at start (move_group forgets its world on a restart).
Every persisted change bumps the v2 scene version (core.bump_for_legacy_change), since it changes its hash.
"""
import copy
import json
import math
import os
import re
import threading
import time

import numpy as np

import geom
from errors import err
from file_store import FileStoreError
from scene_core import IDENTITY_POSE, MESH_EXTENSIONS, canon_pose, convex_hull, parse_mesh_isolated

COLLISION_MODES = ("mesh", "convex_hull")
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def pose_from_dict(d, base=None, cell=None):
    """{x,y,z} + {roll,pitch,yaw} (degrees, sxyz) or {qx,qy,qz,qw}; missing parts from ``base``
    (a pose dict {x,y,z,qx,qy,qz,qw}) or identity. -> pose dict with roll/pitch/yaw too."""
    def num(k, fallback):
        v = d.get(k)
        if v is None:
            return fallback
        try:
            v = float(v)
        except (TypeError, ValueError):
            raise err(400, "bad_pose", f"{k} must be a number")
        if not math.isfinite(v):
            raise err(400, "bad_pose", f"{k} must be finite")
        return v
    b = base or {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    x, y, z = num("x", b["x"]), num("y", b["y"]), num("z", b["z"])
    if any(d.get(k) is not None for k in ("qx", "qy", "qz", "qw")):
        q = [num("qx", 0.0), num("qy", 0.0), num("qz", 0.0), num("qw", 1.0)]
        n = math.sqrt(sum(v * v for v in q))
        if n < 1e-6:
            raise err(400, "bad_pose", "quaternion has ~zero norm")
        q = [v / n for v in q]
    elif any(d.get(k) is not None for k in ("roll", "pitch", "yaw")):
        q = geom.euler_to_quat(math.radians(num("roll", 0.0)), math.radians(num("pitch", 0.0)),
                               math.radians(num("yaw", 0.0)))
    else:
        q = [b["qx"], b["qy"], b["qz"], b["qw"]]
    r, p, yw = geom.matrix_to_euler(geom.quat_to_matrix(q))
    return {"x": x, "y": y, "z": z, "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
            "roll": math.degrees(r), "pitch": math.degrees(p), "yaw": math.degrees(yw)}


def _v2_pose(p):
    return {"position": [p["x"], p["y"], p["z"]], "orientation": [p["qx"], p["qy"], p["qz"], p["qw"]]}


class LegacyScene:
    def __init__(self, io, store, state_file, cell, core, logger=None):
        self.io, self.store, self.state_file, self.cell, self.core = io, store, state_file, cell, core
        self.log = logger or (lambda *a: None)
        self.models = {}
        self.objects = {}
        self._lock = threading.RLock()
        self._restoring = False

    # ---- persistence ----

    def save(self):
        os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
        tmp = self.state_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"models": self.models, "objects": self.objects}, f, indent=2, sort_keys=True)
        os.replace(tmp, self.state_file)
        if not self._restoring:
            self.core.bump_for_legacy_change()

    def restore(self):
        if not os.path.isfile(self.state_file):
            return {"models": 0, "objects": 0, "errors": []}
        with open(self.state_file) as f:
            data = json.load(f)
        errors = []
        with self._lock:
            self._restoring = True
            try:
                for name, info in data.get("models", {}).items():
                    try:
                        self.register_model(name, info["file"], info.get("scale", 1.0), info.get("collision", "mesh"),
                                            overwrite=True, persist=False)
                    except Exception as e:      # noqa: BLE001
                        self.models[name] = dict(info, error=getattr(e, "msg", str(e)))
                        errors.append(f"model {name}: {getattr(e, 'msg', e)}")
                for name, info in data.get("objects", {}).items():
                    info.pop("error", None)
                    try:
                        self._apply(info)
                        if info.get("attached_link"):
                            self._attach_ops(info)
                    except Exception as e:      # noqa: BLE001
                        info["error"] = getattr(e, "msg", str(e))
                        errors.append(f"object {name}: {info['error']}")
                    self.objects[name] = info
                self.save()
            finally:
                self._restoring = False
        return {"models": len(self.models), "objects": len(self.objects), "errors": errors}

    # ---- models ----

    def _key(self, name):
        m = self.models[name]
        return f"model:{name}:{m['collision']}:{m['sha256']}"

    def list_models(self):
        return [dict(m, in_use=sorted(o for o, i in self.objects.items() if i["model"] == m["name"]))
                for m in sorted(self.models.values(), key=lambda m: m["name"])]

    def get_model(self, name):
        if name not in self.models:
            raise err(404, "unknown_model", f"no model named {name!r}")
        return dict(self.models[name])

    def register_model(self, name, file, scale=1.0, collision="mesh", overwrite=False, persist=True):
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise err(400, "bad_name", "name must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}")
        try:
            scale = float(scale)
        except (TypeError, ValueError):
            raise err(400, "bad_scale", "scale must be a number")
        if not 0 < scale < 1000:
            raise err(400, "bad_scale", "scale must be in (0, 1000)")
        if collision not in COLLISION_MODES:
            raise err(400, "bad_collision", f"collision must be one of {COLLISION_MODES}")
        try:
            ap = self.store.resolve(file)
        except FileStoreError as e:
            raise err(e.status, e.code, e.msg)
        if not os.path.isfile(ap):
            raise err(404, "file_not_found", f"no file {file!r} in the /files store")
        if not ap.lower().endswith(MESH_EXTENSIONS):
            raise err(415, "unsupported_format", f"mesh must be one of {MESH_EXTENSIONS} (export STEP/IGES to STL first)")
        if os.path.getsize(ap) == 0:
            raise err(422, "mesh_empty", "the file is empty (0 bytes)")
        with self._lock:
            if name in self.models and not overwrite:
                raise err(409, "conflict", f"model {name!r} exists (pass overwrite=1)")
            verts, faces = parse_mesh_isolated(ap)
            src = len(faces)
            if collision == "convex_hull":
                try:
                    verts, faces = convex_hull(verts)
                except Exception as e:      # noqa: BLE001
                    raise err(422, "hull_failed", f"convex hull failed (degenerate mesh?): {e}")
            sv = verts * scale
            lo, hi = sv.min(axis=0), sv.max(axis=0)
            info = {"name": name, "file": self.store.relpath(ap), "sha256": self.store.sha256(ap), "scale": scale,
                    "collision": collision, "registered": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "triangles": int(len(faces)), "source_triangles": int(src), "vertices": int(len(verts)),
                    "bbox_min": [round(float(v), 4) for v in lo], "bbox_max": [round(float(v), 4) for v in hi],
                    "bbox_size": [round(float(v), 4) for v in hi - lo]}
            self.models[name] = info
            self.core.cache_mesh(self._key(name), verts, faces)
            for o in self.objects.values():          # objects placed with this model pick up the new mesh
                if o["model"] == name:
                    self._apply(o)
                    o.pop("error", None)
                    if o.get("attached_link"):
                        self._attach_ops(o)
            if persist:
                self.save()
        return dict(info)

    def delete_model(self, name, force=False):
        with self._lock:
            if name not in self.models:
                raise err(404, "unknown_model", f"no model named {name!r}")
            users = [o for o, i in self.objects.items() if i["model"] == name]
            if users and not force:
                raise err(409, "in_use", f"model {name!r} is used by objects {users} (pass force=1 to remove them too)")
            for o in users:
                self.remove_object(o, persist=False)
            self.models.pop(name)
            self.save()

    def unused_models(self):
        used = {o["model"] for o in self.objects.values()}
        return sorted(n for n in self.models if n not in used)

    def delete_models(self, names):
        with self._lock:
            for n in names:
                self.models.pop(n, None)
            self._restoring, was = True, self._restoring     # no scene change: no version bump
            try:
                self.save()
            finally:
                self._restoring = was

    def referenced_files(self):
        return {m.get("file") for m in self.models.values() if m.get("file")}

    # ---- objects ----

    def _v2(self, info):
        m = self.models.get(info["model"])
        if m is None or "error" in m:
            raise err(409 if m else 404, "model_unavailable" if m else "unknown_model",
                      f"model {info['model']!r} is not loaded" + (f": {m.get('error')}" if m else ""))
        if self._key(info["model"]) not in self.core.mesh_cache:
            raise err(409, "model_unavailable", f"model {info['model']!r} has no parsed mesh (re-register it)")
        return {"id": info["name"], "frame_id": info.get("frame") or self.cell.base_frame,
                "pose": canon_pose(_v2_pose(info["pose"]), info["name"]),
                "shapes": [{"type": "mesh", "sha256": self._key(info["model"]), "scale": m["scale"],
                            "pose": copy.deepcopy(IDENTITY_POSE)}]}

    def _apply(self, info):
        if not self.io.apply([("add", info["name"], self._v2(info))], self.core.mesh_cache):
            raise err(504, "apply_unconfirmed", "apply_planning_scene returned success=false")

    def _attach_ops(self, info):
        self.core._apply_and_confirm([("attach", info["name"], {"link": info["attached_link"],
                                                                 "touch_links": info.get("touch_links")})],
                                     set(), {info["name"]}, expect_attached={info["name"]})

    def as_v2_objects(self):
        out = {}
        for name, info in self.objects.items():
            m = self.models.get(info["model"], {})
            out[name] = {"id": name, "frame_id": info.get("frame") or self.cell.base_frame,
                         "pose": canon_pose(_v2_pose(info["pose"]), name),
                         "shapes": [{"type": "mesh", "file": m.get("file"), "sha256": m.get("sha256"),
                                     "scale": m.get("scale", 1.0), "pose": copy.deepcopy(IDENTITY_POSE)}],
                         "attached_link": info.get("attached_link"),
                         "meta": {"legacy_model": info["model"], "source": info.get("source")}, "legacy": True}
        return out

    def _frame(self, frame):
        frame = frame or self.cell.base_frame
        if frame == "base":
            raise err(400, "bad_frame", "give legacy object poses in base_link (the pendant 'base' frame is base_link "
                                        "turned 180 deg about z: negate x and y)")
        if frame not in (self.cell.base_frame, self.cell.planning_frame):
            raise err(400, "bad_frame", f"frame must be {self.cell.base_frame}")
        return frame

    def list_objects(self):
        out = {n: dict(i, builtin=False) for n, i in self.objects.items()}
        try:
            world = self.io.world_names()
            others = [n for n in world if n not in out]
            poses = self.io.world_poses(set(others)) if others else {}
            for n in others:
                p = poses.get(n)
                out[n] = {"name": n, "model": None, "source": "scene", "builtin": n in self.cell.builtin_world,
                          "frame": self.cell.planning_frame,
                          "pose": pose_from_dict({"x": p["position"][0], "y": p["position"][1], "z": p["position"][2],
                                                  "qx": p["orientation"][0], "qy": p["orientation"][1],
                                                  "qz": p["orientation"][2], "qw": p["orientation"][3]}) if p else None}
            for n in self.io.attached_names():
                out.setdefault(n, {"name": n, "model": None, "source": "scene", "builtin": False})
        except Exception as e:      # noqa: BLE001
            self.log(f"scene listing failed: {e}")
        return [out[k] for k in sorted(out)]

    def get_object(self, name):
        if name not in self.objects:
            raise err(404, "not_found", f"no object named {name!r}")
        return dict(self.objects[name])

    def _check_name(self, name):
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise err(400, "bad_name", "name must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}")
        if name in self.cell.builtin_world:
            raise err(409, "builtin", f"{name!r} is built-in safety geometry")
        if name in self.core.objects:
            raise err(409, "namespace_owned", f"{name!r} is owned by a /scene/batch namespace")

    def add_object(self, name, model, pose_dict, frame=None, source="manual", persist=True):
        self._check_name(name)
        with self._lock:
            if model not in self.models:
                raise err(404, "unknown_model", f"no model named {model!r}")
            info = {"name": name, "model": model, "pose": pose_from_dict(pose_dict), "frame": self._frame(frame),
                    "source": source or "manual", "attached_link": None, "updated": time.strftime("%Y-%m-%dT%H:%M:%S")}
            if self.objects.get(name, {}).get("attached_link"):
                self.detach(name, persist=False)
            self._apply(info)
            self.objects[name] = info
            if persist:
                self.save()
        return dict(info)

    def update_object(self, name, pose_dict=None, frame=None):
        with self._lock:
            info = self.objects.get(name)
            if info is None:
                raise err(404, "not_found", f"no object named {name!r}")
            if info.get("attached_link"):
                raise err(409, "attached", "detach the object before moving it")
            if pose_dict:
                info["pose"] = pose_from_dict(pose_dict, base=info["pose"])
            if frame:
                info["frame"] = self._frame(frame)
            info["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            self._apply(info)
            self.save()
        return dict(info)

    def remove_object(self, name, force=False, persist=True):
        with self._lock:
            if name in self.cell.builtin_world and not force:
                raise err(409, "builtin", f"{name!r} is built-in safety geometry (force=1 to remove anyway)")
            info = self.objects.pop(name, None)
            if info is None and name not in set(self.io.world_names()) | set(self.io.attached_names()):
                raise err(404, "not_found", f"no object named {name!r}")
            ops = []
            if info and info.get("attached_link"):
                ops.append(("detach", name, {"link": info["attached_link"]}))
            ops.append(("remove", name, None))
            self.io.apply(ops, self.core.mesh_cache)
            if persist:
                self.save()

    def remove_many(self, names):
        with self._lock:
            for n in names:
                if n in self.objects:
                    self.remove_object(n, force=True, persist=False)
            self.save()

    def attach(self, name, link=None, touch_links=None, persist=True):
        with self._lock:
            info = self.objects.get(name)
            if info is None:
                raise err(404, "not_found", f"no object named {name!r}")
            link = link or self.cell.hand_link
            pending = dict(info, attached_link=link, touch_links=touch_links or self.cell.touch_links(link))
            self._attach_ops(pending)               # recorded only once MoveIt confirmed it
            info["attached_link"], info["touch_links"] = pending["attached_link"], pending["touch_links"]
            if persist:
                self.save()
        return dict(info)

    def detach(self, name, persist=True):
        with self._lock:
            info = self.objects.get(name)
            if info is None:
                raise err(404, "not_found", f"no object named {name!r}")
            if info.get("attached_link"):
                self.core._apply_and_confirm([("detach", name, {"link": info["attached_link"]})], {name}, set(),
                                             expect_detached={name})
                info["attached_link"] = None
                info.pop("touch_links", None)
                p = self.io.world_poses({name}).get(name)        # MoveIt left it where the hand was
                if p:
                    info["pose"] = pose_from_dict({"x": p["position"][0], "y": p["position"][1], "z": p["position"][2],
                                                   "qx": p["orientation"][0], "qy": p["orientation"][1],
                                                   "qz": p["orientation"][2], "qw": p["orientation"][3]})
                    info["frame"] = self.cell.base_frame
                info["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            if persist:
                self.save()
        return dict(info)

    def sync(self, source, wanted):
        """Make the objects tagged ``source`` equal to ``wanted``; other sources, built-ins and attached
        objects stay. Everything is validated before the scene is touched."""
        if not source or source in ("scene", "manual"):
            raise err(400, "bad_source", "sync needs a source tag other than 'manual'/'scene'")
        if not isinstance(wanted, list):
            raise err(400, "bad_request", "objects must be a list")
        with self._lock:
            unknown = sorted({w.get("model") for w in wanted if w.get("model") not in self.models})
            if unknown:
                raise err(404, "unknown_model", f"models not registered: {unknown}", unknown_models=unknown)
            names = [w.get("name") for w in wanted]
            if len(set(names)) != len(names):
                raise err(400, "bad_request", "duplicate object names in sync")
            for i, w in enumerate(wanted):
                try:
                    self._check_name(w.get("name"))
                except Exception as e:
                    e.msg = f"objects[{i}]: {e.msg}"
                    raise
                ex = self.objects.get(w["name"])
                if ex and ex["source"] != source:
                    raise err(409, "conflict", f"object {w['name']!r} belongs to source {ex['source']!r}")
                pose_from_dict(w)
                self._frame(w.get("frame"))
            res = {"added": [], "updated": [], "removed": [], "kept_attached": []}
            for w in wanted:
                ex = self.objects.get(w["name"])
                if ex and ex.get("attached_link"):
                    res["kept_attached"].append(w["name"])
                    continue
                self.add_object(w["name"], w.get("model"), w, frame=w.get("frame"), source=source, persist=False)
                res["added" if ex is None else "updated"].append(w["name"])
            for n, info in list(self.objects.items()):
                if info["source"] == source and n not in names:
                    if info.get("attached_link"):
                        res["kept_attached"].append(n)
                        continue
                    self.remove_object(n, persist=False)
                    res["removed"].append(n)
            self.save()
        return res
