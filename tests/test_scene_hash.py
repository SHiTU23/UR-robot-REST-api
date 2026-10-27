"""The scene hash must equal what clients compute (pipeFit checks namespace_hash against its own value).
`client_namespace_hash` below is the client function published in the Franka REST API README, verbatim apart
from the frame default (panda_link0 -> base_link) and dims() (see there); scene_core.hash_objects must agree with
it on random objects.
Also: the scene core against a fake MoveIt (validation, unchanged batches, rollback, dirty namespaces)."""
import hashlib
import json
import math
import os
import random
import sys
import threading
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ur_api"))
import scene_core as SC  # noqa: E402
from errors import ApiError  # noqa: E402


# ---- the README's client-side hash (Franka REST API README, "Hash"), frame default changed to base_link ----
def f(v, d):
    s = "%.*f" % (d, v)
    return s[1:] if s.startswith("-") and float(s) == 0 else s


def pose(p):
    q = p["orientation"]
    n = sum(x * x for x in q) ** .5
    q = [x / n for x in q]
    q = [-x for x in q] if q[3] < 0 else q
    return {"position": [f(x, 6) for x in p["position"]], "orientation": [f(x, 9) for x in q]}


def dims(s):
    # NOTE: the README builds this dict eagerly ({"box": s["size"], "sphere": [s["radius"]], ...}[type]), which
    # raises KeyError for every shape (none has all the keys). Same values, evaluated lazily:
    return {"box": lambda: s["size"], "sphere": lambda: [s["radius"]], "cylinder": lambda: [s["radius"], s["length"]],
            "mesh": lambda: [s.get("scale", 1.0)]}[s["type"]]()


def client_namespace_hash(objects):
    out = [{"id": o["id"], "frame_id": o.get("frame_id", "base_link"), "pose": pose(o["pose"]),
            "attached_link": None,
            "shapes": [{"type": s["type"], "dims": [f(x, 6) for x in dims(s)],
                        "pose": pose(s.get("pose") or {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}),
                        "mesh_sha256": s["sha256"] if s["type"] == "mesh" else None} for s in o["shapes"]]}
           for o in sorted(objects, key=lambda o: o["id"])]
    return "sha256:" + hashlib.sha256(json.dumps(out, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


class Cell:
    base_frame, planning_frame = "base_link", "world"
    builtin_world = ("table",)

    def touch_links(self, link):
        return [link, "rg6_base_link"]


class FakeIO:
    """In-memory move_group: world and attached ids; apply() can be told to fail."""

    def __init__(self):
        self.world, self.attached, self.fail = {}, {}, 0
        self.acm = {"names": [], "values": {}, "defaults": {}}

    def apply(self, ops, cache, acm=None, padding=None, colors=None):
        if self.fail:
            self.fail -= 1
            return False
        for op, oid, arg in ops:
            if op == "remove":
                self.world.pop(oid, None)
            elif op == "add":
                self.world[oid] = arg
            elif op == "attach":
                self.attached[oid] = self.world.pop(oid, None)
            elif op == "detach":
                self.world[oid] = self.attached.pop(oid, None)
        if acm is not None:
            self.acm = acm
        return True

    def world_names(self):
        return list(self.world)

    def attached_names(self):
        return list(self.attached)

    def get_acm(self):
        return self.acm

    def world_poses(self, ids):
        return {}

    def get_padding(self):
        return {}


def rnd_obj(i, rng):
    shapes = []
    for _ in range(rng.randint(1, 4)):
        t = rng.choice(["box", "sphere", "cylinder"])
        s = {"type": t, "pose": {"position": [rng.uniform(-1, 1) for _ in range(3)],
                                 "orientation": [rng.uniform(-1, 1) for _ in range(4)]}}
        if t == "box":
            s["size"] = [rng.uniform(0.001, 1) for _ in range(3)]
        elif t == "sphere":
            s["radius"] = rng.uniform(0.001, 1)
        else:
            s["radius"], s["length"] = rng.uniform(0.001, 1), rng.uniform(0.001, 1)
        shapes.append(s)
    # include the awkward cases: -0.0, tiny negatives that print as -0.000000, w < 0
    pos = [rng.choice([rng.uniform(-2, 2), -0.0, -1e-9]) for _ in range(3)]
    q = [rng.uniform(-1, 1) for _ in range(3)] + [rng.choice([-0.3, 0.7])]
    return {"id": f"pipefit.t.o{i}", "pose": {"position": pos, "orientation": q}, "shapes": shapes}


class TestHash(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.dir = tempfile.mkdtemp()
        from file_store import FileStore
        self.io = FakeIO()
        self.core = SC.SceneCore(self.io, FileStore(os.path.join(self.dir, "files")),
                                 os.path.join(self.dir, "state.json"), Cell())
        self.lock = threading.Lock()

    def test_matches_client_hash(self):
        rng = random.Random(7)
        for trial in range(200):
            objs = [rnd_obj(i, rng) for i in range(rng.randint(1, 6))]
            req = self.core.validate({"namespace": "pipefit.t", "objects": objs, "validate_only": True})
            ours = SC.hash_objects(list(req["objects"].values()))
            self.assertEqual(ours, client_namespace_hash(objs), f"trial {trial}")

    def test_batch_semantics(self):
        rng = random.Random(3)
        objs = [rnd_obj(i, rng) for i in range(3)]
        r = self.core.batch(self.core.validate({"namespace": "pipefit.t", "objects": objs}), self.lock)
        self.assertEqual(len(r["added"]), 3)
        self.assertEqual(r["namespace_hash"], client_namespace_hash(objs))
        self.assertEqual(self.core.version, 1)
        r = self.core.batch(self.core.validate({"namespace": "pipefit.t", "objects": objs}), self.lock)
        self.assertTrue(r["unchanged"])
        self.assertEqual(self.core.version, 1)
        # a failed apply rolls back; a failed rollback marks the namespace dirty
        self.io.fail = 1
        with self.assertRaises(ApiError) as e:
            self.core.batch(self.core.validate({"namespace": "pipefit.t", "objects": objs[:1]}), self.lock)
        self.assertTrue(e.exception.extra["rolled_back"])
        self.assertFalse(self.core.dirty)
        self.io.fail = 2
        with self.assertRaises(ApiError):
            self.core.batch(self.core.validate({"namespace": "pipefit.t", "objects": objs[:1]}), self.lock)
        self.assertTrue(self.core.dirty)
        r = self.core.batch(self.core.validate({"namespace": "pipefit.t", "objects": objs[:1]}), self.lock)
        self.assertFalse(self.core.dirty)                 # a good batch on the namespace resynchronises it
        self.assertEqual(sorted(self.io.world), ["pipefit.t.o0"])

    def test_validation(self):
        bad = [({"namespace": "builtin.x", "objects": []}, "bad_request"),
               ({"namespace": "pipefit.t", "objects": [{"id": "other.o", "shapes": [{"type": "sphere", "radius": 1}]}]},
                "bad_id"),
               ({"namespace": "pipefit.t", "objects": [{"id": "pipefit.t.o", "frame_id": "panda_link0",
                                                        "shapes": [{"type": "sphere", "radius": 1}]}]}, "bad_frame"),
               ({"namespace": "pipefit.t", "objects": [{"id": "pipefit.t.o", "pose": {"position": [0, 0, 0],
                                                                                     "rpy": [0, 0, 0]},
                                                        "shapes": [{"type": "sphere", "radius": 1}]}]}, "bad_pose"),
               ({"namespace": "pipefit.t", "objects": [{"id": "pipefit.t.o", "shapes": [{"type": "box",
                                                                                        "size": [1, 0, 1]}]}]},
                "bad_shape")]
        for body, code in bad:
            with self.assertRaises(ApiError) as e:
                self.core.validate(body)
            self.assertEqual(e.exception.code, code, body)


if __name__ == "__main__":
    unittest.main()
