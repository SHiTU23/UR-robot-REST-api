"""/cell/surfaces: the tables the robot picks from and places on, as collision boxes whose HEIGHT comes from the
client's calibration (camera depth). Their footprints are in config/cell.yaml (section surfaces); a box is centred on
the measured point and its top is at the measured height.

    POST   /cell/surfaces/<name>/calibrate   calibration starts: the surface's box is taken OUT of the collision scene,
                                             so the calibration poses down at a new height are not blocked
    PUT    /cell/surfaces/<name>             calibration done: {point:[x,y,z]} or {points:[[x,y,z],...]} (depth
                                             measurements on the table top), frame_id base_link | base, size_m [x,y]
                                             -> the box is (re)built: top at the median z, centred on the mean x, y
    POST   /cell/surfaces/<name>/cancel      calibration abandoned: the previous box comes back
    GET    /cell/surfaces                    state of each surface: set | calibrating | not_set
    DELETE /cell/surfaces/<name>             remove the box

The boxes live in the scene namespace "cell.surfaces" (ids cell.surfaces.<name>): they persist and are restored at
start like every scene v2 object, show in RViz, and a client's /scene/batch on its own namespace leaves them alone.
Nothing here moves the robot; changes take the motion lock (409 busy while a motion runs). Bins and other objects keep
coming from the client through /scene/batch, e.g. once the bin's position is calibrated."""
import math
import threading
import time

from flask import Blueprint, jsonify, request

from ctx import C
from errors import err

bp = Blueprint("surfaces", __name__)
NS = "cell.surfaces"
MAX_POINTS = 1000
_calibrating = {}            # name -> {"since", "previous": the scene object or None}
_cal_lock = threading.Lock()


def _cfg(name):
    s = (C.cell.cfg.get("surfaces") or {}).get(name)
    if s is None:
        raise err(404, "unknown_surface", f"no surface {name!r}; config/cell.yaml defines "
                  f"{sorted(C.cell.cfg.get('surfaces') or {})}", "name")
    return s


def _oid(name):
    return f"{NS}.{name}"


def _current(name):
    return C.core.objects.get(_oid(name))


def _num(v, path):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise err(400, "bad_request", "must be a finite number", path)
    return float(v)


def _point(p, path, flip):
    if not isinstance(p, list) or len(p) != 3:
        raise err(400, "bad_request", "a point is [x, y, z] in metres", path)
    x, y, z = (_num(v, f"{path}[{i}]") for i, v in enumerate(p))
    return (-x, -y, z) if flip else (x, y, z)


def _body():
    d = request.get_json(silent=True)
    if not isinstance(d, dict):
        raise err(400, "bad_request", "JSON body must be an object")
    return d


def _remove(name):
    if _current(name) is not None:
        C.core.batch(C.core.validate({"namespace": NS, "mode": "remove", "ids": [_oid(name)]}), C.lock)


def _put_object(name, obj):
    """Add / replace the box, then let the configured links (e.g. the robot's base on its own table) touch it."""
    body = {"namespace": NS, "mode": "upsert", "objects": [{k: obj[k] for k in ("id", "frame_id", "pose", "shapes",
                                                                                "meta", "color") if k in obj}]}
    res = C.core.batch(C.core.validate(body), C.lock)
    allow = [lnk for lnk in (_cfg(name).get("allowed_links") or []) if lnk in C.kin.urdf.links]
    if allow:
        C.core.acm_set([{"a": _oid(name), "b": lnk, "allowed": True} for lnk in allow], C.lock)
    return res


def _state(name):
    o = _current(name)
    with _cal_lock:
        cal = _calibrating.get(name)
    out = {"name": name, "id": _oid(name), "status": "calibrating" if cal else ("set" if o else "not_set"),
           "description": _cfg(name).get("description")}
    if cal:
        out["calibrating_since"] = cal["since"]
    if o:
        m = o.get("meta") or {}
        out.update(top_z_m=m.get("top_z_m"), center_xy_m=m.get("center_xy_m"), size_m=m.get("size_m"),
                   measured_at=m.get("measured_at"), n_points=m.get("n_points"), z_spread_m=m.get("z_spread_m"))
    return out


def _covers(m, xy):
    return all(abs(xy[i] - m["center_xy_m"][i]) <= m["size_m"][i] / 2 for i in (0, 1))


def _overlaps(name, m):
    """Warnings when this box, higher than another table, covers the spot measured on that table (or the reverse):
    the higher box then blocks reaching down to the lower table there."""
    out = []
    for other, o in C.core.objects.items():
        om = (o.get("meta") or {}) if other.startswith(NS + ".") and other != _oid(name) else None
        if not om or "top_z_m" not in om:
            continue
        for hi, lo, hi_n, lo_n in ((m, om, name, om["surface"]), (om, m, om["surface"], name)):
            if hi["top_z_m"] > lo["top_z_m"] + 0.005 and _covers(hi, lo["center_xy_m"]):
                out.append(f"the {hi_n} box (top {hi['top_z_m']:.3f} m, {hi['size_m'][0]:.2f} x {hi['size_m'][1]:.2f} m "
                           f"around {hi['center_xy_m']}) covers the spot measured on the lower {lo_n} table "
                           f"{lo['center_xy_m']}: reaching down there is blocked. Make the {hi_n} footprint smaller, or "
                           f"set surfaces.{hi_n}.center_xy_m in config/cell.yaml")
    return out


@bp.route("/cell/surfaces", methods=["GET"])
def surfaces_list():
    """Each configured surface: status (set | calibrating | not_set), top_z_m, center_xy_m, size_m, measured_at."""
    return jsonify({"surfaces": [_state(n) for n in sorted(C.cell.cfg.get("surfaces") or {})],
                    "frame_id": C.cell.base_frame}), 200


@bp.route("/cell/surfaces/<name>/calibrate", methods=["POST"])
def surface_calibrate(name):
    """Calibration of this surface starts: its box leaves the collision scene until PUT (or cancel) brings it back.
    While it is out, NOTHING stops the arm at that table: move slowly and watch."""
    _cfg(name)
    with _cal_lock:
        if name not in _calibrating:
            _calibrating[name] = {"since": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "previous": _current(name)}
    _remove(name)
    return jsonify(dict(_state(name), warning=f"the {name} surface is out of the collision scene until "
                        f"PUT /cell/surfaces/{name} or POST .../cancel")), 200


@bp.route("/cell/surfaces/<name>/cancel", methods=["POST"])
def surface_cancel(name):
    """Calibration abandoned: the box from before /calibrate comes back (if there was one)."""
    _cfg(name)
    with _cal_lock:
        cal = _calibrating.pop(name, None)
    if cal and cal["previous"] is not None:
        _put_object(name, cal["previous"])
    return jsonify(_state(name)), 200


@bp.route("/cell/surfaces/<name>", methods=["PUT", "POST"])
def surface_set(name):
    """Calibration done: the table top measured at {point:[x,y,z]} or {points:[[x,y,z],...]} (frame_id base_link,
    or base = the pendant's frame, x and y negated). The box gets its top at the median z and is centred on the mean
    x, y (or on the configured center_xy_m), with the configured footprint (or size_m [x, y]).
    -> the surface's state, plus warnings."""
    cfg = _cfg(name)
    d = _body()
    frame = d.get("frame_id", C.cell.base_frame)
    if frame not in (C.cell.base_frame, C.cell.planning_frame, "base"):
        raise err(400, "bad_frame", f"frame_id must be {C.cell.base_frame} or base", "frame_id")
    raw = d.get("points", [d["point"]] if "point" in d else None)
    if not isinstance(raw, list) or not raw:
        raise err(400, "bad_request", "give point [x, y, z] or points [[x, y, z], ...]", "points")
    if len(raw) > MAX_POINTS:
        raise err(422, "limits_exceeded", f"at most {MAX_POINTS} points", "points")
    pts = [_point(p, f"points[{i}]", frame == "base") for i, p in enumerate(raw)]
    size = d.get("size_m", cfg["size_m"])
    if not isinstance(size, list) or len(size) != 2 or not all(0.05 <= _num(v, "size_m") <= 10.0 for v in size):
        raise err(400, "bad_request", "size_m is [x, y], each 0.05..10 m", "size_m")
    zs = sorted(p[2] for p in pts)
    z = zs[len(zs) // 2] if len(zs) % 2 else (zs[len(zs) // 2 - 1] + zs[len(zs) // 2]) / 2
    lo, hi = cfg.get("z_range_m", [-1.5, 1.0])
    if not lo <= z <= hi:
        raise err(422, "implausible_height", f"the {name} table top at z {z:.3f} m is outside {lo}..{hi} m "
                  "(surfaces.<name>.z_range_m in config/cell.yaml): check the measurement and its frame", "points")
    if cfg.get("center_xy_m") is not None:      # a fixed footprint: the measurement only sets the height
        cx, cy = (float(v) for v in cfg["center_xy_m"])
    else:
        cx, cy = (sum(p[i] for p in pts) / len(pts) for i in (0, 1))
    t = float(cfg.get("thickness_m", 0.05))
    warnings = []
    spread = zs[-1] - zs[0]
    if spread > 0.01:
        warnings.append(f"the points' heights spread over {spread * 1000:.0f} mm: the table is uneven or tilted, or "
                        "some points are not on the table top; the median is used")
    obj = {"id": _oid(name), "frame_id": C.cell.base_frame,
           "pose": {"position": [cx, cy, z - t / 2], "orientation": [0.0, 0.0, 0.0, 1.0]},
           "shapes": [{"type": "box", "size": [float(size[0]), float(size[1]), t],
                       "pose": {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}}],
           "color": cfg.get("color", [0.45, 0.55, 0.65, 0.5]),
           "meta": {"surface": name, "top_z_m": round(z, 5), "center_xy_m": [round(cx, 5), round(cy, 5)],
                    "size_m": [float(size[0]), float(size[1])], "n_points": len(pts), "z_spread_m": round(spread, 5),
                    "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}}
    warnings += _overlaps(name, obj["meta"])
    _put_object(name, obj)
    with _cal_lock:
        _calibrating.pop(name, None)
    contacts = []
    try:
        from routes_plan import _state as plan_state, _valid
        base, _q = plan_state(None, None)
        _v, contacts = _valid(base)
        contacts = [c for c in contacts if _oid(name) in (c.get("a"), c.get("b"))]
    except Exception:          # noqa: BLE001  (the check is advice; the box is set either way)
        pass
    if contacts:
        warnings.append(f"the robot is touching the new {name} box where it stands now ({contacts[:3]}): every plan "
                        "from here will fail; check the height, or lift the arm with the box out (calibrate)")
    return jsonify(dict(_state(name), warnings=warnings)), 200


@bp.route("/cell/surfaces/<name>", methods=["DELETE"])
def surface_delete(name):
    """Remove the surface's box (and forget a calibration in progress)."""
    _cfg(name)
    with _cal_lock:
        _calibrating.pop(name, None)
    _remove(name)
    return jsonify(_state(name)), 200
