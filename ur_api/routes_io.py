"""/camera/* (wrist camera over HTTP) and /files/* (sandboxed file store for CAD models)."""
import io
import os
import time

import numpy as np
from flask import Blueprint, Response, jsonify, request, send_file

from ctx import C
from file_store import FileStoreError

bp = Blueprint("io", __name__)


@bp.errorhandler(FileStoreError)
def _store_error(e):
    return jsonify({"error": e.code, "msg": e.msg}), e.status


def _flag(name, default=False):
    v = request.args.get(name)
    return default if v is None else v not in ("0", "false", "no", "")


# ------------------------------------------------------------------ camera

def _unavailable(e):
    return jsonify({"error": "camera_unavailable", "msg": e or "camera not reachable"}), 503


def _started():
    ok, e = C.camera.ensure_started()
    return None if ok else _unavailable(e)


@bp.route("/camera/info", methods=["GET"])
def camera_info():
    """Camera status, backend (orbbec | realsense | none), serial, colour intrinsics (fx, fy, ppx, ppy, coeffs),
    frame age. Does not start the camera."""
    return jsonify(C.camera.info()), 200


@bp.route("/camera/config", methods=["GET"])
def camera_config():
    """Without params: the current configuration. With width=&height=&fps=: restart the stream in that colour mode,
    validated on the device; on failure the previous one is restored (400). Depth is aligned to colour."""
    w, h, fps = (request.args.get(k) for k in ("width", "height", "fps"))
    if not any((w, h, fps)):
        return jsonify({"current": {"width": C.camera.width, "height": C.camera.height, "fps": C.camera.fps,
                                    "streaming": C.camera.info()["streaming"]},
                        "supported_modes": C.camera.supported_modes()}), 200
    try:
        vals = {k: int(v) for k, v in (("width", w), ("height", h), ("fps", fps)) if v is not None}
        if any(not 0 < v <= 4096 for v in vals.values()):
            raise ValueError
    except (TypeError, ValueError):
        return jsonify({"error": "width/height/fps must be positive integers"}), 400
    ok, e = C.camera.set_config(**vals)
    if not ok:
        return jsonify({"error": "config_rejected", "msg": e, "restored": {"width": C.camera.width,
                        "height": C.camera.height, "fps": C.camera.fps}}), 400
    return jsonify({"status": "reconfigured", **C.camera.info()}), 200


@bp.route("/camera/start", methods=["GET"])
def camera_start():
    """Start the stream (idempotent; the image endpoints start it too). 503 if the device is missing or busy."""
    r = _started()
    return r if r else (jsonify({"status": "streaming", **C.camera.info()}), 200)


@bp.route("/camera/stop", methods=["GET"])
def camera_stop():
    """Stop the stream and release the device."""
    C.camera.stop()
    return jsonify({"status": "stopped"}), 200


def _image(img, fmt, quality):
    import cv2
    if fmt == "png":
        ok, buf = cv2.imencode(".png", img)
        return ok, buf, "image/png"
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, quality))])
    return ok, buf, "image/jpeg"


@bp.route("/camera/color", methods=["GET"])
def camera_color():
    """Latest colour frame: format=jpg|png (png = lossless), quality=1..100 (jpg, 85). X-Frame-Id / X-Frame-TS
    identify the frameset (compare with /camera/depth, or use /camera/frame for a guaranteed pair)."""
    r = _started()
    if r:
        return r
    fmt = request.args.get("format", "jpg").lower()
    if fmt not in ("jpg", "jpeg", "png"):
        return jsonify({"error": "format must be jpg or png"}), 400
    try:
        q = int(request.args.get("quality", "85"))
    except ValueError:
        return jsonify({"error": "bad quality"}), 400
    fr = C.camera.latest()
    if fr is None:
        return _unavailable("no frame yet")
    ok, buf, mime = _image(fr["color"], fmt, q)
    if not ok:
        return jsonify({"error": "encode failed"}), 500
    resp = Response(buf.tobytes(), mimetype=mime)
    resp.headers["X-Frame-Id"], resp.headers["X-Frame-TS"] = str(fr["id"]), repr(fr["ts"])
    return resp


@bp.route("/camera/depth", methods=["GET"])
def camera_depth():
    """Latest depth aligned to colour as a 16-bit PNG in MILLIMETRES (0 = no data), lossless."""
    r = _started()
    if r:
        return r
    b = C.camera.frame_bundle()
    if b is None:
        return _unavailable("no frame yet")
    import cv2
    ok, buf = cv2.imencode(".png", b["depth_mm"])
    if not ok:
        return jsonify({"error": "png encode failed"}), 500
    resp = Response(buf.tobytes(), mimetype="image/png")
    resp.headers["X-Depth-Units"], resp.headers["X-Frame-Id"], resp.headers["X-Frame-TS"] = \
        "millimeters", str(b["id"]), repr(b["ts"])
    return resp


@bp.route("/camera/frame", methods=["GET"])
def camera_frame():
    """Colour + depth from the SAME frameset: a compressed .npz with color_bgr (HxWx3 u8), depth_mm (HxW u16),
    frame_id, ts, fx, fy, ppx, ppy, depth_scale_m. np.load(io.BytesIO(resp.content))."""
    r = _started()
    if r:
        return r
    b = C.camera.frame_bundle()
    if b is None:
        return _unavailable("no frame yet")
    bio = io.BytesIO()
    np.savez_compressed(bio, color_bgr=b["color"], depth_mm=b["depth_mm"], frame_id=np.int64(b["id"]),
                        ts=np.float64(b["ts"]), fx=np.float64(b["fx"]), fy=np.float64(b["fy"]),
                        ppx=np.float64(b["ppx"]), ppy=np.float64(b["ppy"]), depth_scale_m=np.float64(b["depth_scale_m"]))
    resp = Response(bio.getvalue(), mimetype="application/octet-stream")
    resp.headers["Content-Disposition"] = f"attachment; filename=frame_{b['id']}.npz"
    resp.headers["X-Frame-Id"], resp.headers["X-Frame-TS"] = str(b["id"]), repr(b["ts"])
    return resp


@bp.route("/camera/stream", methods=["GET"])
def camera_stream():
    """Live MJPEG for a browser (<img src="/camera/stream">): view=color|depth, fps=10 (1..30), quality=70,
    max_m=4.0 (depth colour range). Ends when the browser closes it."""
    r = _started()
    if r:
        return r
    view = request.args.get("view", "color").lower()
    if view not in ("color", "depth"):
        return jsonify({"error": "view must be color or depth"}), 400
    try:
        fps = max(1.0, min(30.0, float(request.args.get("fps", "10"))))
        quality = max(1, min(100, int(request.args.get("quality", "70"))))
        max_m = float(request.args.get("max_m", "4.0"))
        if not 0.1 <= max_m <= 20:
            raise ValueError
    except ValueError:
        return jsonify({"error": "bad fps / quality / max_m"}), 400
    import cv2

    def frames():
        last, sent, idle = -1, 0.0, time.time()
        while True:
            fr, now = C.camera.latest(), time.time()
            if fr is None or not C.camera.info()["streaming"]:
                if now - idle > 5.0:
                    return
                time.sleep(0.1)
                continue
            if fr["id"] == last or now - sent < 1.0 / fps:
                time.sleep(0.005)
                continue
            idle = now
            img = C.camera.depth_preview_bgr(max_m) if view == "depth" else fr["color"]
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                return
            last, sent = fr["id"], now
            jpg = buf.tobytes()
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" \
                + jpg + b"\r\n"

    resp = Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    return resp


@bp.route("/camera/depth/preview", methods=["GET"])
def camera_depth_preview():
    """Colourised depth (JET, max_m=4.0 far clip, black = no data) as JPEG, for people."""
    r = _started()
    if r:
        return r
    try:
        max_m = float(request.args.get("max_m", "4.0"))
        if not 0.1 <= max_m <= 20:
            raise ValueError
    except ValueError:
        return jsonify({"error": "bad max_m (0.1..20)"}), 400
    img = C.camera.depth_preview_bgr(max_m)
    if img is None:
        return _unavailable("no frame yet")
    ok, buf, mime = _image(img, "jpg", 85)
    return Response(buf.tobytes(), mimetype=mime)


@bp.route("/camera/depth_at", methods=["GET"])
def camera_depth_at():
    """Median depth (m) in a win x win window (default 5, zeros ignored) at pixel u, v of the aligned images,
    and the deprojected point_camera_m (camera frame: x right, y down, z forward)."""
    r = _started()
    if r:
        return r
    try:
        u, v, win = int(request.args.get("u")), int(request.args.get("v")), int(request.args.get("win", "5"))
        if not 1 <= win <= 51:
            raise ValueError
    except (TypeError, ValueError):
        return jsonify({"error": "need integer u, v (and optional win 1..51)"}), 400
    out = C.camera.depth_at(u, v, win)
    if out is None:
        return _unavailable("no frame yet")
    return jsonify(out), 400 if "error" in out else 200


# ------------------------------------------------------------------ files

def _upload(rel):
    ap = C.store.resolve(rel)
    overwrite = _flag("overwrite", True)
    # only a multipart body is a form upload: touching request.files for other bodies makes Werkzeug parse
    # them as a form, and a raw body sent as x-www-form-urlencoded (urllib's default) would be lost
    if request.mimetype == "multipart/form-data":
        f = request.files.get("file")
        if f is None:
            raise FileStoreError(400, "bad_request", 'multipart upload needs a field named "file"')
        info = C.store.write_stream(ap, f.stream, overwrite=overwrite)
    else:
        info = C.store.write_stream(ap, request.stream, declared_length=request.content_length, overwrite=overwrite)
    return jsonify({"status": "stored", **info}), 201


@bp.route("/files", methods=["GET"], strict_slashes=False)
def files_root():
    """List the store root, or search it: q=<glob> (e.g. *.stl), recursive=1, hash=1 (sha256 per file). Usage and
    quota included."""
    q = request.args.get("q")
    body = {"root": "", "usage_bytes": C.store.usage_bytes(), "quota_bytes": C.store.quota_bytes,
            "max_file_bytes": C.store.max_file_bytes}
    if q:
        body.update(query=q, entries=C.store.search(q, _flag("recursive", True), _flag("hash")))
    else:
        body["entries"] = C.store.listdir(C.store.root, _flag("hash"))
    return jsonify(body), 200


@bp.route("/files/<path:rel>", methods=["GET"])
def files_get(rel):
    """Download the file (ETag = sha256), or list the directory (hash=1 adds sha256)."""
    ap = C.store.resolve(rel)
    if os.path.isdir(ap):
        return jsonify({"root": C.store.relpath(ap), "entries": C.store.listdir(ap, _flag("hash"))}), 200
    if not os.path.isfile(ap):
        raise FileStoreError(404, "not_found", "no such file")
    resp = send_file(ap, as_attachment=_flag("download", True), download_name=os.path.basename(ap), conditional=True)
    resp.headers["ETag"], resp.headers["X-File-Type"] = C.store.sha256(ap), "file"
    return resp


@bp.route("/files/<path:rel>", methods=["PUT"])
def files_put(rel):
    """Upload (create / overwrite) the file: raw body (curl -T part.stl .../files/cad/part.stl) or multipart field
    "file". overwrite=0 refuses to replace. Atomic. 413 over the per-file limit, 507 over the quota."""
    return _upload(rel)


@bp.route("/files/<path:rel>", methods=["POST"])
def files_post(rel):
    """Same as PUT (HTML forms); mkdir=1 creates the directory instead."""
    if _flag("mkdir"):
        return jsonify({"status": "created", **C.store.mkdir(C.store.resolve(rel))}), 201
    return _upload(rel)


@bp.route("/files/<path:rel>", methods=["DELETE"])
def files_delete(rel):
    """Delete the file (a non-empty directory needs recursive=1)."""
    C.store.delete(C.store.resolve(rel), recursive=_flag("recursive"))
    return jsonify({"status": "deleted", "path": rel.strip("/")}), 200
