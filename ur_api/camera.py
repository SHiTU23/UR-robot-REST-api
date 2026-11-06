"""Wrist camera over HTTP (/camera/*): colour + depth aligned to colour, on a background grab thread.

Backends (cell.yaml camera.backend, env UR_CAMERA): ``orbbec`` (Orbbec Gemini 2, the UR10e's camera since
2026-09-14; pyorbbecsdk v2), ``realsense`` (Intel D4xx; pyrealsense2) or ``none``. The device opens lazily
on the first request. Depth is always delivered as uint16 MILLIMETRES aligned to colour, so (u, v) indexes
the same point in both images and deprojection uses the colour intrinsics.

Orbbec facts (checked on this Gemini 2 by the sota UI, 2026-09-14): depth is offered as Y16, Y14 and RLE --
only Y16 is plain uint16, so it is pinned (the others give garbage, not an error); the raw unit is
get_depth_scale() mm, not necessarily 1; keep the Context alive or its device list breaks; colour comes as
MJPG/YUYV/... and is converted in the SDK. No post-processing filters: raw sensor depth, as the Franka API.
"""
import threading
import time

import numpy as np


class _Camera:
    backend = "none"

    def __init__(self, width=1280, height=720, fps=15):
        self.width, self.height, self.fps = width, height, fps
        self._lock = threading.Lock()
        self._start_lock = threading.RLock()     # one start / stop / mode change at a time: requests arrive in parallel
        self._frame = None            # {"color": bgr u8, "depth_mm": u16, "ts", "id"}
        self._running = False
        self._seq = 0
        self._err = None
        self._serial = None
        self._intr = None             # {"fx", "fy", "ppx", "ppy", "coeffs", "model"}
        self._depth_res = None
        self.depth_width, self.depth_height = None, None    # depth sensor mode; None = the largest at this fps
        self.align = True             # True: depth resampled onto the colour image; False: depth at its own resolution
        self._depth_intr = None       # depth-stream intrinsics, needed when not aligned
        self.exposure_wanted = {}     # {"auto", "exposure", "gain"} to re-apply whenever the stream starts

    # backend hooks: _open() -> starts the device, sets _serial/_intr/_depth_res; _grab() -> (bgr, depth_mm) | None;
    # _close()
    def start(self):
        with self._start_lock:
            if self._running:
                return True, None
            try:
                self._open()
            except Exception as e:      # noqa: BLE001
                self._err = str(e)
                try:
                    self._close()
                except Exception:       # noqa: BLE001
                    pass
                return False, str(e)
            self._running, self._err = True, None
            self._loop_thread = threading.Thread(target=self._loop, daemon=True, name=f"{self.backend}-grab")
            self._loop_thread.start()
            return True, None

    def _loop(self):
        while self._running:
            try:
                got = self._grab()
                if got is None:
                    continue
                color, depth = got
                with self._lock:
                    self._seq += 1
                    self._frame = {"color": color, "depth_mm": depth, "ts": time.time(), "id": self._seq}
            except Exception as e:      # noqa: BLE001
                if self._running:
                    self._err = str(e)
                    time.sleep(0.2)

    def stop(self):
        with self._start_lock:
            self._running = False
            t = getattr(self, "_loop_thread", None)
            if t is not None:
                t.join(timeout=2.0)      # the grab loop must be out before the device goes away
            try:
                self._close()
            except Exception:       # noqa: BLE001
                pass
            with self._lock:
                self._frame = None

    def ensure_started(self):
        ok, e = self.start()
        if not ok:
            return False, e
        for _ in range(60):
            if self.latest() is not None:
                return True, None
            time.sleep(0.05)
        return False, self._err or "no frames from the camera yet"

    def latest(self):
        with self._lock:
            return self._frame

    def _settings(self):
        return self.width, self.height, self.fps, self.depth_width, self.depth_height, self.align

    def _apply(self, s):
        self.width, self.height, self.fps, self.depth_width, self.depth_height, self.align = s

    def set_config(self, width=None, height=None, fps=None, depth=None, align=None):
        """depth: (w, h), or "auto" for the largest depth mode at this fps; None keeps the current choice."""
        with self._start_lock:
            return self._set_config(width, height, fps, depth, align)

    def _set_config(self, width, height, fps, depth, align):
        old, was = self._settings(), self._running
        dw, dh = (None, None) if depth == "auto" else depth if depth else old[3:5]
        new = (int(width or old[0]), int(height or old[1]), int(fps or old[2]), dw, dh, old[5] if align is None else bool(align))
        if new == old and was:
            return True, None
        if was:
            self.stop()
        self._apply(new)
        ok, e = self.ensure_started()
        if not ok:
            self._apply(old)
            if was:
                self.ensure_started()
            return False, e
        return True, None

    def supported_modes(self):
        return {"note": "the device decides; /camera/config validates a mode and rolls back on failure"}

    def get_exposure(self):
        raise NotImplementedError(f"colour exposure control is not available on backend {self.backend!r}")

    def set_exposure(self, auto=None, exposure=None, gain=None):
        raise NotImplementedError(f"colour exposure control is not available on backend {self.backend!r}")

    def info(self):
        fr = self.latest()
        out = {"backend": self.backend, "streaming": self._running, "has_frame": fr is not None, "width": self.width,
               "height": self.height, "fps": self.fps, "serial": self._serial, "depth_sensor_res": self._depth_res,
               "depth_mode": "auto" if self.depth_width is None else [self.depth_width, self.depth_height],
               "aligned_to_color": self.align,
               "depth_scale_m": 0.001, "depth_units": "uint16 millimetres", "last_error": self._err}
        if fr is not None:
            out["frame_age_s"] = round(time.time() - fr["ts"], 3)
            h, w = fr["depth_mm"].shape
            out["depth_image_res"] = [w, h]
        if self._intr is not None:
            note = ("colour-stream intrinsics; depth is aligned to colour, so (u,v) indexes both images" if self.align else
                    "colour-stream intrinsics; depth is NOT aligned: use depth_intrinsics for depth pixels, (u,v) differ")
            out["intrinsics"] = dict(self._intr, note=note)
        if not self.align and self._depth_intr is not None:
            out["depth_intrinsics"] = dict(self._depth_intr)
        return out

    def frame_bundle(self):
        fr = self.latest()
        if fr is None:
            return None
        i = self._intr or {}
        di = i if self.align else (self._depth_intr or {})
        return {"color": fr["color"], "depth_mm": fr["depth_mm"], "ts": fr["ts"], "id": fr["id"],
                "fx": i.get("fx", 0.0), "fy": i.get("fy", 0.0), "ppx": i.get("ppx", 0.0), "ppy": i.get("ppy", 0.0),
                "depth_fx": di.get("fx", 0.0), "depth_fy": di.get("fy", 0.0), "depth_ppx": di.get("ppx", 0.0),
                "depth_ppy": di.get("ppy", 0.0), "aligned_to_color": self.align, "depth_scale_m": 0.001}

    def depth_at(self, u, v, win=5):
        fr = self.latest()
        if fr is None:
            return None
        d = fr["depth_mm"]
        h, w = d.shape
        if not (0 <= u < w and 0 <= v < h):
            return {"error": f"pixel out of range (image is {w}x{h})"}
        r = max(1, int(win) // 2)
        patch = d[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return {"u": u, "v": v, "depth_m": None, "msg": "no valid depth at this pixel (hole/too close/too far)"}
        z = float(np.median(valid)) / 1000.0
        out = {"u": u, "v": v, "depth_m": round(z, 4), "valid_samples": int(valid.size)}
        i = self._intr if self.align else self._depth_intr
        if i:
            out["point_camera_m"] = {"x": round((u - i["ppx"]) / i["fx"] * z, 4),
                                     "y": round((v - i["ppy"]) / i["fy"] * z, 4), "z": round(z, 4),
                                     "note": "pinhole deprojection (lens distortion not applied)"}
        return out

    def depth_preview_bgr(self, max_m=4.0):
        import cv2
        fr = self.latest()
        if fr is None:
            return None
        n = np.clip(fr["depth_mm"].astype(np.float32) / (max_m * 1000.0), 0, 1)
        img = cv2.applyColorMap((n * 255).astype(np.uint8), cv2.COLORMAP_JET)
        img[fr["depth_mm"] == 0] = 0
        return img


class NoCamera(_Camera):
    backend = "none"

    def _open(self):
        raise RuntimeError("no camera configured (cell.yaml camera.backend / env UR_CAMERA = orbbec | realsense)")

    def _close(self):
        pass


class OrbbecCamera(_Camera):
    backend = "orbbec"

    def _open(self):
        import pyorbbecsdk as ob
        self.ob = ob
        self._ctx = ob.Context()                  # must stay referenced (device list breaks otherwise)
        self._pipe = ob.Pipeline()
        cp = self._color_profile()
        dp = self._depth_profile()
        cfg = ob.Config()
        cfg.enable_stream(cp)
        cfg.enable_stream(dp)
        self._pipe.start(cfg)
        self._align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM) if self.align else None
        conv = {ob.OBFormat.MJPG: ob.OBConvertFormat.MJPG_TO_RGB888, ob.OBFormat.YUYV: ob.OBConvertFormat.YUYV_TO_RGB888,
                ob.OBFormat.UYVY: ob.OBConvertFormat.UYVY_TO_RGB888, ob.OBFormat.I420: ob.OBConvertFormat.I420_TO_RGB888,
                ob.OBFormat.NV12: ob.OBConvertFormat.NV12_TO_RGB888, ob.OBFormat.NV21: ob.OBConvertFormat.NV21_TO_RGB888}
        self._conv = None
        if cp.get_format() != ob.OBFormat.RGB:
            if cp.get_format() not in conv:
                raise RuntimeError(f"cannot convert colour format {cp.get_format()} to RGB")
            self._conv = ob.FormatConvertFilter()
            self._conv.set_format_convert_format(conv[cp.get_format()])
        try:
            self._serial = self._pipe.get_device().get_device_info().get_serial_number()
        except Exception:       # noqa: BLE001
            self._serial = None
        prm = self._pipe.get_camera_param()
        ri, rd = prm.rgb_intrinsic, prm.rgb_distortion
        self._intr = {"fx": ri.fx, "fy": ri.fy, "ppx": ri.cx, "ppy": ri.cy, "model": "brown_conrady (k1..k6, p1, p2)",
                      "coeffs": [rd.k1, rd.k2, rd.p1, rd.p2, rd.k3, rd.k4, rd.k5, rd.k6]}
        di = prm.depth_intrinsic
        self._depth_intr = {"fx": di.fx, "fy": di.fy, "ppx": di.cx, "ppy": di.cy}
        self.width, self.height = cp.get_width(), cp.get_height()
        self._depth_res = (dp.get_width(), dp.get_height())
        if self.exposure_wanted:
            self._write_exposure(**self.exposure_wanted)

    def _exposure_props(self):
        ob = self.ob
        dev = self._pipe.get_device()
        ids = {"auto": ob.OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, "exposure": ob.OBPropertyID.OB_PROP_COLOR_EXPOSURE_INT,
               "gain": ob.OBPropertyID.OB_PROP_COLOR_GAIN_INT}
        return dev, ids

    @staticmethod
    def _range(r):
        return {"min": int(r.min), "max": int(r.max), "step": int(r.step)}

    def get_exposure(self):
        """{auto, exposure, gain, exposure_range {min, max, step}, gain_range}, read from the camera."""
        with self._start_lock:
            return self._read_exposure()

    def _read_exposure(self):
        dev, ids = self._exposure_props()
        return {"auto": bool(dev.get_bool_property(ids["auto"])), "exposure": dev.get_int_property(ids["exposure"]),
                "gain": dev.get_int_property(ids["gain"]), "exposure_range": self._range(dev.get_int_property_range(ids["exposure"])),
                "gain_range": self._range(dev.get_int_property_range(ids["gain"]))}

    def _write_exposure(self, auto=None, exposure=None, gain=None):
        dev, ids = self._exposure_props()
        if auto is not None:
            dev.set_bool_property(ids["auto"], bool(auto))
        if exposure is not None:
            dev.set_int_property(ids["exposure"], int(exposure))
        if gain is not None:
            dev.set_int_property(ids["gain"], int(gain))

    def set_exposure(self, auto=None, exposure=None, gain=None):
        """Change only what is given. A manual exposure or gain turns auto exposure off, unless auto=True is given.
        Values outside the camera's range raise ValueError; nothing is changed then. Kept across stream restarts."""
        if auto is True and (exposure is not None or gain is not None):
            raise ValueError("exposure and gain are set by the camera while auto=1; send auto=0 with them, or auto=1 alone")
        with self._start_lock:
            return self._set_exposure(auto, exposure, gain)

    def _set_exposure(self, auto, exposure, gain):
        cur = self._read_exposure()
        for key, val in (("exposure", exposure), ("gain", gain)):
            if val is None:
                continue
            r = cur[key + "_range"]
            if not r["min"] <= val <= r["max"]:
                raise ValueError(f"{key} must be in {r['min']}..{r['max']}, got {val}")
        if auto is None and (exposure is not None or gain is not None):
            auto = False
        self._write_exposure(auto, exposure, gain)
        if auto:          # the camera decides again
            self.exposure_wanted = {"auto": True}
        else:
            kept = {k: v for k, v in self.exposure_wanted.items() if k != "auto"}
            kept.update({k: v for k, v in (("exposure", exposure), ("gain", gain)) if v is not None})
            self.exposure_wanted = dict(kept, auto=False if auto is False else self.exposure_wanted.get("auto"))
            if self.exposure_wanted["auto"] is None:
                del self.exposure_wanted["auto"]
        return self._read_exposure()

    def _color_profile(self):
        ob = self.ob
        profiles = self._pipe.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        for fmt in (ob.OBFormat.RGB, ob.OBFormat.UNKNOWN_FORMAT):
            try:
                p = profiles.get_video_stream_profile(self.width, self.height, fmt, self.fps)
                if p is not None:
                    return p
            except Exception:       # noqa: BLE001
                pass
        raise RuntimeError(f"colour {self.width}x{self.height}@{self.fps} is not offered by this camera")

    def _depth_profile(self):
        ob = self.ob
        profiles = self._pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
        best = None
        for i in range(profiles.get_count()):
            p = profiles.get_stream_profile_by_index(i).as_video_stream_profile()
            if p.get_format() != ob.OBFormat.Y16 or p.get_fps() != self.fps:
                continue
            if self.depth_width is not None and (p.get_width(), p.get_height()) != (self.depth_width, self.depth_height):
                continue
            if best is None or p.get_width() * p.get_height() > best.get_width() * best.get_height():
                best = p
        if best is None:
            want = "" if self.depth_width is None else f" {self.depth_width}x{self.depth_height}"
            raise RuntimeError(f"no Y16 depth mode{want} at {self.fps} fps (Y14/RLE are not plain uint16); "
                               "GET /camera/config lists the modes")
        return best

    def supported_modes(self):
        """What this camera offers: {color: [[w, h, fps], ...], depth: [[w, h, fps], ...]} (depth: plain Y16 only)."""
        import pyorbbecsdk as ob
        pipe = getattr(self, "_pipe", None)
        if pipe is None:
            ctx = ob.Context()      # noqa: F841  (must stay referenced while the pipeline is used)
            pipe = ob.Pipeline()
        out = {}
        for key, sensor in (("color", ob.OBSensorType.COLOR_SENSOR), ("depth", ob.OBSensorType.DEPTH_SENSOR)):
            ps = pipe.get_stream_profile_list(sensor)
            modes = set()
            for i in range(ps.get_count()):
                p = ps.get_stream_profile_by_index(i).as_video_stream_profile()
                if key == "color" or p.get_format() == ob.OBFormat.Y16:
                    modes.add((p.get_width(), p.get_height(), p.get_fps()))
            out[key] = [list(m) for m in sorted(modes, reverse=True)]
        return out

    def _grab(self):
        frames = self._pipe.wait_for_frames(1000)
        if frames is None:
            return None
        aligned = frames
        if self._align is not None:
            aligned = self._align.process(frames)
            if aligned is None:
                return None
            aligned = aligned.as_frame_set()
        c, d = aligned.get_color_frame(), aligned.get_depth_frame()
        if c is None or d is None:
            return None
        if self._conv is not None:
            c = self._conv.process(c).as_video_frame()
        rgb = np.frombuffer(c.get_data(), np.uint8).reshape((c.get_height(), c.get_width(), 3))
        raw = np.frombuffer(d.get_data(), np.uint16).reshape((d.get_height(), d.get_width()))
        s = d.get_depth_scale()
        mm = raw.copy() if abs(s - 1.0) < 1e-6 else np.clip(np.round(raw.astype(np.float32) * s), 0, 65535).astype(np.uint16)
        return rgb[..., ::-1].copy(), mm

    def _close(self):
        if getattr(self, "_pipe", None) is not None:
            self._pipe.stop()
            self._pipe = None


class RealsenseCamera(_Camera):
    backend = "realsense"

    def _open(self):
        import pyrealsense2 as rs
        self.rs = rs
        self._pipe = rs.pipeline()
        prof, last = None, None
        modes = [(self.depth_width, self.depth_height)] if self.depth_width is not None else \
            dict.fromkeys([(self.width, self.height), (848, 480), (640, 480), (1280, 720)])
        for dw, dh in modes:
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, self.fps)
            try:
                prof = self._pipe.start(cfg)
                self._depth_res = (dw, dh)
                break
            except Exception as e:      # noqa: BLE001
                last = e
        if prof is None:
            raise last or RuntimeError("no supported stream combination")
        dev = prof.get_device()
        self._serial = dev.get_info(rs.camera_info.serial_number)
        self._scale_mm = float(dev.first_depth_sensor().get_depth_scale()) * 1000.0
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._intr = {"fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy, "model": str(i.model), "coeffs": list(i.coeffs)}
        di = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self._depth_intr = {"fx": di.fx, "fy": di.fy, "ppx": di.ppx, "ppy": di.ppy}
        self._align = rs.align(rs.stream.color) if self.align else None

    def _grab(self):
        fs = self._pipe.wait_for_frames(timeout_ms=5000)
        if self._align is not None:
            fs = self._align.process(fs)
        c, d = fs.get_color_frame(), fs.get_depth_frame()
        if not c or not d:
            return None
        raw = np.asanyarray(d.get_data())
        mm = raw.copy() if abs(self._scale_mm - 1.0) < 1e-9 else \
            np.clip(raw.astype(np.float32) * self._scale_mm, 0, 65535).astype(np.uint16)
        return np.asanyarray(c.get_data()).copy(), mm

    def _close(self):
        if getattr(self, "_pipe", None) is not None:
            self._pipe.stop()
            self._pipe = None


def make_camera(cfg):
    cls = {"orbbec": OrbbecCamera, "realsense": RealsenseCamera}.get((cfg.get("backend") or "none").lower(), NoCamera)
    cam = cls(int(cfg.get("width", 1280)), int(cfg.get("height", 720)), int(cfg.get("fps", 15)))
    if cfg.get("depth_width") and cfg.get("depth_height"):      # otherwise: the largest depth mode at this fps
        cam.depth_width, cam.depth_height = int(cfg["depth_width"]), int(cfg["depth_height"])
    cam.align = bool(cfg.get("align_depth", True))
    wanted = {k: cfg[c] for k, c in (("auto", "auto_exposure"), ("exposure", "exposure"), ("gain", "gain")) if cfg.get(c) is not None}
    cam.exposure_wanted = wanted
    return cam
