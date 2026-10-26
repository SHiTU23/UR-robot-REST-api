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
        self._frame = None            # {"color": bgr u8, "depth_mm": u16, "ts", "id"}
        self._running = False
        self._seq = 0
        self._err = None
        self._serial = None
        self._intr = None             # {"fx", "fy", "ppx", "ppy", "coeffs", "model"}
        self._depth_res = None

    # backend hooks: _open() -> starts the device, sets _serial/_intr/_depth_res; _grab() -> (bgr, depth_mm) | None;
    # _close()
    def start(self):
        with self._lock:
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
        threading.Thread(target=self._loop, daemon=True, name=f"{self.backend}-grab").start()
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
        self._running = False
        time.sleep(0.1)
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

    def set_config(self, width=None, height=None, fps=None):
        old, was = (self.width, self.height, self.fps), self._running
        new = (int(width or old[0]), int(height or old[1]), int(fps or old[2]))
        if new == old and was:
            return True, None
        if was:
            self.stop()
        self.width, self.height, self.fps = new
        ok, e = self.ensure_started()
        if not ok:
            self.width, self.height, self.fps = old
            if was:
                self.ensure_started()
            return False, e
        return True, None

    def supported_modes(self):
        return {"note": "the device decides; /camera/config validates a mode and rolls back on failure"}

    def info(self):
        fr = self.latest()
        out = {"backend": self.backend, "streaming": self._running, "has_frame": fr is not None, "width": self.width,
               "height": self.height, "fps": self.fps, "serial": self._serial, "depth_sensor_res": self._depth_res,
               "depth_scale_m": 0.001, "depth_units": "uint16 millimetres", "last_error": self._err}
        if fr is not None:
            out["frame_age_s"] = round(time.time() - fr["ts"], 3)
        if self._intr is not None:
            out["intrinsics"] = dict(self._intr, note="colour-stream intrinsics; depth is aligned to colour, so (u,v) "
                                                       "indexes both images")
        return out

    def frame_bundle(self):
        fr = self.latest()
        if fr is None:
            return None
        i = self._intr or {}
        return {"color": fr["color"], "depth_mm": fr["depth_mm"], "ts": fr["ts"], "id": fr["id"],
                "fx": i.get("fx", 0.0), "fy": i.get("fy", 0.0), "ppx": i.get("ppx", 0.0), "ppy": i.get("ppy", 0.0),
                "depth_scale_m": 0.001}

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
        if self._intr:
            i = self._intr
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
        self._align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
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
        self.width, self.height = cp.get_width(), cp.get_height()
        self._depth_res = (dp.get_width(), dp.get_height())

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
            if best is None or p.get_width() * p.get_height() > best.get_width() * best.get_height():
                best = p
        if best is None:
            raise RuntimeError(f"no Y16 depth profile at {self.fps} fps (Y14/RLE are not plain uint16)")
        return best

    def _grab(self):
        frames = self._pipe.wait_for_frames(1000)
        if frames is None:
            return None
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
        for dw, dh in dict.fromkeys([(self.width, self.height), (848, 480), (640, 480), (1280, 720)]):
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
        self._align = rs.align(rs.stream.color)

    def _grab(self):
        fs = self._align.process(self._pipe.wait_for_frames(timeout_ms=5000))
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
    return cls(int(cfg.get("width", 1280)), int(cfg.get("height", 720)), int(cfg.get("fps", 15)))
