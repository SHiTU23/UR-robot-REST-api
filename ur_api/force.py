"""End-effector force/torque from the UR10e's built-in flange F/T sensor (the driver's
force_torque_sensor_broadcaster, frame tool0), with the Franka API's tare, pose-dependent compensation and
RViz arrows.

Unlike the Franka's F_ext (an estimate from joint torques), this is a real 6-axis sensor; it still drifts a
little and carries the tool's own weight, so the same tare / compensation logic applies: tare with an empty
gripper, then the net force is what was picked up. /force/zero_sensor zeroes the sensor itself (the driver's
zero_ftsensor) instead of only subtracting a software baseline.
"""
import math
import threading
import time

ZERO = (0.0, 0.0, 0.0)


def _req(value, name, positive):
    f = float(value)
    if not math.isfinite(f):
        raise ValueError(f"{name} must be a finite number")
    if positive and f <= 0:
        raise ValueError(f"{name} must be > 0")
    if not positive and f < 0:
        raise ValueError(f"{name} must be >= 0")
    return f


class ForceMonitor:
    def __init__(self, ros, cell, rate_hz=20.0):
        self.ros, self.cell = ros, cell
        self.scale, self.threshold, self.max_force = 0.02, 0.5, 20.0
        self.torque_scale, self.torque_threshold, self.max_torque = 0.15, 0.05, 2.0
        self._lock = threading.Lock()
        self._bf, self._bt = ZERO, ZERO
        self._calibration = None          # [{config:[6], force:[3], torque:[3]}]
        self._compensate = False
        self._rate = rate_hz
        threading.Thread(target=self._marker_loop, name="force-markers", daemon=True).start()

    # ---- baselines ----

    def _baseline(self, latest):
        with self._lock:
            calib, comp, bf, bt = self._calibration, self._compensate, self._bf, self._bt
        if comp and calib:
            q = self.ros.arm_q()
            if q is not None:
                best = min(calib, key=lambda e: sum((a - b) ** 2 for a, b in zip(q, e["config"])))
                return tuple(best["force"]), tuple(best["torque"]), "calibration"
        return bf, bt, "tare"

    def set_calibration(self, entries):
        entries = list(entries) if entries else None
        with self._lock:
            self._calibration = entries
            if not entries:
                self._compensate = False
        return len(entries) if entries else 0

    def set_compensation(self, on):
        with self._lock:
            if on and not self._calibration:
                return False
            self._compensate = bool(on)
            return True

    def tare(self):
        w = self.ros.wrench
        if w is None or not all(math.isfinite(v) for v in w[0:6]):
            return None
        with self._lock:
            self._bf, self._bt = tuple(w[0:3]), tuple(w[3:6])
        return {"force": dict(zip("xyz", w[0:3])), "torque": dict(zip("xyz", w[3:6]))}

    def untare(self):
        with self._lock:
            self._bf, self._bt = ZERO, ZERO

    # ---- reads ----

    def get_force(self):
        w = self.ros.wrench
        if w is None:
            return {"available": False, "msg": f"no wrench received yet on {self.cell.wrench_topic}"}
        fx, fy, fz, tx, ty, tz, frame, stamp, rx = w
        if not all(math.isfinite(v) for v in (fx, fy, fz, tx, ty, tz)):
            # (mock hardware has no sensor and publishes NaN; NaN is not valid JSON either)
            return {"available": False, "msg": f"the F/T sensor reports non-finite values on {self.cell.wrench_topic}"}
        bf, bt, source = self._baseline(w)
        nf = (fx - bf[0], fy - bf[1], fz - bf[2])
        nt = (tx - bt[0], ty - bt[1], tz - bt[2])
        fm, tm = math.sqrt(sum(v * v for v in nf)), math.sqrt(sum(v * v for v in nt))
        return {"available": True, "frame_id": frame,
                "raw_force": {"x": fx, "y": fy, "z": fz}, "net_force": dict(zip("xyz", nf)),
                "baseline_force": dict(zip("xyz", bf)), "magnitude": fm,
                "raw_magnitude": math.sqrt(fx * fx + fy * fy + fz * fz),
                "raw_torque": {"x": tx, "y": ty, "z": tz}, "net_torque": dict(zip("xyz", nt)),
                "baseline_torque": dict(zip("xyz", bt)), "torque_magnitude": tm,
                "raw_torque_magnitude": math.sqrt(tx * tx + ty * ty + tz * tz),
                "lever_arm": tm / fm if fm > 1e-6 else None, "stamp": stamp, "age_s": round(time.time() - rx, 3),
                "baseline_source": source, "compensated": source == "calibration",
                "tared": source == "tare" and (bf != ZERO or bt != ZERO),
                "sensor": "UR e-series flange F/T sensor"}

    def set_config(self, scale=None, threshold=None, max_force=None, torque_scale=None, torque_threshold=None,
                   max_torque=None):
        for name, v, pos in (("scale", scale, True), ("threshold", threshold, False), ("max_force", max_force, True),
                             ("torque_scale", torque_scale, True), ("torque_threshold", torque_threshold, False),
                             ("max_torque", max_torque, True)):
            if v is not None:
                setattr(self, name, _req(v, name, pos))
        return {k: getattr(self, k) for k in ("scale", "threshold", "max_force", "torque_scale", "torque_threshold",
                                              "max_torque")}

    # ---- RViz arrows (MarkerArray on /ur_ee_force, in the sensor frame) ----

    def _marker_loop(self):
        while True:
            time.sleep(1.0 / self._rate)
            try:
                self._publish()
            except Exception:      # noqa: BLE001
                pass

    def _publish(self):
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import Point, Vector3
        from std_msgs.msg import ColorRGBA
        from visualization_msgs.msg import Marker, MarkerArray
        d = self.get_force()
        if not d.get("available"):
            return
        nf = [d["net_force"][k] for k in "xyz"]
        nt = [d["net_torque"][k] for k in "xyz"]
        if not all(math.isfinite(v) for v in nf + nt):
            return
        stamp = self.ros.node.get_clock().now().to_msg()

        def ramp(mag, mx, lo, hi):
            t = max(0.0, min(1.0, mag / mx)) if mx > 0 else 0.0
            return ColorRGBA(r=lo[0] + (hi[0] - lo[0]) * t, g=lo[1] + (hi[1] - lo[1]) * t,
                             b=lo[2] + (hi[2] - lo[2]) * t, a=0.9)

        def base(mid, kind):
            m = Marker(id=mid, ns="ee_force", type=kind)
            m.header.frame_id, m.header.stamp = d["frame_id"], stamp
            m.lifetime = Duration(sec=0, nanosec=500_000_000)
            m.pose.orientation.w = 1.0
            return m

        out = []
        for aid, tid, vec, mag, scale, thr, col, label in (
                (0, 1, nf, d["magnitude"], self.scale, self.threshold,
                 ramp(d["magnitude"], self.max_force, (0, 1, 0), (1, 0, 0)), "%.1f N" % d["magnitude"]),
                (2, 3, nt, d["torque_magnitude"], self.torque_scale, self.torque_threshold,
                 ramp(d["torque_magnitude"], self.max_torque, (0, .3, 1), (1, 0, 1)), "%.2f N·m" % d["torque_magnitude"])):
            if mag < thr:
                for mid in (aid, tid):
                    m = base(mid, Marker.ARROW)
                    m.action = Marker.DELETE
                    out.append(m)
                continue
            tip = [v * scale for v in vec]
            length = math.sqrt(sum(v * v for v in tip))
            a = base(aid, Marker.ARROW)
            a.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=tip[0], y=tip[1], z=tip[2])]
            shaft = min(0.012, max(0.004, 0.12 * length))
            a.scale = Vector3(x=shaft, y=2 * shaft, z=min(0.04, 0.3 * length))
            a.color = col
            t = base(tid, Marker.TEXT_VIEW_FACING)
            t.pose.position = Point(x=tip[0], y=tip[1], z=tip[2] + 0.04)
            t.scale.z = 0.04
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            t.text = label
            out += [a, t]
        self.ros.marker_pub.publish(MarkerArray(markers=out))
