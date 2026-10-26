"""Mass and centre-of-mass offset of a held object from the flange F/T sensor, robust to a pose-dependent
residual (the Franka API's method):

    measure at N static poses EMPTY, then at the SAME poses HOLDING the object; per pose
    dF_i = F_loaded_i - F_empty_i = the object's weight in the sensor frame (the residual cancels),
    |dF_i| = m g at every pose -> m = median |dF_i| / g;  dtau_i = r x dF_i -> r by least squares.

The poses tilt the tool with small wrist moves (wrist_1, wrist_2), so gravity is seen from several
directions. MoveIt checks these moves against the scene, but the held object only if it is attached to the
hand in the planning scene -- keep clear space around it for the loaded pass.
"""
import math
import time

import numpy as np

G = 9.80665
PERTURBATIONS = [
    [0, 0, 0, 0.0, 0.0, 0],        # base (as captured)
    [0, 0, 0, 0.25, 0.0, 0],       # wrist_1 +
    [0, 0, 0, -0.25, 0.0, 0],      # wrist_1 -
    [0, 0, 0, 0.0, 0.2, 0],        # wrist_2 +
    [0, 0, 0, 0.0, -0.2, 0],       # wrist_2 -
]


class PayloadIdentifier:
    """Motion goes through motion.move_joints; the caller holds the motion lock around the passes."""

    def __init__(self, motion, force_reader, settle_s=1.0, n_samples=20, sample_dt=0.05):
        self.motion, self.read_force = motion, force_reader
        self.settle_s, self.n_samples, self.sample_dt = settle_s, n_samples, sample_dt
        self.reset()

    def reset(self):
        self._configs = self._empty = self._loaded = None

    def _sample(self):
        fs, ts = [], []
        for _ in range(self.n_samples):
            d = self.read_force()
            if d.get("available"):
                f = [d["raw_force"][k] for k in "xyz"]
                t = [d["raw_torque"][k] for k in "xyz"]
                if all(math.isfinite(v) for v in f + t):
                    fs.append(f)
                    ts.append(t)
            time.sleep(self.sample_dt)
        if not fs:
            return None
        return {"force": np.mean(fs, axis=0).tolist(), "torque": np.mean(ts, axis=0).tolist(),
                "force_std": float(np.linalg.norm(np.std(fs, axis=0))), "samples": len(fs)}

    def _run(self, configs):
        """Visit every config, settle, sample; stops at /control/stop (and does not return to base then)."""
        out, stopped = [], False
        self.motion.clear_stop()
        for i, q in enumerate(configs):
            if self.motion.stop_requested():
                out.append({"pose": i, "moved": False, "stopped": True})
                stopped = True
                break
            r = self.motion.move_joints(q, vel=0.1, acc=0.05)
            if not r.get("executed"):
                out.append({"pose": i, "moved": False, "stopped": bool(r.get("stopped")),
                            "plan_error": r.get("plan_error_codes")})
                if r.get("stopped") or self.motion.stop_requested():
                    stopped = True
                    break
                continue
            time.sleep(self.settle_s)
            e = {"pose": i, "moved": True}
            s = self._sample()
            if s:
                e.update(s)
            out.append(e)
        back = False
        if configs and not stopped and not self.motion.stop_requested():
            back = bool(self.motion.move_joints(configs[0], vel=0.1, acc=0.05).get("executed"))
        return out, back

    def record_empty(self):
        base = list(self.motion.current_q())
        self._empty = self._loaded = None
        self._configs = [[b + p for b, p in zip(base, pert)] for pert in PERTURBATIONS]
        out, back = self._run(self._configs)
        self._empty = out
        return {"base_config": [round(v, 5) for v in base], "n_poses": len(self._configs),
                "moved": sum(1 for e in out if e.get("moved")), "returned_to_base": back,
                "stopped": any(e.get("stopped") for e in out), "empty": out}

    def record_loaded(self):
        if self._configs is None:
            raise ValueError("run the empty pass first (/force/identify/empty)")
        out, back = self._run(self._configs)
        self._loaded = out
        return {"n_poses": len(self._configs), "moved": sum(1 for e in out if e.get("moved")),
                "returned_to_base": back, "stopped": any(e.get("stopped") for e in out), "loaded": out}

    def compute(self):
        empty, loaded = self._empty, self._loaded
        if empty is None or loaded is None:
            raise ValueError("need both empty and loaded passes")
        mags, A, b, per = [], [], [], []
        for e, l in zip(empty, loaded):
            if not (e.get("moved") and l.get("moved") and "force" in e and "force" in l):
                continue
            df = np.array(l["force"]) - np.array(e["force"])
            dt = np.array(l["torque"]) - np.array(e["torque"])
            m = float(np.linalg.norm(df))
            mags.append(m)
            per.append({"pose": e["pose"], "dF": df.tolist(), "dF_mag_N": m, "mass_kg": m / G})
            fx, fy, fz = df                          # dt = r x df = -skew(df) r
            A.append([[0.0, -fz, fy], [fz, 0.0, -fx], [-fy, fx, 0.0]])
            b.append((-dt).tolist())
        if not mags:
            raise ValueError("no valid pose pairs (did the arm move and sample?)")
        masses = np.array(mags) / G
        res = {"mass_kg": float(np.median(masses)), "mass_mean_kg": float(np.mean(masses)),
               "mass_std_kg": float(np.std(masses)), "weight_N": float(np.median(mags)), "n_pose_pairs": len(mags),
               "per_pose": per}
        if len(A) >= 2:
            r = np.linalg.lstsq(np.vstack(A), np.concatenate(b), rcond=None)[0]
            res["com_offset_sensor_m"] = r.tolist()
            res["com_distance_m"] = float(np.linalg.norm(r))
        return res

    def calibration(self):
        if self._empty is None or self._configs is None:
            return []
        return [{"config": list(self._configs[e["pose"]]), "force": e["force"], "torque": e["torque"]}
                for e in self._empty if e.get("moved") and "force" in e]
